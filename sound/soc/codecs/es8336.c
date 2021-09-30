// SPDX-License-Identifier: GPL-2.0-only
/*
 * es8336.c -- es8336 ALSA SoC audio driver
 * based on es8316.c
 * Copyright Everest Semiconductor Co.,Ltd
 *
 * Authors: David Yang <yangxiaohua@everest-semi.com>,
 *          Daniel Drake <drake@endlessm.com>
 */

#include <linux/module.h>
#include <linux/acpi.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/mod_devicetable.h>
#include <linux/mutex.h>
#include <linux/regmap.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/tlv.h>
#include <sound/jack.h>
#include <linux/gpio/consumer.h>
#include "es8336-reg.h"

/* In slave mode at single speed, the codec is documented as accepting 5
 * MCLK/LRCK ratios, but we also add ratio 400, which is commonly used on
 * Intel Cherry Trail platforms (19.2MHz MCLK, 48kHz LRCK).
 */
#define NR_SUPPORTED_MCLK_LRCK_RATIOS 6
static const unsigned int supported_mclk_lrck_ratios[] = {
	256, 384, 400, 512, 768, 1024
};

struct es8336_priv {
	struct mutex lock;
	struct clk *mclk;
	struct regmap *regmap;
	struct snd_soc_component *component;
	struct snd_soc_jack *jack;
	int irq;
	unsigned int sysclk;
	unsigned int allowed_rates[NR_SUPPORTED_MCLK_LRCK_RATIOS];
	struct snd_pcm_hw_constraint_list sysclk_constraints;
	bool jd_inverted;
	struct gpio_desc *pa_enable;
};

/*
 * ES8336 controls
 */
static const SNDRV_CTL_TLVD_DECLARE_DB_SCALE(dac_vol_tlv, -9600, 50, 1);
static const SNDRV_CTL_TLVD_DECLARE_DB_SCALE(adc_vol_tlv, -9600, 50, 1);
static const SNDRV_CTL_TLVD_DECLARE_DB_SCALE(alc_max_gain_tlv, -650, 150, 0);
static const SNDRV_CTL_TLVD_DECLARE_DB_SCALE(alc_min_gain_tlv, -1200, 150, 0);
static const SNDRV_CTL_TLVD_DECLARE_DB_SCALE(alc_target_tlv, -1650, 150, 0);
static const SNDRV_CTL_TLVD_DECLARE_DB_RANGE(hpmixer_gain_tlv,
	0, 4, TLV_DB_SCALE_ITEM(-1200, 150, 0),
	8, 11, TLV_DB_SCALE_ITEM(-450, 150, 0),
);

static const SNDRV_CTL_TLVD_DECLARE_DB_RANGE(adc_pga_gain_tlv,
	0, 0, TLV_DB_SCALE_ITEM(-350, 0, 0),
	1, 1, TLV_DB_SCALE_ITEM(0, 0, 0),
	2, 2, TLV_DB_SCALE_ITEM(250, 0, 0),
	3, 3, TLV_DB_SCALE_ITEM(450, 0, 0),
	4, 7, TLV_DB_SCALE_ITEM(700, 300, 0),
	8, 10, TLV_DB_SCALE_ITEM(1800, 300, 0),
);

static const SNDRV_CTL_TLVD_DECLARE_DB_RANGE(hpout_vol_tlv,
	0, 0, TLV_DB_SCALE_ITEM(-4800, 0, 0),
	1, 3, TLV_DB_SCALE_ITEM(-2400, 1200, 0),
);

static const char * const ng_type_txt[] =
	{ "Constant PGA Gain", "Mute ADC Output" };
static const struct soc_enum ng_type =
	SOC_ENUM_SINGLE(ES8336_ADC_ALC_NG, 6, 2, ng_type_txt);

static const char * const adcpol_txt[] = { "Normal", "Invert" };
static const struct soc_enum adcpol =
	SOC_ENUM_SINGLE(ES8336_ADC_MUTE, 1, 2, adcpol_txt);
static const char *const dacpol_txt[] =
	{ "Normal", "R Invert", "L Invert", "L + R Invert" };
static const struct soc_enum dacpol =
	SOC_ENUM_SINGLE(ES8336_DAC_SET1, 0, 4, dacpol_txt);

static const struct snd_kcontrol_new es8336_snd_controls[] = {
	SOC_DOUBLE_TLV("Headphone Mixer Volume", ES8336_HPMIX_VOL,
		       4, 0, 11, 0, hpmixer_gain_tlv),

	SOC_ENUM("Playback Polarity", dacpol),
	SOC_DOUBLE_R_TLV("DAC Playback Volume", ES8336_DAC_VOLL,
			 ES8336_DAC_VOLR, 0, 0xc0, 1, dac_vol_tlv),
	SOC_SINGLE("DAC Soft Ramp Switch", ES8336_DAC_SET1, 4, 1, 1),
	SOC_SINGLE("DAC Soft Ramp Rate", ES8336_DAC_SET1, 2, 4, 0),
	SOC_SINGLE("DAC Notch Filter Switch", ES8336_DAC_SET2, 6, 1, 0),
	SOC_SINGLE("DAC Double Fs Switch", ES8336_DAC_SET2, 7, 1, 0),
	SOC_SINGLE("DAC Stereo Enhancement", ES8336_DAC_SET3, 0, 7, 0),
	SOC_SINGLE("DAC Mono Mix Switch", ES8336_DAC_SET3, 3, 1, 0),

	SOC_ENUM("Capture Polarity", adcpol),
	SOC_SINGLE_TLV("ADC Capture Volume", ES8336_ADC_VOLUME,
		       0, 0xc0, 1, adc_vol_tlv),
	SOC_SINGLE_TLV("ADC PGA Gain Volume", ES8336_ADC_PGAGAIN,
		       4, 10, 0, adc_pga_gain_tlv),
	SOC_SINGLE("ADC Soft Ramp Switch", ES8336_ADC_MUTE, 4, 1, 0),
	SOC_SINGLE("ADC Double Fs Switch", ES8336_ADC_DMIC, 4, 1, 0),

	SOC_SINGLE("ALC Capture Switch", ES8336_ADC_ALC1, 6, 1, 0),
	SOC_SINGLE_TLV("ALC Capture Max Volume", ES8336_ADC_ALC1, 0, 28, 0,
		       alc_max_gain_tlv),
	SOC_SINGLE_TLV("ALC Capture Min Volume", ES8336_ADC_ALC2, 0, 28, 0,
		       alc_min_gain_tlv),
	SOC_SINGLE_TLV("ALC Capture Target Volume", ES8336_ADC_ALC3, 4, 10, 0,
		       alc_target_tlv),
	SOC_SINGLE("ALC Capture Hold Time", ES8336_ADC_ALC3, 0, 10, 0),
	SOC_SINGLE("ALC Capture Decay Time", ES8336_ADC_ALC4, 4, 10, 0),
	SOC_SINGLE("ALC Capture Attack Time", ES8336_ADC_ALC4, 0, 10, 0),
	SOC_SINGLE("ALC Capture Noise Gate Switch", ES8336_ADC_ALC_NG,
		   5, 1, 0),
	SOC_SINGLE("ALC Capture Noise Gate Threshold", ES8336_ADC_ALC_NG,
		   0, 31, 0),
	SOC_ENUM("ALC Capture Noise Gate Type", ng_type),
};

/* Analog Input Mux */
static const char * const es8336_analog_in_txt[] = {
		"lin1-rin1",
		"lin2-rin2",
		"lin1-rin1 with 20db Boost",
		"lin2-rin2 with 20db Boost"
};
static const unsigned int es8336_analog_in_values[] = { 0, 1, 2, 3 };
static const struct soc_enum es8336_analog_input_enum =
	SOC_VALUE_ENUM_SINGLE(ES8336_ADC_PDN_LINSEL, 4, 3,
			      ARRAY_SIZE(es8336_analog_in_txt),
			      es8336_analog_in_txt,
			      es8336_analog_in_values);
static const struct snd_kcontrol_new es8336_analog_in_mux_controls =
	SOC_DAPM_ENUM("Route", es8336_analog_input_enum);

static const char * const es8336_dmic_txt[] = {
		"dmic disable",
		"dmic data at high level",
		"dmic data at low level",
};
static const unsigned int es8336_dmic_values[] = { 0, 1, 2 };
static const struct soc_enum es8336_dmic_src_enum =
	SOC_VALUE_ENUM_SINGLE(ES8336_ADC_DMIC, 0, 3,
			      ARRAY_SIZE(es8336_dmic_txt),
			      es8336_dmic_txt,
			      es8336_dmic_values);
static const struct snd_kcontrol_new es8336_dmic_src_controls =
	SOC_DAPM_ENUM("Route", es8336_dmic_src_enum);

/* hp mixer mux */
static const char * const es8336_hpmux_texts[] = {
	"lin1-rin1",
	"lin2-rin2",
	"lin-rin with Boost",
	"lin-rin with Boost and PGA"
};

static SOC_ENUM_SINGLE_DECL(es8336_left_hpmux_enum, ES8336_HPMIX_SEL,
	4, es8336_hpmux_texts);

static const struct snd_kcontrol_new es8336_left_hpmux_controls =
	SOC_DAPM_ENUM("Route", es8336_left_hpmux_enum);

static SOC_ENUM_SINGLE_DECL(es8336_right_hpmux_enum, ES8336_HPMIX_SEL,
	0, es8336_hpmux_texts);

static const struct snd_kcontrol_new es8336_right_hpmux_controls =
	SOC_DAPM_ENUM("Route", es8336_right_hpmux_enum);

/* headphone Output Mixer */
static const struct snd_kcontrol_new es8336_out_left_mix[] = {
	SOC_DAPM_SINGLE("LLIN Switch", ES8336_HPMIX_SWITCH, 6, 1, 0),
	SOC_DAPM_SINGLE("Left DAC Switch", ES8336_HPMIX_SWITCH, 7, 1, 0),
};
static const struct snd_kcontrol_new es8336_out_right_mix[] = {
	SOC_DAPM_SINGLE("RLIN Switch", ES8336_HPMIX_SWITCH, 2, 1, 0),
	SOC_DAPM_SINGLE("Right DAC Switch", ES8336_HPMIX_SWITCH, 3, 1, 0),
};

/* DAC data source mux */
static const char * const es8336_dacsrc_texts[] = {
	"LDATA TO LDAC, RDATA TO RDAC",
	"LDATA TO LDAC, LDATA TO RDAC",
	"RDATA TO LDAC, RDATA TO RDAC",
	"RDATA TO LDAC, LDATA TO RDAC",
};

static SOC_ENUM_SINGLE_DECL(es8336_dacsrc_mux_enum, ES8336_DAC_SET1,
	6, es8336_dacsrc_texts);

static const struct snd_kcontrol_new es8336_dacsrc_mux_controls =
	SOC_DAPM_ENUM("Route", es8336_dacsrc_mux_enum);

static const struct snd_soc_dapm_widget es8336_dapm_widgets[] = {
	SND_SOC_DAPM_SUPPLY("Bias", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("Analog power", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("Mic Bias", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_INPUT("DMIC"),
	SND_SOC_DAPM_INPUT("MIC1"),
	SND_SOC_DAPM_INPUT("MIC2"),

	/* Input Mux */
	SND_SOC_DAPM_MUX("Differential Mux", SND_SOC_NOPM, 0, 0,
			 &es8336_analog_in_mux_controls),

	SND_SOC_DAPM_SUPPLY("ADC Vref", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("ADC bias", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("ADC Clock", ES8336_CLKMGR_CLKSW, 3, 0, NULL, 0),
	SND_SOC_DAPM_PGA("Line input PGA", ES8336_ADC_PDN_LINSEL,
			 7, 1, NULL, 0),
	SND_SOC_DAPM_ADC("Mono ADC", NULL, ES8336_ADC_PDN_LINSEL, 6, 1),
	SND_SOC_DAPM_MUX("Digital Mic Mux", SND_SOC_NOPM, 0, 0,
			 &es8336_dmic_src_controls),

	/* Digital Interface */
	SND_SOC_DAPM_AIF_OUT("I2S OUT", "I2S1 Capture",  1,
			     ES8336_SERDATA_ADC, 6, 1),
	SND_SOC_DAPM_AIF_IN("I2S IN", "I2S1 Playback", 0,
			    SND_SOC_NOPM, 0, 0),

	SND_SOC_DAPM_MUX("DAC Source Mux", SND_SOC_NOPM, 0, 0,
			 &es8336_dacsrc_mux_controls),

	SND_SOC_DAPM_SUPPLY("DAC Vref", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("DAC Clock", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_DAC("Right DAC", NULL, ES8336_DAC_PDN, 0, 1),
	SND_SOC_DAPM_DAC("Left DAC", NULL, ES8336_DAC_PDN, 4, 1),

	/* Headphone Output Side */
	SND_SOC_DAPM_MUX("Left Headphone Mux", SND_SOC_NOPM, 0, 0,
			 &es8336_left_hpmux_controls),
	SND_SOC_DAPM_MUX("Right Headphone Mux", SND_SOC_NOPM, 0, 0,
			 &es8336_right_hpmux_controls),
	SND_SOC_DAPM_MIXER("Left Headphone Mixer", ES8336_HPMIX_PDN,
			   5, 1, &es8336_out_left_mix[0],
			   ARRAY_SIZE(es8336_out_left_mix)),
	SND_SOC_DAPM_MIXER("Right Headphone Mixer", ES8336_HPMIX_PDN,
			   1, 1, &es8336_out_right_mix[0],
			   ARRAY_SIZE(es8336_out_right_mix)),
	SND_SOC_DAPM_PGA("Left Headphone Mixer Out", ES8336_HPMIX_PDN,
			 4, 1, NULL, 0),
	SND_SOC_DAPM_PGA("Right Headphone Mixer Out", ES8336_HPMIX_PDN,
			 0, 1, NULL, 0),

	SND_SOC_DAPM_OUT_DRV("Left Headphone Charge Pump", ES8336_CPHP_OUTEN,
			     6, 0, NULL, 0),
	SND_SOC_DAPM_OUT_DRV("Right Headphone Charge Pump", ES8336_CPHP_OUTEN,
			     2, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("Headphone Charge Pump", ES8336_CPHP_PDN2,
			    5, 1, NULL, 0),
	SND_SOC_DAPM_SUPPLY("Headphone Charge Pump Clock", SND_SOC_NOPM,
			    0, 0, NULL, 0),
	SND_SOC_DAPM_OUT_DRV("Left Headphone Driver", ES8336_CPHP_OUTEN,
			     5, 0, NULL, 0),
	SND_SOC_DAPM_OUT_DRV("Right Headphone Driver", ES8336_CPHP_OUTEN,
			     1, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("Headphone Out", ES8336_CPHP_PDN1, 2, 1, NULL, 0),

	/* pdn_Lical and pdn_Rical bits are documented as Reserved, but must
	 * be explicitly unset in order to enable HP output
	 */
	SND_SOC_DAPM_SUPPLY("Left Headphone ical", ES8336_CPHP_ICAL_VOL,
			    7, 1, NULL, 0),
	SND_SOC_DAPM_SUPPLY("Right Headphone ical", ES8336_CPHP_ICAL_VOL,
			    3, 1, NULL, 0),

	SND_SOC_DAPM_OUTPUT("HPOL"),
	SND_SOC_DAPM_OUTPUT("HPOR"),
};

static const struct snd_soc_dapm_route es8336_dapm_routes[] = {
	/* Recording */
	{"MIC1", NULL, "Mic Bias"},
	{"MIC2", NULL, "Mic Bias"},
	{"MIC1", NULL, "Bias"},
	{"MIC2", NULL, "Bias"},
	{"MIC1", NULL, "Analog power"},
	{"MIC2", NULL, "Analog power"},

	{"Differential Mux", "lin1-rin1", "MIC1"},
	{"Differential Mux", "lin2-rin2", "MIC2"},
	{"Line input PGA", NULL, "Differential Mux"},

	{"Mono ADC", NULL, "ADC Clock"},
	{"Mono ADC", NULL, "ADC Vref"},
	{"Mono ADC", NULL, "ADC bias"},
	{"Mono ADC", NULL, "Line input PGA"},

	/* It's not clear why, but to avoid recording only silence,
	 * the DAC clock must be running for the ADC to work.
	 */
	{"Mono ADC", NULL, "DAC Clock"},

	{"Digital Mic Mux", "dmic disable", "Mono ADC"},

	{"I2S OUT", NULL, "Digital Mic Mux"},

	/* Playback */
	{"DAC Source Mux", "LDATA TO LDAC, RDATA TO RDAC", "I2S IN"},

	{"Left DAC", NULL, "DAC Clock"},
	{"Right DAC", NULL, "DAC Clock"},

	{"Left DAC", NULL, "DAC Vref"},
	{"Right DAC", NULL, "DAC Vref"},

	{"Left DAC", NULL, "DAC Source Mux"},
	{"Right DAC", NULL, "DAC Source Mux"},

	{"Left Headphone Mux", "lin-rin with Boost and PGA", "Line input PGA"},
	{"Right Headphone Mux", "lin-rin with Boost and PGA", "Line input PGA"},

	{"Left Headphone Mixer", "LLIN Switch", "Left Headphone Mux"},
	{"Left Headphone Mixer", "Left DAC Switch", "Left DAC"},

	{"Right Headphone Mixer", "RLIN Switch", "Right Headphone Mux"},
	{"Right Headphone Mixer", "Right DAC Switch", "Right DAC"},

	{"Left Headphone Mixer Out", NULL, "Left Headphone Mixer"},
	{"Right Headphone Mixer Out", NULL, "Right Headphone Mixer"},

	{"Left Headphone Charge Pump", NULL, "Left Headphone Mixer Out"},
	{"Right Headphone Charge Pump", NULL, "Right Headphone Mixer Out"},

	{"Left Headphone Charge Pump", NULL, "Headphone Charge Pump"},
	{"Right Headphone Charge Pump", NULL, "Headphone Charge Pump"},

	{"Left Headphone Charge Pump", NULL, "Headphone Charge Pump Clock"},
	{"Right Headphone Charge Pump", NULL, "Headphone Charge Pump Clock"},

	{"Left Headphone Driver", NULL, "Left Headphone Charge Pump"},
	{"Right Headphone Driver", NULL, "Right Headphone Charge Pump"},

	{"HPOL", NULL, "Left Headphone Driver"},
	{"HPOR", NULL, "Right Headphone Driver"},

	{"HPOL", NULL, "Left Headphone ical"},
	{"HPOR", NULL, "Right Headphone ical"},

	{"Headphone Out", NULL, "Bias"},
	{"Headphone Out", NULL, "Analog power"},
	{"HPOL", NULL, "Headphone Out"},
	{"HPOR", NULL, "Headphone Out"},
};

static int es8336_set_dai_sysclk(struct snd_soc_dai *codec_dai,
				 int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_component *component = codec_dai->component;
	struct es8336_priv *es8336 = snd_soc_component_get_drvdata(component);
	int i, ret;
	int count = 0;

	es8336->sysclk = freq;

	if (freq == 0) {
		es8336->sysclk_constraints.list = NULL;
		es8336->sysclk_constraints.count = 0;

		return 0;
	}

	ret = clk_set_rate(es8336->mclk, freq);
	if (ret)
		return ret;

	/* Limit supported sample rates to ones that can be autodetected
	 * by the codec running in slave mode.
	 */
	for (i = 0; i < NR_SUPPORTED_MCLK_LRCK_RATIOS; i++) {
		const unsigned int ratio = supported_mclk_lrck_ratios[i];

		if (freq % ratio == 0)
			es8336->allowed_rates[count++] = freq / ratio;
	}

	es8336->sysclk_constraints.list = es8336->allowed_rates;
	es8336->sysclk_constraints.count = count;

	return 0;
}

static int es8336_set_dai_fmt(struct snd_soc_dai *codec_dai,
			      unsigned int fmt)
{
	struct snd_soc_component *component = codec_dai->component;
	u8 serdata1 = 0;
	u8 serdata2 = 0;
	u8 mask;

	if ((fmt & SND_SOC_DAIFMT_MASTER_MASK) != SND_SOC_DAIFMT_CBS_CFS) {
		dev_err(component->dev, "Codec driver only supports slave mode\n");
		return -EINVAL;
	}

	if ((fmt & SND_SOC_DAIFMT_FORMAT_MASK) != SND_SOC_DAIFMT_I2S) {
		dev_err(component->dev, "Codec driver only supports I2S format\n");
		return -EINVAL;
	}

	/* Clock inversion */
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		break;
	case SND_SOC_DAIFMT_IB_IF:
		serdata1 |= ES8336_SERDATA1_BCLK_INV;
		serdata2 |= ES8336_SERDATA2_ADCLRP;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		serdata1 |= ES8336_SERDATA1_BCLK_INV;
		break;
	case SND_SOC_DAIFMT_NB_IF:
		serdata2 |= ES8336_SERDATA2_ADCLRP;
		break;
	default:
		return -EINVAL;
	}

	mask = ES8336_SERDATA1_MASTER | ES8336_SERDATA1_BCLK_INV;
	snd_soc_component_update_bits(component, ES8336_SERDATA1, mask, serdata1);

	mask = ES8336_SERDATA2_FMT_MASK | ES8336_SERDATA2_ADCLRP;
	snd_soc_component_update_bits(component, ES8336_SERDATA_ADC, mask, serdata2);
	snd_soc_component_update_bits(component, ES8336_SERDATA_DAC, mask, serdata2);

	return 0;
}

static int es8336_pcm_startup(struct snd_pcm_substream *substream,
			      struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	struct es8336_priv *es8336 = snd_soc_component_get_drvdata(component);
	static int done = 0;

	if (es8336->sysclk_constraints.list)
		snd_pcm_hw_constraint_list(substream->runtime, 0,
					   SNDRV_PCM_HW_PARAM_RATE,
					   &es8336->sysclk_constraints);

	/* try only once */
	if(!done)
	{
		mdelay(5000);
		printk("i2cset 0x0d 0x08");
		snd_soc_component_write(component, 0x4e, 0xf1);
		snd_soc_component_write(component, 0x4e, 0xf2);
		done = 1;
	}
	return 0;
}

static int es8336_pcm_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	struct es8336_priv *es8336 = snd_soc_component_get_drvdata(component);
	u8 wordlen = 0;
	int i;

	/* Validate supported sample rates that are autodetected from MCLK */
	for (i = 0; i < NR_SUPPORTED_MCLK_LRCK_RATIOS; i++) {
		const unsigned int ratio = supported_mclk_lrck_ratios[i];

		if (es8336->sysclk % ratio != 0)
			continue;
		if (es8336->sysclk / ratio == params_rate(params))
			break;
	}
	if (i == NR_SUPPORTED_MCLK_LRCK_RATIOS)
		return -EINVAL;

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		wordlen = ES8336_SERDATA2_LEN_16;
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		wordlen = ES8336_SERDATA2_LEN_20;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		wordlen = ES8336_SERDATA2_LEN_24;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		wordlen = ES8336_SERDATA2_LEN_32;
		break;
	default:
		return -EINVAL;
	}

	snd_soc_component_update_bits(component, ES8336_SERDATA_DAC,
			    ES8336_SERDATA2_LEN_MASK, wordlen);
	snd_soc_component_update_bits(component, ES8336_SERDATA_ADC,
			    ES8336_SERDATA2_LEN_MASK, wordlen);
	snd_soc_component_write(component, 0x01, 0x7f);
	snd_soc_component_write(component, 0x02, 0x09);
	snd_soc_component_update_bits(component,ES8336_ADC_PDN_LINSEL,0xcf,0x00);
#if 0
	snd_soc_component_write(component, 0x22, 0x30);
#endif
	snd_soc_component_write(component, 0x0a, 0x00);
	snd_soc_component_write(component, 0x0d, 0x00);
	return 0;
}

static int es8336_mute(struct snd_soc_dai *dai, int mute, int direction)
{
	snd_soc_component_update_bits(dai->component, ES8336_DAC_SET1, 0x20,
			    mute ? 0x20 : 0);
	return 0;
}

#define ES8336_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE | \
			SNDRV_PCM_FMTBIT_S24_LE)

static int es8336_resume(struct snd_soc_component *component)
{
	snd_soc_component_write(component, ES8336_RESET_REG00, 0x3f);
	usleep_range(5000, 5500);
	snd_soc_component_write(component, ES8336_RESET_REG00, 0x00);
	msleep(30);
	snd_soc_component_write(component, ES8336_SYS_VMIDSEL_REG0C, 0xFF);
	msleep(30);
	snd_soc_component_write(component, ES8336_CLKMGR_CLKSEL_REG02, 0x09);
	snd_soc_component_write(component, ES8336_CLKMGR_ADCOSR_REG03, 0x32);
	snd_soc_component_write(component, ES8336_CLKMGR_ADCDIV1_REG04, 0x11);
	snd_soc_component_write(component, ES8336_CLKMGR_ADCDIV2_REG05, 0x90);
	snd_soc_component_write(component, ES8336_CLKMGR_DACDIV1_REG06, 0x11);
	snd_soc_component_write(component, ES8336_CLKMGR_DACDIV2_REG07, 0x90);
	snd_soc_component_write(component, ES8336_CLKMGR_CPDIV_REG08, 0x00);
	snd_soc_component_write(component, ES8336_SDP_MS_BCKDIV_REG09, 0x04);
	snd_soc_component_write(component, ES8336_CLKMGR_CLKSW_REG01, 0x7F);
	snd_soc_component_write(component, ES8336_CAL_TYPE_REG1C, 0x0F);
	snd_soc_component_write(component, ES8336_CAL_HPLIV_REG1E, 0x90);
	snd_soc_component_write(component, ES8336_CAL_HPRIV_REG1F, 0x90);
	snd_soc_component_write(component, ES8336_ADC_VOLUME_REG27, 0x00);
	snd_soc_component_write(component, ES8336_ADC_PDN_LINSEL_REG22, 0xc0);
	snd_soc_component_write(component, ES8336_ADC_D2SEPGA_REG24, 0x00);
	snd_soc_component_write(component, ES8336_ADC_DMIC_REG25, 0x08);
	snd_soc_component_write(component, ES8336_DAC_SET2_REG31, 0x20);
	snd_soc_component_write(component, ES8336_DAC_SET3_REG32, 0x00);
	snd_soc_component_write(component, ES8336_DAC_VOLL_REG33, 0x00);
	snd_soc_component_write(component, ES8336_DAC_VOLR_REG34, 0x00);
	snd_soc_component_write(component, ES8336_SDP_ADCFMT_REG0A, 0x00);
	snd_soc_component_write(component, ES8336_SDP_DACFMT_REG0B, 0x00);
	snd_soc_component_write(component, ES8336_SYS_VMIDLOW_REG10, 0x11);
	snd_soc_component_write(component, ES8336_SYS_VSEL_REG11, 0xFC);
	snd_soc_component_write(component, ES8336_SYS_REF_REG12, 0x28);
	snd_soc_component_write(component, ES8336_SYS_LP1_REG0E, 0x04);
	snd_soc_component_write(component, ES8336_SYS_LP2_REG0F, 0x0C);
	snd_soc_component_write(component, ES8336_DAC_PDN_REG2F, 0x11);
	snd_soc_component_write(component, ES8336_HPMIX_SEL_REG13, 0x00);
	snd_soc_component_write(component, ES8336_HPMIX_SWITCH_REG14, 0x88);
	snd_soc_component_write(component, ES8336_HPMIX_PDN_REG15, 0x00);
	snd_soc_component_write(component, ES8336_HPMIX_VOL_REG16, 0xBB);
	snd_soc_component_write(component, ES8336_CPHP_PDN2_REG1A, 0x10);
	snd_soc_component_write(component, ES8336_CPHP_LDOCTL_REG1B, 0x30);
	snd_soc_component_write(component, ES8336_CPHP_PDN1_REG19, 0x02);
	snd_soc_component_write(component, ES8336_CPHP_ICAL_VOL_REG18, 0x00);
	snd_soc_component_write(component, ES8336_GPIO_SEL_REG4D, 0x00);
	snd_soc_component_write(component, ES8336_GPIO_DEBOUNCE, 0xf2);
	snd_soc_component_write(component, ES8336_TESTMODE_REG50, 0xA0);
		snd_soc_component_write(component,0x26,0x00);
	snd_soc_component_write(component, ES8336_TEST1_REG51, 0x00);
	snd_soc_component_write(component, ES8336_TEST2_REG52, 0x00);
	snd_soc_component_write(component, ES8336_SYS_PDN_REG0D, 0x00);
	snd_soc_component_write(component, ES8336_RESET_REG00, 0xC0);
	msleep(50);
	snd_soc_component_write(component, ES8336_ADC_PGAGAIN_REG23, 0x60);
	snd_soc_component_write(component, ES8336_ADC_PDN_LINSEL_REG22, 0x30);
	snd_soc_component_write(component, ES8336_ADC_D2SEPGA_REG24, 0x00);
	/* adc ds mode, HPF enable */
	snd_soc_component_write(component, ES8336_ADC_DMIC_REG25, 0x08);
	snd_soc_component_write(component, ES8336_ADC_ALC1_REG29, 0xcd);
	snd_soc_component_write(component, ES8336_ADC_ALC2_REG2A, 0x08);
	snd_soc_component_write(component, ES8336_ADC_ALC3_REG2B, 0xa0);
	snd_soc_component_write(component, ES8336_ADC_ALC4_REG2C, 0x05);
	snd_soc_component_write(component, ES8336_ADC_ALC5_REG2D, 0x06);
	snd_soc_component_write(component, ES8336_ADC_ALC6_REG2E, 0x61);

	/*
	 * Documentation is unclear, but this value from the vendor driver is
	 * needed otherwise audio output is silent.
	 */
	snd_soc_component_write(component, ES8336_SYS_VMIDSEL, 0xff);

	/*
	 * Documentation for this register is unclear and incomplete,
	 * but here is a vendor-provided value that improves volume
	 * and quality for Intel CHT platforms.
	 */
	snd_soc_component_write(component, ES8336_CLKMGR_ADCOSR, 0x32);
#if 0
	snd_soc_component_write(component, ES8336_SYS_PDN_REG0D, 0x01);
#endif
	return 0;
}

static int es8336_trigger(struct snd_pcm_substream *substream,
				    int cmd, struct snd_soc_dai *dai)
{
	return 0;
}

static const struct snd_soc_dai_ops es8336_ops = {
	.startup = es8336_pcm_startup,
	.hw_params = es8336_pcm_hw_params,
	.set_fmt = es8336_set_dai_fmt,
	.set_sysclk = es8336_set_dai_sysclk,
	.mute_stream = es8336_mute,
	.no_capture_mute = 1,
	.trigger = es8336_trigger,
};

static struct snd_soc_dai_driver es8336_dai = {
	.name = "ES8336 HiFi",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_48000,
		.formats = ES8336_FORMATS,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_48000,
		.formats = ES8336_FORMATS,
	},
	.ops = &es8336_ops,
	.symmetric_rates = 1,
};

static void es8336_enable_micbias_for_mic_gnd_short_detect(
	struct snd_soc_component *component)
{
	struct snd_soc_dapm_context *dapm = snd_soc_component_get_dapm(component);

	snd_soc_dapm_mutex_lock(dapm);
	snd_soc_dapm_force_enable_pin_unlocked(dapm, "Analog power");
	snd_soc_dapm_sync_unlocked(dapm);
	snd_soc_dapm_mutex_unlock(dapm);

	msleep(20);
}

static void es8336_disable_micbias_for_mic_gnd_short_detect(
	struct snd_soc_component *component)
{
	struct snd_soc_dapm_context *dapm = snd_soc_component_get_dapm(component);

	snd_soc_dapm_mutex_lock(dapm);
	snd_soc_dapm_disable_pin_unlocked(dapm, "Analog power");
	snd_soc_dapm_sync_unlocked(dapm);
	snd_soc_dapm_mutex_unlock(dapm);
}

static irqreturn_t es8336_irq(int irq, void *data)
{
	struct es8336_priv *es8336 = data;
	struct snd_soc_component *comp = es8336->component;
	unsigned int flags;
	mutex_lock(&es8336->lock);

	snd_soc_component_write(comp, ES8336_GPIO_DEBOUNCE, 0xf0);
	snd_soc_component_write(comp,0x01,0x7f);
	msleep(500);
	regmap_read(es8336->regmap, ES8336_GPIO_FLAG, &flags);
	if (flags == 0x00)
		goto out; /* Powered-down / reset */

	/* Catch spurious IRQ before set_jack is called */
	if (!es8336->jack)
		goto out;

	if (es8336->jd_inverted)
		flags ^= ES8336_GPIO_FLAG_HP_NOT_INSERTED;

	dev_dbg(comp->dev, "gpio flags %#04x\n", flags);
	if (flags & ES8336_GPIO_FLAG_HP_NOT_INSERTED) {
		gpiod_set_value_cansleep(es8336->pa_enable, true);
		snd_soc_component_write(comp, ES8336_ADC_PDN_LINSEL, 0x20);
	} else if (!(es8336->jack->status & SND_JACK_HEADPHONE)) {
		/* Jack inserted, determine type */
		snd_soc_component_write(comp, ES8336_GPIO_DEBOUNCE, 0xf2);
		snd_soc_component_write(comp, 0x0d, 0x00);
		regmap_read(es8336->regmap, ES8336_GPIO_FLAG, &flags);
		if (es8336->jd_inverted)
			flags ^= ES8336_GPIO_FLAG_HP_NOT_INSERTED;
		dev_dbg(comp->dev, "gpio flags %#04x\n", flags);
		if (flags & ES8336_GPIO_FLAG_HP_NOT_INSERTED) {
			gpiod_set_value_cansleep(es8336->pa_enable, true);
			snd_soc_component_write(comp,ES8336_ADC_PDN_LINSEL,0x20);
		} else if (flags & ES8336_GPIO_FLAG_GM_NOT_SHORTED) {
			/* Open, headset */
			gpiod_set_value_cansleep(es8336->pa_enable, false);
			snd_soc_component_write(comp,ES8336_ADC_PDN_LINSEL,0x30);
			snd_soc_jack_report(es8336->jack,
					    SND_JACK_HEADSET,
					    SND_JACK_HEADSET);
			/* Keep mic-gnd-short detection on for button press */
		} else {
			/* Shorted, headphones */
			gpiod_set_value_cansleep(es8336->pa_enable, false);
			snd_soc_jack_report(es8336->jack,
					    SND_JACK_HEADPHONE,
					    SND_JACK_HEADSET);
			snd_soc_component_write(comp,ES8336_ADC_PDN_LINSEL,0x20);
		}
	} else if (es8336->jack->status & SND_JACK_MICROPHONE) {
		/* Interrupt while jack inserted, report button state */
		gpiod_set_value_cansleep(es8336->pa_enable, false);
		snd_soc_component_write(comp,ES8336_ADC_PDN_LINSEL,0x30);
		if (flags & ES8336_GPIO_FLAG_GM_NOT_SHORTED) {
			/* Open, button release */
			snd_soc_jack_report(es8336->jack, 0, SND_JACK_BTN_0);
		} else {
			/* Short, button press */
			snd_soc_jack_report(es8336->jack,
					    SND_JACK_BTN_0,
					    SND_JACK_BTN_0);
		}
	}

out:
	mdelay(5);
	snd_soc_component_write(comp, ES8336_GPIO_DEBOUNCE, 0xf2);
	mutex_unlock(&es8336->lock);
	return IRQ_HANDLED;
}

static void es8336_enable_jack_detect(struct snd_soc_component *component,
				      struct snd_soc_jack *jack)
{
	struct es8336_priv *es8336 = snd_soc_component_get_drvdata(component);
	/* flag for running the init procedure only once */
	static int done = 0;

	es8336->jd_inverted = false;

	/*
	 * Init es8336->jd_inverted here and not in the probe, as we cannot
	 * guarantee that the bytchr-es8336 driver, which might set this
	 * property, will probe before us.
	 */
	es8336->jd_inverted = device_property_read_bool(component->dev,
							"everest,jack-detect-inverted");

	if(!done)
	{
		mutex_lock(&es8336->lock);

		es8336->jack = jack;

		if (es8336->jack->status & SND_JACK_MICROPHONE)
			es8336_enable_micbias_for_mic_gnd_short_detect(component);

		snd_soc_component_update_bits(component, ES8336_GPIO_DEBOUNCE,
			ES8336_GPIO_ENABLE_INTERRUPT,
			ES8336_GPIO_ENABLE_INTERRUPT);

		mutex_unlock(&es8336->lock);

		/* Enable irq and sync initial jack state */
		enable_irq(es8336->irq);
		es8336_irq(es8336->irq, es8336);
		done = 1;
	}
}

static void es8336_disable_jack_detect(struct snd_soc_component *component)
{
	struct es8336_priv *es8336 = snd_soc_component_get_drvdata(component);

	if (!es8336->jack)
		return; /* Already disabled (or never enabled) */

	disable_irq(es8336->irq);

	mutex_lock(&es8336->lock);

	snd_soc_component_update_bits(component, ES8336_GPIO_DEBOUNCE,
				      ES8336_GPIO_ENABLE_INTERRUPT, 0);

	if (es8336->jack->status & SND_JACK_MICROPHONE) {
		es8336_disable_micbias_for_mic_gnd_short_detect(component);
		snd_soc_jack_report(es8336->jack, 0, SND_JACK_BTN_0);
	}

	es8336->jack = NULL;

	mutex_unlock(&es8336->lock);
}

static int es8336_set_jack(struct snd_soc_component *component,
			   struct snd_soc_jack *jack, void *data)
{
	if (jack)
		es8336_enable_jack_detect(component, jack);
	else
		es8336_disable_jack_detect(component);

	return 0;
}

static int es8336_probe(struct snd_soc_component *component)
{
	struct es8336_priv *es8336 = snd_soc_component_get_drvdata(component);
	int ret;

	es8336->component = component;

	es8336->pa_enable = devm_gpiod_get_optional(component->dev, "pa,enable",
						GPIOD_OUT_LOW);
	if (IS_ERR(es8336->pa_enable)) {
		ret = PTR_ERR(es8336->pa_enable);
		printk("%s, Failed to get pa enable line: %d\n", __func__,  ret);
		return ret;
	}
	gpiod_set_value_cansleep(es8336->pa_enable, true);

	es8336->mclk = devm_clk_get_optional(component->dev, "mclk");
	if (IS_ERR(es8336->mclk)) {
		dev_err(component->dev, "unable to get mclk\n");
		return PTR_ERR(es8336->mclk);
	}
	if (!es8336->mclk)
		dev_warn(component->dev, "assuming static mclk\n");

	ret = clk_prepare_enable(es8336->mclk);
	if (ret) {
		dev_err(component->dev, "unable to enable mclk\n");
		return ret;
	}

	/* Reset codec and enable current state machine */
	snd_soc_component_write(component, ES8336_RESET_REG00, 0x3f);
	usleep_range(5000, 5500);
	snd_soc_component_write(component, ES8336_RESET_REG00, 0x00);
	msleep(30);
	snd_soc_component_write(component, ES8336_SYS_VMIDSEL_REG0C, 0xFF);
	msleep(30);
	snd_soc_component_write(component, ES8336_CLKMGR_CLKSEL_REG02, 0x09);
	snd_soc_component_write(component, ES8336_CLKMGR_ADCOSR_REG03, 0x32);
	snd_soc_component_write(component, ES8336_CLKMGR_ADCDIV1_REG04, 0x11);
	snd_soc_component_write(component, ES8336_CLKMGR_ADCDIV2_REG05, 0x90);
	snd_soc_component_write(component, ES8336_CLKMGR_DACDIV1_REG06, 0x11);
	snd_soc_component_write(component, ES8336_CLKMGR_DACDIV2_REG07, 0x90);
	snd_soc_component_write(component, ES8336_CLKMGR_CPDIV_REG08, 0x00);
	snd_soc_component_write(component, ES8336_SDP_MS_BCKDIV_REG09, 0x04);
	snd_soc_component_write(component, ES8336_CLKMGR_CLKSW_REG01, 0x7F);
	snd_soc_component_write(component, ES8336_CAL_TYPE_REG1C, 0x0F);
	snd_soc_component_write(component, ES8336_CAL_HPLIV_REG1E, 0x90);
	snd_soc_component_write(component, ES8336_CAL_HPRIV_REG1F, 0x90);
	snd_soc_component_write(component, ES8336_ADC_VOLUME_REG27, 0x00);
	snd_soc_component_write(component, ES8336_ADC_PDN_LINSEL_REG22, 0xc0);
	snd_soc_component_write(component, ES8336_ADC_D2SEPGA_REG24, 0x00);
	snd_soc_component_write(component, ES8336_ADC_DMIC_REG25, 0x08);
	snd_soc_component_write(component, ES8336_DAC_SET2_REG31, 0x20);
	snd_soc_component_write(component, ES8336_DAC_SET3_REG32, 0x00);
	snd_soc_component_write(component, ES8336_DAC_VOLL_REG33, 0x00);
	snd_soc_component_write(component, ES8336_DAC_VOLR_REG34, 0x00);
	snd_soc_component_write(component, ES8336_SDP_ADCFMT_REG0A, 0x00);
	snd_soc_component_write(component, ES8336_SDP_DACFMT_REG0B, 0x00);
	snd_soc_component_write(component, ES8336_SYS_VMIDLOW_REG10, 0x11);
	snd_soc_component_write(component, ES8336_SYS_VSEL_REG11, 0xFC);
	snd_soc_component_write(component, ES8336_SYS_REF_REG12, 0x28);
	snd_soc_component_write(component, ES8336_SYS_LP1_REG0E, 0x04);
	snd_soc_component_write(component, ES8336_SYS_LP2_REG0F, 0x0C);
	snd_soc_component_write(component, ES8336_DAC_PDN_REG2F, 0x11);
	snd_soc_component_write(component, ES8336_HPMIX_SEL_REG13, 0x00);
	snd_soc_component_write(component, ES8336_HPMIX_SWITCH_REG14, 0x88);
	snd_soc_component_write(component, ES8336_HPMIX_PDN_REG15, 0x00);
	snd_soc_component_write(component, ES8336_HPMIX_VOL_REG16, 0xBB);
	snd_soc_component_write(component, ES8336_CPHP_PDN2_REG1A, 0x10);
	snd_soc_component_write(component, ES8336_CPHP_LDOCTL_REG1B, 0x30);
	snd_soc_component_write(component, ES8336_CPHP_PDN1_REG19, 0x02);
	snd_soc_component_write(component, ES8336_CPHP_ICAL_VOL_REG18, 0x00);
	snd_soc_component_write(component, ES8336_GPIO_SEL_REG4D, 0x00);
	snd_soc_component_write(component, ES8336_GPIO_DEBOUNCE, 0xf2);
	snd_soc_component_write(component, ES8336_TESTMODE_REG50, 0xA0);
	snd_soc_component_write(component,0x26,0x00);
	snd_soc_component_write(component, ES8336_TEST1_REG51, 0x00);
	snd_soc_component_write(component, ES8336_TEST2_REG52, 0x00);
	snd_soc_component_write(component, ES8336_SYS_PDN_REG0D, 0x00);
	snd_soc_component_write(component, ES8336_RESET_REG00, 0xC0);
	msleep(50);
	snd_soc_component_write(component, ES8336_ADC_PGAGAIN_REG23, 0x60);
	snd_soc_component_write(component, ES8336_ADC_PDN_LINSEL_REG22, 0x30);
	snd_soc_component_write(component, ES8336_ADC_D2SEPGA_REG24, 0x00);
	/* adc ds mode, HPF enable */
	snd_soc_component_write(component, ES8336_ADC_DMIC_REG25, 0x08);
	snd_soc_component_write(component, ES8336_ADC_ALC1_REG29, 0xcd);
	snd_soc_component_write(component, ES8336_ADC_ALC2_REG2A, 0x08);
	snd_soc_component_write(component, ES8336_ADC_ALC3_REG2B, 0xa0);
	snd_soc_component_write(component, ES8336_ADC_ALC4_REG2C, 0x05);
	snd_soc_component_write(component, ES8336_ADC_ALC5_REG2D, 0x06);
	snd_soc_component_write(component, ES8336_ADC_ALC6_REG2E, 0x61);
	/*
	 * Documentation is unclear, but this value from the vendor
	 * driver is needed otherwise audio output is silent.
	 */
	snd_soc_component_write(component, ES8336_SYS_VMIDSEL, 0xff);

	/*
	 * Documentation for this register is unclear and incomplete,
	 * but here is a vendor-provided value that improves volume
	 * and quality for Intel CHT platforms.
	 */
	snd_soc_component_write(component, ES8336_CLKMGR_ADCOSR, 0x32);
	snd_soc_component_write(component, ES8336_SYS_PDN_REG0D, 0x01);

	return 0;
}

static void es8336_remove(struct snd_soc_component *component)
{
	struct es8336_priv *es8336 = snd_soc_component_get_drvdata(component);

	clk_disable_unprepare(es8336->mclk);
}

static const struct snd_soc_component_driver soc_component_dev_es8336 = {
	.probe			= es8336_probe,
	.remove			= es8336_remove,
	.resume			= es8336_resume,
	.set_jack		= es8336_set_jack,
	.controls		= es8336_snd_controls,
	.num_controls		= ARRAY_SIZE(es8336_snd_controls),
	.dapm_widgets		= es8336_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(es8336_dapm_widgets),
	.dapm_routes		= es8336_dapm_routes,
	.num_dapm_routes	= ARRAY_SIZE(es8336_dapm_routes),
	.use_pmdown_time	= 1,
	.endianness		= 1,
	.non_legacy_dai_naming	= 1,
};

static const struct regmap_range es8336_volatile_ranges[] = {
	regmap_reg_range(ES8336_GPIO_FLAG, ES8336_GPIO_FLAG),
};

static const struct regmap_access_table es8336_volatile_table = {
	.yes_ranges	= es8336_volatile_ranges,
	.n_yes_ranges	= ARRAY_SIZE(es8336_volatile_ranges),
};

static const struct regmap_config es8336_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0x53,
	.volatile_table	= &es8336_volatile_table,
	.cache_type = REGCACHE_RBTREE,
};

static int es8336_i2c_probe(struct i2c_client *i2c_client,
			    const struct i2c_device_id *id)
{
	struct device *dev = &i2c_client->dev;
	struct es8336_priv *es8336;
	int ret;

	es8336 = devm_kzalloc(&i2c_client->dev, sizeof(struct es8336_priv),
			      GFP_KERNEL);
	if (es8336 == NULL)
		return -ENOMEM;

	i2c_set_clientdata(i2c_client, es8336);

	es8336->regmap = devm_regmap_init_i2c(i2c_client, &es8336_regmap);
	if (IS_ERR(es8336->regmap))
		return PTR_ERR(es8336->regmap);

	es8336->irq = i2c_client->irq;
	mutex_init(&es8336->lock);

	ret = devm_request_threaded_irq(dev, es8336->irq, NULL, es8336_irq,
					IRQF_TRIGGER_HIGH | IRQF_ONESHOT,
					"es8336", es8336);
	if (ret == 0) {
		/* Gets re-enabled by es8336_set_jack() */
		disable_irq(es8336->irq);
	} else {
		dev_warn(dev, "Failed to get IRQ %d: %d\n", es8336->irq, ret);
		es8336->irq = -ENXIO;
	}

	return devm_snd_soc_register_component(&i2c_client->dev,
				      &soc_component_dev_es8336,
				      &es8336_dai, 1);
}

static const struct i2c_device_id es8336_i2c_id[] = {
	{"es8336", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, es8336_i2c_id);

#ifdef CONFIG_OF
static const struct of_device_id es8336_of_match[] = {
	{ .compatible = "everest,es8336", },
	{},
};
MODULE_DEVICE_TABLE(of, es8336_of_match);
#endif

#ifdef CONFIG_ACPI
static const struct acpi_device_id es8336_acpi_match[] = {
	{"ESSX8336", 0},
	{},
};
MODULE_DEVICE_TABLE(acpi, es8336_acpi_match);
#endif

static struct i2c_driver es8336_i2c_driver = {
	.driver = {
		.name			= "es8336",
		.acpi_match_table	= ACPI_PTR(es8336_acpi_match),
		.of_match_table		= of_match_ptr(es8336_of_match),
	},
	.probe		= es8336_i2c_probe,
	.id_table	= es8336_i2c_id,
};
module_i2c_driver(es8336_i2c_driver);

MODULE_DESCRIPTION("Everest Semi ES8336 ALSA SoC Codec Driver");
MODULE_AUTHOR("David Yang <yangxiaohua@everest-semi.com>");
MODULE_LICENSE("GPL v2");
