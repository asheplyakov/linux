/*
 * es8326.c -- es8326 ALSA SoC audio driver
 *
 * Copyright (c) 2016 Rockchip Electronics Co. Ltd.
 *
 * Author: Mark Brown <will@everset-semi.com>
 * Author: Jianqun Xu <jay.xu@rock-chips.com>
 * Author: Nickey Yang <nickey.yang@rock-chips.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/of_gpio.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/tlv.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <linux/proc_fs.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <sound/jack.h>
#include "es8326.h"

/* codec private data */
struct es8326_priv {
	struct mutex lock;
	struct clk *mclk;
	struct regmap *regmap;
	struct snd_soc_component *component;
	struct snd_soc_jack *jack;
	int irq;
	unsigned int sysclk;
	struct snd_pcm_hw_constraint_list sysclk_constraints;
	bool spk_gpio_level;
	bool jd_inverted;
};

/*
 * ES8326 controls
 */
static const SNDRV_CTL_TLVD_DECLARE_DB_SCALE(es8326_dac_vol_tlv, -9550, 50, 0);
static const SNDRV_CTL_TLVD_DECLARE_DB_SCALE(es8326_adc_vol_tlv, -9550, 50, 0);
static const SNDRV_CTL_TLVD_DECLARE_DB_SCALE(es8326_adc_pga_tlv, 0, 600, 0);
static const SNDRV_CTL_TLVD_DECLARE_DB_SCALE(es8326_softramp_rate, 0, 100, 0);
static const SNDRV_CTL_TLVD_DECLARE_DB_SCALE(es8326_drc_target_tlv, -3200,
					     200, 0);
static const SNDRV_CTL_TLVD_DECLARE_DB_SCALE(es8326_drc_recovery_tlv, -125,
					     250, 0);

static const char * const es8326_winsize[] = {
	"0.25db/2 LRCK",
	"0.25db/4 LRCK",
	"0.25db/8 LRCK",
	"0.25db/16 LRCK",
	"0.25db/32 LRCK",
	"0.25db/64 LRCK",
	"0.25db/128 LRCK",
	"0.25db/256 LRCK",
	"0.25db/512 LRCK",
	"0.25db/1024 LRCK",
	"0.25db/2048 LRCK",
	"0.25db/4096 LRCK",
	"0.25db/8192 LRCK",
	"0.25db/16384 LRCK",
	"0.25db/32768 LRCK",
	"0.25db/65536 LRCK",
};

static const char * const es8326_dacpol_txt[] = {
	"Normal", "R Invert", "L Invert", "L + R Invert"
};
static const struct soc_enum es8326_dacpol =
	SOC_ENUM_SINGLE(ES8326_DAC_DSM, 4, ARRAY_SIZE(es8326_dacpol_txt),
			es8326_dacpol_txt);
static const struct soc_enum es8326_alc_winsize =
	SOC_ENUM_SINGLE(ES8326_ADC_RAMPRATE, 4, ARRAY_SIZE(es8326_winsize),
			es8326_winsize);
static const struct soc_enum es8326_drc_winsize =
	SOC_ENUM_SINGLE(ES8326_DRC_WINSIZE, 4, ARRAY_SIZE(es8326_winsize),
			es8326_winsize);

static const struct snd_kcontrol_new es8326_snd_controls[] = {
	SOC_SINGLE_TLV("DAC Playback Volume", ES8326_DAC_VOL, 0, 0xff, 0,
		       es8326_dac_vol_tlv),
	SOC_ENUM("Playback Polarity", es8326_dacpol),
	SOC_SINGLE_TLV("DAC Ramp Rate", ES8326_DAC_RAMPRATE, 0, 0x0f, 0,
		       es8326_softramp_rate),
	SOC_SINGLE("DRC Switch", ES8326_DRC_RECOVERY, 3, 1, 0),
	SOC_SINGLE_TLV("DRC Recovery Level", ES8326_DRC_RECOVERY, 0, 4, 0,
		       es8326_drc_recovery_tlv),
	SOC_ENUM("DRC Winsize", es8326_drc_winsize),
	SOC_SINGLE_TLV("DRC Target Level", ES8326_DRC_WINSIZE, 0, 0x0f, 0,
		       es8326_drc_target_tlv),

	SOC_DOUBLE_R_TLV("ADC Capture Volume", ES8326_ADC1_VOL,
			 ES8326_ADC2_VOL, 0, 0xff, 0, es8326_adc_vol_tlv),
	SOC_DOUBLE_TLV("ADC PGA Gain Volume", ES8326_ADC_SCALE, 4, 0, 5, 0,
		       es8326_adc_pga_tlv),
	SOC_SINGLE_TLV("ADC Ramp Rate", ES8326_ADC_RAMPRATE, 0, 0x0f, 0,
		       es8326_softramp_rate),
	SOC_SINGLE("ALC Switch", ES8326_ALC_RECOVERY, 3, 1, 0),
	SOC_SINGLE_TLV("ALC Recovery Level", ES8326_ALC_LEVEL, 0, 4, 0,
		       es8326_drc_recovery_tlv),
	SOC_ENUM("ALC Winsize", es8326_alc_winsize),
	SOC_SINGLE_TLV("ALC Target Level", ES8326_ALC_LEVEL, 0, 0x0f, 0,
		       es8326_drc_target_tlv),
};

static const char * const es8326_adc_src_txt[] = {
	"Analog MIC", "DMIC SDINOUT2"
};
static const unsigned int es8326_adc_src_values[] = { 0, 0x44 };
static const struct soc_enum es8326_adc1_src_enum =
	SOC_VALUE_ENUM_SINGLE(ES8326_ADC1_SRC, 0, 0xff,
			      ARRAY_SIZE(es8326_adc_src_txt),
			      es8326_adc_src_txt,
			      es8326_adc_src_values);
static const struct snd_kcontrol_new es8326_mic_mux_controls =
	SOC_DAPM_ENUM("Route", es8326_adc1_src_enum);

static const struct snd_soc_dapm_widget es8326_dapm_widgets[] = {
	SND_SOC_DAPM_REG(snd_soc_dapm_supply, "Mic Bias", ES8326_ANA_MICBIAS,
			 2, 0x3, 3, 0),

	SND_SOC_DAPM_INPUT("MIC1"),
	SND_SOC_DAPM_INPUT("MIC2"),

	SND_SOC_DAPM_MUX("Mic Mux", SND_SOC_NOPM, 0, 0,
			 &es8326_mic_mux_controls),

    	/* Digital Interface */
	SND_SOC_DAPM_AIF_OUT("I2S OUT", "I2S1 Capture", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("I2S IN", "I2S1 Playback", 0, SND_SOC_NOPM, 0, 0),

	SND_SOC_DAPM_DAC("Right DAC", NULL, ES8326_ANA_PWR, 0, 1),
	SND_SOC_DAPM_DAC("Left DAC", NULL, ES8326_ANA_PWR, 1, 1),
	SND_SOC_DAPM_PGA("LHPMIX", ES8326_DAC2HPMIX, 7, 0, NULL, 0),
	SND_SOC_DAPM_PGA("RHPMIX", ES8326_DAC2HPMIX, 3, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("HPOR Cal", ES8326_HP_CTL, 7, 1, NULL, 0),
	SND_SOC_DAPM_SUPPLY("HPOL Cal", ES8326_HP_CTL, 3, 1, NULL, 0),
	SND_SOC_DAPM_REG(snd_soc_dapm_supply, "HPOR Supply", ES8326_HP_CTL,
			 4, 0x7, 7, 0),
	SND_SOC_DAPM_REG(snd_soc_dapm_supply, "HPOL Supply", ES8326_HP_CTL,
			 0, 0x7, 7, 0),
	SND_SOC_DAPM_OUTPUT("HPOL"),
	SND_SOC_DAPM_OUTPUT("HPOR"),
};

static const struct snd_soc_dapm_route es8326_dapm_routes[] = {
	/* Recording */
	{"MIC1", NULL, "Mic Bias"},
	{"MIC2", NULL, "Mic Bias"},

	{"Mic Mux", "DMIC SDINOUT2", "MIC1"},
	{"Mic Mux", "Analog MIC", "MIC2"},

	{"I2S OUT", NULL, "Mic Mux"},

	/* Playback */
	{"Right DAC", NULL, "I2S IN"},
	{"Left DAC", NULL, "I2S IN"},

	{"LHPMIX", NULL, "Left DAC"},
	{"RHPMIX", NULL, "Right DAC"},

	{"HPOR", NULL , "HPOR Cal"},
	{"HPOL", NULL , "HPOL Cal"},

	{"HPOR", NULL , "HPOR Supply"},
	{"HPOL", NULL , "HPOL Supply"},

	{"HPOL", NULL, "LHPMIX"},
	{"HPOR", NULL, "RHPMIX"},
};

struct es8326_clk_coeff {
	u16 fs;
	u32 rate;
	u32 mclk;
	u8 reg4;
	u8 reg5;
	u8 reg6;
	u8 reg7;
	u8 reg8;
	u8 reg9;
	u8 rega;
	u8 regb;
};

/* codec hifi mclk clock divider coefficients */
static const struct es8326_clk_coeff es8326_coeff_div[] = {
	{  32,  8000,   256000, 0x60, 0x00, 0x0f, 0x75, 0x0a, 0x1b, 0x1f, 0x7f},
	{  32, 16000,   512000, 0x20, 0x00, 0x0d, 0x75, 0x0a, 0x1b, 0x1f, 0x3f},
	{  32, 44100,  1411200, 0x00, 0x00, 0x13, 0x2d, 0x0a, 0x0a, 0x1f, 0x1f},
	{  32, 48000,  1536000, 0x00, 0x00, 0x13, 0x2d, 0x0a, 0x0a, 0x1f, 0x1f},
	{  36,  8000,   288000, 0x20, 0x00, 0x0d, 0x75, 0x0a, 0x1b, 0x23, 0x47},
	{  36, 16000,   576000, 0x20, 0x00, 0x0d, 0x75, 0x0a, 0x1b, 0x23, 0x47},
	{  48,  8000,   384000, 0x60, 0x02, 0x1f, 0x75, 0x0a, 0x1b, 0x1f, 0x7f},
	{  48, 16000,   768000, 0x20, 0x02, 0x0f, 0x75, 0x0a, 0x1b, 0x1f, 0x3f},
	{  48, 48000,  2304000, 0x00, 0x02, 0x0d, 0x2d, 0x0a, 0x0a, 0x1f, 0x1f},
	{  64,  8000,   512000, 0x60, 0x00, 0x0d, 0x75, 0x0a, 0x1b, 0x1f, 0x7f},
	{  64, 16000,  1024000, 0x20, 0x00, 0x05, 0x75, 0x0a, 0x1b, 0x1f, 0x3f},

	{  64, 44100,  2822400, 0x00, 0x00, 0x11, 0x2d, 0x0a, 0x0a, 0x1f, 0x1f},
	{  64, 48000,  3072000, 0x00, 0x00, 0x11, 0x2d, 0x0a, 0x0a, 0x1f, 0x1f},
	{  72,  8000,   576000, 0x20, 0x00, 0x13, 0x35, 0x0a, 0x1b, 0x23, 0x47},
	{  72, 16000,  1152000, 0x20, 0x00, 0x05, 0x75, 0x0a, 0x1b, 0x23, 0x47},
	{  96,  8000,   768000, 0x60, 0x02, 0x1d, 0x75, 0x0a, 0x1b, 0x1f, 0x7f},
	{  96, 16000,  1536000, 0x20, 0x02, 0x0d, 0x75, 0x0a, 0x1b, 0x1f, 0x3f},
	{ 100, 48000,  4800000, 0x04, 0x04, 0x1f, 0x2d, 0x0a, 0x0a, 0x1f, 0x1f},
	{ 125, 48000,  6000000, 0x04, 0x04, 0x1f, 0x2d, 0x0a, 0x0a, 0x27, 0x27},
	{ 128,  8000,  1024000, 0x60, 0x00, 0x13, 0x35, 0x0a, 0x1b, 0x1f, 0x7f},
	{ 128, 16000,  2048000, 0x20, 0x00, 0x11, 0x35, 0x0a, 0x1b, 0x1f, 0x3f},

	{ 128, 44100,  5644800, 0x00, 0x00, 0x01, 0x2d, 0x0a, 0x0a, 0x1f, 0x1f},
	{ 128, 48000,  6144000, 0x00, 0x00, 0x01, 0x2d, 0x0a, 0x0a, 0x1f, 0x1f},
	{ 144,  8000,  1152000, 0x20, 0x00, 0x03, 0x35, 0x0a, 0x1b, 0x23, 0x47},
	{ 144, 16000,  2304000, 0x20, 0x00, 0x11, 0x35, 0x0a, 0x1b, 0x23, 0x47},
	{ 192,  8000,  1536000, 0x60, 0x02, 0x0d, 0x75, 0x0a, 0x1b, 0x1f, 0x7f},
	{ 192, 16000,  3072000, 0x20, 0x02, 0x05, 0x75, 0x0a, 0x1b, 0x1f, 0x3f},
	{ 200, 48000,  9600000, 0x04, 0x04, 0x0f, 0x2d, 0x0a, 0x0a, 0x1f, 0x1f},
	{ 250, 48000, 12000000, 0x04, 0x04, 0x0f, 0x2d, 0x0a, 0x0a, 0x27, 0x27},
	{ 256,  8000,  2048000, 0x60, 0x00, 0x11, 0x35, 0x0a, 0x1b, 0x1f, 0x7f},
	{ 256, 16000,  4096000, 0x20, 0x00, 0x01, 0x35, 0x0a, 0x1b, 0x1f, 0x3f},

	{ 256, 44100, 11289600, 0x00, 0x00, 0x10, 0x2d, 0x0a, 0x0a, 0x1f, 0x1f},
	{ 256, 48000, 12288000, 0x00, 0x00, 0x10, 0x2d, 0x0a, 0x0a, 0x1f, 0x1f},
	{ 288,  8000,  2304000, 0x20, 0x00, 0x01, 0x35, 0x0a, 0x1b, 0x23, 0x47},
	{ 384,  8000,  3072000, 0x60, 0x02, 0x05, 0x75, 0x0a, 0x1b, 0x1f, 0x7f},
	{ 384, 16000,  6144000, 0x20, 0x02, 0x03, 0x35, 0x0a, 0x1b, 0x1f, 0x3f},
	{ 384, 48000, 18432000, 0x00, 0x02, 0x01, 0x2d, 0x0a, 0x0a, 0x1f, 0x1f},
	{ 400, 48000, 19200000, 0x09, 0x04, 0x0f, 0x6d, 0x3a, 0x0a, 0x4f, 0x1f},
	{ 500, 48000, 24000000, 0x18, 0x04, 0x1f, 0x2d, 0x0a, 0x0a, 0x1f, 0x1f},
	{ 512,  8000,  4096000, 0x60, 0x00, 0x01, 0x35, 0x0a, 0x1b, 0x1f, 0x7f},
	{ 512, 16000,  8192000, 0x20, 0x00, 0x10, 0x35, 0x0a, 0x1b, 0x1f, 0x3f},

	{ 512, 44100, 22579200, 0x00, 0x00, 0x00, 0x2d, 0x0a, 0x0a, 0x1f, 0x1f},
	{ 512, 48000, 24576000, 0x00, 0x00, 0x00, 0x2d, 0x0a, 0x0a, 0x1f, 0x1f},
	{ 768,  8000,  6144000, 0x60, 0x02, 0x11, 0x35, 0x0a, 0x1b, 0x1f, 0x7f},
	{ 768, 16000, 12288000, 0x20, 0x02, 0x01, 0x35, 0x0a, 0x1b, 0x1f, 0x3f},
	{ 800, 48000, 38400000, 0x00, 0x18, 0x13, 0x2d, 0x0a, 0x0a, 0x1f, 0x1f},
	{ 812, 16000, 13000000, 0x0c, 0x04, 0x0f, 0x2d, 0x0a, 0x0a, 0x31, 0x31},
	{1024,  8000,  8192000, 0x60, 0x00, 0x10, 0x35, 0x0a, 0x1b, 0x1f, 0x7f},
	{1024, 16000, 16384000, 0x20, 0x00, 0x00, 0x35, 0x0a, 0x1b, 0x1f, 0x3f},
	{1152, 16000, 18432000, 0x20, 0x08, 0x11, 0x35, 0x0a, 0x1b, 0x1f, 0x3f},
	{1536,  8000, 12288000, 0x60, 0x02, 0x01, 0x35, 0x0a, 0x1b, 0x1f, 0x7f},

	{1536, 16000, 24576000, 0x20, 0x02, 0x10, 0x35, 0x0a, 0x1b, 0x1f, 0x3f},
	{1625,  8000, 13000000, 0x0c, 0x18, 0x1f, 0x2d, 0x0a, 0x0a, 0x27, 0x27},
	{1625, 16000, 26000000, 0x0c, 0x18, 0x1f, 0x2d, 0x0a, 0x0a, 0x27, 0x27},
	{2048,  8000, 16384000, 0x60, 0x00, 0x00, 0x35, 0x0a, 0x1b, 0x1f, 0x7f},
	{2304,  8000, 18432000, 0x40, 0x02, 0x10, 0x35, 0x0a, 0x1b, 0x1f, 0x5f},
	{3072,  8000, 24576000, 0x60, 0x02, 0x10, 0x35, 0x0a, 0x1b, 0x1f, 0x7f},
	{3250,  8000, 26000000, 0x0c, 0x18, 0x0f, 0x2d, 0x0a, 0x0a, 0x27, 0x27},
	{  21, 48000,  1024320, 0x00, 0x00, 0x09, 0x2d, 0x0a, 0x0a, 0x1f, 0x1f},
	{ 541, 48000, 26000000, 0x00, 0x00, 0x00, 0x35, 0x0a, 0x1b, 0x20, 0x20},
};

static const struct es8326_clk_coeff *es8326_get_coeff(int mclk, int rate)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(es8326_coeff_div); i++) {
		if (es8326_coeff_div[i].rate == rate &&
		    es8326_coeff_div[i].mclk == mclk)
			return &es8326_coeff_div[i];
	}

	return NULL;
}

/* The set of rates we can generate from the above for each SYSCLK */

static unsigned int es8326_rates_12288[] = {
	8000, 12000, 16000, 24000, 24000, 32000, 48000, 96000,
};

static struct snd_pcm_hw_constraint_list es8326_constraints_12288 = {
	.count = ARRAY_SIZE(es8326_rates_12288),
	.list = es8326_rates_12288,
};

static unsigned int es8326_rates_112896[] = {
	8000, 11025, 22050, 44100,
};

static struct snd_pcm_hw_constraint_list es8326_constraints_112896 = {
	.count = ARRAY_SIZE(es8326_rates_112896),
	.list = es8326_rates_112896,
};

static unsigned int es8326_rates_12[] = {
	8000, 11025, 12000, 16000, 22050, 24000, 32000, 44100, 48000,
	48000, 88235, 96000,
};

static struct snd_pcm_hw_constraint_list es8326_constraints_12 = {
	.count = ARRAY_SIZE(es8326_rates_12),
	.list = es8326_rates_12,
};

/*
 * Note that this should be called from init rather than from hw_params.
 */
static int es8326_set_dai_sysclk(struct snd_soc_dai *codec_dai,
				 int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_component *component = codec_dai->component;
	struct es8326_priv *es8326 = snd_soc_component_get_drvdata(component);
	int ret;

	es8326->sysclk = freq;

	if (freq == 0) {
		es8326->sysclk_constraints.list = NULL;
		es8326->sysclk_constraints.count = 0;

		return 0;
	}

	ret = clk_set_rate(es8326->mclk, freq);
	if (ret)
		return ret;

	switch (freq) {
	case 11289600:
	case 18432000:
	case 22579200:
	case 36864000:
		es8326->sysclk_constraints = es8326_constraints_112896;
		return 0;

	case 12288000:
	case 16934400:
	case 24576000:
	case 33868800:
		es8326->sysclk_constraints = es8326_constraints_12288;
		return 0;

	case 12000000:
	case 19200000:
	case 24000000:
		es8326->sysclk_constraints = es8326_constraints_12;
		return 0;

	default:
		return -EINVAL;
	}
}

static int es8326_set_dai_fmt(struct snd_soc_dai *codec_dai, unsigned int fmt)
{
	struct snd_soc_component *component = codec_dai->component;
	u8 iface;

	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		iface = ES8326_FMT_SDP_FMT_I2S;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		iface = ES8326_FMT_SDP_FMT_LJ;
		break;
	case SND_SOC_DAIFMT_DSP_A:
	case SND_SOC_DAIFMT_DSP_B:
		iface = ES8326_FMT_SDP_FMT_DSP;
		break;
	default:
		return -EINVAL;
	}

	snd_soc_component_write(component, ES8326_FMT, iface);
	snd_soc_component_update_bits(component, ES8326_FMT,
				      ES8326_FMT_SDP_FMT_MASK, iface);

	return 0;
}

static int es8326_pcm_startup(struct snd_pcm_substream *substream,
			      struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	struct es8326_priv *es8326 = snd_soc_component_get_drvdata(component);

	/* The set of sample rates that can be supported depends on the
	 * MCLK supplied to the CODEC - enforce this.
	 */

	if (es8326->sysclk_constraints.list)
		snd_pcm_hw_constraint_list(substream->runtime, 0,
					   SNDRV_PCM_HW_PARAM_RATE,
					   &es8326->sysclk_constraints);

	return 0;
}

static int es8326_pcm_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	struct es8326_priv *es8326 = snd_soc_component_get_drvdata(component);
	const struct es8326_clk_coeff *coeff;
	u8 wordlen;

	coeff = es8326_get_coeff(es8326->sysclk, params_rate(params));
	if (!coeff)
		return -EINVAL;

	/* bit size */
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		wordlen = ES8326_FMT_SDP_WL_16;
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		wordlen = ES8326_FMT_SDP_WL_20;
		break;
	case SNDRV_PCM_FORMAT_S18_3LE:
		wordlen = ES8326_FMT_SDP_WL_18;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		wordlen = ES8326_FMT_SDP_WL_24;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		wordlen = ES8326_FMT_SDP_WL_32;
		break;
	default:
		return -EINVAL;
	}

	snd_soc_component_update_bits(component, ES8326_FMT,
				      ES8326_FMT_SDP_WL_MASK, wordlen);

	snd_soc_component_write(component, ES8326_CLK_DIV1, coeff->reg4);
	snd_soc_component_write(component, ES8326_CLK_DIV2, coeff->reg5);
	snd_soc_component_write(component, ES8326_CLK_DLL, coeff->reg6);
	snd_soc_component_write(component, ES8326_CLK_MUX, coeff->reg7);
	snd_soc_component_write(component, ES8326_CLK_ADC_SEL, coeff->reg8);
	snd_soc_component_write(component, ES8326_CLK_DAC_SEL, coeff->reg9);
	snd_soc_component_write(component, ES8326_CLK_ADC_OSR, coeff->rega);
	snd_soc_component_write(component, ES8326_CLK_DAC_OSR, coeff->regb);
	return 0;
}

static int es8326_mute(struct snd_soc_dai *dai, int mute, int stream)
{
	switch (stream) {
	case SNDRV_PCM_STREAM_PLAYBACK:
		snd_soc_component_update_bits(dai->component, ES8326_DAC_MUTE,
				ES8326_DAC_MUTE_S2P_MUTE_MASK,
				mute ?  ES8326_DAC_MUTE_S2P_MUTE_MASK : 0);
		break;
	case SNDRV_PCM_STREAM_CAPTURE:
		snd_soc_component_update_bits(dai->component, ES8326_ADC_MUTE,
				ES8326_ADC_MUTE_P2S_MUTE_MASK,
				mute ? ES8326_ADC_MUTE_P2S_MUTE_MASK : 0);
		break;
	}

	return 0;
}

#define ES8326_FORMATS	(SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE | \
			 SNDRV_PCM_FMTBIT_S24_LE)

static struct snd_soc_dai_ops es8326_ops = {
	.startup = es8326_pcm_startup,
	.hw_params = es8326_pcm_hw_params,
	.set_fmt = es8326_set_dai_fmt,
	.set_sysclk = es8326_set_dai_sysclk,
	.mute_stream = es8326_mute,
};

static struct snd_soc_dai_driver es8326_dai = {
	.name = "ES8316 HiFi",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_96000,
		.formats = ES8326_FORMATS,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_96000,
		.formats = ES8326_FORMATS,
	},
	.ops = &es8326_ops,
	.symmetric_rate = 1,
};

static void es8326_enable_micbias_for_mic_gnd_short_detect(
					struct snd_soc_component *component)
{
	struct snd_soc_dapm_context *dapm = snd_soc_component_get_dapm(component);

	snd_soc_dapm_force_enable_pin(dapm, "Mic Bias");

	msleep(500);
}

static void es8326_disable_micbias_for_mic_gnd_short_detect(
					struct snd_soc_component *component)
{
	struct snd_soc_dapm_context *dapm = snd_soc_component_get_dapm(component);

	snd_soc_dapm_disable_pin(dapm, "Mic Bias");
}

static irqreturn_t es8326_irq(int irq, void *data)
{
	struct es8326_priv *es8326 = data;
	struct snd_soc_component *comp = es8326->component;
	unsigned int flags;

	mutex_lock(&es8326->lock);

	/* Catch spurious IRQ before set_jack is called */
	if (!es8326->jack)
		goto out;

	es8326_enable_micbias_for_mic_gnd_short_detect(comp);

	regmap_read(es8326->regmap, ES8326_HP_DETECT, &flags);
	if (es8326->jd_inverted)
		flags ^= ES8326_HP_DETECT_NOT_INSERTED;

	dev_dbg(comp->dev, "flags: %#04x\n", flags);
	if (flags & ES8326_HP_DETECT_NOT_INSERTED) {
		snd_soc_jack_report(es8326->jack, 0,
				    SND_JACK_HEADSET | SND_JACK_BTN_0);
		dev_dbg(comp->dev, "jack unplugged\n");
	} else {
		if (flags & ES8326_HP_DETECT_GM_NOT_SHORTED) {
			dev_dbg(comp->dev, "headphones detected\n");
			snd_soc_jack_report(es8326->jack, SND_JACK_HEADPHONE,
					    SND_JACK_HEADSET);
		} else {
			dev_dbg(comp->dev, "headset detected\n");
			snd_soc_jack_report(es8326->jack, SND_JACK_HEADSET,
					    SND_JACK_HEADSET);
		}
	}

	if (!(es8326->jack->status & SND_JACK_MICROPHONE))
		es8326_disable_micbias_for_mic_gnd_short_detect(comp);
out:
	mutex_unlock(&es8326->lock);
	return IRQ_HANDLED;
}

static void es8326_enable_jack_detect(struct snd_soc_component *component,
				      struct snd_soc_jack *jack)
{
	struct es8326_priv *es8326 = snd_soc_component_get_drvdata(component);

	/*
	 * Init es8326->jd_inverted here and not in the probe, as we cannot
	 * guarantee that the sof-essx8336 driver, which might set this
	 * property, will probe before us.
	 */
	es8326->jd_inverted =
		device_property_read_bool(component->dev,
					  "everest,jack-detect-inverted");

	mutex_lock(&es8326->lock);

	es8326->jack = jack;

	snd_soc_component_write(component, ES8326_HPJACK_POL,
				ES8326_HPJACK_POL_HP_TYPE_CTIA |
				ES8326_HPJACK_POL_BUTTON_POL_ACTIVE_LOW |
				ES8326_HPJACK_POL_HPJACK_POL_ACTIVE_LOW |
				ES8326_HPJACK_POL_HPINSERT_SEL_PIN9);
	snd_soc_component_write(component, ES8326_INT_SOURCE,
				ES8326_INT_SOURCE_BUTTON |
				ES8326_INT_SOURCE_PIN9);

	mutex_unlock(&es8326->lock);

	/* Enable irq and sync initial jack state */
	enable_irq(es8326->irq);
	es8326_irq(es8326->irq, es8326);
}

static void es8326_disable_jack_detect(struct snd_soc_component *component)
{
	struct es8326_priv *es8326 = snd_soc_component_get_drvdata(component);

	disable_irq(es8326->irq);

	mutex_lock(&es8326->lock);

	snd_soc_component_write(component, ES8326_INT_SOURCE, 0);

	if (es8326->jack->status & SND_JACK_MICROPHONE)
		snd_soc_jack_report(es8326->jack, 0, SND_JACK_BTN_0);

	es8326->jack = NULL;

	mutex_unlock(&es8326->lock);
}

static int es8326_set_jack(struct snd_soc_component *component,
			   struct snd_soc_jack *jack, void *data)
{
	if (jack)
		es8326_enable_jack_detect(component, jack);
	else
		es8326_disable_jack_detect(component);

	return 0;
}

static int es8326_set_bias_level(struct snd_soc_component *component,
				 enum snd_soc_bias_level level)
{
	switch (level) {
	case SND_SOC_BIAS_ON:
		dev_dbg(component->dev, "%s on\n", __func__);
		snd_soc_component_write(component, ES8326_HP_DRIVER, 0);
		snd_soc_component_write(component, ES8326_ANA_PWR, 0);
		snd_soc_component_update_bits(component, ES8326_PGA_PWR,
					      ES8326_PGA_PWR_PDN_PGA |
					      ES8326_PGA_PWR_PDN_MOD |
					      ES8326_PGA_PWR_MODTOP_RST,
					      0);
		snd_soc_component_write(component, ES8326_VMIDSEL,
					ES8326_VMIDSEL_NORMAL);
		break;

	case SND_SOC_BIAS_PREPARE:
		dev_dbg(component->dev, "%s prepare\n", __func__);
		break;

	case SND_SOC_BIAS_STANDBY:
		dev_dbg(component->dev, "%s standby\n", __func__);
		snd_soc_component_update_bits(component, ES8326_HP_DRIVER,
					      ES8326_HP_DRIVER_LP_MASK,
					      ES8326_HP_DRIVER_LP_MASK);
		snd_soc_component_update_bits(component, ES8326_ANA_PWR,
					      ES8326_ANA_PWR_LP_DAC,
					      ES8326_ANA_PWR_LP_DAC);
		break;

	case SND_SOC_BIAS_OFF:
		dev_dbg(component->dev, "%s off\n", __func__);
		snd_soc_component_write(component, ES8326_VMIDSEL,
					ES8326_VMIDSEL_POWER_DOWN);
		snd_soc_component_update_bits(component, ES8326_PGA_PWR,
					      ES8326_PGA_PWR_PDN_PGA |
					      ES8326_PGA_PWR_PDN_MOD |
					      ES8326_PGA_PWR_MODTOP_RST,
					      ES8326_PGA_PWR_PDN_PGA |
					      ES8326_PGA_PWR_PDN_MOD |
					      ES8326_PGA_PWR_MODTOP_RST);
		snd_soc_component_write(component, ES8326_ANA_PWR,
					ES8326_ANA_PWR_PDN_DACR |
					ES8326_ANA_PWR_PDN_DACL |
					ES8326_ANA_PWR_LP_DAC |
					ES8326_ANA_PWR_PDN_VRP |
					ES8326_ANA_PWR_PDN_DACVREFGEN |
					ES8326_ANA_PWR_PDN_ADCVREFGEN |
					ES8326_ANA_PWR_PDN_IBIASGEN |
					ES8326_ANA_PWR_PDN_ANA);
		snd_soc_component_update_bits(component, ES8326_HP_DRIVER,
					      ES8326_HP_DRIVER_PDN_MASK |
					      ES8326_HP_DRIVER_ENREFR_HP,
					      ES8326_HP_DRIVER_PDN_MASK |
					      ES8326_HP_DRIVER_ENREFR_HP);
		break;

	default:
		break;
	}

	return 0;
}

static int es8326_probe(struct snd_soc_component *component)
{
	struct es8326_priv *es8326 = snd_soc_component_get_drvdata(component);
	int ret;

	es8326->component = component;

	es8326->mclk = devm_clk_get_optional(component->dev, "mclk");
	if (IS_ERR(es8326->mclk)) {
		dev_err(component->dev,"unable to get mclk\n");
		return PTR_ERR(es8326->mclk);
	}
	if (!es8326->mclk)
		dev_warn(component->dev,"assuming static mclk\n");

	ret = clk_prepare_enable(es8326->mclk);
	if (ret) {
		dev_err(component->dev,"unable to enable mclk\n");
		return ret;
	}

	snd_soc_component_init_regmap(component, es8326->regmap);

	/* Reset codec and enable current state machine */
	snd_soc_component_write(component, ES8326_RESET, ES8326_RESET_RST_MASK);
	usleep_range(5000, 5500);
	snd_soc_component_write(component, ES8326_RESET, ES8326_RESET_CSM_ON);
	msleep(30);

	snd_soc_component_write(component, ES8326_PULLUP_CTL,
				ES8326_PULLUP_CTL_ADC34_OFF);
	snd_soc_component_write(component, ES8326_CLK_CTL,
				ES8326_CLK_CTL_CLK9_ON |
				ES8326_CLK_CTL_CLK8_ON |
				ES8326_CLK_CTL_CLK3_ON |
				ES8326_CLK_CTL_CLK1_ON |
				ES8326_CLK_CTL_BCLK_ON |
				ES8326_CLK_CTL_MCLK_ON |
				ES8326_CLK_CTL_CPCLK_ON);
	snd_soc_component_write(component, ES8326_CLK_RESAMPLE,
				ES8326_CLK_RESAMPLE_OSC_EN_ALWAYS_ON |
				ES8326_CLK_RESAMPLE_INTCLK_SEL_OSC);
	snd_soc_component_write(component, ES8326_ADC_MUTE, 0);
	snd_soc_component_update_bits(component, ES8326_PGAGAIN,
				      ES8326_PGAGAIN_MIC1SEL,
				      ES8326_PGAGAIN_MIC1SEL);
	return 0;
}

static void es8326_remove(struct snd_soc_component *component)
{
	struct es8326_priv *es8326 = snd_soc_component_get_drvdata(component);

	snd_soc_component_exit_regmap(component);

	clk_disable_unprepare(es8326->mclk);
}

#ifdef CONFIG_PM_SLEEP
static int es8326_suspend(struct device *dev)
{
	struct es8326_priv *es8326 = dev_get_drvdata(dev);

	dev_dbg(dev, "%s: Enter\n", __func__);

	regcache_cache_only(es8326->regmap, true);
	regcache_mark_dirty(es8326->regmap);

	return 0;
}

static int es8326_resume(struct device *dev)
{
	struct es8326_priv *es8326 = dev_get_drvdata(dev);

	dev_dbg(dev, "%s: Enter\n", __func__);

	regcache_cache_only(es8326->regmap, false);
	regcache_sync(es8326->regmap);

	es8326_irq(es8326->irq, es8326);

	return 0;
}
#endif

static const struct dev_pm_ops es8326_pm = {
	SET_SYSTEM_SLEEP_PM_OPS(es8326_suspend, es8326_resume)
};

static struct snd_soc_component_driver soc_component_dev_es8326 = {
	.probe			= es8326_probe,
	.remove			= es8326_remove,
	.set_bias_level		= es8326_set_bias_level,
	.set_jack		= es8326_set_jack,
	.controls		= es8326_snd_controls,
	.num_controls		= ARRAY_SIZE(es8326_snd_controls),
	.dapm_widgets		= es8326_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(es8326_dapm_widgets),
	.dapm_routes		= es8326_dapm_routes,
	.num_dapm_routes	= ARRAY_SIZE(es8326_dapm_routes),
	.use_pmdown_time	= 1,
	.endianness		= 1,
	.non_legacy_dai_naming	= 1,
};

static const struct regmap_range es8326_volatile_ranges[] = {
	regmap_reg_range(ES8326_HP_DETECT, ES8326_HP_DETECT),
};

static const struct regmap_access_table es8326_volatile_table = {
	.yes_ranges	= es8326_volatile_ranges,
	.n_yes_ranges	= ARRAY_SIZE(es8326_volatile_ranges),
};

static const struct regmap_config es8326_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xff,
	.volatile_table	= &es8326_volatile_table,
	.cache_type = REGCACHE_RBTREE,
	.use_single_read = true,
	.use_single_write = true,
};

static int es8326_i2c_probe(struct i2c_client *i2c_client,
			    const struct i2c_device_id *id)
{
	struct device *dev = &i2c_client->dev;
	struct es8326_priv *es8326;
	int ret;

	es8326 = devm_kzalloc(dev, sizeof(*es8326), GFP_KERNEL);
	if (es8326 == NULL)
		return -ENOMEM;

	i2c_set_clientdata(i2c_client, es8326);

	es8326->regmap = devm_regmap_init_i2c(i2c_client, &es8326_regmap);
	if (IS_ERR(es8326->regmap))
		return PTR_ERR(es8326->regmap);

	es8326->irq = i2c_client->irq;
	mutex_init(&es8326->lock);

	ret = devm_request_threaded_irq(dev, es8326->irq, NULL, es8326_irq,
					IRQF_TRIGGER_HIGH | IRQF_ONESHOT,
					"es8326", es8326);
	if (ret == 0) {
		/* Gets re-enabled by es8326_set_jack() */
		disable_irq(es8326->irq);
	} else {
		dev_warn(dev, "Failed to get IRQ %d: %d\n", es8326->irq, ret);
		es8326->irq = -ENXIO;
	}

	return devm_snd_soc_register_component(dev, &soc_component_dev_es8326,
					       &es8326_dai, 1);
}

static const struct i2c_device_id es8326_i2c_id[] = {
	{"es8326", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, es8326_i2c_id);

#ifdef CONFIG_OF
static const struct of_device_id es8326_of_match[] = {
	{ .compatible = "everest,es8326", },
	{}
};
MODULE_DEVICE_TABLE(of, es8326_of_match);
#endif

#ifdef CONFIG_ACPI
static const struct acpi_device_id es8326_acpi_match[] = {
	{"ESSX8326", 0},
	{},
};
MODULE_DEVICE_TABLE(acpi, es8326_acpi_match);
#endif

static struct i2c_driver es8326_i2c_driver = {
	.driver = {
		.name			= "es8326",
		.acpi_match_table	= ACPI_PTR(es8326_acpi_match),
		.of_match_table		= of_match_ptr(es8326_of_match),
		.pm			= &es8326_pm,
	},
	.probe		= es8326_i2c_probe,
	.id_table	= es8326_i2c_id,
};
module_i2c_driver(es8326_i2c_driver);

MODULE_DESCRIPTION("Everest Semi ES8326 ALSA SoC Codec Driver");
MODULE_AUTHOR("David <zhuning@everset-semi.com>");
MODULE_LICENSE("GPL v2");
