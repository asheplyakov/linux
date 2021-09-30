// SPDX-License-Identifier: GPL-2.0-only
/*
 * soc-apci-intel-cfl-match.c - tables and support for CFL ACPI enumeration.
 *
 * Copyright (c) 2019, Intel Corporation.
 *
 */

#include <sound/soc-acpi.h>
#include <sound/soc-acpi-intel-match.h>
#include "../skylake/skl.h"

#ifdef CONFIG_SND_SOC_ES8336
static struct snd_soc_acpi_codecs bxt_codecs =
{
	.num_codecs = 1,
	.codecs = {"ESSX8336"}
};
#endif

struct snd_soc_acpi_mach snd_soc_acpi_intel_cfl_machines[] = {
#ifdef CONFIG_SND_SOC_ES8336
	{
		.id = "ESSX8336",
		.drv_name = "sof-essx8336",
		.fw_filename = "intel/dsp_fw_cnl.bin",
		.machine_quirk = snd_soc_acpi_codec_list,
		.quirk_data = &bxt_codecs,
		.sof_fw_filename = "sof-cfl.ri",
		.sof_tplg_filename = "sof-glk-es8336-ssp0.tplg",
	},
#endif
	{},
};
EXPORT_SYMBOL_GPL(snd_soc_acpi_intel_cfl_machines);

struct snd_soc_acpi_mach snd_soc_acpi_intel_cfl_sdw_machines[] = {
	{}
};
EXPORT_SYMBOL_GPL(snd_soc_acpi_intel_cfl_sdw_machines);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Intel Common ACPI Match module");
