// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for Synopsys DesignWare Cores Mobile Storage Host Controller
 *
 * Copyright (C) 2018 Synaptics Incorporated
 *
 * Author: Jisheng Zhang <jszhang@kernel.org>
 */

#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/sizes.h>
#include <linux/delay.h>

#include "sdhci-pltfm.h"

#define BOUNDARY_OK(addr, len) \
	((addr | (SZ_128M - 1)) == ((addr + len - 1) | (SZ_128M - 1)))

struct dwcmshc_priv {
	u32 min_clock;
	u32 max_clock;
};

/*
 * If DMA addr spans 128MB boundary, we split the DMA transfer into two
 * so that each DMA transfer doesn't exceed the boundary.
 */
static void dwcmshc_adma_write_desc(struct sdhci_host *host, void **desc,
				    dma_addr_t addr, int len, unsigned int cmd)
{

	int tmplen, offset;

	if (likely(!len || BOUNDARY_OK(addr, len))) {
		sdhci_adma_write_desc(host, desc, addr, len, cmd);

		return;
	}

	offset = addr & (SZ_128M - 1);
	tmplen = SZ_128M - offset;
	sdhci_adma_write_desc(host, desc, addr, tmplen, cmd);

	addr += tmplen;
	len -= tmplen;
	sdhci_adma_write_desc(host, desc, addr, len, cmd);
}

static unsigned int dwcmshc_get_max_clock (struct sdhci_host *host)
{
	struct sdhci_pltfm_host *pltfm_host;
	struct dwcmshc_priv *priv;

	pltfm_host = sdhci_priv(host);
	priv = sdhci_pltfm_priv(pltfm_host);
	return priv->max_clock;
}

static unsigned int dwcmshc_get_min_clock (struct sdhci_host *host)
{
	struct sdhci_pltfm_host *pltfm_host;
	struct dwcmshc_priv *priv;

	pltfm_host = sdhci_priv(host);
	priv = sdhci_pltfm_priv(pltfm_host);
	return priv->min_clock;
}

static void dwcmshc_set_clock(struct sdhci_host *host, unsigned int clock)
{
	u16 clk;
	unsigned long timeout;

	/* clk = 0 */
	host->mmc->actual_clock = 0;
	sdhci_writew(host, 0, SDHCI_CLOCK_CONTROL);
	if (clock == 0){
		return;
	}
	
	/* clk = clock */
	struct sdhci_pltfm_host *pltfm_host;
	pltfm_host = sdhci_priv(host);
	clk_set_rate(pltfm_host->clk,clock);

	clk = sdhci_calc_clk(host, clock, &host->mmc->actual_clock);
	clk |= SDHCI_CLOCK_INT_EN;
	sdhci_writew(host, clk, SDHCI_CLOCK_CONTROL);

	/* Wait max 20 ms */
	timeout = 20;
	while (!((clk = sdhci_readw(host, SDHCI_CLOCK_CONTROL)) & SDHCI_CLOCK_INT_STABLE)) {
		if (timeout == 0) {
			pr_err("%s: Internal clock never stabilised.\n", mmc_hostname(host->mmc));
			sdhci_dumpregs(host);
			return;
		}
		timeout--;
		spin_unlock_irq(&host->lock);
		usleep_range(900, 1100);
		spin_lock_irq(&host->lock);
	}
	clk |= SDHCI_CLOCK_CARD_EN;
	sdhci_writew(host, clk, SDHCI_CLOCK_CONTROL);
}

static const struct sdhci_ops sdhci_dwcmshc_ops = {
	.reset			= sdhci_reset,
	.set_clock		= dwcmshc_set_clock,
	.set_bus_width		= sdhci_set_bus_width,
	.set_uhs_signaling	= sdhci_set_uhs_signaling,
	.get_max_clock		= dwcmshc_get_max_clock,
	.get_min_clock		= dwcmshc_get_min_clock,
	.adma_write_desc	= dwcmshc_adma_write_desc,
};

static const struct sdhci_pltfm_data sdhci_dwcmshc_pdata = {
	.ops = &sdhci_dwcmshc_ops,

	/* Deviations from spec. */
	.quirks = SDHCI_QUIRK_CAP_CLOCK_BASE_BROKEN | SDHCI_QUIRK_BROKEN_ADMA,
	// .quirks2 = ,
};

static int dwcmshc_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct sdhci_pltfm_host *pltfm_host;
	struct sdhci_host *host;
	struct dwcmshc_priv *priv;
	int err;
	u32 extra;

	host = sdhci_pltfm_init(pdev, &sdhci_dwcmshc_pdata,
				sizeof(struct dwcmshc_priv));
	if (IS_ERR(host)){
		return PTR_ERR(host);
	}

	/*
	 * extra adma table cnt for cross 128M boundary handling.
	 */
	extra = DIV_ROUND_UP_ULL(dma_get_required_mask(&pdev->dev), SZ_128M);
	if (extra > SDHCI_MAX_SEGS)
		extra = SDHCI_MAX_SEGS;
	host->adma_table_cnt += extra;

	pltfm_host = sdhci_priv(host);
	priv = sdhci_pltfm_priv(pltfm_host);

	pltfm_host->clk = devm_clk_get(&pdev->dev, "core");
	if (IS_ERR(pltfm_host->clk)) {
		err = PTR_ERR(pltfm_host->clk);
		dev_err(&pdev->dev, "failed to get 'core' clk: %d\n", err);
		goto free_pltfm;
	}
	err = clk_prepare_enable(pltfm_host->clk);
	if (err){
		dev_err(&pdev->dev, "failed to prepare clk: %d\n", err);
		goto free_pltfm;
	}

	err = mmc_of_parse(host->mmc);
	if (err){
		dev_err(&pdev->dev, "failed to mmc_of_parse: %d\n", err);
		goto err_clk;
	}

	sdhci_get_of_property(pdev);

	if(!of_property_read_u32(np, "max-clock", &priv->max_clock))
		priv->max_clock = 50*1000*1000;

	if(!of_property_read_u32(np, "min-clock", &priv->min_clock))
		priv->min_clock = 100*1000;


	err = sdhci_add_host(host);
	if (err){
		dev_err(&pdev->dev, "failed to sdhci_add_host: %d\n", err);
		goto err_clk;
	}


	return 0;

err_clk:
	clk_disable_unprepare(pltfm_host->clk);
free_pltfm:
	sdhci_pltfm_free(pdev);

	return err;
}

static int dwcmshc_remove(struct platform_device *pdev)
{
	struct sdhci_host *host = platform_get_drvdata(pdev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);

	sdhci_remove_host(host, 0);

	clk_disable_unprepare(pltfm_host->clk);

	sdhci_pltfm_free(pdev);

	return 0;
}

static const struct of_device_id sdhci_dwcmshc_dt_ids[] = {
	{ .compatible = "snps,dwcmshc-sdhci" },
	{}
};
MODULE_DEVICE_TABLE(of, sdhci_dwcmshc_dt_ids);

static struct platform_driver sdhci_dwcmshc_driver = {
	.driver	= {
		.name	= "sdhci-dwcmshc",
		.of_match_table = sdhci_dwcmshc_dt_ids,
	},
	.probe	= dwcmshc_probe,
	.remove	= dwcmshc_remove,
};
module_platform_driver(sdhci_dwcmshc_driver);

MODULE_DESCRIPTION("SDHCI platform driver for Synopsys DWC MSHC");
MODULE_AUTHOR("Jisheng Zhang <jszhang@kernel.org>");
MODULE_LICENSE("GPL v2");
