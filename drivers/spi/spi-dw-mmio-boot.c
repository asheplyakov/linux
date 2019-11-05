/*
 * Memory-mapped interface driver for DW SPI Core
 *
 * Copyright (c) 2010, Octasic semiconductor.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <asm/mach-baikal/bc.h>

#include "spi-dw.h"

#define DRIVER_NAME "dw_spi_mmio_boot"

struct dw_spi_mmio {
	struct dw_spi  dws;
	struct be_bc   *bc;
	struct clk     *clk;
};

static int dw_spi_mmio_probe(struct platform_device *pdev)
{
	struct dw_spi_mmio *dwsmmio;
	struct device_node *bc_np;
	struct dw_spi *dws;
	struct resource *mem;
	int ret, i;
	int num_cs;

	if (!pdev->dev.of_node) {
		dev_err(&pdev->dev, "of node is unavailable?\n");
		return -EINVAL;
	}

	dwsmmio = devm_kzalloc(&pdev->dev, sizeof(struct dw_spi_mmio),
			GFP_KERNEL);
	if (!dwsmmio)
		return -ENOMEM;

	dws = &dwsmmio->dws;

	/* Get basic io resource and map it */
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem) {
		dev_err(&pdev->dev, "no mem resource?\n");
		return -EINVAL;
	}

	dws->regs = devm_ioremap_resource(&pdev->dev, mem);
	if (IS_ERR(dws->regs)) {
		dev_err(&pdev->dev, "SPI region map failed\n");
		return PTR_ERR(dws->regs);
	}

	/* No IRQ available for this controller */
	dws->irq = -1;

	dwsmmio->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(dwsmmio->clk))
		return PTR_ERR(dwsmmio->clk);
	ret = clk_prepare_enable(dwsmmio->clk);
	if (ret)
		return ret;

	dws->bus_num = of_alias_get_id(pdev->dev.of_node, "ssi");
	if (dws->bus_num < 0)
		dws->bus_num = 0;

	dws->max_freq = clk_get_rate(dwsmmio->clk);

	device_property_read_u32(&pdev->dev, "reg-io-width", &dws->reg_io_width);

	num_cs = 1;

	device_property_read_u32(&pdev->dev, "num-cs", &num_cs);

	dws->num_cs = num_cs;

	for (i = 0; i < dws->num_cs; i++) {
		int cs_gpio = of_get_named_gpio(pdev->dev.of_node,
				"cs-gpios", i);

		if (cs_gpio == -EPROBE_DEFER) {
			ret = cs_gpio;
			goto out;
		}

		if (gpio_is_valid(cs_gpio)) {
			ret = devm_gpio_request(&pdev->dev, cs_gpio,
					dev_name(&pdev->dev));
			if (ret)
				goto out;
		}
	}

	/* Enable the DW SPI controller interface */
	bc_np = of_parse_phandle(pdev->dev.of_node, "boot-controller", 0);
	if (!bc_np) {
		ret = -EINVAL;
		dev_err(&pdev->dev, "boot-controller of-node missing\n");
		goto out;
	}
	dwsmmio->bc = of_find_be_bc_device_by_node(bc_np);
	if (!dwsmmio->bc) {
		ret = -EINVAL;
		dev_err(&pdev->dev, "BE BC search failed\n");
		goto out;
	}
	be_bc_enable_spi(dwsmmio->bc);

	pdev->dev.dma_mask = NULL;
	ret = dw_spi_add_host(&pdev->dev, dws);
	if (ret)
		goto dis_be_bc;

	/* Dump DW component type */
	ret = readl(dws->regs + DW_SPI_VERSION);
	dev_info(&pdev->dev, "DW SPI ID: 0x%08X, Version: %c.%c%c%c\n",
		readl(dws->regs + DW_SPI_IDR), (ret >> 24) & 0xff,
		(ret >> 16) & 0xff, (ret >> 8) & 0xff, ret & 0xff);

	platform_set_drvdata(pdev, dwsmmio);
	return 0;

dis_be_bc:
	be_bc_disable_spi(dwsmmio->bc);

out:
	clk_disable_unprepare(dwsmmio->clk);
	return ret;
}

static int dw_spi_mmio_remove(struct platform_device *pdev)
{
	struct dw_spi_mmio *dwsmmio = platform_get_drvdata(pdev);

	dw_spi_remove_host(&dwsmmio->dws);

	be_bc_disable_spi(dwsmmio->bc);

	clk_disable_unprepare(dwsmmio->clk);

	return 0;
}

static const struct of_device_id dw_spi_mmio_of_match[] = {
	{ .compatible = "snps,dw-apb-ssi-boot", },
	{ /* end of table */}
};
MODULE_DEVICE_TABLE(of, dw_spi_mmio_of_match);

static struct platform_driver dw_spi_mmio_driver = {
	.probe		= dw_spi_mmio_probe,
	.remove		= dw_spi_mmio_remove,
	.driver		= {
		.name	= DRIVER_NAME,
		.of_match_table = dw_spi_mmio_of_match,
	},
};
module_platform_driver(dw_spi_mmio_driver);

MODULE_AUTHOR("Sergey Semin <Sergey.Semin@t-platforms.ru>>");
MODULE_DESCRIPTION("Memory-mapped I/O interface driver for DW SPI-boot Core");
MODULE_LICENSE("GPL v2");
