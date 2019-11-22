/*
 * PCIe RC driver for Synopsys DesignWare Core
 *
 * Copyright (C) 2015-2016 Synopsys, Inc. (www.synopsys.com)
 *
 * Authors: Joao Pinto <Joao.Pinto@synopsys.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/of_gpio.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/resource.h>
#include <linux/signal.h>
#include <linux/types.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>

#include "pcie-designware.h"

/* PCIe Port Logic registers */
#define PLR_OFFSET			0x700
#define PCIE_PHY_DEBUG_R1		(PLR_OFFSET + 0x2c)
#define PCIE_PHY_DEBUG_R1_LINK_UP	(0x1 << 4)
#define PCIE_PHY_DEBUG_R1_LINK_IN_TRAINING	(0x1 << 29)

struct dw_plat_pcie {
	struct pcie_port	pp;	/* pp.dbi_base is DT 0th resource */
};

static int dw_baikal_pcie_link_up(struct pcie_port *pp)
{
	uint32_t val;

	val = readl(pp->dbi_base + PCIE_PHY_DEBUG_R1);
	return ((val & PCIE_PHY_DEBUG_R1_LINK_UP) &&
		(!(val & PCIE_PHY_DEBUG_R1_LINK_IN_TRAINING)));
}

static irqreturn_t dw_plat_pcie_msi_irq_handler(int irq, void *arg)
{
	struct pcie_port *pp = arg;

	return dw_handle_msi_irq(pp);
}

static void dw_plat_pcie_host_init(struct pcie_port *pp)
{
	if (IS_ENABLED(CONFIG_PCI_MSI))
		dw_pcie_msi_init(pp);
}

static struct pcie_host_ops dw_plat_pcie_host_ops = {
	.host_init = dw_plat_pcie_host_init,
	.link_up = dw_baikal_pcie_link_up,
};

static int dw_plat_add_pcie_port(struct pcie_port *pp,
				 struct platform_device *pdev)
{
	struct device *dev = pp->dev;
	int ret;

	pp->irq = platform_get_irq(pdev, 1);
	if (pp->irq < 0)
		return pp->irq;

	if (IS_ENABLED(CONFIG_PCI_MSI)) {
		pp->msi_irq = platform_get_irq(pdev, 0);
		if (pp->msi_irq < 0)
			return pp->msi_irq;

		ret = devm_request_irq(dev, pp->msi_irq,
					dw_plat_pcie_msi_irq_handler,
					IRQF_SHARED | IRQF_NO_THREAD,
					"dw-plat-pcie-msi", pp);
		if (ret) {
			dev_err(dev, "failed to request MSI IRQ\n");
			return ret;
		}
	}

	pp->root_bus_nr = -1;
	pp->ops = &dw_plat_pcie_host_ops;

	ret = dw_pcie_host_init(pp);
	if (ret) {
		dev_err(dev, "failed to initialize host\n");
		return ret;
	}

	return 0;
}

static int dw_plat_pcie_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct dw_plat_pcie *dw_plat_pcie;
	struct gpio_desc *reset_gpiod;
	struct pcie_port *pp;
	struct resource *res;  /* Resource from DT */

	/* Deassert the optional reset signal */
	if (dev->of_node) {
		reset_gpiod = fwnode_get_named_gpiod(&dev->of_node->fwnode,
			"reset-gpio", 0, GPIOD_OUT_LOW, "PCIe reset");
		if (PTR_ERR(reset_gpiod) == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		else if (!IS_ERR(reset_gpiod))
			dev_info(dev, "Reset GPIO deasserted\n");
	}

	dw_plat_pcie = devm_kzalloc(dev, sizeof(*dw_plat_pcie), GFP_KERNEL);
	if (!dw_plat_pcie)
		return -ENOMEM;

	pp = &dw_plat_pcie->pp;
	pp->dev = dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	pp->dbi_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(pp->dbi_base))
		return PTR_ERR(pp->dbi_base);

	return dw_plat_add_pcie_port(pp, pdev);
}

static const struct of_device_id dw_plat_pcie_of_match[] = {
	{ .compatible = "snps,dw-pcie", },
	{},
};

static struct platform_driver dw_plat_pcie_driver = {
	.driver = {
		.name	= "dw-pcie",
		.of_match_table = dw_plat_pcie_of_match,
	},
	.probe = dw_plat_pcie_probe,
};
builtin_platform_driver(dw_plat_pcie_driver);
