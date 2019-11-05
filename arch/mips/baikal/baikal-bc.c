/*
 * Baikal-T SOC platform support code. Boot Controller driver.
 *
 * Copyright (C) 2017 T-platforms JSC
 *
 * Author:
 *   Sergey Semin <Sergey.Semin@t-platforms.ru>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/device.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/delay.h>

#include <asm/mach-baikal/bc.h>

#define DRIVER_NAME		"be-bc"
#define VERSION			"1.02"

#define BE_BC_CSR		0x00
#define BE_BC_MAR		0x04
#define BE_BC_DRID		0x08
#define BE_BC_VID		0x0C
#define BE_BC_CSR_BMODE		(0x3 << 0)
#define BE_BC_CSR_SPI_RDA	(0x1 << 8)
#define BE_BC_CSR_SPI_MDELAY	1

static const char *be_bc_mode_str(unsigned int mode)
{
	switch (mode) {
	case BE_BC_ROM_MODE:
		return "ROM";
	case BE_BC_FLASH_MODE:
		return "Flash";
	default:
		return "Unknown";
	}
}

#ifdef CONFIG_OF
static int of_dev_node_match(struct device *dev, void *data)
{
	return dev->of_node == data;
}

struct be_bc *of_find_be_bc_device_by_node(struct device_node *node)
{
	struct device *dev;
	struct be_bc *bc;

	dev = bus_find_device(&platform_bus_type, NULL, node, of_dev_node_match);
	if (!dev)
		return NULL;

	bc = platform_get_drvdata(to_platform_device(dev));
	put_device(dev);

	return bc;
}
EXPORT_SYMBOL_GPL(of_find_be_bc_device_by_node);
#endif

void be_bc_enable_spi(struct be_bc *bc)
{
	writel(BE_BC_CSR_SPI_RDA, bc->regs + BE_BC_CSR);
	msleep(BE_BC_CSR_SPI_MDELAY);
}
EXPORT_SYMBOL_GPL(be_bc_enable_spi);

void be_bc_disable_spi(struct be_bc *bc)
{
	writel(~BE_BC_CSR_SPI_RDA, bc->regs + BE_BC_CSR);
	msleep(BE_BC_CSR_SPI_MDELAY);
}
EXPORT_SYMBOL_GPL(be_bc_disable_spi);

static int be_bc_drv_probe(struct platform_device *pdev)
{
	struct resource *res;
	unsigned int mode;
	struct be_bc *bc;
	u32 vid, drid;

	bc = devm_kzalloc(&pdev->dev, sizeof(*bc), GFP_KERNEL);
	if (!bc)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	bc->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(bc->regs))
		return PTR_ERR(bc->regs);

	platform_set_drvdata(pdev, bc);

	mode = readl(bc->regs + BE_BC_CSR) & BE_BC_CSR_BMODE;
	drid = readl(bc->regs + BE_BC_DRID);
	vid = readl(bc->regs + BE_BC_VID);

	dev_info(&pdev->dev, "Baikal Electronics Boot Controller Driver\n");
	dev_info(&pdev->dev, "VID: 0x%08x, DRID: 0x%08x\n", vid, drid);
	dev_info(&pdev->dev, "Boot Mode: %s\n", be_bc_mode_str(mode));

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id be_bc_of_match[] = {
	{ .compatible = "be,bc", },
	{ /* end of table */}
};
MODULE_DEVICE_TABLE(of, be_bc_of_match);
#endif

static struct platform_driver be_bc_driver = {
	.probe		= be_bc_drv_probe,
	.driver		= {
		.name	= DRIVER_NAME,
		.of_match_table = of_match_ptr(be_bc_of_match),
	},
};
module_platform_driver(be_bc_driver);

MODULE_VERSION(VERSION);
MODULE_AUTHOR("Sergey Semin <Sergey.Semin@t-platforms.ru>");
MODULE_DESCRIPTION("Baikal Electronics Boot Controller Driver");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_ALIAS("platform:be_bc");
