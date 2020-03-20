/*
 * pcie-baikal - PCIe controller driver for Baikal SoCs
 *
 * Copyright (C) 2019 Baikal Electronics JSC
 * Author: Pavel Parkhomenko <Pavel.Parkhomenko@baikalelectronics.ru>
 *
 * Parts of this file were based on sources as follows:
 * Copyright (C) 2013-2014 Texas Instruments Incorporated - http://www.ti.com
 * Author: Kishon Vijay Abraham I <kishon@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define DEBUG 1

#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqchip/arm-gic-v3.h>
#include <linux/irqdomain.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/baikal/lcru-pcie.h>
#include <linux/msi.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/pci.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/resource.h>
#include <linux/types.h>
#include <linux/moduleparam.h>

#include "pcie-designware.h"
#include "pcie-baikal.h"


#define PCIE_MSI_ADDR_LO        0x820
#define PCIE_MSI_ADDR_HI        0x824


struct baikal_pcie {
	struct dw_pcie	*pci;
	int		bus_nr;
	struct regmap	*lcru;
	struct gpio_desc *reset_gpio;
	char reset_name[32];
};

#define to_baikal_pcie(x)	dev_get_drvdata((x)->dev)
#define LINK_RETRAIN_TIMEOUT HZ

#define PORT_LINK_FAST_LINK_MODE				(1 << 7)	/* Fast Link Mode. */

#define PCIE_PHY_RETRIES	1000000
#define PHY_ALL_LANES		0xF
#define PHY_LANE0			0x1

/* Baikal-specific registers. */
#define PCIE_BK_MGMT_SEL_LANE		(0xd04) /* Select lane. */
#define PCIE_BK_MGMT_CTRL			(0xd08) /* Control management register. */
#define PCIE_BK_MGMT_WRITE_DATA		(0xd0c) /* Data write register. */
#define PCIE_BK_MGMT_READ_DATA		(0xd10) /* Data read register. */

#define PCIE_MISC_CONTROL_1_OFF		(0x8bc) /* to open RO DBI register. */
#define DBI_RO_RW_EN			(1 << 0)

#define PCIE_COHERENCE_CONTROL_3_OFF		(0x8e8) /* to set cache coherence register. */

/* PCIE_BK_MGMT_CTRL */
#define BK_MGMT_CTRL_ADDR_MASK		(0xFFFFF) /* bits [20:0] */
#define BK_MGMT_CTRL_READ			(0 << 29)
#define BK_MGMT_CTRL_WRITE			(1 << 29)
#define BK_MGMT_CTRL_DONE			(1 << 30)
#define BK_MGMT_CTRL_BUSY			(1 << 31)

#define PCIE_LINK_CAPABILITIES_REG              (0x7c)  /* Link Capabilities Register. */
#define PCIE_ROOT_CONTROL_ROOT_CAPABILITIES_REG	(0x8c)	/* Root Control and Capabilities Register. */

#define PCIE_LINK_CONTROL_LINK_STATUS_REG	(0x80)	/* Link Control and Status Register. */
/* LINK_CONTROL_LINK_STATUS_REG */
#define PCIE_CAP_LINK_SPEED_SHIFT		16
#define PCIE_CAP_LINK_SPEED_MASK		0xF0000
#define PCIE_CAP_LINK_SPEED_GEN1		0x1
#define PCIE_CAP_LINK_SPEED_GEN2		0x2
#define PCIE_CAP_LINK_SPEED_GEN3		0x3
#define PCIE_STA_LINK_TRAINING			0x8000000
#define PCIE_STA_LINK_WIDTH_MASK		0x3f00000
#define PCIE_STA_LINK_WIDTH_SHIFT		(20)

#define PCIE_LINK_CONTROL2_LINK_STATUS2_REG     (0xa0)  /* Link Control 2 and Status 2 Register. */
/* PCIE_LINK_CONTROL2_LINK_STATUS2 */
#define PCIE_LINK_CONTROL2_GEN_MASK             (0xF)
#define PCIE_LINK_CONTROL2_GEN1                 (1)
#define PCIE_LINK_CONTROL2_GEN2                 (2)
#define PCIE_LINK_CONTROL2_GEN3                 (3)

static inline int dw_pcie_link_is_training(struct dw_pcie *pci)
{
	int reg = dw_pcie_readl_dbi(pci, PCIE_LINK_CONTROL_LINK_STATUS_REG);
	return reg & PCIE_STA_LINK_TRAINING;
}

static void dw_wait_pcie_link_training_done(struct dw_pcie *pci)
{
	unsigned long start_jiffies = jiffies;
	while (dw_pcie_link_is_training(pci)) {
		if (time_after(jiffies, start_jiffies + LINK_RETRAIN_TIMEOUT)) {
			pr_err("%s: link retrained for too long, timeout occured\n", __func__);
			break;
		}
		udelay(100);
	}
}

static int dw_report_link_performance(struct dw_pcie *pci)
{
	int reg = dw_pcie_readl_dbi(pci, PCIE_LINK_CONTROL_LINK_STATUS_REG);
	int speed = (reg & PCIE_CAP_LINK_SPEED_MASK) >> PCIE_CAP_LINK_SPEED_SHIFT;
	int width = (reg & PCIE_STA_LINK_WIDTH_MASK) >> PCIE_STA_LINK_WIDTH_SHIFT; 
	dev_info(pci->dev, "Link Status is     GEN%d, x%d\n", speed, width);
	return speed;
}

static inline void dw_pcie_link_retrain(struct dw_pcie *pci, int target_speed)
{
	int reg;
	unsigned long start_jiffies;

	// In case link is already training wait for training to complete
	dw_wait_pcie_link_training_done(pci);

	// Set desired speed
	reg = dw_pcie_readl_dbi(pci, PCIE_LINK_CONTROL2_LINK_STATUS2_REG);
	reg &= ~PCIE_LINK_CONTROL2_GEN_MASK;
	reg |= target_speed;
	dw_pcie_writel_dbi(pci, PCIE_LINK_CONTROL2_LINK_STATUS2_REG, reg);

	// Set Retrain Link bit
	reg = dw_pcie_readl_dbi(pci, PCIE_LINK_CONTROL_LINK_STATUS_REG);
	reg |= PCI_EXP_LNKCTL_RL;
	dw_pcie_writel_dbi(pci, PCIE_LINK_CONTROL_LINK_STATUS_REG, reg);

	/* Wait for link training begin */
	start_jiffies = jiffies;
	while (!dw_pcie_link_is_training(pci)) {
		if (time_after(jiffies, start_jiffies + LINK_RETRAIN_TIMEOUT)) {
			pr_err("%s: link retrained for too long, timeout occured\n", __func__);
			break;
		}
		udelay(100);
	}

	/* Wait for link training end */
	dw_wait_pcie_link_training_done(pci);

	if(dw_pcie_wait_for_link(pci) == 0) {
		dw_report_link_performance(pci);
	}
}


static void pcie_link_speed_fixup(struct pci_dev *pdev)
{
	int reg, speed, width, target_speed;
	struct pcie_port *pp = pdev->bus->sysdata;
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);

	reg = dw_pcie_readl_dbi(pci, PCIE_LINK_CAPABILITIES_REG);
	speed = reg & PCI_EXP_LNKCAP_SLS;
	if (speed > PCI_EXP_LNKCAP_SLS_2_5GB) {
		pcie_capability_read_dword(pdev, PCI_EXP_LNKCAP, &reg);
		speed = reg & PCI_EXP_LNKCAP_SLS;
		width = (reg & PCI_EXP_LNKCAP_MLW) >> PCI_EXP_LNKSTA_NLW_SHIFT;
		dev_info(&pdev->dev, "Link Capability is GEN%d, x%d\n", speed, width);
		if (speed > PCI_EXP_LNKCAP_SLS_2_5GB) {
			target_speed = speed;
			if (dw_report_link_performance(pci) < target_speed) {
				dev_info(&pdev->dev, "retrain link to GEN%d\n", target_speed);
				dw_pcie_link_retrain(pci, target_speed);
				dw_report_link_performance(pci);
				return;
			}
		}
	}
}
DECLARE_PCI_FIXUP_FINAL(PCI_ANY_ID, PCI_ANY_ID, pcie_link_speed_fixup);

static int baikal_pcie_link_up(struct dw_pcie *pci)
{
	struct baikal_pcie *rc = to_baikal_pcie(pci);
	uint32_t misc_reg;
	uint32_t class_reg;
	u32	lcru_reg, reg;

	// Set class
	lcru_reg = baikal_pcie_lcru_readl(rc->lcru, BAIKAL_LCRU_PCIE_GEN_CTL(rc->bus_nr));
	baikal_pcie_lcru_writel(rc->lcru, BAIKAL_LCRU_PCIE_GEN_CTL(rc->bus_nr), lcru_reg & (~BAIKAL_PCIE_DBI2_MODE));

	misc_reg = dw_pcie_readl_dbi(pci, PCIE_MISC_CONTROL_1_OFF);
	misc_reg |= DBI_RO_RW_EN;

	dw_pcie_writel_dbi(pci, PCIE_MISC_CONTROL_1_OFF, misc_reg);
	class_reg = dw_pcie_readl_dbi(pci, PCI_CLASS_REVISION);

	class_reg = (0x604 << 16) | (class_reg & 0xff);			// class PCI_PCI_BRIDGE=0x604
	dw_pcie_writel_dbi(pci, PCI_CLASS_REVISION, class_reg);

	misc_reg &= ~DBI_RO_RW_EN;
	dw_pcie_writel_dbi(pci, PCIE_MISC_CONTROL_1_OFF, misc_reg);
	baikal_pcie_lcru_writel(rc->lcru, BAIKAL_LCRU_PCIE_GEN_CTL(rc->bus_nr), lcru_reg);
	reg = baikal_pcie_lcru_readl(rc->lcru, BAIKAL_LCRU_PCIE_STATUS(rc->bus_nr));
	return !!(reg & (BAIKAL_PCIE_RDLH_LINKUP | BAIKAL_PCIE_SMLH_LINKUP));
}

static void baikal_pcie_cease_link(struct baikal_pcie *rc)
{
	u32 reg;
	reg = baikal_pcie_lcru_readl(rc->lcru, BAIKAL_LCRU_PCIE_GEN_CTL(rc->bus_nr));
	reg &= ~BAIKAL_PCIE_LTSSM_ENABLE;
	baikal_pcie_lcru_writel(rc->lcru, BAIKAL_LCRU_PCIE_GEN_CTL(rc->bus_nr), reg);
}

static int baikal_pcie_establish_link(struct baikal_pcie *rc)
{
	struct dw_pcie *pci = rc->pci;
	struct device *dev = pci->dev;
	u32 reg;
	int ok;

	if (baikal_pcie_link_up(pci)) {
		dev_err(dev, "link is already up\n");
		return 0;
	}

	reg = baikal_pcie_lcru_readl(rc->lcru, BAIKAL_LCRU_PCIE_GEN_CTL(rc->bus_nr));
	reg |= BAIKAL_PCIE_LTSSM_ENABLE;
	baikal_pcie_lcru_writel(rc->lcru, BAIKAL_LCRU_PCIE_GEN_CTL(rc->bus_nr), reg);

	ok = dw_pcie_wait_for_link(pci);
	if(ok == 0) {
		dw_report_link_performance(pci);
	}
	return ok;
}

static int baikal_pcie_host_init(struct pcie_port *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct baikal_pcie *baikal_pcie = to_baikal_pcie(pci);

	dw_pcie_setup_rc(pp);
	baikal_pcie_establish_link(baikal_pcie);
	return 0;
}

static int baikal_pcie_msi_host_init(struct pcie_port *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct device *dev = pci->dev;
	struct device_node *np = dev->of_node;
	struct device_node *msi_node;

	/*
	 * The MSI domain is set by the generic of_msi_configure().  This
	 * .msi_host_init() function keeps us from doing the default MSI
	 * domain setup in dw_pcie_host_init() and also enforces the
	 * requirement that "msi-parent" exists.
	 */
	msi_node = of_parse_phandle(np, "msi-parent", 0);
	if (!msi_node) {
		dev_warn(dev, "failed to find msi-parent\n");
		return -EINVAL;
	}

	return 0;
}

static const struct dw_pcie_host_ops baikal_pcie_host_ops = {
	.host_init = baikal_pcie_host_init,
	.msi_host_init = baikal_pcie_msi_host_init,
};

static const struct dw_pcie_ops baikal_pcie_ops = {
	.link_up = baikal_pcie_link_up,
};

static int baikal_pcie_get_msi(struct baikal_pcie *rc,
			struct device_node *msi_node,
			u64 *msi_addr)
{
	struct dw_pcie *pci = rc->pci;
	struct device *dev = pci->dev;
	int ret;
	struct resource res;
	memset(&res, 0, sizeof(res));

	/*
	 * Check if 'msi-parent' points to ARM GICv3 ITS, which is the only
	 * supported MSI controller.
	 */
	if (!of_device_is_compatible(msi_node, "arm,gic-v3-its")) {
		dev_err(dev, "unable to find compatible MSI controller\n");
		return -ENODEV;
	}

	/* derive GITS_TRANSLATER address from GICv3 */
	ret = of_address_to_resource(msi_node, 0, &res);
	if (ret < 0) {
		dev_err(dev, "unable to obtain MSI controller resources\n");
		return ret;
	}

	*msi_addr = res.start + GITS_TRANSLATER;
	return 0;
}

static int baikal_pcie_msi_steer(struct baikal_pcie *rc,
			struct device_node *msi_node)
{
	struct dw_pcie *pci = rc->pci;
	struct device *dev = pci->dev;
	int ret;
	u64 msi_addr;

	ret = baikal_pcie_get_msi(rc, msi_node, &msi_addr);
	if (ret < 0) {
		dev_err(dev, "MSI steering failed\n");
		return ret;
	}

	/* Program the msi_data */
	dw_pcie_write(pci->dbi_base + PCIE_MSI_ADDR_LO, 4,
			lower_32_bits(msi_addr));
	dw_pcie_write(pci->dbi_base + PCIE_MSI_ADDR_HI, 4,
			upper_32_bits(msi_addr));

	return 0;
}

int baikal_msi_init(struct baikal_pcie *rc, struct device_node *node)
{
	if (!of_find_property(node, "msi-controller", NULL)) {
		pr_err("%s: couldn't find msi-controller property in FDT\n", __func__);
		return -ENODEV;
	}

	return 0;
}

static int baikal_pcie_msi_enable(struct baikal_pcie *rc)
{
	struct dw_pcie *pci = rc->pci;
	struct device *dev = pci->dev;
	struct device_node *msi_node;
	int ret;

	/*
	 * The "msi-parent" phandle needs to exist
	 * for us to obtain the MSI node.
	 */

	msi_node = of_parse_phandle(dev->of_node, "msi-parent", 0);
	if (!msi_node) {
		dev_err(dev, "failed to read msi-parent from FDT\n");
		return -ENODEV;
	}

	ret = baikal_pcie_msi_steer(rc, msi_node);
	if (ret)
		goto out_put_node;

out_put_node:
	of_node_put(msi_node);
	return ret;
}

static irqreturn_t baikal_pcie_handle_error_irq(struct baikal_pcie *rc)
{
	return IRQ_HANDLED;
}

static irqreturn_t baikal_pcie_err_irq_handler(int irq, void *priv)
{
	struct baikal_pcie *rc = priv;

	return baikal_pcie_handle_error_irq(rc);
}

static int __init baikal_add_pcie_port(struct baikal_pcie *rc,
				       struct platform_device *pdev)
{
	struct dw_pcie *pci = rc->pci;
	struct pcie_port *pp = &pci->pp;
	struct resource *res;
	int irq;
	int ret;

	pci->dev = &pdev->dev;
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dbi");
	if (res) {
		pci->dbi_base = devm_ioremap_resource(pci->dev, res);
		if (IS_ERR(pci->dbi_base)) {
			dev_err(pci->dev, "error with ioremap\n");
			return PTR_ERR(pci->dbi_base);
		}
	} else {
		dev_err(pci->dev, "missing *dbi* reg space\n");
		return -EINVAL;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(pci->dev, "missing IRQ resource: %d\n", irq);
		return irq;
	}

	/* TODO enable it later */
	ret = request_irq(irq, baikal_pcie_err_irq_handler, IRQF_SHARED,
			"baikal-pcie-error-irq", rc);
	if (ret < 0) {
		dev_err(pci->dev, "failed to request error IRQ %d\n",
			irq);
		return ret;
	}
	/* end TODO */

	if (IS_ENABLED(CONFIG_PCI_MSI)) {
		ret = baikal_pcie_msi_enable(rc);
		if (ret) {
			dev_err(pci->dev, "failed to initialize MSI\n");
			return ret;
		}
	}

	pp->root_bus_nr = -1;
	pp->ops = &baikal_pcie_host_ops;

	ret = dw_pcie_host_init(pp);
	if (ret) {
		dev_err(pci->dev, "failed to initialize host\n");
		return ret;
	}

	return 0;
}

static int baikal_pcie_hw_init_m(struct baikal_pcie *rc)
{
	u32 reg;

	// TODO add PHY configuration if needed

	/* Deassert PHY reset */
	reg = baikal_pcie_lcru_readl(rc->lcru, BAIKAL_LCRU_PCIE_RESET(rc->bus_nr));
	reg &= ~BAIKAL_PCIE_PHY_RESET;
	baikal_pcie_lcru_writel(rc->lcru, BAIKAL_LCRU_PCIE_RESET(rc->bus_nr), reg);

	// TODO timeout?

	/* Enable access to the PHY registers */
	reg = baikal_pcie_lcru_readl(rc->lcru, BAIKAL_LCRU_PCIE_GEN_CTL(rc->bus_nr));
	reg |= (BAIKAL_PCIE_PHY_MGMT_ENABLE | BAIKAL_PCIE_DBI2_MODE);
	baikal_pcie_lcru_writel(rc->lcru, BAIKAL_LCRU_PCIE_GEN_CTL(rc->bus_nr), reg);

	// TODO timeout?

	/* Clear all software controlled resets of the controller */
	reg = baikal_pcie_lcru_readl(rc->lcru, BAIKAL_LCRU_PCIE_RESET(rc->bus_nr));
	reg &= ~(BAIKAL_PCIE_ADB_PWRDWN | BAIKAL_PCIE_HOT_RESET |
			BAIKAL_PCIE_NONSTICKY_RST |	BAIKAL_PCIE_STICKY_RST |
			BAIKAL_PCIE_PWR_RST | BAIKAL_PCIE_CORE_RST | BAIKAL_PCIE_PIPE_RESET);
	baikal_pcie_lcru_writel(rc->lcru, BAIKAL_LCRU_PCIE_RESET(rc->bus_nr), reg);

	// TODO timeout?

	if (IS_ENABLED(CONFIG_PCI_MSI)) {

		/* Set up the MSI translation mechanism: */

		/* First, set MSI_AWUSER to 0 */
		reg = baikal_pcie_lcru_readl(rc->lcru, BAIKAL_LCRU_PCIE_MSI_TRANS_CTL0);
		reg &= ~BAIKAL_PCIE_MSI_AWUSER_MASK;
		reg |= (0 << BAIKAL_PCIE_MSI_AWUSER_SHIFT);
		baikal_pcie_lcru_writel(rc->lcru, BAIKAL_LCRU_PCIE_MSI_TRANS_CTL0, reg);

		// TODO timeout?

		/* Second, enable MSI, the RC number for all RC is 0*/
		reg = baikal_pcie_lcru_readl(rc->lcru, BAIKAL_LCRU_PCIE_MSI_TRANS_CTL2);
		reg |= BAIKAL_PCIE_MSI_TRANS_EN(rc->bus_nr);
		reg &= ~BAIKAL_PCIE_MSI_RCNUM_MASK(rc->bus_nr);
		baikal_pcie_lcru_writel(rc->lcru, BAIKAL_LCRU_PCIE_MSI_TRANS_CTL2, reg);

	}

	return 0;

}

static const struct of_device_id of_baikal_pcie_match[] = {
	{
		.compatible = "baikal,pcie-m",
		.data = baikal_pcie_hw_init_m,
	},
	{},
};

static int __init baikal_pcie_probe(struct platform_device *pdev)
{
	struct dw_pcie *pci;
	struct baikal_pcie *pcie;
	struct device *dev = &pdev->dev;
	const struct of_device_id *of_id;
	int err;
	int (*hw_init_fn)(struct baikal_pcie *);
	u32 index[2];
	enum of_gpio_flags flags;
	int reset_gpio;
	pr_info("%s: ENTER\n", __func__);


	pcie = devm_kzalloc(dev, sizeof(*pcie), GFP_KERNEL);
	if (!pcie) {
		dev_err(dev, "failed to allocate memory\n");
		return -ENOMEM;
	}

	pci = devm_kzalloc(dev, sizeof(*pci), GFP_KERNEL);
	if (!pci) {
		dev_err(dev, "failed to allocate memory [2]\n");
		return -ENOMEM;
	}
	pci->dev = dev;
	pci->ops = &baikal_pcie_ops;

	pcie->pci = pci;

	pcie->lcru = syscon_regmap_lookup_by_phandle(dev->of_node,
					"baikal,pcie-lcru");
	if (IS_ERR(pcie->lcru)) {
		dev_err(dev, "No LCRU phandle specified\n");
		pcie->lcru = NULL;
		return -EINVAL;
	}

	if (of_property_read_u32_array(dev->of_node,
			"baikal,pcie-lcru", index, 2)) {
		dev_err(dev, "failed to read LCRU\n");
		pcie->lcru = NULL;
		return -EINVAL;
	}
	pcie->bus_nr = index[1];

	of_id = of_match_device(of_baikal_pcie_match, dev);
	if (!of_id || !of_id->data) {
		dev_err(dev, "device can't be handled by pcie-baikal\n");
		return -EINVAL;
	}
	hw_init_fn = of_id->data;

	pm_runtime_enable(dev);
	err = pm_runtime_get_sync(dev);
	if (err < 0) {
		dev_err(dev, "pm_runtime_get_sync failed\n");
		goto err_pm_disable;
	}


	baikal_pcie_cease_link(pcie);

	/* LINK DISABLED */
	reset_gpio = of_get_named_gpio_flags(dev->of_node, "reset-gpios", 0, &flags);
	if (reset_gpio != -EPROBE_DEFER && gpio_is_valid(reset_gpio)) {
		unsigned long gpio_flags;

		snprintf(pcie->reset_name, 32, "pcie%d-reset", pcie->bus_nr);
		if (flags & OF_GPIO_ACTIVE_LOW)
			gpio_flags = GPIOF_ACTIVE_LOW | GPIOF_OUT_INIT_LOW;
		else
			gpio_flags = GPIOF_OUT_INIT_HIGH;
		err = devm_gpio_request_one(dev, reset_gpio, gpio_flags,
					    pcie->reset_name);
		if (err) {
			dev_err(dev, "request GPIO failed (%d)\n", err);
			goto err_pm_disable;
		}
		pcie->reset_gpio = gpio_to_desc(reset_gpio);

		udelay(100);
//vvv: do it now or later in baikal_pcie_host_init()?
		gpiod_set_value_cansleep(pcie->reset_gpio, 0);
	}

	err = hw_init_fn(pcie);
	if (err) {
		//dev_info(dev, "PCIe link down\n"); // TODO PHY not initialized!
		err = 0;
		goto err_pm_put;
	}
	/* PHY INITIALIZED */
	platform_set_drvdata(pdev, pcie);

	err = baikal_add_pcie_port(pcie, pdev);
	if (err < 0)
		//goto err_gpio; TODO
		goto err_pm_put;

	return 0;

err_pm_put:
	pm_runtime_put(dev);

err_pm_disable:
	pm_runtime_disable(dev);

err_phy:
	return err;
}

#ifdef CONFIG_PM_SLEEP
static int baikal_pcie_suspend(struct device *dev)
{
	struct baikal_pcie *rc = dev_get_drvdata(dev);
	struct pcie_port *pp = &rc->pp;
	u32 val;

	/* clear MSE */
	val = dw_pcie_readl_rc(pp, PCI_COMMAND);
	val &= ~PCI_COMMAND_MEMORY;
	dw_pcie_writel_rc(pp, PCI_COMMAND, val);

	return 0;
}

static int baikal_pcie_resume(struct device *dev)
{
	struct baikal_pcie *rc = dev_get_drvdata(dev);
	struct pcie_port *pp = &rc->pp;
	u32 val;

	/* set MSE */
	val = dw_pcie_readl_rc(pp, PCI_COMMAND);
	val |= PCI_COMMAND_MEMORY;
	dw_pcie_writel_rc(pp, PCI_COMMAND, val);

	return 0;
}

static int baikal_pcie_suspend_noirq(struct device *dev)
{
	return 0;
}

static int baikal_pcie_resume_noirq(struct device *dev)
{
	return 0;
}
#endif

static const struct dev_pm_ops baikal_pcie_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(baikal_pcie_suspend, baikal_pcie_resume)
	SET_NOIRQ_SYSTEM_SLEEP_PM_OPS(baikal_pcie_suspend_noirq,
				      baikal_pcie_resume_noirq)
};

static struct platform_driver baikal_pcie_driver = {
	.driver = {
		.name	= "baikal-pcie",
		.of_match_table = of_baikal_pcie_match,
		.suppress_bind_attrs = true,
		.pm	= &baikal_pcie_pm_ops,
	},
};
builtin_platform_driver_probe(baikal_pcie_driver, baikal_pcie_probe);
