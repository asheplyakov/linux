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




#if 0
enum baikal_pcie_variant {
	BAIKAL_PCIE_M_X4,
	BAIKAL_PCIE_M_X8
};
#endif

struct baikal_pcie {
	struct pcie_port pp;
//	enum baikal_pcie_variant variant;
	int	bus_nr;
	//void __iomem		*base;		/* DT ti_conf */
	struct regmap *lcru;
#if 0
	int			phy_count;	/* DT phy-names count */
	struct phy		**phy;
#endif
	struct gpio_desc *reset_gpio;
	char reset_name[32];
};

#define to_baikal_pcie(x)	container_of((x), struct baikal_pcie, pp)

#if 0
static inline u32 baikal_pcie_readl(struct baikal_pcie *pcie, u32 offset)
{
	return readl(pcie->base + offset);
}

static inline void baikal_pcie_writel(struct baikal_pcie *pcie, u32 offset,
				      u32 value)
{
	writel(value, pcie->base + offset);
}
#endif

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

#if 0
static inline int baikal_t1_pcie_link_is_down(void)
{
	int reg = READ_PMU_REG(BK_PMU_PCIE_PMSC);
	return (reg & PMU_PCIE_PMSC_LTSSM_STATE_MASK) != LTSSM_L0;
}

static inline int baikal_t1_pcie_link_is_training(void)
{
	int reg = READ_PCIE_REG(PCIE_LINK_CONTROL_LINK_STATUS_REG);
	return reg & PCIE_STA_LINK_TRAINING;
}

static void baikal_t1_wait_pcie_link_training_done(void)
{
	unsigned long start_jiffies = jiffies;
	while (baikal_t1_pcie_link_is_training()) {
		if (time_after(jiffies, start_jiffies + LINK_RETRAIN_TIMEOUT)) {
			pr_err("%s: link retrained for too long, timeout occured\n", __func__);
			break;
		}
		udelay(100);
	}
}

static inline void baikal_t1_pcie_link_retrain(int target_speed)
{
	int reg;
	unsigned long start_jiffies;

	// In case link is already training wait for training to complete
	baikal_t1_wait_pcie_link_training_done();

	wmb();

	// Set desired speed
	reg = READ_PCIE_REG(PCIE_LINK_CONTROL2_LINK_STATUS2_REG);
	reg &= ~PCIE_LINK_CONTROL2_GEN_MASK;
	reg |= target_speed;
	WRITE_PCIE_REG(PCIE_LINK_CONTROL2_LINK_STATUS2_REG, reg);

	wmb();

	// Set Retrain Link bit
	reg = READ_PCIE_REG(PCIE_LINK_CONTROL_LINK_STATUS_REG);
	reg |= PCI_EXP_LNKCTL_RL;
	WRITE_PCIE_REG(PCIE_LINK_CONTROL_LINK_STATUS_REG, reg);

	wmb();

	/* Wait for link training begin */
	start_jiffies = jiffies;
	while (!baikal_t1_pcie_link_is_training()) {
		if (time_after(jiffies, start_jiffies + LINK_RETRAIN_TIMEOUT)) {
			pr_err("%s: link retrained for too long, timeout occured\n", __func__);
			break;
		}
		udelay(100);
	}

	/* Wait for link training end */
	baikal_t1_wait_pcie_link_training_done();

	/* Wait for link is up */
	start_jiffies = jiffies;
	while (baikal_t1_pcie_link_is_down()) {
		if (time_after(jiffies, start_jiffies + LINK_UP_TIMEOUT)) {
			pr_err("%s: link is down for too long, timeout occured\n", __func__);
			break;
		}
		udelay(100);
	}
}

static int baikal_t1_report_link_performance(struct pci_dev *pdev)
{
	int reg = READ_PCIE_REG(PCIE_LINK_CONTROL_LINK_STATUS_REG);
	int speed = (reg & PCIE_CAP_LINK_SPEED_MASK) >> PCIE_CAP_LINK_SPEED_SHIFT;
	int width = (reg & PCIE_STA_LINK_WIDTH_MASK) >> PCIE_STA_LINK_WIDTH_SHIFT; 
	dev_info(&pdev->dev, "Link Status is     GEN%d, x%d\n", speed, width);
	return speed;
}

static void baikal_t1_pcie_link_speed_fixup(struct pci_dev *pdev)
{
	int reg, speed, width, target_speed;
	reg = READ_PCIE_REG(PCIE_LINK_CAPABILITIES_REG);
	speed = reg & PCI_EXP_LNKCAP_SLS;
	if (speed > PCI_EXP_LNKCAP_SLS_2_5GB) {
		pcie_capability_read_dword(pdev, PCI_EXP_LNKCAP, &reg);
		speed = reg & PCI_EXP_LNKCAP_SLS;
		width = (reg & PCI_EXP_LNKCAP_MLW) >> PCI_EXP_LNKSTA_NLW_SHIFT;
		dev_info(&pdev->dev, "Link Capability is GEN%d, x%d\n", speed, width);
		if (speed > PCI_EXP_LNKCAP_SLS_2_5GB) {
			target_speed = speed;
			if (baikal_t1_report_link_performance(pdev) < target_speed) {
				dev_info(&pdev->dev, "retrain link to GEN%d\n", target_speed);
				baikal_t1_pcie_link_retrain(target_speed);
				baikal_t1_report_link_performance(pdev);
				return;
			}
		}
	}
}
#endif

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

static inline int dw_pcie_link_is_training(struct pcie_port *pp)
{
	int reg = dw_pcie_readl_rc(pp,PCIE_LINK_CONTROL_LINK_STATUS_REG);
	return reg & PCIE_STA_LINK_TRAINING;
}

static void dw_wait_pcie_link_training_done(struct pcie_port *pp)
{
	unsigned long start_jiffies = jiffies;
	while (dw_pcie_link_is_training(pp)) {
		if (time_after(jiffies, start_jiffies + LINK_RETRAIN_TIMEOUT)) {
			pr_err("%s: link retrained for too long, timeout occured\n", __func__);
			break;
		}
		udelay(100);
	}
}

static int dw_report_link_performance(struct pcie_port *pp)
{
	int reg = dw_pcie_readl_rc(pp,PCIE_LINK_CONTROL_LINK_STATUS_REG);
	int speed = (reg & PCIE_CAP_LINK_SPEED_MASK) >> PCIE_CAP_LINK_SPEED_SHIFT;
	int width = (reg & PCIE_STA_LINK_WIDTH_MASK) >> PCIE_STA_LINK_WIDTH_SHIFT; 
	dev_info(pp->dev, "Link Status is     GEN%d, x%d\n", speed, width);
	return speed;
}

static inline void dw_pcie_link_retrain(struct pcie_port *pp, int target_speed)
{
	int reg;
	unsigned long start_jiffies;

	// In case link is already training wait for training to complete
	dw_wait_pcie_link_training_done(pp);

	// Set desired speed
	reg = dw_pcie_readl_rc(pp, PCIE_LINK_CONTROL2_LINK_STATUS2_REG);
	reg &= ~PCIE_LINK_CONTROL2_GEN_MASK;
	reg |= target_speed;
	dw_pcie_writel_rc(pp,PCIE_LINK_CONTROL2_LINK_STATUS2_REG, reg);

	// Set Retrain Link bit
	reg = dw_pcie_readl_rc(pp, PCIE_LINK_CONTROL_LINK_STATUS_REG);
	reg |= PCI_EXP_LNKCTL_RL;
	dw_pcie_writel_rc(pp, PCIE_LINK_CONTROL_LINK_STATUS_REG, reg);

	/* Wait for link training begin */
	start_jiffies = jiffies;
	while (!dw_pcie_link_is_training(pp)) {
		if (time_after(jiffies, start_jiffies + LINK_RETRAIN_TIMEOUT)) {
			pr_err("%s: link retrained for too long, timeout occured\n", __func__);
			break;
		}
		udelay(100);
	}

	/* Wait for link training end */
	dw_wait_pcie_link_training_done(pp);

	if(dw_pcie_wait_for_link(pp) == 0) {
		dw_report_link_performance(pp);
	}
}


static void pcie_link_speed_fixup(struct pci_dev *pdev)
{
	int reg, speed, width, target_speed;
	struct pcie_port *pp = pdev->bus->sysdata;

//	pcie_capability_read_dword(pdev, PCI_EXP_LNKCAP, &reg);
//	speed = reg & PCI_EXP_LNKCAP_SLS;
//	width = (reg & PCI_EXP_LNKCAP_MLW) >> PCI_EXP_LNKSTA_NLW_SHIFT;
//	dev_info(&pdev->dev, "Link Capability is GEN%d, x%d\n", speed, width);
//	dw_report_link_performance(pp);


	reg = dw_pcie_readl_rc(pp, PCIE_LINK_CAPABILITIES_REG);
	speed = reg & PCI_EXP_LNKCAP_SLS;
	if (speed > PCI_EXP_LNKCAP_SLS_2_5GB) {
		pcie_capability_read_dword(pdev, PCI_EXP_LNKCAP, &reg);
		speed = reg & PCI_EXP_LNKCAP_SLS;
		width = (reg & PCI_EXP_LNKCAP_MLW) >> PCI_EXP_LNKSTA_NLW_SHIFT;
		dev_info(&pdev->dev, "Link Capability is GEN%d, x%d\n", speed, width);
		if (speed > PCI_EXP_LNKCAP_SLS_2_5GB) {
			target_speed = speed;
			if (dw_report_link_performance(pp) < target_speed) {
				dev_info(&pdev->dev, "retrain link to GEN%d\n", target_speed);
				dw_pcie_link_retrain(pp, target_speed);
				dw_report_link_performance(pp);
				return;
			}
		}
	}
}
DECLARE_PCI_FIXUP_FINAL(PCI_ANY_ID, PCI_ANY_ID, pcie_link_speed_fixup);

static uint32_t dw_pcie_phy_read(struct pcie_port *pp, uint32_t phy_addr)
{
	uint32_t reg;
	int i;

	/* Set lane0 for reading values. */
	dw_pcie_writel_rc(pp, PCIE_BK_MGMT_SEL_LANE, PHY_LANE0);

	/* Write the address of the PHY register. */
	dw_pcie_writel_rc(pp, PCIE_BK_MGMT_CTRL, (phy_addr & BK_MGMT_CTRL_ADDR_MASK) | BK_MGMT_CTRL_READ);

	for (i = 0; i < PCIE_PHY_RETRIES; i++) {
		reg = dw_pcie_readl_rc(pp, PCIE_BK_MGMT_CTRL);
		if (reg & BK_MGMT_CTRL_DONE) {
			/* Read data register. */
			reg = dw_pcie_readl_rc(pp, PCIE_BK_MGMT_READ_DATA);
			pr_debug("%s: phy_addr=0x%x val=0x%x\n", __func__, phy_addr, reg);
			return reg;
		}
	}

	pr_err("%s: timeout expired for phy_addr=0x%x\n", __func__, phy_addr);

	/* return error */
	return -1;
}

static uint32_t dw_pcie_phy_write(struct pcie_port *pp, uint32_t phy_addr, uint32_t val)
{
	uint32_t reg;
	int i;

	pr_debug("%s: phy_addr=0x%x val=0x%x\n", __func__, phy_addr, val);

	/* Set line. */
	dw_pcie_writel_rc(pp, PCIE_BK_MGMT_SEL_LANE, PHY_ALL_LANES);

	/* Write value to data register. */
	dw_pcie_writel_rc(pp, PCIE_BK_MGMT_WRITE_DATA, val);

	/* Write the address of the PHY register. */
	dw_pcie_writel_rc(pp, PCIE_BK_MGMT_CTRL, (phy_addr & BK_MGMT_CTRL_ADDR_MASK) | BK_MGMT_CTRL_WRITE);

	for (i = 0; i < PCIE_PHY_RETRIES; i++) {
		reg = dw_pcie_readl_rc(pp, PCIE_BK_MGMT_CTRL);
		if (reg & BK_MGMT_CTRL_DONE) {
			return 0;
		}
	}

	pr_err("%s: timeout expired for phy_addr=0x%x\n", __func__, phy_addr);

	/* return error */
	return -1;
}

static int baikal_pcie_link_up(struct pcie_port *pp)
{
	struct baikal_pcie *rc = to_baikal_pcie(pp);
	uint32_t misc_reg;
	uint32_t coh_reg;
	uint32_t class_reg;
	u32	lcru_reg;

	// Set class
	lcru_reg = baikal_pcie_lcru_readl(rc->lcru, BAIKAL_LCRU_PCIE_GEN_CTL(rc->bus_nr));
	baikal_pcie_lcru_writel(rc->lcru, BAIKAL_LCRU_PCIE_GEN_CTL(rc->bus_nr), lcru_reg & (~BAIKAL_PCIE_DBI2_MODE));

	misc_reg = dw_pcie_readl_rc(pp, PCIE_MISC_CONTROL_1_OFF);
	misc_reg |= DBI_RO_RW_EN;

	dw_pcie_writel_rc(pp, PCIE_MISC_CONTROL_1_OFF, misc_reg);
	class_reg = dw_pcie_readl_rc(pp, PCI_CLASS_REVISION);

	class_reg = (0x604 << 16) | (class_reg & 0xff);			// class PCI_PCI_BRIDGE=0x604
	dw_pcie_writel_rc(pp, PCI_CLASS_REVISION, class_reg);

	misc_reg &= ~DBI_RO_RW_EN;
	dw_pcie_writel_rc(pp, PCIE_MISC_CONTROL_1_OFF, misc_reg);
	baikal_pcie_lcru_writel(rc->lcru, BAIKAL_LCRU_PCIE_GEN_CTL(rc->bus_nr), lcru_reg);
	// Set coherence
	coh_reg =
		(0x7 << 27) | /* AWCACHE */
		(0x2 << 24) | /* AWDOMAIN */
		(0xb << 19) | /* ARCACHE */
		(0x2 << 16) | /* ARDOMAIN */
		(0xF << 11) | /* AWCACHE mode */
		(0x3 <<  8) | /* AWDOMAIN mode */
		(0xF <<  3) | /* ARCACHE mode */
		(0x3 <<  0) /* ARDOMAIN mode */
		;
//	dw_pcie_writel_rc(pp, PCIE_COHERENCE_CONTROL_3_OFF, coh_reg);

	u32 reg = baikal_pcie_lcru_readl(rc->lcru, BAIKAL_LCRU_PCIE_STATUS(rc->bus_nr));
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
	struct pcie_port *pp = &rc->pp;
	struct device *dev = pp->dev;
	u32 reg;
	int ok;

	if (baikal_pcie_link_up(pp)) {
		dev_err(dev, "link is already up\n");
		return 0;
	}

	reg = baikal_pcie_lcru_readl(rc->lcru, BAIKAL_LCRU_PCIE_GEN_CTL(rc->bus_nr));
	reg |= BAIKAL_PCIE_LTSSM_ENABLE;
	baikal_pcie_lcru_writel(rc->lcru, BAIKAL_LCRU_PCIE_GEN_CTL(rc->bus_nr), reg);

	ok = dw_pcie_wait_for_link(pp);
	if(ok == 0) {
		dw_report_link_performance(pp);
	}
	return ok;
}

#if 0
static void baikal_pcie_enable_interrupts(struct baikal_pcie *rc)
{
	baikal_pcie_writel(rc, PCIECTRL_DRA7XX_CONF_IRQSTATUS_MAIN,
			   ~INTERRUPTS);
	baikal_pcie_writel(rc,
			   PCIECTRL_DRA7XX_CONF_IRQENABLE_SET_MAIN, INTERRUPTS);
	baikal_pcie_writel(rc, PCIECTRL_DRA7XX_CONF_IRQSTATUS_MSI,
			   ~LEG_EP_INTERRUPTS & ~MSI);

	if (IS_ENABLED(CONFIG_PCI_MSI))
		baikal_pcie_writel(rc,
				   PCIECTRL_DRA7XX_CONF_IRQENABLE_SET_MSI, MSI);
	else
		baikal_pcie_writel(rc,
				   PCIECTRL_DRA7XX_CONF_IRQENABLE_SET_MSI,
				   LEG_EP_INTERRUPTS);
}
#endif

#if 0
static irqreturn_t baikal_pcie_msi_irq_handler(int irq, void *arg)
{
    struct pcie_port *pp = arg;

    return dw_handle_msi_irq(pp);
}
#endif

static void baikal_pcie_host_init(struct pcie_port *pp)
{
	struct baikal_pcie *pcie = to_baikal_pcie(pp);

	dw_pcie_setup_rc(pp);

#if 0 //vvv
	if (pcie->reset_gpio) {
		gpiod_set_value_cansleep(pcie->reset_gpio, 0);
		mdelay(20);
	}
#endif

	baikal_pcie_establish_link(pcie);

	//baikal_pcie_enable_interrupts(rc);
}

static int baikal_pcie_msi_host_init(struct pcie_port *pp,
				struct msi_controller *chip)
{
	struct device *dev = pp->dev;
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
		dev_err(dev, "failed to find msi-parent\n");
		return -EINVAL;
	}

	return 0;
}

static struct pcie_host_ops baikal_pcie_host_ops = {
	.link_up = baikal_pcie_link_up,
	.host_init = baikal_pcie_host_init,
	.msi_host_init = baikal_pcie_msi_host_init,
};

#if 0
static int baikal_pcie_intx_map(struct irq_domain *domain, unsigned int irq,
				irq_hw_number_t hwirq)
{
	irq_set_chip_and_handler(irq, &dummy_irq_chip, handle_simple_irq);
	irq_set_chip_data(irq, domain->host_data);

	return 0;
}

static const struct irq_domain_ops intx_domain_ops = {
	.map = baikal_pcie_intx_map,
};

static int baikal_pcie_init_irq_domain(struct pcie_port *pp)
{
	struct device *dev = pp->dev;
	struct device_node *node = dev->of_node;
	struct device_node *pcie_intc_node = of_get_next_child(node, NULL);

	if (!pcie_intc_node) {
		dev_err(dev, "No PCIe Intc node found\n");
		return -ENODEV;
	}

	pp->irq_domain = irq_domain_add_linear(pcie_intc_node, 4,
					       &intx_domain_ops, pp);
	if (!pp->irq_domain) {
		dev_err(dev, "Failed to get a INTx IRQ domain\n");
		return -ENODEV;
	}

	return 0;
}

static irqreturn_t baikal_pcie_msi_irq_handler(int irq, void *arg)
{
	struct baikal_pcie *rc = arg;
	struct pcie_port *pp = &rc->pp;
	u32 reg;

	reg = baikal_pcie_readl(rc, PCIECTRL_DRA7XX_CONF_IRQSTATUS_MSI);

	switch (reg) {
	case MSI:
		dw_handle_msi_irq(pp);
		break;
	case INTA:
	case INTB:
	case INTC:
	case INTD:
		generic_handle_irq(irq_find_mapping(pp->irq_domain, ffs(reg)));
		break;
	}

	baikal_pcie_writel(rc, PCIECTRL_DRA7XX_CONF_IRQSTATUS_MSI, reg);

	return IRQ_HANDLED;
}


static irqreturn_t baikal_pcie_irq_handler(int irq, void *arg)
{
	struct baikal_pcie *rc = arg;
	struct device *dev = rc->pp.dev;
	u32 reg;

	reg = baikal_pcie_readl(rc, PCIECTRL_DRA7XX_CONF_IRQSTATUS_MAIN);

	if (reg & ERR_SYS)
		dev_dbg(dev, "System Error\n");

	if (reg & ERR_FATAL)
		dev_dbg(dev, "Fatal Error\n");

	if (reg & ERR_NONFATAL)
		dev_dbg(dev, "Non Fatal Error\n");

	if (reg & ERR_COR)
		dev_dbg(dev, "Correctable Error\n");

	if (reg & ERR_AXI)
		dev_dbg(dev, "AXI tag lookup fatal Error\n");

	if (reg & ERR_ECRC)
		dev_dbg(dev, "ECRC Error\n");

	if (reg & PME_TURN_OFF)
		dev_dbg(dev,
			"Power Management Event Turn-Off message received\n");

	if (reg & PME_TO_ACK)
		dev_dbg(dev,
			"Power Management Turn-Off Ack message received\n");

	if (reg & PM_PME)
		dev_dbg(dev, "PM Power Management Event message received\n");

	if (reg & LINK_REQ_RST)
		dev_dbg(dev, "Link Request Reset\n");

	if (reg & LINK_UP_EVT)
		dev_dbg(dev, "Link-up state change\n");

	if (reg & CFG_BME_EVT)
		dev_dbg(dev, "CFG 'Bus Master Enable' change\n");

	if (reg & CFG_MSE_EVT)
		dev_dbg(dev, "CFG 'Memory Space Enable' change\n");

	baikal_pcie_writel(rc, PCIECTRL_DRA7XX_CONF_IRQSTATUS_MAIN, reg);

	return IRQ_HANDLED;
}
#endif

static int baikal_pcie_get_msi(struct baikal_pcie *rc,
			struct device_node *msi_node,
			u64 *msi_addr)
{
	struct pcie_port *pp = &rc->pp;
	struct device *dev = pp->dev;
	int ret;
	struct resource res;

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
	struct pcie_port *pp = &rc->pp;
	struct device *dev = pp->dev;
	int ret;
	u64 msi_addr;
	u32 val;

	ret = baikal_pcie_get_msi(rc, msi_node, &msi_addr);
	if (ret < 0) {
		dev_err(dev, "MSI steering failed\n");
		return ret;
	}

	/* Program the msi_data */
	dw_pcie_cfg_write(pp->dbi_base + PCIE_MSI_ADDR_LO, 4,
			lower_32_bits(msi_addr));
	dw_pcie_cfg_write(pp->dbi_base + PCIE_MSI_ADDR_HI, 4,
			upper_32_bits(msi_addr));

	return 0;
}

int baikal_msi_init(struct baikal_pcie *rc, struct device_node *node)
{
	//struct iproc_msi *msi;
	int i, ret;
	unsigned int cpu;

	if (!of_find_property(node, "msi-controller", NULL))
		return -ENODEV;

	/*if (pcie->msi)
		return -EBUSY;

	msi = devm_kzalloc(pcie->dev, sizeof(*msi), GFP_KERNEL);
	if (!msi)
		return -ENOMEM;

	msi->pcie = pcie;
	pcie->msi = msi;
	msi->msi_addr = pcie->base_addr;
	mutex_init(&msi->bitmap_lock);
	msi->nr_cpus = num_possible_cpus();*/

#if 0
	msi->nr_irqs = of_irq_count(node);
	if (!msi->nr_irqs) {
		dev_err(pcie->dev, "found no MSI GIC interrupt\n");
		return -ENODEV;
	}

	if (msi->nr_irqs > NR_HW_IRQS) {
		dev_warn(pcie->dev, "too many MSI GIC interrupts defined %d\n",
		msi->nr_irqs);
		msi->nr_irqs = NR_HW_IRQS;
	}

	if (msi->nr_irqs < msi->nr_cpus) {
		dev_err(pcie->dev,
		"not enough GIC interrupts for MSI affinity\n");
		return -EINVAL;
	}

	if (msi->nr_irqs % msi->nr_cpus != 0) {
		msi->nr_irqs -= msi->nr_irqs % msi->nr_cpus;
		dev_warn(pcie->dev, "Reducing number of interrupts to %d\n",
		msi->nr_irqs);
	}

	switch (pcie->type) {
	case IPROC_PCIE_PAXB_BCMA:
	case IPROC_PCIE_PAXB:
		msi->reg_offsets = iproc_msi_reg_paxb;
		msi->nr_eq_region = 1;
		msi->nr_msi_region = 1;
		break;
	case IPROC_PCIE_PAXC:
		msi->reg_offsets = iproc_msi_reg_paxc;
		msi->nr_eq_region = msi->nr_irqs;
		msi->nr_msi_region = msi->nr_irqs;
		break;
	default:
		dev_err(pcie->dev, "incompatible iProc PCIe interface\n");
		return -EINVAL;
	}

	if (of_find_property(node, "brcm,pcie-msi-inten", NULL))
		msi->has_inten_reg = true;

	msi->nr_msi_vecs = msi->nr_irqs * EQ_LEN;
	msi->bitmap = devm_kcalloc(pcie->dev, BITS_TO_LONGS(msi->nr_msi_vecs),
			sizeof(*msi->bitmap), GFP_KERNEL);
	if (!msi->bitmap)
		return -ENOMEM;

	msi->grps = devm_kcalloc(pcie->dev, msi->nr_irqs, sizeof(*msi->grps),
			GFP_KERNEL);
	if (!msi->grps)
		return -ENOMEM;

	for (i = 0; i < msi->nr_irqs; i++) {
		unsigned int irq = irq_of_parse_and_map(node, i);

		if (!irq) {
			dev_err(pcie->dev, "unable to parse/map interrupt\n");
			ret = -ENODEV;
			goto free_irqs;
		}
		msi->grps[i].gic_irq = irq;
		msi->grps[i].msi = msi;
		msi->grps[i].eq = i;
	}

	/* Reserve memory for event queue and make sure memories are zeroed */
	msi->eq_cpu = dma_alloc_coherent(pcie->dev,
			msi->nr_eq_region * EQ_MEM_REGION_SIZE,
			&msi->eq_dma, GFP_KERNEL);
	if (!msi->eq_cpu) {
		ret = -ENOMEM;
		goto free_irqs;
	}

	ret = iproc_msi_alloc_domains(node, msi);
	if (ret) {
		dev_err(pcie->dev, "failed to create MSI domains\n");
		goto free_eq_dma;
	}

	for_each_online_cpu(cpu) {
		ret = iproc_msi_irq_setup(msi, cpu);
		if (ret)
			goto free_msi_irq;
	}

	iproc_msi_enable(msi);

	return 0;

free_msi_irq:
	for_each_online_cpu(cpu)
		iproc_msi_irq_free(msi, cpu);
		iproc_msi_free_domains(msi);

free_eq_dma:
	dma_free_coherent(pcie->dev, msi->nr_eq_region * EQ_MEM_REGION_SIZE,
		msi->eq_cpu, msi->eq_dma);

free_irqs:
	for (i = 0; i < msi->nr_irqs; i++) {
		if (msi->grps[i].gic_irq)
			irq_dispose_mapping(msi->grps[i].gic_irq);
	}
	pcie->msi = NULL;
	return ret;
#endif
}

static int baikal_pcie_msi_enable(struct baikal_pcie *rc)
{
	struct pcie_port *pp = &rc->pp;
	struct device *dev = pp->dev;
	struct device_node *msi_node;
	int ret;

	/*
	 * The "msi-parent" phandle needs to exist
	 * for us to obtain the MSI node.
	 */

	msi_node = of_parse_phandle(dev->of_node, "msi-parent", 0);
	if (!msi_node)
		return -ENODEV;

	ret = baikal_pcie_msi_steer(rc, msi_node);
	if (ret)
		goto out_put_node;

	//ret = baikal_pcie_msi_init(pcie, msi_node);

out_put_node:
	of_node_put(msi_node);
	return ret;
}

static irqreturn_t baikal_pcie_handle_error_irq(struct baikal_pcie *rc)
{
	u32 reg;
	//struct device *dev = &rc->pp->dev;

#if 0
	reg = ks_pcie_app_readl(ks_pcie, ERR_IRQ_STATUS);
	if (!reg)
		return IRQ_NONE;

	if (reg & ERR_SYS)
		dev_err(dev, "System Error\n");

	if (reg & ERR_FATAL)
		dev_err(dev, "Fatal Error\n");

	if (reg & ERR_NONFATAL)
		dev_dbg(dev, "Non Fatal Error\n");

	if (reg & ERR_CORR)
		dev_dbg(dev, "Correctable Error\n");

	if (!ks_pcie->is_am6 && (reg & ERR_AXI))
		dev_err(dev, "AXI tag lookup fatal Error\n");

	if (reg & ERR_AER || (ks_pcie->is_am6 && (reg & AM6_ERR_AER)))
		dev_err(dev, "ECRC Error\n");

	ks_pcie_app_writel(ks_pcie, ERR_IRQ_STATUS, reg);
#endif

	return IRQ_HANDLED;
}

static irqreturn_t baikal_pcie_err_irq_handler(int irq, void *priv)
{
	struct baikal_pcie *rc = priv;

	return baikal_pcie_handle_error_irq(rc);
}

static void baikal_pcie_fine_tune(struct pcie_port *pp)
{
	int reg;

	/* 5. Set the fast mode. */
	/*reg = READ_PCIE_REG(PCIE_PORT_LINK_CTRL_OFF);
	reg |= FAST_LINK_MODE;
	WRITE_PCIE_REG(PCIE_PORT_LINK_CTRL_OFF, reg);

	reg = dw_pcie_phy_read(PCIE_PHY_DWC_GLBL_PLL_CFG_0);
	reg &= ~PCS_SDS_PLL_FTHRESH_MASK;
	dw_pcie_phy_write(PCIE_PHY_DWC_GLBL_PLL_CFG_0, reg);

	reg = dw_pcie_phy_read(PCIE_PHY_DWC_GLBL_TERM_CFG);
	reg |= FAST_TERM_CAL;
	dw_pcie_phy_write(PCIE_PHY_DWC_GLBL_TERM_CFG, reg);

	reg = dw_pcie_phy_read(PCIE_PHY_DWC_RX_LOOP_CTRL);
	reg |= (FAST_OFST_CNCL | FAST_DLL_LOCK);
	dw_pcie_phy_write(PCIE_PHY_DWC_RX_LOOP_CTRL, reg);

	reg = dw_pcie_phy_read(PCIE_PHY_DWC_TX_CFG_0);
	reg |= (FAST_TRISTATE_MODE | FAST_RDET_MODE | FAST_CM_MODE);
	dw_pcie_phy_write(PCIE_PHY_DWC_TX_CFG_0, reg);*/

	/* 6. Set number of lanes. */
#if 0
	reg = dw_pcie_readl_rc(pp, PCIE_GEN2_CTRL_OFF);
	reg &= ~NUM_OF_LANES_MASK;
	reg |= (0x4 << NUM_OF_LANES_SHIFT);
	//reg |= DIRECT_SPEED_CHANGE; // also force Directed Speed Change
	dw_pcie_writel_rc(pp, PCIE_GEN2_CTRL_OFF, reg);
#endif

	/* 7. Enable GEN3 */
	/*reg = READ_PCIE_REG(PCIE_GEN3_EQ_CONTROL_OFF);
	reg &= ~(GEN3_EQ_FB_MODE_MASK | GEN3_EQ_PSET_REQ_VEC_MASK);
	reg |= ((GEN3_EQ_EVAL_2MS_DISABLE) | (0x1 << GEN3_EQ_FB_MODE_SHIFT) |
		(0x1 << GEN3_EQ_PSET_REQ_VEC_SHIFT));
	WRITE_PCIE_REG(PCIE_GEN3_EQ_CONTROL_OFF, reg);

	WRITE_PCIE_REG(PCIE_LANE_EQUALIZATION_CONTROL01_REG, 0);
	WRITE_PCIE_REG(PCIE_LANE_EQUALIZATION_CONTROL23_REG, 0);

	dw_pcie_phy_write(PCIE_PHY_DWC_RX_PRECORR_CTRL, 0);
	dw_pcie_phy_write(PCIE_PHY_DWC_RX_CTLE_CTRL, 0x200);
	dw_pcie_phy_write(PCIE_PHY_DWC_RX_VMA_CTRL, 0xc000);
	dw_pcie_phy_write(PCIE_PHY_DWC_PCS_LANE_VMA_FINE_CTRL_0, 0);
	dw_pcie_phy_write(PCIE_PHY_DWC_PCS_LANE_VMA_FINE_CTRL_1, 0);
	dw_pcie_phy_write(PCIE_PHY_DWC_PCS_LANE_VMA_FINE_CTRL_2, 0);
	dw_pcie_phy_write(PCIE_PHY_DWC_EQ_WAIT_TIME, 0xa);*/

	/* 7.1 Disable entire DFE */
	/*reg = dw_pcie_phy_read(pp, PCIE_PHY_DWC_RX_LOOP_CTRL);
	reg |= 0x2;
	dw_pcie_phy_write(pp, PCIE_PHY_DWC_RX_LOOP_CTRL, reg);*/

	//reg = dw_pcie_phy_read(PCIE_PHY_DWC_RX_AEQ_VALBBD_2);
    reg = 0x3F;
    dw_pcie_phy_write(pp, PCIE_PHY_DWC_RX_AEQ_VALBBD_2, reg);

	//reg = dw_pcie_phy_read(PCIE_PHY_DWC_RX_AEQ_VALBBD_1);
	reg = 0;
	dw_pcie_phy_write(pp, PCIE_PHY_DWC_RX_AEQ_VALBBD_1, reg);
	dw_pcie_phy_write(pp, PCIE_PHY_DWC_RX_AEQ_VALBBD_0, reg);

}

static int __init baikal_add_pcie_port(struct baikal_pcie *rc,
				       struct platform_device *pdev)
{
	struct pcie_port *pp = &rc->pp;
	struct resource *res;
	int irq;
	int ret;

	pp->dev = &pdev->dev;

	/* irq */
	/*pp->irq = platform_get_irq(pdev, 1);
	if (pp->irq < 0)
		return pp->irq;*/

	/*
	if (IS_ENABLED(CONFIG_PCI_MSI)) {
		pp->msi_irq = platform_get_irq(pdev, 0);
	if (pp->msi_irq < 0)
		return pp->msi_irq;

		ret = devm_request_irq(pp->dev, pp->msi_irq,
			baikal_pcie_msi_irq_handler,
			IRQF_SHARED, "baikal-pcie-msi", pp);
		if (ret) {
			dev_err(pp->dev, "failed to request MSI IRQ\n");
			return ret;
		}
	}*/
	/* end irq */

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dbi");
	if (res) {
		pp->dbi_base = devm_ioremap(pp->dev, res->start, resource_size(res));
		if (!pp->dbi_base) {
			dev_err(pp->dev, "error with ioremap\n");
			return -ENOMEM;
		}
	} else {
		dev_err(pp->dev, "missing *dbi* reg space\n");
		return -EINVAL;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(pp->dev, "missing IRQ resource: %d\n", irq);
		return irq;
	}

	/* TODO enable it later */
	ret = request_irq(irq, baikal_pcie_err_irq_handler, IRQF_SHARED,
			"baikal-pcie-error-irq", rc);
	if (ret < 0) {
		dev_err(pp->dev, "failed to request error IRQ %d\n",
			irq);
		return ret;
	}
	/* end TODO */

	if (IS_ENABLED(CONFIG_PCI_MSI)) {
		ret = baikal_pcie_msi_enable(rc);
		if (ret) {
			dev_err(pp->dev, "failed to initialize MSI\n");
			return ret;
		}
	}

	pp->root_bus_nr = -1;
	pp->ops = &baikal_pcie_host_ops;

	//baikal_pcie_fine_tune(pp);

	ret = dw_pcie_host_init(pp);
	if (ret) {
		dev_err(pp->dev, "failed to initialize host\n");
		return ret;
	}

	return 0;
}

static int baikal_pcie_hw_init_m(struct baikal_pcie *rc)
{
	unsigned int timeout = 10;
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
		//reg |= BAIKAL_PCIE_MSI_RCNUM(rc->bus_nr);
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
int	be_debug = 0;
static int __init baikal_pcie_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *res;
	struct baikal_pcie *pcie;
	const struct of_device_id *of_id;
	int err;
	int (*hw_init_fn)(struct baikal_pcie *);
	u32 index[2];
	enum of_gpio_flags flags;
	int reset_gpio;
	dev_dbg(dev, "pci be_debug %x\n", be_debug);


	if (be_debug) {
		if(be_debug == 1) {
			while(be_debug) {}
		} else if(be_debug == 2) 
			return -EBUSY;
	}

	pcie = devm_kzalloc(dev, sizeof(*pcie), GFP_KERNEL);
	if (!pcie)
		return -ENOMEM;

	pcie->lcru = syscon_regmap_lookup_by_phandle(dev->of_node,
					"baikal,pcie-lcru");
	if (IS_ERR(pcie->lcru)) {
		dev_err(dev, "No LCRU phandle specified\n");
		pcie->lcru = NULL;
		return -EINVAL;
	}

	if (of_property_read_u32_array(dev->of_node,
			"baikal,pcie-lcru", index, 2)) {
		pcie->lcru = NULL;
		return -EINVAL;
	}
	pcie->bus_nr = index[1];

	of_id = of_match_device(of_baikal_pcie_match, dev);
	if (!of_id || !of_id->data)
		return -EINVAL;
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

	err = baikal_add_pcie_port(pcie, pdev);
	if (err < 0)
		//goto err_gpio; TODO
		goto err_pm_put;

	platform_set_drvdata(pdev, pcie);
	return 0;

err_pm_put:
	pm_runtime_put(dev);

err_pm_disable:
	pm_runtime_disable(dev);

err_phy:
	/*while (--i >= 0) {
		phy_power_off(phy[i]);
		phy_exit(phy[i]);
	}*/

	return err;


#if 0

	data = rcar_pci_read_reg(pcie, MACSR);
	dev_info(dev, "PCIe x%d: link up\n", (data >> 20) & 0x3f);

	/* LINK UP */

	if (IS_ENABLED(CONFIG_PCI_MSI)) {
		err = rcar_pcie_enable_msi(pcie);
		if (err < 0) {
			dev_err(dev, "failed to enable MSI support: %d\n", err);
			goto err_pm_put;
		}
	}

	/* MSI INITIALIZED */

	err = rcar_pcie_enable(pcie);
	if (err)
		goto err_pm_put;

	return 0;

err_pm_put:
	pm_runtime_put(dev);

err_pm_disable:
	pm_runtime_disable(dev);
	return err;
#endif

}

#if 0
static int __init _baikal_pcie_probe(struct platform_device *pdev)
{
	u32 reg;
	int ret;
	int irq;
	int i;
	int phy_count;
	struct phy **phy;
	void __iomem *base;
	struct resource *res;
	struct baikal_pcie *dra7xx;
	struct pcie_port *pp;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	char name[10];
	int gpio_sel;
	enum of_gpio_flags flags;
	unsigned long gpio_flags;

	dra7xx = devm_kzalloc(dev, sizeof(*dra7xx), GFP_KERNEL);
	if (!dra7xx)
		return -ENOMEM;

	pp = &dra7xx->pp;
	pp->dev = dev;
	pp->ops = &baikal_pcie_host_ops;

	/*
	irq = platform_get_irq_byname(pdev, "msi");
	if (irq < 0) {
		dev_err(dev, "missing IRQ resource\n");
		return -EINVAL;
	}

	ret = devm_request_irq(dev, irq, baikal_pcie_irq_handler,
			       IRQF_SHARED, "baikal-pcie-msi", dra7xx);
	if (ret) {
		dev_err(dev, "failed to request irq\n");
		return ret;
	}*/

	/*
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "ti_conf");
	base = devm_ioremap_nocache(dev, res->start, resource_size(res));
	if (!base)
		return -ENOMEM;

	phy_count = of_property_count_strings(np, "phy-names");
	if (phy_count < 0) {
		dev_err(dev, "unable to find the strings\n");
		return phy_count;
	}

	phy = devm_kzalloc(dev, sizeof(*phy) * phy_count, GFP_KERNEL);
	if (!phy)
		return -ENOMEM;

	for (i = 0; i < phy_count; i++) {
		snprintf(name, sizeof(name), "pcie-phy%d", i);
		phy[i] = devm_phy_get(dev, name);
		if (IS_ERR(phy[i]))
			return PTR_ERR(phy[i]);

		ret = phy_init(phy[i]);
		if (ret < 0)
			goto err_phy;

		ret = phy_power_on(phy[i]);
		if (ret < 0) {
			phy_exit(phy[i]);

			goto err_phy;
		}
	}
	*/




	/* PHY INITIALIZED */

	dra7xx->base = base;
//	dra7xx->phy = phy;
//	dra7xx->phy_count = phy_count;

	pm_runtime_enable(dev);
	ret = pm_runtime_get_sync(dev);
	if (ret < 0) {
		dev_err(dev, "pm_runtime_get_sync failed\n");
		goto err_get_sync;
	}

	gpio_sel = of_get_gpio_flags(dev->of_node, 0, &flags);
	if (gpio_is_valid(gpio_sel)) {
		gpio_flags = (flags & OF_GPIO_ACTIVE_LOW) ?
				GPIOF_OUT_INIT_LOW : GPIOF_OUT_INIT_HIGH;
		ret = devm_gpio_request_one(dev, gpio_sel, gpio_flags,
					    "pcie_reset");
		if (ret) {
			dev_err(dev, "gpio%d request failed, ret %d\n",
				gpio_sel, ret);
			goto err_gpio;
		}
	} else if (gpio_sel == -EPROBE_DEFER) {
		ret = -EPROBE_DEFER;
		goto err_gpio;
	}

    reg = baikal_pcie_readl(dra7xx, PCIECTRL_DRA7XX_CONF_DEVICE_CMD);
	reg &= ~LTSSM_EN;
	baikal_pcie_writel(dra7xx, PCIECTRL_DRA7XX_CONF_DEVICE_CMD, reg);
#endif

#if 0
static int __init dra7xx_add_pcie_port(struct baikal_pcie *dra7xx,
                       struct platform_device *pdev)
{
    int ret;
    struct pcie_port *pp = &dra7xx->pp;
    struct device *dev = pp->dev;
    struct resource *res;

    pp->irq = platform_get_irq(pdev, 1);
    if (pp->irq < 0) {
        dev_err(dev, "missing IRQ resource\n");
        return -EINVAL;
    }

    ret = devm_request_irq(dev, pp->irq, baikal_pcie_msi_irq_handler,
                   IRQF_SHARED | IRQF_NO_THREAD,
                   "dra7-pcie-msi", dra7xx);
    if (ret) {
        dev_err(dev, "failed to request irq\n");
        return ret;
    }

    if (!IS_ENABLED(CONFIG_PCI_MSI)) {
        ret = baikal_pcie_init_irq_domain(pp);
        if (ret < 0)
            return ret;
    }

	/* MSI INITIALIZED */

    res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "rc_dbics");
    pp->dbi_base = devm_ioremap(dev, res->start, resource_size(res));
    if (!pp->dbi_base)
        return -ENOMEM;

    ret = dw_pcie_host_init(pp);
    if (ret) {
        dev_err(dev, "failed to initialize host\n");
        return ret;
    }

	/* LINK UP */

    return 0;
}
#endif



#if 0
	ret = dra7xx_add_pcie_port(dra7xx, pdev);
	if (ret < 0)
		goto err_gpio;

	platform_set_drvdata(pdev, dra7xx);
	return 0;

err_gpio:
	pm_runtime_put(dev);

err_get_sync:
	pm_runtime_disable(dev);

err_phy:
	while (--i >= 0) {
		phy_power_off(phy[i]);
		phy_exit(phy[i]);
	}

	return ret;
}
#endif

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
	struct baikal_pcie *dra7xx = dev_get_drvdata(dev);
#if 0
	int count = dra7xx->phy_count;

	while (count--) {
		phy_power_off(dra7xx->phy[count]);
		phy_exit(dra7xx->phy[count]);
	}
#endif

	return 0;
}

static int baikal_pcie_resume_noirq(struct device *dev)
{
	struct baikal_pcie *dra7xx = dev_get_drvdata(dev);
//	int phy_count = dra7xx->phy_count;
	int ret;
	int i;

#if 0
	for (i = 0; i < phy_count; i++) {
		ret = phy_init(dra7xx->phy[i]);
		if (ret < 0)
			goto err_phy;

		ret = phy_power_on(dra7xx->phy[i]);
		if (ret < 0) {
			phy_exit(dra7xx->phy[i]);
			goto err_phy;
		}
	}
#endif
	return 0;
#if 0
err_phy:
	while (--i >= 0) {
		phy_power_off(dra7xx->phy[i]);
		phy_exit(dra7xx->phy[i]);
	}
#endif
	return ret;
}
#endif

module_param(be_debug, int, 0644);

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
	.probe = baikal_pcie_probe,
};
builtin_platform_driver(baikal_pcie_driver);
