/*
 *  Baikal-T SOC platform support code.
 *
 *  Copyright (C) 2015,2016 Baikal Electronics JSC.
 *
 *  This program is free software; you can distribute it and/or modify it
 *  under the terms of the GNU General Public License (Version 2) as
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope it will be useful, but WITHOUT
 *  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 *  for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place - Suite 330, Boston MA 02111-1307, USA.
 *
 *  BAIKAL MIPS boards specific PCI support.
 */

#include <linux/of_irq.h>
#include <linux/of_pci.h>
#include <linux/delay.h>

#ifdef CONFIG_BAIKAL_EDMA
#include <linux/dma/baikal.h>
struct baikal_dma_chip *baikal_dma_chip;
#endif /* CONFIG_BAIKAL_EDMA */

#include "pci-baikal.h"

#define LINK_RETRAIN_TIMEOUT HZ
#define LINK_UP_TIMEOUT HZ

#define NEC_VENDOR_ID		0x1033
#define NEC_USB3_DEVID		0x0194

#define SM_VENDOR_ID		0x126f
#define SM750_DEVID			0x0750

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
#ifdef CONFIG_MIPS_BAIKAL_T1 /* we have Baikal-T1 chip which can perform up to GEN3 */
			target_speed = speed;
#else /* we have Baikal-T chip which is limited to GEN2 */
			target_speed = PCI_EXP_LNKCAP_SLS_5_0GB;
#endif
			if (baikal_t1_report_link_performance(pdev) < target_speed) {
				dev_info(&pdev->dev, "retrain link to GEN%d\n", target_speed);
				baikal_t1_pcie_link_retrain(target_speed);
				baikal_t1_report_link_performance(pdev);
				return;
			}
		}
	}
}
DECLARE_PCI_FIXUP_FINAL(PCI_ANY_ID, PCI_ANY_ID, baikal_t1_pcie_link_speed_fixup);

#ifdef CONFIG_BAIKAL_EDMA
static void baikal_t1_pcie_dmac_fixup(struct pci_dev *pdev)
{
	if (baikal_dma_chip) {
		dev_info(&pdev->dev, "eDMA support enabled - use eDMA-aware device driver to utilize\n");
		dev_set_drvdata(&pdev->bus->dev, baikal_dma_chip);
	}
}
DECLARE_PCI_FIXUP_FINAL(PCI_ANY_ID, PCI_ANY_ID, baikal_t1_pcie_dmac_fixup);
#endif /* CONFIG_BAIKAL_EDMA */

int pcibios_plat_dev_init(struct pci_dev *dev)
{
	uint16_t config;
	uint32_t dconfig;
	int pos;

	/* Enable reporting System errors and parity errors on all devices */
	/* Enable parity checking and error reporting */
	pci_read_config_word(dev, PCI_COMMAND, &config);
	config |= PCI_COMMAND_PARITY | PCI_COMMAND_SERR;
	pci_write_config_word(dev, PCI_COMMAND, config);


	/* Enable the PCIe normal error reporting */
	config = PCI_EXP_DEVCTL_CERE;	/* Correctable Error Reporting */
	config |= PCI_EXP_DEVCTL_NFERE;	/* Non-Fatal Error Reporting */
	config |= PCI_EXP_DEVCTL_FERE;	/* Fatal Error Reporting */
	config |= PCI_EXP_DEVCTL_URRE;	/* Unsupported Request */
	pcie_capability_set_word(dev, PCI_EXP_DEVCTL, config);

	/* Find the Advanced Error Reporting capability */
	pos = pci_find_ext_capability(dev, PCI_EXT_CAP_ID_ERR);

	if (pos) {
		/* Clear Uncorrectable Error Status */
		pci_read_config_dword(dev, pos + PCI_ERR_UNCOR_STATUS,
				&dconfig);
		pci_write_config_dword(dev, pos + PCI_ERR_UNCOR_STATUS,
				dconfig);
		/* Enable reporting of all uncorrectable errors */
		/* Uncorrectable Error Mask - turned on bits disable errors */
		pci_write_config_dword(dev, pos + PCI_ERR_UNCOR_MASK, 0);
		/*
		 * Leave severity at HW default. This only controls if
		 * errors are reported as uncorrectable or
		 * correctable, not if the error is reported.
		 */
		/* PCI_ERR_UNCOR_SEVER - Uncorrectable Error Severity */
		/* Clear Correctable Error Status */
		pci_read_config_dword(dev, pos + PCI_ERR_COR_STATUS, &dconfig);
		pci_write_config_dword(dev, pos + PCI_ERR_COR_STATUS, dconfig);
		/* Enable reporting of all correctable errors */
		/* Correctable Error Mask - turned on bits disable errors */
		pci_write_config_dword(dev, pos + PCI_ERR_COR_MASK, 0);
#ifdef DW_CHECK_ECRC
		/* Advanced Error Capabilities */
		pci_read_config_dword(dev, pos + PCI_ERR_CAP, &dconfig);
		/* ECRC Generation Enable */
		if (dconfig & PCI_ERR_CAP_ECRC_GENC)
			dconfig |= PCI_ERR_CAP_ECRC_GENE;
		/* ECRC Check Enable */
		if (dconfig & PCI_ERR_CAP_ECRC_CHKC)
			dconfig |= PCI_ERR_CAP_ECRC_CHKE;
		pci_write_config_dword(dev, pos + PCI_ERR_CAP, dconfig);
#endif /* DW_CHECK_ECRC */
		/* PCI_ERR_HEADER_LOG - Header Log Register (16 bytes) */
		/* Report all errors to the root complex */
		pci_write_config_dword(dev, pos + PCI_ERR_ROOT_COMMAND,
				PCI_ERR_ROOT_CMD_COR_EN |
				PCI_ERR_ROOT_CMD_NONFATAL_EN |
				PCI_ERR_ROOT_CMD_FATAL_EN);
		/* Clear the Root status register */
		pci_read_config_dword(dev, pos + PCI_ERR_ROOT_STATUS, &dconfig);
		pci_write_config_dword(dev, pos + PCI_ERR_ROOT_STATUS, dconfig);
	}

	return 0;
}

int  pcibios_map_irq(const struct pci_dev *dev, u8 slot, u8 pin)
{
	dev_info(&dev->dev, "Device %x:%x, irq %d\n", dev->vendor,
			dev->device, dev->irq);
	return dev->irq;
}


static void quirk_remap_bar_pci_to_paddr(struct pci_dev *dev)
{
	int i;

	for (i = 0; i < PCI_STD_RESOURCE_END; i++) {
		struct resource *r = &dev->resource[i];

		if (r->flags & IORESOURCE_MEM && resource_size(r) > 0) {
			r->start = BAIKAL_MAP_PCI_BUS_TO_PADDR(r->start);
			r->end = BAIKAL_MAP_PCI_BUS_TO_PADDR(r->end);
			dev_info(&dev->dev, "BAR %d: remapped %pR", i, r);
		}
	}
}

static void quirk_fix_pci_irq(struct pci_dev *dev)
{
	if (dev->irq == 0xff) {
		dev->irq = 0;
	}
	dev_info(&dev->dev,"quirk irq: %d\n", dev->irq);

}


DECLARE_PCI_FIXUP_FINAL(PCI_ANY_ID, PCI_ANY_ID, quirk_remap_bar_pci_to_paddr);
DECLARE_PCI_FIXUP_FINAL(PCI_ANY_ID, PCI_ANY_ID, quirk_fix_pci_irq);
