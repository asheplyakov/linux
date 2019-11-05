// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for AX99100 PCI-SPI multi-interface device
 *
 * Copyright (C) 2018 T-platforms JSC (fancer.lancer@gmail.com)
 */

#ifndef SPI_AX99100_H
#define SPI_AX99100_H

#include <linux/spinlock.h>
#include <linux/pci.h>
#include <linux/spi/spi.h>
#include <linux/i2c.h>

/* TODO delete this declaration when moving to the newest kernel */
#include <linux/kernel_compat.h>

/*
 * AX99100 SPI-PCI resources
 *
 * NOTE External clocks max frequency is platform-dependent. This
 * driver is using the neutral value of 100MHz.
 */
#define PCI_BAR_MASK		(BIT(0) | BIT(1) | BIT(5))
#define SPI_CKS125_MAX_FREQ	125000000
#define SPI_CKS100_MAX_FREQ	100000000
#define SPI_CKSEXT_MAX_FREQ	 24000000
#define SPI_MAX_LEN		65535
#define I2C_NUM_ITER		100

/*
 * AX99100 SPI interface configuration registers (PIO - BAR0)
 */
#define SPI_CMR			0x000
#define  SPI_CMR_SSP		BIT(0)
#define  SPI_CMR_MODE(_x)	((_x) << 1 & GENMASK(2, 1))
#define  SPI_CMR_LSB		BIT(3)
#define  SPI_CMR_MEN		BIT(4)
#define  SPI_CMR_ASS		BIT(5)
#define  SPI_CMR_SWE		BIT(6)
#define  SPI_CMR_SSOE		BIT(7)
#define SPI_CSS			0x001
#define  SPI_CSS_CKS_125MHZ	 0x00
#define  SPI_CSS_CKS_100MHZ	 0x01
#define  SPI_CSS_CKS_EXT	 0x02
#define  SPI_CSS_DIVEN		BIT(2)
#define SPI_BRR			0x004
#define SPI_DS			0x005
#define SPI_DT			0x006
#define SPI_DAOF		0x007
#define SPI_TOF0		0x008
#define SPI_TOF1		0x009
#define SPI_TOF2		0x00A
#define SPI_TOF3		0x00B
#define SPI_TOF4		0x00C
#define SPI_TOF5		0x00D
#define SPI_TOF6		0x00E
#define SPI_TOF7		0x00F
#define SPI_DFL0		0x010
#define  SPI_DFL_FLBT(_x)	((_x) & GENMASK(9, 0))
#define SPI_SSOL		0x012
#define  SPI_SSOL_SS(_x)	((_x) & GENMASK(2, 0))
#define  SPI_SSOL_EDE		BIT(3)
#define  SPI_SSOL_STOL(_x)	((_x) << 4 & GENMASK(6, 4))
#define SPI_DCR			0x013
#define  SPI_DCR_ETDMA		BIT(0)
#define  SPI_DCR_ERDMA		BIT(1)
#define  SPI_DCR_OPCFE		BIT(2)
#define  SPI_DCR_DMA_GO		BIT(3)
#define  SPI_DCR_FBT		BIT(4)
#define  SPI_DCR_CSS		BIT(5)
#define  SPI_DCR_TCI_EN		BIT(6)
#define  SPI_DCR_TERRI_EN	BIT(7)
#define SPI_MISR		0x014
#define  SPI_MISR_TC		BIT(0)
#define  SPI_MISR_TERR		BIT(1)

/*
 * Port control registers including DMA and reset functions (MMIO - BAR1)
 */
#define TDMA_SAR0		0x080
#define TDMA_SAR1		0x084
#define TDMA_LR			0x088
#define  TDMA_LR_LEN(_x)	((_x) & GENMASK(15, 0))
#define TDMA_STAR		0x08C
#define  TDMA_STAR_START	BIT(0)
#define TDMA_STPR		0x090
#define  TDMA_STPR_STOP		BIT(0)
#define TDMA_SR			0x094
#define  TDMA_SR_DONE		BIT(0)
#define  TDMA_SR_STOP		BIT(1)
#define  TDMA_SR_CPLAB		BIT(2)
#define  TDMA_SR_CPLUR		BIT(3)
#define  TDMA_SR_POISON		BIT(4)
#define  TDMA_SR_ERR		BIT(5)
#define TDMA_BNTR		0x098
#define  TDMA_BNTR_BNT(_x)	((_x) & GENMASK(15, 0))
#define RDMA_SAR0		0x100
#define RDMA_SAR1		0x104
#define RDMA_LR			0x108
#define  RDMA_LR_LEN(_x)	((_x) & GENMASK(15, 0))
#define RDMA_STAR		0x10C
#define  RDMA_STAR_START	BIT(0)
#define RDMA_STPR		0x110
#define  RDMA_STPR_STOP		BIT(0)
#define RDMA_SR			0x114
#define  RDMA_SR_DONE		BIT(0)
#define  RDMA_SR_STOP		BIT(1)
#define  RDMA_SR_ERR		BIT(3)
#define  RDMA_SR_RDY		BIT(4)
#define RDMA_BNRR		0x118
#define  RDMA_BNRR_BNR(_x)	((_x) & GENMASK(15, 0))
#define SPI_SWRST		0x238

/*
 * Generic interface registers: i2c (MMIO - BAR5)
 */
#define I2C_CR			0x0C8
#define  I2C_CR_DATA(_x)	((_x) & GENMASK(7, 0))
#define  I2C_CR_MA(_x)		((_x) << 8 & GENMASK(23, 8))
#define  I2C_CR_MAF		BIT(24)
#define  I2C_CR_ADR(_x)		((_x) << 25 & GENMASK(30, 25))
#define  I2C_CR_RW_IND		BIT(31)
#define I2C_SCLPR		0x0CC
#define  I2C_SCLPR_100KHZ	(0xEE << 16 | 0x183)
#define  I2C_SCLPR_400KHZ	(0x3C << 16 | 0x61)
#define I2C_SCLCR		0x0D0
#define  I2C_SCLCR_ADR_LSB(_x)	((_x) & BIT(0))
#define  I2C_SCLCR_SBRT_ns(_x)	(((_x) / 16) << 1 & GENMASK(8, 1))
#define  I2C_SCLCR_SHSC_ns(_x)	(((_x) / 16) << 9 & GENMASK(24, 9))
#define  I2C_SCLCR_LHCEF	BIT(29)
#define  I2C_SCLCR_RCVF		BIT(30)
#define  I2C_SCLCR_NACK		BIT(31)
#define I2C_BFTR		0x0D4
#define  I2C_BFTR_BFT_ns(_x)	(((_x) / 16) & GENMASK(15, 0))

struct ax99100_pci {
	struct pci_dev *pdev;

	int irq;
	void __iomem *sm;
	void __iomem *dm;
	void __iomem *im;
};

struct ax99100_spi {
	struct spi_controller *ctlr;

	u32 sclk;
};

struct ax99100_i2c {
	struct i2c_adapter adapter;
};

struct ax99100_data {
	struct ax99100_pci pci;

	struct ax99100_spi spi;

#ifdef CONFIG_SPI_AX99100_I2C
	struct ax99100_i2c i2c;
#endif
};

#endif /* SPI_AX99100_H */
