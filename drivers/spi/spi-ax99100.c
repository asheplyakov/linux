// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for AX99100 PCI-SPI multi-interface device
 *
 * Copyright (C) 2018 T-platforms JSC (fancer.lancer@gmail.com)
 */

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/pci.h>
#include <linux/io.h>
#include <linux/spi/spi.h>
#include <linux/i2c.h>

#include "spi-ax99100.h"

/*
 * AX99100 PCI device
 *
 * Asix AX99100 chip is multi-IO PCIe device, which provides an access
 * to Local Bus, Parallal and Serial Ports, SPI, i2c and GPIO interfaces
 * over single PCIe endpoint. The availability of certain ports depends on
 * the chip mode (CHIP_MODE[2-0] pins state). In case if CHIP_MODE={110,100}
 * the chip exposes the SPI-interface at function 3, while functions 0 and 1
 * turned to be the Serial Ports (see 82500_ax99100 driver). i2c and GPIO
 * interfaces are accessible in any chip mode, but GPIO interrupts are
 * available for function 0 only. That's why GPIO interface support code is
 * implemented in 8250_ax99100 driver.
 */

/*
 * AX99100 PCI-SPI driver
 *
 * SPI-interface configuration registers are exposed by BAR0 of PCIe config
 * space, BAR1 is used for DMA setup and BAR5 provides an access to common
 * interfaces like i2c/GPIO (use 8250_ax99100 driver to acce GPIOs). AX99100
 * IO-controller provides a features reach SPI interface like four SPI modes,
 * CS polarity invertion, LSB mode, clock source and devider settings, various
 * delays, fragmented transactions, embedded DMA engine. Of course we don't
 * use all of them for the driver implementation due to compatibility reason.
 * Particularly since the device FIFO is limited by 8 bytes only, the driver
 * implements the DMA-based transactions.
 *
 * AX99100 PCI-SPI driver is created on top of the generic implementation of
 * transfer_one_message() provided by the SPI core. In particular,
 * the developed callback methods perform the following functionality:
 * - prepare_transfer_hardware   - enable SPI master controller
 * - unprepare_transfer_hardware - disable SPI master controller
 * - prepare_message   - if transfer buffers aren't DMA-mapped, then map them
 * - unprepare_message - unmap buffers if mapping was done by prepare_message
 * - set_cs - select/deselect the corresponding CS pin in EDE-less config
 * - transfer_one - set transaction mode, LSB and speed setting, then submit
 *                  it for execution by device DMA engine.
 * - handle_err - reset SPI-controller in case of an error.
 *
 * Non of synchronization primitives are used since transfer operations are
 * performed by a single non-reentrant kernel thread.
 *
 * TODO
 * 1) Create an adaptive SPI baud rate algo by switching between available
 * clock sources: 125MHz internal PLL, 100MHz PCIe reference clock and custom
 * external clock - EXT_CLK. Internal PLL source is currently used by this
 * driver, which practically gives at most 62.5 MHz SPI clocks.
 * 2) Add an external chip-select decoder support. SPI-controller provides a way
 * to handle the external decoder connected to the SS pins (like sn74as138).
 * The feature is activated by setting SPI_SSOL_EDE bit allows to increase
 * a maximum number of SPI-slaves from 3 to 7.
 */
static inline u8 ax99100_spi_read(struct ax99100_data *ax, unsigned long addr)
{
	return ioread8(ax->pci.sm + addr);
}

static inline void ax99100_spi_write(struct ax99100_data *ax, u8 val,
				     unsigned long addr)
{
	iowrite8(val, ax->pci.sm + addr);
}

static inline u32 ax99100_dma_read(struct ax99100_data *ax, unsigned long addr)
{
	return ioread32(ax->pci.dm + addr);
}

static inline void ax99100_dma_write(struct ax99100_data *ax, u32 val,
				     unsigned long addr)
{
	iowrite32(val, ax->pci.dm + addr);
}

static void ax99100_spi_handle_err(struct spi_controller *ctrl,
				   struct spi_message *msg)
{
	struct ax99100_data *ax = spi_controller_get_devdata(ctrl);

	ax99100_dma_write(ax, 1, SPI_SWRST);
}

static int ax99100_spi_prep_hw(struct spi_controller *ctlr)
{
	struct ax99100_data *ax = spi_controller_get_devdata(ctlr);

	/* Just enable SPI master contoller and CS signals for now */
	ax99100_spi_write(ax, SPI_CMR_MEN | SPI_CMR_SSOE, SPI_CMR);

	return 0;
}

static int ax99100_spi_unprep_hw(struct spi_controller *ctlr)
{
	struct ax99100_data *ax = spi_controller_get_devdata(ctlr);

	/* Disable SPI master contoller and put CS signals to tri-state */
	ax99100_spi_write(ax, 0, SPI_CMR);

	return 0;
}

static inline int ax99100_spi_map_dma(struct spi_controller *ctlr,
				      struct spi_transfer *t)
{
	if (t->tx_buf) {
		t->tx_dma = dma_map_single(&ctlr->dev, (void *)t->tx_buf,
					   t->len, DMA_TO_DEVICE);
		if (dma_mapping_error(&ctlr->dev, t->tx_dma))
			return -ENOMEM;
	}

	if (t->rx_buf) {
		t->rx_dma = dma_map_single(&ctlr->dev, t->rx_buf,
					   t->len, DMA_FROM_DEVICE);
		if (dma_mapping_error(&ctlr->dev, t->rx_dma))
			goto err_unmap_dma;
	}

	return 0;

err_unmap_dma:
	if (t->tx_buf)
		dma_unmap_single(&ctlr->dev, t->tx_dma, t->len, DMA_TO_DEVICE);

	return -ENOMEM;
}

static inline void ax99100_spi_unmap_dma(struct spi_controller *ctlr,
					 struct spi_transfer *t)
{
	if (t->tx_buf)
		dma_unmap_single(&ctlr->dev, t->tx_dma, t->len, DMA_TO_DEVICE);

	if (t->rx_buf)
		dma_unmap_single(&ctlr->dev, t->rx_dma, t->len, DMA_FROM_DEVICE);
}

static int ax99100_spi_prep_msg(struct spi_controller *ctlr,
				struct spi_message *msg)
{
	struct spi_transfer *t;
	int ret, cntr = 0;

	/* Don't do anything since protocol driver already mapped the buffers */
	if (msg->is_dma_mapped)
		return 0;

	/* ax99100 provides a well resolved tx/rx SPI interface by means of
	 * emebedded DMA engine only. So we need to map all the buffers before
	 * actually submit the transfers
	 */
	list_for_each_entry(t, &msg->transfers, transfer_list) {
		ret = ax99100_spi_map_dma(ctlr, t);
		if (ret)
			goto err_unmap_dma;

		cntr++;
	}

	return 0;

err_unmap_dma:
	list_for_each_entry(t, &msg->transfers, transfer_list) {
		if (cntr-- <= 0)
			break;

		ax99100_spi_unmap_dma(ctlr, t);
	}

	return ret;
}

static int ax99100_spi_unprep_msg(struct spi_controller *ctlr,
				  struct spi_message *msg)
{
	struct spi_transfer *t;

	/* Protocol driver shall unmap the DMA buffers, the mapping was his
	 * doing in the first place
	 */
	if (msg->is_dma_mapped)
		return 0;

	list_for_each_entry(t, &msg->transfers, transfer_list) {
		ax99100_spi_unmap_dma(ctlr, t);
	}

	return 0;
}

static void ax99100_spi_set_cs(struct spi_device *spi, bool is_high)
{
	/* TODO replace master with controller field */
	struct ax99100_data *ax = spi_controller_get_devdata(spi->master);
	u8 ssol = 0x7;

	/* Since EDE isn't supported just set the shifted SS level */
	if (!is_high)
		ssol &= ~(1 << spi->chip_select);

	ax99100_spi_write(ax, SPI_SSOL_SS(ssol), SPI_SSOL);
}

static void ax99100_spi_set_mode(struct ax99100_data *ax, struct spi_device *spi)
{
	u8 cmr = SPI_CMR_MEN | SPI_CMR_SSOE;

	/* bits_per_word setting should have been verified by SPI core */

	cmr |= SPI_CMR_MODE(spi->mode);

	if (spi->mode & SPI_LSB_FIRST)
		cmr |= SPI_CMR_LSB;

	ax99100_spi_write(ax, cmr, SPI_CMR);
}

static void ax99100_spi_set_clk(struct ax99100_data *ax, struct spi_transfer *t)
{
	u8 divider;
	u32 sclk;

	/* Calculate the source clocks divider so the max frequency would
	 * be limited to 62.5MHz.
	 */
	divider = DIV_ROUND_UP(SPI_CKS125_MAX_FREQ, t->speed_hz);
	divider = clamp_t(u8, divider, 2, 255);

	sclk = SPI_CKS125_MAX_FREQ / divider;
	if (sclk == ax->spi.sclk)
		return;

	/* Disable, set then enable the clocks source selector */
	ax->spi.sclk = sclk;
	ax99100_spi_write(ax, SPI_CSS_CKS_125MHZ, SPI_CSS);
	ax99100_spi_write(ax, divider, SPI_BRR);
	ax99100_spi_write(ax, SPI_CSS_CKS_125MHZ | SPI_CSS_DIVEN, SPI_CSS);
}

static int ax99100_spi_submit(struct ax99100_data *ax, struct spi_transfer *t)
{
	u8 sdcr = SPI_DCR_DMA_GO | SPI_DCR_TCI_EN | SPI_DCR_TERRI_EN;

	if (t->len > SPI_MAX_LEN)
		return -EINVAL;

	/* Mapping has been done either by a protocol driver or by the
	 * prepare_message callback method. Here we just initialize the
	 * DMA engine and execute the transaction.
	 */
	if (t->tx_buf) {
		sdcr |= SPI_DCR_ETDMA;

		ax99100_dma_write(ax, (u32)t->tx_dma, TDMA_SAR0);
		ax99100_dma_write(ax, (u32)(t->tx_dma >> 32), TDMA_SAR1);
		ax99100_dma_write(ax, TDMA_LR_LEN(t->len), TDMA_LR);
		ax99100_dma_write(ax, TDMA_STAR_START, TDMA_STAR);
	} else {
		ax99100_dma_write(ax, 0, TDMA_SAR0);
		ax99100_dma_write(ax, 0, TDMA_SAR1);
		ax99100_dma_write(ax, 0, TDMA_LR);
	}

	if (t->rx_buf) {
		sdcr |= SPI_DCR_ERDMA;

		ax99100_dma_write(ax, (u32)t->rx_dma, RDMA_SAR0);
		ax99100_dma_write(ax, (u32)(t->rx_dma >> 32), RDMA_SAR1);
		ax99100_dma_write(ax, RDMA_LR_LEN(t->len), RDMA_LR);
		ax99100_dma_write(ax, RDMA_STAR_START, RDMA_STAR);
	} else {
		ax99100_dma_write(ax, 0, RDMA_SAR0);
		ax99100_dma_write(ax, 0, RDMA_SAR1);
		ax99100_dma_write(ax, 0, RDMA_LR);
	}

	/* Submit the transaction */
	ax99100_spi_write(ax, sdcr, SPI_DCR);

	return 1;
}

static void ax99100_spi_stop(struct ax99100_data *ax)
{
	/* Just stop both TX and RX DMA transfers */
	ax99100_dma_write(ax, TDMA_STPR_STOP, TDMA_STPR);
	ax99100_dma_write(ax, RDMA_STPR_STOP, RDMA_STPR);

	udelay(10);
}

static int ax99100_spi_transfer_one(struct spi_controller *ctlr,
				    struct spi_device *spi,
				    struct spi_transfer *t)
{
	struct ax99100_data *ax = spi_controller_get_devdata(ctlr);

	ax99100_spi_set_mode(ax, spi);

	ax99100_spi_set_clk(ax, t);

	return ax99100_spi_submit(ax, t);
}

static irqreturn_t ax99100_spi_isr(int irq, void *devid)
{
	struct ax99100_data *ax = devid;
	irqreturn_t ret = IRQ_NONE;
	u32 tdma_sr, rdma_sr;
	u8 smisr;

	/* Find out the cause of the IRQ */
	smisr = ax99100_spi_read(ax, SPI_MISR);
	if (smisr & SPI_MISR_TERR) {
		tdma_sr = ax99100_dma_read(ax, TDMA_SR);
		rdma_sr = ax99100_dma_read(ax, RDMA_SR);

		dev_err(&ax->spi.ctlr->dev, "Transfer failed: tx %x, rx %x\n",
			tdma_sr, rdma_sr);

		ret = IRQ_HANDLED;
	}
	if (smisr & SPI_MISR_TC) {
		spi_finalize_current_transfer(ax->spi.ctlr);

		ret = IRQ_HANDLED;
	}
	if (ret == IRQ_HANDLED)
		ax99100_spi_write(ax, smisr, SPI_MISR);

	return ret;
}

static int ax99100_spi_init(struct ax99100_data *ax)
{
	struct spi_controller *ctlr;
	struct pci_dev *pdev = ax->pci.pdev;
	int ret;

	/* Reset SPI interface before using it */
	ax99100_dma_write(ax, 1, SPI_SWRST);

	ctlr = spi_alloc_master(&pdev->dev, 0);
	if (!ctlr) {
		dev_err(&pdev->dev, "Memory allocation failed for SPI\n");
		return -ENOMEM;
	}

	/* ax99100 DMA supports 64-bit addresses, so try 64-bits DMA mask first.
	 * If it fails, then fall back to the 32-bits DMA addresses
	 */
	ret = dma_coerce_mask_and_coherent(&ctlr->dev, DMA_BIT_MASK(64));
	if (ret) {
		ret = dma_set_mask_and_coherent(&ctlr->dev, DMA_BIT_MASK(32));
		if (ret) {
			dev_err(&pdev->dev, "Couldn't set DMA mask\n");
			goto err_spi_free_master;
		}
	}

	ax->spi.ctlr = ctlr;
	ctlr->bus_num = -1;
	ctlr->num_chipselect = 3;
	ctlr->mode_bits = SPI_CPOL | SPI_CPHA | SPI_NO_CS | SPI_LSB_FIRST;
	ctlr->bits_per_word_mask = BIT(7);
	ctlr->min_speed_hz = SPI_CKS125_MAX_FREQ / 255;
	ctlr->max_speed_hz = SPI_CKS125_MAX_FREQ / 2;
	ctlr->flags = 0;
	ctlr->handle_err = ax99100_spi_handle_err;
	ctlr->prepare_transfer_hardware = ax99100_spi_prep_hw;
	ctlr->unprepare_transfer_hardware = ax99100_spi_unprep_hw;
	ctlr->prepare_message = ax99100_spi_prep_msg;
	ctlr->unprepare_message = ax99100_spi_unprep_msg;
	ctlr->set_cs = ax99100_spi_set_cs;
	ctlr->transfer_one = ax99100_spi_transfer_one;

	spi_controller_set_devdata(ctlr, ax);

	ret = devm_spi_register_controller(&pdev->dev, ctlr);
	if (ret) {
		dev_err(&pdev->dev, "Couldn't register SPI controller\n");
		goto err_spi_free_master;
	}

	ret = devm_request_irq(&pdev->dev, ax->pci.irq, ax99100_spi_isr,
			       IRQF_ONESHOT | IRQF_SHARED, "ax99100_spi", ax);
	if (ret) {
		dev_err(&pdev->dev, "Failed to set IRQ#%d handler\n",
			ax->pci.irq);
		goto err_spi_free_master;
	}

	dev_info(&pdev->dev, "SPI-%d interface created\n", ctlr->bus_num);

	return 0;

err_spi_free_master:
	spi_controller_put(ctlr);

	return ret;
}

static void ax99100_spi_clear(struct ax99100_data *ax)
{
	ax99100_spi_stop(ax);

	/* Manually free IRQ otherwise PCI free irq vectors will fail */
	devm_free_irq(&ax->pci.pdev->dev, ax->pci.irq, ax);
}

/*
 * AX99100 PCI-i2c interface driver
 *
 * AX99100 chips provides a functionally limited smbus-interface. It is made
 * mostly for EEPROM read/write operations and provides just SMBus
 * byte-read/byte-write/word-write data operations. Even though the operations
 * set is very limited it shall be enough to access some smbus-sensors,
 * i2c-muxes, GPIO-expanders.
 */
#ifdef CONFIG_SPI_AX99100_I2C
static inline u32 ax99100_i2c_read(struct ax99100_data *ax, unsigned long addr)
{
	return ioread32(ax->pci.im + addr);
}

static inline void ax99100_i2c_write(struct ax99100_data *ax, u32 val,
				     unsigned long addr)
{
	iowrite32(val, ax->pci.im + addr);
}

static int ax99100_i2c_xfer(struct i2c_adapter *adapter,
		u16 addr, unsigned short flags, char read_write, u8 command,
		int size, union i2c_smbus_data *data)
{
	struct ax99100_data *ax = adapter->algo_data;
	u32 sdacr = 0, sclcr;
	int itr;

	sclcr = ax99100_i2c_read(ax, I2C_SCLCR) & ~I2C_SCLCR_ADR_LSB(1);

	/* Collect the transfer settings */
	switch (size) {
	case I2C_SMBUS_BYTE_DATA:
		sdacr |= I2C_CR_MA(command) | I2C_CR_DATA(data->byte);
		break;
	case I2C_SMBUS_WORD_DATA:
		if (read_write == I2C_SMBUS_READ) {
			dev_warn(&adapter->dev, "Unsupported read-word op\n");
			return -EOPNOTSUPP;
		}
		sdacr |= I2C_CR_MAF |
			 I2C_CR_MA(((u16)command << 8) | (data->word & 0xFF)) |
			 I2C_CR_DATA(data->word >> 8);
		break;
	default:
		dev_warn(&adapter->dev, "Unsupported transaction %d\n", size);
		return -EOPNOTSUPP;
	}

	if (read_write == I2C_SMBUS_WRITE)
		sdacr |= I2C_CR_RW_IND;

	sdacr |= I2C_CR_ADR(addr >> 1);
	sclcr |= I2C_SCLCR_ADR_LSB(addr & 0x1);

	/* Execute the SMBus transaction over the i2c interface. We check
	 * the transaction status at most I2C_NUM_ITER times until give up
	 * and return an error.
	 */
	ax99100_i2c_write(ax, sclcr, I2C_SCLCR);
	ax99100_i2c_write(ax, sdacr, I2C_CR);

	for (itr = 0; itr < I2C_NUM_ITER; itr++) {
		sclcr = ax99100_i2c_read(ax, I2C_SCLCR);

		if (!(sclcr & (I2C_SCLCR_RCVF | I2C_SCLCR_NACK)))
			break;

		/* Wait after the status check because sometimes the
		 * acknowledgement comes before the code gets to the status
		 * read point.
		 */
		usleep_range(70, 100);
	}

	if (itr == I2C_NUM_ITER) {
		/* Failed to recover the line */
		if (sclcr & I2C_SCLCR_RCVF)
			return -EIO;

		/* Client is unavailable */
		if (sclcr & I2C_SCLCR_NACK)
			return -ENXIO;
	}

	if (read_write == I2C_SMBUS_READ) {
		sdacr = ax99100_i2c_read(ax, I2C_CR);
		data->byte = I2C_CR_DATA(sdacr);
	}

	return 0;
}

static u32 ax99100_i2c_func(struct i2c_adapter *adapter)
{
	return I2C_FUNC_SMBUS_BYTE_DATA | I2C_FUNC_SMBUS_WRITE_WORD_DATA;
}

static struct i2c_algorithm ax99100_i2c_algo = {
	.smbus_xfer = ax99100_i2c_xfer,
	.functionality = ax99100_i2c_func
};

static int ax99100_i2c_init(struct ax99100_data *ax)
{
	struct i2c_adapter *adapter = &ax->i2c.adapter;
	struct pci_dev *pdev = ax->pci.pdev;
	int ret;

	snprintf(adapter->name, sizeof(adapter->name), "ax99100_i2c");
	adapter->class = I2C_CLASS_HWMON | I2C_CLASS_SPD;
        adapter->algo = &ax99100_i2c_algo;
        adapter->algo_data = ax;
	adapter->dev.parent = &pdev->dev;

	ax99100_i2c_write(ax, I2C_SCLPR_400KHZ, I2C_SCLPR);

	ret = i2c_add_adapter(adapter);
	if (ret) {
		dev_err(&pdev->dev, "Couldn't register i2c controller\n");
		return ret;
	}

	dev_info(&pdev->dev, "i2c-%d 400KHz interface created\n", adapter->nr);

	return 0;
}

static void ax99100_i2c_clear(struct ax99100_data *ax)
{
	i2c_del_adapter(&ax->i2c.adapter);
}
#else
static int ax99100_i2c_init(struct ax99100_data *ax)
{
	return 0;
}

static void ax99100_i2c_clear(struct ax99100_data *ax) {}
#endif /* CONFIG_SPI_AX99100_I2C */

/*
 * AX99100 PCI general driver
 *
 * The PCI-SPI function is activated by setting the CHIP_MODE to 110/100.
 * In this case the PCI device provides three PCIe functions:
 * Functions 0-1: Unified/Multiprotocol Serial ports
 * Function  3: SPI controller
 * Each of the functions also provides an access to common interfaces like
 * i2c and GPIO by means of BAR5 MMIO registers. This driver supports the SPI
 * controller and i2c interface only. GPIO-related code is moved to serial
 * ports driver (see 8250_ax99100 driver).
 */
static struct ax99100_data *ax99100_data_create(struct pci_dev *pdev)
{
	struct ax99100_data *ax;

	ax = devm_kzalloc(&pdev->dev, sizeof(*ax), GFP_KERNEL);
	if (!ax) {
		dev_err(&pdev->dev, "Memory allocation failed for data\n");
		return ERR_PTR(-ENOMEM);
	}

	ax->pci.pdev = pdev;

	return ax;
}

static int ax99100_pci_init(struct ax99100_data *ax)
{
	struct pci_dev *pdev = ax->pci.pdev;
	void __iomem * const *bars;
	int ret;

	ret = pcim_enable_device(pdev);
	if (ret) {
		dev_err(&pdev->dev, "Failed to enable PCIe device\n");
		return ret;
	}

	pci_set_master(pdev);

	/* Map all PCIe memory regions provided by the device */
	ret = pcim_iomap_regions(pdev, PCI_BAR_MASK, "ax99100_spi");
	if (ret) {
		dev_err(&pdev->dev, "Failed to request PCI regions\n");
		goto err_clear_master;
	}

	bars = pcim_iomap_table(pdev);
	ax->pci.sm = bars[0];
	ax->pci.dm = bars[1];
	ax->pci.im = bars[5];

	pci_set_drvdata(pdev, ax);

	ret = pci_alloc_irq_vectors(pdev, 1, 1, PCI_IRQ_MSI | PCI_IRQ_LEGACY);
	if (ret != 1) {
		dev_err(&pdev->dev, "Failed to alloc INTx IRQ vector\n");
		goto err_clear_drvdata;
	}

	ax->pci.irq = pci_irq_vector(pdev, 0);
	if (ax->pci.irq < 0) {
		dev_err(&pdev->dev, "Failed to get IRQ vector\n");
		goto err_free_irq_vectors;	
	}

	return 0;

err_free_irq_vectors:
        pci_free_irq_vectors(pdev);

err_clear_drvdata:
	pci_set_drvdata(pdev, NULL);

err_clear_master:
	pci_clear_master(pdev);

	return ret;
}

static void ax99100_pci_clear(struct ax99100_data *ax)
{
	pci_free_irq_vectors(ax->pci.pdev);

	pci_set_drvdata(ax->pci.pdev, NULL);

	pci_clear_master(ax->pci.pdev);
}

static int ax99100_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	struct ax99100_data *ax;
	int ret;

	ax = ax99100_data_create(pdev);
	if (IS_ERR(ax))
		return PTR_ERR(ax);

	ret = ax99100_pci_init(ax);
	if (ret)
		return ret;

	ret = ax99100_spi_init(ax);
	if (ret)
		goto err_pci_clear;

	ret = ax99100_i2c_init(ax);
	if (ret)
		goto err_spi_clear;

	return 0;

err_spi_clear:
	ax99100_spi_clear(ax);

err_pci_clear:
	ax99100_pci_clear(ax);

	return ret;
}

static void ax99100_remove(struct pci_dev *pdev)
{
	struct ax99100_data *ax = pci_get_drvdata(pdev);

	ax99100_i2c_clear(ax);

	ax99100_spi_clear(ax);

	ax99100_pci_clear(ax);
}

static const struct pci_device_id ax99100_pci_id[] = {
	{ PCI_VENDOR_ID_ASIX, PCI_DEVICE_ID_ASIX_AX99100, 0xa000, 0x6000 },
	{ },
};
MODULE_DEVICE_TABLE(pci, ax99100_pci_id);

static struct pci_driver ax99100_driver = {
	.name           = "ax99100_spi",
	.id_table       = ax99100_pci_id,
	.probe          = ax99100_probe,
	.remove         = ax99100_remove
};
module_pci_driver(ax99100_driver);

MODULE_DESCRIPTION("Asix AX99100 PCI-SPI driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Sergey Semin <fancer.lancer@gmail.com>");
