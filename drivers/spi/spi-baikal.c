/*
 * Baikal Electronics Spi Flash Driver.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/highmem.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/scatterlist.h>
#include <linux/mfd/syscon.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/acpi.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include "spi-dw.h"


/* Slave spi_dev related */
struct chip_data {
	u8 tmode;	/* TR/TO/RO/EEPROM */
	u8 type;	/* SPI/SSP/MicroWire */

	u8 poll_mode;	/* 1 means use poll mode */

	u16 clk_div;	/* baud rate divider */
	u32 speed_hz;	/* baud rate */
	void (*cs_control)(u32 command);
};

static void dw_set_cs (struct spi_device *spi, bool enable)
{
	struct dw_spi *dws = spi_controller_get_devdata(spi->controller);
	struct chip_data *chip = spi_get_ctldata(spi);

	if (chip && chip->cs_control)
		chip->cs_control(enable);

	if (enable)
		dw_writel(dws, DW_SPI_SER, BIT(spi->chip_select));
	else if (dws->cs_override)
		dw_writel(dws, DW_SPI_SER, 0);
}

/* Return the max entries we can fill into tx fifo */
static inline u32 tx_max (struct dw_spi *dws)
{
	u32 tx_left, tx_room, rxtx_gap;

	tx_left = (dws->tx_end -dws->tx) / dws->n_bytes;
	tx_room = dws->fifo_len -dw_readl(dws, DW_SPI_TXFLR);

	/*
	 * Another concern is about the tx/rx mismatch, we
	 * though to use (dws->fifo_len - rxflr - txflr) as
	 * one maximum value for tx, but it doesn't cover the
	 * data which is out of tx/rx fifo and inside the
	 * shift registers. So a control from sw point of
	 * view is taken.
	 */
	rxtx_gap = ((dws->rx_end -dws->rx) -(dws->tx_end -dws->tx))/dws->n_bytes;

	return min3(tx_left, tx_room, (u32) (dws->fifo_len -rxtx_gap));
}

/* Return the max entries we should read out of rx fifo */
static inline u32 rx_max (struct dw_spi *dws)
{
	u32 rx_left = (dws->rx_end - dws->rx) / dws->n_bytes;

	return min_t(u32, rx_left, dw_readl(dws, DW_SPI_RXFLR));
}

static void dw_writer (struct dw_spi *dws)
{
	u32 max = tx_max(dws);
	u16 txw = 0;

	while (max--) {
		/* Set the tx word if the transfer's original "tx" is not null */
		if (dws->tx_end - dws->len) {
			if (dws->n_bytes == 1)
				txw = *(u8 *)(dws->tx);
			else
				txw = *(u16 *)(dws->tx);
		}
		dw_write_io_reg(dws, DW_SPI_DR, txw);
		dws->tx += dws->n_bytes;
	}

	/* restart transmite, if stoped */
	dw_writel(dws, DW_SPI_SER, 1);  // !! fixme, use BIT(spi->chip_select)
}

static void dw_reader (struct dw_spi *dws)
{
	u32 max = rx_max(dws);
	u16 rxw;

	while (max--) {
		rxw = dw_read_io_reg(dws, DW_SPI_DR);
		/* Care rx only if the transfer's original "rx" is not null */
		if (dws->rx_end - dws->len) {
			if (dws->n_bytes == 1)
				*(u8 *)(dws->rx) = rxw;
			else
				*(u16 *)(dws->rx) = rxw;
		}
		dws->rx += dws->n_bytes;
	}
}

static void int_error_stop (struct dw_spi *dws, const char *msg)
{
	spi_reset_chip(dws);

	dev_err(&dws->master->dev, "%s\n", msg);
	dws->master->cur_msg->status = -EIO;
	spi_finalize_current_transfer(dws->master);
}

static irqreturn_t interrupt_transfer (struct dw_spi *dws)
{
	u16 irq_status = dw_readl(dws, DW_SPI_ISR);

	/* Error handling */
	if (irq_status & (SPI_INT_TXOI | SPI_INT_RXOI | SPI_INT_RXUI)) {
		dw_readl(dws, DW_SPI_ICR);
		int_error_stop(dws, "interrupt_transfer: fifo overrun/underrun");
		return IRQ_HANDLED;
	}

	dw_reader(dws);
	if (dws->rx_end == dws->rx) {
		spi_mask_intr(dws, SPI_INT_TXEI);
		spi_finalize_current_transfer(dws->master);
		return IRQ_HANDLED;
	}
	if (irq_status & SPI_INT_TXEI) {
		spi_mask_intr(dws, SPI_INT_TXEI);
		dw_writer(dws);
		/* Enable TX irq always, it will be disabled when RX finished */
		spi_umask_intr(dws, SPI_INT_TXEI);
	}

	return IRQ_HANDLED;
}

static irqreturn_t dw_spi_irq (int irq, void *dev_id)
{
	struct spi_controller *master = dev_id;
	struct dw_spi *dws = spi_controller_get_devdata(master);
	u16 irq_status = dw_readl(dws, DW_SPI_ISR) & 0x3f;

	if (!irq_status)
		return IRQ_NONE;

	if (!master->cur_msg) {
		spi_mask_intr(dws, SPI_INT_TXEI);
		return IRQ_HANDLED;
	}

	return dws->transfer_handler(dws);
}

/* Must be called inside pump_transfers() */
static int poll_transfer (struct dw_spi *dws)
{
	do {
		dw_writer(dws);
		dw_reader(dws);
		cpu_relax();
	} while (dws->rx_end > dws->rx);

	return 0;
}

static int dw_spi_transfer_one (struct spi_controller *master,
		struct spi_device *spi, struct spi_transfer *transfer)
{
	struct dw_spi *dws = spi_controller_get_devdata(master);
	struct chip_data *chip = spi_get_ctldata(spi);
	u8 imask = 0;
	u16 txlevel = 0;
	u32 cr0;
	int ret;

	dws->dma_mapped = 0;
	dws->tx = (void *)transfer->tx_buf;
	dws->tx_end = dws->tx + transfer->len;
	dws->rx = transfer->rx_buf;
	dws->rx_end = dws->rx + transfer->len;
	dws->len = transfer->len;

	spi_enable_chip(dws, 0);

	/* Handle per transfer options for bpw and speed */
	if (transfer->speed_hz != dws->current_freq) {
		if (transfer->speed_hz != chip->speed_hz) {
			/* clk_div doesn't support odd number */
			chip->clk_div = (DIV_ROUND_UP(dws->max_freq, transfer->speed_hz) + 1) & 0xfffe;
			chip->speed_hz = transfer->speed_hz;
		}
		dws->current_freq = transfer->speed_hz;
		spi_set_clk(dws, chip->clk_div);
	}

	dws->n_bytes = DIV_ROUND_UP(transfer->bits_per_word, BITS_PER_BYTE);
	dws->dma_width = DIV_ROUND_UP(transfer->bits_per_word, BITS_PER_BYTE);

	/* Default SPI mode is SCPOL = 0, SCPH = 0 */
	cr0 = (transfer->bits_per_word - 1)
		| (chip->type << SPI_FRF_OFFSET)
		| (((spi->mode & SPI_CPOL) ? 1 : 0) << SPI_SCOL_OFFSET)
		| (((spi->mode & SPI_CPHA) ? 1 : 0) << SPI_SCPH_OFFSET)
		| (chip->tmode << SPI_TMOD_OFFSET);

	/*
	 * Adjust transfer mode if necessary. Requires platform dependent
	 * chipselect mechanism.
	 */
	if (chip->cs_control) {
		if (dws->rx && dws->tx)
			chip->tmode = SPI_TMOD_TR;
		else if (dws->rx)
			chip->tmode = SPI_TMOD_RO;
		else
			chip->tmode = SPI_TMOD_TO;
		cr0 &= ~SPI_TMOD_MASK;
		cr0 |= (chip->tmode << SPI_TMOD_OFFSET);
	}
	dw_writel(dws, DW_SPI_CTRL0, cr0);

	/* Check if current transfer is a DMA transaction */
	if (master->can_dma && master->can_dma(master, spi, transfer))
		dws->dma_mapped = master->cur_msg_mapped;

	/* For poll mode just disable all interrupts */
	spi_mask_intr(dws, 0xff);

	/*
	 * Interrupt mode
	 * we only need set the TXEI IRQ, as TX/RX always happen syncronizely
	 */
	if (dws->dma_mapped) {
		ret = dws->dma_ops->dma_setup(dws, transfer);
		if (ret < 0) {
			spi_enable_chip(dws, 1);
			return ret;
		}
	} else if (!chip->poll_mode) {
		txlevel = min_t(u16, dws->fifo_len / 2, dws->len / dws->n_bytes);
		dw_writel(dws, DW_SPI_TXFLTR, txlevel);

		/* Set the interrupt mask */
		imask |= SPI_INT_TXEI | SPI_INT_TXOI | SPI_INT_RXUI | SPI_INT_RXOI;
		spi_umask_intr(dws, imask);
		dws->transfer_handler = interrupt_transfer;
	}
	spi_enable_chip(dws, 1);
	if (dws->dma_mapped) {
		ret = dws->dma_ops->dma_transfer(dws, transfer);
		if (ret < 0)
			return ret;
	}
	if (chip->poll_mode)
		return poll_transfer(dws);
	return 1;
}

static void dw_spi_handle_err (struct spi_controller *master,
		struct spi_message *msg)
{
	struct dw_spi *dws = spi_controller_get_devdata(master);
	if (dws->dma_mapped)
		dws->dma_ops->dma_stop(dws);
	spi_reset_chip(dws);
}

/* This may be called twice for each spi dev */
static int dw_spi_setup (struct spi_device *spi)
{
	struct dw_spi_chip *chip_info = NULL;
	struct chip_data *chip;

	/* Only alloc on first setup */
	chip = spi_get_ctldata(spi);
	if (!chip) {
		chip = kzalloc(sizeof(struct chip_data), GFP_KERNEL);
		if (!chip)
			return -ENOMEM;
		spi_set_ctldata(spi, chip);
	}

	/*
	 * Protocol drivers may change the chip settings, so...
	 * if chip_info exists, use it
	 */
	chip_info = spi->controller_data;

	/* chip_info doesn't always exist */
	if (chip_info) {
		if (chip_info->cs_control)
			chip->cs_control = chip_info->cs_control;
		chip->poll_mode = chip_info->poll_mode;
		chip->type = chip_info->type;
	}
	chip->tmode = SPI_TMOD_TR;
	return 0;
}

static void dw_spi_cleanup (struct spi_device *spi)
{
	struct chip_data *chip = spi_get_ctldata(spi);
	kfree(chip);
	spi_set_ctldata(spi, NULL);
}

/* Restart the controller, disable all interrupts, clean rx fifo */
static void spi_hw_init (struct device *dev, struct dw_spi *dws)
{
	spi_reset_chip(dws);

	/*
	 * Try to detect the FIFO depth if not set by interface driver,
	 * the depth could be from 2 to 256 from HW spec
	 */
	if (!dws->fifo_len) {
		u32 fifo;
		for (fifo = 1; fifo < 256; fifo++) {
			dw_writel(dws, DW_SPI_TXFLTR, fifo);
			if (fifo != dw_readl(dws, DW_SPI_TXFLTR))
				break;
		}
		dw_writel(dws, DW_SPI_TXFLTR, 0);
		dws->fifo_len = (fifo == 1) ? 0 : fifo;
		dws->fifo_len /= 4;
		dev_dbg(dev, "Detected FIFO size: %u bytes\n", dws->fifo_len);
	}

	/* enable HW fixup for explicit CS deselect for Amazon's alpine chip */
	if (dws->cs_override)
		dw_writel(dws, DW_SPI_CS_OVERRIDE, 0xF);
}

int dw_spi_baikal_add_host (struct device *dev, struct dw_spi *dws)
{
	struct spi_controller *master;
	int ret;

	BUG_ON(dws == NULL);

	master = spi_alloc_master(dev, 0);
	if (!master)
		return -ENOMEM;

	dws->master = master;
	dws->type = SSI_MOTO_SPI;
	dws->dma_inited = 0;
	dws->dma_addr = (dma_addr_t)(dws->paddr + DW_SPI_DR);
	spi_controller_set_devdata(master, dws);

	ret = request_irq(dws->irq, dw_spi_irq, IRQF_SHARED, dev_name(dev),
			  master);
	if (ret < 0) {
		dev_err(dev, "can not get IRQ\n");
		goto err_free_master;
	}

	master->use_gpio_descriptors = true;
	master->mode_bits = SPI_CPOL | SPI_CPHA | SPI_LOOP;
	master->bits_per_word_mask =  SPI_BPW_RANGE_MASK(4, 16);
	master->bus_num = dws->bus_num;
	master->num_chipselect = dws->num_cs;
	master->setup = dw_spi_setup;
	master->cleanup = dw_spi_cleanup;
	master->transfer_one = dw_spi_transfer_one;
	master->handle_err = dw_spi_handle_err;
	master->max_speed_hz = dws->max_freq;
	master->dev.of_node = dev->of_node;
	master->dev.fwnode = dev->fwnode;

	if (dws->set_cs)
		master->set_cs = dws->set_cs;

	/* Basic HW init */
	spi_hw_init(dev, dws);

	if (dws->dma_ops && dws->dma_ops->dma_init) {
		ret = dws->dma_ops->dma_init(dws);
		if (ret) {
			dev_warn(dev, "DMA init failed\n");
			dws->dma_inited = 0;
		} else
			master->can_dma = dws->dma_ops->can_dma;
	}

	ret = devm_spi_register_controller(dev, master);
	if (ret) {
		dev_err(&master->dev, "problem registering spi master\n");
		goto err_dma_exit;
	}
	return 0;

err_dma_exit:
	if (dws->dma_ops && dws->dma_ops->dma_exit)
		dws->dma_ops->dma_exit(dws);
	spi_enable_chip(dws, 0);
	free_irq(dws->irq, master);
err_free_master:
	spi_controller_put(master);
	return ret;
}

void dw_spi_baikal_remove_host (struct dw_spi *dws)
{
	if (dws->dma_ops && dws->dma_ops->dma_exit)
		dws->dma_ops->dma_exit(dws);
	spi_shutdown_chip(dws);
	free_irq(dws->irq, dws->master);
}


/*
----------------------
MMIO
----------------------
*/

struct dw_spi_mmio {
	struct dw_spi  dws;
	struct clk     *clk;
	void           *priv;
};

static int probe (struct platform_device *pdev)
{
	int (*init_func)(struct platform_device *pdev,
			 struct dw_spi_mmio *dwsmmio);
	struct dw_spi_mmio *dwsmmio;
	struct dw_spi *dws;
	struct resource *mem;
	int ret;
	int num_cs;

	dwsmmio = devm_kzalloc(&pdev->dev, sizeof(struct dw_spi_mmio), GFP_KERNEL);
	if (!dwsmmio)
		return -ENOMEM;

	dws = &dwsmmio->dws;

	/* Get basic io resource and map it */
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	dws->regs = devm_ioremap_resource(&pdev->dev, mem);
	if (IS_ERR(dws->regs)) {
		dev_err(&pdev->dev, "SPI region map failed\n");
		return PTR_ERR(dws->regs);
	}

	dws->irq = platform_get_irq(pdev, 0);
	if (dws->irq < 0) {
		dev_err(&pdev->dev, "no irq resource?\n");
		return dws->irq; /* -ENXIO */
	}

	dwsmmio->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(dwsmmio->clk))
		return PTR_ERR(dwsmmio->clk);
	ret = clk_prepare_enable(dwsmmio->clk);
	if (ret)
		return ret;

	dws->bus_num = pdev->id;
	dws->max_freq = clk_get_rate(dwsmmio->clk);
	num_cs = 4;
	device_property_read_u32(&pdev->dev, "num-cs", &num_cs);
	dws->num_cs = num_cs;

	init_func = device_get_match_data(&pdev->dev);
	if (init_func) {
		ret = init_func(pdev, dwsmmio);
		if (ret)
			goto out;
	}

	ret = dw_spi_baikal_add_host(&pdev->dev, dws);
	if (ret)
		goto out;

	pdev->dev.dma_mask = NULL;

	platform_set_drvdata(pdev, dwsmmio);
	return 0;
out:
	clk_disable_unprepare(dwsmmio->clk);
	return ret;
}

static int remove (struct platform_device *pdev)
{
	struct dw_spi_mmio *dwsmmio = platform_get_drvdata(pdev);
	dw_spi_baikal_remove_host(&dwsmmio->dws);
	clk_disable_unprepare(dwsmmio->clk);
	return 0;
}

static const struct of_device_id  be_spi_table[] = {
	{ .compatible = "be,dw-spi", },
	{ /* end of table */}
};
MODULE_DEVICE_TABLE(of, be_spi_table);

static struct platform_driver   be_spi_driver = {
	.probe      = probe,
	.remove     = remove,
	.driver     = {
		.name   = "be,dw-spi",
		.of_match_table = be_spi_table,
	},
};
module_platform_driver(be_spi_driver);

MODULE_DESCRIPTION("Baikal Electronics Spi Flash Driver");
MODULE_LICENSE("Dual BSD/GPL");
