/*
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

#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/highmem.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/gpio.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/scatterlist.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <linux/property.h>
#include <linux/io.h>

#include "spi-db-espi.h"


/**
 * ----------------
 * spi_reset_master
 * ----------------
 * @param spi    [description]
 * @param enable [description]
 */
static void spi_reset_master (struct spi_master *master)
{
	struct drv_data *dws = spi_master_get_devdata(master);

	/* reset (clk,irq,fifo) */
	/* NORESET -> RESET */
	espi_cr1_t cr1;
	cr1.val = 0;
	cr1.bits.scr = ESPI_CR1_SCR_NORESET;
	dw_writel(dws, ESPI_CR1, cr1.val);
	cr1.bits.scr = ESPI_CR1_SCR_RESET;
	dw_writel(dws, ESPI_CR1, cr1.val);

	/* clear registers */
	dw_writel(dws, ESPI_CR2, 0);
	dw_writel(dws, ESPI_TX_FAETR, ESPI_TX_FAETR_DISABLE);
	dw_writel(dws, ESPI_RX_FAFTR, ESPI_RX_FAFTR_DISABLE);
	dw_writel(dws, ESPI_ISR, ESPI_ISR_CLEAR);
	dw_writel(dws, ESPI_IMR, ESPI_IMR_DISABLE);
	dw_writel(dws, ESPI_CR3, 0);    /* disable espi functions */
	dw_writel(dws, ESPI_RBCR, 0xFF);
}

/**
 * -----------
 * spi_set_cs
 * -----------
 * @param spi    [description]
 * @param enable [description]
 */
static void spi_set_cs (struct spi_device *spi, bool enable)
{
	struct spi_master *master;
	struct drv_data *dws;
	espi_cr2_t cr2;

	master = spi->master;
	dws = spi_master_get_devdata(master);

	/* check */
	if (spi->chip_select > master->num_chipselect){
		dev_err(&master->dev, "chip-select too big. (cs=%d, max=%d)\n",
			spi->chip_select, master->num_chipselect-1);
		return;
	}

	/* set */
	cr2.val = dw_readl(dws, ESPI_CR2);
	cr2.bits.sso = spi->chip_select;
	dw_writel(dws, ESPI_CR2, cr2.val);
}

/**
 * -----------
 * spi_writer
 * -----------
 * @param dws [description]
 */
static void spi_writer (struct drv_data *dws)
{
	volatile uint8_t data = ESPI_DUMMY_DATA;
	uint32_t free = dws->fifo_len - dw_readl(dws, ESPI_TX_FBCAR);
	uint32_t cnt = min(free,dws->len);

	while (cnt--) {
		if(dws->tx)
			data = *dws->tx++;
		dw_writel(dws, ESPI_TX_FIFO, data);
		dws->len--;
	}
}

/**
 * ----------
 * spi_reader
 * ----------
 * @param dws [description]
 */
static void spi_reader (struct drv_data *dws)
{
	volatile uint8_t data;
	uint32_t cnt = dw_readl(dws, ESPI_RX_FBCAR);

	while (cnt--) {
		data = dw_readl(dws, ESPI_RX_FIFO);
		if (dws->rx)
			*dws->rx++ = data;
	}
}

/**
 * ----------
 * spi_irq_handler
 * ----------
 * @param  irq    [description]
 * @param  dev_id [description]
 * @return        [description]
 */
static irqreturn_t spi_irq_handler (int irq, void *dev_id)
{
	struct spi_master *master;
	struct drv_data *dws;
	espi_irq_t status;

	master = dev_id;
	dws = spi_master_get_devdata(master);

	/* irq */
	status.val = dw_readl(dws, ESPI_ISR);
	dw_writel(dws, ESPI_ISR, ESPI_ISR_CLEAR);
	dw_writel(dws, ESPI_IMR, ESPI_IMR_DISABLE);

	/* -------- */
	/* check    */
	/* -------- */
	if (!status.val) {
		return IRQ_NONE;
	}
	if (!master->cur_msg) {
		spi_reset_master(master);
		return IRQ_HANDLED;
	}
	if (status.bits.tx_underflow && dws->len) {
		dev_err(&master->dev, "error: fifo underflow\n");
		goto err;
	}
	if (status.bits.tx_overrun |
		status.bits.rx_underrun |
		status.bits.rx_overrun) {
		dev_err(&master->dev, "error: fifo overrun/underrun\n");
		goto err;
	}

	/* -------- */
	/* transfer */
	/* -------- */
	spi_reader(dws);
	if (!dws->len) {
		spi_finalize_current_transfer(master);
		return IRQ_HANDLED;
	}
	spi_writer(dws);

	/* irq */
	dw_writel(dws, ESPI_IMR, status.val);
	return IRQ_HANDLED;


	/* -------- */
	/* err      */
	/* -------- */
	err:
		spi_reset_master(master);
		master->cur_msg->status = -EIO;
		spi_finalize_current_transfer(master);
		return IRQ_HANDLED;
}


/**
 * ------------------
 * spi_transfer_one
 * ------------------
 * @param  master   [description]
 * @param  spi      [description]
 * @param  transfer [description]
 * @return          [description]
 */
static int spi_transfer_one(
		struct spi_master   *master,
		struct spi_device   *spi,
		struct spi_transfer *transfer)
{
	struct drv_data *dws = spi_master_get_devdata(spi->master);
	espi_cr1_t cr1;
	espi_cr2_t cr2;
	espi_irq_t mask;

	/* --------- */
	/* transfer  */
	/* --------- */
	dws->tx  = (void*) transfer->tx_buf;
	dws->rx  = (void*) transfer->rx_buf;
	dws->len = transfer->len;


	/* ---------- */
	/* control    */
	/* ---------- */
	cr1.val = dw_readl(dws, ESPI_CR1);
	cr1.bits.scr = ESPI_CR1_SCR_NORESET;
	cr1.bits.sce = ESPI_CR1_SCE_CORE_ENABLE;
	cr1.bits.mss = ESPI_CR1_MSS_MODE_MASTER;
	cr1.bits.cph = (spi->mode|SPI_CPHA)? ESPI_CR1_CPH_CLKPHASE_ODD : ESPI_CR1_CPH_CLKPHASE_EVEN;
	cr1.bits.cpo = (spi->mode|SPI_CPOL)? ESPI_CR1_CPO_CLKPOLAR_LOW : ESPI_CR1_CPO_CLKPOLAR_HIGH;
	dw_writel(dws, ESPI_CR1, cr1.val);

	cr2.val = dw_readl(dws, ESPI_CR2);
	cr2.bits.srd = ESPI_CR2_SRD_RX_ENABLE;
	cr2.bits.mte = ESPI_CR2_MTE_TX_DISABLE;
	cr2.bits.sri = ESPI_CR2_SRI_FIRST_RESIEV;
	cr2.bits.mlb = (spi->mode|SPI_LSB_FIRST)? ESPI_CR2_MLB_LSB : ESPI_CR2_MLB_MSB;
	dw_writel(dws, ESPI_CR2, cr2.val);


	/* ---------- */
	/* threshold  */
	/* ---------- */
	dw_writel(dws, ESPI_TX_FAETR, dws->tx_almost_empty);
	dw_writel(dws, ESPI_RX_FAFTR, dws->rx_almost_full);


	/* ---------- */
	/* interrupts */
	/* ---------- */
	mask.val = 0;
	/* errors */
	mask.bits.tx_overrun      = ESPI_IRQ_ENABLE;
	mask.bits.rx_overrun      = ESPI_IRQ_ENABLE;
	mask.bits.rx_underrun     = ESPI_IRQ_ENABLE;
	/* transfer */
	mask.bits.tx_underflow    = ESPI_IRQ_ENABLE;
	mask.bits.tx_almost_empty = ESPI_IRQ_ENABLE;
	mask.bits.rx_almost_full  = ESPI_IRQ_ENABLE;
	dw_writel(dws, ESPI_IMR, mask.val);


	/* ------------ */
	/* prepare data */
	/* ------------ */
	spi_writer(dws);


	/* -------------- */
	/* start transfer */
	/* -------------- */
	cr2.val = dw_readl(dws, ESPI_CR2);
	cr2.bits.mte = ESPI_CR2_MTE_TX_ENABLE;
	dw_writel(dws, ESPI_CR2, cr2.val);

	return 0;
}


/**
 * ---------------------
 * driver
 * ---------------------
 * @param  pdev [description]
 * @return      [description]
 */
static int driver (struct platform_device *pdev)
{
	struct device *dev = &(pdev->dev);
	struct drv_data *dws;
	struct resource *mem;

	/* init */
	dws = devm_kzalloc(dev, sizeof(struct drv_data), GFP_KERNEL);
	if (!dws){
		dev_err(dev, "no memory\n");
		return -ENOMEM;
	}
	platform_set_drvdata(pdev, dws);

	/* reg */
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	dws->regs = devm_ioremap_resource(dev, mem);
	if (IS_ERR(dws->regs)) {
		dev_err(dev, "SPI region map failed\n");
		return PTR_ERR(dws->regs);
	}

	/* irq */
	dws->irq = platform_get_irq(pdev, 0);
	if (dws->irq < 0) {
		dev_err(&pdev->dev, "no irq resource\n");
		return dws->irq;
	}

	/* fifo-len */
	dws->fifo_len        = ESPI_FIFO_LEN;
	dws->tx_almost_empty = dws->fifo_len/2;
	dws->rx_almost_full  = dws->fifo_len/2;
	device_property_read_u32(dev, "fifo-len",        &dws->fifo_len);
	device_property_read_u32(dev, "tx-almost-empty", &dws->tx_almost_empty);
	device_property_read_u32(dev, "rx-almost-full",  &dws->rx_almost_full);

	return 0;
}


/**
 * ---------------------
 * master
 * ---------------------
 * @param  pdev [description]
 * @return      [description]
 */
static int master (struct platform_device *pdev)
{
	struct spi_master *master;
	struct device *dev = &pdev->dev;
	struct drv_data *dws = platform_get_drvdata(pdev);
	int ret;

	/* init */
	master = spi_alloc_master(dev, 0);
	if (!master)
		return -ENOMEM;
	dws->master = master;

	/* irq */
	ret = request_irq(dws->irq, spi_irq_handler, IRQF_SHARED, DRIVER_NAME, master);
	if (ret < 0) {
		dev_err(dev, "can not get IRQ\n");
		goto err_free_master;
	}

	/* property */
	master->num_chipselect = 8;
	master->min_speed_hz = ESPI_FREQUENCY;
	master->max_speed_hz = ESPI_FREQUENCY;
	device_property_read_u32(dev, "spi-min-frequency", &master->min_speed_hz);
	device_property_read_u32(dev, "spi-max-frequency", &master->max_speed_hz);
	device_property_read_u16(dev, "num-cs",            &master->num_chipselect);
	master->mode_bits = SPI_CPHA | SPI_CPOL | SPI_LSB_FIRST;

	/* operation */
	master->transfer_one  = spi_transfer_one;
	master->set_cs        = spi_set_cs;

	spi_reset_master(master);

	/* register */
	master->dev.of_node = dev->of_node;
	ret = devm_spi_register_master(dev, master);
	if (ret) {
		dev_err(&master->dev, "problem registering spi master\n");
		goto err_free_irq;
	}
	return 0;


err_free_irq:
	free_irq(dws->irq, master);
err_free_master:
	spi_master_put(master);
	return ret;
}


/**
 * ----------------
 * probe
 * ----------------
 * @param  pdev [description]
 * @return      [description]
 */
static int probe (struct platform_device *pdev)
{
	int ret;

	ret = driver(pdev);
	if (ret)
		return ret;

	ret = master(pdev);
	if (ret)
		return ret;

	return 0;
}


/**
 * ----------------
 * remove host
 * ----------------
 * @param  pdev [description]
 * @return      [description]
 */
static int remove (struct platform_device *pdev)
{
	struct drv_data *dws = platform_get_drvdata(pdev);
	spi_reset_master(dws->master);
	free_irq(dws->irq, dws->master);
	return 0;
}


/* ---------------- */
/* PLATFORM         */
/* ---------------- */
static const struct of_device_id dw_spi_mmio_of_match[] = {
	{ .compatible = DRIVER_NAME, },
	{ /* end of table */ }
};
MODULE_DEVICE_TABLE(of, dw_spi_mmio_of_match);

static struct platform_driver dw_spi_mmio_driver = {
	.probe		= probe,
	.remove		= remove,
	.driver		= {
		.name	= DRIVER_NAME,
		.of_match_table = dw_spi_mmio_of_match,
	},
};
module_platform_driver(dw_spi_mmio_driver);

MODULE_AUTHOR("");
MODULE_DESCRIPTION("");
MODULE_LICENSE("GPL v2");




