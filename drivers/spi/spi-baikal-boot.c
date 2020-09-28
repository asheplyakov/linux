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
#include <linux/spinlock.h>
#include "spi-dw.h"


struct dw_boot_spi {
	void __iomem *regs;
	unsigned long paddr;

	u32 fifo_len;       /* depth of the FIFO buffer */
	u32 max_freq;       /* max bus freq supported */
	u32 reg_io_width;   /* DR I/O width in bytes */
	u32 bus_num;
	u32 num_cs;         /* supported slave numbers */

	char tx[512];

	struct clk *clk;
};

static inline u32 dw_boot_readl (struct dw_boot_spi *dws, u32 offset)
{
	return __raw_readl(dws->regs + offset);
}

static inline void dw_boot_writel (struct dw_boot_spi *dws, u32 offset, u32 val)
{
	__raw_writel(val, dws->regs + offset);
}

static void spi_set_mode (struct dw_boot_spi *dws, int mode)
{
	struct {
		uint32_t dfs    :4; /* data frame size */
		uint32_t frf    :2; /* frame format (0-spi, 1-ssp, 2-micro, 3-reserved) */
		uint32_t scph   :1; /* clk phase */
		uint32_t scpol  :1; /* clk polarity */
		uint32_t tmod   :2; /* transfer mode (0-tx|rx, 1-tx, 2-rx, 3-eeprom) */
		uint32_t slv_oe :1; /* (ignore) slave output enable */
		uint32_t srl    :1; /* (ignore) shift register loop */
		uint32_t cfs    :4; /* (ignore) control frame size */
		uint32_t _      :16;
	} ctr0;

	*(u32*)&ctr0 = dw_boot_readl (dws, DW_SPI_CTRL0);
	ctr0.tmod = mode;
	dw_boot_writel(dws, DW_SPI_CTRL0, *(u32*)&ctr0);
}


static int boot_spi_write (struct spi_master *master, int chip_select,
	const uint8_t* tx1, const uint8_t* tx2, int len1, int len2)
{
	struct dw_boot_spi *dws;
	int i, n1, n2;
	const uint8_t* end1 = tx1 + len1;
	const uint8_t* end2 = tx2 + len2;
	unsigned long flags;


	DEFINE_SPINLOCK(mLock);
	spin_lock_irqsave(&mLock, flags);   /* Critical section - ON */

	dws = spi_master_get_devdata(master);

	dw_boot_writel(dws, DW_SPI_SER, 0);
	dw_boot_writel(dws, DW_SPI_SSIENR, 0);
	spi_set_mode(dws, SPI_TMOD_TO);

	dw_boot_writel(dws, DW_SPI_SSIENR, 1);  /* ebable fifo */

	n1 = (len1 > dws->fifo_len    )? dws->fifo_len     : len1;  /* fill fifo */
	n2 = (len2 > dws->fifo_len -n1)? dws->fifo_len -n1 : len2;
	for (i = 0; i < n1; i++)
		dw_boot_writel(dws, DW_SPI_DR, *tx1++);
	for (i = 0; i < n2; i++)
		dw_boot_writel(dws, DW_SPI_DR, *tx2++);

	dw_boot_writel(dws, DW_SPI_SER, chip_select);   /* start sending */

	while (tx1 != end1) {   /* regular transfer 1 */
		if(dw_boot_readl(dws, DW_SPI_SR) & SR_TF_NOT_FULL)
			dw_boot_writel(dws, DW_SPI_DR, *tx1++);
	}

	while (tx2 != end2) {   /* regular transfer 2 */
		if(dw_boot_readl(dws, DW_SPI_SR) & SR_TF_NOT_FULL)
			dw_boot_writel(dws, DW_SPI_DR, *tx2++);
	}

	while(!(dw_boot_readl(dws, DW_SPI_SR) & SR_BUSY))   /* wait */
		;

	spin_unlock_irqrestore(&mLock, flags);  /* Critical section - OFF */

	udelay(10);

	return 0;
}

static int boot_spi_read (struct spi_master *master, int chip_select,
	const uint8_t* tx, uint8_t* rx, int lentx, int lenrx)
{
	int i;
	uint8_t* const rxend = rx + lenrx;
	struct dw_boot_spi *dws;
	unsigned long flags;

	DEFINE_SPINLOCK(mLock);
	spin_lock_irqsave(&mLock, flags);                   /* Critical section - ON */

	dws = spi_master_get_devdata(master);

	dw_boot_writel(dws, DW_SPI_SER, 0);
	dw_boot_writel(dws, DW_SPI_SSIENR, 0);
	spi_set_mode(dws, SPI_TMOD_EPROMREAD);

	dw_boot_writel(dws, DW_SPI_CTRL1, lenrx - 1);       /* rx config */
	dw_boot_writel(dws, DW_SPI_SSIENR, 1);              /* ebable fifo */

	for (i = 0; i < lentx; i++)                         /* fill config */
		dw_boot_writel(dws, DW_SPI_DR, tx[i]);

	dw_boot_writel(dws, DW_SPI_SER, chip_select);       /* start sending */

	while (rx != rxend) {                               /* read incoming data */
		if(dw_boot_readl(dws, DW_SPI_SR) & SR_RF_NOT_EMPT)
			*rx++ = dw_boot_readl(dws, DW_SPI_DR);
	}
	spin_unlock_irqrestore(&mLock, flags);              /* Critical section - OFF */

	return 0;
}

static int boot_spi_transfer_one_message (struct spi_master *master, struct spi_message *msg)
{
	struct list_head *const head = &msg->transfers;
	struct spi_transfer *pos, *next;
	int select = BIT(msg->spi->chip_select);
	int err = 0;
	int i;

	char *rx = NULL;
	int rx_len = 0;
	int tx_len = 0;

	struct dw_boot_spi *dws = spi_master_get_devdata(master);

	/* decode */
	list_for_each_entry(pos, head, transfer_list)
	{
		if (pos->tx_buf) {
			for (i=0; i<pos->len; ++i)
				dws->tx[tx_len+i] = ((char*)pos->tx_buf)[i];

			tx_len += pos->len;
			if(tx_len > sizeof(dws->tx)){
				err = -2;
				goto exit;
			}
		}

		if (pos->rx_buf) {
			if (rx) {
				err = -3;
				goto exit;
			}
			rx = pos->rx_buf;
			rx_len += pos->len;
		}
	}
	msg->actual_length += tx_len + rx_len;

	/* send */
	if(rx)
		boot_spi_read  (master, select, dws->tx, rx,   tx_len, rx_len);
	else
		boot_spi_write (master, select, dws->tx, NULL, tx_len, 0);



exit:
	msg->status = err;
	spi_finalize_current_message(master);
	if(err)
		dev_err(&master->dev, "-- error %d\n", err);

	return err;
}

static int fifo_len (struct dw_boot_spi *dws)
{
	u32 txfltr = dw_boot_readl(dws, DW_SPI_TXFLTR);
	u32 fifo;
	for (fifo = 1; fifo < 256; fifo++) {
		dw_boot_writel(dws, DW_SPI_TXFLTR, fifo);
		if (fifo != dw_boot_readl(dws, DW_SPI_TXFLTR))
			break;
	}
	dw_boot_writel(dws, DW_SPI_TXFLTR, txfltr);

	return (fifo == 1) ? 0:fifo;
}

static void init (struct dw_boot_spi *dws)
{
	/* clear */
	int i;
	for (i = 0; i < DW_SPI_DR; i+= sizeof(uint32_t))
		dw_boot_writel(dws, i, 0);

	/* baudr */
	dw_boot_writel(dws, DW_SPI_BAUDR, 6);   /* todo: use dws->clk to init baudrate */
}

static int add_host (struct device *dev, struct dw_boot_spi *dws)
{
	struct spi_master *master;
	int ret;

	master = spi_alloc_master(dev, 0);
	if (!master){
		dev_err(&master->dev, "-- alloc\n");
		return -ENOMEM;
	}

	master->bus_num                 = dws->bus_num;
	master->num_chipselect          = dws->num_cs;
	master->mode_bits               = SPI_CPOL | SPI_CPHA | SPI_LOOP;
	master->bits_per_word_mask      = SPI_BPW_MASK(8) | SPI_BPW_MASK(16);
	master->max_speed_hz            = dws->max_freq;
	master->dev.of_node             = dev->of_node;
	master->transfer_one_message    = boot_spi_transfer_one_message;

	spi_master_set_devdata(master, dws);
	ret = devm_spi_register_master(dev, master);
	if (ret) {
		dev_err(&master->dev, "-- problem registering spi master\n");
		spi_master_put(master);
	}
	return ret;
}

static int probe (struct platform_device *pdev)
{
	struct dw_boot_spi *dws;
	struct resource *mem;
	int ret;

	/* alloc dws */
	dws = devm_kzalloc(&pdev->dev, sizeof(struct dw_boot_spi), GFP_KERNEL);
	if (!dws) {
		dev_err(&pdev->dev, "-- alloc\n");
		return -ENOMEM;
	}

	/* Get basic io resource and map it */
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem) {
		dev_err(&pdev->dev, "-- get resource?\n");
		return -EINVAL;
	}

	dws->regs = devm_ioremap_resource(&pdev->dev, mem);
	if (IS_ERR(dws->regs)) {
		dev_err(&pdev->dev, "-- ioremap\n");
		return PTR_ERR(dws->regs);
	}

	dws->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(dws->clk)) {
		dev_err(&pdev->dev, "-- clk get\n");
		return PTR_ERR(dws->clk);
	}

	ret = clk_prepare_enable(dws->clk);
	if (ret) {
		dev_err(&pdev->dev, "-- clk prepare\n");
		return ret;
	}

	/* get spi parameters */
	dws->bus_num = of_alias_get_id(pdev->dev.of_node, "ssi");
	dws->max_freq = clk_get_rate(dws->clk);
	device_property_read_u32(&pdev->dev, "num-cs", &dws->num_cs);
	dws->fifo_len = fifo_len(dws);

	init(dws);

	/* add host */
	ret = add_host(&pdev->dev, dws);
	if (ret){
		clk_disable_unprepare(dws->clk);
		dev_err(&pdev->dev, "-- add_host\n");
		return ret;
	}
	platform_set_drvdata(pdev, dws);

	return 0;
}

static int remove (struct platform_device *pdev)
{
	struct dw_boot_spi *dws = platform_get_drvdata(pdev);
	clk_disable_unprepare(dws->clk);
	dw_boot_writel(dws, DW_SPI_SSIENR, 0);
	dw_boot_writel(dws, DW_SPI_BAUDR, 0);
	return 0;
}

static const struct of_device_id   be_spi_boot_table[] = {
	{ .compatible = "be,dw-spi-boot", },
	{ /* end of table */}
};
MODULE_DEVICE_TABLE(of, be_spi_boot_table);

static struct platform_driver be_spi_boot_driver = {
	.probe      = probe,
	.remove     = remove,
	.driver     = {
		.name   = "be,dw-spi-boot",
		.of_match_table = be_spi_boot_table,
	},
};
module_platform_driver(be_spi_boot_driver);

MODULE_DESCRIPTION("Baikal Electronics Spi Flash Driver");
MODULE_LICENSE("Dual BSD/GPL");
