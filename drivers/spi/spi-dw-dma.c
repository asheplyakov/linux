
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/types.h>

#include "spi-dw.h"

#include <linux/platform_data/dma-dw.h>
#include <linux/delay.h>


#define RX_BUSY     0
#define TX_BUSY     1


static int spi_check_status (struct dw_spi *dws)
{
	uint8_t sr = dw_readl(dws, DW_SPI_SR);
	uint8_t busy = (sr & (1 << 0));    /* SSI Busy Flag. */
	return busy;
}

static irqreturn_t transfer_handler (struct dw_spi *dws)
{
	u16 irq_status = dw_readl(dws, DW_SPI_ISR);
	if (!irq_status)
		return IRQ_NONE;

	dw_readl(dws, DW_SPI_ICR);
	spi_reset_chip(dws);

	dev_err(&dws->master->dev, "%s: FIFO overrun/underrun\n", __func__);
	dws->master->cur_msg->status = -EIO;

	spi_finalize_current_transfer(dws->master);
	return IRQ_HANDLED;
}

static enum dma_slave_buswidth __maybe_unused convert_dma_width (u32 dma_width)
{
	if (dma_width == 1)
		return DMA_SLAVE_BUSWIDTH_1_BYTE;
	else if (dma_width == 2)
		return DMA_SLAVE_BUSWIDTH_2_BYTES;
	return DMA_SLAVE_BUSWIDTH_UNDEFINED;
}

static void spi_wait_status(struct dw_spi *dws)
{
	/* wait */
	int try = 100000;
	while(spi_check_status(dws) && try--){
		udelay(1);
	}
}

static void tx_done (void *arg)
{
	/* dws->dma_chan_busy is set before the dma transfer starts,
	 callback for tx channel will clear a corresponding bit. */

	struct dw_spi *dws = arg;
	spi_wait_status(dws);
	clear_bit(TX_BUSY, &dws->dma_chan_busy);
	if (test_bit(RX_BUSY, &dws->dma_chan_busy))
		return;
	spi_finalize_current_transfer(dws->master);
}

static void rx_done (void *arg)
{
	/* dws->dma_chan_busy is set before the dma transfer starts,
	callback for rx channel will clear a corresponding bit. */

	struct dw_spi *dws = arg;
	spi_wait_status(dws);
	clear_bit(RX_BUSY, &dws->dma_chan_busy);
	if (test_bit(TX_BUSY, &dws->dma_chan_busy))
		return;
	spi_finalize_current_transfer(dws->master);
}

static struct dma_async_tx_descriptor *prepare_tx (
	struct dw_spi       *dws,
	struct spi_transfer *xfer)
{
	struct dma_async_tx_descriptor *desc;
	struct dma_slave_config config = {0};
	int ret;

	if (!xfer->tx_buf)
		return NULL;

	/* slave config */
	config.direction      = DMA_MEM_TO_DEV;
	config.device_fc      = false;
	config.src_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	config.dst_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	config.dst_maxburst   = dws->fifo_len/2;
	config.dst_addr       = dws->dma_addr;

	ret = dmaengine_slave_config(dws->txchan, &config);
	if (ret) {
		dev_err(&dws->master->dev, "Failed to set DMA config\n");
		return NULL;
	}

	/* descriptor */
	desc = dmaengine_prep_slave_sg(dws->txchan,
		xfer->tx_sg.sgl, xfer->tx_sg.nents,
		DMA_MEM_TO_DEV, DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!desc) {
		dev_err(&dws->master->dev, "Failed to prepare DMA-slave\n");
		return NULL;
	}

	/* callback */
	desc->callback = tx_done;
	desc->callback_param = dws;

	return desc;
}

static struct dma_async_tx_descriptor *prepare_rx (
	struct dw_spi       *dws,
	struct spi_transfer *xfer)
{
	struct dma_async_tx_descriptor *desc;
	struct dma_slave_config config = {0};
	int ret;

	if (!xfer->rx_buf)
		return NULL;

	/* slave config */
	config.direction      = DMA_DEV_TO_MEM;
	config.device_fc      = false;
	config.src_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	config.dst_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	config.src_maxburst   = dws->fifo_len/2;
	config.src_addr       = dws->dma_addr;

	ret = dmaengine_slave_config(dws->rxchan, &config);
	if (ret) {
		dev_err(&dws->master->dev, "Failed to set DMA config\n");
		return NULL;
	}

	/* descriptor */
	desc = dmaengine_prep_slave_sg(dws->rxchan,
		xfer->rx_sg.sgl, xfer->rx_sg.nents,
		DMA_DEV_TO_MEM, DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!desc) {
		dev_err(&dws->master->dev, "Failed to prepare DMA-slave\n");
		return NULL;
	}

	/* callback */
	desc->callback = rx_done;
	desc->callback_param = dws;

	return desc;
}

static int init (struct dw_spi *dws)
{
	struct device *dev = &(dws->master->dev);

	dws->rxchan = dma_request_slave_channel(dev, "rx");
	dws->txchan = dma_request_slave_channel(dev, "tx");

	if (!dws->rxchan || !dws->txchan){
		if(dws->txchan)
			dma_release_channel(dws->txchan);
		if(dws->rxchan)
			dma_release_channel(dws->rxchan);
		return -EBUSY;
	}

	dws->master->dma_rx = dws->rxchan;
	dws->master->dma_tx = dws->txchan;
	dws->transfer_handler = transfer_handler;

	dws->dma_inited = 1;

	return 0;
}

static void exit (struct dw_spi *dws)
{
	if (!dws->dma_inited)
		return;

	dmaengine_terminate_all(dws->txchan);
	dmaengine_terminate_all(dws->rxchan);

	dma_release_channel(dws->txchan);
	dma_release_channel(dws->rxchan);
}

static bool can (
	struct spi_master *master,
	struct spi_device *spi,
	struct spi_transfer *xfer)
{
	/*
	// todo: use this
	struct dw_spi *dws = spi_master_get_devdata(master);
	return (dws->dma_inited) && (xfer->len > 256);
	*/

	return 1;
}

static int setup (struct dw_spi *dws, struct spi_transfer *xfer)
{
	u16 dma_ctrl = 0;
	u32 cr0;
	u32 tmode;

	/* clear */
	// todo: dont clean all regs
	dw_writel(dws, DW_SPI_IMR,    0);
	dw_writel(dws, DW_SPI_RXFLTR, 0);
	dw_writel(dws, DW_SPI_TXFLTR, 0);
	dw_writel(dws, DW_SPI_TXFLR,  0);
	dw_writel(dws, DW_SPI_RXFLR,  0);
	dw_writel(dws, DW_SPI_SER,    0);
	dw_writel(dws, DW_SPI_CTRL1,  0);
	dw_writel(dws, DW_SPI_DMACR,  0);
	dw_writel(dws, DW_SPI_DMATDLR,0);
	dw_writel(dws, DW_SPI_DMATDLR,0);

	/* MODE */
	if (dws->rx && dws->tx)	tmode = SPI_TMOD_TR;
	else if (dws->rx)	    tmode = SPI_TMOD_RO;
	else			        tmode = SPI_TMOD_TO;

	cr0 = dw_readl(dws, DW_SPI_CTRL0);
	cr0 &= ~SPI_TMOD_MASK;
	cr0 |= (tmode << SPI_TMOD_OFFSET);
	dw_writel(dws, DW_SPI_CTRL0, cr0);

	/* SIZE */
	if (!dws->tx && dws->rx) {
		if(dws->len > 64*1024)
			return -1;
		dw_writel(dws, DW_SPI_CTRL1, dws->len-1);
	}

	/* DMATDLR */
	/* DMACR */
	if (dws->tx) {
		dma_ctrl |= SPI_DMA_TDMAE;
		dw_writel(dws, DW_SPI_DMATDLR, dws->fifo_len/2);
	}
	if (dws->rx) {
		dma_ctrl |= SPI_DMA_RDMAE;
		dw_writel(dws, DW_SPI_DMARDLR, dws->fifo_len/2 -1);
	}
	dw_writel(dws, DW_SPI_DMACR, dma_ctrl);

	return 0;
}

static int transfer (struct dw_spi *dws, struct spi_transfer *xfer)
{
	struct dma_async_tx_descriptor *rxdesc;
	struct dma_async_tx_descriptor *txdesc;

	/* rx must be started before tx due to spi instinct */
	rxdesc = prepare_rx(dws, xfer);
	if (rxdesc) {
		set_bit(RX_BUSY, &dws->dma_chan_busy);
		dmaengine_submit(rxdesc);
		dma_async_issue_pending(dws->rxchan);    // start
	}

	txdesc = prepare_tx(dws, xfer);
	if (txdesc) {
		set_bit(TX_BUSY, &dws->dma_chan_busy);
		dmaengine_submit(txdesc);
		dma_async_issue_pending(dws->txchan);    // start
	}

	if (!dws->tx && dws->rx)
		dw_writel(dws, DW_SPI_DR, 0);  // write dummy data to start read-only mode
	dw_writel(dws, DW_SPI_SER, 1<<1);  // start spi


	return 0;
}

static void stop (struct dw_spi *dws)
{
	if (test_bit(TX_BUSY, &dws->dma_chan_busy)) {
		dmaengine_terminate_all(dws->txchan);
		clear_bit(TX_BUSY, &dws->dma_chan_busy);
	}
	if (test_bit(RX_BUSY, &dws->dma_chan_busy)) {
		dmaengine_terminate_all(dws->rxchan);
		clear_bit(RX_BUSY, &dws->dma_chan_busy);
	}
}

static struct dw_spi_dma_ops  dma_ops = {
	.dma_init     = init,
	.dma_exit     = exit,
	.can_dma      = can,
	.dma_setup    = setup,
	.dma_transfer = transfer,
	.dma_stop     = stop,
};



void spi_dma_init (struct dw_spi *dws)
{
	dws->dma_ops = &dma_ops;
}
