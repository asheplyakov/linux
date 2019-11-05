/*
 * Designware SPI core controller driver (refer pxa2xx_spi.c)
 *
 * Copyright (c) 2009, Intel Corporation.
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

#define DRIVER_NAME "dw_spi_boot"
#define SPI_DW_BOOT_LOWLEVEL_DEBUG 0

static void spi_clean(struct dw_boot_spi *dws)
{
    dw_boot_writel(dws, DW_SPI_SER, 0);
    dw_boot_writel(dws, DW_SPI_SSIENR, 0);
}

static void spi_set_mode (struct dw_boot_spi *dws, int mode)
{
    struct {
        uint32_t  dfs    :4;    /* data frame size */
        uint32_t  frf    :2;    /* frame format (0-spi, 1-ssp, 2-micro, 3-reserved) */
        uint32_t  scph   :1;    /* clk phase */
        uint32_t  scpol  :1;    /* clk polarity */
        uint32_t  tmod   :2;    /* transfer mode (0-tx|rx, 1-tx, 2-rx, 3-eeprom) */
        uint32_t  slv_oe :1;    /* (ignore) slave output enable */
        uint32_t  srl    :1;    /* (ignore) shift register loop */
        uint32_t  cfs    :4;    /* (ignore) control frame size */
        uint32_t  _      :16;
    } ctr0;

    // spi_ctrlr0_t ctr0;
    *(u32*)&ctr0 = dw_boot_readl (dws, DW_SPI_CTRL0);
    ctr0.tmod = mode;
    dw_boot_writel(dws, DW_SPI_CTRL0, *(u32*)&ctr0);
}


static int boot_spi_write(struct spi_master *master, int chip_select,
    const uint8_t* tx1, const uint8_t* tx2, int len1, int len2)
{
    struct dw_boot_spi *dws;
    int i, n1, n2;
    const uint8_t* end1 = tx1 + len1;
    const uint8_t* end2 = tx2 + len2;
    unsigned long flags;

    local_irq_save(flags);               /* Critical section - ON */

    dws = spi_master_get_devdata(master);

    spi_clean(dws);
    spi_set_mode(dws, SPI_TMOD_TO);

    dw_boot_writel(dws, DW_SPI_SSIENR, 1);               /* ebable fifo */

    n1 = (len1 > dws->fifo_len    )? dws->fifo_len     : len1;  /* fill fifo */
    n2 = (len2 > dws->fifo_len -n1)? dws->fifo_len -n1 : len2;
    for (i = 0; i < n1; i++)
        dw_boot_writel(dws, DW_SPI_DR, *tx1++);
    for (i = 0; i < n2; i++)
        dw_boot_writel(dws, DW_SPI_DR, *tx2++);

    dw_boot_writel(dws, DW_SPI_SER, chip_select);        /* start sending */

    while (tx1 != end1) {                           /* regular transfer 1 */
        if(dw_boot_readl(dws, DW_SPI_SR) & SR_TF_NOT_FULL){
            dw_boot_writel(dws, DW_SPI_DR, *tx1++);
        }
    }
    while (tx2 != end2) {                           /* regular transfer 2 */
        if(dw_boot_readl(dws, DW_SPI_SR) & SR_TF_NOT_FULL){
            dw_boot_writel(dws, DW_SPI_DR, *tx2++);
        }
    }

    while(!(dw_boot_readl(dws, DW_SPI_SR) & SR_BUSY))    /* wait */
        ;

    local_irq_restore(flags);          /* Critical section - OFF */

    udelay(10);                                     /* don't delete */

    return 0;
}

static int boot_spi_read(struct spi_master *master, int chip_select,
    const uint8_t* tx, uint8_t* rx, int lentx, int lenrx)
{
    uint8_t* const rxend = rx + lenrx;
    struct dw_boot_spi *dws;
    unsigned long flags;
    int i;

    local_irq_save(flags);               /* Critical section - ON */

    if(!tx || !rx || !lentx || !lenrx || lentx > 4 || lenrx > 64*1024){
        return -1;
    }

    dws = spi_master_get_devdata(master);

    spi_clean(dws);
    spi_set_mode(dws, SPI_TMOD_EPROMREAD);

    dw_boot_writel(dws, DW_SPI_CTRL1, lenrx - 1);        /* rx config */
    dw_boot_writel(dws, DW_SPI_SSIENR, 1);               /* ebable fifo */

    for (i = 0; i < lentx; i++) {                   /* fill config */
        dw_boot_writel(dws, DW_SPI_DR, tx[i]);
    }

    dw_boot_writel(dws, DW_SPI_SER, chip_select);        /* start sending */

    while (rx != rxend) {                           /* read incoming data */
        if(dw_boot_readl(dws, DW_SPI_SR) & SR_RF_NOT_EMPT){
            *rx++ = dw_boot_readl(dws, DW_SPI_DR);
        }
    }

    local_irq_restore(flags);          /* Critical section - OFF */

    return 0;
}

static char direction (char cmd)
{
    /**
     * cmd + read
     * RDID        Read Identification                 9Fh  0   0     (1 to 20)
     * READ        Read Data Bytes                     03h  3   0     (1 to ∞)
     * FAST_READ   Read Data Bytes at Higher Speed     0Bh  3   8     (1 to ∞)
     * DOFR        Dual Output Fast Read               3Bh  3   8     (1 to ∞)
     * DIOFR       Dual Input/Output Fast Read         BBh  3   8     (1 to ∞)
     * QOFR        Quad Output Fast Read               6Bh  3   8     (1 to ∞)
     * QIOFR       Quad Input/Output Fast Read         EBh  3   10    (1 to ∞)
     * ROTP        Read OTP (Read of OTP area)         4Bh  3   8     (1 to 65)
     * RDSR        Read Status Register                05h  0   0     (1 to ∞)
     * RDLR        Read Lock Register                  E8h  3   0     (1 to ∞)
     * RFSR        Read Flag Status Register           70h  0   0     (1 to ∞)
     * RDNVCR      Read NV Configuration Register      B5h  0   0     (2)
     * RDVCR       Read Volatile Config Register       85h  0   0     (1 to ∞)
     * RDVECR      Read Volatile Enhanced Config Reg   65h  0   0     (1 to ∞)
     *
     * cmd + write
     * PP          Page Program                        02h  3   0     (1 to 256)
     * DIFP        Dual Input Fast Program             A2h  3   0     (1 to 256)
     * DIEFP       Dual Input Extended Fast Program    D2h  3   0     (1 to 256)
     * QIFP        Quad Input Fast Program             32h  3   0     (1 to 256)
     * QIEFP       Quad Input Extended Fast Program    12h  3   0     (1 to 256)
     * POTP        Program OTP (Program of OTP area)   42h  3   0     (1 to 65)
     * WRSR        Write Status Register               01h  0   0     (1)
     * WRLR        Write to Lock Register              E5h  3   0     (1)
     * WRNVCR      Write NV Configuration Register     B1h  0   0     (2)
     * WRVCR       Write Volatile Config Register      81h  0   0     (1)
     * WRVECR      Write Volatile Enhanced Config Reg  61h  0   0     (1)
     *
     * cmd + null
     * WREN        Write Enable                        06h  0   0     (0)
     * WRDI        Write Disable                       04h  0   0     (0)
     * SSE         SubSector Erase                     20h  3   0     (0)
     * SE          Sector Erase                        D8h  3   0     (0)
     * BE          Bulk Erase                          C7h  0   0     (0)
     * PER         Program/Erase Resume                7Ah  0   0     (0)
     * PES         Program/Erase Suspend               75h  0   0     (0)
     * CLFSR       Clear Flag Status Register          50h  0   0     (0)
     */

    switch(cmd & 0xff)
    {
        /* cmd + read */
        case 0x9F:
        case 0x03:
        case 0x0B:
        case 0x3B:
        case 0xBB:
        case 0x6B:
        case 0xEB:
        case 0x4B:
        case 0x05:
        case 0xE8:
        case 0x70:
        case 0xB5:
        case 0x85:
        case 0x65:
            return 'r';

        /* cmd + write */
        case 0x02:
        case 0xA2:
        case 0xD2:
        case 0x32:
        case 0x12:
        case 0x42:
        case 0x01:
        case 0xE5:
        case 0xB1:
        case 0x81:
        case 0x61:
            return 'w';

        /* cmd + null */
        case 0x06:
        case 0x04:
        case 0x20:
        case 0xD8:
        case 0xC7:
        case 0x7A:
        case 0x75:
        case 0x50:
            return 'n';
    }
    return 0;
}

static int boot_spi_transfer_one_message(struct spi_master *master, struct spi_message *msg)
{

    struct list_head *const head = &msg->transfers;
    struct spi_transfer *pos, *next;
    int select = BIT(msg->spi->chip_select);
    int err = 0;

    list_for_each_entry(pos, head, transfer_list)
    {
        const void *out1 = 0;
        const void *out2 = 0;
        void *in = 0;
        int len1 = 0, len2 = 0, len3 = 0;

        char cmd = *(char*) pos->tx_buf;
        next = list_next_entry(pos, transfer_list);
        switch(direction(cmd)){
            case 'w':
            out1 = pos->tx_buf;
            out2 = next->tx_buf;
            len1 = pos->len;
            len2 = next->len;
            err = boot_spi_write(master, select, out1, out2, len1, len2);
            if(err)
                goto exit;
            msg->actual_length += len1 + len2;
            pos = next;
            break;

            case 'r':
            out1 = pos->tx_buf;
            in   = next->rx_buf;
            len1 = pos->len;
            len3 = next->len;
            err = boot_spi_read(master, select, out1, in, len1, len3);
            if(err)
                goto exit;
            msg->actual_length += len1 + len3;
            pos = next;
            break;

            case 'n':
            out1 = pos->tx_buf;
            len1 = pos->len;
            err = boot_spi_write(master, select, out1, 0, len1, 0);
            if(err)
                goto exit;
            msg->actual_length += len1;
            break;

            default:
            err = -1;
            goto exit;
        }
        udelay(pos->delay_usecs);

#if SPI_DW_BOOT_LOWLEVEL_DEBUG
        {
            int i;
            const uint8_t *x;

            x = out1;
            if(x){
                printk("> ");
                for(i=0; i<len1; i++){
                    printk("%02x ", x[i]);
                }
                printk("\n");
            }
            x = out2;
            if(x){
                printk("> ");
                for(i=0; i<len2; i++){
                    printk("%02x ", x[i]);
                }
                printk("\n");
            }
            x = in;
            if(x){
                printk("< ");
                for(i=0; i<len3; i++){
                    printk("%02x ", x[i]);
                }
                printk("\n");
            }

            printk("\n");
        }
#endif

    }  /* list_for_each_entry */

exit:
    msg->status = err;
    spi_finalize_current_message(master);
    return err;
}

static int fifo_len(struct dw_boot_spi *dws)
{
    u32 txfltr = dw_boot_readl(dws, DW_SPI_TXFLTR);

    u32 fifo;
    for (fifo = 1; fifo < 256; fifo++) {
        dw_boot_writel(dws, DW_SPI_TXFLTR, fifo);
        if (fifo != dw_boot_readl(dws, DW_SPI_TXFLTR))
            break;
    }
    dw_boot_writel(dws, DW_SPI_TXFLTR, txfltr);
    return (fifo == 1) ? 0 : fifo/(dws->reg_io_width);
}

static void init(struct dw_boot_spi *dws)
{
    /* clear all registers */
    #if 1
    int i;
    for (i = 0; i < DW_SPI_DR; i+= sizeof(uint32_t)) {
        dw_boot_writel(dws, i, 0);
    }
    #else
    dw_boot_writel(dws, DW_SPI_CTRL0, 0);
    dw_boot_writel(dws, DW_SPI_CTRL1, 0);
    dw_boot_writel(dws, DW_SPI_SSIENR, 0);
    dw_boot_writel(dws, DW_SPI_MWCR, 0);
    dw_boot_writel(dws, DW_SPI_SER, 0);
    dw_boot_writel(dws, DW_SPI_BAUDR, 0);
    dw_boot_writel(dws, DW_SPI_TXFLTR, 0);
    dw_boot_writel(dws, DW_SPI_RXFLTR, 0);
    dw_boot_writel(dws, DW_SPI_TXFLR, 0);
    dw_boot_writel(dws, DW_SPI_RXFLR, 0);
    dw_boot_writel(dws, DW_SPI_SR, 0);
    dw_boot_writel(dws, DW_SPI_IMR, 0);
    dw_boot_writel(dws, DW_SPI_ISR, 0);
    dw_boot_writel(dws, DW_SPI_RISR, 0);
    dw_boot_writel(dws, DW_SPI_TXOICR, 0);
    dw_boot_writel(dws, DW_SPI_RXOICR, 0);
    dw_boot_writel(dws, DW_SPI_RXUICR, 0);
    dw_boot_writel(dws, DW_SPI_MSTICR, 0);
    dw_boot_writel(dws, DW_SPI_ICR, 0);
    dw_boot_writel(dws, DW_SPI_DMACR, 0);
    dw_boot_writel(dws, DW_SPI_DMATDLR, 0);
    dw_boot_writel(dws, DW_SPI_DMARDLR, 0);
    #endif

    /* use dws parameters */
    dw_boot_writel(dws, DW_SPI_BAUDR, 6);   /* todo: use dws->clk to init baudrate */
}

static int add_host(struct device *dev, struct dw_boot_spi *dws)
{
    struct spi_master *master;
    int ret;

    master = spi_alloc_master(dev, 0);
    if (!master){
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
        dev_err(&master->dev, "problem registering spi master\n");
        spi_master_put(master);
    }
    return ret;
}

static int probe(struct platform_device *pdev)
{
    struct dw_boot_spi *dws;
    struct resource *mem;
    struct device_node *bc_np;
    int ret;

    /* alloc dws */
    dws = devm_kzalloc(&pdev->dev, sizeof(struct dw_boot_spi), GFP_KERNEL);
    if (!dws) {
        return -ENOMEM;
    }

    /* Get basic io resource and map it */
    mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (!mem) {
        dev_err(&pdev->dev, "no mem resource?\n");
        return -EINVAL;
    }

    dws->regs = devm_ioremap_resource(&pdev->dev, mem);
    if (IS_ERR(dws->regs)) {
        dev_err(&pdev->dev, "SPI region map failed\n");
        return PTR_ERR(dws->regs);
    }

    dws->clk = devm_clk_get(&pdev->dev, NULL);
    if (IS_ERR(dws->clk)) {
        return PTR_ERR(dws->clk);
    }

    ret = clk_prepare_enable(dws->clk);
    if (ret) {
        return ret;
    }

    /* get spi parameters */
    dws->bus_num = of_alias_get_id(pdev->dev.of_node, "ssi");
    dws->max_freq = clk_get_rate(dws->clk);
    device_property_read_u32(&pdev->dev, "reg-io-width", &dws->reg_io_width);
    dws->num_cs = 4;
    device_property_read_u32(&pdev->dev, "num-cs", &dws->num_cs);
    dws->fifo_len = fifo_len(dws);

    /* Enable the DW SPI controller interface */
    bc_np = of_parse_phandle(pdev->dev.of_node, "boot-controller", 0);
    if (!bc_np) {
        ret = -EINVAL;
        dev_err(&pdev->dev, "boot-controller of-node missing\n");
        goto out;
    }
    dws->bc = of_find_be_bc_device_by_node(bc_np);
    if (!dws->bc) {
        ret = -EINVAL;
        dev_err(&pdev->dev, "BE BC search failed\n");
	goto out;
    }
    be_bc_enable_spi(dws->bc);

    /* use parameters to init spi */
    init(dws);

    /* add host */
    ret = add_host(&pdev->dev, dws);
    if (ret)
        goto dis_be_bc;

    platform_set_drvdata(pdev, dws);
    return 0;

dis_be_bc:
    be_bc_disable_spi(dws->bc);

out:
    clk_disable_unprepare(dws->clk);
    return ret;
}

static int remove(struct platform_device *pdev)
{
    struct dw_boot_spi *dws = platform_get_drvdata(pdev);

    clk_disable_unprepare(dws->clk);
    be_bc_disable_spi(dws->bc);

    return 0;
}

static const struct of_device_id boot_spi_of_match[] = {
    { .compatible = "snps,dw-spi-boot", },
    { /* end of table */}
};
MODULE_DEVICE_TABLE(of, boot_spi_of_match);

static struct platform_driver dw_spi_mmio_driver = {
    .probe      = probe,
    .remove     = remove,
    .driver     = {
        .name   = DRIVER_NAME,
        .of_match_table = boot_spi_of_match,
    },
};
module_platform_driver(dw_spi_mmio_driver);

MODULE_DESCRIPTION("interface driver for DW SPI Boot flash (without GPIO, IRQ)");
MODULE_LICENSE("GPL v2");
