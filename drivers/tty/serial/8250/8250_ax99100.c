// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for AX99100 PCI-Serial device
 *
 * Copyright (C) 2018 T-platforms JSC (fancer.lancer@gmail.com)
 */

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/pci.h>
#include <linux/io.h>
#include <linux/serial_8250.h>
#include <linux/gpio/driver.h>
#include <linux/irq.h>

#include "8250.h"

/* TODO delete this declaration when moving to the newest kernel */
#include <linux/kernel_compat.h>

/*
 * AX99100 PCI device
 *
 * Asix AX99100 chip is multi-IO PCIe device, which provides an access
 * to Local Bus, Parallal and Serial Ports, SPI, i2c and GPIO interfaces
 * over single PCIe endpoint. The availability of certain ports depends on
 * the chip mode (CHIP_MODE[2-0] pins state):
 * Mode 000 - 1x Local Bus,
 * Mode 001 - 2x Serial Ports, 1x Parallel Port,
 * Mode 010 - 2x Multi-protocol Serial Ports, 1x Parallel Port,
 * Mode 011 - 4x Serial Ports,
 * Mode 100 - 2x Multi-protocol Serial Ports, 1x GPIO block, 1x SPI,
 * Mode 101 - 2x Serial Ports, 2x Multi-protocol Serial Port,
 * Mode 110 - 2x Serial Port, 1x GPIO block, 1x SPI,
 * Mode 111 - 4x Multi-protocol Serial Port.
 *
 * As you can see serial ports are accessible in nearly any chip mode except
 * mode 000, when local bus controller is only available. In case of being
 * activated the ports always occupy functions 0 and 1, as well as functions
 * 2 and 3 when mode 011/111 is selected.
 *
 * i2c and GPIO interfaces are accessible in any chip mode, but GPIO interrupts
 * are available for function 0 only. That's why GPIO interface support code is
 * implemented here, while i2c-interface support code is available in
 * spi-ax99100 driver.
 */

/*
 * AX99100 SP-PCI resources
 */
#define PCI_BAR_MASK		BIT(1) | BIT(5)
#define GPIO_MODE_ANY_NUM	8
#define GPIO_MODE_SPI_NUM	17
#define CHIP_MODE_LB		0
#define CHIP_MODE_2S_1P		1
#define CHIP_MODE_2MP_1P	2
#define CHIP_MODE_4S		3
#define CHIP_MODE_2MP_1SPI	4
#define CHIP_MODE_2S_2MP	5
#define CHIP_MODE_2S_1SPI	6
#define CHIP_MODE_4MP		7

/*
 * Serial Port: 8250 serial registers (PIO - BAR0)
 */
#define SP_PIO_8250		0x0

/*
 * Serial Port: common/8250 serial registers, DMA engine and IRQs (MMIO - BAR1)
 */
#define SP_SSR0			0x200
#define SP_SSR1			0x204
#define SP_GER			0x208
#define SP_GOR			0x20C
#define SP_GPER			0x210
#define SP_BCSR			0x214
#define  SP_BCSR_1_838MHZ	0x00
#define  SP_BCSR_125MHZ		0x01
#define  SP_BCSR_EXT		0x02
#define  SP_BCSR_SET(_x, _cs)	(((_x) & ~GENMASK(1, 0)) | (_cs))
#define SP_GIR			0x218
#define SP_SSR2			0x21C
#define SP_TFC			0x220
#define SP_RFC			0x224
#define SP_TTL			0x228
#define SP_RTL			0x22C
#define SP_FCLTL		0x230
#define SP_FCUTL		0x234
#define SP_SWRST		0x238

#define SP_MMIO_BASE		0x280

#define SP_GIS			0x3A0
#define  SP_GIS_TDMA		BIT(1)
#define  SP_GIS_RDMA		BIT(2)
#define  SP_GIS_PDON		BIT(3)
#define  SP_GIS_GPIO		BIT(4)
#define SP_GIC			0x3A4
#define SP_GIE			0x3A8
#define  SP_GIE_TDMA		BIT(1)
#define  SP_GIE_RDMA		BIT(2)
#define  SP_GIE_PDON		BIT(3)
#define  SP_GIE_GPIO		BIT(4)

/*
 * Generic interface registers: GPIO (MMIO - BAR5)
 */
#define GPIO_SET(_r, _b) 	((_r) | BIT(_b))
#define GPIO_CLEAR(_r, _b)	((_r) & ~BIT(_b))
#define GPIO_CHECK(_r, _b)	(!!((_r) & BIT(_b)))
#define GPIO_PIN		0x3C0
#define GPIO_DIR		0x3C4
#define GPIO_EM			0x3C8
#define GPIO_OD			0x3CC
#define GPIO_PU			0x3D0
#define GPIO_EDS		0x3D4
#define  GPIO_EDS_ECSS		BIT(24)
#define  GPIO_EDS_EOES		BIT(25)
#define GPIO_EDE		0x3D8
#define  GPIO_EDE_MR(_r)	(((_r) & GENMASK(31,29)) >> 29)
#define GPIO_CTR		0x3DC
#define  GPIO_CTR_WE		BIT(24)

struct ax99100_sp {
	int line;
};

struct ax99100_gpio {
	struct gpio_chip chip;
	u32 mask;

	raw_spinlock_t lock;
	raw_spinlock_t wa_lock;
};
#define to_ax99100_gpio_chip(_chip) \
	container_of(container_of(_chip, struct ax99100_gpio, chip), \
		     struct ax99100_data, gpio)
#define to_ax99100_irq_data(_data) \
	to_ax99100_gpio_chip(irq_data_get_irq_chip_data(_data))

struct ax99100_pci {
	struct pci_dev *pdev;

	int irq;
	void __iomem *sm;
	void __iomem *im;
};

struct ax99100_data {
	struct ax99100_pci pci;

	struct ax99100_sp sp;

#ifdef CONFIG_SERIAL_8250_AX99100_GPIO
	struct ax99100_gpio gpio;
#endif
};

/*
 * AX99100 PCI-SP driver
 *
 * Each serial port provides a functionality of standard 16C450, 16C550,
 * 16C650, 16C750 and 16C950 with an ability to handle a connection over
 * RS232, RS422 and RS485 protocols. The baud rate can be setup up to 25Mbps
 * with FIFO up to 256 bytes. In case if it is necessary there is an embedded
 * DMA engine. This driver enables the 16C950 standard of the serial ports.
 *
 * TODO:
 * 1) Use the embedded DMA engine to store and load data to the chip FIFOs.
 * This shall move the memcpy work-load to the chip.
 * 2) Create RS485 configuration callback. The chip also provides a way to
 * automatically handle the full and half-duplex communication protocols
 * like RS485, to suppress an RS485-echos, to set the TXEN/RXEN pin activation
 * delays.
 * 3) Create an adaptive baud rate calculation algo. There is special register,
 * which lets to set the reference clocks source like: basic 1.838235MHz,
 * internal PLL of 125MHz and a custom external clocks EXT_CLK. This feature
 * permits to set the serial port bauds up to 25Mbps (by means of 125MHz clock
 * source).
 */
static inline u32 ax99100_sp_read(struct ax99100_data *ax, unsigned long addr)
{
	return ioread8(ax->pci.sm + addr);
}

static inline void ax99100_sp_write(struct ax99100_data *ax, u32 val,
				    unsigned long addr)
{
	iowrite8(val, ax->pci.sm + addr);
}

static int ax99100_sp_init(struct ax99100_data *ax)
{
	struct uart_8250_port uart = {0};
	int ret = 0;
	u32 val;

	/* Reset serial port before usage */
	ax99100_sp_write(ax, 1, SP_SWRST);

	/* Set the clocks source to be of internal pll with 1838235Hz */
	val = ax99100_sp_read(ax, SP_BCSR);
	ax99100_sp_write(ax, SP_BCSR_SET(val, SP_BCSR_1_838MHZ), SP_BCSR);

	uart.port.dev = &ax->pci.pdev->dev;
	uart.port.iobase = pci_resource_start(ax->pci.pdev, 0) + SP_PIO_8250;
	uart.port.iotype = UPIO_PORT;

	uart.port.irq = ax->pci.irq;
	uart.port.private_data = ax;
	uart.port.type = PORT_16C950;
	/* Default clock source is 1838235 with TCLKR devider being 16 */
	uart.port.uartclk = 115200 * 16;
	uart.tx_loadsz = 128;
	uart.port.fifosize = 128;

	uart.port.flags = UPF_HARD_FLOW | UPF_SOFT_FLOW | UPF_SHARE_IRQ | 
			  UPF_FIXED_TYPE;
	uart.capabilities = UART_CAP_FIFO | UART_CAP_EFR | UART_CAP_SLEEP;

	ret = serial8250_register_8250_port(&uart);
	if (ret < 0) {
		dev_err(&ax->pci.pdev->dev, "Failed to add 8250 port\n");
		return ret;
	}

	ax->sp.line = ret;

	dev_info(&ax->pci.pdev->dev, "ttyS%d interface created\n", ax->sp.line);

	return 0;
}

static void ax99100_sp_clear(struct ax99100_data *ax)
{
	serial8250_unregister_port(ax->sp.line);
}

/*
 * AX99100 PCI-GPIO interface driver
 *
 * GPIO-interface provides an access to up to 24 GPIOs. GPIOs from 0 to 7 are
 * supported for any device mode. GPIOs from 8 to 16 are available for modes
 * 100 and 110. The rest of GPIOs are available for usage only if SPI-function
 * is disabled by EEPROM firmware (this isn't supported at the moment). The
 * GPIO interface functionality is handled by function 0 serial device driver,
 * due to two reasons: IRQs generation is only availabe on function 0, GPIO
 * registers are shared between all PCIe functions so we need to select a single
 * bus-device instance to handle the race condition.
 *
 * AX99100 GPIOs provides many configurations like: pins current state,
 * direction, open-drain mode and pull-up resisters, edge-triggered maskable
 * events. These facilities are used to create a fully functional gpio-irq
 * driver.
 */
#ifdef CONFIG_SERIAL_8250_AX99100_GPIO
static inline u32 ax99100_gpio_read(struct ax99100_data *ax, unsigned long addr)
{
	return ioread32(ax->pci.im + addr);
}

static inline void ax99100_gpio_write(struct ax99100_data *ax, u32 val,
				      unsigned long addr)
{
	iowrite32(val, ax->pci.im + addr);
}

static int ax99100_gpio_get_dir(struct gpio_chip *chip, unsigned offset)
{
	struct ax99100_data *ax = to_ax99100_gpio_chip(chip);
	u32 val;

	val = ax99100_gpio_read(ax, GPIO_DIR);

	return GPIO_CHECK(val, offset);
}

static int ax99100_gpio_dir_in(struct gpio_chip *chip, unsigned offset)
{
	struct ax99100_data *ax = to_ax99100_gpio_chip(chip);
	unsigned long flags;
	u32 val;

	/* Documentation vaguely states that GPIOs 2 - 5 can't be setup as
	 * inputs due to being used for CHIP_MODE settings latching on reset.
	 * At the same time examples do use them as inputs, so lets rely
	 * on examples until it's practically proved otherwise.
	 */
	raw_spin_lock_irqsave(&ax->gpio.lock, flags);
	val = ax99100_gpio_read(ax, GPIO_DIR);
	ax99100_gpio_write(ax, GPIO_SET(val, offset), GPIO_DIR);
	raw_spin_unlock_irqrestore(&ax->gpio.lock, flags);

	return 0;
}

static int ax99100_gpio_dir_out(struct gpio_chip *chip, unsigned offset,
				int value)
{
	struct ax99100_data *ax = to_ax99100_gpio_chip(chip);
	unsigned long flags;
	u32 val;

	raw_spin_lock_irqsave(&ax->gpio.lock, flags);
	val = ax99100_gpio_read(ax, GPIO_PIN);
	if (value)
		val = GPIO_SET(val, offset);
	else
		val = GPIO_CLEAR(val, offset);
	ax99100_gpio_write(ax, val, GPIO_PIN);
	val = ax99100_gpio_read(ax, GPIO_DIR);
	ax99100_gpio_write(ax, GPIO_CLEAR(val, offset), GPIO_DIR);
	raw_spin_unlock_irqrestore(&ax->gpio.lock, flags);

	return 0;
}

static int ax99100_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	struct ax99100_data *ax = to_ax99100_gpio_chip(chip);
	u32 val;

	val = ax99100_gpio_read(ax, GPIO_PIN);

	return GPIO_CHECK(val, offset);
}

static void ax99100_gpio_set(struct gpio_chip *chip, unsigned offset,
			     int value)
{
	struct ax99100_data *ax = to_ax99100_gpio_chip(chip);
	unsigned long flags;
	u32 val;

	raw_spin_lock_irqsave(&ax->gpio.lock, flags);
	val = ax99100_gpio_read(ax, GPIO_PIN);
	if (value)
		val = GPIO_SET(val, offset);
	else
		val = GPIO_CLEAR(val, offset);
	ax99100_gpio_write(ax, val, GPIO_PIN);
	raw_spin_unlock_irqrestore(&ax->gpio.lock, flags);
}

static void ax99100_gpio_set_multi(struct gpio_chip *chip, unsigned long *mask,
				   unsigned long *bits)
{
	struct ax99100_data *ax = to_ax99100_gpio_chip(chip);
	unsigned long flags;
	u32 val;

	raw_spin_lock_irqsave(&ax->gpio.lock, flags);
	val = ax99100_gpio_read(ax, GPIO_PIN);
	ax99100_gpio_write(ax, (val & *mask) | (*bits & *mask), GPIO_PIN);
	raw_spin_unlock_irqrestore(&ax->gpio.lock, flags);
}

static void ax99100_gpio_irq_mask(struct irq_data *d)
{
	struct ax99100_data *ax = to_ax99100_irq_data(d);
	unsigned offset = d->hwirq;
	unsigned long flags;
	u32 val;

	raw_spin_lock_irqsave(&ax->gpio.lock, flags);
	val = ax99100_gpio_read(ax, GPIO_EDE);
	ax99100_gpio_write(ax, GPIO_CLEAR(val, offset), GPIO_EDE);
	raw_spin_unlock_irqrestore(&ax->gpio.lock, flags);
}

static void ax99100_gpio_irq_unmask(struct irq_data *d)
{
	struct ax99100_data *ax = to_ax99100_irq_data(d);
	unsigned offset = d->hwirq;
	unsigned long flags;
	u32 val;

	raw_spin_lock_irqsave(&ax->gpio.lock, flags);
	val = ax99100_gpio_read(ax, GPIO_EDE);
	ax99100_gpio_write(ax, GPIO_SET(val, offset), GPIO_EDE);
	raw_spin_unlock_irqrestore(&ax->gpio.lock, flags);
}

static int ax99100_gpio_irq_set_type(struct irq_data *d, unsigned int flow_type)
{
	struct ax99100_data *ax = to_ax99100_irq_data(d);
	unsigned offset = d->hwirq;
	unsigned long flags;
	u32 rf, both;
	int ret = 0;

	raw_spin_lock_irqsave(&ax->gpio.lock, flags);

	both = ax99100_gpio_read(ax, GPIO_CTR);

	switch(flow_type) {
	case IRQ_TYPE_EDGE_RISING:
	case IRQ_TYPE_EDGE_FALLING:
		rf = ax99100_gpio_read(ax, GPIO_EM);
		if (flow_type == IRQ_TYPE_EDGE_RISING)
			ax99100_gpio_write(ax, GPIO_SET(rf, offset), GPIO_EM);
		else
			ax99100_gpio_write(ax, GPIO_CLEAR(rf, offset), GPIO_EM);
		ax99100_gpio_write(ax, GPIO_CLEAR(both, offset), GPIO_CTR);
		break;
	case IRQ_TYPE_EDGE_BOTH:
		ax99100_gpio_write(ax, GPIO_SET(both, offset), GPIO_CTR);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	raw_spin_unlock_irqrestore(&ax->gpio.lock, flags);

	return ret;
}

static struct irq_chip ax99100_gpio_ic = {
	.name = "ax99100_gpio_ic",
	.irq_mask = ax99100_gpio_irq_mask,
	.irq_unmask = ax99100_gpio_irq_unmask,
	.irq_set_type = ax99100_gpio_irq_set_type
};

static irqreturn_t ax99100_gpio_isr(struct ax99100_data *ax)
{
	struct gpio_chip *chip = &ax->gpio.chip;
	unsigned long flags, offset, val;

	val = ax99100_gpio_read(ax, GPIO_EDS);
	if (!(val & ax->gpio.mask))
		return IRQ_NONE;

	for_each_set_bit(offset, &val, chip->ngpio) {
		raw_spin_lock_irqsave(&ax->gpio.wa_lock, flags);
		generic_handle_irq(irq_find_mapping(chip->irqdomain, offset));
		raw_spin_unlock_irqrestore(&ax->gpio.wa_lock, flags);
	}

	ax99100_gpio_write(ax, val, GPIO_EDS);

	return IRQ_HANDLED;
}

static int ax99100_gpio_init(struct ax99100_data *ax)
{
	struct gpio_chip *chip = &ax->gpio.chip;
	struct pci_dev *pdev = ax->pci.pdev;
	u32 mode;
	int ret;

	/* Initialize GPIO controller for function 0 only */
	if (PCI_FUNC(pdev->devfn))
		return 0;

	/* Retrieve ax99100 chip mode */
	mode = ax99100_gpio_read(ax, GPIO_EDE);
	mode = GPIO_EDE_MR(mode);

	/* Add GPIO-chip with num of pins corresponding to the mode */
	chip->label = "ax99100_gpio";
	chip->dev = &ax->pci.pdev->dev;
	chip->get_direction = ax99100_gpio_get_dir;
	chip->direction_input = ax99100_gpio_dir_in;
	chip->direction_output = ax99100_gpio_dir_out;
	chip->get = ax99100_gpio_get;
	chip->set = ax99100_gpio_set;
	chip->set_multiple = ax99100_gpio_set_multi;
	chip->can_sleep = false;
	chip->base = -1;
	if (mode == CHIP_MODE_2S_1SPI || mode == CHIP_MODE_2MP_1SPI) {
		chip->ngpio = GPIO_MODE_SPI_NUM;
		ax->gpio.mask = GENMASK(GPIO_MODE_SPI_NUM - 1, 0);
	} else {
		chip->ngpio = GPIO_MODE_ANY_NUM;
		ax->gpio.mask = GENMASK(GPIO_MODE_ANY_NUM - 1, 0);
	}

	raw_spin_lock_init(&ax->gpio.lock);

	ret = gpiochip_add(chip);
	if (ret) {
		dev_err(&pdev->dev, "Failed to add GPIO-chip\n");
		return ret;
	}

	raw_spin_lock_init(&ax->gpio.wa_lock);

	/* Disable any GPIO events at probe method */
	ax99100_gpio_write(ax, 0, GPIO_EDE);

	/* Add IRQ-chip corresponding to the GPIOs */
	ret = gpiochip_irqchip_add(chip, &ax99100_gpio_ic, 0, handle_simple_irq,
				   IRQ_TYPE_NONE);
	if (ret) {
		dev_err(&pdev->dev, "Failed to add IRQ-chip\n");
		goto err_gpiochip_remove;
	}

	gpiochip_set_chained_irqchip(chip, &ax99100_gpio_ic, ax->pci.irq, NULL);

	dev_info(&pdev->dev, "GPIO-%d (%d) interface created\n",
		 chip->base, chip->ngpio);

	return 0;

err_gpiochip_remove:
	gpiochip_remove(chip);

	return ret;
}

static void ax99100_gpio_clear(struct ax99100_data *ax)
{
	/* Clear GPIO controller for function 0 only */
	if (PCI_FUNC(ax->pci.pdev->devfn))
		return;

	gpiochip_remove(&ax->gpio.chip);
}
#else
static irqreturn_t ax99100_gpio_isr(struct ax99100_data *ax)
{
	return IRQ_NONE;
}

static int ax99100_gpio_init(struct ax99100_data *ax)
{
	return 0;
}

static void ax99100_gpio_clear(struct ax99100_data *ax) {}
#endif /* CONFIG_SERIAL_8250_AX99100_GPIO */

/*
 * AX99100 PCI general driver
 *
 * The PCI-SP functions are activated by nearly any CHIP_MODE except 000.
 * in this case the PCI device provides from 2 to 4 serial functions.
 * Each of the functions also provide an access to common interfaces like
 * i2c and GPIO by means of BAR5 MMIO registers. This driver supports the
 * serial ports and GPIO interface only.
 */
static irqreturn_t ax99100_pci_isr(int irq, void *devid)
{
	struct ax99100_data *ax = devid;
	irqreturn_t ret = IRQ_NONE;
	u32 val;

	val = ax99100_sp_read(ax, SP_GIS);
	if (val & SP_GIS_GPIO)
		ret = ax99100_gpio_isr(ax);

	/* NOTE In future this function shall also serve to handle the DMA
	 * interrupts, but for now we're only working with GPIO IRQs here.
	 */

	ax99100_sp_write(ax, val, SP_GIC);

	return ret;
}

static int ax99100_isr_init(struct ax99100_data *ax)
{
	int ret = 0;

	ret = devm_request_irq(&ax->pci.pdev->dev, ax->pci.irq, ax99100_pci_isr,
			       IRQF_SHARED, "ax99100_sp", ax);
	if (ret) {
		dev_err(&ax->pci.pdev->dev, "Failed to set IRQ#%d handler\n",
			ax->pci.irq);
	} else {
		/* Enable GPIO IRQs for now only. Serial Port IRQs are setup by
		 * normal port registers passed to 8250 subsystem.
		 */
		ax99100_sp_write(ax, SP_GIE_GPIO, SP_GIE);
	}

	return ret;
}

static void ax99100_isr_clear(struct ax99100_data *ax)
{
	ax99100_sp_write(ax, 0, SP_GIE);

	/* Manually free IRQ otherwise PCI free irq vectors will fail */
	devm_free_irq(&ax->pci.pdev->dev, ax->pci.irq, ax);
}

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

	/* Map BAR1 and BAR5 only while BAR0 shall be occupied by Serial 8250
	 * subsystem.
	 */
	ret = pcim_iomap_regions(pdev, PCI_BAR_MASK, "ax99100_sp");
	if (ret) {
		dev_err(&pdev->dev, "Failed to request PCI regions\n");
		goto err_clear_master;
	}

	bars = pcim_iomap_table(pdev);
	ax->pci.sm = bars[1];
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

	ret = ax99100_sp_init(ax);
	if (ret)
		goto err_pci_clear;

	ret = ax99100_gpio_init(ax);
	if (ret)
		goto err_sp_clear;

	ret = ax99100_isr_init(ax);
	if (ret)
		goto err_gpio_clear;

	return 0;

err_gpio_clear:
	ax99100_gpio_clear(ax);

err_sp_clear:
	ax99100_sp_clear(ax);

err_pci_clear:
	ax99100_pci_clear(ax);

	return ret;
}

static void ax99100_remove(struct pci_dev *pdev)
{
	struct ax99100_data *ax = pci_get_drvdata(pdev);

	ax99100_isr_clear(ax);

	ax99100_gpio_clear(ax);

	ax99100_sp_clear(ax);

	ax99100_pci_clear(ax);
}

static const struct pci_device_id ax99100_pci_id[] = {
	{ PCI_VENDOR_ID_ASIX, PCI_DEVICE_ID_ASIX_AX99100, 0xa000, 0x1000 },
	{ },
};
MODULE_DEVICE_TABLE(pci, ax99100_pci_id);

static struct pci_driver ax99100_driver = {
	.name           = "ax99100_sp",
	.id_table       = ax99100_pci_id,
	.probe          = ax99100_probe,
	.remove         = ax99100_remove
};
module_pci_driver(ax99100_driver);

MODULE_DESCRIPTION("Asix AX99100 PCI-Serial driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Sergey Semin <fancer.lancer@gmail.com>");
