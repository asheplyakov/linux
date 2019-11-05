#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/backlight.h>
#include <linux/fb.h>
#include <linux/backlight.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/of.h>

#include "sm750.h"
#include "sm750_help.h"
#include "ddk750_reg.h"
#include "ddk750_help.h"

#define FREQUENCY         96
#define BACKLIGHT_MIN     1
#define BACKLIGHT_DEF     BACKLIGHT_MAX
#define BACKLIGHT_MAX     10

static void sm750_set_bl(unsigned char pwm, unsigned int level)
{
	unsigned int ulReg, value, base;

	base = PWM_0 + 4*pwm;
	ulReg = PEEK32(base);

	/* Fields are the same for any PWM register */
	value = ((MHZ(FREQUENCY)/10) * level) >> 15;
	ulReg = FIELD_VALUE(ulReg, PWM_0, HIGH, value);

	value = ((MHZ(FREQUENCY)/10) * (BACKLIGHT_MAX - level)) >> 15;
	ulReg = FIELD_VALUE(ulReg, PWM_0, LOW, value);

	ulReg = FIELD_VALUE(ulReg, PWM_0, STATUS, PWM_0_STATUS_ENABLED | PWM_O_STATUS_INTERRUPT_PENDING);

	POKE32(base, ulReg);
}

static irqreturn_t sm750_bl_up_isr(int irq, void* dev_id)
{
	struct sm750_dev *sm750_dev = dev_id;
	unsigned long irqflags;

	spin_lock_irqsave(&sm750_dev->bl.count_lock, irqflags);
	if(sm750_dev->bl.count < BACKLIGHT_MAX) {
		sm750_dev->bl.count++;
		sm750_set_bl(sm750_dev->bl.pwm, sm750_dev->bl.count);
	}
	spin_unlock_irqrestore(&sm750_dev->bl.count_lock, irqflags);

	return IRQ_HANDLED;
}

static irqreturn_t sm750_bl_down_isr(int irq, void* dev_id)
{
	struct sm750_dev *sm750_dev = dev_id;
	unsigned long irqflags;

	spin_lock_irqsave(&sm750_dev->bl.count_lock, irqflags);
	if(sm750_dev->bl.count > BACKLIGHT_MIN) {
		sm750_dev->bl.count--;
		sm750_set_bl(sm750_dev->bl.pwm, sm750_dev->bl.count);
	}
	spin_unlock_irqrestore(&sm750_dev->bl.count_lock, irqflags);

	return IRQ_HANDLED;
}

static int sm750_bl_gpio_init(struct sm750_dev *sm750_dev)
{
	struct fwnode_handle *fwnode;
	struct device_node *devnode;
	int ret = 0;
	u32 pwm;

	/* Silently fail if no specific nodes/properties found */
	devnode = of_find_node_by_name(NULL, "platform_setting");
	if (!devnode)
		return -ENOENT;

	fwnode = &(devnode->fwnode);

	if (of_property_read_u32(devnode, "sm750,backlight-pwm", &pwm)) {
		ret = -ENOENT;
		goto err_node_put;
	}

	if (pwm >= 3) {
		ret = -EINVAL;
		pr_err("sm750: invalid pwm id %u (must be 0, 1 or 2)\n", pwm);
		goto err_node_put;
	}
	sm750_dev->bl.pwm = pwm;

	sm750_dev->bl.up = fwnode_get_named_gpiod(fwnode, "sm750,backlight-gpios",
						  0, GPIOD_IN, "bri_up");
	if (IS_ERR(sm750_dev->bl.up)) {
                ret = PTR_ERR(sm750_dev->bl.up);
		pr_err("sm750: no backlight up gpio found\n");
		goto err_node_put;
	}

	sm750_dev->bl.down = fwnode_get_named_gpiod(fwnode, "sm750,backlight-gpios",
						    1, GPIOD_IN, "bri_down");
	if (IS_ERR(sm750_dev->bl.down)) {
		gpiod_put(sm750_dev->bl.up);
                ret = PTR_ERR(sm750_dev->bl.down);
		pr_err("sm750: no backlight down gpio found\n");
	}

err_node_put:
	of_node_put(devnode);

	return ret;
}

static void sm750_bl_gpio_clear(struct sm750_dev *sm750_dev)
{
	gpiod_put(sm750_dev->bl.down);
	gpiod_put(sm750_dev->bl.up);
}

static void sm750_bl_init_pwm(struct sm750_dev *sm750_dev)
{
	unsigned int ulReg, base;

	base = PWM_0 + 4*sm750_dev->bl.pwm;

	/* Brrr, helper is a macro... */
	ulReg = PEEK32(GPIO_MUX);
	if (sm750_dev->bl.pwm == 2)
		ulReg = FIELD_SET(ulReg, GPIO_MUX, 19, PWM);
	else if (sm750_dev->bl.pwm == 1)
		ulReg = FIELD_SET(ulReg, GPIO_MUX, 18, PWM);
	else
		ulReg = FIELD_SET(ulReg, GPIO_MUX, 17, PWM);
	POKE32(GPIO_MUX, ulReg);

	ulReg = PEEK32(base);
	ulReg = FIELD_SET(ulReg, PWM_0, STATUS, ENABLED);
	POKE32(base, ulReg);
}

static int sm750_bl_irq_init(struct sm750_dev *sm750_dev)
{
	int ret, irq_up, irq_down;

	spin_lock_init(&sm750_dev->bl.count_lock);

	irq_up = gpiod_to_irq(sm750_dev->bl.up);
	irq_down = gpiod_to_irq(sm750_dev->bl.down);
	if (irq_up < 0 || irq_down < 0) {
		pr_err("sm750: gpio to irq mapping failed\n");
		return irq_up < 0 ? irq_up : irq_down;
	}

	ret = request_threaded_irq(irq_up, NULL, sm750_bl_up_isr,
		IRQF_TRIGGER_RISING | IRQF_ONESHOT, "bri_up", sm750_dev);
	if (ret) {
		pr_err("sm750: failed to set bri_up isr\n");
		return ret;
	}

	ret = request_threaded_irq(irq_down, NULL, sm750_bl_down_isr,
		IRQF_TRIGGER_RISING | IRQF_ONESHOT, "bri_up", sm750_dev);
	if (ret) {
		free_irq(irq_up, sm750_dev);
		pr_err("sm750: failed to set bri_down isr\n");
		return ret;
	}

	return 0;
}

static void sm750_bl_irq_clear(struct sm750_dev *sm750_dev)
{
	free_irq(gpiod_to_irq(sm750_dev->bl.down), sm750_dev);
	free_irq(gpiod_to_irq(sm750_dev->bl.up), sm750_dev);
}

static int sm750_bl_update_status(struct backlight_device *bd)
{
	struct sm750_dev *sm750_dev = bl_get_data(bd);
	unsigned long irqflags;
	int level;

	if (bd->props.power != FB_BLANK_UNBLANK ||
	    bd->props.fb_blank != FB_BLANK_UNBLANK)
		level = 0;
	else
		level = bd->props.brightness;

	spin_lock_irqsave(&sm750_dev->bl.count_lock, irqflags);
	sm750_dev->bl.count = level;
	sm750_set_bl(sm750_dev->bl.pwm, sm750_dev->bl.count);
	spin_unlock_irqrestore(&sm750_dev->bl.count_lock, irqflags);

	return 0;
}

static const struct backlight_ops sm750_bl_ops = {
	.update_status  = sm750_bl_update_status,
};

static int sm750_bl_create_dev(struct sm750_dev *sm750_dev)
{
	struct backlight_properties props = {};

	props.type = BACKLIGHT_RAW;
	props.max_brightness = BACKLIGHT_MAX;
	sm750_dev->bl.bd = backlight_device_register("sm750bl", &sm750_dev->pdev->dev,
						     sm750_dev, &sm750_bl_ops, &props);
	if (IS_ERR(sm750_dev->bl.bd)) {
		pr_err("sm750: backlight device registration failed\n");
		return PTR_ERR(sm750_dev->bl.bd);
	}
	sm750_dev->bl.bd->props.brightness = BACKLIGHT_DEF;
	sm750_dev->bl.bd->props.power = FB_BLANK_UNBLANK;
	sm750_dev->bl.bd->props.fb_blank = FB_BLANK_UNBLANK;
	backlight_update_status(sm750_dev->bl.bd);

	return 0;
}

static void sm750_bl_remove_dev(struct sm750_dev *sm750_dev)
{
	backlight_device_unregister(sm750_dev->bl.bd);
}

int sm750_setup_bl(struct sm750_dev *sm750_dev)
{
	int ret;

	sm750_dev->bl.bl_registered = false;

	ret = sm750_bl_gpio_init(sm750_dev);
	if (ret)
		return ret;

	sm750_bl_init_pwm(sm750_dev);

	ret = sm750_bl_irq_init(sm750_dev);
	if (ret)
		goto err_gpio_clear;

	ret = sm750_bl_create_dev(sm750_dev);
	if (ret)
		goto err_irq_clear;

	sm750_dev->bl.bl_registered = true;

	pr_info("sm750 backlight pwm%hhu initialized\n", sm750_dev->bl.pwm);

	return 0;

err_irq_clear:
	sm750_bl_irq_clear(sm750_dev);

err_gpio_clear:
	sm750_bl_gpio_clear(sm750_dev);

	return ret;
}

void sm750_remove_bl(struct sm750_dev *sm750_dev)
{
	if (sm750_dev->bl.bl_registered) {
		sm750_bl_remove_dev(sm750_dev);
		sm750_bl_irq_clear(sm750_dev);
		sm750_bl_gpio_clear(sm750_dev);

		sm750_dev->bl.bl_registered = false;

		pr_info("sm750: backlight unloaded\n");
	}
}
