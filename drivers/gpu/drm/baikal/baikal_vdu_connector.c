/*
 * Copyright (C) 2019-2020 Baikal Electronics JSC
 *
 * Author: Pavel Parkhomenko <Pavel.Parkhomenko@baikalelectronics.ru>
 *
 * Parts of this file were based on sources as follows:
 *
 * Copyright (c) 2006-2008 Intel Corporation
 * Copyright (c) 2007 Dave Airlie <airlied@linux.ie>
 * Copyright (C) 2011 Texas Instruments
 * (C) COPYRIGHT 2012-2013 ARM Limited. All rights reserved.
 *
 * This program is free software and is provided to you under the terms of the
 * GNU General Public License version 2 as published by the Free Software
 * Foundation, and any use by you of this program is subject to the terms of
 * such GNU licence.
 *
 */

/**
 * baikal_vdu_connector.c
 * Implementation of the connector functions for Baikal Electronics BE-M1000 SoC's VDU
 */

#include <linux/clk.h>
#include <linux/input.h>
#include <linux/module.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_of.h>
#include <drm/drm_panel.h>
#include <drm/drm_probe_helper.h>

#include "baikal_vdu_drm.h"
#include "baikal_vdu_regs.h"

#define BAIKAL_VDU_MIN_BRIGHTNESS	0
#define BAIKAL_VDU_DEFAULT_BRIGHTNESS	50
#define BAIKAL_VDU_BRIGHTNESS_STEP	5
#define BAIKAL_VDU_DEFAULT_PWM_FREQ	10000


#define to_baikal_vdu_private(x) \
	container_of(x, struct baikal_vdu_private, connector)

static void baikal_vdu_drm_connector_destroy(struct drm_connector *connector)
{
	struct baikal_vdu_private *priv = to_baikal_vdu_private(connector);

	if (priv->panel)
		drm_panel_detach(priv->panel);

	drm_connector_unregister(connector);
	drm_connector_cleanup(connector);
}

static enum drm_connector_status baikal_vdu_drm_connector_detect(
		struct drm_connector *connector, bool force)
{
	struct baikal_vdu_private *priv = to_baikal_vdu_private(connector);

	return (priv->panel ?
		connector_status_connected :
		connector_status_disconnected);
}

static int baikal_vdu_drm_connector_helper_get_modes(
		struct drm_connector *connector)
{
	struct baikal_vdu_private *priv = to_baikal_vdu_private(connector);

	if (!priv->panel)
		return 0;

	return drm_panel_get_modes(priv->panel);
}

const struct drm_connector_funcs connector_funcs = {
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = baikal_vdu_drm_connector_destroy,
	.detect = baikal_vdu_drm_connector_detect,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

const struct drm_connector_helper_funcs connector_helper_funcs = {
	.get_modes = baikal_vdu_drm_connector_helper_get_modes,
};

static const struct drm_encoder_funcs encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

int baikal_vdu_lvds_connector_create(struct drm_device *dev)
{
	struct baikal_vdu_private *priv = dev->dev_private;
	struct drm_connector *connector = &priv->connector;
	struct drm_encoder *encoder = &priv->encoder;
	int ret = 0;

	ret = drm_connector_init(dev, connector, &connector_funcs,
				 DRM_MODE_CONNECTOR_LVDS);
	if (ret) {
		dev_err(dev->dev, "drm_connector_init failed: %d\n", ret);
		goto out;
	}
	drm_connector_helper_add(connector, &connector_helper_funcs);
	ret = drm_panel_attach(priv->panel, connector);
	if (ret) {
		dev_err(dev->dev, "drm_panel_attache returned %d\n", ret);
		goto out;
	}
	ret = drm_encoder_init(dev, encoder, &encoder_funcs,
			       DRM_MODE_ENCODER_LVDS, NULL);
	if (ret) {
		dev_err(dev->dev, "drm_encoder_init failed: %d\n", ret);
		goto out;
	}
	encoder->crtc = &priv->crtc;
	encoder->possible_crtcs = drm_crtc_mask(encoder->crtc);
	ret = drm_connector_attach_encoder(connector, encoder);
	if (ret) {
		dev_err(dev->dev, "drm_connector_attach_encoder failed: %d\n", ret);
		goto out;
	}
	ret = drm_connector_register(connector);
	if (ret) {
		dev_err(dev->dev, "drm_connector_register failed: %d\n", ret);
		goto out;
	}
out:
	return ret;
}

static int baikal_vdu_backlight_update_status(struct backlight_device *bl_dev)
{
	struct baikal_vdu_private *priv = bl_get_data(bl_dev);
	int brightness_on = 1;
	int brightness = bl_dev->props.brightness;
	u8 pwmdc;

	if (bl_dev->props.power != FB_BLANK_UNBLANK ||
	    bl_dev->props.fb_blank != FB_BLANK_UNBLANK ||
	    bl_dev->props.state & (BL_CORE_SUSPENDED | BL_CORE_FBBLANK)) {
		brightness_on = 0;
		brightness = priv->min_brightness;
	}

	if (priv->enable_gpio)
		gpiod_set_value_cansleep(priv->enable_gpio, brightness_on);

	pwmdc = brightness ? ((brightness << 6) / 25 - 1) : 0;

	writel(pwmdc, priv->regs + PWMDCR);

	return 0;
}

static const struct backlight_ops baikal_vdu_backlight_ops = {
	.options        = BL_CORE_SUSPENDRESUME,
	.update_status	= baikal_vdu_backlight_update_status,
};

static void baikal_vdu_input_event(struct input_handle *handle,
				   unsigned int type, unsigned int code,
				   int value)
{
	struct baikal_vdu_private *priv = handle->private;
	int brightness;

	if (type != EV_KEY || value == 0)
		return;

	switch (code) {
	case KEY_BRIGHTNESSDOWN:
		brightness = priv->bl_dev->props.brightness -
			     priv->brightness_step;
		if (brightness >= priv->min_brightness)
			backlight_device_set_brightness(priv->bl_dev,
							brightness);
		break;

	case KEY_BRIGHTNESSUP:
		brightness = priv->bl_dev->props.brightness +
			     priv->brightness_step;
		backlight_device_set_brightness(priv->bl_dev, brightness);
		break;

	case KEY_BRIGHTNESS_TOGGLE:
		priv->brightness_on = !priv->brightness_on;
		if (priv->brightness_on)
			backlight_enable(priv->bl_dev);
		else
			backlight_disable(priv->bl_dev);
		break;

	default:
		return;
	}

	backlight_force_update(priv->bl_dev, BACKLIGHT_UPDATE_HOTKEY);
}

static int baikal_vdu_input_connect(struct input_handler *handler,
				    struct input_dev *dev,
				    const struct input_device_id *id)
{
	struct input_handle *handle;
	int ret;

	handle = kzalloc(sizeof(*handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->private = handler->private;
	handle->name = KBUILD_MODNAME;
	handle->dev = dev;
	handle->handler = handler;

	ret = input_register_handle(handle);
	if (ret)
		goto err_free_handle;

	ret = input_open_device(handle);
	if (ret)
		goto err_unregister_handle;

	return 0;

err_unregister_handle:
	input_unregister_handle(handle);
err_free_handle:
	kfree(handle);
	return ret;
}

static void baikal_vdu_input_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static const struct input_device_id baikal_vdu_input_ids[] = {
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT,
		.evbit = { BIT_MASK(EV_KEY) },
	},

	{ },    /* Terminating entry */
};

MODULE_DEVICE_TABLE(input, baikal_vdu_input_ids);

int baikal_vdu_backlight_create(struct drm_device *drm)
{
	struct baikal_vdu_private *priv = drm->dev_private;
	struct device *dev = drm->dev;
	struct backlight_properties props;
	struct input_handler *handler;
	struct device_node *node;
	u32 min_brightness = BAIKAL_VDU_MIN_BRIGHTNESS;
	u32 dfl_brightness = BAIKAL_VDU_DEFAULT_BRIGHTNESS;
	u32 brightness_step = BAIKAL_VDU_BRIGHTNESS_STEP;
	u32 pwm_frequency = 0;
	int ret = 0;
	unsigned long rate;
	unsigned int pwmfr = 0;

	priv->enable_gpio = devm_gpiod_get_optional(dev, "enable", GPIOD_ASIS);
	if (IS_ERR(priv->enable_gpio)) {
		dev_warn(dev, "failed to get ENABLE GPIO\n");
		priv->enable_gpio = NULL;
	}

	if (priv->enable_gpio && gpiod_get_direction(priv->enable_gpio) != 0)
		gpiod_direction_output(priv->enable_gpio, 1);

	node = of_get_child_by_name(dev->of_node, "backlight");
	if (!node)
		return 0;

	of_property_read_u32(node, "min-brightness-level", &min_brightness);
	of_property_read_u32(node, "default-brightness-level", &dfl_brightness);
	of_property_read_u32(node, "brightness-level-step", &brightness_step);
	of_property_read_u32(node, "pwm-frequency", &pwm_frequency);

	if (pwm_frequency == 0) {
		dev_warn(dev, "using default PWM frequency %u\n",
			 BAIKAL_VDU_DEFAULT_PWM_FREQ);
		pwm_frequency = BAIKAL_VDU_DEFAULT_PWM_FREQ;
	}

	memset(&props, 0, sizeof(props));
	props.max_brightness = 100;
	props.type = BACKLIGHT_RAW;
	props.scale = BACKLIGHT_SCALE_LINEAR;

	if (min_brightness > props.max_brightness) {
		dev_warn(dev, "invalid min brightness level: %u, using %u\n",
			 min_brightness, props.max_brightness);
		min_brightness = props.max_brightness;
	}

	if (dfl_brightness > props.max_brightness ||
	    dfl_brightness < min_brightness) {
		dev_warn(dev,
			 "invalid default brightness level: %u, using %u\n",
			 dfl_brightness, props.max_brightness);
		dfl_brightness = props.max_brightness;
	}

	priv->min_brightness = min_brightness;
	priv->brightness_step = brightness_step;
	priv->brightness_on = true;

	props.brightness = dfl_brightness;
	props.power = FB_BLANK_UNBLANK;

	priv->bl_dev =
		devm_backlight_device_register(dev, dev_name(dev), dev, priv,
					       &baikal_vdu_backlight_ops,
					       &props);
	if (IS_ERR(priv->bl_dev)) {
		dev_err(dev, "failed to register backlight device\n");
		ret = PTR_ERR(priv->bl_dev);
		priv->bl_dev = NULL;
		goto out;
	}

	handler = devm_kzalloc(dev, sizeof(*handler), GFP_KERNEL);
	if (!handler) {
		dev_err(dev, "failed to allocate input handler\n");
		ret = -ENOMEM;
		goto out;
	}

	handler->private = priv;
	handler->event = baikal_vdu_input_event;
	handler->connect = baikal_vdu_input_connect;
	handler->disconnect = baikal_vdu_input_disconnect;
	handler->name = KBUILD_MODNAME;
	handler->id_table = baikal_vdu_input_ids;

	ret = input_register_handler(handler);
	if (ret) {
		dev_err(dev, "failed to register input handler\n");
		goto out;
	}

	/* Hold PWM Clock Domain Reset, disable clocking */
	writel(0, priv->regs + PWMFR);

	rate = clk_get_rate(priv->clk);
	pwmfr |= PWMFR_PWMFCD(rate / pwm_frequency - 1) | PWMFR_PWMFCI;
	writel(pwmfr, priv->regs + PWMFR);

	/* Release PWM Clock Domain Reset, enable clocking */
	writel(pwmfr | PWMFR_PWMPCR | PWMFR_PWMFCE, priv->regs + PWMFR);

	backlight_update_status(priv->bl_dev);
out:
	of_node_put(node);
	return ret;
}
