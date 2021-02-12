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

#ifndef __BAIKAL_VDU_DRM_H__
#define __BAIKAL_VDU_DRM_H__

#include <drm/drm_gem.h>
#include <drm/drm_simple_kms_helper.h>

struct clk;
struct drm_device;
struct drm_fbdev_cma;
struct drm_panel;

struct baikal_vdu_drm_connector {
	struct drm_connector connector;
	struct drm_panel *panel;
};

struct baikal_vdu_private {
	struct drm_device *drm;

	struct baikal_vdu_drm_connector connector;
	struct drm_crtc crtc;
	struct drm_encoder encoder;
	struct drm_bridge *bridge;
	struct drm_plane primary;

	void *regs;
	struct clk *clk;
	spinlock_t lock;
	u32 counters[20];
	int mode_fixup;

	u32 fb_addr;
	u32 fb_end;
};

#define to_baikal_vdu_drm_connector(x) \
	container_of(x, struct baikal_vdu_drm_connector, connector)

extern const struct drm_encoder_funcs baikal_vdu_encoder_funcs;

/* CRTC Functions */
int baikal_vdu_crtc_create(struct drm_device *dev);
irqreturn_t baikal_vdu_irq(int irq, void *data);

int baikal_vdu_primary_plane_init(struct drm_device *dev);

/* Connector Functions */
int baikal_vdu_connector_create(struct drm_device *dev);

/* Encoder Functions */
int baikal_vdu_encoder_init(struct drm_device *dev);

/* GEM Functions */
int baikal_vdu_dumb_create(struct drm_file *file_priv,
		      struct drm_device *dev,
		      struct drm_mode_create_dumb *args);

void baikal_vdu_debugfs_init(struct drm_minor *minor);

#endif /* __BAIKAL_VDU_DRM_H__ */
