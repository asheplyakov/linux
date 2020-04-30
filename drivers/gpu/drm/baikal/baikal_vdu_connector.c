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
#include <linux/version.h>
#include <linux/shmem_fs.h>
#include <linux/dma-buf.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_of.h>
#include <drm/drm_panel.h>
#include <drm/drm_probe_helper.h>

#include "baikal_vdu_drm.h"
#include "baikal_vdu_regs.h"
#include "baikal_vdu_helper.h"

static void baikal_vdu_drm_connector_destroy(struct drm_connector *connector)
{
	struct baikal_vdu_drm_connector *vdu_connector =
		to_baikal_vdu_drm_connector(connector);

	if (vdu_connector->panel)
		drm_panel_detach(vdu_connector->panel);

	drm_connector_unregister(connector);
	drm_connector_cleanup(connector);
}

static enum drm_connector_status baikal_vdu_drm_connector_detect(
		struct drm_connector *connector, bool force)
{
	struct baikal_vdu_drm_connector *vdu_connector =
		to_baikal_vdu_drm_connector(connector);

	return (vdu_connector->panel ?
		connector_status_connected :
		connector_status_disconnected);
}

static int baikal_vdu_drm_connector_helper_get_modes(
		struct drm_connector *connector)
{
	struct baikal_vdu_drm_connector *vdu_connector =
		to_baikal_vdu_drm_connector(connector);

	if (!vdu_connector) {
		pr_err("%s: vdu_connector == NULL\n", __func__);
		return 0;
	}
	if (!vdu_connector->panel)
		return 0;

	return drm_panel_get_modes(vdu_connector->panel, connector);
}

const struct drm_connector_funcs connector_funcs = {
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = baikal_vdu_drm_connector_destroy,
	.detect = baikal_vdu_drm_connector_detect,
	//.dpms = drm_atomic_helper_connector_dpms, // TODO enable it?
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

/* Walks the OF graph to find the endpoint node and then asks DRM 
 * to look up the panel or the bridge connected to the node found
 */
int get_panel_or_bridge(struct device *dev,
		struct drm_panel **panel, struct drm_bridge **bridge)
{
	struct device_node *endpoint = NULL;
	struct device_node *remote;
	struct device_node *old_remote = NULL;
	struct device_node *np = dev->of_node;
	int ep_count = 0;

	for_each_endpoint_of_node(np, endpoint) {

		remote = of_graph_get_remote_port_parent(endpoint);
		if (old_remote && remote != old_remote) {
			dev_err(dev, "all endpoints must be connected to the same panel or bridge %d\n", ep_count);
			of_node_put(endpoint);
			return -EINVAL;
		}

		/* don't proceed if we have an endpoint but no panel node
		 * or bridge node tied to it */
		if (!remote) {
			dev_err(dev, "no valid remote node connected to the endpoint@%d\n", ep_count);
			of_node_put(endpoint);
			return -EINVAL;
		}

		ep_count++;
		of_node_put(remote);
		old_remote = remote;
	}

	if (!ep_count) {
		dev_err(dev, "no endpoints found connected either to panel or bridge\n");
		return -EINVAL;
	}

	if (!of_device_is_available(remote)) {
		dev_err(dev, "not available for remote node\n");
		return -EINVAL;
	}

	return baikal_of_find_panel_or_bridge(remote, panel, bridge);

}

int baikal_vdu_connector_create(struct drm_device *dev)
{
	struct baikal_vdu_private *priv = dev->dev_private;
	struct baikal_vdu_drm_connector *vdu_connector = &priv->connector;
	struct drm_connector *connector = &vdu_connector->connector;

	drm_connector_init(dev, connector, &connector_funcs,
			DRM_MODE_CONNECTOR_LVDS);
	drm_connector_helper_add(connector, &connector_helper_funcs);
	drm_panel_attach(vdu_connector->panel, connector);

	return 0;
}
