/*
 * Copyright (C) 2019-2020 Baikal Electronics JSC
 *
 * Author: Pavel Parkhomenko <Pavel.Parkhomenko@baikalelectronics.ru>
 *
 * This is a header file for baikal_vdu_helper.c
 */

#ifndef __BAIKAL_VDU_HELPER_H__
#define __BAIKAL_VDU_HELPER_H__

#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_fb_cma_helper.h>
#include <drm/drm_panel.h>

/**
 * struct drm_format_info - information about a DRM format
 * @format: 4CC format identifier (DRM_FORMAT_*)
 * @depth: Color depth (number of bits per pixel excluding padding bits),
 *	valid for a subset of RGB formats only. This is a legacy field, do not
 *	use in new code and set to 0 for new formats.
 * @num_planes: Number of color planes (1 to 3)
 * @cpp: Number of bytes per pixel (per plane)
 * @hsub: Horizontal chroma subsampling factor
 * @vsub: Vertical chroma subsampling factor
 */

const struct drm_format_info *baikal_vdu_format_info(u32 format);
int baikal_vdu_format_plane_cpp(uint32_t format, int plane);
int baikal_of_find_panel_or_bridge(struct device_node *remote,
				   struct drm_panel **panel,
				   struct drm_bridge **bridge);
dma_addr_t baikal_vdu_fb_cma_get_gem_addr(struct drm_framebuffer *fb,
                   struct drm_plane_state *state,
                   unsigned int plane);
#endif /* __BAIKAL_VDU_HELPER_H__ */
