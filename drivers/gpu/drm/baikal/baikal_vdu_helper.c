/*
 * Copyright (C) 2019-2020 Baikal Electronics JSC
 *
 * Author: Pavel Parkhomenko <Pavel.Parkhomenko@baikalelectronics.ru>
 *
 * This file contains backports of DRM structures and functions
 * which are absent in 4.9 kernel but required by Baikal VDU DRM driver.
 */
#include <drm/drm_encoder.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_gem.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_fb_cma_helper.h>
#include <drm/drm_panel.h>
#include <drm/drm_of.h>

/**
 * baikal_vdu_format_plane_cpp - determine the bytes per pixel value
 * @format: pixel format (DRM_FORMAT_*)
 * @plane: plane index
 *
 * Returns:
 * The bytes per pixel value for the specified plane.
 */
static int baikal_vdu_format_plane_cpp(uint32_t format, int plane)
{
	const struct drm_format_info *info;

	info = drm_format_info(format);
	if (!info || plane >= info->num_planes)
		return 0;

	return info->cpp[plane];
}

/**
 * baikal_vdu_fb_cma_get_gem_addr() - Get physical address for framebuffer
 * @fb: The framebuffer
 * @state: Which state of drm plane
 * @plane: Which plane
 * Return the CMA GEM address for given framebuffer.
 *
 * This function will usually be called from the PLANE callback functions.
 */
dma_addr_t baikal_vdu_fb_cma_get_gem_addr(struct drm_framebuffer *fb,
                   struct drm_plane_state *state,
                   unsigned int plane)
{
	dma_addr_t paddr;

	if (plane >= 4)
		return 0;

	paddr = drm_fb_cma_get_gem_obj(fb, plane)->paddr + fb->offsets[plane];
	paddr += baikal_vdu_format_plane_cpp(fb->format->format, plane) * (state->src_x >> 16);
	paddr += fb->pitches[plane] * (state->src_y >> 16);

	return paddr;
}
