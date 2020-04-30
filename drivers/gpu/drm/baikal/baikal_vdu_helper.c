/*
 * Copyright (C) 2019-2020 Baikal Electronics JSC
 *
 * Author: Pavel Parkhomenko <Pavel.Parkhomenko@baikalelectronics.ru>
 *
 * This file contains backports of DRM structures and functions
 * which are absent in 4.9 kernel but required by Baikal VDU DRM driver.
 */
#include <drm/drm_encoder.h>
#include <drm/drm_gem.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_fb_cma_helper.h>
#include <drm/drm_panel.h>
#include <drm/drm_of.h>

struct drm_format_info {
	u32 format;
	u8 depth;
	u8 num_planes;
	u8 cpp[3];
	u8 hsub;
	u8 vsub;
};

const struct drm_format_info *__drm_format_info(u32 format);

/**
 * baikal_vdu_format_info - query information for a given format
 * @format: pixel format (DRM_FORMAT_*)
 *
 * The caller should only pass a supported pixel format to this function.
 * Unsupported pixel formats will generate a warning in the kernel log.
 *
 * Returns:
 * The instance of struct drm_format_info that describes the pixel format, or
 * NULL if the format is unsupported.
 */
const struct drm_format_info *baikal_vdu_format_info(u32 format)
{
	const struct drm_format_info *info;

	info = __drm_format_info(format);
	WARN_ON(!info);
	return info;
}

/**
 * baikal_vdu_format_plane_cpp - determine the bytes per pixel value
 * @format: pixel format (DRM_FORMAT_*)
 * @plane: plane index
 *
 * Returns:
 * The bytes per pixel value for the specified plane.
 */
int baikal_vdu_format_plane_cpp(uint32_t format, int plane)
{
	const struct drm_format_info *info;

	info = baikal_vdu_format_info(format);
	if (!info || plane >= info->num_planes)
		return 0;

	return info->cpp[plane];
}

int baikal_of_find_panel_or_bridge(struct device_node *remote,
				   struct drm_panel **panel,
				   struct drm_bridge **bridge)
{
	int ret = -EPROBE_DEFER;
	if (!remote)
		return -EINVAL;

	if (!panel && !bridge)
		return -EINVAL;


	if (panel) {
		DRM_DEBUG_DRIVER("attempt to find panel for %s ... ", remote->full_name);
		*panel = of_drm_find_panel(remote);
		if (!IS_ERR(*panel)) {
			ret = 0;
			DRM_DEBUG_DRIVER("OK");
		} else {
			ret = PTR_ERR(*panel);
			*panel = NULL;
			DRM_DEBUG_DRIVER("NOPE\n");
		}
	}

	/* No panel found yet, check for a bridge next. */
	if (bridge) {
		if (ret) {
			DRM_DEBUG_DRIVER("attempt to find bridge for %s ... ", remote->full_name);
			*bridge = of_drm_find_bridge(remote);
			if (*bridge) {
				DRM_DEBUG_DRIVER("OK\n");
				ret = 0;
			} else {
				DRM_DEBUG_DRIVER("NOPE\n");
			}
		} else {
			*bridge = NULL;
		}

	}

	of_node_put(remote);
	return ret;
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
