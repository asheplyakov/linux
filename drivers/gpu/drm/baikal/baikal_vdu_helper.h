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

dma_addr_t baikal_vdu_fb_cma_get_gem_addr(struct drm_framebuffer *fb,
                   struct drm_plane_state *state,
                   unsigned int plane);
#endif /* __BAIKAL_VDU_HELPER_H__ */
