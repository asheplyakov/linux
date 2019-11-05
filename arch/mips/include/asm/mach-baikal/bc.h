/*
 * Baikal-T SOC platform support code. Boot Controller driver.
 *
 * Copyright (C) 2017 T-platforms JSC
 *
 * Author:
 *   Sergey Semin <Sergey.Semin@t-platforms.ru>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef __ASM_BAIKAL_BC_H_
#define __ASM_BAIKAL_BC_H_

#include <linux/device.h>

enum be_bc_boot_mode {
	BE_BC_ROM_MODE = 0,
	BE_BC_FLASH_MODE = 2
};

struct be_bc {
	void __iomem *regs;
};

/* API Functions */
#if IS_ENABLED(CONFIG_OF)
extern struct be_bc *of_find_be_bc_device_by_node(struct device_node *node);
#else
static inline struct be_bc *of_find_be_bc_device_by_node(struct device_node *node)
{
	return NULL;
}
#endif

extern void be_bc_enable_spi(struct be_bc *bc);
extern void be_bc_disable_spi(struct be_bc *bc);

#endif
