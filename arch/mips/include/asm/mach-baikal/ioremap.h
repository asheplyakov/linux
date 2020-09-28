/*
 *      Baikal-T SOC platform support code.
 *
 *      Copyright (C) 2018 Baikal Electronics.
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License
 *	as published by the Free Software Foundation; either version
 *	2 of the License, or (at your option) any later version.
 */
#ifndef __ASM_MACH_BAIKAL_IOREMAP_H
#define __ASM_MACH_BAIKAL_IOREMAP_H

#include <linux/types.h>
#include <asm/addrspace.h>

#ifdef CONFIG_PCI
#include <pci-t1.h>
#endif

/*
 * Allow physical addresses to be fixed up to help peripherals located
 * outside the low 32-bit range -- generic pass-through version.
 */
static inline phys_addr_t fixup_bigphys_addr(phys_addr_t phys_addr, phys_addr_t size)
{
	return phys_addr;
}

static inline void __iomem *plat_ioremap(phys_addr_t offset, unsigned long size,
	unsigned long flags)
{
#ifdef CONFIG_PCI
	if ((offset >= PCI_BUS_PHYS_PCIMEM_BASE_ADDR) && (offset < PCI_BUS_PHYS_PCIMEM_LIMIT_ADDR)) {
		return (void __iomem *)KSEG1ADDR(BAIKAL_MAP_PCI_BUS_TO_PADDR((unsigned long)offset));
	}
#endif	
	return NULL;
}

static inline int plat_iounmap(const volatile void __iomem *addr)
{
#ifdef CONFIG_PCI
	return ((BAIKAL_MAP_PADDR_TO_PCI_BUS(CPHYSADDR((unsigned long)addr)) >= PCI_BUS_PHYS_PCIMEM_BASE_ADDR) &&
		(BAIKAL_MAP_PADDR_TO_PCI_BUS(CPHYSADDR((unsigned long)addr)) < PCI_BUS_PHYS_PCIMEM_LIMIT_ADDR));
#else
	return 0;
#endif
}
#endif /* __ASM_MACH_BAIKAL_IOREMAP_H */
