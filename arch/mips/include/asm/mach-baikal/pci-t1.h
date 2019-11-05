/*
 *  Baikal-T SOC platform support code.
 *
 *  Copyright (C) 2015-2018 Baikal Electronics.
 *
 *  This program is free software; you can distribute it and/or modify it
 *  under the terms of the GNU General Public License (Version 2) as
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope it will be useful, but WITHOUT
 *  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 *  for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place - Suite 330, Boston MA 02111-1307, USA.
 *
 *  BAIKAL MIPS boards specific PCI support.
 */

#ifndef __ASM_MACH_BAIKAL_PCI_T1_H__
#define __ASM_MACH_BAIKAL_PCI_T1_H__

#define	PHYS_PCI_START_ADDR		(0x08000000)
#define	PHYS_PCI_END_ADDR		(0x1BDC0000)

#define PCI_BUS_ALIGN_OFFSET		(0x18000000LL)
#define BAIKAL_MAP_PCI_BUS_TO_PADDR(x)  ((x) - PCI_BUS_ALIGN_OFFSET)
#define BAIKAL_MAP_PADDR_TO_PCI_BUS(x)  ((x) + PCI_BUS_ALIGN_OFFSET)

#define	PHYS_PCIMEM_BASE_ADDR		(PHYS_PCI_START_ADDR)
#define PHYS_PCIMEM_SIZE		(0x10410000)
#define	PHYS_PCIMEM_LIMIT_ADDR		(PHYS_PCIMEM_BASE_ADDR + PHYS_PCIMEM_SIZE - 1)
#define IATU_MEM_INDEX			2
#define PCI_BUS_PHYS_PCIMEM_BASE_ADDR	(PHYS_PCIMEM_BASE_ADDR + PCI_BUS_ALIGN_OFFSET)
#define PCI_BUS_PHYS_PCIMEM_LIMIT_ADDR	(PHYS_PCIMEM_LIMIT_ADDR + PCI_BUS_ALIGN_OFFSET)

#define	PHYS_PCI_RD0_BASE_ADDR		(PHYS_PCIMEM_LIMIT_ADDR + 1)
#ifdef CONFIG_PCI_ECAM
#define PHYS_PCI_RD0_SIZE		(0x00210000)
#else
#define PHYS_PCI_RD0_SIZE		(0x00010000)
#endif /* CONFIG_PCI_ECAM */
#define PHYS_PCI_RD0_LIMIT_ADDR		(PHYS_PCI_RD0_BASE_ADDR + PHYS_PCI_RD0_SIZE - 1)
#define IATU_RD0_INDEX			0

#define PHYS_PCI_RD1_BASE_ADDR		(PHYS_PCI_RD0_LIMIT_ADDR + 1)
#ifdef CONFIG_PCI_ECAM
#define PHYS_PCI_RD1_SIZE		(0x02F00000)
#else
#define PHYS_PCI_RD1_SIZE		(0x00010000)
#endif /* CONFIG_PCI_ECAM */
#define PHYS_PCI_RD1_LIMIT_ADDR		(PHYS_PCI_RD1_BASE_ADDR + PHYS_PCI_RD1_SIZE - 1)
#define IATU_RD1_INDEX			1

#define PHYS_PCI_MSI_SIZE		(0x00010000)
#define	PHYS_PCI_MSI_BASE_ADDR		(PHYS_PCI_END_ADDR - PHYS_PCI_MSI_SIZE)

#define PHYS_PCIIO_BASE_ADDR 		(PHYS_PCI_RD1_LIMIT_ADDR + 1)
#define PHYS_PCIIO_LIMIT_ADDR		(PHYS_PCI_MSI_BASE_ADDR - 1)
#define PHYS_PCIIO_SIZE			(PHYS_PCIIO_LIMIT_ADDR - PHYS_PCIIO_BASE_ADDR)
#define IATU_IO_INDEX			3

#endif /* __ASM_MACH_BAIKAL_PCI_T1_H__ */
