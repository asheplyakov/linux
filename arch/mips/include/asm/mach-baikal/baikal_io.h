/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 1994, 1995 Waldorf GmbH
 * Copyright (C) 1994 - 2000, 06 Ralf Baechle
 * Copyright (C) 1999, 2000 Silicon Graphics, Inc.
 * Copyright (C) 2004, 2005  MIPS Technologies, Inc.  All rights reserved.
 *	Author: Maciej W. Rozycki <macro@mips.com>
 */
#ifndef _BAIKAL_IO_H
#define _BAIKAL_IO_H

#define __BUILD_MEMORY_SINGLE_BE(pfx, bwlq, type, irq)				\
										\
static inline void pfx##write##bwlq(type val,					\
				volatile void __iomem *mem)			\
{                                                                              	\
	volatile type *__mem;							\
	type __val;								\
										\
	__mem = (void *)__swizzle_addr_##bwlq((unsigned long)(mem));		\
										\
	__val = pfx##ioswab##bwlq(__mem, val);					\
										\
	if (sizeof(type) != sizeof(u64) || sizeof(u64) == sizeof(long))	        \
		*__mem = __val;							\
	else{								        \
		*__mem = __val;							\
		*(__mem + 1)= __val >> 32;				        \
		smp_wmb();							\
	}							 		\
}										\
										\
static inline type pfx##read##bwlq(const volatile void __iomem *mem)	  	\
{									   	\
	volatile type *__mem;						   	\
	type __val;								\
	u32 low, high;								\
									   	\
	__mem = (void *)__swizzle_addr_##bwlq((unsigned long)(mem));	   	\
									  	\
	if (sizeof(type) != sizeof(u64) || sizeof(u64) == sizeof(long))	        \
		__val = *__mem;						   	\
	else{								   	\
		low = *__mem;							\
		high = *(__mem + 1);						\
		__val = (u64)((low + ((u64)high << 32)));			\
		smp_rmb();							\
	}									\
									   	\
	return pfx##ioswab##bwlq(__mem, __val);					\
}									   	\

#define __BUILD_IOPORT_SINGLE_BE(pfx, bwlq, type, p, slow)			\
										\
static inline void pfx##out##bwlq##p(type val, unsigned long port)		\
{										\
	volatile type *__addr;							\
	type __val;								\
										\
	war_octeon_io_reorder_wmb();						\
										\
	__addr = (void *)__swizzle_addr_##bwlq(mips_io_port_base + port); 	\
										\
	__val = pfx##ioswab##bwlq(__addr, val);					\
										\
	/* Really, we want this to be atomic */					\
	BUILD_BUG_ON(sizeof(type) > sizeof(unsigned long));			\
										\
	*__addr = __val;							\
	slow;									\
}										\
										\
static inline type pfx##in##bwlq##p(unsigned long port)				\
{										\
	volatile type *__addr;							\
	type __val;								\
										\
	__addr = (void *)__swizzle_addr_##bwlq(mips_io_port_base + port); 	\
										\
	BUILD_BUG_ON(sizeof(type) > sizeof(unsigned long));			\
										\
	__val = *__addr;							\
	slow;									\
										\
	/* prevent prefetching of coherent DMA data prematurely */		\
	rmb();									\
	return pfx##ioswab##bwlq(__addr, __val);				\
}
#define __BUILD_MEMORY_PFX_BE(bus, bwlq, type)					\
										\
__BUILD_MEMORY_SINGLE_BE(bus, bwlq, type, 1)

#define BUILDIO_MEM_BE(bwlq, type)						\
										\
__BUILD_MEMORY_PFX_BE(__raw_, bwlq, type)					\
__BUILD_MEMORY_PFX_BE(, bwlq, type)						\
__BUILD_MEMORY_PFX_BE(__mem_, bwlq, type)					\

#endif /* BAIKAL_IO_H */
