// SPDX-License-Identifier: GPL-2.0
/*
 * Linux 4.4.x <- 4.19.x compatibility header file
 *
 * Copyright (C) 2018 T-platforms JSC (fancer.lancer@gmail.com)
 */

#ifndef KERNEL_COMPAT_H
#define KERNEL_COMPAT_H

#include <linux/version.h>
#include <linux/spi/spi.h>
#include <linux/pci.h>
#include <linux/msi.h>
#include <linux/kernel.h>
#include <linux/dmaengine.h>

/* Mac has been available in since kernel 4.8.0 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,8,0)
#define PCI_IRQ_LEGACY          (1 << 0) /* allow legacy interrupts */
#define PCI_IRQ_MSI             (1 << 1) /* allow MSI interrupts */
#define PCI_IRQ_MSIX            (1 << 2) /* allow MSI-X interrupts */
#define PCI_IRQ_ALL_TYPES \
	(PCI_IRQ_LEGACY | PCI_IRQ_MSI | PCI_IRQ_MSIX)
#endif

/* This method was exported only sincekernel 4.20 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,20,0)
#if IS_ENABLED(CONFIG_OF)
static inline int __spi_of_device_match(struct device *dev, void *data)
{
	return dev->of_node == data;
}

/* must call put_device() when done with returned spi_device device */
static inline struct spi_device *of_find_spi_device_by_node(struct device_node *node)
{
	struct device *dev = bus_find_device(&spi_bus_type, NULL, node,
						__spi_of_device_match);
	return dev ? to_spi_device(dev) : NULL;
}
#endif /* IS_ENABLED(CONFIG_OF) */
#endif

/**
 * pci_alloc_irq_vectors - allocate multiple IRQs for a device
 * @dev:                PCI device to operate on
 * @min_vecs:           minimum number of vectors required (must be >= 1)
 * @max_vecs:           maximum (desired) number of vectors
 * @flags:              flags or quirks for the allocation
 *
 * Allocate up to @max_vecs interrupt vectors for @dev, using MSI-X or MSI
 * vectors if available, and fall back to a single legacy vector
 * if neither is available.  Return the number of vectors allocated,
 * (which might be smaller than @max_vecs) if successful, or a negative
 * error code on error. If less than @min_vecs interrupt vectors are
 * available for @dev the function will fail with -ENOSPC.
 *
 * To get the Linux IRQ number used for a vector that can be passed to
 * request_irq() use the pci_irq_vector() helper.
 *
 * Available since kernel 4.8.0
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,8,0)
static inline int pci_alloc_irq_vectors(struct pci_dev *dev, unsigned int min_vecs,
					unsigned int max_vecs, unsigned int flags)
{
	int vecs = -ENOSPC;

	if (flags & PCI_IRQ_MSIX) {
		vecs = pci_enable_msix_range(dev, NULL, min_vecs, max_vecs);
		if (vecs > 0)
			return vecs;
	}

	if (flags & PCI_IRQ_MSI) {
		vecs = pci_enable_msi_range(dev, min_vecs, max_vecs);
		if (vecs > 0)
			return vecs;
	}

	/* use legacy irq if allowed */
	if ((flags & PCI_IRQ_LEGACY) && min_vecs == 1) {
		pci_intx(dev, 1);
		return 1;
	}

	return vecs;
}
#endif

/**
 * pci_free_irq_vectors - free previously allocated IRQs for a device
 * @dev:                PCI device to operate on
 *
 * Undoes the allocations and enabling in pci_alloc_irq_vectors().
 *
 * Available since kernel 4.8.0
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,8,0)
static inline void pci_free_irq_vectors(struct pci_dev *dev)
{
	pci_disable_msix(dev);
	pci_disable_msi(dev);
	pci_intx(dev, 0);
}
#endif

/**
 * pci_irq_vector - return Linux IRQ number of a device vector
 * @dev: PCI device to operate on
 * @nr: device-relative interrupt vector index (0-based).
 *
 * Available since kernel 4.8.0
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,8,0)
static inline int pci_irq_vector(struct pci_dev *dev, unsigned int nr)
{
	if (dev->msix_enabled) {
		struct msi_desc *entry;
		int i = 0;

		for_each_pci_msi_entry(entry, dev) {
			if (i == nr)
				return entry->irq;
			i++;
		}
		WARN_ON_ONCE(1);
		return -EINVAL;
	}

	if (dev->msi_enabled) {
		struct msi_desc *entry = first_pci_msi_entry(dev);

		if (WARN_ON_ONCE(nr >= entry->nvec_used))
			return -EINVAL;
	} else {
		if (WARN_ON_ONCE(nr > 0))
			return -EINVAL;
	}

	return dev->irq + nr;
}
#endif

/*
 * bitmap_from_u64 - Check and swap words within u64.
 *  @mask: source bitmap
 *  @dst:  destination bitmap
 *
 * In 32-bit Big Endian kernel, when using (u32 *)(&val)[*]
 * to read u64 mask, we will get the wrong word.
 * That is "(u32 *)(&val)[0]" gets the upper 32 bits,
 * but we expect the lower 32-bits of u64.
 *
 * Available since kernel 4.9.0
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,9,0)
static inline void bitmap_from_u64(unsigned long *dst, u64 mask)
{
	dst[0] = mask & ULONG_MAX;

	if (sizeof(mask) > sizeof(unsigned long))
		dst[1] = mask >> 32;
}
#endif

/* Callback wrapper has been available since kernel 4.11.0 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,11,0)
static inline struct dma_async_tx_descriptor *dmaengine_prep_dma_memcpy(
		struct dma_chan *chan, dma_addr_t dest, dma_addr_t src,
		size_t len, unsigned long flags)
{
	if (!chan || !chan->device || !chan->device->device_prep_dma_memcpy)
		return NULL;

	return chan->device->device_prep_dma_memcpy(chan, dest, src,
						    len, flags);
}
#endif

/**
 * dmaengine_terminate_async() - Terminate all active DMA transfers
 * @chan: The channel for which to terminate the transfers
 *
 * Calling this function will terminate all active and pending descriptors
 * that have previously been submitted to the channel. It is not guaranteed
 * though that the transfer for the active descriptor has stopped when the
 * function returns. Furthermore it is possible the complete callback of a
 * submitted transfer is still running when this function returns.
 *
 * dmaengine_synchronize() needs to be called before it is safe to free
 * any memory that is accessed by previously submitted descriptors or before
 * freeing any resources accessed from within the completion callback of any
 * perviously submitted descriptors.
 *
 * This function can be called from atomic context as well as from within a
 * complete callback of a descriptor submitted on the same channel.
 *
 * If none of the two conditions above apply consider using
 * dmaengine_terminate_sync() instead.
 *
 * Available since kernel 4.5.0
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,5,0)
static inline int dmaengine_terminate_async(struct dma_chan *chan)
{
	if (chan->device->device_terminate_all)
		return chan->device->device_terminate_all(chan);

	return -EINVAL;
}
#endif

/**
 * dmaengine_synchronize() - Synchronize DMA channel termination
 * @chan: The channel to synchronize
 *
 * Synchronizes to the DMA channel termination to the current context. When this
 * function returns it is guaranteed that all transfers for previously issued
 * descriptors have stopped and and it is safe to free the memory assoicated
 * with them. Furthermore it is guaranteed that all complete callback functions
 * for a previously submitted descriptor have finished running and it is safe to
 * free resources accessed from within the complete callbacks.
 *
 * The behavior of this function is undefined if dma_async_issue_pending() has
 * been called between dmaengine_terminate_async() and this function.
 *
 * This function must only be called from non-atomic context and must not be
 * called from within a complete callback of a descriptor submitted on the same
 * channel.
 *
 * Available since kernel 4.5.0
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,5,0)
static inline void dmaengine_synchronize(struct dma_chan *chan)
{
	might_sleep();

	/*if (chan->device->device_synchronize)
		chan->device->device_synchronize(chan);
	 */
}
#endif

/**
 * dmaengine_terminate_sync() - Terminate all active DMA transfers
 * @chan: The channel for which to terminate the transfers
 *
 * Calling this function will terminate all active and pending transfers
 * that have previously been submitted to the channel. It is similar to
 * dmaengine_terminate_async() but guarantees that the DMA transfer has actually
 * stopped and that all complete callbacks have finished running when the
 * function returns.
 *
 * This function must only be called from non-atomic context and must not be
 * called from within a complete callback of a descriptor submitted on the same
 * channel.
 *
 * Available since kernel 4.5.0
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,5,0)
static inline int dmaengine_terminate_sync(struct dma_chan *chan)
{
	int ret;

	ret = dmaengine_terminate_async(chan);
	if (ret)
		return ret;

	dmaengine_synchronize(chan);

	return 0;
}
#endif

/*
 * Some headers and declrations have been moved since kernel 4.11.0
 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,11,0)
#include <linux/sched/types.h>
#include <linux/sched/signal.h>
#else
#include <linux/sched.h>
#include <linux/signal.h>
#endif

/* Kthread interface naming has been changed since 4.9.0 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,9,0)
#define init_kthread_work kthread_init_work
#define queue_kthread_work kthread_queue_work
#define flush_kthread_work kthread_flush_work
#define init_kthread_worker kthread_init_worker
#define flush_kthread_worker kthread_flush_worker
#endif

/* SPI-master naming has been changed to SPI-controller since 4.13.0 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,13,0)
#define spi_controller				spi_master

#define SPI_CONTROLLER_HALF_DUPLEX		SPI_MASTER_HALF_DUPLEX
#define SPI_CONTROLLER_NO_RX			SPI_MASTER_NO_RX
#define SPI_CONTROLLER_NO_TX			SPI_MASTER_NO_TX
#define SPI_CONTROLLER_MUST_RX			SPI_MASTER_MUST_RX
#define SPI_CONTROLLER_MUST_TX			SPI_MASTER_MUST_TX

#define spi_controller_get_devdata(_ctlr)	spi_master_get_devdata(_ctlr)
#define spi_controller_set_devdata(_ctlr, _data) \
	spi_master_set_devdata(_ctlr, _data)
#define spi_controller_get(_ctlr)		spi_master_get(_ctlr)
#define spi_controller_put(_ctlr)		spi_master_put(_ctlr)
#define spi_controller_suspend(_ctlr)		spi_master_suspend(_ctlr)
#define spi_controller_resume(_ctlr)		spi_master_resume(_ctlr)

#define spi_register_controller(_ctlr)		spi_register_master(_ctlr)
#define devm_spi_register_controller(_dev, _ctlr) \
	devm_spi_register_master(_dev, _ctlr)
#define spi_unregister_controller(_ctlr)	spi_unregister_master(_ctlr)
#endif

#endif /* KERNEL_COMPAT_H */
