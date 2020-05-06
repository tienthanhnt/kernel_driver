/*
 *  drivers/net/comcerto/smi.c
 *  Shared Memory Interface support, part of Virtual Ethernet Driver
 *
 *  Copyright (C) 2009 Mindspeed Technologies
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */

#include <asm/io.h>
#include <asm/arch/msp.h>

#include "smi.h"

struct smi_fqueue	*smi_csp2msp_queue;
struct smi_fqueue	*smi_msp2csp_queue;
struct smi_fpart	*smi_eth_part;
struct smi_fpart	*smi_csme_part;
ulong			smi_ncnb_phys;

static volatile u32	*smi_csp2msp_queue_storage;	/* helper: virtual address of smi_csp2msp_queue::storage */
static volatile u32	*smi_msp2csp_queue_storage;


void smi_init(void)
{
	smi_csp2msp_queue = (struct smi_fqueue *) msp_phys_to_virt(__raw_readl(SMI_CSP2MSP_QUEUE_VADDR));
	smi_csp2msp_queue_storage = (u32 *) msp_phys_to_virt(smi_csp2msp_queue->storage);

	smi_msp2csp_queue = (struct smi_fqueue *) msp_phys_to_virt(__raw_readl(SMI_MSP2CSP_QUEUE_VADDR));
	smi_msp2csp_queue_storage = (u32 *) msp_phys_to_virt(smi_msp2csp_queue->storage);

	smi_eth_part = (struct smi_fpart *) msp_phys_to_virt(__raw_readl(SMI_ETH_PART_VADDR));
	smi_csme_part = (struct smi_fpart *) msp_phys_to_virt(__raw_readl(SMI_CSME_PART_VADDR));

	smi_ncnb_phys = __raw_readl(SMI_NCNB_START_VADDR);
}

static inline struct smi_fdesc *__smi_alloc(struct smi_fpart *fpart)
{
	struct smi_fdesc *fdesc;

	if (fpart->freeblk) {
		fdesc = msp_phys_to_virt(fpart->freeblk);
		fpart->freeblk = fdesc->next;
		fdesc->fpart = msp_virt_to_phys(fpart);
	} else
		fdesc = NULL;

	return fdesc;
}

static inline void __smi_free(struct smi_fpart *fpart, struct smi_fdesc *fdesc)
{
	fdesc->next = fpart->freeblk;
	fpart->freeblk = msp_virt_to_phys(fdesc);
}

struct smi_fdesc *smi_alloc(struct smi_fpart *fpart)
{
	struct smi_fdesc *fdesc;

	smi_lock(&fpart->lock);

	fdesc = __smi_alloc(fpart);

	smi_unlock(&fpart->lock);

	return fdesc;
}

void smi_free(struct smi_fdesc *fdesc)
{
	struct smi_fpart *fpart = msp_phys_to_virt(fdesc->fpart);

	smi_lock(&fpart->lock);

	__smi_free(fpart, fdesc);

	smi_unlock(&fpart->lock);
}

struct smi_fdesc *smi_alloc_irqsafe(struct smi_fpart *fpart)
{
	struct smi_fdesc *fdesc;
	ulong flags;

	smi_lock_irqsave(&fpart->lock, flags);

	fdesc = __smi_alloc(fpart);

	smi_unlock_irqrestore(&fpart->lock, flags);

	return fdesc;
}

void smi_free_irqsafe(struct smi_fdesc *fdesc)
{
	struct smi_fpart *fpart = msp_phys_to_virt(fdesc->fpart);
	ulong flags;

	smi_lock_irqsave(&fpart->lock, flags);

	__smi_free(fpart, fdesc);

	smi_unlock_irqrestore(&fpart->lock, flags);
}

static inline int __smi_enqueue(struct smi_fqueue *fqueue, struct smi_fdesc *fdesc)
{
	u16 put;
	int rc;

	fdesc->next = 0;

	put = fqueue->put;

	if (++put >= fqueue->size)
		put = 0;

	if (put != fqueue->get) {
		rc = 1;
		smi_csp2msp_queue_storage[fqueue->put] = msp_virt_to_phys(fdesc);
		fqueue->put = put;
	} else
		rc = 0;

	return rc;
}

static inline ulong __smi_dequeue(struct smi_fqueue *fqueue)
{
	u32 get;
	ulong phys;

	get = fqueue->get;

	if (fqueue->put != get) {
		phys = smi_msp2csp_queue_storage[get++];

		if (get == fqueue->size)
			get = 0;

		fqueue->get = get;
	} else
		phys = 0;

	return phys;
}

int smi_enqueue(struct smi_fqueue *fqueue, struct smi_fdesc *fdesc)
{
	int rc;

	smi_lock(&fqueue->lock);

	rc = __smi_enqueue(fqueue, fdesc);

	smi_unlock(&fqueue->lock);

	comcerto_softirq_set(IRQ_PTP1);

	return rc;
}

ulong smi_dequeue(struct smi_fqueue *fqueue)
{
	ulong phys;

	smi_lock(&fqueue->lock);

	phys = __smi_dequeue(fqueue);

	smi_unlock(&fqueue->lock);

	return phys;
}

int smi_enqueue_irqsafe(struct smi_fqueue *fqueue, struct smi_fdesc *fdesc)
{
	int rc;
	ulong flags;

	smi_lock_irqsave(&fqueue->lock, flags);

	rc = __smi_enqueue(fqueue, fdesc);

	smi_unlock_irqrestore(&fqueue->lock, flags);

	comcerto_softirq_set(IRQ_PTP1);

	return rc;
}

ulong smi_dequeue_irqsafe(struct smi_fqueue *fqueue)
{
	ulong phys;
	ulong flags;

	smi_lock_irqsave(&fqueue->lock, flags);

	phys = __smi_dequeue(fqueue);

	smi_unlock_irqrestore(&fqueue->lock, flags);

	return phys;
}
