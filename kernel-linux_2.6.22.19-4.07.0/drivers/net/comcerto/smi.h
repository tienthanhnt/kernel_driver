/*
 *  drivers/net/comcerto/smi.h
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

#ifndef __SMI_H
#define __SMI_H

#include <linux/kernel.h>

struct smi_fpart {
	volatile ulong	freeblk;
	ulong		storage;
	u32		blksz;
	u32		blkcnt;
	ulong		end_of_storage;
	volatile u8	lock;
	u8		reserved1;
	u16		reserved2;
};

struct smi_fqueue {
	ulong		storage;
	u32		reserved1;
	volatile u16	get;
	volatile u16	put;
	u16		size;
	volatile u8	lock;
};

struct smi_fdesc {
	ulong		next;
	u32		reserved1[37];
	ulong		fpart;
	u32		reserved2;
	u16		release;
	u16		padding;
	u32		reserved3;
	u32		length;
	u32		offset;
	u32		interface;
	u32		reserved4;
	u8		payload[1];
};


extern struct smi_fqueue	*smi_csp2msp_queue;
extern struct smi_fqueue	*smi_msp2csp_queue;
extern struct smi_fpart		*smi_eth_part;
extern struct smi_fpart		*smi_csme_part;
extern ulong			smi_ncnb_phys;


/*
 * swpb - C wrapper for SWPB ARM instruction, used for spin locks
 *
 * Note: bytep must point to non-cacheable region
 */
static inline u8 __swpb(u8 v, volatile u8 *bytep)
{
	u8 rc;

	__asm volatile ("swpb %0, %1, [%2]"
			:"=&r"(rc)
			:"r"(v), "r"(bytep)
			:"memory");
	return rc;
}

/*
 * smi_lock - spin lock
 *
 */
static inline void __smi_lock(volatile u8 *lock)
{
	u8 t;

	do {
		t = __swpb(0xFF, lock);
	} while (t == 0xFF);
}

/*
 * smi_unlock - spin unlock
 *
 */
static inline void __smi_unlock(volatile u8 *lock)
{
	__swpb(0x00, lock);
}

#define			smi_lock(lock)				__smi_lock(lock)
#define			smi_unlock(lock)			__smi_unlock(lock)
#define			smi_lock_irqsave(lock, flags)		do { local_irq_save(flags); __smi_lock(lock);		} while (0)
#define			smi_unlock_irqrestore(lock, flags)	do { __smi_unlock(lock); local_irq_restore(flags);	} while (0)

void			smi_init(void);

struct smi_fdesc	*smi_alloc(struct smi_fpart *fpart);
struct smi_fdesc	*smi_alloc_irqsafe(struct smi_fpart *fpart);
void			smi_free(struct smi_fdesc *fdesc);
void			smi_free_irqsafe(struct smi_fdesc *fdesc);

int			smi_enqueue(struct smi_fqueue *fqueue, struct smi_fdesc *fdesc);
int			smi_enqueue_irqsafe(struct smi_fqueue *fqueue, struct smi_fdesc *fdesc);
ulong			smi_dequeue(struct smi_fqueue *fqueue);
ulong			smi_dequeue_irqsafe(struct smi_fqueue *fqueue);

#endif
