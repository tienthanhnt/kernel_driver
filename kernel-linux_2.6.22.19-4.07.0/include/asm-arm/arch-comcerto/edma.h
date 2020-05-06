/*
 *  linux/include/asm-arm/arch-comcerto/edma.h
 *
 *  Copyright (C) 2009 Mindspeed Technologies, Inc.
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef __ASM_ARCH_EDMA_H
#define __ASM_ARCH_EDMA_H

#include <linux/types.h>

/* EDMA buffer descriptor, alignment - 8 bytes */
struct edma_bdesc {
	u32	bptr;		/* physical address */
	u32	bcontrol;	/* control flags, see EDMA_BCONTROL_XXX */
};

/* EDMA frame descriptor, alignment - 16 bytes */
struct edma_fdesc {
	u32	next;		/* physical address */
	u32	system;		/* not used by hardware */
	u32	fstatus;	/* frame status, bit-fields are set by EMAC, see EMAC_FSTATUS_XXX */
	u32	fcontrol;	/* control flags, see EDMA_FCONTROL_XXX */

	struct edma_bdesc	bdesc;		/* there may be several buffer descriptors, we use only one per frame */

	u32	alignment[2];	/* align structure size to 16 bytes */
};

#define EDMA_FSTATUS_FDONE		0x80000000

#define EDMA_FCONTROL_IRQEN		0x00000004
#define EDMA_FCONTROL_FLAST		0x00000002
#define EDMA_FCONTROL_FREADY		0x00000001

#define EDMA_BCONTROL_BLAST		0x00010000
#define EDMA_BCONTROL_BLEN_MASK		0x0000FFFF

#define EDMA_TX_START(base)		(*(volatile u32*)(((u32)(base)) + 0x00))
#define EDMA_TX_HEAD(base)		(*(volatile u32*)(((u32)(base)) + 0x04))
#define EDMA_TX_BURST(base)		(*(volatile u32*)(((u32)(base)) + 0x08))
#define EDMA_TX_CONTROL(base)		(*(volatile u32*)(((u32)(base)) + 0x10))
#define EDMA_TX_IRQ(base)		(*(volatile u32*)(((u32)(base)) + 0x14))
#define EDMA_TX_RESET(base)		(*(volatile u32*)(((u32)(base)) + 0x20))

#define EDMA_RX_START(base)		(*(volatile u32*)(((u32)(base)) + 0x80))
#define EDMA_RX_HEAD(base)		(*(volatile u32*)(((u32)(base)) + 0x84))
#define EDMA_RX_BURST(base)		(*(volatile u32*)(((u32)(base)) + 0x88))
#define EDMA_RX_CONTROL(base)		(*(volatile u32*)(((u32)(base)) + 0x90))
#define EDMA_RX_IRQ(base)		(*(volatile u32*)(((u32)(base)) + 0x94))
#define EDMA_RX_RESET(base)		(*(volatile u32*)(((u32)(base)) + 0xA0))

#define EDMA_START			0x00000001

#define EDMA_BURST_MASK			0x000000FF

#define EDMA_IRQ_DONE			(1 <<  2)
#define EDMA_CONTROL_STOPDONE		(1 <<  8)

#define	EDMA_FDESC_ALIGN_BYTES		0x10

#endif	/* __ASM_ARCH_EDMA_H */
