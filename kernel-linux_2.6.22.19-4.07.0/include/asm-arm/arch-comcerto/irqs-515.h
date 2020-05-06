/*
 *  linux/include/asm-arm/arch-comcerto/irqs-515.h
 *
 *  Copyright (C) 2004,2005 Mindspeed Technologies, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef __ASM_ARCH_IRQS_515_H
#define __ASM_ARCH_IRQS_515_H

/* IRQ STATUS_0, IRQ MASK_0 and FIQ MASK_0 Register Bit Definition */

#define NR_IRQS                 64

#define IRQ_TIMERA		31
#define IRQ_TIMERB		30
#define IRQ_XDONE_1		29
#define IRQ_XDONE_0		28
#define IRQ_MDMA_DONE		27
#define IRQ_RESERVED26		26
#define IRQ_RESERVED25		25
#define IRQ_RESERVED24		24
#define IRQ_RESERVED23		23
#define IRQ_RESERVED22		22
#define IRQ_SPU0		21
#define IRQ_EDMA0TX		20
#define IRQ_EDMA0RX		19
#define IRQ_HITXFUL		18
#define IRQ_HIRXEMT		17
#define IRQ_HI			16
#define IRQ_TDM_TIMER		15
#define IRQ_SPI			14
#define IRQ_RESERVED13		13
#define IRQ_RESERVED12		12
#define IRQ_RESERVED11		11
#define IRQ_I2C			10
#define IRQ_RESERVED9		9
#define IRQ_TDMA		8
#define IRQ_PUDMATX		7
#define IRQ_PUDMARX		6
#define IRQ_PUI			5
#define IRQ_EMAC0		4
#define IRQ_PTP2		3
#define IRQ_PTP1		2
#define IRQ_PTP0		1
#define IRQ_STATUS_REG_1	0

/* IRQ STATUS_1, IRQ MASK_1 and FIQ MASK_1 Register Bit Definition  */

#define IRQ_SMC_ERR		(32 + 31)
#define IRQ_ERRIDMATX		(32 + 30)
#define IRQ_ERRIDMARX		(32 + 29)
#define IRQ_ERRTDTX		(32 + 28)
#define IRQ_ERRTDRX		(32 + 27)
#define IRQ_UART1		(32 + 26)
#define IRQ_TDTXEMT_3		(32 + 25)
#define IRQ_TDRXFUL_3		(32 + 24)
#define IRQ_TDTXUDR_3		(32 + 23)
#define IRQ_TDRXOVR_3		(32 + 22)
#define IRQ_TDTXEMT_2		(32 + 21)
#define IRQ_TDRXFUL_2		(32 + 20)
#define IRQ_TDTXUDR_2		(32 + 19)
#define IRQ_TDRXOVR_2		(32 + 18)
#define IRQ_TDTXEMT_1		(32 + 17)
#define IRQ_TDRXFUL_1		(32 + 16)
#define IRQ_TDTXUDR_1		(32 + 15)
#define IRQ_TDRXOVR_1		(32 + 14)
#define IRQ_TDTXEMT_0		(32 + 13)
#define IRQ_TDRXFUL_0		(32 + 12)
#define IRQ_TDTXUDR_0		(32 + 11)
#define IRQ_TDRXOVR_0		(32 + 10)
#define IRQ_UART0		(32 +  9)
#define IRQ_G7			(32 +  8)
#define IRQ_G6			(32 +  7)
#define IRQ_G5			(32 +  6)
#define IRQ_G4			(32 +  5)
#define IRQ_G3			(32 +  4)
#define IRQ_G2			(32 +  3)
#define IRQ_G1			(32 +  2)
#define IRQ_G0			(32 +  1)
#define IRQ_STATUS_REG_0	(32 +  0)

#endif /* __ASM_ARCH_IRQS_515_H */
