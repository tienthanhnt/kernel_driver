/*
 *  linux/include/asm-arm/arch-comcerto/irqs-300.h
 *
 *  Copyright (C) 2004,2005 Mindspeed Technologies, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef __ASM_ARCH_IRQS_300_H
#define __ASM_ARCH_IRQS_300_H

/* IRQ STATUS_0, IRQ MASK_0 and FIQ MASK_0 Register Bit Definition */

#define NR_IRQS                 64
#define FIQ_START		NR_IRQS

#define IRQ_TIMERA		31
#define IRQ_TIMERB		30
#define IRQ_XDONE_1		29
#define IRQ_XDONE_0		28
#define IRQ_MDMA_DONE		27
#define IRQ_SPI1		26
#define IRQ_SPI0		25
#define IRQ_I2C			24
#define IRQ_TIMERC		23
#define IRQ_TIMERD		22
#define IRQ_IDMA		21
#ifdef CONFIG_ARCH_M823V1
#define IRQ_SPU1		20
#endif
#define IRQ_SPU0		19
#define IRQ_HITXFUL		18
#define IRQ_HIRXEMT		17
#define IRQ_HI			16
#define IRQ_TDM_TIMER		15
#define IRQ_EDMA1TX		14
#define IRQ_EDMA1RX		13
#define IRQ_GEMAC1		12
#define IRQ_EDMA0TX		11
#define IRQ_EDMA0RX		10
#define IRQ_GEMAC0		9
#define IRQ_TDMA		8
#ifdef CONFIG_ARCH_M823V1
#define IRQ_PUDMATX		7
#define IRQ_PUDMARX		6
#define IRQ_PUI			5
#endif
#define IRQ_PTP2		3
#define IRQ_PTP1		2
#define IRQ_PTP0		1
#define IRQ_STATUS_REG_1	0

/* IRQ STATUS_1, IRQ MASK_1 and FIQ MASK_1 Register Bit Definition  */

#define IRQ_SMC_ERR		(32 + 31)
#define IRQ_ERRMDMA		(32 + 30)
#define IRQ_ERRIDMA		(32 + 29)
#define IRQ_ERRTDTX		(32 + 28)
#define IRQ_ERRTDRX		(32 + 27)
#define IRQ_TDMTSB		(32 + 26)
#define IRQ_UART1		(32 + 25)
#define IRQ_NTG			(32 + 24)
#define IRQ_UEXP		(32 + 23)
#define IRQ_G32			(32 + 22)
#define IRQ_ERRTDMA		(32 + 10)
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
	

#endif /* __ASM_ARCH_IRQS_300_H */
