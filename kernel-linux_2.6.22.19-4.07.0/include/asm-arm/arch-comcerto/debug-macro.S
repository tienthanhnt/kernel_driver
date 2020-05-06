/*
 *  linux/include/asm-arm/arch-comcerto/debug-macro.S
 *
 *  Copyright (C) 2004,2005 Mindspeed Technologies, Inc.
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

#if defined(CONFIG_ARCH_M825XX1)
#define UART_LSR	0x10
#else
#define UART_LSR	0x14
#endif

	.macro	addruart,rx
	mrc	p15, 0, \rx, c1, c0
	tst	\rx, #1			@ MMU enabled?
	moveq	\rx, #0x10000000	@ physical base address
	movne	\rx, #0xd0000000	@ virtual base
#if defined(CONFIG_ARCH_M825XX1)
	orr	\rx, \rx, #0x00090000	@ UART0
#else
#ifdef CONFIG_COMCERTO_UART0_SUPPORT
	orr	\rx, \rx, #0x00090000	@ UART0
#elif CONFIG_COMCERTO_UART1_SUPPORT
	orr	\rx, \rx, #0x00094000	@ UART1
#else
#error no uart defined
#endif
#endif
	.endm

	.macro	senduart,rd,rx
	strb	\rd, [\rx, #0x00]
	.endm

	.macro	waituart,rd,rx
1001:	ldrb	\rd, [\rx, #UART_LSR]
	tst	\rd, #0x20		@ wait for THRE
	beq	1001b
	.endm

	.macro	busyuart,rd,rx
1001:	ldrb	\rd, [\rx, #UART_LSR]
	and	\rd, \rd, #0x60
	teq	\rd, #0x60		@ wait for TEMT and THRE
	bne	1001b
	.endm
