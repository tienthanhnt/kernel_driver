/*
 *  linux/include/asm-arm/arch-comcerto/uncompress.h
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
#ifndef __ASM_ARCH_UNCOMPRESS_H
#define __ASM_ARCH_UNCOMPRESS_H

#if defined(CONFIG_ARCH_M825XX1)
	#define UART_DR		(*(volatile unsigned long *)0x10090000)
	#define UART_LSR	(*(volatile unsigned long *)0x10090010)

#else
	#ifdef CONFIG_COMCERTO_UART0_SUPPORT
		#define UART_DR		(*(volatile unsigned long *)0x10090000)
		#define UART_LSR	(*(volatile unsigned long *)0x10090014)
	#elif CONFIG_COMCERTO_UART1_SUPPORT
		#define UART_DR		(*(volatile unsigned long *)0x10094000)
		#define UART_LSR	(*(volatile unsigned long *)0x10094014)
	#else
		#error no uart configured
	#endif
#endif

static inline void putc(int c)
{
	while (!(UART_LSR & 0x20))
		barrier();

	UART_DR = c;
}

static void flush(void)
{
}

/*
 * nothing to do
 */
#define arch_decomp_setup()

#define arch_decomp_wdog()

#endif /* __ASM_ARCH_UNCOMPRESS_H */
