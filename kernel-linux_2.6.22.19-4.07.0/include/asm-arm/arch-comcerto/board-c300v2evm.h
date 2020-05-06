/*
 * include/asm-arm/arch-comcerto/board-c300v2.h
 *
 * Copyright (C) 2010 Mindspeed Technologies, Inc.
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

#ifndef __ASM_ARCH_BOARD_C300V2_H
#define __ASM_ARCH_BOARD_C300V2_H

#define COMCERTO_NOR16_BASE			COMCERTO_IBR_CS2_REMAP_BASE

#define COMCERTO_NAND_BASE			COMCERTO_CS5_BASE

#define COMCERTO_GPIO_IRQ_SLIC0			 0
#define COMCERTO_GPIO_IN_NAND_BR		 1
#define COMCERTO_GPIO_OUT_PERIPH_RESET		 2
#define COMCERTO_GPIO_IRQ_MUX			 3

#define COMCERTO_GPIO_IRQ_SLIC_CON0_INT0	16
#define COMCERTO_GPIO_IRQ_SLIC_CON0_INT1	17
#define COMCERTO_GPIO_IRQ_SLIC_CON0_INT2	18
#define COMCERTO_GPIO_IRQ_SLIC_CON0_INT3	19
#define COMCERTO_GPIO_IRQ_SLIC_CON0_INT4	20
#define COMCERTO_GPIO_IRQ_SLIC_CON0_INT5	21
#define COMCERTO_GPIO_IRQ_SLIC_CON0_INT6	22
#define COMCERTO_GPIO_IRQ_SLIC_CON0_INT7	23

#define COMCERTO_GPIO_IRQ_PCI_INTA		24
#define COMCERTO_GPIO_IRQ_SLIC_CON1_INT7	29

#endif /* __ASM_ARCH_BOARD_C300V2_H */
