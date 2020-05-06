/*
 * include/asm-arm/arch-comcerto/board-megamombasa515.h
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

#ifndef __ASM_ARCH_BOARD_MEGAMOMBASA_515_H
#define __ASM_ARCH_BOARD_MEGAMOMBASA_515_H

#define COMCERTO_AHBCLK_HZ		132500000	/* 132.5MHz */

#define COMCERTO_NOR16_BASE		0x80000000

#define COMCERTO_NAND_BASE		COMCERTO_CS3_BASE	/* Address where flash is mapped */

#define COMCERTO_GPIO_IRQ_SLIC		0
#define COMCERTO_GPIO_IN_NAND_BR	1
#define COMCERTO_GPIO_OUT_NAND_CE	3
#define COMCERTO_GPIO_OUT_NAND_CLE	4
#define COMCERTO_GPIO_OUT_NAND_ALE	5
#define COMCERTO_GPIO_IRQ_PCI		6

#endif /* __ASM_ARCH_BOARD_MEGAMOMBASA_515_H */
