/*
 * include/asm-arm/arch-comcerto/board.h
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

#ifndef __ASM_ARCH_BOARD_H
#define __ASM_ARCH_BOARD_H

#include <linux/autoconf.h>

#if defined(CONFIG_EVM_MEGAMOMBASA515)
	#include "board-megamombasa515.h"
#elif defined(CONFIG_EVM_C300V1)
	#include "board-c300evm.h"
#elif defined(CONFIG_EVM_C300V2)
	#include "board-c300v2evm.h"
#else
	#error "Unsupported EVM"
#endif

#endif /* __ASM_ARCH_BOARD_H */
