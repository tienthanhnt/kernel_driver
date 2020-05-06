/*
 * include/asm-arm/arch-comcerto/hardware.h
 *
 * Copyright (C) 2004-2010 Mindspeed Technologies, Inc.
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

#ifndef __ASM_ARCH_HARDWARE_H
#define __ASM_ARCH_HARDWARE_H

#include <linux/autoconf.h>

#if defined(CONFIG_ARCH_M823XX)
	#include "comcerto-300.h"
#elif defined(CONFIG_ARCH_M825XX1)
	#include "comcerto-530.h"
#elif defined(CONFIG_ARCH_M825XX2)
	#include "comcerto-515.h"
#elif defined(CONFIG_ARCH_M828XX)
	#include "comcerto-800.h"
#else
	#error "Unsupported CPU"
#endif

#include "board.h"

/* macro to get at IO space when running virtually */
#define APB_VADDR(x)			((x) - APB_MEMORY_PHY + APB_MEMORY_VADDR)

#define pcibios_assign_all_busses()	1

#define PCIBIOS_MIN_IO			0x20000000
#define PCIBIOS_MIN_MEM			0x30000000

#ifndef __ASSEMBLY__

#include <asm/io.h>

static inline int comcerto_arm1_is_running(void)
{
	return __raw_readl(COMCERTO_INTC_ARM_CONTROL) >> 31;
}

static inline u32 comcerto_pll_get_bus_clock_hz(void)
{
#ifndef CONFIG_ARCH_M823V2
	return COMCERTO_AHBCLK_HZ;
#else
	u32 clkr, clkd, clkf;

	clkf = clkr = clkd = __raw_readl(COMCERTO_PLL_AMBA);
	clkf &= 0x0FFF;
	clkd = (clkr >> 12) & 0x0F;
	clkr = (clkr >> 16) & 0x3F;

	return ((25*(clkf + 1)) / (clkr + 1) / (clkd + 1) / 4)*1000*1000;
#endif
}

#ifndef CONFIG_ARCH_M823V2
#define comcerto_pll_bus_clock_hz	COMCERTO_AHBCLK_HZ
#define comcerto_pll_bus_clock_mhz	(COMCERTO_AHBCLK_HZ/1000/1000)
#else
extern u32 comcerto_pll_bus_clock_hz;
extern u32 comcerto_pll_bus_clock_mhz;
#endif

#endif

#endif /* __ASM_ARCH_HARDWARE_H */

