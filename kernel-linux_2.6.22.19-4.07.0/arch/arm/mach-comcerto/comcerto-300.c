/*
 * linux/arch/arm/mach-comcerto/comcerto-300.c
 *
 * Copyright (C) 2008-2010 Mindspeed Technologies, Inc.
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
#include <linux/autoconf.h>
#include <linux/sched.h>

#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/io.h>
#include <asm/sizes.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/arch/hardware.h>
#include <asm/arch/irq.h>
#include <asm/arch/irqs.h>
#include <asm/arch/msp.h>


static struct map_desc comcerto300_io_desc[] __initdata = {
#ifndef CONFIG_MSP_NONE
	{
		.virtual	= SDRAM_MSP_MEMORY_VADDR,
		.pfn		= __phys_to_pfn(SDRAM_MSP_MEMORY_PHY),
		.length		= SDRAM_MSP_MEMORY_SIZE,
		.type		= MT_DEVICE
	},
#endif
	{
		.virtual	= ARAM_MEMORY_VADDR,
		.pfn		= __phys_to_pfn(ARAM_MEMORY_PHY),
		.length		= ARAM_MEMORY_SIZE,
		.type		= MT_DEVICE
	}, {
		.virtual	= ERAM_MEMORY_VADDR,
		.pfn		= __phys_to_pfn(ERAM_MEMORY_PHY),
		.length		= ERAM_MEMORY_SIZE,
		.type		= MT_DEVICE
	}, {
		.virtual	= APB_MEMORY_VADDR,
		.pfn		= __phys_to_pfn(APB_MEMORY_PHY),
		.length		= APB_MEMORY_SIZE,
		.type		= MT_DEVICE
	}, {
		.virtual	= COMCERTO_UEXP_DA_VADDR_BASE,
		.pfn		= __phys_to_pfn(COMCERTO_UEXP_DA_BASE),
		.length		= SZ_4K,
		.type		= MT_DEVICE
	},
};

/* NOTE: order is important here, interrupts with a higher priority should go
 *	 earlier in table. comcerto_irq_init() will use descriptor index to
 *	 program INTC priority.
 */
static struct comcerto_irq_desc comcerto300_irq_desc[] __initdata =
{
	{ IRQ_TIMERB,		comcerto_handle_secondary_level_irq,	},
	{ IRQ_SPI1,		comcerto_handle_secondary_level_irq,	},
	{ IRQ_SPI0,		comcerto_handle_secondary_level_irq,	},
	{ IRQ_I2C,		comcerto_handle_secondary_level_irq,	},
	{ IRQ_EDMA0RX,		handle_simple_irq,			},
	{ IRQ_EDMA1RX,		handle_simple_irq,			},
	{ IRQ_EDMA0TX,		handle_simple_irq,			},
	{ IRQ_EDMA1TX,		handle_simple_irq,			},
	{ IRQ_PTP1,		handle_level_irq,			},
	{ IRQ_PTP0,		handle_level_irq,			},
	{ IRQ_STATUS_REG_1,	NULL,					},	/* fake handler to set STATUS1 priority */
	{ IRQ_GEMAC0,		comcerto_handle_secondary_level_irq,	},
	{ IRQ_GEMAC1,		comcerto_handle_secondary_level_irq,	},
	{ IRQ_HI,		comcerto_handle_secondary_level_irq,	},

	/* interrupts on the STATUS1, no separate priorities for these, see IRQ_STATUS1 entry above */
	{ IRQ_UART1,		comcerto_handle_secondary_level_irq,	},
	{ IRQ_G32,		comcerto_handle_secondary_level_irq,	},
	{ IRQ_UART0,		comcerto_handle_secondary_level_irq,	},
};

void __init comcerto300_map_io(void)
{
	iotable_init(comcerto300_io_desc, ARRAY_SIZE(comcerto300_io_desc));
}

#ifdef CONFIG_ARCH_M823V2
u32 comcerto_pll_bus_clock_hz;
u32 comcerto_pll_bus_clock_mhz;
#endif

void __init comcerto300_fixup(struct machine_desc *desc, struct tag *unused, char **cmdline, struct meminfo *mi)
{
#ifdef CONFIG_ARCH_M823V2
	u32 aux;
	u32 clkr, clkd, clkf;

	adjust_cr(CR_FI, CR_FI);
	asm("mrc p15, 0, %0, c1, c0, 1  @ get aux control" : "=r" (aux) : : "cc");
	aux |= 0x80000000;
	asm volatile("mcr p15, 0, %0, c1, c0, 1 @ set aux control" : : "r" (aux) : "cc");
	isb();

	clkf = clkr = clkd = __raw_readl(COMCERTO_PLL_AMBA);
	clkf &= 0x0FFF;
	clkd = (clkr >> 12) & 0x0F;
	clkr = (clkr >> 16) & 0x3F;

	comcerto_pll_bus_clock_mhz = (25*(clkf + 1)) / (clkr + 1) / (clkd + 1) / 4;
	comcerto_pll_bus_clock_hz = comcerto_pll_bus_clock_mhz*1000*1000;
#endif

#ifdef CONFIG_MSP_CARRIER
	ved_wait_msp();
#endif
}

void __init comcerto300_irq_init(void)
{
	printk(KERN_ERR "CSP: ARM%c, bus %uMHz\n", comcerto_arm1_is_running()?'1':'0', comcerto_pll_bus_clock_mhz);

	comcerto_irq_init(comcerto300_irq_desc, ARRAY_SIZE(comcerto300_irq_desc));
}

