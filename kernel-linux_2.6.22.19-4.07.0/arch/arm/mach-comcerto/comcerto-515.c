/*
 *  linux/arch/arm/mach-comcerto/comcerto-515.c
 *
 *  Copyright (C) 2004,2005,2008 Mindspeed Technologies, Inc.
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


static struct map_desc comcerto515_io_desc[] __initdata = {
#ifndef CONFIG_MSP_NONE
	{
		.virtual	= SDRAM_MSP_MEMORY_VADDR,
		.pfn		= __phys_to_pfn(SDRAM_MSP_MEMORY_PHY),
		.length		= SDRAM_MSP_MEMORY_SIZE,
		.type		= MT_DEVICE
	},
#endif
#if (ARAM_MEMORY_SIZE > 0)
	{
		.virtual	= ARAM_MEMORY_VADDR,
		.pfn		= __phys_to_pfn(ARAM_MEMORY_PHY),
		.length		= ARAM_MEMORY_SIZE,
		.type		= MT_DEVICE
	},
#endif
	{
		.virtual	= IRAM_MEMORY_VADDR,
		.pfn		= __phys_to_pfn(IRAM_MEMORY_PHY),
		.length		= IRAM_MEMORY_SIZE,
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
	},
};

/* NOTE: order is important here, interrupts with a higher priority should go
 *	 earlier in table. comcerto_irq_init() will use descriptor index to
 *	 program INTC priority.
 */
static struct comcerto_irq_desc comcerto515_irq_desc[] __initdata =
{
	{ IRQ_TIMERB,		comcerto_handle_secondary_level_irq,	},
	{ IRQ_SPI,		comcerto_handle_secondary_level_irq,	},
	{ IRQ_I2C,		comcerto_handle_secondary_level_irq,	},
	{ IRQ_EDMA0RX,		handle_simple_irq,			},
	{ IRQ_EDMA0TX,		handle_simple_irq,			},
	{ IRQ_PTP1,		handle_level_irq,			},
	{ IRQ_PTP0,		handle_level_irq,			},
	{ IRQ_STATUS_REG_1,	NULL,					},	/* fake handler to set STATUS1 priority */
	{ IRQ_EMAC0,		comcerto_handle_secondary_level_irq,	},
	{ IRQ_HI,		comcerto_handle_secondary_level_irq,	},

	/* interrupts on the STATUS1, no separate priorities for these, see IRQ_STATUS1 entry above */
	{ IRQ_UART1,		comcerto_handle_secondary_level_irq,	},
	{ IRQ_UART0,		comcerto_handle_secondary_level_irq,	},
};


void __init comcerto515_map_io(void)
{
	iotable_init(comcerto515_io_desc, ARRAY_SIZE(comcerto515_io_desc));
}

void __init comcerto515_fixup(struct machine_desc *desc, struct tag *unused, char **cmdline, struct meminfo *mi)
{
	/* enable async ARM clock mode */
	adjust_cr(0xc0000000, 0xc0000000);
}

void __init comcerto515_irq_init(void)
{
	printk(KERN_ERR "CSP: ARM%c\n", comcerto_arm1_is_running()?'1':'0');

	comcerto_irq_init(comcerto515_irq_desc, ARRAY_SIZE(comcerto515_irq_desc));
}
