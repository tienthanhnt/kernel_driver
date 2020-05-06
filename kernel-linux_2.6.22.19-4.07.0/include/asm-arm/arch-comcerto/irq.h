/*
 *  linux/include/asm-arm/arch-comcerto/irq.h
 *
 *  Copyright (C) 2004,2008 Mindspeed Technologies, Inc.
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
#ifndef __ASM_ARCH_IRQ_H
#define __ASM_ARCH_IRQ_H

#include <linux/irq.h>


struct comcerto_irq_desc {
	char num;
	irq_flow_handler_t handler;
};

void fastcall comcerto_handle_secondary_level_irq(unsigned int irq, struct irq_desc *desc);
void __init comcerto_irq_init(struct comcerto_irq_desc *irq_table, int size);
void comcerto_irq_fixup(void);

#ifndef CONFIG_ARCH_M825XX1
/* For all devices with priority support we provide a call to change STATUS1
 * priority from the EVM-specific code. Priority value should be in [0..31] range.
 */
void __init comcerto_irq_status1_priority_set(unsigned char priority);
#endif

static inline unsigned int comcerto_irq_mask(int irq)
{
	return 1 << ((irq) & 0x1f);
}

static inline void comcerto_softirq_set(int irq)
{
	u32 reg;

	if (irq < 32)
		reg = COMCERTO_INTC_SET_STATUS0;
	else
		reg = COMCERTO_INTC_SET_STATUS1;

	__raw_writel(comcerto_irq_mask(irq), reg);
}

static inline int comcerto_softirq_check(int irq)
{
	u32 reg;

	if (irq < 32)
		reg = COMCERTO_INTC_STATUS0;
	else
		reg = COMCERTO_INTC_STATUS1;

	return __raw_readl(reg) & comcerto_irq_mask(irq) ? 1 : 0;
}

static inline void comcerto_irq_ack(int irq)
{
	u32 reg;

	if (irq < 32)
		reg = COMCERTO_INTC_STATUS0;
	else
		reg = COMCERTO_INTC_STATUS1;

	__raw_writel(comcerto_irq_mask(irq), reg);
}

#endif /* __ASM_ARCH_IRQ_H */
