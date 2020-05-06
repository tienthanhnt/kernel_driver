/*
 *  linux/arch/arm/mach-comcerto/irq.c
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

#include <linux/ptrace.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/kernel_stat.h>

#include <asm/irq.h>
#include <asm/io.h>
#include <asm/mach/irq.h>
#include <asm/arch/hardware.h>

#include <asm/arch/gpio.h>
#include <asm/arch/irq.h>
#include <asm/arch/irqs.h>


#ifdef CONFIG_CSP_ARM0

static void comcerto_irq_arm0_mask_0(unsigned int irq)
{
	__raw_writel(__raw_readl(COMCERTO_INTC_ARM0_IRQMASK0) & ~(1UL << irq), COMCERTO_INTC_ARM0_IRQMASK0);
}

static void comcerto_irq_arm0_mask_ack_0(unsigned int irq)
{
	__raw_writel(__raw_readl(COMCERTO_INTC_ARM0_IRQMASK0) & ~(1UL << irq), COMCERTO_INTC_ARM0_IRQMASK0);
	__raw_writel(1UL << irq, COMCERTO_INTC_STATUS0);
}


static void comcerto_irq_arm0_unmask_0(unsigned int irq)
{
	__raw_writel(__raw_readl(COMCERTO_INTC_ARM0_IRQMASK0) | (1UL << irq), COMCERTO_INTC_ARM0_IRQMASK0);
}

static void comcerto_irq_arm0_mask_1(unsigned int irq)
{
	irq -= 32;
	__raw_writel(__raw_readl(COMCERTO_INTC_ARM0_IRQMASK1) & ~(1UL << irq), COMCERTO_INTC_ARM0_IRQMASK1);
}

static void comcerto_irq_arm0_mask_ack_1(unsigned int irq)
{
	irq -= 32;
	__raw_writel(__raw_readl(COMCERTO_INTC_ARM0_IRQMASK1) & ~(1UL << irq), COMCERTO_INTC_ARM0_IRQMASK1);
	__raw_writel(1UL << irq, COMCERTO_INTC_STATUS1);
}

static void comcerto_irq_arm0_unmask_1(unsigned int irq)
{
	irq -= 32;
	__raw_writel(__raw_readl(COMCERTO_INTC_ARM0_IRQMASK1) | (1UL << irq), COMCERTO_INTC_ARM0_IRQMASK1);
}

#endif

static void comcerto_irq_arm1_mask_0(unsigned int irq)
{
	__raw_writel(__raw_readl(COMCERTO_INTC_ARM1_IRQMASK0) & ~(1UL << irq), COMCERTO_INTC_ARM1_IRQMASK0);
}

static void comcerto_irq_arm1_mask_ack_0(unsigned int irq)
{
	__raw_writel(__raw_readl(COMCERTO_INTC_ARM1_IRQMASK0) & ~(1UL << irq), COMCERTO_INTC_ARM1_IRQMASK0);
	__raw_writel(1UL << irq, COMCERTO_INTC_STATUS0);
}

static void comcerto_irq_arm1_unmask_0(unsigned int irq)
{
	__raw_writel(__raw_readl(COMCERTO_INTC_ARM1_IRQMASK0) | (1UL << irq), COMCERTO_INTC_ARM1_IRQMASK0);
}

static int comcerto_set_irq_type_0(unsigned int irq, unsigned int type)
{
	return 0;
}

static void comcerto_irq_arm1_mask_1(unsigned int irq)
{
	irq -= 32;
	__raw_writel(__raw_readl(COMCERTO_INTC_ARM1_IRQMASK1) & ~(1UL << irq), COMCERTO_INTC_ARM1_IRQMASK1);
}

static void comcerto_irq_ack_0(unsigned int irq)
{
	__raw_writel(1UL << irq, COMCERTO_INTC_STATUS0);
}

static void comcerto_irq_ack_1(unsigned int irq)
{
	irq -= 32;
	__raw_writel(1UL << irq, COMCERTO_INTC_STATUS1);
}

static void comcerto_irq_arm1_mask_ack_1(unsigned int irq)
{
	irq -= 32;
	__raw_writel(__raw_readl(COMCERTO_INTC_ARM1_IRQMASK1) & ~(1UL << irq), COMCERTO_INTC_ARM1_IRQMASK1);
	__raw_writel(1UL << irq, COMCERTO_INTC_STATUS1);
}

static void comcerto_irq_arm1_unmask_1(unsigned int irq)
{
	irq -= 32;
	__raw_writel(__raw_readl(COMCERTO_INTC_ARM1_IRQMASK1) | (1UL << irq), COMCERTO_INTC_ARM1_IRQMASK1);
}

static int comcerto_set_irq_type_1(unsigned int irq, unsigned int type)
{
	return 0;
}

static struct irq_chip comcerto_irq_chip_0 = { 
	.name		= "Comcerto INTC0",
	.ack		= comcerto_irq_ack_0,
	.set_type	= comcerto_set_irq_type_0,
	/* .disable, .mask, .unmask, .mask_ack - initialized in comcerto_irq_init */
};

static struct irq_chip comcerto_irq_chip_1 = {
	.name		= "Comcerto INTC1",
	.ack		= comcerto_irq_ack_1,
	.set_type	= comcerto_set_irq_type_1,
	/* .disable, .mask, .unmask, .mask_ack - initialized in comcerto_irq_init */
};

static u32 irq_self_clear_mask = 0;

extern int noirqdebug;

void fastcall comcerto_handle_secondary_level_irq(unsigned int irq, struct irq_desc *desc)
{
	unsigned int cpu = smp_processor_id();
	struct irqaction *action;
	irqreturn_t action_ret;

	spin_lock(&desc->lock);
	desc->chip->mask(irq);

	if (unlikely(desc->status & IRQ_INPROGRESS))
		goto out_unlock;
	desc->status &= ~(IRQ_REPLAY | IRQ_WAITING);
	kstat_cpu(cpu).irqs[irq]++;

	/*
	 * If its disabled or no action available
	 * keep it masked and get out of here
	 */
	action = desc->action;
	if (unlikely(!action || (desc->status & IRQ_DISABLED))) {
		desc->status |= IRQ_PENDING;
		goto out_unlock;
	}

	desc->status |= IRQ_INPROGRESS;
	desc->status &= ~IRQ_PENDING;
	spin_unlock(&desc->lock);

	action_ret = handle_IRQ_event(irq, action);
	if (!noirqdebug)
		note_interrupt(irq, desc, action_ret);

	spin_lock(&desc->lock);
	desc->status &= ~IRQ_INPROGRESS;

	desc->chip->ack(irq);

	if (!(desc->status & IRQ_DISABLED) && desc->chip->unmask)
		desc->chip->unmask(irq);
out_unlock:
	spin_unlock(&desc->lock);
}

#ifndef CONFIG_ARCH_M825XX1

static u8 comcerto_status0_priorities[32] __initdata = {0, };

static __init void comcerto_irq_priority_set(unsigned int irq, unsigned int prio)
{
	unsigned int prio_reg, prio_shift, prio_mask;

	comcerto_status0_priorities[irq] = prio + 1;	/* we store +1 priority to reserve zero for non-used slots */


	prio_reg = COMCERTO_INTC_ARM0_PRIORITY;
	if (comcerto_arm1_is_running())
		prio_reg += 64;		/* ARM1_PRIORITY if we run on ARM1 */

	prio_reg += 4 * (prio / 4);	/* actual priority register in block */
	prio_shift = ((prio % 4) << 3);
	prio_mask = 0x1f << prio_shift;
	__raw_writel((__raw_readl(prio_reg) & ~prio_mask) | (irq << prio_shift), prio_reg);
}

void __init comcerto_irq_status1_priority_set(unsigned char priority)
{
	int i;
	unsigned long flags;

	if ((priority & ~31) != 0)
		BUG();

	priority++;

	local_irq_save(flags);

	/* look at all interrupts except STATUS1 and fix all priorities >= specified */
	for (i = 1; i < 32; i++)
		if (comcerto_status0_priorities[i] >= priority && comcerto_status0_priorities[i] > 0)
			comcerto_status0_priorities[i]++;

	comcerto_status0_priorities[0] = priority;

	/* program all non-zero priorities, don't forget to decrement priority value taken from array */
	for (i = 0; i < 32; i++)
		if (comcerto_status0_priorities[i] > 0)
			comcerto_irq_priority_set(i, comcerto_status0_priorities[i] - 1);

	local_irq_restore(flags);
}

#else

#define comcerto_irq_priority_set(irq, priority) do {} while (0)

#endif /* !CONFIG_ARCH_M825XX1 */


void __init comcerto_irq_init(struct comcerto_irq_desc *irq_table, int size)
{
	unsigned int irq, prio = 0;
	int i;

	if (comcerto_arm1_is_running()) {
		/* ARM1 */

		/* mask all interrupts */
		__raw_writel(0, COMCERTO_INTC_ARM1_IRQMASK0);
		__raw_writel(0, COMCERTO_INTC_ARM1_IRQMASK1);

		comcerto_irq_chip_0.disable	= comcerto_irq_arm1_mask_0;
		comcerto_irq_chip_0.mask	= comcerto_irq_arm1_mask_0;
		comcerto_irq_chip_0.unmask	= comcerto_irq_arm1_unmask_0;
		comcerto_irq_chip_0.mask_ack	= comcerto_irq_arm1_mask_ack_0;

		comcerto_irq_chip_1.disable	= comcerto_irq_arm1_mask_1;
		comcerto_irq_chip_1.mask	= comcerto_irq_arm1_mask_1;
		comcerto_irq_chip_1.unmask	= comcerto_irq_arm1_unmask_1;
		comcerto_irq_chip_1.mask_ack	= comcerto_irq_arm1_mask_ack_1;
	} else {
#ifdef CONFIG_CSP_ARM0
		/* ARM0 */

		/* mask all interrupts */
		__raw_writel(0, COMCERTO_INTC_ARM0_IRQMASK0);
		__raw_writel(0, COMCERTO_INTC_ARM0_IRQMASK1);

		comcerto_irq_chip_0.disable	= comcerto_irq_arm0_mask_0;
		comcerto_irq_chip_0.mask	= comcerto_irq_arm0_mask_0;
		comcerto_irq_chip_0.unmask	= comcerto_irq_arm0_unmask_0;
		comcerto_irq_chip_0.mask_ack	= comcerto_irq_arm0_mask_ack_0;

		comcerto_irq_chip_1.disable	= comcerto_irq_arm0_mask_1;
		comcerto_irq_chip_1.mask	= comcerto_irq_arm0_mask_1;
		comcerto_irq_chip_1.unmask	= comcerto_irq_arm0_unmask_1;
		comcerto_irq_chip_1.mask_ack	= comcerto_irq_arm0_mask_ack_1;
#else
		BUG();
#endif
	}

	for (i = 0; i < size; i++) {
		irq = irq_table[i].num;

		/* setup interrupt handler and priority */
		if (irq < 32) {
			/* All STATUS0 interrupts must be setup from the single table declared in comcerto-xxx.c
			 * file. Use description index as a priority.
			 */
			comcerto_irq_priority_set(irq, prio++);
			set_irq_chip(irq, &comcerto_irq_chip_0);
			irq_self_clear_mask |= comcerto_irq_mask(irq);
		} else
			set_irq_chip(irq, &comcerto_irq_chip_1);

		if (irq_table[i].handler != NULL) {
			set_irq_handler(irq, irq_table[i].handler);
			set_irq_flags(irq, IRQF_VALID);
		}
	}

	comcerto_irq_fixup();
}

void comcerto_irq_fixup(void)
{
#ifndef CONFIG_ARCH_M825XX1
	/* make sure there're not self clear bits for CSP STATUS0 interrupts */
	__raw_writel(__raw_readl(COMCERTO_INTC_SELF_CLEAR) & ~irq_self_clear_mask, COMCERTO_INTC_SELF_CLEAR);
#endif
}
