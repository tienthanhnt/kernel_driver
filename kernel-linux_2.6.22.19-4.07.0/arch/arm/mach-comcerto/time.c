/*
 * linux/arch/arm/mach-comcerto/time.c
 *
 * Copyright (C) 2008 Mindspeed Technologies, Inc.
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

#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/clocksource.h>
#include <linux/clockchips.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/arch/hardware.h>
#include <asm/mach/time.h>


struct comcerto_clock_device 
{
	int timer;
	struct clock_event_device device;
};

static inline void timer_irq_enable(int t)
{
	__raw_writel(__raw_readl(COMCERTO_TIMER_IRQ_MASK) | (1 << t), COMCERTO_TIMER_IRQ_MASK);
}

static inline void timer_irq_disable(int t)
{
	__raw_writel(__raw_readl(COMCERTO_TIMER_IRQ_MASK) & ~(1 << t), COMCERTO_TIMER_IRQ_MASK);
}

static inline void timer_irq_ack(int t)
{
	__raw_writel(1 << t, COMCERTO_TIMER_STATUS_CLR);
}

/* This function must be called with interrupts disabled. We need to start as close from zero
 * as possible - if we don't clear counter before setting high bound value we'll have innacurate
 * results in ONESHOT mode.
 */
static inline void __timer_set(int t, unsigned int bound)
{
	u32 bound_reg, count_reg;

	if (t == 1) {
		bound_reg = COMCERTO_TIMER1_HIGH_BOUND;
		count_reg = COMCERTO_TIMER1_CURRENT_COUNT;
	}
	else if (t == 2) {
		__raw_writel(0, COMCERTO_TIMER2_LOW_BOUND);
		bound_reg = COMCERTO_TIMER2_HIGH_BOUND;
		count_reg = COMCERTO_TIMER2_CURRENT_COUNT;
	}
	else {
		__raw_writel(0, COMCERTO_TIMER3_LOW_BOUND);
		bound_reg = COMCERTO_TIMER3_HIGH_BOUND;
		count_reg = COMCERTO_TIMER3_CURRENT_COUNT;
	}

	/* set zero to bound and wait until current will go zero */
	__raw_writel(0, bound_reg);
	while (__raw_readl(count_reg) != 0);

	/* now write correct bound and clear interrupt status if reasonable long wait was scheduled */
	__raw_writel(bound, bound_reg);
	if (bound > comcerto_pll_bus_clock_mhz)
		timer_irq_ack(t);
}

static inline u32 timer_get(int t)
{
	u32 current_reg;

	if (t == 1)
		current_reg = COMCERTO_TIMER1_CURRENT_COUNT;
	else if (t == 2)
		current_reg = COMCERTO_TIMER2_CURRENT_COUNT;
	else
		current_reg = COMCERTO_TIMER3_CURRENT_COUNT;

	return __raw_readl(current_reg);
}

static irqreturn_t comcerto_clock_interrupt(int irq, void *dev_id)
{
	u32 status;
	struct comcerto_clock_device *clock = dev_id;
	struct clock_event_device *dev = &clock->device;

	status = __raw_readl(COMCERTO_TIMER_STATUS) & (1 << clock->timer);
	if (status) {
		timer_irq_ack(clock->timer);

		/* we need to disable interrupt to simulate ONESHOT mode */
		if (dev->mode != CLOCK_EVT_MODE_PERIODIC)
			timer_irq_disable(clock->timer);

		dev->event_handler(dev);
	}

	return IRQ_HANDLED;
}

static cycle_t comcerto_timer3_read(void)
{
	return timer_get(3);
}

static int comcerto_clock_set_next_event(unsigned long evt, struct clock_event_device *dev)
{
	unsigned long flags;
	struct comcerto_clock_device *clock = container_of(dev, typeof(*clock), device);

	local_irq_save(flags);

	__timer_set(clock->timer, evt);

	/* enable interrupt for ONESHOT mode */
	if (dev->mode == CLOCK_EVT_MODE_ONESHOT)
		timer_irq_enable(clock->timer);

	local_irq_restore(flags);

	return 0;
}

static void comcerto_clock_set_mode(enum clock_event_mode mode, struct clock_event_device *dev)
{
	int old_inactive_mode, new_inactive_mode;
	struct comcerto_clock_device *clock = container_of(dev, typeof(*clock), device);

	/* This timer is true PERIODIC in hardware, to emulate ONESHOT we need to keep it masked
	 * until set_next_event() call. Enable interrupt only for PERIODIC mode.
	 */
	old_inactive_mode = (dev->mode != CLOCK_EVT_MODE_PERIODIC);
	new_inactive_mode = (mode != CLOCK_EVT_MODE_PERIODIC);
	if (old_inactive_mode != new_inactive_mode) {
		if (new_inactive_mode)
			timer_irq_disable(clock->timer);
		else
			timer_irq_enable(clock->timer);
	}
}

static struct clocksource clocksource =
{
	.name	= "timer3",
	.rating	= 200,
	.read	= comcerto_timer3_read,
	.mask	= CLOCKSOURCE_MASK(32),
	.shift	= 24,
	.flags	= CLOCK_SOURCE_IS_CONTINUOUS,
};

static struct comcerto_clock_device clock =
{
	.timer	= 1,
	.device	=
	{
		.name		= "timer1",
		.features	= CLOCK_EVT_FEAT_PERIODIC | CLOCK_EVT_FEAT_ONESHOT,
		.rating		= 200,
		.shift		= 24,
		.set_mode	= comcerto_clock_set_mode,
		.set_next_event	= comcerto_clock_set_next_event,
	},
};

static struct irqaction clock_irq =
{
	.name           = "timer",
	.flags          = IRQF_DISABLED | IRQF_TIMER | IRQF_IRQPOLL,
	.handler        = comcerto_clock_interrupt,
	.dev_id		= &clock,
};

static void __init_refok comcerto_timer_init(void)
{
	__raw_writel(0, COMCERTO_TIMER3_CTRL);
	__timer_set(3, 0xFFFFFFFF);

	timer_irq_disable(clock.timer);
	__timer_set(clock.timer, comcerto_pll_bus_clock_hz / HZ);

	clocksource.mult = clocksource_hz2mult(comcerto_pll_bus_clock_hz, clocksource.shift);
	clocksource_register(&clocksource);

	clock.device.mult = div_sc(comcerto_pll_bus_clock_hz, NSEC_PER_SEC, clock.device.shift);
	clock.device.max_delta_ns = clockevent_delta2ns(0x3FFFFFFF, &clock.device);
	clock.device.min_delta_ns = clockevent_delta2ns(1, &clock.device);
	clock.device.cpumask = cpumask_of_cpu(0);
	clockevents_register_device(&clock.device);

	setup_irq(IRQ_TIMERB, &clock_irq);
}

struct sys_timer comcerto_timer =
{
	.init = comcerto_timer_init,
};
