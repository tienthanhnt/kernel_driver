/*
 *  linux/arch/arm/mach-comcerto/gpio.c
 *
 *  Copyright (C) 2006 Mindspeed Technologies, Inc.
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <asm/io.h>
#include <asm/arch/hardware.h>
#include <asm/arch/gpio.h>


#if !defined(CONFIG_ARCH_M823XX)
static u8 gpio_irq_level[8];

/* The GPIO IRQ block generates interrupts only on rising/falling edges of the GPIO pin signal.
 * To avoid loosing interrupts or having spurious interrupts care must be taken.
 * The general strategy is to loop and poll the GPIO pin to make sure no interrupts are missed.
 * The GPIO IRQ must be acked inside the loop at each iteration. If it was acked
 * before the loop there would be a race condition(1) where we exit comcerto_handle_gpio_level_irq() with
 * the GPIO IRQ set, even if the source was already handled. If it was acked after the loop
 * there would be a race condition(2) where we ack a GPIO IRQ but the source is not yet handled.
 * The GPIO IRQ must be acked after all the driver handlers have been called (after handle_simple_irq())
 * to also avoid the race mentioned in (1) above.
 */
void comcerto_handle_gpio_level_irq(unsigned int irq, struct irq_desc *desc)
{
	int gpio = irq_to_gpio(irq);
	char pending;

	spin_lock(&desc->lock);

	desc->chip->mask(irq);

	do {
		spin_unlock(&desc->lock);

		handle_simple_irq(irq, desc);

		spin_lock(&desc->lock);

		desc->chip->ack(irq);

		if (!desc->action || (desc->status & IRQ_DISABLED))
			break;

		pending = gpio_get_value(gpio);

		if (gpio_irq_level[gpio] == GPIO_LEVEL_LOW)
			pending = (pending == 0);
	} while (pending);

	desc->chip->unmask(irq);

	spin_unlock(&desc->lock);
}
#endif

void __init comcerto_gpio_init(struct comcerto_gpio_desc *gpio_table, int size)
{
	int i;
	int gpio, gpio_mask;

	printk(KERN_INFO "Comcerto GPIO(s):\n");

	for (i = 0; i < size; i++) {
		gpio = gpio_table[i].gpio;
		gpio_mask = comcerto_gpio_mask(gpio);

		switch (gpio_table[i].type)
		{
		case GPIO_IRQ:
			printk(KERN_INFO "%d(irq=%d, type=%d, trigger=%d)\n",
					 gpio,
					 gpio_to_irq(gpio),
					 gpio_table[i].conf.irq.type,
					 gpio_table[i].conf.irq.trigger
			);

			if (gpio_table[i].conf.irq.type == GPIO_IRQ_LEVEL) {
#if defined(CONFIG_ARCH_M823XX)
				u32 bits;

				if (gpio < 8)	/* for the GPIO 8..31 we unmask it when request_irq is called */
					__raw_writel(__raw_readl(COMCERTO_GPIO_LEVEL_INT_ENABLE) | gpio_mask, COMCERTO_GPIO_LEVEL_INT_ENABLE);

				bits = __raw_readl(COMCERTO_GPIO_LEVEL_INT_POLARITY) & ~gpio_mask;
				if (gpio_table[i].conf.irq.trigger == GPIO_LEVEL_LOW)
					bits |= gpio_mask;
				__raw_writel(bits, COMCERTO_GPIO_LEVEL_INT_POLARITY);
#else

				gpio_irq_level[gpio] = gpio_table[i].conf.irq.trigger;

				if (gpio_table[i].conf.irq.trigger == GPIO_LEVEL_HIGH)
					__raw_writel(__raw_readl(COMCERTO_GPIO_RISING_EDGE_INT_ENABLE) | gpio_mask, COMCERTO_GPIO_RISING_EDGE_INT_ENABLE);
				else
					__raw_writel(__raw_readl(COMCERTO_GPIO_FALLING_EDGE_INT_ENABLE) | gpio_mask, COMCERTO_GPIO_FALLING_EDGE_INT_ENABLE);
#endif
			} else {
				if (gpio_table[i].conf.irq.trigger == GPIO_EDGE_RISING)
					__raw_writel(__raw_readl(COMCERTO_GPIO_RISING_EDGE_INT_ENABLE) | gpio_mask, COMCERTO_GPIO_RISING_EDGE_INT_ENABLE);
				else
					__raw_writel(__raw_readl(COMCERTO_GPIO_FALLING_EDGE_INT_ENABLE) | gpio_mask, COMCERTO_GPIO_FALLING_EDGE_INT_ENABLE);
			}

			break;

		case GPIO_INPUT:
		default:
			gpio_direction_input(gpio);
			printk(KERN_INFO "%d(in =%d)\n", gpio, gpio_get_value(gpio));
			break;

		case GPIO_OUTPUT:
			gpio_direction_output(gpio, gpio_table[i].conf.out.default_value);
			printk(KERN_INFO "%d(out=%d)\n", gpio, (comcerto_gpio_read_output() & gpio_mask) ? 1 : 0);
			break;
		}
	}
}
