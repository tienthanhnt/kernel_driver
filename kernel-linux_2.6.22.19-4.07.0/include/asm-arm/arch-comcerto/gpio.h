/*
 *  linux/include/asm-arm/arch-comcerto/gpio.h
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
#ifndef __ASM_ARCH_GPIO_H
#define __ASM_ARCH_GPIO_H

#include <asm/io.h>
#include <asm/arch/irq.h>

#define GPIO_NONE		(-1)
#define GPIO_INPUT		0
#define GPIO_OUTPUT		1
#define GPIO_IRQ		2

#define GPIO_IRQ_EDGE		0x0
#define GPIO_IRQ_LEVEL		0x1

#define GPIO_EDGE_FALLING	0x0
#define GPIO_EDGE_RISING	0x1

#define GPIO_LEVEL_LOW		GPIO_EDGE_FALLING
#define GPIO_LEVEL_HIGH		GPIO_EDGE_RISING


/* GPIO bit mask for a given GPIO pin */
#define comcerto_gpio_mask(gpio)	(1 << ((gpio) & 0x1f))

/* GPIO pin number to IRQ number, keep this macro so it can be used as a static initializer */
#define comcerto_gpio_to_irq(gpio)	(((gpio) & 0x7) + 1 + 32)

/* IRQ number to GPIO pin number, keep this macro so it can be used as a static initializer */
#define comcerto_irq_to_gpio(irq)	((((irq) & 0x1f) - 1) & 0x7)

/* GPIO bit mask for a given IRQ number */
#define comcerto_irq_gpio_mask(irq)	comcerto_gpio_mask(irq_to_gpio(irq))

/* IRQ bit mask for a given GPIO pin */
#define comcerto_gpio_irq_mask(gpio)	comcerto_irq_mask(gpio_to_irq(gpio))


struct comcerto_gpio_desc {
	u32 gpio;
	char type;
	union {
		struct {
			char type;
			char trigger;
		} irq;
		struct {
			char default_value;
		} out;
	} conf;
};


#define comcerto_gpio_read_output()		__raw_readl(COMCERTO_GPIO_OUTPUT)

#define comcerto_gpio_write_output(value)	__raw_writel(value, COMCERTO_GPIO_OUTPUT)

#define comcerto_gpio_ctrl(value, mask) \
	do {												\
		u32 status;										\
		unsigned long flags;									\
		local_irq_save(flags);									\
		while (((status = __raw_readl(COMCERTO_GPIO_IOCTRL)) & (mask)) != (value)) {		\
			__raw_writel(0x55555555, COMCERTO_GPIO_LOCK);					\
			__raw_writel((status & ~(mask)) | (value), COMCERTO_GPIO_IOCTRL);		\
		}											\
		local_irq_restore(flags);								\
	} while (0)

#if !defined(CONFIG_ARCH_M823XX)
void comcerto_handle_gpio_level_irq(unsigned int irq, struct irq_desc *desc);
#endif

void __init comcerto_gpio_init(struct comcerto_gpio_desc *gpio_table, int size);

static inline void gpio_set_value(unsigned gpio, int value);

/* Generic GPIO support */

static inline int gpio_request(unsigned gpio, const char *label)
{
	return 0;
}

static inline void gpio_free(unsigned gpio)
{
	return;
}

static inline int gpio_direction_input(unsigned gpio)
{
	__raw_writel(__raw_readl(COMCERTO_GPIO_OUTPUT_ENABLE) & ~comcerto_gpio_mask(gpio), COMCERTO_GPIO_OUTPUT_ENABLE);

	return 0;
}

static inline int gpio_direction_output(unsigned gpio, int level)
{
	gpio_set_value(gpio, level);
	__raw_writel(__raw_readl(COMCERTO_GPIO_OUTPUT_ENABLE) | comcerto_gpio_mask(gpio), COMCERTO_GPIO_OUTPUT_ENABLE);

	return 0;
}

/* This function returns value of the corresponding *input* pin.
 * Use comcerto_gpio_read_output() for reading output pins state.
 */
static inline int gpio_get_value(unsigned gpio)
{
	u32 in_reg = __raw_readl(COMCERTO_GPIO_INPUT) & comcerto_gpio_mask(gpio);

	return in_reg ? 1 : 0;
}

static inline void gpio_set_value(unsigned gpio, int value)
{
	u32 out_reg, mask;

	mask = comcerto_gpio_mask(gpio);
	out_reg = comcerto_gpio_read_output() & ~mask;
	if (value)
		out_reg |= mask;

	comcerto_gpio_write_output(out_reg);
}

#include <asm-generic/gpio.h>		/* cansleep wrappers */

static inline int gpio_to_irq(unsigned gpio)
{
	return comcerto_gpio_to_irq(gpio);
}

static inline int irq_to_gpio(unsigned irq)
{
	return comcerto_irq_to_gpio(irq);
}

#endif /* __ASM_ARCH_GPIO_H */
