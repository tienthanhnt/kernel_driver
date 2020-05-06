/*
 *  linux/include/asm-arm/arch-comcerto/entry-macro.S
 *
 *  Copyright (C) 2008 Mindspeed Technologies, Inc.
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

#if defined(CONFIG_ARCH_COMCERTO)

#include <asm/arch/hardware.h>

/* define some helpers - we need to have offsets from base to have compact
 * assembly
 */
#define INTC_BASE	APB_VADDR(COMCERTO_INTC_BASE)
#define INTC_STATUS0	(COMCERTO_INTC_STATUS0	- INTC_BASE)
#define INTC_STATUS1	(COMCERTO_INTC_STATUS1	- INTC_BASE)

#ifdef CONFIG_CSP_ARM0

/* ARM0 support is enabled, we may run on ARM0 with ARM1 in reset, this
 * path is a bit slower than if we run on ARM1 only
 *
 * How we detect/support ARM0/ARM1:
 * 1. ARM1 CONTROL register 0x100A0018 bit 0x80000000 is set to '1'
 *    when ARM1 is running. If it's so - we run on ARM1, if it's not
 *    we run on ARM0.
 * 2. ARM1 MASK0 and MASK1 are +8 from corresponding ARM0 registers,
 *    ARM1 WINNER is +64 from ARM0 WINNER
 * 3. ARM1 CONTROL value is shifted right to produce offsets for ARM1
 *    MASKs and WINNER.
 * 4. For K2 we mask out all bits except of 31th since it supports ARM0
 *    enable.
 */
#define INTC_MASK0	(COMCERTO_INTC_ARM0_IRQMASK0	- INTC_BASE)
#define INTC_MASK1	(COMCERTO_INTC_ARM0_IRQMASK1	- INTC_BASE)
#define INTC_WINNER	(COMCERTO_INTC_ARM0_WINNER	- INTC_BASE)
#define INTC_ARM1_CTRL	(COMCERTO_INTC_ARM_CONTROL	- INTC_BASE)

#else

/* ARM0 support is disabled, we run on ARM1 with firmware running on ARM0,
 * simple case - we just need direct offsets for ARM1
 */
#define INTC_MASK0	(COMCERTO_INTC_ARM1_IRQMASK0	- INTC_BASE)
#define INTC_MASK1	(COMCERTO_INTC_ARM1_IRQMASK1	- INTC_BASE)
#define INTC_WINNER	(COMCERTO_INTC_ARM1_WINNER	- INTC_BASE)

#endif

		.macro	disable_fiq
		.endm

		.macro  get_irqnr_preamble, base, tmp
		.endm

		.macro  arch_ret_to_user, tmp1, tmp2
		.endm

#if	defined(CONFIG_ARCH_M823XX)
		.macro get_irqnr_and_base, irqnr, irqstat, base, tmp

		/* Use priority winner int number for STATUS0. WINNER contains 0x20
		 * if no interrupt on STATUS0 was detected or
		 * int number otherwise.
		 */
		ldr	\base, =INTC_BASE		/* load base address, it's needed for all cases */
#ifdef CONFIG_CSP_ARM0
		ldr	\tmp, [\base, #INTC_ARM1_CTRL]
	#ifdef CONFIG_ARCH_M823V2
		and	\tmp, \tmp, #0x80000000		/* mask out ARM0 enable bit for K2 */
	#endif
		add	\base, \tmp, lsr #25		/* add +64 for the ARM1 case */
#endif
		ldr	\irqnr, [\base, #INTC_WINNER]
		cmp	\irqnr, #0			/* zero corresponds to STATUS1 interrupts */
		beq	1002f

		cmp	\irqnr, #32			/* set a flag (irq found or not) for the upper-level */
		b	1001f				/* and go out unconditionally, since we checked both STATUS[01] at this point */

1002:
#ifdef CONFIG_CSP_ARM0
		sub	\base, \tmp, lsr #25		/* restore base to address STATUS1 */
#endif
		/* no loops and branches - it's modern ARM */
		ldr	\irqstat, [\base, #INTC_STATUS1]
#ifdef CONFIG_CSP_ARM0
		add	\base, \tmp, lsr #28		/* add +8 for the ARM1 case */
#endif
		ldr	\tmp, [\base, #INTC_MASK1]
		ands	\irqstat, \irqstat, \tmp	/* if nothing left we use cond. prefix on following opcodes instead of branch */
		clzne	\irqstat, \irqstat		/* count leading zeroes */
		rsbne	\irqnr, \irqstat, #63		/* normalize to the int number (+32 for the STATUS1 interrupts) */

1001:
		.endm
#elif	defined(CONFIG_ARCH_M825XX1)
		.macro get_irqnr_and_base, irqnr, irqstat, base, tmp

		/* no priority support - try masked STATUS0 */
		ldr	\base, =INTC_BASE
		ldr	\irqstat, [\base, #INTC_STATUS0]
#ifdef CONFIG_CSP_ARM0
		ldr	\tmp, [\base, #INTC_ARM1_CTRL]
		add	\base, \tmp, lsr #28		/* add +8 for the ARM1 case */
#endif
		ldr	\tmp, [\base, #INTC_MASK0]
		ands	\irqnr, \irqstat, \tmp		/* mask status to check what we effectively have for this ARM */
		bne	1001f				/* there's an irq on STATUS0 - check it */

		/* no interrupts on STATUS0, check if we have them on STATUS1 */
		ands	\irqstat, \irqstat, #1		/* note that we use non-masked status bits */
		bne	1002f

		/* no interrupts on STATUS1, set a 'no-irq' flag and go out */				
		movs	\irqnr, #0			/* set a 'no-irq' flag for the upper code */
		b	1003f

1001:
		/* loop masked status to detect first set bit */
		mov	\irqstat, \irqnr
		mov	\irqnr, #0			/* start from zero */

1004:		tst	\irqstat, #1
		bne	1003f				/* found interrupt, go out */
		add	\irqnr, \irqnr, #1
		mov	\irqstat, \irqstat, lsr #1
		cmp	\irqnr, #32
		bcc	1004b				/* loop until we tried all bits */

		/* in case smth went wrong and we found nothing on STATUS0 it's safe to fail to STATUS1 check */

1002:
		/* check STATUS1 */
#ifndef CONFIG_CSP_ARM0
		ldr	\irqstat, [\base, #INTC_STATUS1]
#else
		ldr	\base, =INTC_BASE
		ldr	\irqstat, [\base, #INTC_STATUS1]
		ldr	\tmp, [\base, #INTC_ARM1_CTRL]
		add	\base, \tmp, lsr #28		/* add +8 for the ARM1 case */
#endif
		ldr	\tmp, [\base, #INTC_MASK1]
		ands	\irqstat, \irqstat, \tmp
		mov	\irqnr, #32			/* start from 32 and go up to 63 on STATUS1 */

1005:		tst	\irqstat, #1
		bne	1003f
		add	\irqnr, \irqnr, #1
		mov	\irqstat, \irqstat, lsr #1
		cmp	\irqnr, #64
		bcc	1005b

1003:
		.endm
#elif	defined(CONFIG_ARCH_M825XX2) || defined(CONFIG_ARCH_M828XX)
		.macro get_irqnr_and_base, irqnr, irqstat, base, tmp

		/* use WINNER register to check STATUS0 and decide whether we need to check STATUS1 */
		ldr	\base, =INTC_BASE
		ldr	\irqnr, [\base, #INTC_WINNER]
		cmp	\irqnr, #0
		addeq	\irqnr, \irqnr, #32
		beq	1002f

		cmp	\irqnr, #32
		b	1001f

1002:
		/* check STATUS1 */
		ldr	\irqstat, [\base, #INTC_STATUS1]
		ldr	\tmp, [\base, #INTC_MASK1]
		ands	\irqstat, \irqstat, \tmp
		beq	1001f

		/* \irqnr == 32 at this point, see check above */
1003:		tst	\irqstat, #1
		bne	1001f
		add	\irqnr, \irqnr, #1
		mov	\irqstat, \irqstat, lsr #1
		cmp	\irqnr, #64
		bcc	1003b

1001:
		.endm
#elif

#error "Unsupported CPU"

#endif

                .macro  irq_prio_table
                .endm
#endif
