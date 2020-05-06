/*
 * drivers/spi/comcerto_spi_fiq.S
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

#include <linux/linkage.h>
#include <asm/hardware.h>
#include <asm/arch/irqs.h>
#include <asm/arch/spi.h>


.global comcerto_spi_fiq_start
.global comcerto_spi_fiq_end

comcerto_spi_fiq_start:
	ldr	r8, spi0_base
	ldr	r12, intc_status0
	ldr	r9, [r12]
	mov	r11, #0

	ldr	r10, comcerto_spi_intc_fiq_mask
	ldr	r10, [r10]
	and	r9, r9, r10
	tst	r9, #(1 << IRQ_SPI0)
	adrne	r13, comcerto_spi0_fiq_rx_buf
	bne	start_8_bit_per_frame		/* SPI0 run-time data */

	add	r8, r8, #(COMCERTO_SPI1_BASE - COMCERTO_SPI0_BASE)	/* SPI1 base address */
	adr	r13, comcerto_spi1_fiq_rx_buf	/* SPI1 run-time data */

start_8_bit_per_frame:
	ldr	r9, [r8, #COMCERTO_SPI_RXFLR]	/* number of frames in RX FIFO */
	ldr	r11, [r13]			/* load rx buffer address */

	ldr	r12, [r8, #COMCERTO_SPI_CTRLR0]
	tst	r12, #8				/* is 8 or 16 bits per frame selected ? */
	bne	start_16_bit_per_frame

1:
	ldr	r10, [r8, #COMCERTO_SPI_DR]	/* read byte */
	strb	r10, [r11], #1			/* write it to rx buffer, pointer ++ */
	subs	r9, r9, #1
	bne	1b
	str	r11, [r13], #4			/* save rx buffer address, pointer ++ */

	ldr	r9, [r8, #COMCERTO_SPI_TXFLR]	/* number of frames in TX FIFO */
	rsb	r9, r9, #(COMCERTO_SPI_FIFO_DEPTH-1)	/* calculate number of free frames in FIFO, -1 is for shifter */
	ldmia	r13, {r11, r12}			/* load tx counter and tx buffer address */

	cmp	r11, #0
	beq	go_irq				/* completed, transfer control to IRQ level */
1:
	ldrb	r10, [r12], #1			/* load byte from tx buffer, pointer ++*/
	str	r10, [r8, #COMCERTO_SPI_DR]	/* write byte */
	subs	r11, r11, #1
	beq	go_irq				/* completed */
	subs	r9, r9, #1
	bne	1b				/* loop until we have space in FIFO */

go_fiq:
	/* we haven't finished FIQ part yet */

	/* save tx address and counter and ack IRQ */
	stmia	r13, {r11, r12}

	/* ack interrupt */
	ldr	r11, [r13, #8]
	ldr	r9, intc_status0
	str	r11, [r9]
	ldr	r11, [r8, #COMCERTO_SPI_ICR]

	subs	pc, lr, #4

go_irq:
	/* finished FIQ part, don't bother saving tx address and counter, but transfer control to IRQ level */

	/* mask FIQ */
	ldr	r9, comcerto_spi_intc_fiq_mask
	ldr	r10, [r9]
	ldr	r11, [r13, #8]
	bic	r10, r10, r11
	str	r10, [r9]

	/* ack interrupt */
	ldr	r10, intc_status0		/* clear interrupt in INTC */
	str	r11, [r10]
	ldr	r10, [r8, #COMCERTO_SPI_ICR]	/* clear interrupts in controller */

	/* enable IRQ */
	mov	r10, #0
	str	r10, [r8, #COMCERTO_SPI_TXFTLR]	/* enable TX FIFO EMPTY interrupt on completely empty FIFO */
	ldr	r10, [r9, #-4]
	orr	r10, r10, r11
	str	r10, [r9, #-4]

	subs	pc, lr, #4

start_16_bit_per_frame:
1:
	ldr	r10, [r8, #COMCERTO_SPI_DR]	/* read word */
	strh	r10, [r11], #2			/* write it to rx buffer, pointer ++ */
	subs	r9, r9, #1
	bne	1b
	str	r11, [r13], #4			/* save rx buffer address, pointer ++ */

	ldr	r9, [r8, #COMCERTO_SPI_TXFLR]	/* number of frames in TX FIFO */
	rsb	r9, r9, #(COMCERTO_SPI_FIFO_DEPTH-1)	/* calculate number of free frames in FIFO, -1 is for shifter */
	ldmia	r13, {r11, r12}			/* load tx counter and tx buffer address */

	cmp	r11, #0
	beq	go_irq				/* completed, transfer control to IRQ level */
1:
	ldrh	r10, [r12], #2			/* load byte from tx buffer, pointer ++ */
	str	r10, [r8, #COMCERTO_SPI_DR]	/* write byte */
	subs	r11, r11, #2
	beq	go_irq				/* completed */
	subs	r9, r9, #1
	bne	1b
	b	go_fiq

.global	comcerto_spi_intc_fiq_mask
comcerto_spi_intc_fiq_mask:
	.word	0

.global comcerto_spi0_fiq_rx_buf
.global comcerto_spi0_fiq_tx_count
.global comcerto_spi0_fiq_tx_buf
comcerto_spi0_fiq_rx_buf:
	.word	0
comcerto_spi0_fiq_tx_count:
	.word	0
comcerto_spi0_fiq_tx_buf:
	.word	0
	.word	(1 << IRQ_SPI0)

.global comcerto_spi1_fiq_rx_buf
.global comcerto_spi1_fiq_tx_count
.global comcerto_spi1_fiq_tx_buf
comcerto_spi1_fiq_rx_buf:
	.word	0
comcerto_spi1_fiq_tx_count:
	.word	0
comcerto_spi1_fiq_tx_buf:
	.word	0
	.word	(1 << IRQ_SPI1)

spi0_base:
	.word	APB_VADDR(COMCERTO_SPI0_BASE)
intc_status0:
	.word	COMCERTO_INTC_STATUS0		/* no APB_VADDR() wrapper - this value is already translated */

comcerto_spi_fiq_end:
