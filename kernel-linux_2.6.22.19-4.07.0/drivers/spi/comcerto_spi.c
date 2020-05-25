/*
 * drivers/spi/comcerto_spi.c
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

#include <linux/version.h>
#include <linux/autoconf.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <asm/delay.h>
#include <asm/fiq.h>
#include <asm/io.h>
#include <asm/arch/spi.h>


MODULE_AUTHOR("Mindspeed Technologies, Inc.");
MODULE_DESCRIPTION("Comcerto SPI bus driver");
MODULE_LICENSE("GPL");


#define SPI_VERSION             "1.0.0"

#define SPI_BAUDR_MIN		2
#define SPI_BAUDR_MAX		0xFFFE
#define SPI_SPEED_MAX		(4*1000*1000)
#define SPI_SPEED_MIN		(comcerto_pll_bus_clock_hz / SPI_BAUDR_MAX)
#define SPI_FRAME_SIZE_MIN	4
#define SPI_FRAME_SIZE_MAX	16
#define SPI_CHIP_SELECT_MAX	15


#define spi_err(dev, fmt, args...)	dev_err(dev, "%s: " fmt, __FUNCTION__, ##args)
#define spi_dbg(dev, fmt, args...)	dev_dbg(dev, "%s: " fmt, __FUNCTION__, ##args)
#define spi_warn(dev, fmt, args...)	dev_warn(dev, "%s: " fmt, __FUNCTION__, ##args)


static int loopback = 0;
module_param(loopback, bool, S_IRUGO);
MODULE_PARM_DESC(loopback, "Force loopback mode: 0=normal mode, 1=loopback mode");
static int polling = 0;
module_param(polling, bool, S_IRUGO);
MODULE_PARM_DESC(polling, "Force polling mode: 0=interrupt mode (if possible), 1=polling mode");


struct comcerto_spi
{
	ulong			membase;
	int			irq;
	struct device		*dev;
	struct spi_master	*master;

	spinlock_t		lock;

	struct spi_message	*message;	/* current message */
	struct spi_transfer	*rx_tr;		/* the first transfer that was put into buffer, needed to update rx_buf when buffer is completed */
	struct spi_transfer	*tx_tr;		/* the last transfer that was put into buffer, needed to walk through the message */
	int			rx_len;		/* number of bytes read */
	int			tx_len;		/* number of bytes written */

	/* core parameters, to avoid reading them from regs */
	u8			chip_select;
	u8			mode;
	u8			bits_per_word;
	u8			bytes_per_word;
	u8			ssienr;
	u32			ctrlr0;
	u32			speed_hz;

#ifdef CONFIG_SPI_COMCERTO_FIQ
	u32			**fiq_rx_buf;
	u32			**fiq_tx_buf;
	u32			*fiq_tx_count;
	u32			fiq_mask;
	u32			fiq_mask_reg;
	struct tasklet_struct	queue_tasklet;
#endif
	int			len;		/* buffer length, bytes */
	u8			*buf;		/* linearized transfer buffer */
	u8			buf_space[1];
};


#ifdef CONFIG_SPI_COMCERTO_FIQ
extern u32 comcerto_spi_intc_fiq_mask;
extern u32 *comcerto_spi0_fiq_rx_buf;
extern u32 *comcerto_spi0_fiq_tx_buf;
extern u32 comcerto_spi0_fiq_tx_count;
extern u32 *comcerto_spi1_fiq_rx_buf;
extern u32 *comcerto_spi1_fiq_tx_buf;
extern u32 comcerto_spi1_fiq_tx_count;
#endif

static inline int fifo_rx_used(struct comcerto_spi *adapter)
{
	return __raw_readl(adapter->membase + COMCERTO_SPI_RXFLR) * adapter->bytes_per_word;
}

static inline int fifo_tx_used(struct comcerto_spi *adapter)
{
	return __raw_readl(adapter->membase + COMCERTO_SPI_TXFLR) * adapter->bytes_per_word;
}

static inline int fifo_tx_free(struct comcerto_spi *adapter)
{
	return COMCERTO_SPI_FIFO_DEPTH*adapter->bytes_per_word - fifo_tx_used(adapter);
}

static void spi_set_ssienr(struct comcerto_spi *adapter, u8 ssienr)
{
	__raw_writel(ssienr, adapter->membase + COMCERTO_SPI_SSIENR);
	adapter->ssienr = ssienr;
}

static void spi_set_speed(struct comcerto_spi *adapter, u32 speed_hz)
{
	u32 baudr;
	u32 real_speed_hz;

	if (speed_hz > 0) {
		/* calculate divisor, it must be even */
		baudr = (comcerto_pll_bus_clock_hz/speed_hz + 1) & ~1;

		if (unlikely(baudr < SPI_BAUDR_MIN))
			baudr = SPI_BAUDR_MIN;
		if (unlikely(baudr > SPI_BAUDR_MAX))
			baudr = SPI_BAUDR_MAX;

		real_speed_hz = comcerto_pll_bus_clock_hz / baudr;
	} else
		baudr = 0;

	if (baudr)
		spi_dbg(adapter->dev, "requested %uHz (real %uHz), divisor %#x\n",
			speed_hz, real_speed_hz, baudr);
	else
		spi_dbg(adapter->dev, "disable serial clock\n");
	__raw_writel(baudr, adapter->membase + COMCERTO_SPI_BAUDR);

	adapter->speed_hz = speed_hz;	/* note that we save requested speed */
}

static void spi_set_control(struct comcerto_spi *adapter, u8 mode, u8 bits_per_word)
{
	u32 ctrlr0 = adapter->ctrlr0;

	/* mask out bits we may touch */
	ctrlr0 &= ~(SPI_CTRLR0_SCPOL | SPI_CTRLR0_SCPH | SPI_CTRLR0_DFS_MASK);

	if (mode & SPI_CPOL)
		ctrlr0 |= SPI_CTRLR0_SCPOL;

	if (mode & SPI_CPHA)
		ctrlr0 |= SPI_CTRLR0_SCPH;

	ctrlr0 |= (bits_per_word - 1) & 15;

	__raw_writel(ctrlr0, adapter->membase + COMCERTO_SPI_CTRLR0);

	adapter->ctrlr0 = ctrlr0;
	adapter->mode = mode;
	adapter->bits_per_word = bits_per_word;
	adapter->bytes_per_word = bits_per_word/9 + 1;
}

static void spi_reset(struct comcerto_spi *adapter)
{
	unsigned long flags;

	spin_lock_irqsave(&adapter->lock, flags);

	__raw_writel(0, adapter->membase + COMCERTO_SPI_IMR);	/* mask all IRQs */

	spi_set_ssienr(adapter, 0);	/* disable operations: halt transfers, clear FIFOs */
	spi_set_speed(adapter, 0);	/* disable serial clock */

	/* set defaults
	 * ---
	 * 15:12 CFS	0000 control frame size (not supported in Motorola SPI mode)
	 *    11 SRL	   0 shift register loop (normal mode operation)
	 *    10 SLV_OE	   0 slave output enable (not used for master device)
	 *  9: 8 TMOD	   0 transfer mode (read-write)
	 *  5: 4 FRF	  00 frame format (Motorola SPI)
	 */

	/* this call will setup SCPOL, SCPH and DFS fields and write value to register */
	spi_set_control(adapter, SPI_CPOL | SPI_CPHA, 8);

	spin_unlock_irqrestore(&adapter->lock, flags);
}

static int __spi_message_setup(struct comcerto_spi *adapter, struct spi_device *spi, struct spi_transfer *tr)
{
	u32 speed_hz = spi->max_speed_hz;
	u8 bits_per_word = spi->bits_per_word;

	/* we at the ::bits_per_word and ::speed_hz parameters of the first transfer,
	 * it's possible to override device parameters at the start of series of transfers
	 * which use continous chip select
	 */
	if (tr->bits_per_word) {
		bits_per_word = tr->bits_per_word;
		if (bits_per_word < SPI_FRAME_SIZE_MIN || bits_per_word > SPI_FRAME_SIZE_MAX)
			return -EINVAL;
	}

	if (tr->speed_hz) {
		speed_hz = tr->speed_hz;
		if (speed_hz < SPI_SPEED_MIN || speed_hz > SPI_SPEED_MAX)
			return -EINVAL;
	}

	spi_set_ssienr(adapter, 0);	/* SSI disable to allow programming registers below and reset FIFOs */

	__raw_writel(0, adapter->membase + COMCERTO_SPI_SER);	/* zero to chip select to disable transfer start before we filled FIFO */
	adapter->chip_select = spi->chip_select;

	if (speed_hz != adapter->speed_hz)
		spi_set_speed(adapter, speed_hz);

	if (spi->mode != adapter->mode || bits_per_word != adapter->bits_per_word)
		spi_set_control(adapter, spi->mode, bits_per_word);

	return 0;
}

static inline void fifo_read(struct comcerto_spi *adapter, int len)
{
	int n = len;

	if (likely(adapter->bits_per_word <= 8)) {
		u8 *buf = adapter->buf + adapter->rx_len;

		while (n--)
			*buf++ = __raw_readl(adapter->membase + COMCERTO_SPI_DR);
	} else {
		u16 *buf = (u16*)(adapter->buf + adapter->rx_len);

		n >>= 1;
		while (n--)
			*buf++ = __raw_readl(adapter->membase + COMCERTO_SPI_DR);
	}

	adapter->rx_len += len;
}

static inline void fifo_write(struct comcerto_spi *adapter, int len)
{
	int n = len;

	if (likely(adapter->bits_per_word <= 8)) {
		u8 *buf = adapter->buf + adapter->tx_len;

		while (n--)
			__raw_writel(*buf++, adapter->membase + COMCERTO_SPI_DR);
	} else {
		u16 *buf = (u16*)(adapter->buf + adapter->tx_len);

		n >>= 1;
		while (n--)
			__raw_writel(*buf++, adapter->membase + COMCERTO_SPI_DR);
	}

	/* if FIFO filled for the first time - start transfer by setting chip select */
	if (adapter->tx_len == 0)
		__raw_writel(adapter->chip_select, adapter->membase + COMCERTO_SPI_SER);

	adapter->tx_len += len;
}

static int buffer_fill(struct comcerto_spi *adapter)
{
	struct spi_message *msg = adapter->message;
	struct spi_transfer *tr = adapter->tx_tr;

	adapter->len = 0;
	adapter->rx_len = 0;
	adapter->tx_len = 0;

	while (1) {
		if (adapter->len + tr->len > PAGE_SIZE)
			return -E2BIG;

		if (tr->tx_buf) {
			memcpy(adapter->buf + adapter->len, tr->tx_buf, tr->len);
			msg->actual_length += tr->len;
		} else
			memzero(adapter->buf + adapter->len, tr->len);

		adapter->len += tr->len;

		if (list_is_last(&tr->transfer_list, &msg->transfers))
			break;

		tr = list_entry(tr->transfer_list.next, struct spi_transfer, transfer_list);

		if (tr->delay_usecs || tr->cs_change)
			break;
	}

	adapter->tx_tr = tr;

	return 0;
}

static void buffer_empty(struct comcerto_spi *adapter)
{
	int len;
	struct spi_message *msg = adapter->message;
	struct spi_transfer *tr = adapter->rx_tr;

	len = 0;

	while (1) {
		if (tr->rx_buf) {
			memcpy(tr->rx_buf, adapter->buf + len, tr->len);
			msg->actual_length += tr->len;
		}

		len += tr->len;

		tr = list_entry(tr->transfer_list.next, struct spi_transfer, transfer_list);

		if (tr == adapter->tx_tr)
			break;
	}

	adapter->rx_tr = tr;
}

static void buffer_process(struct comcerto_spi *adapter)
{
	int len;

	spi_set_ssienr(adapter, 1);

	do {
		len = fifo_rx_used(adapter);
		if (len)
			fifo_read(adapter, len);

		if (adapter->tx_len < adapter->len) {
			/* take in account shifter reg, if we fill FIFO up when shifter is filled
			 * we may get rx overrun if something delays reception path
			 */
			len = fifo_tx_free(adapter) - 1;
			if (len > 0) {
				if (len > (adapter->len - adapter->tx_len))
					len = adapter->len - adapter->tx_len;
				fifo_write(adapter, len);
			}
		}
	} while (adapter->rx_len < adapter->len);
}

#ifdef CONFIG_SPI_COMCERTO_FIQ
static void buffer_start(struct comcerto_spi *adapter)
{
	int len;

	spi_set_ssienr(adapter, 1);

	len = fifo_tx_free(adapter) - 1;
	if (len > (adapter->len - adapter->tx_len))
		len = adapter->len - adapter->tx_len;
	fifo_write(adapter, len);
}

static void spi_int_setup(struct comcerto_spi *adapter)
{
	buffer_start(adapter);

	if (adapter->tx_len >= adapter->len) {
		/* everything is in TX FIFO, no FIQ is needed, unmask IRQ */

		*adapter->fiq_rx_buf = (u32*) adapter->buf;	/* this field is used by IRQ handler */

		__raw_writel(0, adapter->membase + COMCERTO_SPI_TXFTLR);
		__raw_writel(1, adapter->membase + COMCERTO_SPI_IMR);

		__raw_writel(__raw_readl(adapter->fiq_mask_reg-4) | adapter->fiq_mask, adapter->fiq_mask_reg-4);
		__raw_writel(__raw_readl(adapter->fiq_mask_reg) & ~adapter->fiq_mask, adapter->fiq_mask_reg);
	} else {
		/* unmask FIQ, we need to refill TX FIFO */

		*adapter->fiq_rx_buf = (u32*) adapter->buf;
		*adapter->fiq_tx_buf = (u32*) ((u8*)adapter->buf + adapter->tx_len);
		*adapter->fiq_tx_count = adapter->len - adapter->tx_len;

		__raw_writel(8, adapter->membase + COMCERTO_SPI_TXFTLR);
		__raw_writel(1, adapter->membase + COMCERTO_SPI_IMR);

		__raw_writel(__raw_readl(adapter->fiq_mask_reg-4) & ~adapter->fiq_mask, adapter->fiq_mask_reg-4);
		__raw_writel(__raw_readl(adapter->fiq_mask_reg) | adapter->fiq_mask, adapter->fiq_mask_reg);
	}
}
#endif

/*
 * Here we get after either from IRQ after completing tx part of buffer in FIQ or from
 * the transfer() function after filling the buffer, we need to finish processing
 * completely and switch to the rest of transfers. If all transfers are completed -
 * message is completed as well. Switch to the next message is handled in upper
 * level code.
 *
 * Return value indicates the status:
 * < 0 - message failed, error status updated
 * = 0 - message completed ok
 * > 0 - control transferred to FIQ/IRQ, IRQ handler will schedule tasklet for completion
 */
static int __spi_message_continue(struct comcerto_spi *adapter)
{
	int rc = 0;
	struct spi_message *msg = adapter->message;

	while (1) {
		buffer_process(adapter);
		buffer_empty(adapter);

		if (list_is_last(&adapter->rx_tr->transfer_list, &msg->transfers)) {
			/* this is the last transfer, indicate that we need to complete */
			spi_set_ssienr(adapter, 0);

			break;
		}

		rc = buffer_fill(adapter);
		if (rc)
			break;

		rc = __spi_message_setup(adapter, msg->spi, adapter->rx_tr);
		if (rc)
			break;

#ifdef CONFIG_SPI_COMCERTO_FIQ
		if (polling <= 0 && adapter->len > 8) {
			/* transfer is long enough to go via FIQ */
			spi_int_setup(adapter);
			return 1;	/* indicate that we went into FIQ */
		}
#endif
	}

	spi_set_ssienr(adapter, 0);

	msg->status = rc;

	return rc;
}

static int __spi_message_start(struct comcerto_spi *adapter, struct spi_message *msg)
{
	int rc;

	msg->status = -EINPROGRESS;
	msg->actual_length = 0;

	adapter->message = msg;
	adapter->rx_tr = list_first_entry(&msg->transfers, struct spi_transfer, transfer_list);
	adapter->tx_tr = adapter->rx_tr;

	rc = buffer_fill(adapter);
	if (rc)
		goto err;

	rc = __spi_message_setup(adapter, msg->spi, adapter->rx_tr);
	if (rc)
		goto err;

#ifdef CONFIG_SPI_COMCERTO_FIQ
	if (polling <= 0 && adapter->len > 8) {
		/* transfer is long enough to go via interrupt */
		spi_int_setup(adapter);
		goto fiq;
	}
#endif
	rc = __spi_message_continue(adapter);
	if (rc > 0)
		goto fiq;		/* control was transferred to FIQ, just return */

	spi_set_ssienr(adapter, 0);

err:
	msg->status = rc;
	msg->complete(msg->context);

	adapter->message = NULL;	/* completed, mark we don't have queued messages */
	return rc;

fiq:
	return 1;
}

static int comcerto_spi_transfer(struct spi_device *spi, struct spi_message *msg)
{
	struct comcerto_spi *adapter = spi_master_get_devdata(spi->master);
	int rc = 0;
	unsigned long flags;

	if (list_empty(&msg->transfers))
		return -EINVAL;

	spin_lock_irqsave(&adapter->lock, flags);

#ifdef CONFIG_SPI_COMCERTO_FIQ
	INIT_LIST_HEAD(&msg->queue);

	if (adapter->message) {
		list_add_tail(&msg->queue, &adapter->message->queue);
		spin_unlock_irqrestore(&adapter->lock, flags);
		return 0;
	}
#endif

	rc = __spi_message_start(adapter, msg);
	if (rc > 0)
		rc = 0;

	spin_unlock_irqrestore(&adapter->lock, flags);

	return rc;
}

static int comcerto_spi_setup(struct spi_device *spi)
{
	int rc = 0;

	spi_dbg(&spi->dev, "bits per word %u, max speed %uHz, mode %#x\n",
		spi->bits_per_word, spi->max_speed_hz, spi->mode);

	if (!spi->bits_per_word)
		spi->bits_per_word = 8;

	if (spi->bits_per_word < SPI_FRAME_SIZE_MIN || spi->bits_per_word > SPI_FRAME_SIZE_MAX) {
		spi_err(&spi->dev, "bits per word (frame size) %u out of range %u..%u\n",
			spi->max_speed_hz, SPI_FRAME_SIZE_MIN, SPI_FRAME_SIZE_MAX);
		rc = -EINVAL;
		goto err;
	}

	if (spi->max_speed_hz < SPI_SPEED_MIN) {
		spi_err(&spi->dev, "such low speed %u isn't supported, min is %u\n",
			spi->max_speed_hz, SPI_SPEED_MIN);
		rc = -EINVAL;
		goto err;
	}

	if (spi->max_speed_hz > SPI_SPEED_MAX) {
		spi_warn(&spi->dev, "decreasing speed %u to max supported %u\n",
			spi->max_speed_hz, SPI_SPEED_MAX);
		spi->max_speed_hz = SPI_SPEED_MAX;
	}

	if (spi->chip_select > SPI_CHIP_SELECT_MAX) {
		spi_err(&spi->dev, "chip select %u out of range 0..%u\n",
			spi->max_speed_hz, SPI_CHIP_SELECT_MAX);
		rc = -EINVAL;
		goto err;
	}

	if (spi->mode & SPI_CS_HIGH) {
		spi_err(&spi->dev, "chip select active high isn't supported\n");
		rc = -EINVAL;
		goto err;
	}

	if (spi->mode & SPI_LSB_FIRST) {
		spi_err(&spi->dev, "LSB first mode isn't supported\n");
		rc = -EINVAL;
		goto err;
	}

err:
	return rc;
}

void comcerto_spi_set_loopback(struct spi_master *master, int loopback)
{
	struct comcerto_spi *adapter = spi_master_get_devdata(master);
	unsigned long flags;

	spin_lock_irqsave(&adapter->lock, flags);

	if (loopback)
		adapter->ctrlr0 |= SPI_CTRLR0_SRL;
	else
		adapter->ctrlr0 &= ~SPI_CTRLR0_SRL;

	__raw_writel(adapter->ctrlr0, adapter->membase + COMCERTO_SPI_CTRLR0);

	spin_unlock_irqrestore(&adapter->lock, flags);
}

#ifdef CONFIG_SPI_COMCERTO_FIQ
/*
 * Current adapter::message is completed, call completion and check whether we have
 * anything else to do.
 */
static void spi_message_next(ulong arg)
{
	struct comcerto_spi *adapter = (void*) arg;
	struct spi_message *msg = adapter->message;
	unsigned long flags;

	spin_lock_irqsave(&adapter->lock, flags);

	msg->complete(msg->context);

	if (!list_empty(&msg->queue)) {
		struct spi_message *msg_next;

		msg_next = list_entry(msg->queue.next, struct spi_message, queue);

		list_del(&msg->queue);

		__spi_message_start(adapter, msg_next);
	} else
		adapter->message = NULL;

	spin_unlock_irqrestore(&adapter->lock, flags);
}

/*
 * IRQ handler is called when FIQ completed tx part of transfer.
 * It finishes rx part and tries to complete the message. Next message
 * can be picked by tasklet function spi_message_next().
 */
static irqreturn_t spi_interrupt(int irq, void *dev_id)
{
	int rc = 0;
	struct comcerto_spi *adapter = dev_id;
	struct spi_message *msg = adapter->message;

	__raw_writel(0, adapter->membase + COMCERTO_SPI_IMR);		/* disable IRQ */

	/* here we get after completing tx part of buffer, we need to finish rx
	 * part completely and switch to another part of message
	 */
	adapter->tx_len = adapter->len;					/* no tx required */
	adapter->rx_len = (u8*)(*adapter->fiq_rx_buf) - adapter->buf;	/* calculate how much we have processed on rx path */

	rc = __spi_message_continue(adapter);
	if (rc > 0)
		goto fiq;		/* positive return value: control was transferred to FIQ again */

	/* message either completed or failed, schedule tasklet to finish it and pick the next if any */
	spi_set_ssienr(adapter, 0);
	msg->status = rc;
	tasklet_schedule(&adapter->queue_tasklet);

fiq:
	return IRQ_HANDLED;
}
#endif

static int __init comcerto_spi_probe(struct platform_device *pdev)
{
	struct spi_master *master;
	struct comcerto_spi *adapter;
	struct resource *res;
	int rc = -EINVAL;
	printk("====== %s\n", __FUNCTION__);
	spi_dbg(&pdev->dev, "registering device\n");

	master = spi_alloc_master(&pdev->dev, sizeof(struct comcerto_spi) + PAGE_SIZE + 32);
	if (!master) {
		spi_err(&pdev->dev, "couldn't allocate memory\n");
		rc = -ENOMEM;
		goto err0;
	}

	adapter = spi_master_get_devdata(master);
	adapter->dev = master->cdev.dev;
	adapter->master = master;
	adapter->buf = (void*) (((u32)&adapter->buf_space[0] + 31) & ~31);

	spin_lock_init(&adapter->lock);

#ifdef CONFIG_SPI_COMCERTO_FIQ
	if (polling <= 0) {
		extern u8 comcerto_spi_fiq_start, comcerto_spi_fiq_end;

		adapter->irq = platform_get_irq(pdev, 0);
		if (adapter->irq < 0) {
			spi_err(&pdev->dev, "no IRQ resource\n");
			rc = -EINVAL;
			goto err1;
		}

		if (request_irq(adapter->irq, spi_interrupt, IRQF_DISABLED, pdev->name, adapter)) {
			spi_err(&pdev->dev, "failed to request IRQ%d\n", adapter->irq);
			goto err1;
		}

		if (pdev->id == 0) {
			adapter->fiq_rx_buf = (void*) (0xFFFF001C + (u32) &comcerto_spi0_fiq_rx_buf - (u32) &comcerto_spi_fiq_start);
			adapter->fiq_tx_buf = (void*) (0xFFFF001C + (u32) &comcerto_spi0_fiq_tx_buf - (u32) &comcerto_spi_fiq_start);
			adapter->fiq_tx_count = (void*) (0xFFFF001C + (u32) &comcerto_spi0_fiq_tx_count - (u32) &comcerto_spi_fiq_start);
			adapter->fiq_mask = 1 << IRQ_SPI0;
		} else {
			adapter->fiq_rx_buf = (void*) (0xFFFF001C + (u32) &comcerto_spi1_fiq_rx_buf - (u32) &comcerto_spi_fiq_start);
			adapter->fiq_tx_buf = (void*) (0xFFFF001C + (u32) &comcerto_spi1_fiq_tx_buf - (u32) &comcerto_spi_fiq_start);
			adapter->fiq_tx_count = (void*) (0xFFFF001C + (u32) &comcerto_spi1_fiq_tx_count - (u32) &comcerto_spi_fiq_start);
			adapter->fiq_mask = 1 << IRQ_SPI1;
		}

		if (comcerto_arm1_is_running()) {
			adapter->fiq_mask_reg = COMCERTO_INTC_ARM1_FIQMASK0;
			comcerto_spi_intc_fiq_mask = COMCERTO_INTC_ARM1_FIQMASK0;
		} else {
			adapter->fiq_mask_reg = COMCERTO_INTC_ARM0_FIQMASK0;
			comcerto_spi_intc_fiq_mask = COMCERTO_INTC_ARM0_FIQMASK0;
		}

		set_fiq_handler(&comcerto_spi_fiq_start, &comcerto_spi_fiq_end - &comcerto_spi_fiq_start);
	}

	tasklet_init(&adapter->queue_tasklet, spi_message_next, (ulong) adapter);
#endif

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		spi_err(&pdev->dev, "no memory resource\n");
        	rc = -EINVAL;
        	goto err1;
	}

	/* should be mapped in arch/arm/mach-comcerto/comcerto-xxx.c */
	adapter->membase = APB_VADDR(res->start);

	spi_reset(adapter);
	comcerto_spi_set_loopback(master, loopback);

	master->bus_num		= pdev->id;
	master->num_chipselect	= 16;
	master->setup		= comcerto_spi_setup;
	master->transfer	= comcerto_spi_transfer;
	master->cleanup		= NULL;

	platform_set_drvdata(pdev, master);

	rc = spi_register_master(master);
        if (rc != 0) {
        	spi_err(&pdev->dev, "error registering SPI master\n");
        	goto err1;
	}

	return 0;

err1:
	spi_master_put(master);

err0:
	return rc;
}

static int comcerto_spi_remove(struct platform_device *pdev)
{
	struct spi_master *master = platform_get_drvdata(pdev);
	struct comcerto_spi *adapter = spi_master_get_devdata(master);

#ifdef CONFIG_SPI_COMCERTO_FIQ
	if (polling <= 0)
		free_irq(adapter->irq, adapter);
#endif

	spi_reset(adapter);

	spi_unregister_master(master);

	return 0;
}

static struct platform_driver comcerto_spi_driver = {
	.driver	= {
		.name	= "comcerto_spi",
		.owner	= THIS_MODULE,
	},
	.probe	= comcerto_spi_probe,
	.remove	= __devexit_p(comcerto_spi_remove),
};

static int __init comcerto_spi_init(void)
{
	printk(KERN_INFO "comcerto-spi: %s: loaded version %s\n", __FUNCTION__, SPI_VERSION);

	if (platform_driver_register(&comcerto_spi_driver)) {
		printk(KERN_ERR "comcerto-spi: %s: error registering driver\n", __FUNCTION__);
		goto err0;
	}

	return 0;

err0:
	return -1;
}

static void __exit comcerto_spi_exit(void)
{
	platform_driver_unregister(&comcerto_spi_driver);
}

EXPORT_SYMBOL(comcerto_spi_set_loopback);

module_init(comcerto_spi_init);
module_exit(comcerto_spi_exit);
