/*
 * arch/arm/mach-comcerto/pci.c
 *
 * Copyright (C) 2004-2010 Mindspeed Technologies, Inc.
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
#include <linux/version.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/spinlock.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/delay.h>

#include <asm/irq.h>
#include <asm/delay.h>
#include <asm/sizes.h>
#include <asm/mach/pci.h>
#include <asm/arch/gpio.h>
#include <asm/mach/irq.h>


#define FLUSH_RX		0x80
#define MAX_RETRIES		20
#define MAX_BURST_LEN		16

#define BME_RXTRGABRT		0x0080000
#define BME_MSTRABRT		0x0100000
#define BME_SYSERROR		0x0200000
#define BME_PARERROR		0x0400000
#define BME_RETRY		0x0800000

#define BME_RXTRGABRTIAK	0x008
#define BME_MSTRABRTIAK		0x010
#define BME_SYSERRORIAK		0x020
#define BME_PARERRORIAK		0x040
#define BME_RETRYIAK		0x100

#define TXFF_RES		0x1000
#define RXFF_RES		0x2000

#define ENABLE_HI_IRQ		0

static spinlock_t comcerto_pci_lock;

static void __comcerto_pci_host_init(void);


/**
 * __comcerto_pci_reset -
 *
 */
static void __comcerto_pci_reset(void)
{
	printk(KERN_INFO "Comcerto PCI: reset\n");

	if (__raw_readl(COMCERTO_HMB_AX_FIFO_DEPTH))
		printk(KERN_INFO "Comcerto PCI: fifo not empty\n");

	__raw_writel(0x00, COMCERTO_PHI_PCI_IF_CONTROL);
	__raw_writel(0x3000, COMCERTO_HMB_FIFO_CONTROL);

	comcerto_gpio_ctrl(0x2 << 16, 0x3 << 16);

	mdelay(10);

	comcerto_gpio_ctrl(0x0 << 16, 0x3 << 16);

	__comcerto_pci_host_init();

	__raw_writel(0x0008, COMCERTO_HMB_FIFO_CONTROL);	
}

/**
 * comcerto_host_cfg_address -
 *
 */
static inline u32 comcerto_host_cfg_address(int where)
{
	return (1 << 16) | where;
}

/**
 * comcerto_cfg_address -
 *
 */
static u32 comcerto_cfg_address(struct pci_bus *bus, int devfn, int where)
{
	u32 addr;

	if (bus->number > 0)
		addr = (bus->number << 16) | (PCI_SLOT(devfn) << 11);
	else
		/* we skip/hide the host slot which is at 0x10000 */
		addr = 1 << (PCI_SLOT(devfn) + 16 + 1);

	addr |= (PCI_FUNC(devfn) << 8) | where;

	return addr;
}

/**
 * __comcerto_pci_check -
 *
 */
static int __comcerto_pci_check (int rx)
{
	u32 ctrl, status;
	u32 retry = 0;

	while ((ctrl = __raw_readl(COMCERTO_PHI_PCI_IF_CONTROL)) & 0x02) {
		status = __raw_readl(COMCERTO_PHI_PCI_IF_STATUS);

		if (status & BME_RXTRGABRT) {
			if (rx)
				printk(KERN_ERR "Comcerto PCI: Read Rx Target abort %#x %#x\n", status, ctrl);
			else
				printk(KERN_ERR "Comcerto PCI: Write Rx Target abort %#x %#x\n", status, ctrl);

			goto err;

		} else if (status & BME_MSTRABRT) {
			if (rx)
				printk(KERN_ERR "Comcerto PCI: Read Master abort %#x %#x\n", status, ctrl);
			else
				printk(KERN_ERR "Comcerto PCI: Write Master abort %#x %#x\n", status, ctrl);

			goto err;

		} else if (status & BME_SYSERROR) {
			if (rx)
				printk(KERN_ERR "Comcerto PCI: Read System error %#x %#x\n", status, ctrl);
			else
				printk(KERN_ERR "Comcerto PCI: Write System error %#x %#x\n", status, ctrl);

			goto err;

		} else if (status & BME_PARERROR) {
			if (rx)
				printk(KERN_ERR "Comcerto PCI: Read Parity error %#x %#x\n", status, ctrl);
			else
				printk(KERN_ERR "Comcerto PCI: Write Parity error %#x %#x\n", status, ctrl);

			goto err;

		} else if (status & BME_RETRY) {
			__raw_writel(BME_RETRYIAK, COMCERTO_PHI_PCI_IF_STATUS);

			if (retry++ > MAX_RETRIES) {
				if (rx)
					printk(KERN_ERR "Comcerto PCI: Excessive Read Retries %#x, %#x, %d\n", status, ctrl, retry);
				else
					printk(KERN_ERR "Comcerto PCI: Excessive Write Retries %#x, %#x, %d\n", status, ctrl, retry);

				goto err;
			}
		}
	}

	/* finish config cycle */
	__raw_writel(0x38, COMCERTO_PHI_PCI_IF_CONTROL);

	return 0;

  err:
	/* finish config cycle */
	__raw_writel(0x38, COMCERTO_PHI_PCI_IF_CONTROL);

	/* clear all error bits */
	__raw_writel(BME_RXTRGABRTIAK | BME_MSTRABRTIAK | BME_SYSERRORIAK | BME_PARERRORIAK | BME_RETRYIAK, COMCERTO_PHI_PCI_IF_STATUS);

	/* reset fifo */
	/* when reading from PCI target TX fifo is used */
	/* when writing to PCI target RX fifo is used */
	if (rx)
		__raw_writel (TXFF_RES, COMCERTO_PHI_APB_FIFO_CONTROL);
	else
		__raw_writel (RXFF_RES, COMCERTO_PHI_APB_FIFO_CONTROL);

	udelay(100);

	if (status & BME_RXTRGABRT)
		__comcerto_pci_reset();

	return -1;
}

/**
 * __comcerto_pci_read_config -
 *
 */
static int __comcerto_pci_read_config(u32 addr, int size, u8 space_type, u32 *value)
{
	int rc = 0;

	__raw_writel(addr, COMCERTO_PHI_BME_DMA_START_ADDR);		/* load destination of value */
	__raw_writel(size, COMCERTO_PHI_BME_DMA_XFER_LEN);			/* load length in bytes */
	__raw_writel(1, COMCERTO_PHI_BME_DMA_BURST_SIZE);			/* burst length in PCI data cycles, i.e, 32 bits */

	/* initiate PCI dma */
	if (space_type == PCI_SPACE_TYPE_CONFIG_TYPE0)
		__raw_writel(0x4001F, COMCERTO_PHI_PCI_IF_CONTROL);
	else
		__raw_writel(0xC001F, COMCERTO_PHI_PCI_IF_CONTROL);

	rc = __comcerto_pci_check(1);
	if (!rc) {
		switch (size) {
		case 1:
			*value = __raw_readb(COMCERTO_PHI_APB_FIFO_DATA8);
			break;

		case 2:
			*value = __raw_readw(COMCERTO_PHI_APB_FIFO_DATA16);
			break;

		case 4:
			*value = __raw_readl(COMCERTO_PHI_APB_FIFO_DATA32);
			break;
		}
	 }

	return rc;
}

/**
 * comcerto_pci_read_config -
 *
 */
static int comcerto_pci_read_config(u32 addr, int size, u8 space_type, u32 *value)
{
	unsigned long flags;
	int rc;

	spin_lock_irqsave(&comcerto_pci_lock, flags);

	rc = __comcerto_pci_read_config(addr, size, space_type, value);

	spin_unlock_irqrestore(&comcerto_pci_lock, flags);

	return rc;
}


/**
 * comcerto_pci_read -
 *
 */
void comcerto_pci_read(u32 addr, int size, u8 *buf, u8 space_type)
{
	unsigned long flags;
	u32 burst = (size + 3) / 4;
	int rc;

//	printk(KERN_INFO "read: addr: %#x, size: %#x, type: %#x\n", addr, size, space_type);

	if (burst > MAX_BURST_LEN)
		burst = MAX_BURST_LEN;

	spin_lock_irqsave(&comcerto_pci_lock, flags);

	__raw_writel(addr, COMCERTO_PHI_BME_DMA_START_ADDR);		/* load destination of value */
	__raw_writel(size, COMCERTO_PHI_BME_DMA_XFER_LEN);		/* load length in bytes */
	__raw_writel(burst, COMCERTO_PHI_BME_DMA_BURST_SIZE);		/* burst length in PCI data cycles, i.e, 32 bits */

	/* initiate PCI dma */
	if (space_type == PCI_SPACE_TYPE_IO)
		__raw_writel(0x0001E, COMCERTO_PHI_PCI_IF_CONTROL);
	else
		__raw_writel(0x0001F, COMCERTO_PHI_PCI_IF_CONTROL);

	rc = __comcerto_pci_check(1);
	if (!rc) {
		do {
			if (size >= 4) {
				*((u32 *)buf) = __raw_readl(COMCERTO_PHI_APB_FIFO_DATA32);
				buf += 4;
				size -= 4;
			} else if (size >= 2) {
				*((u16 *)buf) = __raw_readw(COMCERTO_PHI_APB_FIFO_DATA16);
				buf += 2;
				size -= 2;
			} else if (size >= 1) {
				*buf = __raw_readb(COMCERTO_PHI_APB_FIFO_DATA8);
				buf++;
				size--;
			}
		} while (size);
	} else
		printk(KERN_ERR "read: addr: %#x, size: %#x, type: %#x\n", addr, size, space_type);

	spin_unlock_irqrestore(&comcerto_pci_lock, flags);
}


/**
 * comcerto_pci_readl -
 *
 */
u32 comcerto_pci_readl(u32 addr, u8 space_type)
{
	u32 value = 0;
	unsigned long flags;
	int rc;

//	printk(KERN_INFO "readl: addr: %#x, value: %#x, type: %#x\n", addr, value, space_type);

	spin_lock_irqsave(&comcerto_pci_lock, flags);

	__raw_writel(addr, COMCERTO_PHI_BME_DMA_START_ADDR);		/* load destination of value */
	__raw_writel(4, COMCERTO_PHI_BME_DMA_XFER_LEN);			/* load length in bytes */
	__raw_writel(1, COMCERTO_PHI_BME_DMA_BURST_SIZE);			/* burst length in PCI data cycles, i.e, 32 bits */

	/* initiate PCI dma */
	if (space_type == PCI_SPACE_TYPE_IO)
		__raw_writel(0x0001E, COMCERTO_PHI_PCI_IF_CONTROL);
	else
		__raw_writel(0x0001F, COMCERTO_PHI_PCI_IF_CONTROL);

	rc = __comcerto_pci_check(1);
	if (!rc)
		value = __raw_readl(COMCERTO_PHI_APB_FIFO_DATA32);
	else
		printk(KERN_ERR "readl: addr: %#x, value: %#x, type: %#x\n", addr, value, space_type);

	spin_unlock_irqrestore(&comcerto_pci_lock, flags);

	return value;
}

/**
 * comcerto_pci_readw -
 *
 */
u16 comcerto_pci_readw(u32 addr, u8 space_type)
{
	u16 value = 0;
	unsigned long flags;
	int rc;

//	printk(KERN_INFO "readw: addr: %#x, value: %#x, type: %#x\n", addr, value, space_type);

	spin_lock_irqsave(&comcerto_pci_lock, flags);

	__raw_writel(addr, COMCERTO_PHI_BME_DMA_START_ADDR);		/* load destination of value */
	__raw_writel(2, COMCERTO_PHI_BME_DMA_XFER_LEN);			/* load length in bytes */
	__raw_writel(1, COMCERTO_PHI_BME_DMA_BURST_SIZE);		/* burst length in PCI data cycles, i.e, 32 bits */

	/* initiate PCI dma */
	if (space_type == PCI_SPACE_TYPE_IO)
		__raw_writel(0x0001E, COMCERTO_PHI_PCI_IF_CONTROL);
	else
		__raw_writel(0x0001F, COMCERTO_PHI_PCI_IF_CONTROL);

	rc = __comcerto_pci_check(1);
	if (!rc)
		value = __raw_readw(COMCERTO_PHI_APB_FIFO_DATA16);
	else
		printk(KERN_ERR "readw: addr: %#x, value: %#x, type: %#x\n", addr, value, space_type);

	spin_unlock_irqrestore(&comcerto_pci_lock, flags);

	return value;
}

/**
 * comcerto_pci_readb -
 *
 */
u8 comcerto_pci_readb(u32 addr, u8 space_type)
{
	u8 value = 0;
	unsigned long flags;
	int rc;

//	printk(KERN_INFO "readb: addr: %#x, value: %#x, type: %#x\n", addr, value, space_type);

	spin_lock_irqsave(&comcerto_pci_lock, flags);

	__raw_writel(addr, COMCERTO_PHI_BME_DMA_START_ADDR);		/* load destination of value */
	__raw_writel(1, COMCERTO_PHI_BME_DMA_XFER_LEN);			/* load length in bytes */
	__raw_writel(1, COMCERTO_PHI_BME_DMA_BURST_SIZE);			/* burst length in PCI data cycles, i.e, 32 bits */

	/* initiate PCI dma */
	if (space_type == PCI_SPACE_TYPE_IO)
		__raw_writel(0x0001E, COMCERTO_PHI_PCI_IF_CONTROL);
	else
		__raw_writel(0x0001F, COMCERTO_PHI_PCI_IF_CONTROL);

	rc = __comcerto_pci_check(1);
	if (!rc)
		value = __raw_readb(COMCERTO_PHI_APB_FIFO_DATA8);
	else
		printk(KERN_ERR "readb: addr: %#x, value: %#x, type: %#x\n", addr, value, space_type);

	spin_unlock_irqrestore(&comcerto_pci_lock, flags);

	return value;
}

/**
 * __comcerto_pci_write_config -
 *
 */
static int __comcerto_pci_write_config(u32 addr, int size, u32 value, u8 space_type)
{
	int rc = 0;

	switch (size) {
	case 1:
		__raw_writeb(value, COMCERTO_PHI_APB_FIFO_DATA8);
		break;

	case 2:
		__raw_writew(value, COMCERTO_PHI_APB_FIFO_DATA16);
		break;

	case 4:
		__raw_writel(value, COMCERTO_PHI_APB_FIFO_DATA32);
		break;
	}

	__raw_writel(FLUSH_RX, COMCERTO_PHI_APB_FIFO_INTACK);		/* flush out the value */
	__raw_writel(addr, COMCERTO_PHI_BME_DMA_START_ADDR);		/* load destination of value */
	__raw_writel(size, COMCERTO_PHI_BME_DMA_XFER_LEN);		/* load length in bytes */
	__raw_writel(1, COMCERTO_PHI_BME_DMA_BURST_SIZE);		/* burst length in PCI data cycles, i.e, 32 bits */

	/* initiate PCI dma */
	if (space_type == PCI_SPACE_TYPE_CONFIG_TYPE0)
		__raw_writel(0x4001B, COMCERTO_PHI_PCI_IF_CONTROL);	
	else
		__raw_writel(0xC001B, COMCERTO_PHI_PCI_IF_CONTROL);

	rc = __comcerto_pci_check(0);

	return rc;
}

/**
 * comcerto_pci_write_config -
 *
 */
static int comcerto_pci_write_config(u32 addr, int size, u32 value, u8 space_type)
{
	unsigned long flags;
	int rc;

	spin_lock_irqsave(&comcerto_pci_lock, flags);

	rc = __comcerto_pci_write_config(addr, size, value, space_type);

	spin_unlock_irqrestore(&comcerto_pci_lock, flags);

	return rc;
}


/**
 * comcerto_pci_write -
 *
 */
void comcerto_pci_write(u32 addr, int size, u8 *buf, u8 space_type)
{
	int size_tmp = size;
	u32 burst = (size + 3) / 4;
	unsigned long flags;

//	printk(KERN_INFO "write: addr: %#x, size: %#x, type: %#x\n", addr, size, space_type);

	if (burst > MAX_BURST_LEN)
		burst = MAX_BURST_LEN;

	spin_lock_irqsave(&comcerto_pci_lock, flags);

	do {
		if (size >= 4) {
			__raw_writel(*((u32 *)buf), COMCERTO_PHI_APB_FIFO_DATA32);
			buf += 4;
			size -= 4;
		} else if (size >= 2) {
			__raw_writew(*((u16 *)buf), COMCERTO_PHI_APB_FIFO_DATA16);
			buf += 2;
			size -= 2;
		} else if (size >= 1) {
			__raw_writeb(*buf, COMCERTO_PHI_APB_FIFO_DATA8);
			buf++;
			size--;
		}
	} while (size);

	__raw_writel(FLUSH_RX, COMCERTO_PHI_APB_FIFO_INTACK);		/* flush out the value */
	__raw_writel(addr, COMCERTO_PHI_BME_DMA_START_ADDR);		/* load destination of value */
	__raw_writel(size_tmp, COMCERTO_PHI_BME_DMA_XFER_LEN);		/* load length in bytes */
	__raw_writel(burst, COMCERTO_PHI_BME_DMA_BURST_SIZE);		/* burst length in PCI data cycles, i.e, 32 bits */

	/* initiate PCI dma */
	if (space_type == PCI_SPACE_TYPE_IO)
		__raw_writel(0x0001A, COMCERTO_PHI_PCI_IF_CONTROL);
	else
		__raw_writel(0x0001B, COMCERTO_PHI_PCI_IF_CONTROL);

	if (__comcerto_pci_check(0))
		printk(KERN_ERR "write: addr: %#x, size: %#x, type: %#x\n", addr, size_tmp, space_type);

	spin_unlock_irqrestore(&comcerto_pci_lock, flags);
}


/**
 * comcerto_pci_writeb -
 *
 */
void comcerto_pci_writeb(u32 addr, u8 value, u8 space_type)
{
	unsigned long flags;

//	printk(KERN_INFO "writeb: addr: %#x, value: %#x, type: %#x\n", addr, value, space_type);

	spin_lock_irqsave(&comcerto_pci_lock, flags);

	__raw_writeb(value, COMCERTO_PHI_APB_FIFO_DATA8);

	__raw_writel(FLUSH_RX, COMCERTO_PHI_APB_FIFO_INTACK);		/* flush out the value */
	__raw_writel(addr, COMCERTO_PHI_BME_DMA_START_ADDR);		/* load destination of value */
	__raw_writel(1, COMCERTO_PHI_BME_DMA_XFER_LEN);			/* load length in bytes */
	__raw_writel(1, COMCERTO_PHI_BME_DMA_BURST_SIZE);		/* burst length in PCI data cycles, i.e, 32 bits */

	/* initiate PCI dma */
	if (space_type == PCI_SPACE_TYPE_IO)
		__raw_writel(0x0001A, COMCERTO_PHI_PCI_IF_CONTROL);
	else
		__raw_writel(0x0001B, COMCERTO_PHI_PCI_IF_CONTROL);

	if (__comcerto_pci_check(0))
		printk(KERN_ERR "writeb: addr: %#x, value: %#x, type: %#x\n", addr, value, space_type);

	spin_unlock_irqrestore(&comcerto_pci_lock, flags);
}

/**
 * comcerto_pci_writew -
 *
 */
void comcerto_pci_writew(u32 addr, u16 value, u8 space_type)
{
	unsigned long flags;

//	printk(KERN_INFO "writew: addr: %#x, value: %#x, type: %#x\n", addr, value, space_type);

	spin_lock_irqsave(&comcerto_pci_lock, flags);

	__raw_writew(value, COMCERTO_PHI_APB_FIFO_DATA16);

	__raw_writel(FLUSH_RX, COMCERTO_PHI_APB_FIFO_INTACK);		/* flush out the value */
	__raw_writel(addr, COMCERTO_PHI_BME_DMA_START_ADDR);		/* load destination of value */
	__raw_writel(2, COMCERTO_PHI_BME_DMA_XFER_LEN);			/* load length in bytes */
	__raw_writel(1, COMCERTO_PHI_BME_DMA_BURST_SIZE);		/* burst length in PCI data cycles, i.e, 32 bits */

	/* initiate PCI dma */
	if (space_type == PCI_SPACE_TYPE_IO)
		__raw_writel(0x0001A, COMCERTO_PHI_PCI_IF_CONTROL);
	else
		__raw_writel(0x0001B, COMCERTO_PHI_PCI_IF_CONTROL);

	if (__comcerto_pci_check(0))
		printk(KERN_ERR "writew: addr: %#x, value: %#x, type: %#x\n", addr, value, space_type);

	spin_unlock_irqrestore(&comcerto_pci_lock, flags);
}

/**
 * comcerto_pci_writel -
 *
 */
void comcerto_pci_writel(u32 addr, u32 value, u8 space_type)
{
	unsigned long flags;

//	printk(KERN_INFO "writel: addr: %#x, value: %#x, type: %#x\n", addr, value, space_type);

	spin_lock_irqsave(&comcerto_pci_lock, flags);

	__raw_writel(value, COMCERTO_PHI_APB_FIFO_DATA32);

	__raw_writel(FLUSH_RX, COMCERTO_PHI_APB_FIFO_INTACK);		/* flush out the value */
	__raw_writel(addr, COMCERTO_PHI_BME_DMA_START_ADDR);		/* load destination of value */
	__raw_writel(4, COMCERTO_PHI_BME_DMA_XFER_LEN);			/* load length in bytes */
	__raw_writel(1, COMCERTO_PHI_BME_DMA_BURST_SIZE);		/* burst length in PCI data cycles, i.e, 32 bits */

	/* initiate PCI dma */
	if (space_type == PCI_SPACE_TYPE_IO)
		__raw_writel(0x0001A, COMCERTO_PHI_PCI_IF_CONTROL);
	else
		__raw_writel(0x0001B, COMCERTO_PHI_PCI_IF_CONTROL);

	if (__comcerto_pci_check(0))
		printk(KERN_ERR "writel: addr: %#x, value: %#x, type: %#x\n", addr, value, space_type);

	spin_unlock_irqrestore(&comcerto_pci_lock, flags);
}

/**
 * comcerto_read_config -
 *
 */
static int comcerto_read_config(struct pci_bus *bus, unsigned int devfn, int where, int size, u32 *value)
{
	u32 addr = comcerto_cfg_address (bus, devfn, where);
	int rc;

	BUG_ON (((where & 0x3) + size) > 4);

	if (bus->number > 0)
		rc = comcerto_pci_read_config(addr, size, PCI_SPACE_TYPE_CONFIG_TYPE1, value);
	else
		rc = comcerto_pci_read_config(addr, size, PCI_SPACE_TYPE_CONFIG_TYPE0, value);

//	printk(KERN_INFO "read bus: %d, devfn: %#x, where: %#x, size: %#x, addr: %#x, value: %#x\n", bus->number, devfn, where, size, addr, *value);

	if (rc)
		return PCIBIOS_DEVICE_NOT_FOUND;

	return PCIBIOS_SUCCESSFUL;
}

/**
 * __comcerto_host_read_config -
 *
 */
static int __comcerto_host_read_config(int where, int size, u32 *value)
{
	u32 addr = comcerto_host_cfg_address (where);
	int rc;

	rc = __comcerto_pci_read_config(addr, size, PCI_SPACE_TYPE_CONFIG_TYPE0, value);

//	printk(KERN_INFO "read where: %#x, size: %#x, addr: %#x, value: %#x\n", where, size, addr, *value);

	if (rc)
		return PCIBIOS_DEVICE_NOT_FOUND;

	return PCIBIOS_SUCCESSFUL;
}


/**
 * comcerto_write_config -
 *
 */
static int comcerto_write_config(struct pci_bus *bus, unsigned int devfn, int where, int size, u32 value)
{
	u32 addr = comcerto_cfg_address (bus, devfn, where);
	int rc;

//	printk(KERN_INFO "write bus: %d, devfn: %#x, where: %#x, size: %#x, addr: %#x, value: %#x\n", bus->number, devfn, where, size, addr, value);

	BUG_ON (((where & 0x3) + size) > 4);

	if (bus->number > 0)
		rc = comcerto_pci_write_config(addr, size, value, PCI_SPACE_TYPE_CONFIG_TYPE1);
	else
		rc = comcerto_pci_write_config(addr, size, value, PCI_SPACE_TYPE_CONFIG_TYPE0);

	if (rc)
		return PCIBIOS_DEVICE_NOT_FOUND;

	return PCIBIOS_SUCCESSFUL;
}

/**
 * __comcerto_host_write_config -
 *
 */
static int __comcerto_host_write_config(int where, int size, u32 value)
{
	u32 addr = comcerto_host_cfg_address (where);
	int rc;

//	printk(KERN_INFO "write where: %#x, size: %#x, addr: %#x, value: %#x\n", where, size, addr, value);

	rc = __comcerto_pci_write_config(addr, size, value, PCI_SPACE_TYPE_CONFIG_TYPE0);

	if (rc)
		return PCIBIOS_DEVICE_NOT_FOUND;

	return PCIBIOS_SUCCESSFUL;
}


static struct pci_ops comcerto_ops = {
	.read = comcerto_read_config,
	.write = comcerto_write_config,
};


static void __comcerto_pci_host_init(void)
{
	u32 val;

	if (__comcerto_host_write_config(PCI_COMMAND, 2, PCI_COMMAND_MASTER | PCI_COMMAND_MEMORY))
		printk(KERN_ERR "Comcerto PCI: host PCI_COMMAND write failed\n");
	else if (__comcerto_host_read_config(PCI_COMMAND, 2, &val)) {
		printk(KERN_ERR "Comcerto PCI: host PCI_COMMAND read failed\n");
	} else if (val != (PCI_COMMAND_MASTER | PCI_COMMAND_MEMORY))
		printk(KERN_ERR "Comcerto PCI: host PCI_COMMAND write failed %#x\n", val);

	if (__comcerto_host_write_config(PCI_BASE_ADDRESS_1, 4, COMCERTO_PCIDMA_PCI_BASE_ADDR))
		printk(KERN_ERR "Comcerto PCI: host PCI_BASE_ADDRESS_1 write failed\n");
	else if (__comcerto_host_read_config(PCI_BASE_ADDRESS_1, 4, &val)) {
		printk(KERN_ERR "Comcerto PCI: host PCI_BASE_ADDRESS_1 read failed\n");
	} else if (val != COMCERTO_PCIDMA_PCI_BASE_ADDR)
		printk(KERN_ERR "Comcerto PCI: host PCI_BASE_ADDRESS_1 write failed %#x\n", val);

	if (__comcerto_host_write_config(PCI_BASE_ADDRESS_2, 4, 0))
		printk(KERN_ERR "Comcerto PCI: host PCI_BASE_ADDRESS_2 write failed\n");
	else if (__comcerto_host_read_config(PCI_BASE_ADDRESS_2, 4, &val)) {
		printk(KERN_ERR "Comcerto PCI: host PCI_BASE_ADDRESS_2 read_failed\n");
	} else if (val != 0)
		printk(KERN_ERR "Comcerto PCI: host PCI_BASE_ADDRESS_2 write failed %#x\n", val);
}

void __init comcerto_preinit(void)
{
	u32 ctl, val;

	/* config Host Memory Bridge (hmb) Parameters */
	__raw_writel(COMCERTO_PCIDMA_SYS_BASE_ADDR, COMCERTO_HMB_SYS_BASE_ADDR1);
	__raw_writel(COMCERTO_PCIDMA_SIZE_MASK, COMCERTO_HMB_SIZE_MASK1);

	__raw_writel(0, COMCERTO_HMB_SYS_BASE_ADDR2);
	__raw_writel(~(SZ_4K - 1), COMCERTO_HMB_SIZE_MASK2);

	/* all 5 ports are allowed to request for PCI bus arbitration pci device to be bus master */
	ctl = __raw_readl(COMCERTO_HMB_PCI_ARBITER_CONTROL);
	__raw_writel(0xff3e, COMCERTO_HMB_PCI_ARBITER_CONTROL);	/* priority scheme = round robin, 5 ports enabled, aging period = 255 */
	ctl = __raw_readl(COMCERTO_HMB_PCI_ARBITER_CONTROL);
	printk(KERN_INFO "Comcerto PCI: HMB pci arbiter control: %#x\n", ctl);

	__raw_writel(0x5a, COMCERTO_HMB_MODE_CONTROL);	/* read prefetch size = 16 words, programio = HMDMA,
									   timeout = no timeout, timeout cycles = 5 cycles */
	ctl = __raw_readl(COMCERTO_HMB_MODE_CONTROL);
	printk(KERN_INFO "Comcerto PCI: HMB mode control: %#x\n", ctl);

	ctl = __raw_readl(COMCERTO_HMB_INT_CONTROL);
#if ENABLE_HI_IRQ
	__raw_writel(ctl | 3, COMCERTO_HMB_INT_CONTROL);	/* enable all interrupts */
#else
	__raw_writel(ctl & 3, COMCERTO_HMB_INT_CONTROL);	/* disable all interrupts */
#endif
	ctl = __raw_readl(COMCERTO_HMB_INT_CONTROL);
	printk(KERN_INFO "Comcerto PCI: HMB interrupt control: %#x\n", ctl);

	/* Set PCI Bus enable (No need, handled by HBMODE_n & HBBURSTEN_n pins)
	   __raw_writel(__raw_readl(CHAGALL_GPIO_OUTPUT_REG+0x44)|0x00030000, CHAGALL_GPIO_OUTPUT_REG+0x44);
	 */

	/* clear all error bits */
	__raw_writel(BME_RXTRGABRTIAK | BME_MSTRABRTIAK | BME_SYSERRORIAK | BME_PARERRORIAK | BME_RETRYIAK, COMCERTO_PHI_PCI_IF_STATUS);

	/* Configure Host bus device */
	if (__comcerto_host_read_config(PCI_VENDOR_ID, 4, &val)) {
		printk(KERN_ERR "Comcerto PCI: PCI Host device not detected\n");
	} else if (val != 0x013018dc) {
		printk(KERN_ERR "Comcerto PCI: PCI Host device not detected\n");
		return;
	}

	__comcerto_pci_host_init();

	/* enable hmb operation */
	ctl = __raw_readl(COMCERTO_HMB_STATE_CONTROL);
	printk(KERN_INFO "Comcerto PCI: HMB state control: %#x\n", ctl);
	__raw_writel(ctl | 1, COMCERTO_HMB_STATE_CONTROL);	/* HMB Mode = enable */
}

#if ENABLE_HI_IRQ
static irqreturn_t comcerto_pci_irq(int irq, void *dev_id)
{
	u32 status;

	status = __raw_readl(COMCERTO_HMB_STATUS);
	if (!(status & 3))
		goto out;

	if (status & 1)
		printk(KERN_ERR "Comcerto PCI: HMB Retry/Disconnect error\n");

	if (status & 2)
		printk(KERN_ERR "Comcerto PCI: HMB PCI System error\n");

	printk(KERN_INFO "Comcerto PCI: HMB error count %#x\n", status >> 16);	

	__raw_writel(3, COMCERTO_HMB_INTACK);

out:
	return IRQ_HANDLED;
}
#endif /* ENABLE_HI_IRQ */

static void __init comcerto_postinit(void)
{
#if ENABLE_HI_IRQ
	if (request_irq(IRQ_HI, comcerto_pci_irq, IRQF_DISABLED, "Comcerto PCI", NULL))
		printk(KERN_ERR "Comcerto PCI: can't request IRQ_HI (%d)\n", IRQ_HI);
#endif
}

static int __init comcerto_pci_resources(struct resource **resource)
{
	struct resource *res;

	res = kmalloc(sizeof(struct resource) * 2, GFP_KERNEL);
	if (!res)
		panic("Comcerto PCI: unable to alloc resources");

	memset(res, 0, sizeof(struct resource) * 2);

	/* These are physical address space ranges, though they aren't mapped to
	 * anything at the hardware level, all PCI IO/MEM accesses are done indirectly
	 */
	res[0].start = COMCERTO_PCI_IO_BASE;
	res[0].end   = COMCERTO_PCI_IO_BASE + COMCERTO_PCI_IO_SIZE;
	res[0].name  = "Comcerto PCI I/O Space";
	res[0].flags = IORESOURCE_IO;

	res[1].start = COMCERTO_PCI_MEM_BASE;
	res[1].end   = COMCERTO_PCI_MEM_BASE + COMCERTO_PCI_MEM_SIZE;
	res[1].name  = "Comcerto PCI Memory Space";
	res[1].flags = IORESOURCE_MEM;

	request_resource(&ioport_resource, &res[0]);
	request_resource(&iomem_resource, &res[1]);

	resource[0] = &res[0];
	resource[1] = &res[1];
	resource[2] = NULL;

	return 1;
}

static int __init comcerto_setup(int nr, struct pci_sys_data *sys)
{
	if (nr != 0)
		return 0;

	return comcerto_pci_resources(sys->resource);
}

static struct pci_bus *__init comcerto_scan_bus(int nr, struct pci_sys_data *sys)
{
	if (nr != 0)
		return NULL;

	return pci_scan_bus(sys->busnr, &comcerto_ops, sys);
}

static int __init comcerto_map_irq(struct pci_dev *dev, u8 slot, u8 pin)
{
	int irq = -1;

	if (!dev)
		return -1;

	if (dev->bus->number > 0) {
		irq = dev->bus->self->irq;
	} else {
#ifdef COMCERTO_GPIO_IRQ_PCI
		irq = comcerto_gpio_to_irq(COMCERTO_GPIO_IRQ_PCI);
#endif
	}

	return irq;
}

static u8 __init comcerto_swizzle(struct pci_dev *dev, u8 * pin)
{
	return PCI_SLOT(dev->devfn);
}


static struct hw_pci comcerto_pci __initdata = {
	.swizzle = comcerto_swizzle,
	.map_irq = comcerto_map_irq,
	.setup = comcerto_setup,
	.nr_controllers = 1,
	.scan = comcerto_scan_bus,
	.preinit = comcerto_preinit,
	.postinit = comcerto_postinit,
};


int __init comcerto_pci_init(void)
{
	u32 syscfg;

	syscfg = __raw_readl(COMCERTO_GPIO_SYSTEM_CONFIG);
	if ((syscfg & 0x2004) != 0x0004) {
		printk(KERN_INFO "PCI: Comcerto device is not configured as PCI host\n");
		goto err;
	}

	spin_lock_init(&comcerto_pci_lock);

	pci_common_init(&comcerto_pci);

	return 0;

err:
	return -1;
}

subsys_initcall(comcerto_pci_init);

EXPORT_SYMBOL(comcerto_pci_read);
EXPORT_SYMBOL(comcerto_pci_readb);
EXPORT_SYMBOL(comcerto_pci_readw);
EXPORT_SYMBOL(comcerto_pci_readl);

EXPORT_SYMBOL(comcerto_pci_write);
EXPORT_SYMBOL(comcerto_pci_writeb);
EXPORT_SYMBOL(comcerto_pci_writew);
EXPORT_SYMBOL(comcerto_pci_writel);
