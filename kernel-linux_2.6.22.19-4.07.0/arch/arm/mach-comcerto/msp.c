/*
 * arch/arm/mach-comcerto/msp.c
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
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/ptrace.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/delay.h>
#include <asm/arch/irq.h>
#include <asm/arch/msp.h>
#include <asm/arch/msp-ioctl.h>


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Copyright (C) Mindspeed Technologies, Inc.");


#define MSP_NAME		"msp-driver"

/* == 0 - no debugging printk at all
 * != 0 - KERN_DEBUG messages enabled
 */
#define MSP_DEBUG_LEVEL         0
  
#define PRINTKE(fmt, args...)	do { printk(KERN_ERR   MSP_NAME": " fmt, ## args); } while (0)
#define PRINTKI(fmt, args...)	do { printk(KERN_INFO  MSP_NAME": " fmt, ## args); } while (0)
#if (MSP_DEBUG_LEVEL > 0)
#define PRINTKD(fmt, args...)	do { printk(KERN_DEBUG MSP_NAME": " fmt, ## args); } while (0)
#else
#define PRINTKD(...)		do { } while (0)
#endif

static struct mutex		msp_reset_lock;

struct msp_private
{
	unsigned long	pos;
	struct mutex	lock;
};

static struct msp_region msp_regions[] =
{
	{
		.index		= 0,
		.name		= "SDRAM",
		.phys_addr	= SDRAM_MSP_MEMORY_PHY,
		.virt_addr	= SDRAM_MSP_MEMORY_VADDR,
		.size		= SDRAM_MSP_MEMORY_SIZE,
	},
#if defined(ARAM_MEMORY_SIZE) && (ARAM_MEMORY_SIZE > 0)
	{
		.index		= 0,
		.name		= "ARAM",
		.phys_addr	= ARAM_MEMORY_PHY,
		.virt_addr	= ARAM_MEMORY_VADDR,
		.size		= ARAM_MEMORY_SIZE,
	},
#endif
#if defined(IRAM_MEMORY_SIZE) && (IRAM_MEMORY_SIZE > 0)
	{
		.index		= 0,
		.name		= "IRAM",
		.phys_addr	= IRAM_MEMORY_PHY,
		.virt_addr	= IRAM_MEMORY_VADDR,
		.size		= IRAM_MEMORY_SIZE,
	},
#endif
#if defined(ERAM_MEMORY_SIZE) && (ERAM_MEMORY_SIZE > 0)
	{
		.index		= 0,
		.name		= "ERAM",
		.phys_addr	= ERAM_MEMORY_PHY,
		.virt_addr	= ERAM_MEMORY_VADDR,
		.size		= ERAM_MEMORY_SIZE,
	},
#endif
};

#define MSP_RESET_IRQ		IRQ_G7

static int msp_soft_reset(u32 timeout_ms)
{
	int err = 0;
	u32 count_ms = 0;
	ulong flags;

	local_irq_save(flags);

	comcerto_softirq_set(MSP_RESET_IRQ);

	/* timeout is passed in ms, but scale for udelay(0.1ms) busy waiting */
	timeout_ms *= 10;

	/* wait for the interrupt to be cleared */
	while (comcerto_softirq_check(MSP_RESET_IRQ)) {
		udelay(100);

		if (++count_ms >= timeout_ms) {
			err = -ETIME;
			break;
		}
	}

	local_irq_restore(flags);

	return err;
}

int msp_reset(u32 timeout_ms)
{
	int err = 0;

	err = msp_soft_reset(timeout_ms);

#ifdef CONFIG_ARCH_M823V2
	if (err)
		PRINTKE("soft reset timeout - couldn't reset MSP\n");
	
	__raw_writel(0x80000000, COMCERTO_INTC_ARM_CONTROL);	/* ARM0: reset, ARM: running */

	err = 0;
#endif

	return err;
}

static int msp_start(u32 timeout_ms)
{
	int err = 0;

#ifndef CONFIG_ARCH_M823V2
	err = msp_reset(timeout_ms);
#else
	/* firmware entry point is 0x0100, fixed at the moment */
	__raw_writel(0xE51FF004, SDRAM_MSP_MEMORY_VADDR);		/* ldr pc, [pc, #-4] */
	__raw_writel(0x00000100, SDRAM_MSP_MEMORY_VADDR+4);		/* jump location */

	__raw_writel(0xC0000000, COMCERTO_INTC_ARM_CONTROL);		/* ARM1: running, ARM: running */
#endif

	return err;
}

static struct msp_region *msp_region_find_by_phys(ulong phys_addr)
{
	int i;
	struct msp_region *region;

	for (i = 0, region = msp_regions; i < ARRAY_SIZE(msp_regions); i++, region++) {
		if (phys_addr >= region->phys_addr && phys_addr < (region->phys_addr + region->size))
			return region;
	}

	return NULL;
}

static int msp_region_check_range(struct msp_region *region, ulong phys_addr, uint size)
{
	ulong region_start, region_end, range_start, range_end;

	if (!region) {
		region = msp_region_find_by_phys(phys_addr);
		if (!region)
			goto err;
	}

	range_start = phys_addr;
	range_end = range_start + size;

	region_start = region->phys_addr;
	region_end = region_start + region->size;
	if (range_start >= region_start && range_end <= region_end)
		goto ok;

err:
	return -ENODEV;

ok:
	return 0;
}

static inline void *msp_region_phys_to_virt(struct msp_region *region, ulong phys_addr)
{
	return (void*)(region->virt_addr + (phys_addr - region->phys_addr));
}

static int msp_open(struct inode *inode, struct file *file)
{
	struct msp_private *msp;

	msp = kzalloc(sizeof(*msp), GFP_KERNEL);
	if (!msp) {
		PRINTKE("failed to allocate memory\n");
		goto err;
	}

	mutex_init(&msp->lock);

	file->private_data = msp;

	return 0;

err:
	return -ENOMEM;
}

static int msp_release(struct inode *inode, struct file *file)
{
	struct msp_private *msp = file->private_data;

	if (msp) {
		mutex_destroy(&msp->lock);
		kfree(msp);
	}

	return 0;
}

static int msp_ioctl_region(struct msp_region __user *uregion)
{
	int index, err = 0;

	if (get_user(index, &uregion->index)) {
		err = -EFAULT;
		goto err;
	}

	if (index >= ARRAY_SIZE(msp_regions)) {
		err = -ENODEV;
		goto err;
	}

	if (copy_to_user(uregion, &msp_regions[index], sizeof(*uregion))) {
		err = -EFAULT;
		goto err;
	}

err:
	return err;
}

static int msp_ioctl_reset(int __user *ureset)
{
	int kreset, err = 0;
	int msp_reset_state = 0;

	mutex_lock(&msp_reset_lock);

#ifndef CONFIG_ARCH_M823V2
	/* if MSP is running IRQMASK for STATUS0 will never be zero */
	msp_reset_state = __raw_readl(COMCERTO_INTC_ARM0_IRQMASK0) == 0;
#else
	/* for the M823V2 we know for sure... */
	msp_reset_state = ((__raw_readl(COMCERTO_INTC_ARM_CONTROL) >> 30) & 1) == 0;
#endif

	if (get_user(kreset, ureset)) {
		err = -EFAULT;
		goto err_unlock;
	}

	/* user may request the current state without actual changes */
	if (kreset != 0 && kreset != 1)
		goto no_action;

	PRINTKI("MSP: request to %s\n", kreset == 0 ? "start":"reset");
	if (kreset == msp_reset_state) {
		PRINTKI("nothing to do\n");
		goto no_action;
	}

	if (kreset == 0) {
		comcerto_eth_stop();

		if (!msp_start(10000)) {
			if (ved_wait_msp() == 0) {
				comcerto_irq_fixup();
				PRINTKI("start ok\n");

				ved_start();
				comcerto_eth_start_eth2();

				msp_reset_state = 0;
			} else {
				comcerto_eth_start();

				PRINTKE("timeout - no SVREADY from MSP\n");
				err = -EIO;
				goto err_unlock;
			}
		} else {
			comcerto_eth_start();

			PRINTKE("timeout - couldn't start MSP\n");
			err = -EIO;
			goto err_unlock;
		}
	} else {
		ved_stop();

		err = msp_reset(100);

		comcerto_irq_ack(IRQ_PTP0);

		if (!err) {
			PRINTKI("reset ok\n");
			msp_reset_state = 1;
		}

		comcerto_eth_start();

		if (err) {
			PRINTKE("timeout - couldn't reset MSP\n");
			err = -EIO;
			goto err_unlock;
		}
	}

no_action:
	if (put_user(msp_reset_state, ureset))
		err = -EFAULT;

err_unlock:
	mutex_unlock(&msp_reset_lock);

	return err;
}

static int msp_ioctl_memzero(struct msp_memzero_region __user *uregion)
{
	int err = 0;
	struct msp_memzero_region dst;
	struct msp_region *region;
	void *ptr;

	if (copy_from_user(&dst, uregion, sizeof(dst))) {
		err = -EFAULT;
		goto err;
	}

	region = msp_region_find_by_phys(dst.phys_addr);
	if (!region) {
		err = -ENODEV;
		goto err;
	}

	if (msp_region_check_range(region, dst.phys_addr, dst.size)) {
		err = -EFAULT;
		goto err;
	}

	ptr = msp_region_phys_to_virt(region, dst.phys_addr);
	memzero_ncnb(ptr, dst.size);

err:
	return err;
}

static int msp_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	int err = 0;

	switch (cmd) {
	case MSP_IOCTL_VERSION:
		if (put_user((unsigned int)MSP_VERSION(MSP_MAJOR, MSP_MINOR), ((unsigned int __user*)arg)))
			err = -EFAULT;
		break;

	case MSP_IOCTL_REGION:
		err = msp_ioctl_region((struct msp_region __user*)arg);
		break;

	case MSP_IOCTL_RESET:
		err = msp_ioctl_reset((int __user *)arg);
		break;

	case MSP_IOCTL_MEMZERO:
		err = msp_ioctl_memzero((struct msp_memzero_region __user *)arg);
		break;

	default:
		err = -EINVAL;
	}

	return err;
}

static loff_t msp_lseek(struct file *file, loff_t offset, int orig)
{
	loff_t ret;
	struct msp_private *msp = file->private_data;

	mutex_lock(&msp->lock);

	switch (orig) {
	case 0:	/* SEEK_SET */
		if (msp_region_check_range(NULL, offset, 1)) {
			ret = -ENODEV;
			goto err_unlock;
		}

		msp->pos = offset;
		ret = msp->pos;
		force_successful_syscall_return();
		break;

	case 1:	/* SEEK_CUR */
		if (msp_region_check_range(NULL, msp->pos + offset, 1)) {
			ret = -ENODEV;
			goto err_unlock;
		}

		msp->pos += offset;
		ret = msp->pos;
		force_successful_syscall_return();
		break;

	default:/* SEEK_END - isn't supported, use msp_region::size */
		ret = -ENODEV;
	}

err_unlock:
	mutex_unlock(&msp->lock);

	return ret;
}

static ssize_t msp_read_write(struct msp_private *msp, char __user *buf, size_t count, int rflag)
{
	ulong p = msp->pos;
	size_t sz;
	ssize_t len = 0;
	char *ptr;
	struct msp_region *region;

	mutex_lock(&msp->lock);

	region = msp_region_find_by_phys(msp->pos);

	if (!region) {
		len = -ENODEV;
		goto err_unlock;
	}

	if (msp_region_check_range(region, msp->pos, count)) {
		len = -ENOMEM;
		goto err_unlock;
	}

	while (count > 0) {
		ptr = msp_region_phys_to_virt(region, p);

		if (-p & (PAGE_SIZE - 1))
			sz = -p & (PAGE_SIZE - 1);
		else
			sz = PAGE_SIZE;

		sz = min(sz, count);

		if (rflag) {
			if (copy_to_user(buf, ptr, sz)) {
				len = -EFAULT;
				goto err_unlock;
			}
		} else {
			if (copy_from_user(ptr, buf, sz)) {
				len = -EFAULT;
				goto err_unlock;
			}
		}

		buf += sz;
		p += sz;
		count -= sz;
		len += sz;
	}

	msp->pos += len;

err_unlock:
	mutex_unlock(&msp->lock);

	return len;
}

static ssize_t msp_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	struct msp_private *msp = file->private_data;

	return msp_read_write(msp, buf, count, 1);
}

static ssize_t msp_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	struct msp_private *msp = file->private_data;

	return msp_read_write(msp, (char __user*)buf, count, 0);
}

#ifdef CONFIG_MSP_CARRIER
/*
 * TIMER0 is always used by firmware to generate a periodic clock. It will be initialized
 * for sure before CSP kernel is up - use it for udelay implementation.
 */
void msp_timer0_udelay(unsigned int us)
{
	u32 stamp0 = __raw_readl(COMCERTO_TIMER0_CURRENT_COUNT);
	u32 stamp1;
	u32 bound = __raw_readl(COMCERTO_TIMER0_HIGH_BOUND);
	u32 ticks_to_wait, ticks_gone;

	ticks_to_wait = us * comcerto_pll_bus_clock_mhz;
	ticks_gone = 0;

	while (ticks_to_wait > ticks_gone) {
		stamp1 = __raw_readl(COMCERTO_TIMER0_CURRENT_COUNT);

#ifdef	CONFIG_ARCH_M825XX1
		/* M825xx1 internal timer is decrementing */
		if (stamp1 < stamp0)
			ticks_gone += stamp0 - stamp1;
		else
			ticks_gone += stamp0 + (bound - stamp1);
#else
		/* M823xx and M825xx2 - incrementing */
		if (stamp1 > stamp0)
			ticks_gone += stamp1 - stamp0;
		else
			ticks_gone += stamp1 + (bound - stamp0);
#endif
		stamp0 = stamp1;
	}
}
#endif

static struct file_operations msp_fops =
{
	.owner		= THIS_MODULE,
	.open		= msp_open,
	.release	= msp_release,
	.llseek		= msp_lseek,
	.read		= msp_read,
	.write		= msp_write,
	.ioctl		= msp_ioctl,
};

static int __init msp_module_init(void)
{
	int rc;

	if (!comcerto_arm1_is_running()) {
		PRINTKI("disabled\n"); 
		return -ENODEV;
	}

	PRINTKI("version %u.%u\n", MSP_MAJOR, MSP_MINOR);

	mutex_init(&msp_reset_lock);

	rc = register_chrdev(MSP_DEVICE_MAJOR_NUM, MSP_NAME, &msp_fops);
	if (rc < 0) {
		PRINTKE("failed to register device\n");
		goto err;
	}

	return 0;

err:
	return rc;
}

static void __exit msp_module_exit(void)
{
	unregister_chrdev(MSP_DEVICE_MAJOR_NUM, MSP_NAME);

	mutex_destroy(&msp_reset_lock);
}

module_init(msp_module_init);
module_exit(msp_module_exit);
