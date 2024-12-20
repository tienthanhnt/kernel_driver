/*
 * drivers/char/watchdog/comcerto_wdt.c
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

#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/watchdog.h>
#include <asm/bitops.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/arch/hardware.h>
#include <asm/arch/wdt.h>

#define WDT_NAME					"comcerto_wdt"
#define WDT_DEFAULT_TIMEOUT				5
#define WDT_MAX_TIMEOUT					(0xFFFFFFFF / comcerto_pll_bus_clock_hz)

#define COMCERTO_TIMER_WDT_CONTROL_RESET_ENABLE		(1 << 0)
#define COMCERTO_TIMER_WDT_CONTROL_TIMER_DISABLE	(1 << 1)
#define COMCERTO_GPIO_WDT_GPIO_LINE_SELECT_ACTIVE_HIGH	(1 << 6)
#define COMCERTO_GPIO_WDT_GPIO_LINE_SELECT_ENABLE	(1 << 7)
#define COMCERTO_TIMER_WDT_CONTROL_WIDTH_SHIFT		16

static int wd_heartbeat = WDT_DEFAULT_TIMEOUT;
static int nowayout = WATCHDOG_NOWAYOUT;

module_param(wd_heartbeat, int, 0);
MODULE_PARM_DESC(wd_heartbeat, "Watchdog heartbeat in seconds. (default="__MODULE_STRING(WDT_DEFAULT_TIMEOUT) ")");

#ifdef CONFIG_WATCHDOG_NOWAYOUT
module_param(nowayout, int, 0);
MODULE_PARM_DESC(nowayout, "Watchdog cannot be stopped once started (default=" __MODULE_STRING(WATCHDOG_NOWAYOUT) ")");
#endif

static unsigned long comcerto_wdt_busy;
static char expect_close;
static spinlock_t wdt_lock;

/*
 * Set a new heartbeat value for the watchdog device. If the heartbeat value is
 * incorrect we keep the old value and return -EINVAL. If successfull we return 0.
 */
static int comcerto_wdt_set_heartbeat(int t)
{
	if (t < 1 || t > WDT_MAX_TIMEOUT)
		return -EINVAL;

	wd_heartbeat = t;
	return 0;
}

/*
 * Write wd_heartbeat to high bound register.
 */
static void comcerto_wdt_set_timeout(void)
{
	__raw_writel(wd_heartbeat * comcerto_pll_bus_clock_hz, COMCERTO_TIMER_WDT_HIGH_BOUND);
}

/*
 * Disable the watchdog.
 */
static void comcerto_wdt_stop(void)
{
	unsigned long flags;
	u32 wdt_control;

	spin_lock_irqsave(&wdt_lock, flags);	

	wdt_control = __raw_readl(COMCERTO_TIMER_WDT_CONTROL);

	__raw_writel(wdt_control | COMCERTO_TIMER_WDT_CONTROL_TIMER_DISABLE, COMCERTO_TIMER_WDT_CONTROL);
	
	spin_unlock_irqrestore(&wdt_lock, flags);
	comcerto_wdt_set_timeout();
}

/*
 * Enable the watchdog.
 */
static void comcerto_wdt_start(void)
{
	unsigned long flags;
	u32 wdt_control;
			
	spin_lock_irqsave(&wdt_lock, flags);
						
	wdt_control = __raw_readl(COMCERTO_TIMER_WDT_CONTROL);

	__raw_writel(wdt_control & ~COMCERTO_TIMER_WDT_CONTROL_TIMER_DISABLE, COMCERTO_TIMER_WDT_CONTROL);

	spin_unlock_irqrestore(&wdt_lock, flags);
}

/*
 * Disable WDT and:
 * - set max. possible timeout to avoid reset, it can occur 
 * since current counter value could be bigger then 
 * high bound one at the moment
 * - set system reset parameter
 * - configure GPIO parameters:
 *   - line number (pin)
 *   - output polarity
 *   - pulse width
 * Function is called once at start (while configuration), 
 * and it's safe not to disable/enable IRQs.
 */
static void comcerto_wdt_config(struct comcerto_wdt_data *config)
{
	u32 wdt_control = COMCERTO_TIMER_WDT_CONTROL_TIMER_DISABLE;
	u32 gpio_control = 0;
	u32 gpio_pulse_width_clk;
	
	comcerto_wdt_stop();
	
	__raw_writel(~0, COMCERTO_TIMER_WDT_HIGH_BOUND);			/* write max timout */

	if (config->gpio_polarity_active_high)
		gpio_control |= COMCERTO_GPIO_WDT_GPIO_LINE_SELECT_ACTIVE_HIGH;	

	if (config->gpio_line_enable)
		gpio_control |= COMCERTO_GPIO_WDT_GPIO_LINE_SELECT_ENABLE;

	gpio_control |= (config->gpio_line_number & 0x001F);
	
	__raw_writel(gpio_control, COMCERTO_GPIO_WDT_GPIO_LINE_SELECT);		/* GPIO config */

	if (config->system_reset)
		wdt_control |= COMCERTO_TIMER_WDT_CONTROL_RESET_ENABLE;

	gpio_pulse_width_clk = config->gpio_pulse_width_us * comcerto_pll_bus_clock_mhz;
	if (gpio_pulse_width_clk > 0xFFFF)
		gpio_pulse_width_clk = 0xFFFF;
	wdt_control |= gpio_pulse_width_clk << COMCERTO_TIMER_WDT_CONTROL_WIDTH_SHIFT;

	__raw_writel(wdt_control, COMCERTO_TIMER_WDT_CONTROL);			/* WDT config */
}

/*
 * Watchdog device is opened, and watchdog starts running.
 */
static int comcerto_wdt_open(struct inode *inode, struct file *file)
{
	if (test_and_set_bit(0, &comcerto_wdt_busy))
		return -EBUSY;

	comcerto_wdt_set_timeout();
	comcerto_wdt_start();

	return nonseekable_open(inode, file);
}

/*
 * Release the watchdog device.
 * If CONFIG_WATCHDOG_NOWAYOUT is NOT defined and expect_close == 42 
 * i.e. magic char 'V' has been passed while write() then the watchdog
 * is also disabled.
 */
static int comcerto_wdt_release(struct inode *inode, struct file *file)
{
	if (expect_close == 42) {
		comcerto_wdt_stop();	/* disable the watchdog when file is closed */
		clear_bit(0, &comcerto_wdt_busy);
	} else {
		printk(KERN_CRIT "%s: closed unexpectedly. WDT will not stop!\n", WDT_NAME);
	}

	expect_close = 0;
	return 0;
}

/*
 * Handle commands from user-space.
 */
static int comcerto_wdt_ioctl(struct inode *inode, struct file *file, uint cmd, ulong arg)
{
	void __user *argp = (void __user *)arg;
	int __user *p = argp;
	int new_value;
	int err;
	static struct watchdog_info comcerto_wdt_info = {
		.options = 	WDIOF_SETTIMEOUT | 
				WDIOF_MAGICCLOSE |
				WDIOF_KEEPALIVEPING,
		.firmware_version = 1,
	};

	switch(cmd) {
	case WDIOC_KEEPALIVE:
		comcerto_wdt_set_timeout();
		break;

	case WDIOC_GETSUPPORT:
		strncpy(comcerto_wdt_info.identity, WDT_NAME, sizeof(comcerto_wdt_info.identity));
		if (copy_to_user(argp, &comcerto_wdt_info, sizeof(comcerto_wdt_info)) != 0) {
                	err = -EFAULT;
			goto err;
		}
		break;
				  
	case WDIOC_SETTIMEOUT:
		if (get_user(new_value, p)) {
			err = -EFAULT;
			goto err;
		}

		if (comcerto_wdt_set_heartbeat(new_value)) {
			err = -EINVAL;
			goto err;	
		}

		comcerto_wdt_set_timeout();

		return put_user(wd_heartbeat, p);
		break;

	case WDIOC_GETTIMEOUT:
		return put_user(wd_heartbeat, p);
		break;

	case WDIOC_GETSTATUS:
	case WDIOC_GETBOOTSTATUS:
		return put_user(0, p);
		break;

	case WDIOC_SETOPTIONS:
		if (get_user(new_value, p)) {
			err = -EFAULT;
			goto err;
		}

		if (new_value & WDIOS_DISABLECARD)
			comcerto_wdt_stop();

		if (new_value & WDIOS_ENABLECARD)
			comcerto_wdt_start();

		break;

	default:
		err = -ENOIOCTLCMD;
		goto err;
		break;		
	}

	return 0;

err:
	return err;
}

/*
 * Pat the watchdog whenever device is written to.
 */
static ssize_t comcerto_wdt_write(struct file *file, const char *buf, size_t len, loff_t *ppos)
{
	if (len) {
		if (!nowayout) {
			size_t i;
			char c;

			/* in case it was set long ago */
			expect_close = 0;

			for (i = 0; i != len; i++) {
				if (get_user(c, buf + i))
					return -EFAULT;

				if (c == 'V')
					expect_close = 42;
			}
		}
		
		comcerto_wdt_set_timeout();
	}
	
	return len;
}

static const struct file_operations comcerto_wdt_fops = {
	.owner		= THIS_MODULE,
	.llseek		= no_llseek,
	.ioctl		= comcerto_wdt_ioctl,
	.open		= comcerto_wdt_open,
	.release	= comcerto_wdt_release,
	.write		= comcerto_wdt_write,
};

static struct miscdevice comcerto_wdt_miscdev = {
	.minor		= WATCHDOG_MINOR,
	.name		= WDT_NAME,
	.fops		= &comcerto_wdt_fops,
};

static int __init comcerto_wdt_probe(struct platform_device *pdev)
{
	int res;
	printk("WDT");
	printk("++++comcerto-wdt: %s: loaded version \n", __FUNCTION__);
	if (comcerto_wdt_miscdev.parent)
		return -EBUSY;

	comcerto_wdt_miscdev.parent = &pdev->dev;
	comcerto_wdt_config((struct comcerto_wdt_data*) pdev->dev.platform_data);

	res = misc_register(&comcerto_wdt_miscdev);
	if (res)
		return res;

	printk(KERN_INFO "%s: support registered\n", WDT_NAME);

        /* check that the heartbeat value is within range; if not reset to the default */
        if (comcerto_wdt_set_heartbeat(wd_heartbeat)) {
                comcerto_wdt_set_heartbeat(WDT_DEFAULT_TIMEOUT);
				
                printk(KERN_INFO "%s: wd_heartbeat value is out of range: 1..%u, using %d\n",
                        WDT_NAME, WDT_MAX_TIMEOUT, WDT_DEFAULT_TIMEOUT);
        }
										
	return 0;
}

static int __exit comcerto_wdt_remove(struct platform_device *pdev)
{
	int res;

	res = misc_deregister(&comcerto_wdt_miscdev);
	if (!res)
		comcerto_wdt_miscdev.parent = NULL;

	return res;
}

static struct platform_driver comcerto_wdt_driver = {
	.probe		= comcerto_wdt_probe,
	.remove		= __exit_p(comcerto_wdt_remove),
	.driver		= {
		.name	= WDT_NAME,
		.owner	= THIS_MODULE,
	},
};

static int __init comcerto_wdt_init(void)
{
        spin_lock_init(&wdt_lock);

	return platform_driver_register(&comcerto_wdt_driver);
}

static void __exit comcerto_wdt_exit(void)
{
	platform_driver_unregister(&comcerto_wdt_driver);
}

module_init(comcerto_wdt_init);
module_exit(comcerto_wdt_exit);

MODULE_AUTHOR("Mindspeed Technologies, Inc.");
MODULE_DESCRIPTION("Watchdog driver for Comcerto device");
MODULE_LICENSE("GPL");
MODULE_ALIAS_MISCDEV(WATCHDOG_MINOR);
