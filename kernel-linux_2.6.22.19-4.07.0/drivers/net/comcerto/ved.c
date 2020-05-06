/*
 * drivers/net/comcerto/ved.c
 * Virtual Ethernet Driver
 *
 * Copyright (C) 2009 Mindspeed Technologies
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */

#include <linux/kernel.h>
#include <linux/autoconf.h>
#include <linux/if_vlan.h>
#include <linux/etherdevice.h>
#include <linux/proc_fs.h>
#include <linux/syscalls.h>
#include <asm/io.h>
#include <asm/delay.h>
#include <asm/arch/irq.h>
#include <asm/arch/msp.h>

#include "eth.h"
#include "smi.h"

#define VED_VERSION		"0.1.0"

#define IFACE_ETH1		1
#define IFACE_ETH0		2
#define IFACE_ETH2		3

#define VED_TX_WAKE_TIMEOUT_MS	10			/* tx queue stop timeout on resources shortage */

static struct proc_dir_entry *ved_proc_dir;		/* "/proc/net/ved" dir for bridge and info files */


static struct ved {
	struct net_device_stats stats;
	struct smi_fpart	*tx_fp;
	struct timer_list	tx_wake_timer;
	u32			tx_interface;

	u32			tx_err_timer;
	u32			rx_err_skb_alloc;
} ved_devs[3];


static int bridge_setup(char *str);
static int ved_xmit(struct sk_buff *skb, struct net_device *dev);


static struct workqueue_struct	*ved_work_queue;
static struct ved_alert_work {
	struct work_struct	work;
	char	type[8];
	char	data[600];
	int	len;
} ved_alert_work;
static char	ved_alert_action_command[64]="/usr/local/sbin/msp-alert";

static u32	ved_interrupts;
static u32	ved_dequeued;
static u32	ved_err_alloc;
static u32	ved_err_dequeued;
static u32	ved_enqueued;
static u32	ved_err_enqueued;
static u32	ved_tx_bridged;
static u32	ved_rx_bridged;

static char	ved_irq_requested = 0;	/* whether we registered IRQ handler */
static char	ved_stopped = 0;	/* whether we stopped VED from outside */

static u8	bridge_mac[6];		/* bridging MAC, must be set either from /proc or by boot parameters */
static char	bridge_enabled = 0;	/* if non-zero - briding is turned on */


static inline int ved_is_owner(struct net_device *dev)
{
	return dev->hard_start_xmit == ved_xmit;
}

static inline int ved_ratelimit(void)
{
#ifndef DEBUG
	return net_ratelimit();
#endif

	return 1;	/* don't limit debugging output if DEBUG is defined */
}

static void ved_tx_add_timer(struct net_device *dev)
{
	struct ved *ved = dev->priv;

	if (ved_ratelimit())
		dev_warn(&dev->dev, "scheduling TX queue timer\n");

	ved->tx_wake_timer.expires = jiffies + (HZ*VED_TX_WAKE_TIMEOUT_MS)/1000;
	add_timer(&ved->tx_wake_timer);

	ved->tx_err_timer++;
}

static int ved_tx_check_resources(struct ved *ved)
{
	struct smi_fqueue *fq = smi_csp2msp_queue;
	u16 put;
	ulong flags;

	/* we are not going to change anything, so smi_fpart locking isn't needed */
	if (ved->tx_fp->freeblk == 0)
		goto err;

	/* here we need to lock since we operate with two variables,
	 * this is a check whether we have some space in the circular buffer
	 */
	smi_lock_irqsave(&fq->lock, flags);
	put = fq->put + 1;
	if (put >= fq->size)
		put = 0;
	if (put == fq->get) {
		smi_unlock_irqrestore(&fq->lock, flags);
		goto err;
	}
	smi_unlock_irqrestore(&fq->lock, flags);

	return 0;

err:
	return 1;
}

static void ved_tx_wake_timer(ulong data)
{
	struct net_device *dev = (struct net_device*)data;
	struct ved *ved = dev->priv;

	if (ved_tx_check_resources(ved))
		goto err;

	netif_wake_queue(dev);

	if (ved_ratelimit())
		dev_warn(&dev->dev, "waking TX queue\n");

	return;

err:
	/* some resource is still unavailable, rescheduling timer */
	ved_tx_add_timer(dev);
}

/*
 * This function is called when we receive invalid fdesc. It may cause fault,
 * because we dump payload of the fdesc (payload field value may be invalid
 * as well).
 */
static void fdesc_dump(struct smi_fdesc *fdesc)
{
	int i, len;
	char buf[200];

	printk(KERN_ERR "VED: invalid fdesc (phys: %08lx) content: part: %08lx, release: %04x, len: %08x, offs: %08x, interface: %08x\n",
		msp_virt_to_phys(fdesc), fdesc->fpart, fdesc->release, fdesc->length,
		fdesc->offset, fdesc->interface);

	len = snprintf(buf, sizeof(buf), "VED: fdesc payload:");
	for (i = 0; i < 30; i++)
		len += snprintf(buf + len, sizeof(buf), " %04x", ((u16 *)fdesc->payload)[i]);
	printk(KERN_ERR "%s\n", buf);
}

static inline int fdesc_check(struct smi_fdesc *fdesc)
{
	if (fdesc->offset + fdesc->length > VLAN_ETH_FRAME_LEN)
		return -1;

	if (fdesc->interface < IFACE_ETH1 || fdesc->interface > IFACE_ETH2)
		return -1;

	return 0;
}

static int ved_alert_action(void *arg)
{
	int err;
	static char *argv[] = {ved_alert_action_command, ved_alert_work.type, ved_alert_work.data, NULL};
	extern char *envp_init[];

	daemonize("ved-action");
	err = kernel_execve(ved_alert_action_command, argv, envp_init);
	printk(KERN_ERR "VED: failed to execute alert action command: '%s'\n", ved_alert_action_command);

	return 0;
}

static void ved_alert_handler(struct work_struct *work)
{
	if (!msp_controls_eth0()) {
		printk(KERN_ERR "VED: MSP soft-reset: %s\n", msp_reset(100) ? "failed":"ok");

		ved_stop();
		comcerto_eth_start();
	}

	kernel_thread(ved_alert_action, NULL, CLONE_KERNEL);
}

static inline int fdesc_check_alert(u32 phys)
{
	struct smi_fdesc *fdesc = msp_phys_to_virt(phys);
	u8 *payload;
	ulong flags;
	int i;

	if (__raw_readl(CBL_FLAGS_VADDR) & CBL_FLAGS_ALERT_ADDR_MASK) {
		/* newer firmware sets alert addr in embedded memory */
		if (phys != __raw_readl(SMI_ALERT_VADDR))
			goto not_an_alert;
	} else {
		u16 *buf;

		/* older firmware, we need to check payload, check lower MSP mem boundary first
		 * (upper was checked in caller code)
		 */
		if (phys > SDRAM_MSP_MEMORY_PHY + SDRAM_MSP_MEMORY_SIZE)
			goto not_an_alert;

		if (fdesc_check(fdesc))
			goto not_an_alert;

		buf = (u16*)(fdesc->payload + fdesc->offset);

		/* CSMENCAPS packet type, big endian */
		if (buf[6] != __constant_htons(0x889B))
			goto not_an_alert;

		/* command class 0xC1, command type 0x75 */
		if (buf[11] != 0xC175)
			goto not_an_alert;
	}

	local_irq_save(flags);

	payload = fdesc->payload + fdesc->offset;

	printk(KERN_ERR "\nMSP ALERT (type 0x%04X), frame dump:\n", ((u16*)payload)[14]);
	for (i = 0; i < fdesc->length; i++) {
		if ((i & 15) == 0)
			printk(KERN_ERR "%03X: ", i);
		printk("%02X%c", payload[i], ((i & 15) != 15 && i < fdesc->length - 1) ? ' ':'\n');
	}

	if ((__raw_readl(COMCERTO_INTC_ARM0_IRQMASK0) | __raw_readl(COMCERTO_INTC_ARM0_IRQMASK1)) == 0) {
		int len;

		/* MSP really crashed, we need to take a direct control over Ethernet interfaces */
		printk(KERN_ERR "VED: running alert actions\n");

		sprintf(ved_alert_work.type, "%04X", ((u16*)payload)[14]);

		len = min(sizeof(ved_alert_work.data)/2-1, fdesc->length);
		for (i = 0; i < len; i++)
			sprintf(ved_alert_work.data + i*2, "%02X", payload[i]);

		queue_work(ved_work_queue, (struct work_struct*) &ved_alert_work);
	}

	local_irq_restore(flags);

	return 0;

not_an_alert:
	return -1;
}

static void fdesc_clean_reserved(struct smi_fdesc *fdesc)
{
	memzero_ncnb(fdesc->reserved1, sizeof(fdesc->reserved1));
	fdesc->reserved2 = 0;
	fdesc->reserved3 = 0;
	fdesc->reserved4 = 0;
}

/*
 * This function should be called with interrupts off. It uses non-IRQ safe smi_*() calls.
 */
static int bridge_csme(struct smi_fdesc *fdesc, u8 *data_addr)
{
	u16 eth_packet_type;

	eth_packet_type = *(u16 *) (data_addr + 6 + 6);		/* 6 bytes per src and dest mac */

	if ((eth_packet_type == htons(0x889B)) && (fdesc->interface == IFACE_ETH0)) {
		/* all CSME packets coming from the eth0 queued back to MSP (eth1) */
		fdesc_clean_reserved(fdesc);
		fdesc->interface = IFACE_ETH1;

		if (!smi_enqueue(smi_csp2msp_queue, fdesc)) {
			if (ved_ratelimit())
				printk(KERN_ERR "VED: smi_enqueue failed\n");

			smi_free(fdesc);

			ved_err_enqueued++;
		} else
			ved_enqueued++;

		ved_rx_bridged++;

		goto bridged;
	}

	if (fdesc->interface == IFACE_ETH1) {
		/* MSP sends only CSME via eth1, change MAC to bridge host and queue back to MSP (eth0) */
		memcpy_ncnb(data_addr, bridge_mac, 6);

		memcpy_ncnb(data_addr + 6, comcerto_net_devs[0].dev_addr, 6);

		fdesc_clean_reserved(fdesc);
		fdesc->interface = IFACE_ETH0;

		if (!smi_enqueue(smi_csp2msp_queue, fdesc)) {
			if (ved_ratelimit())
				printk(KERN_ERR "VED: smi_enqueue failed\n");

			smi_free(fdesc);

			ved_err_enqueued++;
		} else
			ved_enqueued++;

		ved_tx_bridged++;

		goto bridged;
	}

	return 0;

bridged:
	return 1;
}

static void ved_rx(struct net_device *dev, char *buf, int len)
{
	struct ved *ved = dev->priv;
	struct sk_buff *skb;

	skb = dev_alloc_skb(len + 2);
	if (!skb) {
		if (ved_ratelimit())
			dev_err(&dev->dev, "low memory - packet dropped\n");

		ved->stats.rx_dropped++;

		ved->rx_err_skb_alloc++;

		return;
	}

	skb_reserve(skb, 2);	/* align IP header on 16 bytes boundary */

	memcpy(skb_put(skb, len), buf, len);

	skb->dev = dev;
	skb->protocol = eth_type_trans(skb, dev);

	if (netif_rx(skb) == NET_RX_DROP) {
		ved->stats.rx_dropped++;
	} else {
		ved->stats.rx_packets++;
		ved->stats.rx_bytes += len;

		dev->last_rx = jiffies;
	}
}

/*
 * This function should be called with interrupts off. It uses non-IRQ safe smi_*() calls.
 */
static void ved_read_packet(void)
{
	struct net_device *dev = NULL;
	ulong phys;
	struct smi_fdesc *fdesc;
	void *data_addr;

	while ((phys = smi_dequeue(smi_msp2csp_queue))) {
		/* basic check on physical address */
		if (phys < smi_ncnb_phys || phys > SDRAM_MSP_MEMORY_PHY + SDRAM_MSP_MEMORY_SIZE) {
			/* it may be an alert, which resides outside shared partitions */
			if (fdesc_check_alert(phys) == 0)
				goto phys_ok;

			printk(KERN_ERR "VED: invalid fdesc address: %08lx\n", phys);

			ved_err_dequeued++;
			continue;
		}

phys_ok:
		fdesc = msp_phys_to_virt(phys);
		if (fdesc_check(fdesc)) {
			fdesc_dump(fdesc);

			ved_err_dequeued++;
			goto release;
		}

		ved_dequeued++;

		data_addr = fdesc->payload + fdesc->offset;
		if (bridge_enabled) {
			if (bridge_csme(fdesc, data_addr))
				continue;
		}

		if (fdesc->interface == IFACE_ETH1) {
			dev = &comcerto_net_devs[1];
#if defined(CONFIG_ARCH_M823XX) || defined(CONFIG_ARCH_M828XX)
		} else if (fdesc->interface == IFACE_ETH2 && ved_is_owner(&comcerto_net_devs[2])) {
			dev = &comcerto_net_devs[2];
#endif
		} else {
			dev = &comcerto_net_devs[0];
		}

		ved_rx(dev, data_addr, fdesc->length);

release:
		if (fdesc->release)
			smi_free(fdesc);
	}
}

static irqreturn_t ved_interrupt(int irq, void *dev_id)
{
	ved_interrupts++;

	ved_read_packet();

	return IRQ_HANDLED;
}

static int ved_write_packet(struct sk_buff *skb, struct net_device *dev)
{
	struct ved *ved = dev->priv;
	struct smi_fdesc *fdesc;

	if ((skb->len == 0) || (skb->len > VLAN_ETH_FRAME_LEN))
		return -EINVAL;

	fdesc = smi_alloc_irqsafe(ved->tx_fp);
	if (!fdesc) {
		ved_err_alloc++;

		if (ved_ratelimit())
			dev_err(&dev->dev, "%s smi_alloc failed\n",
				ved->tx_fp == smi_csme_part?"smi_csme_part":"smi_eth_part");

		goto err;
	}

	fdesc_clean_reserved(fdesc);

	fdesc->interface = ved->tx_interface;
	fdesc->length = skb->len;
	fdesc->release = 1;
	fdesc->offset = 0;

	memcpy_ncnb(fdesc->payload, skb->data, skb->len);

	if (!smi_enqueue_irqsafe(smi_csp2msp_queue, fdesc)) {
		smi_free_irqsafe(fdesc);

		if (ved_ratelimit())
			dev_err(&dev->dev, "smi_enqueue failed\n");

		ved_err_enqueued++;

		goto err;
	} else
		ved_enqueued++;

	return 0;

err:
	/* we failed either to allocate part or queue fdesc, stop TX and start timeout */

	return -ENOMEM;
}

static int ved_xmit(struct sk_buff *skb, struct net_device *dev)
{
	int result;
	struct ved *ved = dev->priv;

	result = ved_write_packet(skb, dev);
	if (result) {
		/* we failed to get some resource - stop queue and schedule timeout */
		result = NETDEV_TX_BUSY;
		goto stop_tx;
	}

	dev_kfree_skb(skb);

	dev->trans_start = jiffies;

	ved->stats.tx_packets++;
	ved->stats.tx_bytes += skb->len;

	/* check if we have enough resources - if not stop tx, we may avoid loosing next packet */
	if (ved_tx_check_resources(ved)) {
		result = NETDEV_TX_OK;
		goto stop_tx;
	}

	return NETDEV_TX_OK;

stop_tx:
	netif_stop_queue(dev);
	ved_tx_add_timer(dev);

	return result;
}

int ved_wait_msp(void)
{
	int timeout_ms = 0;

	while (timeout_ms < 5500) {
		if (comcerto_softirq_check(IRQ_PTP0))
			break;

		/* this function may be called at the time udelay() support is not available, so
		 * using timer0 udelay which is always fine when we booted after firmware
		 */
		msp_timer0_udelay(1000);
		timeout_ms++;
	}

	return comcerto_softirq_check(IRQ_PTP0) ? 0 : -ETIME;
}

static int ved_open(struct net_device *dev)
{
	int err = 0;

	/* if VED is stopped do nothing and report ok to the kernel -
	 * interface will be really opened on ved_start()
	 */
	if (ved_stopped)
		return 0;

	/* we share single IRQ handler for all interfaces  */
	if (!ved_irq_requested) {
		ulong flags;

		err = request_irq(IRQ_PTP0, ved_interrupt, IRQF_SHARED, "ved", &comcerto_net_devs);
		if (err) {
			dev_err(&dev->dev, "failed to get the IRQ %d\n", IRQ_PTP0);
			goto out;
		}

		ved_irq_requested = 1;

		/* check if there are any pending received packets */

		local_irq_save(flags);

		ved_read_packet();

		local_irq_restore(flags);
	}

	netif_carrier_on(dev);
	netif_start_queue(dev);

out:
	return err;
}

static int ved_release(struct net_device *dev)
{
	struct ved *ved = dev->priv;
	int i, stopped = 1;

	if (ved_stopped)
		return 0;	/* interface is already closed, just tell the kernel everything is ok */

	netif_stop_queue(dev);	/* can't transmit any more */

	del_timer(&ved->tx_wake_timer);

	/* if all are stopped - disable IRQ */
	for (i = 0; i < 3; i++) {
		if (!ved_is_owner(&comcerto_net_devs[i]))
			continue;

		if (!netif_queue_stopped(&comcerto_net_devs[i])) {
			stopped = 0;
			break;
		}
	}

	if (ved_irq_requested && stopped) {
		ved_stopped = 1;

		free_irq(IRQ_PTP0, &comcerto_net_devs);

		ved_irq_requested = 0;
	}

	return 0;
}

static struct net_device_stats *ved_get_stats(struct net_device *dev)
{
	struct ved *ved = dev->priv;

	return &ved->stats;
}

int ved_stop(void)
{
	int i;

	if (ved_stopped)
		return 0;

	for (i = 0; i < 3; i++)
		if (ved_is_owner(&comcerto_net_devs[i]))
			ved_release(&comcerto_net_devs[i]);

	return 0;
}

static void ved_start_device(int device_id)
{
	struct net_device *dev = &comcerto_net_devs[device_id];
	struct ved *ved = &ved_devs[device_id];

	dev->priv = ved;
	dev->open = ved_open;
	dev->stop = ved_release;
	dev->poll = NULL;
	dev->weight = 0;
	dev->get_stats = ved_get_stats;
	dev->hard_start_xmit = ved_xmit;
	dev->set_multicast_list = NULL;
	dev->set_mac_address = NULL;
	dev->irq = IRQ_PTP0;

	dev->flags &= ~IFF_MULTICAST;

	if (dev->flags & IFF_UP)
		ved_open(dev);
}

int ved_start(void)
{
	int res;

	res = ved_wait_msp();
	if (res)
		return res;

	smi_init();

	ved_devs[0].tx_fp = smi_eth_part;
	ved_devs[1].tx_fp = smi_csme_part;
	ved_devs[2].tx_fp = smi_eth_part;

	ved_stopped = 0;

	if (msp_controls_eth0())
		ved_start_device(0);

	ved_start_device(1);

#if defined(CONFIG_ARCH_M823XX) || defined(CONFIG_ARCH_M828XX)
	if (msp_controls_eth2())
		ved_start_device(2);
#endif

	return 0;
}

static int __init ved_init(int device_id)
{
	struct net_device *dev = &comcerto_net_devs[device_id];
	struct ved *ved;

	SET_MODULE_OWNER(dev);

	ved = &ved_devs[device_id];

	init_timer(&ved->tx_wake_timer);
	ved->tx_wake_timer.function = &ved_tx_wake_timer;
	ved->tx_wake_timer.data = (unsigned long)dev;

	if (device_id == 1) {
		dev->flags |= IFF_NOARP;
		ved->tx_fp = smi_csme_part;
		ved->tx_interface = IFACE_ETH1;
#if defined(CONFIG_ARCH_M823XX) || defined(CONFIG_ARCH_M828XX)
	} else if (device_id == 2) {
		ved->tx_fp = smi_eth_part;
		ved->tx_interface = IFACE_ETH2;
#endif
	} else {
		ved->tx_fp = smi_eth_part;
		ved->tx_interface = IFACE_ETH0;
	}

#if defined(CONFIG_ARCH_M823XX) || defined(CONFIG_ARCH_M828XX)
	/* check whether firmware supports the second interface */
	if (device_id == 2) {
		if (msp_controls_eth2()) {
			printk(KERN_INFO "VED: eth2 support enabled\n");
		} else
			return -EBUSY;
	}
#endif

	dev->priv = ved;
	dev->open = ved_open;
	dev->stop = ved_release;
	dev->get_stats = ved_get_stats;
	dev->hard_start_xmit = ved_xmit;
	dev->irq = IRQ_PTP0;

	dev->flags &= ~IFF_MULTICAST;

	netif_stop_queue(dev);
	register_netdev(dev);

	return 0;
}

static int ved_proc_info_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	struct net_device *dev;
	struct ved *ved;
	int i, len = 0;

	for (i = 0; i < 3; i++) {
		dev = &comcerto_net_devs[i];
		if (!ved_is_owner(dev))
			continue;

		ved = dev->priv;

		len += snprintf(page + len,
			PAGE_SIZE - len,
			"%s:\n\t"
			"tx timer starts: %u, rx err skb alloc: %u\n",
			dev->name,
			ved->tx_err_timer, ved->rx_err_skb_alloc);
	}

	len += snprintf(page + len, PAGE_SIZE - len, "\nBridging:\n\t");
	if (bridge_enabled) {
		len += snprintf(page + len,
			PAGE_SIZE - len,
			"MAC                     : %02X.%02X.%02X.%02X.%02X.%02X\n\t"
			"packets bridged rx/tx   : %u/%u\n",
			bridge_mac[0], bridge_mac[1], bridge_mac[2],
			bridge_mac[3], bridge_mac[4], bridge_mac[5],
			ved_rx_bridged, ved_tx_bridged);
	} else
		len += snprintf(page + len, PAGE_SIZE - len, "disabled\n");

	len += snprintf(page + len,
		PAGE_SIZE - len,
		"\nSMI info:\n\t"
		"interrupts served, IRQ%02d: %u\n\t"
		"frames alloc fails      : %u\n\t"
		"frames enqueued ok/err  : %u/%u\n\t"
		"frames dequeued ok/err  : %u/%u\n\t"
		"NCNB area               : %p\n\t"
		"CSP to MSP queue        : %p\n\t"
		"MSP to CSP queue        : %p\n\t"
		"ETH  part               : %p\n\t"
		"CSME part               : %p\n",
		IRQ_PTP0, ved_interrupts,
		ved_err_alloc,
		ved_enqueued, ved_err_enqueued,
		ved_dequeued, ved_err_dequeued,
		msp_phys_to_virt(smi_ncnb_phys),
		smi_csp2msp_queue, smi_msp2csp_queue,
		smi_csme_part, smi_eth_part);

	if (len <= off + count)
		*eof = 1;

	*start = page + off;
	len -= off;
	if (len > count)
		len = count;

	if (len < 0)
		len = 0;

	return len;
}

static int ved_proc_bridge_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	int len = 0;

	if (bridge_enabled)
		len += snprintf(page, PAGE_SIZE, "%02X:%02X:%02X:%02X:%02X:%02X\n",
			bridge_mac[0], bridge_mac[1], bridge_mac[2], bridge_mac[3], bridge_mac[4], bridge_mac[5]);

	return len;
}

static int ved_proc_bridge_write(struct file *file, const char __user *buffer, unsigned long count, void *data)
{
	char *str;

	str = kzalloc(count + 1, GFP_KERNEL);
	if (!str)
		return -ENOMEM;

	if (copy_from_user(str, buffer, count)) {
		kfree(str);
		return -EFAULT;
	}

	bridge_setup(str);

	kfree(str);

	return count;
}

static int ved_proc_alert_action_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	int len = 0;

	len += snprintf(page, PAGE_SIZE, "%s\n", ved_alert_action_command);

	return len;
}

static int ved_proc_alert_action_write(struct file *file, const char __user *buffer, unsigned long count, void *data)
{
	int i;

	if (count > sizeof(ved_alert_action_command)-1)
		count = sizeof(ved_alert_action_command)-1;

	if (copy_from_user(ved_alert_action_command, buffer, count))
		return -EFAULT;

	for (i = 0; i < count - 1; i++)
		if (ved_alert_action_command[i] == '\n' || ved_alert_action_command[i] == '\r')
			break;

	ved_alert_action_command[i] = 0;

	return count;
}

static int __init ved_init_module(void)
{
	if (!comcerto_arm1_is_running()) {
		printk(KERN_INFO "VED: disabled\n");
		return -ENODEV;
	}

	printk(KERN_INFO "VED: version %s\n", VED_VERSION);

	smi_init();

	comcerto_eth_base_init();

	ved_init(0);

	ved_init(1);

#if defined(CONFIG_ARCH_M823XX) || defined(CONFIG_ARCH_M828XX)
	ved_init(2);
#endif

	ved_proc_dir = proc_mkdir("ved", proc_net);
	if (ved_proc_dir) {
		struct proc_dir_entry *pe;

		pe = create_proc_entry("bridge", 0664, ved_proc_dir);
		if (pe) {
			pe->read_proc = ved_proc_bridge_read;
			pe->write_proc = ved_proc_bridge_write;
		} else
			printk(KERN_ERR "VED: failed to create /proc/net/ved/bridge proc entry\n");

		create_proc_read_entry("info", 0444, ved_proc_dir, ved_proc_info_read, NULL);

		pe = create_proc_entry("msp-alert-action", 0664, ved_proc_dir);
		if (pe) {
			pe->read_proc = ved_proc_alert_action_read;
			pe->write_proc = ved_proc_alert_action_write;
		} else
			printk(KERN_ERR "VED: failed to create /proc/net/ved/msp-alert-action proc entry\n");
	} else
		printk(KERN_ERR "VED: failed to create /proc/net/ved dir\n");

	ved_work_queue = create_singlethread_workqueue("ved");
	INIT_WORK((struct work_struct*) &ved_alert_work, ved_alert_handler);

	return 0;
}

static void __exit ved_cleanup_module(void)
{
	/* not really supported */
}

static int bridge_setup(char *str)
{
	int i;

	if (strlen(str) >= 17) {
		for (i = 0; i < 6; i++, str += 3)
			bridge_mac[i] = simple_strtol(str, NULL, 16);
		bridge_enabled = 1;
	} else
		bridge_enabled = 0;

	if (bridge_enabled)
		printk(KERN_INFO "VED: bridging is enabled, host MAC %02X:%02X:%02X:%02X:%02X:%02X\n",
			 bridge_mac[0], bridge_mac[1], bridge_mac[2], bridge_mac[3], bridge_mac[4], bridge_mac[5]);
	else
		printk(KERN_INFO "VED: bridging is disabled\n");

	return 1;
}

__setup("bridge=", bridge_setup);

module_init(ved_init_module);
module_exit(ved_cleanup_module);
