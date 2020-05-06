/*
 * linux/drivers/net/comcerto/eth.c
 *
 * Copyright (C) 2009 Mindspeed Technologies, Inc.
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
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/etherdevice.h>
#include <linux/mii.h>
#include <linux/netdevice.h>
#include <linux/platform_device.h>
#include <asm/io.h>
#include <asm/arch/edma.h>
#include <asm/arch/gemac.h>
#include <asm/arch/hardware.h>
#include <asm/arch/irq.h>
#include <asm/arch/memory.h>
#include <asm/arch/msp.h>

#include "emac.h"
#include "eth.h"

#if defined(CONFIG_ARCH_M823XX)
struct net_device comcerto_net_devs[3];
#else
struct net_device comcerto_net_devs[2];
#endif

static int __init net_dev_init(struct net_device *dev, char *dev_name, u8 *mac_addr)
{
	int err;

	if (!is_valid_ether_addr(mac_addr)) {
		printk(KERN_ERR "ETH: invalid MAC address %02X.%02X.%02X.%02X.%02X.%02X\n",
			mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
		return -EADDRNOTAVAIL;
	}

	memcpy_fromio(dev->dev_addr, mac_addr, ETH_ALEN);

	err = dev_alloc_name(dev, dev_name);
	if (err < 0) {
		printk(KERN_ERR "ETH: failed to allocate dev name '%s'\n", dev_name);
		return err;
	}

	ether_setup(dev);

	return 0;
}

int __init comcerto_eth_base_init(void)
{
	int err;

	if (*comcerto_net_devs[0].name != 0)
		return -EBUSY;

	err = net_dev_init(&comcerto_net_devs[0], "eth0", CBL_M1_VADDR);
	if (err)
		return err;

	err = net_dev_init(&comcerto_net_devs[1], "eth1", CBL_M4_VADDR);
	if (err)
		return err;

#if defined(CONFIG_ARCH_M823XX)
	err = net_dev_init(&comcerto_net_devs[2], "eth2", CBL_M5_VADDR);
	if (err)
		return err;
#endif

	return 0;
}

#ifdef CONFIG_COMCERTO_ETH

struct edma_fdesc_ring {
	u16			head, tail;
	u16			size;
	struct edma_fdesc	*virt;
	u32			phys;
};

static struct comcerto_eth {
	struct edma_fdesc_ring	rxr;
	struct edma_fdesc_ring	txr;

	void			*edma_base;
	void			*emac_base;
	struct emac		*emac;

	void			*dummy_rx_virt;
	dma_addr_t		dummy_rx_phys;

	int			irq_rx;
	int			irq_tx;
	char			irq_rx_name[12];
	char			irq_tx_name[12];

	u32			irq_rx_mask;
	volatile u32		*irq_mask_reg;

	struct platform_device	*pdev;
	struct device		*dev;
	struct net_device	*net_dev;
	struct net_device_stats	stats;
	u8			device_id;
} eth_devs[3];

static int edma_fdesc_ring_alloc(struct comcerto_eth *eth, struct edma_fdesc_ring *ring, u16 size, u32 offset)
{
	int dma_size = size * sizeof(struct edma_fdesc);

	ring->size = size;

	ring->virt = dma_alloc_coherent(NULL, dma_size, &ring->phys, GFP_KERNEL);
	if (ring->virt == NULL)
		return -ENOMEM;

	ring->head = ring->tail = 0;

	memset(ring->virt, 0, dma_size);

	return 0;
}

static void edma_fdesc_ring_free(struct edma_fdesc_ring *ring)
{
	dma_free_coherent(NULL, ring->size * sizeof(struct edma_fdesc), ring->virt, ring->phys);
}

static void edma_fdesc_ring_purge(struct edma_fdesc_ring *ring)
{
	int i;
	struct edma_fdesc *fdesc;

	for (i = 0; i < ring->size; i++) {
		fdesc = &ring->virt[i];
		if (fdesc->system)
			dev_kfree_skb((struct sk_buff*) fdesc->system);
		fdesc->system = 0;
		fdesc->fstatus = 0;
		fdesc->fcontrol = 0;
		fdesc->bdesc.bptr = 0;
		fdesc->bdesc.bcontrol = 0;
	}

	ring->head = ring->tail = 0;
}

static inline u16 edma_fdesc_ring_inc_index(struct edma_fdesc_ring *ring, u16 index)
{
	if (++index >= ring->size)
		index = 0;
	return index;
}

static inline void edma_rx_irq_disable(struct comcerto_eth *eth)
{
	ulong flags;

	local_irq_save(flags);

	__raw_writel(__raw_readl(eth->irq_mask_reg) & ~eth->irq_rx_mask, eth->irq_mask_reg);

	local_irq_restore(flags);
}

static inline void edma_rx_irq_enable(struct comcerto_eth *eth)
{
	ulong flags;

	local_irq_save(flags);

	__raw_writel(__raw_readl(eth->irq_mask_reg) | eth->irq_rx_mask, eth->irq_mask_reg);

	local_irq_restore(flags);
}

static int eth_init_resources(struct platform_device *pdev, struct comcerto_eth *eth)
{
	char *name;
	struct resource *r;

	name = "tx";
	eth->irq_tx = platform_get_irq_byname(pdev, name);
	if (eth->irq_tx < 0)
		goto err_irq;

	name = "rx";
	eth->irq_rx = platform_get_irq_byname(pdev, name);
	if (eth->irq_rx < 0)
		goto err_irq;

	name = "emac";
	r = platform_get_resource_byname(pdev, IORESOURCE_MEM, name);
	if (!r)
		goto err_mem;
	eth->emac_base = (void*) APB_VADDR(r->start);

	name = "edma";
	r = platform_get_resource_byname(pdev, IORESOURCE_MEM, name);
	if (!r)
		goto err_mem;
	eth->edma_base = (void*) APB_VADDR(r->start);

	eth->irq_rx_mask = comcerto_irq_mask(eth->irq_rx);
	if (comcerto_arm1_is_running())
		eth->irq_mask_reg = (volatile u32*) COMCERTO_INTC_ARM1_IRQMASK0;
	else
		eth->irq_mask_reg = (volatile u32*) COMCERTO_INTC_ARM0_IRQMASK0;

	return 0;
err_mem:
	dev_err(eth->dev, "no '%s' memory region defined\n", name);
	return -EINVAL;
err_irq:
	dev_err(eth->dev, "no '%s' IRQ defined\n", name);
	return -EINVAL;
}

static int eth_alloc_rx_entry(struct comcerto_eth *eth, struct edma_fdesc *fdesc)
{
	struct net_device *dev = eth->net_dev;
	struct sk_buff *skb;

#if defined(CONFIG_ARCH_M823XX)
	skb = dev_alloc_skb(1536);
#else
	skb = dev_alloc_skb(2048);
#endif
	if (!skb) {
		if (net_ratelimit())
			dev_err(eth->dev, "failed to allocate skb\n");

		/* if we can't allocate skb - put a dummy buffer and drop this packet when received */
		fdesc->system = 0;
		fdesc->bdesc.bptr = eth->dummy_rx_phys;
	} else {
		skb->dev = dev;

		fdesc->system = (u32) skb;
		fdesc->bdesc.bptr = virt_to_phys(skb->data);
	}

	fdesc->fcontrol = EDMA_FCONTROL_FREADY | EDMA_FCONTROL_IRQEN;
	fdesc->bdesc.bcontrol = 0;

	return 0;
}

static int eth_alloc(struct comcerto_eth *eth)
{
	int i;

	if (edma_fdesc_ring_alloc(eth, &eth->rxr, 128, eth->device_id*(128+128)))
		return -ENOMEM;

	if (edma_fdesc_ring_alloc(eth, &eth->txr, 128, eth->device_id*(128+128) + 128))
		goto err_free_rx;

	eth->dummy_rx_virt = dma_alloc_coherent(NULL, PAGE_SIZE, &eth->dummy_rx_phys, GFP_KERNEL);
	if (eth->dummy_rx_virt == NULL)
		goto err_free_tx;

	/* init rings - set ::next pointers of frame descriptors */

	for (i = 0; i < eth->txr.size; i++)
		eth->txr.virt[i].next = i < (eth->txr.size - 1) ?
			eth->txr.phys + (i+1)*sizeof(struct edma_fdesc) : eth->txr.phys;

	for (i = 0; i < eth->rxr.size; i++)
		eth->rxr.virt[i].next = i < (eth->rxr.size - 1) ?
			eth->rxr.phys + (i+1)*sizeof(struct edma_fdesc) : eth->rxr.phys;

	return 0;

err_free_tx:
	edma_fdesc_ring_free(&eth->txr);
err_free_rx:
	edma_fdesc_ring_free(&eth->rxr);

	eth->txr.virt = NULL;
	dev_err(eth->dev, "failed to allocate DMA buffers\n");

	return -ENOMEM;
}

static void eth_free(struct comcerto_eth *eth)
{
	if (eth->txr.virt != NULL) {
		edma_fdesc_ring_free(&eth->txr);
		edma_fdesc_ring_free(&eth->rxr);
		dma_free_coherent(NULL, PAGE_SIZE, eth->dummy_rx_virt, eth->dummy_rx_phys);
		eth->txr.virt = NULL;
	}
}

static irqreturn_t edma_rx_interrupt(int irq, void *dev_id)
{
	struct comcerto_eth *eth = dev_id;
	struct net_device *dev = eth->net_dev;

	if (netif_rx_schedule_prep(dev)) {
		edma_rx_irq_disable(eth);
		__netif_rx_schedule(dev);
	} else {
		edma_rx_irq_disable(eth);
		BUG();
	}

	return IRQ_HANDLED;
}

static irqreturn_t edma_tx_interrupt(int irq, void *dev_id)
{
	struct comcerto_eth *eth = dev_id;
	struct edma_fdesc *fdesc;
	struct sk_buff *skb;

	while (1) {
		comcerto_irq_ack(irq);

		fdesc = &eth->txr.virt[eth->txr.head];
		if ((fdesc->fstatus & EDMA_FSTATUS_FDONE) == 0)
			break;

		skb = (struct sk_buff*) fdesc->system;

		if (emac_check_tx_frame(eth->emac, fdesc->fstatus) >= 0) {
			eth->stats.tx_packets++;
			eth->stats.tx_bytes += skb->len;
		} else
			eth->stats.tx_errors++;

		fdesc->system = 0;
		fdesc->fstatus = 0;
		fdesc->fcontrol = 0;

		dev_kfree_skb_irq(skb);
		eth->txr.head = edma_fdesc_ring_inc_index(&eth->txr, eth->txr.head);
	}

	EDMA_TX_START(eth->edma_base) = EDMA_START;

	return IRQ_HANDLED;
}

static void edma_start(struct comcerto_eth *eth)
{
	int i;

	eth->txr.head = eth->txr.tail = 0;
	eth->rxr.head = eth->rxr.tail = 0;

	for (i = 0; i < eth->rxr.size; i++)
		eth_alloc_rx_entry(eth, &eth->rxr.virt[i]);
	eth->rxr.tail = eth->rxr.size - 1;

	EDMA_TX_BURST(eth->edma_base) = 255;
	EDMA_TX_START(eth->edma_base) = 0;
	EDMA_TX_HEAD(eth->edma_base) = eth->txr.phys;

	EDMA_RX_BURST(eth->edma_base) = 255;
	EDMA_RX_HEAD(eth->edma_base) = eth->rxr.phys;
	EDMA_RX_START(eth->edma_base) = EDMA_START;
}

static void edma_reset(struct comcerto_eth *eth)
{
	EDMA_RX_RESET(eth->edma_base) = 1;
	EDMA_TX_RESET(eth->edma_base) = 1;
	udelay(100);
}

static void edma_stop(struct comcerto_eth *eth)
{
	edma_reset(eth);

	edma_fdesc_ring_purge(&eth->rxr);
	edma_fdesc_ring_purge(&eth->txr);
}

static int eth_open(struct net_device *dev)
{
	int err;
	struct comcerto_eth *eth = dev->priv;

	snprintf(eth->irq_rx_name, sizeof(eth->irq_rx_name), "%s rx", dev->name);
	comcerto_irq_ack(eth->irq_rx);
	err = request_irq(eth->irq_rx, edma_rx_interrupt, IRQF_DISABLED, eth->irq_rx_name, eth);
	if (err) {
		dev_err(eth->dev, "failed to request IRQ%d\n", eth->irq_rx);
		goto err;
	}

	snprintf(eth->irq_tx_name, sizeof(eth->irq_tx_name), "%s tx", dev->name);
	comcerto_irq_ack(eth->irq_tx);
	err = request_irq(eth->irq_tx, edma_tx_interrupt, IRQF_DISABLED, eth->irq_tx_name, eth);
	if (err) {
		dev_err(eth->dev, "failed to request IRQ%d\n", eth->irq_tx);
		goto err_tx;
	}

	netif_poll_enable(dev);

	edma_reset(eth);
	emac_stop(eth->emac);

	emac_set_mac_addr(eth->emac, 0, dev->dev_addr);
	emac_start(eth->emac);

	edma_start(eth);

	netif_start_queue(dev);

	return 0;

err_tx:
	free_irq(eth->irq_rx, eth);
err:
	return err;
}

static int eth_stop(struct net_device *dev)
{
	struct comcerto_eth *eth = dev->priv;

	if (dev->flags & IFF_UP) {
		free_irq(eth->irq_rx, eth);
		free_irq(eth->irq_tx, eth);

		emac_stop(eth->emac);
		edma_stop(eth);

		netif_poll_disable(dev);
		netif_stop_queue(dev);
	}

	return 0;
}

static int eth_poll(struct net_device *dev, int *budget)
{
	int i, limit = min(dev->quota, *budget), head = 0;
	struct edma_fdesc *fdesc;
	struct comcerto_eth *eth = dev->priv;
	u32 len;
	struct sk_buff *skb;

	for (i = 0; i < limit; i++) {
		comcerto_irq_ack(eth->irq_rx);

		fdesc = &eth->rxr.virt[eth->rxr.head];
		if ((fdesc->fstatus & EDMA_FSTATUS_FDONE) == 0)
			break;

		len = fdesc->bdesc.bcontrol & EDMA_BCONTROL_BLEN_MASK;
		skb = (void*) fdesc->system;
		if (skb) {
			if (emac_check_rx_frame(eth->emac, fdesc->fstatus) >= 0) {
				emac_csum_rx_frame(eth->emac, fdesc->fstatus, skb);
				fdesc->system = 0;
				fdesc->fstatus = 0;

				consistent_sync(skb->data, skb->len, DMA_FROM_DEVICE);

				skb_put(skb, len);
				skb->protocol = eth_type_trans(skb, dev);

				eth->stats.rx_packets++;
				eth->stats.rx_bytes += len;
				dev->last_rx = jiffies;

				netif_receive_skb(skb);
			} else {
				fdesc->fstatus = 0;

				eth->stats.rx_errors++;
			}
		} else {
			/* no skb was allocated for this fdesc, drop it */
			eth->stats.rx_dropped++;
		}

		if (!fdesc->system)
			eth_alloc_rx_entry(eth, fdesc);

		head = eth->rxr.head;
		eth->rxr.head = edma_fdesc_ring_inc_index(&eth->rxr, eth->rxr.head);
	}

	if (i) {
		eth->rxr.virt[head].fcontrol = 0;
		eth->rxr.virt[eth->rxr.tail].fcontrol = EDMA_FCONTROL_FREADY | EDMA_FCONTROL_IRQEN;
		eth->rxr.tail = head;
	}

	dev->quota -= i;
	*budget -= i;

	if (EDMA_RX_START(eth->edma_base) == 0) {
		emac_kick_rx(eth->emac);

		EDMA_RX_HEAD(eth->edma_base) = EDMA_RX_HEAD(eth->edma_base);

		EDMA_RX_START(eth->edma_base) = EDMA_START;

		emac_start_rx(eth->emac);
	}

	if (i < limit || !netif_running(dev)) {
		netif_rx_complete(dev);
		edma_rx_irq_enable(eth);

		return 0;
	}

	return 1;
}

static int eth_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct comcerto_eth *eth = dev->priv;
	struct edma_fdesc *fdesc;
	unsigned long flags;

	local_irq_save(flags);

	fdesc = &eth->txr.virt[eth->txr.tail];

	if (fdesc->fcontrol != 0) {	/* is frame descriptor free ? */
		dev_err(eth->dev, "no free space, stopping tx\n");

		/* no free space - stop queue and return BUSY */
		eth->stats.tx_dropped++;

		netif_poll_disable(dev);
		netif_stop_queue(dev);
		local_irq_restore(flags);

		return NETDEV_TX_BUSY;
	}

	fdesc->bdesc.bptr = virt_to_phys(skb->data);
	consistent_sync(skb->data, skb->len, DMA_TO_DEVICE);

	fdesc->bdesc.bcontrol = skb->len | EDMA_BCONTROL_BLAST;
	fdesc->system = (u32) skb;
	fdesc->fstatus = 0;
	fdesc->fcontrol = EDMA_FCONTROL_FREADY | EDMA_FCONTROL_IRQEN;

	/* move tail to the next descriptor */
	eth->txr.tail = edma_fdesc_ring_inc_index(&eth->txr, eth->txr.tail);

	EDMA_TX_START(eth->edma_base) = EDMA_START;

	local_irq_restore(flags);

	return NETDEV_TX_OK;
}

static struct net_device_stats *eth_get_stats(struct net_device *dev)
{
	struct comcerto_eth *eth = dev->priv;

	return &eth->stats;
}

static void eth_set_multicast_list(struct net_device *dev)
{
	struct comcerto_eth *eth = dev->priv;

	emac_set_promiscuous(eth->emac, dev->flags & IFF_PROMISC);	/* set promiscuous mode if requested */

	/* FIXME: support multicast and loopback */
}

static int eth_set_mac_address(struct net_device *dev, void *addr)
{
	struct sockaddr *macaddr = (struct sockaddr*) addr;
	struct comcerto_eth *eth = dev->priv;

	if (!is_valid_ether_addr(macaddr->sa_data))
		return -EADDRNOTAVAIL;

	memcpy(dev->dev_addr, macaddr->sa_data, dev->addr_len);

	emac_set_mac_addr(eth->emac, 0, macaddr->sa_data);

	return 0;
}

static int eth_probe(struct platform_device *pdev)
{
	int device_id = pdev->id;
	int err;
	char msg[64];
	struct net_device *dev = NULL;
	struct comcerto_eth *eth = NULL;

	/* device_id may be 0 or 1, multiplied by 2 it produces interface index - 0 or 2 (1 is reserved for VED eth1) */
	if (device_id < 0)
		device_id = 0;
	if (device_id > 1) {
		dev_err(&pdev->dev, "driver supports two devices max, device id %d is invalid, aborting\n", device_id);
		return -ENODEV;
	}

	dev_dbg(&pdev->dev, "probe\n");

	if (pdev->dev.platform_data == NULL) {
		dev_err(&pdev->dev, "no platform data, aborting\n");
		return -EINVAL;
	}

	dev = &comcerto_net_devs[device_id*2];
	eth = &eth_devs[device_id];
	eth->device_id = device_id;
	dev->priv = eth;

	platform_set_drvdata(pdev, dev);

	eth->pdev = pdev;
	eth->dev = &pdev->dev;
	eth->net_dev = dev;

	err = eth_init_resources(pdev, eth);
	if (err < 0)
		goto err;

	err = eth_alloc(eth);
	if (err < 0) {
		dev_err(eth->dev, "failed to allocate resources\n");
		goto err;
	}

#ifdef CONFIG_MSP_CARRIER
	/* we need to check whether the interface is supported by firmware or CSP */
	if ((device_id == 0 && msp_controls_eth0()) || (device_id == 1 && msp_controls_eth2())) {
		dev_info(&pdev->dev, "eth%d support disabled\n", device_id*2);
		return -EBUSY;
	}
#endif

	dev->irq = eth->irq_rx;

	SET_MODULE_OWNER(dev);
	SET_NETDEV_DEV(dev, &pdev->dev);

	dev->open = eth_open;
	dev->stop = eth_stop;
	dev->poll = eth_poll;
	dev->weight = 16;
	dev->hard_start_xmit = eth_xmit;
	dev->get_stats = eth_get_stats;
	dev->set_multicast_list = eth_set_multicast_list;
	dev->set_mac_address = eth_set_mac_address;

	err = register_netdev(dev);
	if (err) {
		dev_err(&pdev->dev, "failed to register net device '%s', aborting\n", dev->name);
		goto err_free;
	}

	eth->emac = emac_open(dev, pdev, eth->emac_base);
	if (!eth->emac) {
		dev_err(eth->dev, "failed to configure EMAC\n");
		err = -ENXIO;
		goto err_free;
	}

	dev->base_addr = (unsigned long) eth->emac;	/* needed for lower layer */

	snprintf(msg, sizeof(msg), "registered device '%s', MAC %02X:%2X:%2X:%02X:%2X:%2X",
			dev->name,
			dev->dev_addr[0], dev->dev_addr[1], dev->dev_addr[2],
			dev->dev_addr[3], dev->dev_addr[4], dev->dev_addr[5]);
	dev_info(&pdev->dev, "%s\n", msg);

	return 0;

err_free:
	eth_free(eth);

err:
	return err;
}

static int eth_remove(struct platform_device *pdev)
{
	/* not supported */

	return 0;
}

static struct platform_driver eth_driver = {
	.probe = eth_probe,
	.remove = eth_remove,
	.driver = {
		.name = "comcerto-eth",
	},
};

static int __init comcerto_eth_init(void)
{
	int err;

	err = comcerto_eth_base_init();
	if (!err)
		err = platform_driver_register(&eth_driver);

	return err;
}

static void __exit comcerto_eth_exit(void)
{
	/* not supported */
}

module_init(comcerto_eth_init);
module_exit(comcerto_eth_exit);

static int eth_start_device(int device_id)
{
	struct net_device *dev = &comcerto_net_devs[device_id*2];
	struct comcerto_eth *eth = &eth_devs[device_id];

	dev->irq = eth->irq_rx;

	SET_MODULE_OWNER(dev);
	SET_NETDEV_DEV(dev, eth->dev);

	dev->open = eth_open;
	dev->stop = eth_stop;
	dev->poll = eth_poll;
	dev->weight = 16;
	dev->hard_start_xmit = eth_xmit;
	dev->get_stats = eth_get_stats;
	dev->set_multicast_list = eth_set_multicast_list;
	dev->set_mac_address = eth_set_mac_address;
	dev->priv = eth;

	if (!eth->emac) {
		eth->emac = emac_open(dev, eth->pdev, eth->emac_base);
		if (!eth->emac)
			return -ENXIO;
	}

	dev->base_addr = (unsigned long) eth->emac;	/* needed for lower layer */

	netif_stop_queue(dev);
	emac_stop(eth->emac);
	edma_reset(eth);

	if (dev->flags & IFF_UP)
		eth_open(dev);

	return 0;
}

void comcerto_eth_start(void)
{
	struct net_device *dev;

	dev = &comcerto_net_devs[0];
	if (dev->hard_start_xmit != eth_xmit)
		eth_start_device(0);

#if defined(CONFIG_ARCH_M823XX)
	/* we have 3 network devices total, but 2 physical devices, that's why we pass '1' into eth_start_device() */
	dev = &comcerto_net_devs[2];
	if (dev->hard_start_xmit != eth_xmit)
		eth_start_device(1);
#endif
}

void comcerto_eth_start_eth2(void)
{
#if defined(CONFIG_ARCH_M823XX)
	struct net_device *dev = &comcerto_net_devs[2];

	if (dev->hard_start_xmit != eth_xmit && !msp_controls_eth2())
		eth_start_device(1);
#endif
}

void comcerto_eth_stop(void)
{
	struct net_device *dev;

	dev = &comcerto_net_devs[0];
	if (dev->hard_start_xmit == eth_xmit) {
		eth_stop(dev);
		dev->hard_start_xmit = NULL;
	}

#if defined(CONFIG_ARCH_M823XX)
	dev = &comcerto_net_devs[2];
	if (dev->hard_start_xmit == eth_xmit) {
		eth_stop(dev);
		dev->hard_start_xmit = NULL;

		/* needed to clear __LINK_STATE_RX_SCHED bit, otherwise deadlock in dev_close()
		 * after MSP reload
		 */
		netif_poll_enable(dev);
	}
#endif
}

#endif
