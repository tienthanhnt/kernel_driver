/*
 * linux/drivers/net/comcerto/gemac.c
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
 *
 */
#include <linux/kernel.h>
#include <linux/phy.h>
#include <asm/delay.h>
#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/arch/gemac.h>
#include <asm/arch/msp.h>

#include "emac.h"

struct emac {
	void	*gemac_base;
	int	mode;
	int	duplex;
	int	speed;
	int	link;
	int	phy_addr;

	struct device		*dev;
	struct net_device	*net_dev;
	struct phy_device	*phy_dev;
};


static inline void gemac_halt_tx(struct emac *emac)
{
	GEMAC_NET_CONTROL(emac->gemac_base) |= GEMAC_NET_CONTROL_TX_HALT;	/* wait until frame will be transmitted */
}

static inline void gemac_stop_tx(struct emac *emac)
{
	GEMAC_NET_CONTROL(emac->gemac_base) &= ~GEMAC_NET_CONTROL_TX_ENABLE;	/* don't wait until frame will be transmitted */
}

static inline void gemac_stop_rx(struct emac *emac)
{
	GEMAC_NET_CONTROL(emac->gemac_base) &= ~GEMAC_NET_CONTROL_RX_ENABLE;
}

static inline void gemac_start_tx(struct emac *emac)
{
	GEMAC_NET_CONTROL(emac->gemac_base) |= GEMAC_NET_CONTROL_TX_ENABLE;
}

static inline void gemac_start_rx(struct emac *emac)
{
	GEMAC_NET_CONTROL(emac->gemac_base) |= GEMAC_NET_CONTROL_RX_ENABLE;
}

static int gemac_get_speed(void *gemac_base)
{
	int mii_speed = SPEED_10;
	u32 conf = GEMAC_CONFIG(gemac_base);

	if (conf & GEMAC_CONFIG_SPEED_SEL)
		conf >>= GEMAC_CONFIG_SPEED_GEM_SHIFT;
	else
		conf >>= GEMAC_CONFIG_SPEED_PHY_SHIFT;

	switch (conf & GEMAC_CONFIG_SPEED_MASK) {
	case GEMAC_CONFIG_SPEED_10:
		mii_speed = SPEED_10;
		break;
	case GEMAC_CONFIG_SPEED_100:
		mii_speed = SPEED_100;
		break;
	case GEMAC_CONFIG_SPEED_1000:
		mii_speed = SPEED_1000;
		break;
	default:
		BUG();
	}

	return	mii_speed;
}

static int gemac_set_speed(void *gemac_base, int speed)
{
	u32 gemac_speed = 0;
	u32 conf;

	conf = GEMAC_CONFIG(gemac_base);
	if ((conf & GEMAC_CONFIG_SPEED_SEL) == 0)
		BUG();

	conf &= ~(GEMAC_CONFIG_SPEED_MASK << GEMAC_CONFIG_SPEED_GEM_SHIFT);

	switch (speed) {
	case SPEED_10:
		gemac_speed = GEMAC_CONFIG_SPEED_10;
		break;
	case SPEED_100:
		gemac_speed = GEMAC_CONFIG_SPEED_100;
		break;
	case SPEED_1000:
		gemac_speed = GEMAC_CONFIG_SPEED_1000;
		break;
	default:
		BUG();
	}

	GEMAC_CONFIG(gemac_base) = conf | (gemac_speed << GEMAC_CONFIG_SPEED_GEM_SHIFT);

	return 0;
}

static int gemac_get_duplex(void *gemac_base)
{
	int mii_duplex = DUPLEX_HALF;
	u32 conf = GEMAC_CONFIG(gemac_base);

	if (conf & GEMAC_CONFIG_DUPLEX_SEL)
		conf >>= GEMAC_CONFIG_DUPLEX_GEM_SHIFT;
	else
		conf >>= GEMAC_CONFIG_DUPLEX_PHY_SHIFT;

	switch (conf & GEMAC_CONFIG_DUPLEX_MASK) {
	case GEMAC_CONFIG_DUPLEX_HALF:
		mii_duplex = DUPLEX_HALF;
		break;
	case GEMAC_CONFIG_DUPLEX_FULL:
		mii_duplex = DUPLEX_FULL;
		break;
	default:
		BUG();
	}

	return mii_duplex;
}

static int gemac_set_duplex(void *gemac_base, int duplex)
{
	u32 gemac_duplex = 0;
	u32 conf;

	conf = GEMAC_CONFIG(gemac_base);
	if ((conf & GEMAC_CONFIG_DUPLEX_SEL) == 0)
		BUG();

	conf &= ~(GEMAC_CONFIG_DUPLEX_MASK << GEMAC_CONFIG_DUPLEX_GEM_SHIFT);

	switch (duplex) {
	case DUPLEX_HALF:
		gemac_duplex = GEMAC_CONFIG_DUPLEX_HALF;
		break;
	case DUPLEX_FULL:
		gemac_duplex = GEMAC_CONFIG_DUPLEX_FULL;
		break;
	default:
		BUG();
	}

	GEMAC_CONFIG(gemac_base) = conf | (gemac_duplex << GEMAC_CONFIG_DUPLEX_GEM_SHIFT);

	return 0;
}

static int emac_config_mode(struct emac *emac, int mode)
{
	u32 conf;

	conf = GEMAC_CONFIG(emac->gemac_base);

	switch (mode) {
	case GEMAC_CONFIG_MODE_BOOT:
		break;
	case GEMAC_CONFIG_MODE_PIN:
		conf &= ~GEMAC_CONFIG_MODE_SEL;
		break;
	case GEMAC_CONFIG_MODE_MII:
	case GEMAC_CONFIG_MODE_GMII:
	case GEMAC_CONFIG_MODE_RMII:
	case GEMAC_CONFIG_MODE_RGMII:
#ifdef CONFIG_ARCH_M823V1
	case GEMAC_CONFIG_MODE_SGMII:
	case GEMAC_CONFIG_MODE_TBI:
#else
	case GEMAC_CONFIG_MODE_SMII:
#endif
		conf &= ~(GEMAC_CONFIG_MODE_MASK << GEMAC_CONFIG_MODE_GEM_SHIFT);
		conf |= (mode << GEMAC_CONFIG_MODE_GEM_SHIFT) | GEMAC_CONFIG_MODE_SEL;
		break;
	default:
		return -EINVAL;
	}

	GEMAC_CONFIG(emac->gemac_base) = conf;

	/* we don't set mode in two states above, so reload it from config value */
	if (conf & GEMAC_CONFIG_MODE_SEL)
		mode = (conf >> GEMAC_CONFIG_MODE_GEM_SHIFT) & GEMAC_CONFIG_MODE_MASK;
	else
		mode = (conf >> GEMAC_CONFIG_MODE_PIN_SHIFT) & GEMAC_CONFIG_MODE_MASK;

	emac->mode = mode;

#ifdef CONFIG_ARCH_M823V1
	/* select TBI for TBI/SGMII or MII/GMII for the rest of modes */
	conf = GEMAC_NET_CONFIG(emac->gemac_base) & ~GEMAC_NET_CONFIG_PCS_SELECT;
	if (mode == GEMAC_CONFIG_MODE_SGMII || mode == GEMAC_CONFIG_MODE_TBI)
		conf |= GEMAC_NET_CONFIG_PCS_SELECT;
	GEMAC_NET_CONFIG(emac->gemac_base) = conf;
#endif

	return 0;
}

static int emac_config_speed(struct emac *emac, int speed)
{
	int mii_speed;
	u32 conf;

	conf = GEMAC_CONFIG(emac->gemac_base);

	/* for 10,100,1000 - encode to MII format and pass to common setter */
	switch (emac->speed) {
	case GEMAC_CONFIG_SPEED_PHY:
		GEMAC_CONFIG(emac->gemac_base) = conf & ~GEMAC_CONFIG_SPEED_SEL;
		return 0;
	case GEMAC_CONFIG_SPEED_10:
		mii_speed = SPEED_10;
		break;
	case GEMAC_CONFIG_SPEED_100:
		mii_speed = SPEED_100;
		break;
	case GEMAC_CONFIG_SPEED_1000:
		mii_speed = SPEED_1000;
		break;
	default:
		return -EINVAL;
	}

	emac->speed = speed;

	GEMAC_CONFIG(emac->gemac_base) = conf | GEMAC_CONFIG_SPEED_SEL;
	gemac_set_speed(emac->gemac_base, mii_speed);

	return 0;
}

static int emac_config_duplex(struct emac *emac, int duplex)
{
	int mii_duplex;
	u32 conf;

	conf = GEMAC_CONFIG(emac->gemac_base);

	switch (emac->duplex) {
	case GEMAC_CONFIG_DUPLEX_PHY:
		GEMAC_CONFIG(emac->gemac_base) = conf & ~GEMAC_CONFIG_DUPLEX_SEL;
		return 0;
	case GEMAC_CONFIG_DUPLEX_HALF:
		mii_duplex = DUPLEX_HALF;
		break;
	case GEMAC_CONFIG_DUPLEX_FULL:
		mii_duplex = DUPLEX_FULL;
		break;
	default:
		return -EINVAL;
	}

	emac->duplex = duplex;

	GEMAC_CONFIG(emac->gemac_base) = conf | GEMAC_CONFIG_DUPLEX_SEL;
	gemac_set_duplex(emac->gemac_base, mii_duplex);

	return 0;
}

/*
 * This is used to map GEMAC mode to PHY layer interface value
 */
static phy_interface_t gemac_get_phy_interface(void *gemac_base)
{
	u32 conf, mode;
	phy_interface_t interface = PHY_INTERFACE_MODE_GMII;

	conf = GEMAC_CONFIG(gemac_base);
	if (conf & GEMAC_CONFIG_MODE_SEL)	
		mode = conf >> GEMAC_CONFIG_MODE_GEM_SHIFT;
	else
		mode = conf >> GEMAC_CONFIG_MODE_PIN_SHIFT;
	mode &= GEMAC_CONFIG_MODE_MASK;

	switch (mode) {
	case GEMAC_CONFIG_MODE_MII:
		interface = PHY_INTERFACE_MODE_MII;
		break;
	case GEMAC_CONFIG_MODE_RMII:
		interface = PHY_INTERFACE_MODE_RMII;
		break;
	case GEMAC_CONFIG_MODE_RGMII:
		interface = PHY_INTERFACE_MODE_RGMII;
		break;
#ifdef CONFIG_ARCH_M823V1
	case GEMAC_CONFIG_MODE_SGMII:
		interface = PHY_INTERFACE_MODE_SGMII;
		break;
	case GEMAC_CONFIG_MODE_TBI:
		interface = PHY_INTERFACE_MODE_TBI;
#else
	case GEMAC_CONFIG_MODE_SMII:
		interface = PHY_INTERFACE_MODE_SGMII;	/* shut the PHY layer up */
#endif
	}

	return interface;
}

static void gemac_adjust_link(struct net_device *net_dev)
{
	struct emac *emac = (struct emac*) net_dev->base_addr;	/* must be set in upper eth code */
	int changed = 0;

	if (emac->speed >= 0 && emac->phy_dev->speed != emac->speed) {
		changed++;

		switch (emac->phy_dev->speed) {
		case SPEED_10:
		case SPEED_100:
		case SPEED_1000:
			emac->speed = emac->phy_dev->speed;
			gemac_set_speed(emac->gemac_base, emac->speed);
			break;
		default:
			dev_warn(&net_dev->dev, "invalid speed reported by PHY: %d\n", emac->phy_dev->speed);
			changed--;
		}
	}

	if (emac->duplex >= 0 && emac->phy_dev->duplex != emac->duplex) {
		changed++;

		emac->duplex = emac->phy_dev->duplex;
		gemac_set_duplex(emac->gemac_base, emac->duplex);
	}

	if (emac->phy_dev->link != emac->link) {
		changed++;

		emac->link = emac->phy_dev->link;
	}

	if (changed) {
		if (emac->link)
			printk(KERN_INFO "%s: link up, %dMbps, %s-duplex\n",
				net_dev->name, emac->speed, emac->duplex == DUPLEX_HALF ? "half" : "full");
		else
			printk(KERN_INFO "%s: link down\n", net_dev->name);
	}
}

struct emac *emac_open(struct net_device *net_dev, struct platform_device *pdev, void *gemac_base)
{
	struct emac *emac;
	struct device *dev = &pdev->dev;
	struct comcerto_gemac_platform_data *platform = dev->platform_data;

	emac = kzalloc(sizeof(*emac), GFP_KERNEL);
	if (!emac)
		return NULL;

	emac->gemac_base = gemac_base;
	emac->dev = dev;
	emac->net_dev = net_dev;
	emac->phy_addr = platform->phy_addr;

	/* the first GEMAC controls MDIO bus, preserve MDIO enable bit for it */
	if (pdev->id == 0)
		GEMAC_NET_CONTROL(gemac_base) = GEMAC_NET_CONTROL(gemac_base) & GEMAC_NET_CONTROL_MDIO_ENABLE;
	else
		GEMAC_NET_CONTROL(gemac_base) = 0;
	GEMAC_NET_CONFIG(gemac_base) = (GEMAC_NET_CONFIG(gemac_base) & GEMAC_NET_CONFIG_PCS_SELECT) | GEMAC_NET_CONFIG_RECEIVE_1536;

	if (emac_config_mode(emac, platform->mode))
		goto err;

	if (platform->mode != GEMAC_CONFIG_MODE_BOOT) {
		if (emac_config_speed(emac, platform->speed))
			goto err;

		if (emac_config_duplex(emac, platform->duplex))
			goto err;
	}

	/* If speed or duplex are requested to set by PHY hardware, set variables to negative,
	 * so we won't change these fields in adjust link. Otherwise read current speed and
	 * duplex settings.
	 */
	if (emac->speed != GEMAC_CONFIG_SPEED_PHY)
		emac->speed = gemac_get_speed(emac->gemac_base);
	else
		emac->speed = -1;

	if (emac->duplex != GEMAC_CONFIG_SPEED_PHY)
		emac->duplex = gemac_get_duplex(emac->gemac_base);
	else
		emac->duplex = -1;

	emac->link = 0;		/* start with link off */

	return emac;
err:
	kfree(emac);
	return NULL;
}

int emac_close(struct emac *emac)
{
	if (!emac)
		return -EINVAL;

	emac_stop(emac);

	if (emac->phy_dev)
		phy_disconnect(emac->phy_dev);

	kfree(emac);

	return 0;
}

int emac_get_broadcast(struct emac *emac)
{
	return GEMAC_NET_CONFIG(emac->gemac_base) & GEMAC_NET_CONFIG_NO_BROADCAST ? 0 : 1;
}

void emac_set_broadcast(struct emac *emac, int broadcast)
{
	u32 conf = GEMAC_NET_CONFIG(emac->gemac_base);

	conf &= ~GEMAC_NET_CONFIG_NO_BROADCAST;
	if (!broadcast)
		conf |= GEMAC_NET_CONFIG_NO_BROADCAST;

	GEMAC_NET_CONFIG(emac->gemac_base) = conf;
}

int emac_get_unicast(struct emac *emac)
{
	return GEMAC_NET_CONFIG(emac->gemac_base) & GEMAC_NET_CONFIG_UNICAST ? 1 : 0;
}

void emac_set_unicast(struct emac *emac, int unicast)
{
	u32 conf = GEMAC_NET_CONFIG(emac->gemac_base);

	conf &= ~GEMAC_NET_CONFIG_UNICAST;
	if (unicast)
		conf |= GEMAC_NET_CONFIG_UNICAST;

	GEMAC_NET_CONFIG(emac->gemac_base) = conf;
}

int emac_get_multicast(struct emac *emac)
{
	return GEMAC_NET_CONFIG(emac->gemac_base) & GEMAC_NET_CONFIG_MULTICAST ? 1 : 0;
}

void emac_set_multicast(struct emac *emac, int multicast)
{
	u32 conf = GEMAC_NET_CONFIG(emac->gemac_base);

	conf &= ~GEMAC_NET_CONFIG_MULTICAST;
	if (multicast)
		conf |= GEMAC_NET_CONFIG_MULTICAST;

	GEMAC_NET_CONFIG(emac->gemac_base) = conf;
}

static void gemac_set_rx_fcs_remove(void *gemac_base, int rx_fcs_remove)
{
	u32 conf = GEMAC_NET_CONFIG(gemac_base);

	conf &= ~GEMAC_NET_CONFIG_RX_FCS_REMOVE;
	if (rx_fcs_remove)
		conf |= GEMAC_NET_CONFIG_RX_FCS_REMOVE;

	GEMAC_NET_CONFIG(gemac_base) = conf;
}

static void gemac_set_rx_cs_offload(void *gemac_base, int rx_checksum_offload)
{
	u32 conf = GEMAC_NET_CONFIG(gemac_base);

	conf &= ~GEMAC_NET_CONFIG_RX_CS_OFFLOAD;
	if (rx_checksum_offload)
		conf |= GEMAC_NET_CONFIG_RX_CS_OFFLOAD;

	GEMAC_NET_CONFIG(gemac_base) = conf;
}

int emac_get_promiscuous(struct emac *emac)
{
	return GEMAC_NET_CONFIG(emac->gemac_base) & GEMAC_NET_CONFIG_COPY_ALL ? 1 : 0;
}

void emac_set_promiscuous(struct emac *emac, int promiscuous)
{
	u32 conf = GEMAC_NET_CONFIG(emac->gemac_base);

	conf &= ~GEMAC_NET_CONFIG_COPY_ALL;
	if (promiscuous)
		conf |= GEMAC_NET_CONFIG_COPY_ALL;

	GEMAC_NET_CONFIG(emac->gemac_base) = conf;
}

static void gemac_reset(struct emac *emac)
{
	u32 dummy;

	gemac_stop_tx(emac);
	gemac_stop_rx(emac);

	GEMAC_IRQ_DISABLE(emac->gemac_base) = -1;	/* disable all IRQs */
	dummy = GEMAC_IRQ_STATUS(emac->gemac_base);

	/* set MAC registers to default state */
}

int emac_get_mac_addr(struct emac *emac, int id, u8 *mac_addr)
{
	u32 bottom, top;

	if (id & ~3)
		return -EINVAL;

	bottom = GEMAC_SPEC_ADDR_BOTTOM(emac->gemac_base, id);
	top = GEMAC_SPEC_ADDR_TOP(emac->gemac_base, id);

	mac_addr[0] = bottom;
	mac_addr[1] = bottom >> 8;
	mac_addr[2] = bottom >> 16;
	mac_addr[3] = bottom >> 24;
	mac_addr[4] = top;
	mac_addr[5] = top >> 8;

	return 0;
}

int emac_set_mac_addr(struct emac *emac, int id, u8 *mac_addr)
{
	u32 bottom, top;

	if (id & ~3)
		return -EINVAL;

	bottom = mac_addr[0] | (mac_addr[1] << 8) | (mac_addr[2] << 16) | (mac_addr[3] << 24);
	top = mac_addr[4] | (mac_addr[5] << 8);

	GEMAC_SPEC_ADDR_BOTTOM(emac->gemac_base, id) = bottom;
	GEMAC_SPEC_ADDR_TOP(emac->gemac_base, id) = top;

	return 0;
}

void emac_start(struct emac *emac)
{
	gemac_reset(emac);

	GEMAC_TX_CONTROL(emac->gemac_base) = GEMAC_TX_CONTROL_DMAIF_ENABLE | GEMAC_TX_CONTROL_CRC_ENABLE
		| GEMAC_TX_CONTROL_RETR_ENABLE;

	GEMAC_RX_CONTROL(emac->gemac_base) = GEMAC_RX_CONTROL_DMAIF_ENABLE;
	GEMAC_RX_STATUS_PACK_SIZE(emac->gemac_base) = 0x80;

	GEMAC_HCSM_FIFO_CTRL(emac->gemac_base) = GEMAC_HCSM_FIFO_CTRL_RXWRREQEN | GEMAC_HCSM_FIFO_CTRL_TXRDREQEN | GEMAC_HCSM_FIFO_CTRL_TXFFRES;
	GEMAC_HCSM_RX_FIFO_HIGH(emac->gemac_base) = 0xE0;
	GEMAC_HCSM_RX_FIFO_LOW(emac->gemac_base) = 0x40;

	GEMAC_GEM_FIFO_CTRL(emac->gemac_base) = GEMAC_GEM_FIFO_CTRL_TXFF_EN | GEMAC_GEM_FIFO_CTRL_HBTXRQ_EN
		| GEMAC_GEM_FIFO_CTRL_RXFF_EN | GEMAC_GEM_FIFO_CTRL_HBRXRQ_EN | GEMAC_GEM_FIFO_CTRL_TXCP_INH;
	GEMAC_GEM_RX_FIFO_HIGH(emac->gemac_base) = 0x17;
	GEMAC_GEM_RX_FIFO_LOW(emac->gemac_base) = 0x16;

	gemac_reset(emac);

	emac_set_promiscuous(emac, 0);
	emac_set_broadcast(emac, 1);
	emac_set_unicast(emac, 1);
	emac_set_multicast(emac, 0);
	gemac_set_rx_fcs_remove(emac->gemac_base, 1);
	gemac_set_rx_cs_offload(emac->gemac_base, 1);

	gemac_start_tx(emac);
	gemac_start_rx(emac);

	if (emac->phy_addr >= 0) {
		if (!emac->phy_dev) {
			char phy_id[BUS_ID_SIZE];

			snprintf(phy_id, BUS_ID_SIZE, PHY_ID_FMT, 0, emac->phy_addr);

			emac->phy_dev = phy_connect(emac->net_dev,
				phy_id,
				gemac_adjust_link,
				0,
				gemac_get_phy_interface(emac->gemac_base));

			if (!IS_ERR(emac->phy_dev)) {
				emac->phy_dev->supported &= (SUPPORTED_10baseT_Half
					| SUPPORTED_10baseT_Full
					| SUPPORTED_100baseT_Half
					| SUPPORTED_100baseT_Full
					| SUPPORTED_1000baseT_Half
					| SUPPORTED_1000baseT_Full
					| SUPPORTED_Autoneg
					| SUPPORTED_MII);
				emac->phy_dev->advertising = emac->phy_dev->supported;
			} else {
				dev_err(emac->dev, "failed to connect to PHY\n");
				emac->phy_dev = NULL;
			}
		} else {
			/* we connected to PHY earlier, restart timer is enough */
			phy_start_machine(emac->phy_dev, NULL);
		}

		phy_start(emac->phy_dev);
	}
}

void emac_stop(struct emac *emac)
{
	if (emac->phy_dev) {
		phy_stop_machine(emac->phy_dev);
		phy_stop(emac->phy_dev);
	}

	gemac_halt_tx(emac);
	gemac_stop_rx(emac);
}

void emac_kick_rx(struct emac *emac)
{
	ulong flags;

	local_irq_save(flags);

	GEMAC_NET_CONTROL(emac->gemac_base) &= ~GEMAC_NET_CONTROL_RX_ENABLE;
	GEMAC_RX_CONTROL(emac->gemac_base) &= ~GEMAC_RX_CONTROL_DMAIF_ENABLE;
	GEMAC_HCSM_FIFO_CTRL(emac->gemac_base) |= GEMAC_HCSM_FIFO_CTRL_TXFFRES;
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	asm volatile ("nop");
	GEMAC_RX_CONTROL(emac->gemac_base) |= GEMAC_RX_CONTROL_DMAIF_ENABLE;

	local_irq_restore(flags);
}

void emac_start_rx(struct emac *emac)
{
	GEMAC_NET_CONTROL(emac->gemac_base) |= GEMAC_NET_CONTROL_RX_ENABLE;
}

int emac_check_rx_frame(struct emac *emac, u32 fstatus)
{
	if (fstatus & GEMAC_FSTATUS_RX_BAD)
		return -1;

	return 0;
}

int emac_check_tx_frame(struct emac *emac, u32 fstatus)
{
	if (fstatus & (GEMAC_FSTATUS_TX_ERR_RETRIES | GEMAC_FSTATUS_TX_LATE_COLLISION | GEMAC_FSTATUS_TX_FIFO_UNDERRUN))
		return -1;

	return 0;
}

void emac_csum_rx_frame(struct emac *emac, u32 fstatus, struct sk_buff *skb)
{
	if (fstatus & GEMAC_FSTATUS_RX_CS_IP) {
		if (fstatus & (GEMAC_FSTATUS_RX_CS_TCP | GEMAC_FSTATUS_RX_CS_UDP))
			skb->ip_summed = CHECKSUM_UNNECESSARY;
	} else
		skb->ip_summed = CHECKSUM_NONE;
}
