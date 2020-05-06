/*
 * linux/drivers/net/comcerto/emac.c
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
 *
 */

#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/phy.h>
#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/arch/emac-500.h>

#include "emac.h"

struct emac {
	void	*emac_base;
	int	mode;
	int	duplex;
	int	speed;
	int	link;
	int	phy_addr;

	struct device		*dev;
	struct net_device	*net_dev;
	struct phy_device	*phy_dev;
};


static void emac_adjust_link(struct net_device *net_dev)
{
	struct emac *emac = (struct emac*) net_dev->base_addr;	/* must be set in upper eth code */
	int changed = 0;
	u32 phy_controls_link = EMAC_ICR(emac->emac_base) & EMAC_ICR_PHY_CONTROLS_LINK;

	if (emac->speed >= 0 && emac->phy_dev->speed != emac->speed) {
		changed++;

		switch (emac->phy_dev->speed) {
		case SPEED_10:
		case SPEED_100:
			emac->speed = emac->phy_dev->speed;
			if (!phy_controls_link) {
				if (emac->phy_dev->speed == SPEED_10)
					EMAC_OCR(emac->emac_base) &= ~EMAC_OCR_MII_100;
				else
					EMAC_OCR(emac->emac_base) |= EMAC_OCR_MII_100;
			}
			break;
		default:
			dev_warn(&net_dev->dev, "invalid speed reported by PHY: %d\n", emac->phy_dev->speed);
			changed--;
		}
	}

	if (emac->duplex >= 0 && emac->phy_dev->duplex != emac->duplex) {
		changed++;

		emac->duplex = emac->phy_dev->duplex;
		if (!phy_controls_link) {
			if (emac->duplex == DUPLEX_HALF)
				EMAC_OCR(emac->emac_base) &= ~EMAC_OCR_MII_FD;
			else
				EMAC_OCR(emac->emac_base) |= EMAC_OCR_MII_FD;
		}
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

static inline void emac_halt_tx(struct emac *emac)
{
	EMAC_TX_CONTROL(emac->emac_base) |= EMAC_TX_CONTROL_HALT;	/* wait until frame will be transmitted */
}

static inline void emac_stop_tx(struct emac *emac)
{
	EMAC_TX_CONTROL(emac->emac_base) &= ~EMAC_TX_CONTROL_ENABLE;
}

static inline void emac_start_tx(struct emac *emac)
{
	EMAC_TX_CONTROL(emac->emac_base) &= ~EMAC_TX_CONTROL_HALT;
	EMAC_TX_CONTROL(emac->emac_base) |= EMAC_TX_CONTROL_ENABLE;
}

static inline void emac_stop_rx(struct emac *emac)
{
	EMAC_RX_CONTROL(emac->emac_base) &= ~EMAC_RX_CONTROL_ENABLE;
}

void emac_start_rx(struct emac *emac)
{
	EMAC_RX_CONTROL(emac->emac_base) |= EMAC_RX_CONTROL_ENABLE;
}

static phy_interface_t emac_get_phy_interface(void *emac_base)
{
	if (EMAC_OCR(emac_base) & EMAC_OCR_SELECT_MII_MODE)
		return PHY_INTERFACE_MODE_MII;

	return PHY_INTERFACE_MODE_RMII;
}

void emac_start(struct emac *emac)
{
	u32 ocr;

	if (emac->mode != EMAC_CONFIG_MODE_BOOT) {
		ocr = (0x20 << EMAC_OCR_RX_PACKET_SIZE_SHIFT) | EMAC_OCR_CAM_HIT | EMAC_OCR_MII_CONN | EMAC_OCR_MII_LINK_UP | 8;
		if (emac->mode != EMAC_CONFIG_MODE_RMII_PHY) {
			if (emac->speed == SPEED_100)
				ocr |= EMAC_OCR_MII_100;
			if (emac->duplex == DUPLEX_FULL)
				ocr |= EMAC_OCR_MII_FD;
			EMAC_ICR(emac->emac_base) &= ~EMAC_ICR_PHY_CONTROLS_LINK;
		} else
			EMAC_ICR(emac->emac_base) |= EMAC_ICR_PHY_CONTROLS_LINK;

		if (emac->mode == EMAC_CONFIG_MODE_MII)
			ocr |= EMAC_OCR_SELECT_MII_MODE;
		else
			ocr &= ~EMAC_OCR_SELECT_MII_MODE;

		EMAC_OCR(emac->emac_base) = ocr;

		EMAC_MAC_FIFO_CTRL(emac->emac_base) = EMAC_MAC_FIFO_CTRL_TXFF_EN | EMAC_MAC_FIFO_CTRL_HBTXRQ_EN
			| EMAC_MAC_FIFO_CTRL_RXFF_EN | EMAC_MAC_FIFO_CTRL_HBRXRQ_EN;
		EMAC_MAC_TX_FIFO_HIGH(emac->emac_base) = 0xF3;
		EMAC_MAC_TX_FIFO_LOW(emac->emac_base) = 0xF0;
		EMAC_MAC_RX_FIFO_HIGH(emac->emac_base) = 0x19;
		EMAC_MAC_RX_FIFO_LOW(emac->emac_base) = 0x18;
	
		EMAC_HCSM_TX_FIFO_HIGH(emac->emac_base) = 0xC0;
		EMAC_HCSM_TX_FIFO_LOW(emac->emac_base) = 0x20;
		EMAC_HCSM_RX_FIFO_HIGH(emac->emac_base) = 0xE0;
		EMAC_HCSM_RX_FIFO_LOW(emac->emac_base) = 0x40;
		EMAC_HCSM_FIFO_CTRL(emac->emac_base) = EMAC_HCSM_FIFO_CTRL_RXDREQWE | EMAC_HCSM_FIFO_CTRL_TXDREQRE;

		EMAC_MAC_CONTROL(emac->emac_base) = EMAC_MAC_CONTROL_CONN_MODE_MII | EMAC_MAC_CONTROL_FD | EMAC_MAC_CONTROL_RESET;
	
		while (EMAC_MAC_CONTROL(emac->emac_base) & EMAC_MAC_CONTROL_RESET)
			;

		emac_set_promiscuous(emac, 0);
	} else {
		EMAC_MAC_FIFO_CTRL(emac->emac_base) = EMAC_MAC_FIFO_CTRL_TXFF_EN | EMAC_MAC_FIFO_CTRL_HBTXRQ_EN
			| EMAC_MAC_FIFO_CTRL_RXFF_EN | EMAC_MAC_FIFO_CTRL_HBRXRQ_EN;

		EMAC_HCSM_FIFO_CTRL(emac->emac_base) = EMAC_HCSM_FIFO_CTRL_RXDREQWE | EMAC_HCSM_FIFO_CTRL_TXDREQRE;

		EMAC_MAC_CONTROL(emac->emac_base) &= ~(EMAC_MAC_CONTROL_RESET | EMAC_MAC_CONTROL_HALT | EMAC_MAC_CONTROL_HALT_REQUEST);
		while (EMAC_MAC_CONTROL(emac->emac_base) & EMAC_MAC_CONTROL_RESET)
			;
	}

	EMAC_ARC_CONTROL(emac->emac_base) = EMAC_ARC_CONTROL_BCAST_ACCEPT | EMAC_ARC_CONTROL_STATION_ACCEPT;

	emac_start_tx(emac);
	emac_start_rx(emac);

	if (emac->phy_addr >= 0) {
		if (!emac->phy_dev) {
			char phy_id[BUS_ID_SIZE];

			snprintf(phy_id, BUS_ID_SIZE, PHY_ID_FMT, 0, emac->phy_addr);

			emac->phy_dev = phy_connect(emac->net_dev,
				phy_id,
				emac_adjust_link,
				0,
				emac_get_phy_interface(emac->emac_base));

			if (!IS_ERR(emac->phy_dev)) {
				emac->phy_dev->supported &=
					  SUPPORTED_10baseT_Half | SUPPORTED_10baseT_Full
					| SUPPORTED_100baseT_Half | SUPPORTED_100baseT_Full
					| SUPPORTED_Autoneg | SUPPORTED_MII;
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

	emac_halt_tx(emac);
	emac_stop_rx(emac);
}

int emac_get_promiscuous(struct emac *emac)
{
	u32 arc = EMAC_ARC_CONTROL(emac->emac_base) & (EMAC_ARC_CONTROL_COMPARE_ENABLE | EMAC_ARC_CONTROL_NEGATIVE_ARC);
	u32 ocr = EMAC_OCR(emac->emac_base) & EMAC_ARC_CONTROL_COMPARE_ENABLE;

	if (arc == EMAC_ARC_CONTROL_NEGATIVE_ARC && ocr == EMAC_ARC_CONTROL_COMPARE_ENABLE)
		return 1;

	return 0;
}

void emac_set_promiscuous(struct emac *emac, int promiscuous)
{
	u32 arc = EMAC_ARC_CONTROL(emac->emac_base);
	u32 ocr = EMAC_OCR(emac->emac_base);

	arc &= ~(EMAC_ARC_CONTROL_COMPARE_ENABLE | EMAC_ARC_CONTROL_NEGATIVE_ARC);
	ocr &= ~EMAC_OCR_TOSS_INHIBIT;
	if (promiscuous) {
		arc |= EMAC_ARC_CONTROL_NEGATIVE_ARC;
		ocr |= EMAC_OCR_TOSS_INHIBIT;
	} else {
		arc |= EMAC_ARC_CONTROL_COMPARE_ENABLE;
	}

	EMAC_ARC_CONTROL(emac->emac_base) = arc;
	EMAC_OCR(emac->emac_base) = ocr;
}

int emac_check_rx_frame(struct emac *emac, u32 fstatus)
{
	if (fstatus & EMAC_FSTATUS_RX_GOOD)
		return 0;

	return -1;
}

int emac_check_tx_frame(struct emac *emac, u32 fstatus)
{
	if (fstatus & EMAC_FSTATUS_TX_BAD)
		return -1;

	return 0;
}

int emac_set_mac_addr(struct emac *emac, int id, u8 *mac_addr)
{
	u32 u0, u1;

	if (id > 20)
		return -EINVAL;

	u0 = EMAC_ARC_MEMORY(emac->emac_base, id*6);
	u1 = EMAC_ARC_MEMORY(emac->emac_base, id*6+4);

	if (id&1) {
		u0 &= 0xFFFF0000;
		u0 |= (mac_addr[1] << 8) | mac_addr[0];
		u1 = (mac_addr[5] << 24) | (mac_addr[4] << 16) | (mac_addr[3] << 8) | mac_addr[2];
	} else {
		u0 = (mac_addr[3] << 24) | (mac_addr[2] << 16) | (mac_addr[1] << 8) | mac_addr[0];
		u1 &= 0x0000FFFF;
		u0 |= (mac_addr[5] << 24) | mac_addr[4];
	}

	EMAC_ARC_MEMORY(emac->emac_base, id*6) = u0;
	EMAC_ARC_MEMORY(emac->emac_base, id*6+4) = u1;

	EMAC_ARC_ENABLE(emac->emac_base) |= 1 << id;

	return 0;
}

struct emac *emac_open(struct net_device *net_dev, struct platform_device *pdev, void *emac_base)
{
	struct emac *emac;
	struct device *dev = &pdev->dev;
	struct comcerto_emac_platform_data *platform = dev->platform_data;

	emac = kzalloc(sizeof(*emac), GFP_KERNEL);
	if (!emac)
		return NULL;

	emac->emac_base = emac_base;
	emac->dev = dev;
	emac->net_dev = net_dev;
	emac->phy_addr = platform->phy_addr;

	emac->mode = platform->mode;
	emac->speed = platform->speed == EMAC_CONFIG_SPEED_10 ? SPEED_10 : SPEED_100;
	emac->duplex = platform->duplex == EMAC_CONFIG_DUPLEX_HALF ? DUPLEX_HALF : DUPLEX_FULL;

	emac->link = 0;		/* start with link off */

	return emac;
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
