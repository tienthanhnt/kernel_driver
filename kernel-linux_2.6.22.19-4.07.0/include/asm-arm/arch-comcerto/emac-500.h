/*
 * include/asm-arm/arch-comcerto/emac.h
 *
 * (C) Copyright 2009
 * Mindspeed Technologies, Inc. <www.mindspeed.com>
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

#ifndef __ASM_ARCH_EMAC_H
#define __ASM_ARCH_EMAC_H

#include <linux/types.h>

#define EMAC_HCSM_FIFO_CTRL(base)	(*(volatile u32*)(((u32)(base)) + 0x0000))
#define EMAC_HCSM_TX_FIFO_HIGH(base)	(*(volatile u32*)(((u32)(base)) + 0x0018))
#define EMAC_HCSM_TX_FIFO_LOW(base)	(*(volatile u32*)(((u32)(base)) + 0x001C))
#define EMAC_HCSM_RX_FIFO_HIGH(base)	(*(volatile u32*)(((u32)(base)) + 0x0028))
#define EMAC_HCSM_RX_FIFO_LOW(base)	(*(volatile u32*)(((u32)(base)) + 0x002C))

#define EMAC_MAC_FIFO_CTRL(base)	(*(volatile u32*)(((u32)(base)) + 0xD000))
#define EMAC_MAC_TX_FIFO_HIGH(base)	(*(volatile u32*)(((u32)(base)) + 0xD014))
#define EMAC_MAC_TX_FIFO_LOW(base)	(*(volatile u32*)(((u32)(base)) + 0xD018))
#define EMAC_MAC_RX_FIFO_HIGH(base)	(*(volatile u32*)(((u32)(base)) + 0xD024))
#define EMAC_MAC_RX_FIFO_LOW(base)	(*(volatile u32*)(((u32)(base)) + 0xD028))

#define EMAC_OCR(base)			(*(volatile u32*)(((u32)(base)) + 0xE000))
#define EMAC_ICR(base)			(*(volatile u32*)(((u32)(base)) + 0xE004))
#define EMAC_ARC_MEMORY(base,offset)	(*(volatile u32*)(((u32)(base)) + 0xE200 + (u32)(offset)))
#define EMAC_MAC_CONTROL(base)		(*(volatile u32*)(((u32)(base)) + 0xE340))
#define EMAC_ARC_CONTROL(base)		(*(volatile u32*)(((u32)(base)) + 0xE344))
#define EMAC_TX_CONTROL(base)		(*(volatile u32*)(((u32)(base)) + 0xE348))
#define EMAC_RX_CONTROL(base)		(*(volatile u32*)(((u32)(base)) + 0xE350))
#define EMAC_MDIO_DATA(base)		(*(volatile u32*)(((u32)(base)) + 0xE358))
#define EMAC_MDIO_CONTROL(base)		(*(volatile u32*)(((u32)(base)) + 0xE35C))
#define EMAC_ARC_ENABLE(base)		(*(volatile u32*)(((u32)(base)) + 0xE368))

#define EMAC_HCSM_FIFO_CTRL_RXDREQWE	(1 <<  2)
#define EMAC_HCSM_FIFO_CTRL_TXDREQRE	(1 <<  3)
#define EMAC_HCSM_FIFO_CTRL_TXFF_RES	(1 << 12)

#define EMAC_MAC_FIFO_CTRL_TXFF_EN	(1 <<  0)
#define EMAC_MAC_FIFO_CTRL_HBTXRQ_EN	(1 <<  1)
#define EMAC_MAC_FIFO_CTRL_RXFF_EN	(1 <<  3)
#define EMAC_MAC_FIFO_CTRL_HBRXRQ_EN	(1 <<  4)
#define EMAC_MAC_FIFO_CTRL_TXCP_INH	(1 <<  5)

#define EMAC_OCR_MII_FD			(1 <<  4)
#define EMAC_OCR_MII_100		(1 <<  5)
#define EMAC_OCR_MII_LINK_UP		(1 <<  6)
#define EMAC_OCR_MII_CONN		(1 <<  7)
#define EMAC_OCR_CAM_HIT		(1 <<  9)
#define EMAC_OCR_LOOPBACK		(1 << 10)
#define EMAC_OCR_TOSS_INHIBIT		(1 << 11)
#define EMAC_OCR_RX_PACKET_SIZE_MASK	255
#define EMAC_OCR_RX_PACKET_SIZE_SHIFT	16
#define EMAC_OCR_SELECT_MII_MODE	(1 << 25)

#define EMAC_ICR_PHY_CONTROLS_LINK	(1 << 15)

#define EMAC_MDIO_CONTROL_WRITE		(1 << 10)
#define EMAC_MDIO_CONTROL_BUSY		(1 << 11)

#define EMAC_MAC_CONTROL_HALT_REQUEST	(1 <<  0)
#define EMAC_MAC_CONTROL_HALT		(1 <<  1)
#define EMAC_MAC_CONTROL_RESET		(1 <<  2)
#define EMAC_MAC_CONTROL_FD		(1 <<  3)
#define EMAC_MAC_CONTROL_LOOPBACK	(1 <<  4)
#define EMAC_MAC_CONTROL_CONN_MODE_MASK	(3 <<  5)
#define EMAC_MAC_CONTROL_CONN_MODE_AUTO (0 <<  5)
#define EMAC_MAC_CONTROL_CONN_MODE_10	(1 <<  5)
#define EMAC_MAC_CONTROL_CONN_MODE_MII	(2 <<  5)

#define EMAC_ARC_CONTROL_STATION_ACCEPT	(1 <<  0)
#define EMAC_ARC_CONTROL_GROUP_ACCEPT	(1 <<  1)
#define EMAC_ARC_CONTROL_BCAST_ACCEPT	(1 <<  2)
#define EMAC_ARC_CONTROL_NEGATIVE_ARC	(1 <<  3)
#define EMAC_ARC_CONTROL_COMPARE_ENABLE	(1 <<  4)

#define EMAC_TX_CONTROL_ENABLE		(1 <<  0)
#define EMAC_TX_CONTROL_HALT		(1 <<  1)
#define EMAC_TX_CONTROL_NO_PAD		(1 <<  2)
#define EMAC_TX_CONTROL_NO_CRC		(1 <<  3)
#define EMAC_TX_CONTROL_NO_EX_DEFERRAL	(1 <<  5)
#define EMAC_TX_CONTROL_SEND_PAUSE	(1 <<  6)

#define EMAC_RX_CONTROL_ENABLE		(1 <<  0)
#define EMAC_RX_CONTROL_HALT		(1 <<  1)
#define EMAC_RX_CONTROL_LONG_ENABLE	(1 <<  2)
#define EMAC_RX_CONTROL_SHORT_ENABLE	(1 <<  3)
#define EMAC_RX_CONTROL_STRIP_CRC	(1 <<  4)
#define EMAC_RX_CONTROL_PASS_CONTROL	(1 <<  5)
#define EMAC_RX_CONTROL_IGNORE_CRC	(1 <<  6)

#define EMAC_FSTATUS_RX_LENGTH_ERR	(1 <<  0)
#define EMAC_FSTATUS_RX_CONTROL		(1 <<  1)
#define EMAC_FSTATUS_RX_ALIGN_ERR	(1 <<  4)
#define EMAC_FSTATUS_RX_CRC_ERR		(1 <<  5)
#define EMAC_FSTATUS_RX_OVERFLOW	(1 <<  6)
#define EMAC_FSTATUS_RX_LONG_ERR	(1 <<  7)
#define EMAC_FSTATUS_RX_PARITY_ERR	(1 <<  9)
#define EMAC_FSTATUS_RX_GOOD		(1 << 10)
#define EMAC_FSTATUS_RX_HALTED		(1 << 11)
#define EMAC_FSTATUS_RX_MCAST		(1 << 13)
#define EMAC_FSTATUS_RX_BCAST		(1 << 14)
#define EMAC_FSTATUS_RX_VLAN		(1 << 15)
#define EMAC_FSTATUS_RX_PAUSE		(1 << 16)

#define EMAC_FSTATUS_TX_COLLISIONS	15
#define EMAC_FSTATUS_TX_EX_COLLISION	(1 <<  4)
#define EMAC_FSTATUS_TX_DEFERRED	(1 <<  5)
#define EMAC_FSTATUS_TX_PAUSED		(1 <<  6)
#define EMAC_FSTATUS_TX_UNDERUN		(1 <<  8)
#define EMAC_FSTATUS_TX_EX_DEFERRAL	(1 <<  9)
#define EMAC_FSTATUS_TX_LATE_COLLISION	(1 << 12)
#define EMAC_FSTATUS_TX_PARITY_ERR	(1 << 13)
#define EMAC_FSTATUS_TX_HALTED		(1 << 15)
#define EMAC_FSTATUS_TX_SQ_ERR		(1 << 16)	/* signal quality */
#define EMAC_FSTATUS_TX_MCAST		(1 << 17)
#define EMAC_FSTATUS_TX_BCAST		(1 << 18)
#define EMAC_FSTATUS_TX_VLAN		(1 << 19)
#define EMAC_FSTATUS_TX_CONTROL		(1 << 20)
#define EMAC_FSTATUS_TX_PAUSE		(1 << 21)
#define EMAC_FSTATUS_TX_HNR		(1 << 22)	/* host not responding was tranmitted before */

#define EMAC_FSTATUS_TX_BAD		(EMAC_FSTATUS_TX_EX_COLLISION	\
					|EMAC_FSTATUS_TX_UNDERUN	\
					|EMAC_FSTATUS_TX_EX_DEFERRAL	\
					|EMAC_FSTATUS_TX_LATE_COLLISION	\
					|EMAC_FSTATUS_TX_PARITY_ERR)

enum {
	EMAC_CONFIG_MODE_BOOT		= -1,	/* use mode selected by bootloader, e.g. don't touch anything
						 * at init phase (including speed and duplex)
						 */
	EMAC_CONFIG_MODE_MII		= 0,
	EMAC_CONFIG_MODE_RMII		= 1,
	EMAC_CONFIG_MODE_RMII_PHY	= 2,	/* the same as RMII, but link (speed & duplex, etc) is controlled by PHY */
};

enum {
	EMAC_CONFIG_SPEED_10		= 0,
	EMAC_CONFIG_SPEED_100		= 1,
};

enum {
	EMAC_CONFIG_DUPLEX_HALF		= 0,
	EMAC_CONFIG_DUPLEX_FULL		= 1,
};

struct comcerto_emac_platform_data {
	int	mode;		/* see EMAC_CONFIG_MODE_XXX */
	int	speed;		/* see EMAC_CONFIG_SPEED_XXX, ignored if ::mode == GEMAC_CONFIG_MODE_BOOT */
	int	duplex;		/* see EMAC_CONFIG_DUPLEX_XXX, ignored if ::mode == GEMAC_CONFIG_MODE_BOOT */	

	int	phy_addr;
};

struct comcerto_emac_mdio_platform_data {
	u32	phy_mask;	/* set 0xFFFFFFFF to disable PHY support */
};

#endif	/* __ASM_ARCH_EMAC_H */
