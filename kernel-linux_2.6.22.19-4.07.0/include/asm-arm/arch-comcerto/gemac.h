/*
 * include/asm-arm/arch-comcerto/gemac.h
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

#ifndef __ASM_ARCH_GEMAC_H
#define __ASM_ARCH_GEMAC_H

#include <linux/types.h>

#define GEMAC_HCSM_FIFO_CTRL(base)	(*(volatile u32*)(((u32)(base)) + 0x0000))
#define GEMAC_HCSM_RX_FIFO_HIGH(base)	(*(volatile u32*)(((u32)(base)) + 0x0028))
#define GEMAC_HCSM_RX_FIFO_LOW(base)	(*(volatile u32*)(((u32)(base)) + 0x002C))
#define GEMAC_GEM_FIFO_CTRL(base)	(*(volatile u32*)(((u32)(base)) + 0xD000))
#define GEMAC_GEM_RX_FIFO_HIGH(base)	(*(volatile u32*)(((u32)(base)) + 0xD024))
#define GEMAC_GEM_RX_FIFO_LOW(base)	(*(volatile u32*)(((u32)(base)) + 0xD028))
#define GEMAC_NET_CONTROL(base)		(*(volatile u32*)(((u32)(base)) + 0xE000))
#define GEMAC_NET_CONFIG(base)		(*(volatile u32*)(((u32)(base)) + 0xE004))
#define GEMAC_NET_STATUS(base)		(*(volatile u32*)(((u32)(base)) + 0xE008))
#define GEMAC_IRQ_STATUS(base)		(*(volatile u32*)(((u32)(base)) + 0xE024))
#define GEMAC_IRQ_DISABLE(base)		(*(volatile u32*)(((u32)(base)) + 0xE02C))
#define GEMAC_PHY_MAINTENANCE(base)	(*(volatile u32*)(((u32)(base)) + 0xE034))
#define GEMAC_SPEC_ADDR_BOTTOM(base,id)	(*(volatile u32*)(((u32)(base)) + 0xE088 + (id)*8))
#define GEMAC_SPEC_ADDR_TOP(base,id)	(*(volatile u32*)(((u32)(base)) + 0xE08C + (id)*8))
#define GEMAC_CONFIG(base)		(*(volatile u32*)(((u32)(base)) + 0xF000))
#define GEMAC_TX_CONTROL(base)		(*(volatile u32*)(((u32)(base)) + 0xF004))
#define GEMAC_RX_CONTROL(base)		(*(volatile u32*)(((u32)(base)) + 0xF010))
#define GEMAC_RX_STATUS_PACK_SIZE(base)	(*(volatile u32*)(((u32)(base)) + 0xF014))
#define GEMAC_PCS_CONTROL(base)		(*(volatile u32*)(((u32)(base)) + 0xE200))

#define GEMAC_HCSM_FIFO_CTRL_RXWRREQEN	(1 <<  2)
#define GEMAC_HCSM_FIFO_CTRL_TXRDREQEN	(1 <<  3)
#define GEMAC_HCSM_FIFO_CTRL_TXFFRES	(1 << 12)

#define GEMAC_GEM_FIFO_CTRL_TXFF_EN	(1 <<  0)
#define GEMAC_GEM_FIFO_CTRL_HBTXRQ_EN	(1 <<  1)
#define GEMAC_GEM_FIFO_CTRL_RXFF_EN	(1 <<  3)
#define GEMAC_GEM_FIFO_CTRL_HBRXRQ_EN	(1 <<  4)
#define GEMAC_GEM_FIFO_CTRL_TXCP_INH	(1 <<  5)

#define GEMAC_NET_CONTROL_RX_ENABLE	(1 <<  2)
#define GEMAC_NET_CONTROL_TX_ENABLE	(1 <<  3)
#define GEMAC_NET_CONTROL_MDIO_ENABLE	(1 <<  4)
#define GEMAC_NET_CONTROL_TX_HALT	(1 << 10)

#define GEMAC_NET_CONFIG_RX_CS_OFFLOAD	(1 << 24)
#define GEMAC_NET_CONFIG_MDC_CLK_MASK	7
#define GEMAC_NET_CONFIG_MDC_CLK_SHIFT	18
#define GEMAC_NET_CONFIG_RX_FCS_REMOVE	(1 << 17)
#define GEMAC_NET_CONFIG_PCS_SELECT	(1 << 11)
#define GEMAC_NET_CONFIG_RECEIVE_1536	(1 <<  8)
#define GEMAC_NET_CONFIG_UNICAST	(1 <<  7)
#define GEMAC_NET_CONFIG_MULTICAST	(1 <<  6)
#define GEMAC_NET_CONFIG_NO_BROADCAST	(1 <<  5)
#define GEMAC_NET_CONFIG_COPY_ALL	(1 <<  4)

#define GEMAC_NET_STATUS_PHY_IDLE	(1 <<  2)

#define GEMAC_PHY_MAINTENANCE_DATA_MASK	0xFFFF
#define GEMAC_PHY_MAINTENANCE_REG_MASK	0x1F
#define GEMAC_PHY_MAINTENANCE_REG_SHIFT	18
#define GEMAC_PHY_MAINTENANCE_PHY_MASK	0x1F
#define GEMAC_PHY_MAINTENANCE_PHY_SHIFT	23
#define GEMAC_PHY_MAINTENANCE_MAGIC	(2 << 16)
#define GEMAC_PHY_MAINTENANCE_WRITE	(1 << 28)
#define GEMAC_PHY_MAINTENANCE_READ	(2 << 28)
#define GEMAC_PHY_MAINTENANCE_CLAUSE22	(1 << 30)

#define GEMAC_PCS_CONTROL_ENABLE_ANEG	(1 << 12)

#define GEMAC_CONFIG_MODE_SEL		(1 <<  0)
#define GEMAC_CONFIG_MODE_MASK		7
#define GEMAC_CONFIG_MODE_GEM_SHIFT	1
#define GEMAC_CONFIG_MODE_PIN_SHIFT	4
#define GEMAC_CONFIG_MODE_MASK_GEM	(GEMAC_CONFIG_MODE_MASK << GEMAC_CONFIG_MODE_GEM_SHIFT)
#define GEMAC_CONFIG_MODE_MASK_PIN	(GEMAC_CONFIG_MODE_MASK << GEMAC_CONFIG_MODE_PIN_SHIFT)
#define GEMAC_CONFIG_DUPLEX_SEL		(1 <<  8)
#define GEMAC_CONFIG_DUPLEX_MASK	1
#define GEMAC_CONFIG_DUPLEX_GEM_SHIFT	9
#define GEMAC_CONFIG_DUPLEX_PHY_SHIFT	10
#define GEMAC_CONFIG_DUPLEX_MASK_GEM	(GEMAC_CONFIG_DUPLEX_MASK << GEMAC_CONFIG_DUPLEX_GEM_SHIFT)
#define GEMAC_CONFIG_DUPLEX_MASK_PHY	(GEMAC_CONFIG_DUPLEX_MASK << GEMAC_CONFIG_DUPLEX_PHY_SHIFT)
#define GEMAC_CONFIG_SPEED_SEL		(1 << 11)
#define GEMAC_CONFIG_SPEED_MASK		3
#define GEMAC_CONFIG_SPEED_GEM_SHIFT	12
#define GEMAC_CONFIG_SPEED_PHY_SHIFT	14
#define GEMAC_CONFIG_SPEED_MASK_GEM	(GEMAC_CONFIG_SPEED_MASK << GEMAC_CONFIG_SPEED_GEM_SHIFT)
#define GEMAC_CONFIG_SPEED_MASK_PHY	(GEMAC_CONFIG_SPEED_MASK << GEMAC_CONFIG_SPEED_PHY_SHIFT)
#define GEMAC_CONFIG_M2M_MASK		(0x7F << 18)
#define GEMAC_CONFIG_M2M_100		(1 << 19)
#define GEMAC_CONFIG_M2M_FD		(1 << 20)
#define GEMAC_CONFIG_M2M_LINK_UP	(1 << 21)
#define GEMAC_CONFIG_M2M_PRESET		(0x60 << 18)
#ifndef	CONFIG_ARCH_M823V2
#define GEMAC_CONFIG_REFCLK_SERDES	(1 << 26)
#endif

#define GEMAC_TX_CONTROL_DMAIF_ENABLE	(1 <<  0)
#define GEMAC_TX_CONTROL_CRC_ENABLE	(1 <<  1)
#define GEMAC_TX_CONTROL_RETR_ENABLE	(1 <<  2)
#ifdef	CONFIG_ARCH_M823V2
#define GEMAC_TX_CONTROL_RGMII_MASK	(3 <<  8)
#define GEMAC_TX_CONTROL_RGMII_3_3V	(0 <<  8)
#define GEMAC_TX_CONTROL_RGMII_2_5V	(1 <<  8)
#define GEMAC_TX_CONTROL_RGMII_BYPASS	(2 <<  8)
#endif

#define GEMAC_RX_CONTROL_DMAIF_ENABLE	(1 <<  0)

#define GEMAC_FSTATUS_RX_BAD		(1 <<  0)
#define GEMAC_FSTATUS_RX_CS_IP		(1 << 22)
#define GEMAC_FSTATUS_RX_CS_TCP		(1 << 23)
#define GEMAC_FSTATUS_RX_CS_UDP		(1 << 24)

#define GEMAC_FSTATUS_TX_ERR_RETRIES	(1 <<  0)
#define GEMAC_FSTATUS_TX_LATE_COLLISION	(1 <<  1)
#define GEMAC_FSTATUS_TX_FIFO_UNDERRUN	(1 <<  3)

enum {
	GEMAC_CONFIG_MODE_PIN	= -2,	/* use pin strapping */
	GEMAC_CONFIG_MODE_BOOT	= -1,	/* use mode selected by bootloader, e.g. don't touch anything
					 * at init phase (including speed and duplex)
					 */
	GEMAC_CONFIG_MODE_MII	= 0,
	GEMAC_CONFIG_MODE_GMII	= 1,
	GEMAC_CONFIG_MODE_RMII	= 2,
	GEMAC_CONFIG_MODE_RGMII	= 3,
#ifndef	CONFIG_ARCH_M823V2
	GEMAC_CONFIG_MODE_SGMII	= 4,
	GEMAC_CONFIG_MODE_TBI	= 7,
#else
	GEMAC_CONFIG_MODE_SMII	= 6,
#endif
};

enum {
	GEMAC_CONFIG_SPEED_PHY	= -1,	/* use speed reported by PHY */
	GEMAC_CONFIG_SPEED_10	= 0,
	GEMAC_CONFIG_SPEED_100	= 1,
	GEMAC_CONFIG_SPEED_1000	= 2,
};

enum {
	GEMAC_CONFIG_DUPLEX_PHY	= -1,	/* use duplex reported by PHY */
	GEMAC_CONFIG_DUPLEX_HALF= 0,
	GEMAC_CONFIG_DUPLEX_FULL= 1,
};

struct comcerto_gemac_platform_data {
	int	mode;		/* see GEMAC_CONFIG_MODE_XXX */
	int	speed;		/* see GEMAC_CONFIG_SPEED_XXX, ignored if ::mode == GEMAC_CONFIG_MODE_BOOT */
	int	duplex;		/* see GEMAC_CONFIG_DUPLEX_XXX, ignored if ::mode == GEMAC_CONFIG_MODE_BOOT */	

	int	phy_addr;
};

struct comcerto_gemac_mdio_platform_data {
	int	mdio_speed_hz;	/* must not be higher than 2.5MHz */
	u32	phy_mask;	/* set 0xFFFFFFFF to disable PHY support */
};

#endif	/* __ASM_ARCH_GEMAC_H */
