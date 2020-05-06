/*
 *  linux/drivers/net/comcerto/emac.h
 *
 *  Copyright (C) 2009 Mindspeed Technologies, Inc.
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

#ifndef __EMAC_H
#define __EMAC_H

#include <linux/interrupt.h>
#include <linux/netdevice.h>
#include <linux/platform_device.h>
#include <linux/skbuff.h>

struct emac	*emac_open(struct net_device *net_dev, struct platform_device *pdev, void *emac_base);
int		emac_close(struct emac *emac);

int		emac_check_rx_frame(struct emac *emac, u32 fstatus);
int		emac_check_tx_frame(struct emac *emac, u32 fstatus);
#if defined(CONFIG_ARCH_M823XX)
void		emac_csum_rx_frame(struct emac *emac, u32 fstatus, struct sk_buff *skb);
#else
static inline void emac_csum_rx_frame(struct emac *emac, u32 fstatus, struct sk_buff *skb) {};
#endif

void		emac_start(struct emac *emac);
void		emac_stop(struct emac *emac);
#if defined(CONFIG_ARCH_M823XX)
void		emac_kick_rx(struct emac *emac);
#else
static inline void emac_kick_rx(struct emac *emac) {};
#endif
void		emac_start_rx(struct emac *emac);

int		emac_get_promiscuous(struct emac *emac);
void		emac_set_promiscuous(struct emac *emac, int promiscuous);

int		emac_get_broadcast(struct emac *emac);
void		emac_set_broadcast(struct emac *emac, int broadcast);

int		emac_get_mac_addr(struct emac *emac, int id, u8 *mac_addr);
int		emac_set_mac_addr(struct emac *emac, int id, u8 *mac_addr);

#endif
