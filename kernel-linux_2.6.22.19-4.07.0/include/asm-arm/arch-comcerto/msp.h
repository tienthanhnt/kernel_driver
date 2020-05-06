/*
 *  linux/include/asm-arm/arch-comcerto/msp.h
 *
 *  Copyright (C) 2004,2005 Mindspeed Technologies, Inc.
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
#ifndef __ASM_ARCH_COMCERTO_MSP_H
#define __ASM_ARCH_COMCERTO_MSP_H

#include <linux/autoconf.h>
#include <linux/types.h>
#include <asm/arch/irq.h>
#include <asm/arch/irqs.h>

#if defined(CONFIG_ARCH_M823XX)
	#include "msp-300.h"
#elif defined(CONFIG_ARCH_M825XX1) || defined(CONFIG_ARCH_M825XX2) || defined(CONFIG_ARCH_M828XX)
	#include "msp-500.h"
#elif
	#error "Unsupported CPU"
#endif

#define CBL_MAGIC_NUM			0x98765432
#define CBL_FLAGS_ETHADDR_MASK		(1 <<  0)
#define CBL_FLAGS_ALERT_ADDR_MASK	(1 <<  8)

#define CBL_M2_VADDR			(CBL_CONFIG_VADDR + 0x0001)
#define CBL_M4_VADDR			(CBL_CONFIG_VADDR + 0x0007)
#define CBL_MAGIC_NUM_VADDR		(CBL_CONFIG_VADDR + 0x0010)
#define CBL_FLAGS_VADDR			(CBL_CONFIG_VADDR + 0x0014)
#define CBL_M1_VADDR			(CBL_CONFIG_VADDR + 0x0018)
#define CBL_M5_VADDR			(CBL_CONFIG_VADDR + 0x0024)

/* SMI parameters, stored in the same part of embedded memory */
#define SMI_CSME_PART_VADDR		(SMI_CONFIG_VADDR + 0x0100)
#define SMI_ETH_PART_VADDR		(SMI_CONFIG_VADDR + 0x0104)
#define SMI_CSP2MSP_QUEUE_VADDR		(SMI_CONFIG_VADDR + 0x0108)
#define SMI_MSP2CSP_QUEUE_VADDR		(SMI_CONFIG_VADDR + 0x010C)
#define SMI_NCNB_START_VADDR		(SMI_CONFIG_VADDR + 0x0110)
#define SMI_ALERT_VADDR			(SMI_CONFIG_VADDR + 0x0114)

void msp_timer0_udelay(unsigned int us);

int msp_reset(u32 timeout_ms);

/* two functions provided by VED driver, we use them in MSP reset/start sequence */
#ifdef CONFIG_COMCERTO_VED
int ved_stop(void);
int ved_start(void);
int ved_wait_msp(void);
#else
static inline int ved_stop(void)	{ return 0; }
static inline int ved_start(void)	{ return 0; }
static inline int ved_wait_msp(void)	{ return 0; }
#endif
#ifdef CONFIG_COMCERTO_ETH
void comcerto_eth_start(void);
void comcerto_eth_start_eth2(void);
void comcerto_eth_stop(void);
#else
static inline void comcerto_eth_start(void)	{ }
static inline void comcerto_eth_start_eth2(void){ }
static inline void comcerto_eth_stop(void)	{ }
#endif

static inline int msp_controls_eth0(void)
{
	if (__raw_readl(COMCERTO_INTC_ARM0_IRQMASK0) & comcerto_irq_mask(IRQ_EDMA0RX))
		return 1;

	return 0;
}

#if defined(CONFIG_ARCH_M823XX)
static inline int msp_controls_eth2(void)
{
	if (__raw_readl(COMCERTO_INTC_ARM0_IRQMASK0) & comcerto_irq_mask(IRQ_EDMA1RX))
		return 1;

	return 0;
}
#else
static inline int msp_controls_eth2(void)
{
	return 0;
}
#endif

#ifndef CONFIG_ARCH_M825XX1
#define	memcpy_ncnb(dst, src, len)	memcpy(dst, src, len)
#define memzero_ncnb(dst, len)		memset(dst, 0, len)
#else
extern void memcpy_ncnb(void *dst, const void *src, unsigned int len);
extern void memzero_ncnb(void *dst, unsigned int len);
#endif

#endif /* __ASM_ARCH_COMCERTO_MSP_H */
