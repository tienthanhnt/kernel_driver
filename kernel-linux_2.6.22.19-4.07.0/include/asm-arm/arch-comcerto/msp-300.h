/*
 *  linux/include/asm-arm/arch-comcerto/msp-300.h
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
#ifndef __ASM_ARCH_COMCERTO_MSP_300_H
#define __ASM_ARCH_COMCERTO_MSP_300_H


#define SMI_CONFIG_VADDR		((void *)(ARAM_MEMORY_VADDR))
#define CBL_CONFIG_VADDR		((void *)(ARAM_MEMORY_VADDR + 0x1250))

static inline void *msp_sdram_to_virt(ulong phys_addr)
{
	return (void*)(phys_addr - SDRAM_MSP_MEMORY_PHY + SDRAM_MSP_MEMORY_VADDR);
}

static inline ulong msp_virt_to_sdram(void *virt_addr)
{
	return (ulong)virt_addr - SDRAM_MSP_MEMORY_VADDR + SDRAM_MSP_MEMORY_PHY;
}

static inline void *msp_aram_to_virt(ulong phys_addr)
{
	return (void*)(phys_addr - ARAM_MEMORY_PHY + ARAM_MEMORY_VADDR);
}

static inline ulong msp_virt_to_aram(void *virt_addr)
{
	return (ulong)virt_addr - ARAM_MEMORY_VADDR + ARAM_MEMORY_PHY;
}

static inline void *msp_eram_to_virt(ulong phys_addr)
{
	return (void*)(phys_addr - ERAM_MEMORY_PHY + ERAM_MEMORY_VADDR);
}

static inline ulong msp_virt_to_eram(void *virt_addr)
{
	return (ulong)virt_addr - ERAM_MEMORY_VADDR + ERAM_MEMORY_PHY;
}

#ifdef CONFIG_DEBUG_VM
/* full check to make sure address is inside of the given region */
#define CHECK_ADDR(addr, start, len)	((ulong)(addr) >= (start) && (ulong)(addr) < (start) + (len))
#else
/* relaxed check, in this case the order of checks does matter in functions below */
#define CHECK_ADDR(addr, start, len)	((ulong)(addr) < (start) + (len))
#endif

static inline void *msp_phys_to_virt(ulong phys_addr)
{
	if (CHECK_ADDR(phys_addr, SDRAM_MSP_MEMORY_PHY, SDRAM_MSP_MEMORY_SIZE))
		return msp_sdram_to_virt(phys_addr);
	if (CHECK_ADDR(phys_addr, ERAM_MEMORY_PHY, ERAM_MEMORY_SIZE))
		return msp_eram_to_virt(phys_addr);
	if (CHECK_ADDR(phys_addr, ARAM_MEMORY_PHY, ARAM_MEMORY_SIZE))
		return msp_aram_to_virt(phys_addr);

	return (void*)0;
}

static inline ulong msp_virt_to_phys(void *virt_addr)
{
	if (CHECK_ADDR(virt_addr, SDRAM_MSP_MEMORY_VADDR, SDRAM_MSP_MEMORY_SIZE))
		return msp_virt_to_sdram(virt_addr);
	if (CHECK_ADDR(virt_addr, ERAM_MEMORY_VADDR, ERAM_MEMORY_SIZE))
		return msp_virt_to_eram(virt_addr);
	if (CHECK_ADDR(virt_addr, ARAM_MEMORY_VADDR, ARAM_MEMORY_SIZE))
		return msp_virt_to_aram(virt_addr);

	return (ulong)-1;
}

#endif /* __ASM_ARCH_COMCERTO_MSP_300_H */
