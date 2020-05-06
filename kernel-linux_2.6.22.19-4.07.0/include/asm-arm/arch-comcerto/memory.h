/*
 * include/asm-arm/arch-comcerto/memory.h
 *
 * Copyright (C) 2004,2005 Mindspeed Technologies, Inc.
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

#ifndef __ASM_ARCH_MEMORY_H
#define __ASM_ARCH_MEMORY_H

#include <linux/autoconf.h>

#ifdef CONFIG_EVM_C300V2

/* 512M of SDRAM is available on C300_V2, we need to map it into extended memory */
#ifdef CONFIG_MSP_CARRIER
	#define COMCERTO_SDRAM_BASE	0x81000000
#else
	#define COMCERTO_SDRAM_BASE	0x80000000
#endif

#else

#ifdef CONFIG_MSP_CARRIER
	#define COMCERTO_SDRAM_BASE	0x01000000
#else
	#define COMCERTO_SDRAM_BASE	0x00000000
#endif

#endif

#define PHYS_OFFSET			COMCERTO_SDRAM_BASE

#define COMCERTO_PCIDMA_SIZE		(512*1024*1024)
#define COMCERTO_PCIDMA_SIZE_MASK	(~(COMCERTO_PCIDMA_SIZE - 1))
#define COMCERTO_PCIDMA_SYS_BASE_ADDR	0x80000000	/* Must cover all physical memory and be aligned on PCIDMA_SIZE */
#define COMCERTO_PCIDMA_PCI_BASE_ADDR	0xA0000000	/* Use ARM physical addresses on the PCI bus */


#define __virt_to_bus(x)		__virt_to_phys(x)
#define __bus_to_virt(x)		__phys_to_virt(x)

#define virt_to_pci_bus(x)		(__virt_to_phys(x) - COMCERTO_PCIDMA_SYS_BASE_ADDR + COMCERTO_PCIDMA_PCI_BASE_ADDR)
#define pci_bus_to_virt(x)		(__phys_to_virt(x) - COMCERTO_PCIDMA_PCI_BASE_ADDR + COMCERTO_PCIDMA_SYS_BASE_ADDR)
#define is_pci_device(dev)		(dev && (strncmp(dev->bus->name, "pci", 3) == 0))

#define __arch_page_to_dma(dev, page)	({is_pci_device(dev) ?				\
		(dma_addr_t)virt_to_pci_bus((unsigned long)page_address(page)) :	\
		(dma_addr_t)__virt_to_bus((unsigned long)page_address(page));})

#define __arch_dma_to_virt(dev, addr)	({is_pci_device(dev) ?				\
		(void *)pci_bus_to_virt(addr) :						\
		(void *)__bus_to_virt(addr);})

#define __arch_virt_to_dma(dev, addr)	({is_pci_device(dev) ?				\
		(dma_addr_t)virt_to_pci_bus((unsigned long)addr) :			\
		(dma_addr_t)__virt_to_bus((unsigned long)addr);})

#endif /* __ASM_ARCH_MEMORY_H */
