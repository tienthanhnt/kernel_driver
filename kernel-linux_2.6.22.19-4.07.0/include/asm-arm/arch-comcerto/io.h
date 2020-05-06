/*
 *  linux/include/asm-arm/arch-comcerto/io.h
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
#ifndef __ASM_ARM_ARCH_IO_H
#define __ASM_ARM_ARCH_IO_H

#include <asm/io.h>
#include <asm/arch/hardware.h>

#define IO_SPACE_LIMIT          0x2000ffff

#if !defined(CONFIG_PCI)

#define __io(a)			((void __iomem *)(a))
#define __mem_pci(a)	(a)

#else

#define __comcerto_io(a)	(a)
#define __comcerto_mem_pci(a)	(a)

extern u8 comcerto_pci_readb(u32 addr, u8 space_type);
extern u16 comcerto_pci_readw(u32 addr, u8 space_type);
extern u32 comcerto_pci_readl(u32 addr, u8 space_type);

extern void comcerto_pci_writeb(u32 addr, u8 value, u8 space_type);
extern void comcerto_pci_writew(u32 addr, u16 value, u8 space_type);
extern void comcerto_pci_writel(u32 addr, u32 value, u8 space_type);

#define COMCERTO_PCI_IO_BASE	PCIBIOS_MIN_IO
#define COMCERTO_PCI_IO_SIZE	(IO_SPACE_LIMIT - PCIBIOS_MIN_IO)

#define COMCERTO_PCI_MEM_BASE	PCIBIOS_MIN_MEM
#define COMCERTO_PCI_MEM_SIZE	0x01ffffff

#define COMCERTO_PCI_MEM_VBASE	0xe3000000


#define PCI_SPACE_TYPE_CONFIG_TYPE0	0x1
#define PCI_SPACE_TYPE_CONFIG_TYPE1	0x2
#define PCI_SPACE_TYPE_IO		0x3
#define PCI_SPACE_TYPE_MEM		0x4

#define IS_PCI_MEM_PADDR(addr)	(((unsigned long)(addr) >= PCIBIOS_MIN_MEM) && ((unsigned long)(addr) <= (PCIBIOS_MIN_MEM + COMCERTO_PCI_MEM_SIZE)))
#define IS_PCI_MEM_VADDR(addr)	(((unsigned long)(addr) >= COMCERTO_PCI_MEM_VBASE) && ((unsigned long)(addr) <= (COMCERTO_PCI_MEM_VBASE + COMCERTO_PCI_MEM_SIZE)))

#define __iomem_to_pci(virt)	((unsigned long)(virt) - COMCERTO_PCI_MEM_VBASE + COMCERTO_PCI_MEM_BASE)
#define __pci_to_iomem(phy)	((phy) - COMCERTO_PCI_MEM_BASE + COMCERTO_PCI_MEM_VBASE)

static inline void __comcerto_writesb(void __iomem *addr, const void *data, int bytelen)
{
	unsigned char * datap = (unsigned char *)data;  
	if (IS_PCI_MEM_VADDR(addr))
		while (bytelen--)
			comcerto_pci_writeb(__iomem_to_pci(addr), *(datap)++, PCI_SPACE_TYPE_MEM);
	else
		__raw_writesb(addr, data, bytelen);
}

static inline void __comcerto_writesw(void __iomem *addr, const void *data, int wordlen)
{
	unsigned short * datap = (unsigned short *)data;  
	if (IS_PCI_MEM_VADDR(addr))
		while (wordlen--)
			comcerto_pci_writew(__iomem_to_pci(addr), *(datap)++, PCI_SPACE_TYPE_MEM);
	else
		__raw_writesw(addr, data, wordlen);
}

static inline void __comcerto_writesl(void __iomem *addr, const void *data, int longlen)
{
	unsigned int * datap = (unsigned int *)data;  
	if (IS_PCI_MEM_VADDR(addr))
		while (longlen--)
			comcerto_pci_writel(__iomem_to_pci(addr), *(datap)++, PCI_SPACE_TYPE_MEM);
	else
		__raw_writesl(addr, data, longlen);
}

static inline void __comcerto_readsb(void __iomem *addr, void *data, int bytelen)
{
	unsigned char * datap = (unsigned char *)data;  
	if (IS_PCI_MEM_VADDR(addr))
		while (bytelen--)
			*(datap)++ = comcerto_pci_readb(__iomem_to_pci(addr), PCI_SPACE_TYPE_MEM);
	else
		__raw_readsb(addr, data, bytelen);
}

static inline void __comcerto_readsw(void __iomem *addr, void *data, int wordlen)
{
	unsigned short * datap = (unsigned short *)data;  
	if (IS_PCI_MEM_VADDR(addr))
		while (wordlen--)
			*(datap)++ = comcerto_pci_readw(__iomem_to_pci(addr), PCI_SPACE_TYPE_MEM);
	else
		__raw_readsw(addr, data, wordlen);
}

static inline void __comcerto_readsl(void __iomem *addr, void *data, int longlen)
{
	unsigned int * datap = (unsigned int *)data;  
	if (IS_PCI_MEM_VADDR(addr))
		while (longlen--)
			*(datap)++ = comcerto_pci_readl(__iomem_to_pci(addr), PCI_SPACE_TYPE_MEM);
	else
		__raw_readsl(addr, data, longlen);
}

#define __comcerto_writeb(v,a)	(IS_PCI_MEM_VADDR(a) ? comcerto_pci_writeb(__iomem_to_pci(a), (v), PCI_SPACE_TYPE_MEM) : __raw_writeb((v), (a)))
#define __comcerto_writew(v,a)	(IS_PCI_MEM_VADDR(a) ? comcerto_pci_writew(__iomem_to_pci(a), (v), PCI_SPACE_TYPE_MEM) : __raw_writew((v), (a)))
#define __comcerto_writel(v,a)	(IS_PCI_MEM_VADDR(a) ? comcerto_pci_writel(__iomem_to_pci(a), (v), PCI_SPACE_TYPE_MEM) : __raw_writel((v), (a)))
#define __comcerto_readb(a)		(IS_PCI_MEM_VADDR(a) ? comcerto_pci_readb(__iomem_to_pci(a), PCI_SPACE_TYPE_MEM) : __raw_readb(a))
#define __comcerto_readw(a)		(IS_PCI_MEM_VADDR(a) ? comcerto_pci_readw(__iomem_to_pci(a), PCI_SPACE_TYPE_MEM) : __raw_readw(a))
#define __comcerto_readl(a)		(IS_PCI_MEM_VADDR(a) ? comcerto_pci_readl(__iomem_to_pci(a), PCI_SPACE_TYPE_MEM) : __raw_readl(a))


#define readb(c) ({ unsigned int __v = __comcerto_readb(__comcerto_mem_pci(c)); __v; })
#define readw(c) ({ unsigned int __v = le16_to_cpu(__comcerto_readw(__comcerto_mem_pci(c))); __v; })
#define readl(c) ({ unsigned int __v = le32_to_cpu(__comcerto_readl(__comcerto_mem_pci(c))); __v; })
#define readb_relaxed(addr) readb(addr)
#define readw_relaxed(addr) readw(addr)
#define readl_relaxed(addr) readl(addr)

#define readsb(p,d,l)		__comcerto_readsb(__comcerto_mem_pci(p),d,l)
#define readsw(p,d,l)		__comcerto_readsw(__comcerto_mem_pci(p),d,l)
#define readsl(p,d,l)		__comcerto_readsl(__comcerto_mem_pci(p),d,l)

#define writeb(v,c)		__comcerto_writeb(v,__comcerto_mem_pci(c))
#define writew(v,c)		__comcerto_writew(cpu_to_le16(v),__comcerto_mem_pci(c))
#define writel(v,c)		__comcerto_writel(cpu_to_le32(v),__comcerto_mem_pci(c))

#define writesb(p,d,l)		__comcerto_writesb(__comcerto_mem_pci(p),d,l)
#define writesw(p,d,l)		__comcerto_writesw(__comcerto_mem_pci(p),d,l)
#define writesl(p,d,l)		__comcerto_writesl(__comcerto_mem_pci(p),d,l)


#define __ioport_to_pci(phy)	(phy)

static inline void __comcerto_outsb(unsigned int port, const void *data, int bytelen)
{
	unsigned char * datap = (unsigned char *)data;  
	while (bytelen--)
		comcerto_pci_writeb(__ioport_to_pci(port), *(datap)++, PCI_SPACE_TYPE_IO);
}

static inline void __comcerto_outsw(unsigned int port, const void *data, int wordlen)
{
	unsigned short * datap = (unsigned short *)data;  
	while (wordlen--)
		comcerto_pci_writew(__ioport_to_pci(port), *(datap)++, PCI_SPACE_TYPE_IO);
}

static inline void __comcerto_outsl(unsigned int port, const void *data, int longlen)
{
	unsigned int * datap = (unsigned int *)data;  
	while (longlen--)
		comcerto_pci_writel(__ioport_to_pci(port), *(datap)++, PCI_SPACE_TYPE_IO);
}

static inline void __comcerto_insb(unsigned int port, void *data, int bytelen)
{
	unsigned char * datap = (unsigned char *)data;  
	while (bytelen--)
		*(datap)++ = comcerto_pci_readb(__ioport_to_pci(port), PCI_SPACE_TYPE_IO);
}

static inline void __comcerto_insw(unsigned int port, void *data, int wordlen)
{
	unsigned short * datap = (unsigned short *)data;  
	while (wordlen--)
		*(datap)++ = comcerto_pci_readw(__ioport_to_pci(port), PCI_SPACE_TYPE_IO);
}

static inline void __comcerto_insl(unsigned int port, void *data, int longlen)
{
	unsigned int * datap = (unsigned int *)data;  
	while (longlen--)
		*(datap)++ = comcerto_pci_readl(__ioport_to_pci(port), PCI_SPACE_TYPE_IO);
}

#define __comcerto_outb(v,a)	(comcerto_pci_writeb(__ioport_to_pci(a), (v), PCI_SPACE_TYPE_IO))
#define __comcerto_outw(v,a)	(comcerto_pci_writew(__ioport_to_pci(a), (v), PCI_SPACE_TYPE_IO))
#define __comcerto_outl(v,a)	(comcerto_pci_writel(__ioport_to_pci(a), (v), PCI_SPACE_TYPE_IO))
#define __comcerto_inb(a)	(comcerto_pci_readb(__ioport_to_pci(a), PCI_SPACE_TYPE_IO))
#define __comcerto_inw(a)	(comcerto_pci_readw(__ioport_to_pci(a), PCI_SPACE_TYPE_IO))
#define __comcerto_inl(a)	(comcerto_pci_readl(__ioport_to_pci(a), PCI_SPACE_TYPE_IO))

#define outb(v,p)		__comcerto_outb(v,__comcerto_io(p))
#define outw(v,p)		__comcerto_outw(cpu_to_le16(v),__comcerto_io(p))
#define outl(v,p)		__comcerto_outl(cpu_to_le32(v),__comcerto_io(p))
#define inb(p)	({ __u8 __v = __comcerto_inb(__comcerto_io(p)); __v; })
#define inw(p)	({ __u16 __v = le16_to_cpu(__comcerto_inw(__comcerto_io(p))); __v; })
#define inl(p)	({ __u32 __v = le32_to_cpu(__comcerto_inl(__comcerto_io(p))); __v; })

#define outsb(p,d,l)		__comcerto_outsb(__comcerto_io(p),d,l)
#define outsw(p,d,l)		__comcerto_outsw(__comcerto_io(p),d,l)
#define outsl(p,d,l)		__comcerto_outsl(__comcerto_io(p),d,l)
#define insb(p,d,l)		__comcerto_insb(__comcerto_io(p),d,l)
#define insw(p,d,l)		__comcerto_insw(__comcerto_io(p),d,l)
#define insl(p,d,l)		__comcerto_insl(__comcerto_io(p),d,l)

#define	ioread8(p)		__comcerto_readb(p)
#define	ioread16(p)		__comcerto_readw(p)
#define	ioread32(p)		__comcerto_readl(p)

#define	ioread8_rep(p, v, c)	__comcerto_readsb(p, v, c)
#define	ioread16_rep(p, v, c)	__comcerto_readsw(p, v, c)
#define	ioread32_rep(p, v, c)	__comcerto_readsl(p, v, c)

#define	iowrite8(v,p)		__comcerto_writeb(v,p)
#define	iowrite16(v,p)		__comcerto_writew(v,p)
#define	iowrite32(v,p)		__comcerto_writel(v,p)

#define	iowrite8_rep(p, v, c)	__comcerto_writesb(p, v, c)
#define	iowrite16_rep(p, v, c)	__comcerto_writesw(p, v, c)
#define	iowrite32_rep(p, v, c)	__comcerto_writesl(p, v, c)

#define	ioport_map(port, nr)	((void __iomem*)__comcerto_io(port))
#define	ioport_unmap(addr)

#define memset_io(c,v,l)	_memset_io(__comcerto_mem_pci(c),(v),(l))
#define memcpy_fromio(a,c,l)	_memcpy_fromio((a),__comcerto_mem_pci(c),(l))
#define memcpy_toio(c,a,l)	_memcpy_toio(__comcerto_mem_pci(c),(a),(l))

#define __arch_ioremap(c,s,a)	__comcerto_ioremap((c),(s),(a))
#define __arch_iounmap(a)	__comcerto_iounmap(a)

static inline void __iomem *__comcerto_ioremap(unsigned long cookie, size_t size, unsigned long align)
{
	if (IS_PCI_MEM_PADDR(cookie))
		return (void *)__pci_to_iomem(cookie);
	else
		return __arm_ioremap(cookie, size, align);
}

static inline void __comcerto_iounmap(void __iomem *addr)
{
	if (!IS_PCI_MEM_VADDR(addr))
		__iounmap(addr);
}

#endif /* !defined(CONFIG_PCI) */

#endif /* __ASM_ARM_ARCH_IO_H */
