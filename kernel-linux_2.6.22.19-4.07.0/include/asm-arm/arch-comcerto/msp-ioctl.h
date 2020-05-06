/*
 *  linux/arch/arm/mach-comcerto/msp-ioctl.h
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

#ifndef __MSP_IOCTL_H
#define __MSP_IOCTL_H

#ifdef __KERNEL__
#include <linux/ioctl.h>
#else
#include <sys/ioctl.h>
#endif


#define MSP_MAJOR		0
#define MSP_MINOR		1
#define MSP_VERSION(maj, min)	(((maj) << 16) | (min))
#define MSP_VERSION_MAJOR(v)	((v) >> 16)
#define MSP_VERSION_MINOR(v)	((v) & 0xffff)

/* default node name */
#define MSP_DEVICE_NAME		"/dev/msp"

/* make sure major doesn't conflict with another devices */
#define MSP_DEVICE_MAJOR_NUM	237

struct msp_region
{
	int		index;		/* in * must be filled by user-level, starting from 0 */
	char		name[16];	/* out* associated name (SDRAM, IRAM, ERAM, etc) */
	unsigned long	phys_addr;	/* out* physical address of region */
	unsigned long	virt_addr;	/* out* kernel virtual address */
	unsigned int	size;		/* out* size in bytes */
};

struct msp_memzero_region
{
	unsigned long	phys_addr;	/* in * physical address of region to clear */
	unsigned int	size;		/* in * size of region in bytes */
};

#define MSP_IOCTL_MAGIC		'm'
/* get module version */
#define MSP_IOCTL_VERSION	_IOR (MSP_IOCTL_MAGIC, 1, unsigned int*)
/* get info on a given MSP memory area (for coredump) */
#define MSP_IOCTL_REGION	_IOWR(MSP_IOCTL_MAGIC, 2, struct msp_region*)
/* 0 - release from reset, 1 - reset, anything else - get reset state */
#define MSP_IOCTL_RESET		_IOWR(MSP_IOCTL_MAGIC, 3, int*)
/* 0 - release from reset, 1 - reset, anything else - get reset state */
#define MSP_IOCTL_MEMZERO	_IOW (MSP_IOCTL_MAGIC, 4, struct msp_memzero_region*)

#endif /* __MSP_IOCTL_H */
