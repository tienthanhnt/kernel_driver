/*
 * arch/arm/mach-comcerto/board-megamombasa515.c
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
 */

#include <linux/autoconf.h>
#include <linux/sched.h>
#include <linux/device.h>
#include <linux/serial_8250.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/physmap.h>
#include <linux/spi/spi.h>

#include <asm/sizes.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/io.h>

#include <asm/mach/arch.h>

#include <asm/arch/hardware.h>
#include <asm/arch/irq.h>
#include <asm/arch/irqs.h>
#include <asm/arch/emac-500.h>
#include <asm/arch/gpio.h>

extern struct sys_timer comcerto_timer;
extern void __init comcerto515_map_io(void);
extern void __init comcerto515_fixup(struct machine_desc *desc, struct tag *unused, char **cmdline, struct meminfo *mi);
extern void __init comcerto515_irq_init(void);

#if defined(CONFIG_SPI_COMCERTO) || defined(CONFIG_SPI_COMCERTO_MODULE)
#define	SPI_ENABLED	1
#else
#undef	SPI_ENABLED
#endif

/* --------------------------------------------------------------------
 *  IRQ(s)
 * -------------------------------------------------------------------- */
static struct comcerto_irq_desc megamombasa515_irq_desc[] __initdata =
{
	{comcerto_gpio_to_irq(COMCERTO_GPIO_IRQ_SLIC),	comcerto_handle_gpio_level_irq},
	{comcerto_gpio_to_irq(COMCERTO_GPIO_IRQ_PCI),	comcerto_handle_gpio_level_irq},
};


static void __init megamombasa515_irq_init(void)
{
	/* common Comcerto 515 arch interrupts */
	comcerto515_irq_init();

	/* board-specific interrupts */
	comcerto_irq_init(megamombasa515_irq_desc, ARRAY_SIZE(megamombasa515_irq_desc));
}

/* --------------------------------------------------------------------
 *  GPIO(s)
 * -------------------------------------------------------------------- */
static struct comcerto_gpio_desc megamombasa515_gpio_desc[] __initdata =
{
#if defined(CONFIG_COMCERTO_MTD_NAND)
	{
		.gpio = COMCERTO_GPIO_IN_NAND_BR,
		.type = GPIO_INPUT,
	},
	{
		.gpio = COMCERTO_GPIO_OUT_NAND_CE,
		.type = GPIO_OUTPUT,
		.conf.out = {
			.default_value = 1,
		},
	},
	{
		.gpio = COMCERTO_GPIO_OUT_NAND_CLE,
		.type = GPIO_OUTPUT,
		.conf.out = {
			.default_value = 0,
		},
	},
	{
		.gpio = COMCERTO_GPIO_OUT_NAND_ALE,
		.type = GPIO_OUTPUT,
		.conf.out = {
			.default_value = 0,
		},
	},
#endif
	{
		.gpio = COMCERTO_GPIO_IRQ_SLIC,
		.type = GPIO_IRQ,
		.conf.irq = {
			.type = GPIO_IRQ_LEVEL,
			.trigger = GPIO_LEVEL_LOW,
		},
	},
	{
		.gpio = COMCERTO_GPIO_IRQ_PCI,
		.type = GPIO_IRQ,
		.conf.irq = {
			.type = GPIO_IRQ_LEVEL,
			.trigger = GPIO_LEVEL_LOW,
		},
	},
};

static void __init megamombasa515_gpio_init(void)
{
	comcerto_gpio_init(megamombasa515_gpio_desc, ARRAY_SIZE(megamombasa515_gpio_desc));

#ifdef CONFIG_COMCERTO_UART0_SUPPORT
	comcerto_gpio_ctrl(0x3 << 12, 0x3 << 12);
#else
	comcerto_gpio_ctrl(0x0 << 12, 0x3 << 12);
#endif

#ifdef CONFIG_COMCERTO_UART1_SUPPORT
	comcerto_gpio_ctrl(0x3 << 14, 0x3 << 14);
#else
	comcerto_gpio_ctrl(0x0 << 14, 0x3 << 14);
#endif

#ifdef SPI_ENABLED
	comcerto_gpio_ctrl(0x3 <<  4, 0x3 <<  4);
#else
	comcerto_gpio_ctrl(0x0 <<  4, 0x3 <<  4);
#endif
}

/* --------------------------------------------------------------------
 *  NOR device
 * -------------------------------------------------------------------- */
#ifdef CONFIG_COMCERTO_MTD_NOR
static struct mtd_partition megamombasa515_nor_parts[] = {
	{
		.name	= "nor boot",
		.size	= 192 * SZ_1K,
		.offset	= 0,
	},
	{
		.name	= "nor boot env",
		.size	= 64 * SZ_1K,
		.offset	= MTDPART_OFS_APPEND,
	},
	{
		.name	= "nor csp",
		.size	= SZ_2M - 256 * SZ_1K,
		.offset	= MTDPART_OFS_APPEND,
	},
	{
		.name	= "nor msp",
		.size	= SZ_2M,
		.offset	= MTDPART_OFS_APPEND,
	},
};


static struct resource megamombasa515_nor_resources[] = {
	{
		.start	= COMCERTO_NOR16_BASE,
		.end	= COMCERTO_NOR16_BASE + SZ_4M - 1,
		.flags	= IORESOURCE_MEM,
	},
};


static struct physmap_flash_data megamombasa515_nor_data = {
	.width		= 2,
	.nr_parts	= ARRAY_SIZE(megamombasa515_nor_parts),
	.parts		= megamombasa515_nor_parts,
};


static struct platform_device megamombasa515_nor = {
	.name			= "physmap-flash",
	.id			= 0,
	.num_resources		= ARRAY_SIZE(megamombasa515_nor_resources),
	.resource		= megamombasa515_nor_resources,
	.dev = {
		.platform_data	= &megamombasa515_nor_data,
	},
};
#endif

/* --------------------------------------------------------------------
 *  NAND device
 * -------------------------------------------------------------------- */
#ifdef CONFIG_COMCERTO_MTD_NAND
static struct mtd_partition megamombasa515_nand_parts[] = {
	{
		.name	= "nand csp",
		.size	= SZ_2M,
		.offset	= 0,
	},
	{
		.name	= "nand msp",
		.size	= SZ_2M,
		.offset	= MTDPART_OFS_APPEND,
	},
	{
		.name	= "nand fs",
		.size	= 252 * SZ_1M,
		.offset	= MTDPART_OFS_APPEND,
	},
};


static struct resource megamombasa515_nand_resources[] = {
	{
		.start	= COMCERTO_NAND_BASE,
		.end	= COMCERTO_NAND_BASE + 0x200 - 1,
		.flags	= IORESOURCE_MEM,
	},
};


#ifdef CONFIG_MTD_PARTITIONS
static const char *megamombasa515_part_probes[] = { "cmdlinepart", NULL };
#endif

#define COMCERTO_NAND_CE_MASK		comcerto_gpio_mask(COMCERTO_GPIO_OUT_NAND_CE)
#define COMCERTO_NAND_ALE_MASK		comcerto_gpio_mask(COMCERTO_GPIO_OUT_NAND_ALE)
#define COMCERTO_NAND_CLE_MASK		comcerto_gpio_mask(COMCERTO_GPIO_OUT_NAND_CLE)
#define COMCERTO_NAND_CMD_MASK		(COMCERTO_NAND_CE_MASK | COMCERTO_NAND_ALE_MASK | COMCERTO_NAND_CLE_MASK)

static void megamombasa515_nand_cmd_ctrl(struct mtd_info *mtd, int cmd, unsigned int ctrl)
{
	if (ctrl & NAND_CTRL_CHANGE) {
		unsigned int bits;

		bits = comcerto_gpio_read_output() & ~COMCERTO_NAND_CMD_MASK;

		if (~ctrl & NAND_NCE)
			bits |= COMCERTO_NAND_CE_MASK;

		if (ctrl & NAND_ALE)
			bits |= COMCERTO_NAND_ALE_MASK;

		if (ctrl & NAND_CLE)
			bits |= COMCERTO_NAND_CLE_MASK;

		comcerto_gpio_write_output(bits);
	}

	if (cmd != NAND_CMD_NONE) {
		struct nand_chip *chip = mtd->priv;

		writeb(cmd, chip->IO_ADDR_W);
	}
}

static int megamombasa515_nand_dev_ready(struct mtd_info *mtd)
{
	return gpio_get_value(COMCERTO_GPIO_IN_NAND_BR);
}


static struct platform_nand_data megamombasa515_nand_data = {
	.chip = {
		.nr_chips = 1,
		.chip_offset = 0,
		.nr_partitions = ARRAY_SIZE(megamombasa515_nand_parts),
		.partitions = megamombasa515_nand_parts,
		.chip_delay = 20,
#ifdef CONFIG_MTD_PARTITIONS
		.part_probe_types = megamombasa515_part_probes,
#endif
	},
	.ctrl = {
		.hwcontrol = NULL,
		.dev_ready = megamombasa515_nand_dev_ready,
		.select_chip = NULL,
		.cmd_ctrl = megamombasa515_nand_cmd_ctrl,
	},
};

static struct platform_device megamombasa515_nand = {
	.name			= "gen_nand",
	.id			= 0,
	.num_resources		= ARRAY_SIZE(megamombasa515_nand_resources),
	.resource		= megamombasa515_nand_resources,
	.dev = {
		.platform_data	= &megamombasa515_nand_data,
	},
};
#endif

/* --------------------------------------------------------------------
 *  SPI bus controller
 * -------------------------------------------------------------------- */
#ifdef SPI_ENABLED
static struct resource megamombasa515_spi_resources[] = {
	{
		.start	= COMCERTO_SPI0_BASE,
		.end	= COMCERTO_SPI0_BASE + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= IRQ_SPI,
		.flags	= IORESOURCE_IRQ,
	},
};


static struct platform_device megamombasa515_spi = {
	.name = "comcerto_spi",
	.id = 0,
	.num_resources = ARRAY_SIZE(megamombasa515_spi_resources),
	.resource = megamombasa515_spi_resources,
};

struct legerity_platform_data {
	int	type;
	int	dummy;
};

static struct legerity_platform_data megamombasa515_legerity0_platform_data = {
	.type = 4,
	.dummy = 0,
};

static struct spi_board_info megamombasa515_legerity_spi_info = {
	.modalias = "legerity",
	.chip_select = 1,
	.max_speed_hz = 4*1000*1000,
	.bus_num = 0,
	.irq = -1,
	.mode = SPI_MODE_3,
	.platform_data = &megamombasa515_legerity0_platform_data,
};
#endif

/* --------------------------------------------------------------------
 *  Serial interface
 * -------------------------------------------------------------------- */
#if defined(CONFIG_COMCERTO_UART0_SUPPORT) || defined(CONFIG_COMCERTO_UART1_SUPPORT)
static struct plat_serial8250_port megamombasa515_uart_data[] = {
#ifdef CONFIG_COMCERTO_UART0_SUPPORT
	{
		.mapbase	= COMCERTO_UART0_BASE,
		.membase	= (void *)APB_VADDR(COMCERTO_UART0_BASE),
		.irq		= IRQ_UART0,
		.flags		= UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,
		.iotype		= UPIO_MEM,
		.regshift	= 2,
		.uartclk	= COMCERTO_AHBCLK_HZ,
	},
#endif
#ifdef CONFIG_COMCERTO_UART1_SUPPORT
	{
		.mapbase	= COMCERTO_UART1_BASE,
		.membase	= (void *)APB_VADDR(COMCERTO_UART1_BASE),
		.irq		= IRQ_UART1,
		.flags		= UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,
		.iotype		= UPIO_MEM,
		.regshift	= 2,
		.uartclk	= COMCERTO_AHBCLK_HZ,
	},
#endif
	{
		.flags		= 0,
	},
};

static struct platform_device megamombasa515_uart = {
	.name			= "serial8250",
	.id			= PLAT8250_DEV_PLATFORM,
	.dev = {
		.platform_data	= megamombasa515_uart_data,
	},
};
#endif

/* --------------------------------------------------------------------
 *  EMAC0 - physical ethernet interface (eth0)
 *
 *  Interface is controlled by firmware in master mode. EMAC can be
 *  used by kernel only if no firmware is used at all or firmware is
 *  stopped.
 * -------------------------------------------------------------------- */
#if defined(CONFIG_CSP_ARM0) || defined(CONFIG_COMCERTO_ETH)
static struct comcerto_emac_platform_data megamombasa515_emac0_data = {
	.mode		= EMAC_CONFIG_MODE_BOOT,
	.speed		= EMAC_CONFIG_SPEED_100,
	.duplex		= EMAC_CONFIG_DUPLEX_FULL,

	.phy_addr	= -1,	/* 0, negative value disables PHY support */
};

static struct resource megamombasa515_eth0_resources[] = {
	{
		.name   = "emac",
		.start  = COMCERTO_EMAC0_BASE,
		.end    = COMCERTO_EMAC0_BASE + SZ_64K -1,
		.flags  = IORESOURCE_MEM,
	},
	{
		.name   = "edma",
		.start  = COMCERTO_EDMA_EMAC0_BASE,
		.end    = COMCERTO_EDMA_EMAC0_BASE + 0x1FF,
		.flags  = IORESOURCE_MEM,
	},
	{
		.name	= "rx",
		.start	= IRQ_EDMA0RX,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name	= "tx",
		.start	= IRQ_EDMA0TX,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device megamombasa515_eth0 = {
	.name		= "comcerto-eth",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(megamombasa515_eth0_resources),
	.resource	= megamombasa515_eth0_resources,
	.dev = {
		.platform_data = &megamombasa515_emac0_data,
	},
};
#endif

/* --------------------------------------------------------------------
 *  MDIO interface
 * -------------------------------------------------------------------- */
#if defined(CONFIG_CSP_ARM0) || defined(CONFIG_COMCERTO_ETH)
static struct comcerto_emac_mdio_platform_data megamombasa515_mdio_data = {
	.phy_mask	= 0xFFFFFFFE,
};

static struct resource megamombasa515_mdio_resources[] = {
	{
		.name   = "emac",
		.start  = COMCERTO_EMAC0_BASE,
		.end    = COMCERTO_EMAC0_BASE + SZ_64K -1,
		.flags  = IORESOURCE_MEM,
	},
};

static struct platform_device megamombasa515_mdio = {
	.name		= "comcerto-mdio",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(megamombasa515_mdio_resources),
	.resource	= megamombasa515_mdio_resources,
	.dev = {
		.platform_data = &megamombasa515_mdio_data,
	},
};
#endif

static struct platform_device *megamombasa515_devices[] __initdata = {
#if defined(CONFIG_COMCERTO_UART0_SUPPORT) || defined(CONFIG_COMCERTO_UART1_SUPPORT)
	&megamombasa515_uart,
#endif
#ifdef CONFIG_COMCERTO_MTD_NAND
	&megamombasa515_nand,
#endif
#ifdef CONFIG_COMCERTO_MTD_NOR
	&megamombasa515_nor,
#endif
#ifdef SPI_ENABLED
	&megamombasa515_spi,
#endif
#if defined(CONFIG_CSP_ARM0) || defined(CONFIG_COMCERTO_ETH)
	&megamombasa515_eth0,
	&megamombasa515_mdio,
#endif
};

static int __init megamombasa515_init(void)
{
	megamombasa515_gpio_init();

	platform_add_devices(megamombasa515_devices, ARRAY_SIZE(megamombasa515_devices));

#ifdef SPI_ENABLED
	spi_register_board_info(&megamombasa515_legerity_spi_info, 1);
#endif

	return 0;
}


arch_initcall(megamombasa515_init);


MACHINE_START(COMCERTO, "Megamombasa Comcerto 515")
	/* Mindspeed Technologies Inc. */
	.phys_io        = APB_MEMORY_PHY,
	.io_pg_offst    = ((APB_MEMORY_VADDR)>>18)&0xfffc,
	.boot_params	= COMCERTO_SDRAM_BASE + 0x100,
	.fixup		= comcerto515_fixup,
	.map_io		= comcerto515_map_io,
	.init_irq	= megamombasa515_irq_init,
	.timer		= &comcerto_timer,
MACHINE_END
