/*
 * arch/arm/mach-comcerto/board-c300v2evm.c
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
#include <linux/i2c.h>

#include <asm/sizes.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/io.h>

#include <asm/mach/arch.h>

#include <asm/arch/hardware.h>
#include <asm/arch/irq.h>
#include <asm/arch/irqs.h>
#include <asm/arch/gemac.h>
#include <asm/arch/gpio.h>
#include <asm/arch/wdt.h>

extern struct sys_timer comcerto_timer;
extern void __init comcerto300_map_io(void);
extern void __init comcerto300_fixup(struct machine_desc *desc, struct tag *unused, char **cmdline, struct meminfo *mi);
extern void __init comcerto300_irq_init(void);

#if defined(CONFIG_SPI_COMCERTO) || defined(CONFIG_SPI_COMCERTO_MODULE)
#define	SPI_ENABLED	1
#else
#undef	SPI_ENABLED
#endif

/* --------------------------------------------------------------------
 *  IRQ(s)
 * -------------------------------------------------------------------- */
static struct comcerto_irq_desc c300v2evm_irq_desc[] __initdata =
{
	{comcerto_gpio_to_irq(COMCERTO_GPIO_IRQ_SLIC0),	comcerto_handle_secondary_level_irq},
	{comcerto_gpio_to_irq(COMCERTO_GPIO_IRQ_MUX),	comcerto_handle_secondary_level_irq},
};


static void __init c300v2evm_irq_init(void)
{
	/* common Comcerto 300 arch interrupts */
	comcerto300_irq_init();

	/* board-specific interrupts */
	comcerto_irq_init(c300v2evm_irq_desc, ARRAY_SIZE(c300v2evm_irq_desc));
}

/* --------------------------------------------------------------------
 *  GPIO(s)
 * -------------------------------------------------------------------- */
static struct comcerto_gpio_desc c300v2evm_gpio_desc[] __initdata =
{
	{	/* make sure that peripheral stuff is out of reset */
		.gpio = COMCERTO_GPIO_OUT_PERIPH_RESET,
		.type = GPIO_OUTPUT,
		.conf.out = {
			.default_value = 1
		},
	},

#if defined(CONFIG_MTD_NAND_COMCERTO)
	{
		.gpio = COMCERTO_GPIO_IN_NAND_BR,
		.type = GPIO_INPUT,
	},
#endif

	{
		.gpio = COMCERTO_GPIO_IRQ_SLIC0,
		.type = GPIO_IRQ,
		.conf.irq = {
			.type = GPIO_IRQ_LEVEL,
			.trigger = GPIO_LEVEL_LOW
		},
	},

#if defined(CONFIG_PCI)
	{
		.gpio = COMCERTO_GPIO_IRQ_PCI_INTA,
		.type = GPIO_IRQ,
		.conf.irq = {
			.type = GPIO_IRQ_LEVEL,
			.trigger = GPIO_LEVEL_LOW
		},
	},
#endif
};

static void __init c300v2evm_gpio_init(void)
{
	comcerto_gpio_init(c300v2evm_gpio_desc, ARRAY_SIZE(c300v2evm_gpio_desc));

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
	comcerto_gpio_ctrl(0x3 << 28, 0x3 << 28);	/* enable SPI1 bus */
	comcerto_gpio_ctrl(0x3 <<  4, 0x3 <<  4);	/* enable SPI0 bus */
#else
	comcerto_gpio_ctrl(0x0 << 28, 0x3 << 28);	/* disable SPI1 bus */
	comcerto_gpio_ctrl(0x0 <<  4, 0x3 <<  4);	/* disable SPI0 bus */
#endif

#ifdef CONFIG_I2C_COMCERTO
	comcerto_gpio_ctrl(0x3 <<  0, 0x3 <<  0);
#else
	comcerto_gpio_ctrl(0x0 <<  0, 0x3 <<  0);
#endif
}

/* --------------------------------------------------------------------
 *  NOR device
 * -------------------------------------------------------------------- */
#ifdef CONFIG_COMCERTO_MTD_NOR
static struct mtd_partition c300v2evm_nor_parts[] = {
	{
		.name	= "boot",
		.size	= 256*1024,
		.offset	= 0,
	},
	{
		.name	= "csp",
		.size	= 1792*1024,
		.offset	= MTDPART_OFS_APPEND,
	},
	{
		.name	= "msp",
		.size	= 4*SZ_1M,
		.offset	= MTDPART_OFS_APPEND,
	},
	{
		.name	= "fs",
		.size	= 58*SZ_1M,
		.offset	= MTDPART_OFS_APPEND,
	},
};

static struct resource c300v2evm_nor_resources[] = {
	{
		.start	= COMCERTO_NOR16_BASE,
		.end	= COMCERTO_NOR16_BASE + SZ_64M - 1,
		.flags	= IORESOURCE_MEM,
	},
};


static struct physmap_flash_data c300v2evm_nor_data = {
	.width		= 2,
	.nr_parts	= ARRAY_SIZE(c300v2evm_nor_parts),
	.parts		= c300v2evm_nor_parts,
};


static struct platform_device c300v2evm_nor = {
	.name			= "physmap-flash",
	.id			= 0,
	.num_resources		= ARRAY_SIZE(c300v2evm_nor_resources),
	.resource		= c300v2evm_nor_resources,
	.dev = {
		.platform_data	= &c300v2evm_nor_data,
	},
};
#endif

/* --------------------------------------------------------------------
 *  NAND device
 * -------------------------------------------------------------------- */
#ifdef CONFIG_COMCERTO_MTD_NAND
static struct mtd_partition c300v2evm_nand_parts[] = {
	{
		.name	= "csp",
		.size	= 2 * SZ_1M,
		.offset	= 0,
	},
	{
		.name	= "msp",
		.size	= 4*SZ_1M,
		.offset	= MTDPART_OFS_APPEND,
	},
	{
		.name	= "spare",
		.size	= 218*SZ_1M,
		.offset	= MTDPART_OFS_APPEND,
	},
	{
		.name	= "fs",
		.size	= 32*SZ_1M,
		.offset	= MTDPART_OFS_APPEND,
	},
};

struct resource c300v2evm_nand_resources[] = {
	{
		.start	= COMCERTO_NAND_BASE,
		.end	= COMCERTO_NAND_BASE + 0x200 - 1,
		.flags	= IORESOURCE_MEM,
	},
};


#ifdef CONFIG_MTD_PARTITIONS
static const char *c300v2evm_part_probes[] = { "cmdlinepart", NULL };
#endif


static void c300v2evm_nand_cmd_ctrl(struct mtd_info *mtd, int cmd, unsigned int ctrl)
{
	if (ctrl & NAND_CTRL_CHANGE) {
		unsigned int bits = __raw_readl(COMCERTO_UEXP_DA_CONTROL) & ~(UEXP_DA_CONTROL_ALE| UEXP_DA_CONTROL_CLE);

		if (ctrl & NAND_ALE)
			bits |= UEXP_DA_CONTROL_ALE;

		if (ctrl & NAND_CLE)
			bits |= UEXP_DA_CONTROL_CLE;

		__raw_writel(bits, COMCERTO_UEXP_DA_CONTROL);
	}

	if (cmd != NAND_CMD_NONE) {
		struct nand_chip *chip = mtd->priv;

		writeb(cmd, chip->IO_ADDR_W);
	}
}

static int c300v2evm_nand_dev_ready(struct mtd_info *mtd)
{
	return gpio_get_value(COMCERTO_GPIO_IN_NAND_BR);
}


static struct platform_nand_data c300v2evm_nand_data = {
	.chip = {
		.nr_chips = 1,
		.chip_offset = 0,
		.nr_partitions = ARRAY_SIZE(c300v2evm_nand_parts),
		.partitions = c300v2evm_nand_parts,
		.chip_delay = 20,
#ifdef CONFIG_MTD_PARTITIONS
		.part_probe_types = c300v2evm_part_probes,
#endif
	},
	.ctrl = {
		.hwcontrol = NULL,
		.dev_ready = c300v2evm_nand_dev_ready,
		.select_chip = NULL,
		.cmd_ctrl = c300v2evm_nand_cmd_ctrl,
	},
};

static struct platform_device c300v2evm_nand = {
	.name			= "gen_nand",
	.id			= 0,
	.num_resources		= ARRAY_SIZE(c300v2evm_nand_resources),
	.resource		= c300v2evm_nand_resources,
	.dev = {
		.platform_data	= &c300v2evm_nand_data,
	},
};
#endif

/* --------------------------------------------------------------------
 *  SPI bus controller
 * -------------------------------------------------------------------- */
#ifdef SPI_ENABLED
static struct resource c300v2evm_spi0_resources[] = {
	{
		.start	= COMCERTO_SPI0_BASE,
		.end	= COMCERTO_SPI0_BASE + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= IRQ_SPI0,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct resource c300v2evm_spi1_resources[] = {
	{
		.start	= COMCERTO_SPI1_BASE,
		.end	= COMCERTO_SPI1_BASE + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= IRQ_SPI1,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device c300v2evm_spi0 = {
	.name = "comcerto_spi",
	.id = 0,
	.num_resources = ARRAY_SIZE(c300v2evm_spi0_resources),
	.resource = c300v2evm_spi0_resources,
};

static struct platform_device c300v2evm_spi1 = {
	.name = "comcerto_spi",
	.id = 1,
	.num_resources = ARRAY_SIZE(c300v2evm_spi1_resources),
	.resource = c300v2evm_spi1_resources,
};

struct legerity_platform_data {
	int	type;
	int	dummy;
};

static struct legerity_platform_data c300v2evm_legerity0_platform_data = {
	.type = 5,
	.dummy = 0,
};

static struct legerity_platform_data c300v2evm_legerity1_platform_data = {
	.type = 5,
	.dummy = 0,
};

static struct spi_board_info c300v2evm_legerity_spi_info[] = {
	{
		.modalias = "legerity",
		.chip_select = 6,
		.max_speed_hz = 4*1000*1000,
		.bus_num = 1,
		.irq = -1,
		.mode = SPI_MODE_3,
		.platform_data = &c300v2evm_legerity0_platform_data,
	},
	{
		.modalias = "legerity",
		.chip_select = 7,
		.max_speed_hz = 4*1000*1000,
		.bus_num = 1,
		.irq = -1,
		.mode = SPI_MODE_3,
		.platform_data = &c300v2evm_legerity1_platform_data,
	},
};

static struct spi_board_info spi0_info[] = {
	{
		.modalias = "spi0",
		.chip_select = 1,
		.max_speed_hz = 1000*1000,
		.bus_num = 0,
		.irq = -1,
		.mode = SPI_MODE_3,
		.platform_data = &c300v2evm_legerity0_platform_data,
	},
};

#endif

/* --------------------------------------------------------------------
 *  Serial interface
 * -------------------------------------------------------------------- */
#if defined(CONFIG_COMCERTO_UART0_SUPPORT) || defined(CONFIG_COMCERTO_UART1_SUPPORT)
static struct plat_serial8250_port c300v2evm_uart_data[] = {
#ifdef CONFIG_COMCERTO_UART0_SUPPORT
	{
		.mapbase	= COMCERTO_UART0_BASE,
		.membase	= (void *)APB_VADDR(COMCERTO_UART0_BASE),
		.irq		= IRQ_UART0,
		.flags		= UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,
		.iotype		= UPIO_DWAPB,
		.regshift	= 2,
		.private_data	= (void *)COMCERTO_UART0_USR,
	},
#endif
#ifdef CONFIG_COMCERTO_UART1_SUPPORT
	{
		.mapbase	= COMCERTO_UART1_BASE,
		.membase	= (void *)APB_VADDR(COMCERTO_UART1_BASE),
		.irq		= IRQ_UART1,
		.flags		= UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,
		.iotype		= UPIO_DWAPB,
		.regshift	= 2,
		.private_data	= (void *)COMCERTO_UART1_USR,
	},
#endif
	{
		.flags		= 0,
	},
};

static struct platform_device c300v2evm_uart = {
	.name			= "serial8250",
	.id			= PLAT8250_DEV_PLATFORM,
	.dev = {
		.platform_data	= c300v2evm_uart_data,
	},
};
#endif

/* --------------------------------------------------------------------
 *  I2C bus controller
 * -------------------------------------------------------------------- */
#ifdef CONFIG_I2C_COMCERTO
static struct resource c300v2evm_i2c_resources[] = {
	{
		.start	= COMCERTO_I2C_BASE,
		.end	= COMCERTO_I2C_BASE + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= IRQ_I2C,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device c300v2evm_i2c = {
	.name		= "comcerto_i2c",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(c300v2evm_i2c_resources),
	.resource	= c300v2evm_i2c_resources,
};

#endif

/* --------------------------------------------------------------------
 *  Watchdog
 * -------------------------------------------------------------------- */
#ifdef CONFIG_COMCERTO_WATCHDOG

static struct comcerto_wdt_data c300v2evm_wdt_data = {
	.system_reset	 		= 1,
	.gpio_polarity_active_high	= 0,
	.gpio_line_enable 		= 1,
	.gpio_pulse_width_us		= 100,
	.gpio_line_number		= 2,
};

static struct platform_device c300v2evm_wdt = {
	.name	= "comcerto_wdt",
	.id	= -1,
	.dev = {
		.platform_data	= &c300v2evm_wdt_data,
	}
};
#endif

/* --------------------------------------------------------------------
 *  GEMAC0 - the first physical ethernet interface (eth0)
 *  GEMAC1 - the second physical ethernet interface (eth2)
 *
 *  Interface is controlled either by firmware or kernel. GEMAC0 can be
 *  used by kernel only if no firmware is used at all or firmware is
 *  stopped. GEMAC1 can be used if firmware doesn't control it (special
 *  key is needed). Don't be confused with platform stuff naming - eth
 *  suffix here corresponds to Ethernet driver instance.
 * -------------------------------------------------------------------- */
#if defined(CONFIG_CSP_ARM0) || defined(CONFIG_COMCERTO_ETH)
static struct comcerto_gemac_platform_data c300v2evm_gemac0_data = {
	.mode		= GEMAC_CONFIG_MODE_BOOT,
	.speed		= GEMAC_CONFIG_SPEED_1000,
	.duplex		= GEMAC_CONFIG_DUPLEX_FULL,

	.phy_addr	= -1,	/* 0, negative value disables PHY support */
};

static struct resource c300v2evm_eth0_resources[] = {
	{
		.name   = "emac",
		.start  = COMCERTO_GEMAC0_BASE,
		.end    = COMCERTO_GEMAC0_BASE + SZ_64K -1,
		.flags  = IORESOURCE_MEM,
	},
	{
		.name   = "edma",
		.start  = COMCERTO_EDMA_GEMAC0_BASE,
		.end    = COMCERTO_EDMA_GEMAC0_BASE + 0x1FF,
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
	{
		.name	= "emac",
		.start	= IRQ_GEMAC0,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device c300v2evm_eth0 = {
	.name		= "comcerto-eth",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(c300v2evm_eth0_resources),
	.resource	= c300v2evm_eth0_resources,
	.dev = {
		.platform_data = &c300v2evm_gemac0_data,
	},
};

static struct comcerto_gemac_platform_data c300v2evm_gemac1_data = {
	.mode		= GEMAC_CONFIG_MODE_BOOT,
	.speed		= GEMAC_CONFIG_SPEED_1000,
	.duplex		= GEMAC_CONFIG_DUPLEX_FULL,

	.phy_addr	= -1,	/* 1, negative value disables PHY support */
};

static struct resource c300v2evm_eth1_resources[] = {
	{
		.name   = "emac",
		.start  = COMCERTO_GEMAC1_BASE,
		.end    = COMCERTO_GEMAC1_BASE + SZ_64K -1,
		.flags  = IORESOURCE_MEM,
	},
	{
		.name   = "edma",
		.start  = COMCERTO_EDMA_GEMAC1_BASE,
		.end    = COMCERTO_EDMA_GEMAC1_BASE + 0x1FF,
		.flags  = IORESOURCE_MEM,
	},
	{
		.name	= "rx",
		.start	= IRQ_EDMA1RX,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name	= "tx",
		.start	= IRQ_EDMA1TX,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name	= "emac",
		.start	= IRQ_GEMAC1,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device c300v2evm_eth1 = {
	.name		= "comcerto-eth",
	.id		= 1,
	.num_resources	= ARRAY_SIZE(c300v2evm_eth1_resources),
	.resource	= c300v2evm_eth1_resources,
	.dev = {
		.platform_data = &c300v2evm_gemac1_data,
	},
};
#endif

/* --------------------------------------------------------------------
 *  MDIO interface (GEMAC0 bus)
 *
 *  Both PHYs are controlled via GEMAC0 MDIO bus, if you enable MDIO
 *  support for the second GEMAC's PHY (c300v2evm_gemac1_data.phy_addr > 0)
 *  you must not use Comcerto MDIO API.
 * -------------------------------------------------------------------- */
#if defined(CONFIG_CSP_ARM0) || defined(CONFIG_COMCERTO_ETH)
static struct comcerto_gemac_mdio_platform_data c300v2evm_mdio_data = {
	.mdio_speed_hz	= 2500000,
	.phy_mask	= 0xFFFFFFFC,
};

static struct resource c300v2evm_mdio_resources[] = {
	{
		.name   = "emac",
		.start  = COMCERTO_GEMAC0_BASE,
		.end    = COMCERTO_GEMAC0_BASE + SZ_64K -1,
		.flags  = IORESOURCE_MEM,
	},
};

static struct platform_device c300v2evm_mdio = {
	.name		= "comcerto-mdio",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(c300v2evm_mdio_resources),
	.resource	= c300v2evm_mdio_resources,
	.dev = {
		.platform_data = &c300v2evm_mdio_data,
	},
};
#endif

static struct platform_device *c300v2evm_devices[] __initdata = {
#if defined(CONFIG_COMCERTO_UART0_SUPPORT) || defined(CONFIG_COMCERTO_UART1_SUPPORT)
	&c300v2evm_uart,
#endif
#ifdef CONFIG_COMCERTO_MTD_NAND
	&c300v2evm_nand,
#endif
#ifdef CONFIG_COMCERTO_MTD_NOR
	&c300v2evm_nor,
#endif
#ifdef CONFIG_I2C_COMCERTO
	&c300v2evm_i2c,
#endif
#ifdef SPI_ENABLED
	&c300v2evm_spi0,
	&c300v2evm_spi1,
#endif
#ifdef CONFIG_COMCERTO_WATCHDOG
	&c300v2evm_wdt,
#endif
#if defined(CONFIG_CSP_ARM0) || defined(CONFIG_COMCERTO_ETH)
	&c300v2evm_eth0,
	&c300v2evm_eth1,
	&c300v2evm_mdio,
#endif
};


static int __init c300v2evm_init(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(c300v2evm_uart_data) - 1; i++)
		c300v2evm_uart_data[i].uartclk = comcerto_pll_bus_clock_hz;

	c300v2evm_gpio_init();

	platform_add_devices(c300v2evm_devices, ARRAY_SIZE(c300v2evm_devices));

#ifdef SPI_ENABLED
	spi_register_board_info(c300v2evm_legerity_spi_info, 2);
#endif

	return 0;
}

arch_initcall(c300v2evm_init);


MACHINE_START(COMCERTO, "C300 V2 EVM (x690)")
	/* Mindspeed Technologies Inc. */
	.phys_io        = APB_MEMORY_PHY,
	.io_pg_offst    = ((APB_MEMORY_VADDR)>>18) & 0xFFFC,
	.boot_params	= COMCERTO_SDRAM_BASE + 0x100,
	.fixup		= comcerto300_fixup,
	.map_io		= comcerto300_map_io,
	.init_irq	= c300v2evm_irq_init,
	.timer		= &comcerto_timer,
MACHINE_END
