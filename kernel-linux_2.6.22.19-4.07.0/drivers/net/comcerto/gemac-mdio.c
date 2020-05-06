/*
 *  linux/drivers/net/comcerto/gemac-mdio.c
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
 *
 */
#include <linux/kernel.h>
#include <linux/phy.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <asm/delay.h>
#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/arch/gemac.h>

struct mdio {
	void		*gemac_base;
	struct mii_bus	bus;
	struct device	*dev;
	int		mdio_speed_hz;
	int		phy_irqs[32];
};

static void mdio_set_clock(struct mdio *mdio, u32 clock_hz);

static void mdio_fixup(struct mdio *mdio)
{
	if ((GEMAC_NET_CONTROL(mdio->gemac_base) & GEMAC_NET_CONTROL_MDIO_ENABLE))
		return;

	dev_warn(mdio->dev, "re-enabling MDIO interface\n");

	GEMAC_NET_CONTROL(mdio->gemac_base) |= GEMAC_NET_CONTROL_MDIO_ENABLE;
	mdio_set_clock(mdio, mdio->mdio_speed_hz);
}

static int mdio_wait_phy_idle(struct mdio *mdio)
{
	unsigned long start = jiffies;

	mdio_fixup(mdio);

	while ((GEMAC_NET_STATUS(mdio->gemac_base) & GEMAC_NET_STATUS_PHY_IDLE) == 0) {
		mdio_fixup(mdio);

		if (time_after(jiffies, start + HZ/10))
			return -ETIME;
	}

	return 0;
}

static int mdio_write(struct mii_bus *bus, int phy_addr, int phy_reg, u16 value)
{
	struct mdio *mdio = bus->priv;
	u32 data;

	if (mdio_wait_phy_idle(mdio))
		return -ETIME;

	data = GEMAC_PHY_MAINTENANCE_CLAUSE22 | GEMAC_PHY_MAINTENANCE_WRITE | GEMAC_PHY_MAINTENANCE_MAGIC |
		((phy_addr & GEMAC_PHY_MAINTENANCE_PHY_MASK) << GEMAC_PHY_MAINTENANCE_PHY_SHIFT) |
		((phy_reg & GEMAC_PHY_MAINTENANCE_REG_MASK) << GEMAC_PHY_MAINTENANCE_REG_SHIFT) |
		value;

	GEMAC_PHY_MAINTENANCE(mdio->gemac_base) = data;

	return 0;
}

static int mdio_read(struct mii_bus *bus, int phy_addr, int phy_reg)
{
	struct mdio *mdio = bus->priv;
	u32 data;
	u16 value;

	if (mdio_wait_phy_idle(mdio))
		return -ETIME;

	data = GEMAC_PHY_MAINTENANCE_CLAUSE22 | GEMAC_PHY_MAINTENANCE_READ | GEMAC_PHY_MAINTENANCE_MAGIC |
		((phy_addr & GEMAC_PHY_MAINTENANCE_PHY_MASK) << GEMAC_PHY_MAINTENANCE_PHY_SHIFT) |
		((phy_reg & GEMAC_PHY_MAINTENANCE_REG_MASK) << GEMAC_PHY_MAINTENANCE_REG_SHIFT);

	GEMAC_PHY_MAINTENANCE(mdio->gemac_base) = data;
	if (mdio_wait_phy_idle(mdio))
		return -ETIME;

	value = GEMAC_PHY_MAINTENANCE(mdio->gemac_base);

	return value;
}

static void mdio_set_clock(struct mdio *mdio, u32 clock_hz)
{
	u32 divisors[7] = {8,16,32,48,64,96,128};	/* 7th (224) is the failback */
	u32 conf, mdc_divisor;

	conf = GEMAC_NET_CONFIG(mdio->gemac_base);

	for (mdc_divisor = 0; mdc_divisor < ARRAY_SIZE(divisors); mdc_divisor++)
		if (clock_hz * divisors[mdc_divisor] > comcerto_pll_bus_clock_hz)
			break;

	conf &= ~(GEMAC_NET_CONFIG_MDC_CLK_MASK << GEMAC_NET_CONFIG_MDC_CLK_SHIFT);
	conf |= mdc_divisor << GEMAC_NET_CONFIG_MDC_CLK_SHIFT;

	GEMAC_NET_CONFIG(mdio->gemac_base) = conf;
}

static int mdio_probe(struct platform_device *pdev)
{
	struct mii_bus *bus;
	int err = 0;
	struct comcerto_gemac_mdio_platform_data *platform = pdev->dev.platform_data;
	struct resource *r;
	struct mdio *mdio;

	if (platform == NULL) {
		dev_err(&pdev->dev, "no platform data, aborting\n");
		return -EINVAL;
	}

	r = platform_get_resource_byname(pdev, IORESOURCE_MEM, "emac");
	if (!r) {
		dev_err(&pdev->dev, "no 'emac' memory region defined\n");
		return -EINVAL;
	}

	mdio = kzalloc(sizeof(struct mdio), GFP_KERNEL);
	if (mdio == NULL)
		return -ENOMEM;

	mdio->gemac_base = (void*) APB_VADDR(r->start);
	mdio->dev = &pdev->dev;
	mdio->mdio_speed_hz = platform->mdio_speed_hz;

	bus = &mdio->bus;
	bus->name = "Comcerto GEMAC MDIO Bus";
	bus->read = mdio_read;
	bus->write = mdio_write;
	bus->id = pdev->id;
	bus->phy_mask = platform->phy_mask;
	memset(mdio->phy_irqs, -1, sizeof(mdio->phy_irqs));
	bus->irq = mdio->phy_irqs;
	bus->dev = &pdev->dev;

	bus->priv = mdio;

	mdio_set_clock(mdio, platform->mdio_speed_hz);

	GEMAC_NET_CONTROL(mdio->gemac_base) |= GEMAC_NET_CONTROL_MDIO_ENABLE;

	err = mdiobus_register(bus);
	if (err != 0) {
		dev_err(mdio->dev, "failed to register as MDIO bus\n");
		goto err;
	}

	dev_set_drvdata(&pdev->dev, mdio);

	return 0;

err:
	kfree(mdio);
	return err;
}

static int mdio_remove(struct platform_device *pdev)
{
	struct mdio *mdio = dev_get_drvdata(&pdev->dev);

	mdiobus_unregister(&mdio->bus);
	kfree(mdio);

	return 0;
}

static struct platform_driver mdio_driver = {
	.probe = mdio_probe,
	.remove = mdio_remove,
	.driver = {
		.name = "comcerto-mdio",
	},
};

static int __init comcerto_mdio_init(void)
{
	int err;

	err = platform_driver_register(&mdio_driver);

	return err;
}

static void __exit comcerto_mdio_exit(void)
{
	platform_driver_unregister(&mdio_driver);
}

module_init(comcerto_mdio_init);
module_exit(comcerto_mdio_exit);
