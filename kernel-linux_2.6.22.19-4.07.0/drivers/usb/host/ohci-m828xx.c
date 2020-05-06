/*
 *  linux/drivers/usb/host/ohci-m828xx.c
 *
 *  Copyright (C) Mindspeed Technologies
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/platform_device.h>
#include <asm/arch/gpio.h>


extern int usb_disabled(void);

static void m828xx_start_hc(struct usb_hcd *hcd, struct platform_device *pdev)
{
	dev_dbg(&pdev->dev, "starting M828xx OHCI USB Controller\n");

	/*
	 * reset the USB IDMA and set the lock transfer size 
	 * 	 
	 */
	writel(0x0300000a, COMCERTO_PLL_USB_REG);	// set clk to 48 (refclk is 12Mhz * mult 4)
	
	writel(7, hcd->regs + M828XX_UHDMA_LOKSZ_OFFSET);
	writel(1, hcd->regs + M828XX_UHDMA_RESET_OFFSET);

	/* enable OC pin */
	comcerto_gpio_ctrl(0x3 << 22, 0x3 << 22);
}

static void m828xx_stop_hc(struct usb_hcd *hcd, struct platform_device *pdev)
{
	dev_dbg(&pdev->dev, "stopping M828xx OHCI USB Controller\n");

	/*
	 * Put the USB IDMA into reset.
	 */
	writel(1, hcd->regs + M828XX_UHDMA_RESET_OFFSET);
}

void usb_hcd_m828xx_remove(struct usb_hcd *, struct platform_device *);

/* configure so an HC device and id are always provided */
/* always called with process context; sleeping is OK */


/**
 * usb_hcd_m828xx_probe - initialize M828xx HCDs
 * Context: !in_interrupt()
 *
 * Allocates basic resources for this USB host controller, and
 * then invokes the start() method for the HCD associated with it
 * through the hotplug entry's driver_data.
 *
 * Store this function in the HCD's struct pci_driver as probe().
 */
int usb_hcd_m828xx_probe(const struct hc_driver *driver,
			 struct platform_device *pdev)
{
	int retval;
	struct usb_hcd *hcd = 0;
	struct ohci_hcd *ohci;
	unsigned long base, len;

	if (pdev->num_resources != 2) {
		printk(KERN_ERR "hcd probe: invalid num_resources: %i\n", 
		       pdev->num_resources);
		return -ENODEV;
	}

	if (pdev->resource[0].flags != IORESOURCE_MEM 
	    || pdev->resource[1].flags != IORESOURCE_IRQ) {
		printk(KERN_ERR "hcd probe: invalid resource type\n");
		return -ENODEV;
	}

	base = pdev->resource[0].start;
	len = pdev->resource[0].end - pdev->resource[0].start + 1;
	
	if (!request_mem_region(base, len, hcd_name)) { 
		dev_dbg(&pdev->dev, "request_mem_region failed\n");
		return -EBUSY;
	}

	hcd = usb_create_hcd(driver, &pdev->dev, pdev->dev.bus_id);
	if (hcd == NULL) {
		dev_dbg(&pdev->dev, "hcd_alloc failed\n");
		retval = -ENOMEM;
		goto err1;
	}

	platform_set_drvdata(pdev, hcd);
	ohci = hcd_to_ohci(hcd);
	ohci_hcd_init(ohci);

	/* this is statically remapped in arch/arm/mach-comcerto/comcerto-xxx.c */
	hcd->regs = (void *)APB_VADDR(base);
	hcd->self.controller = &pdev->dev;

	m828xx_start_hc(hcd, pdev);

	retval = usb_add_hcd(hcd, pdev->resource[1].start, IRQF_DISABLED); 
	if (retval != 0) {
		dev_dbg(&pdev->dev, "request_irq failed\n");
		retval = -EBUSY;
		goto err2;
	}

	dev_info(&pdev->dev, "at 0x%p, irq %d\n", hcd->regs, hcd->irq);

	return 0;

 err2:
	platform_set_drvdata(pdev, NULL);
	usb_put_hcd(hcd);

 err1:
	m828xx_stop_hc(hcd, pdev);

	release_mem_region(base, len);

	return retval;
}


/* may be called without controller electrically present */
/* may be called with controller, bus, and devices active */

/**
 * usb_hcd_m828xx_remove - shutdown processing for M828xx HCDs
 * @dev: USB Host Controller being removed
 * Context: !in_interrupt()
 *
 * Reverses the effect of usb_hcd_omap_probe(), first invoking
 * the HCD's stop() method.  It is always called from a thread
 * context, normally "rmmod", "apmd", or something similar.
 *
 */
void usb_hcd_m828xx_remove(struct usb_hcd *hcd, struct platform_device *pdev)
{
	unsigned long base, len;

	dev_info(&pdev->dev, "remove: state %x\n", hcd->state);

	if (in_interrupt())
		BUG();

	dev_dbg(&pdev->dev, "roothub graceful disconnect\n");

	usb_remove_hcd(hcd);

	m828xx_stop_hc(hcd, pdev);

	base = pdev->resource[0].start;
	len = pdev->resource[0].end - pdev->resource[0].start + 1;

	release_mem_region(base, len);

	usb_put_hcd(hcd);
}

/*-------------------------------------------------------------------------*/

static int __devinit ohci_m828xx_start(struct usb_hcd *hcd)
{
	struct ohci_hcd	*ohci = hcd_to_ohci(hcd);
	int ret;

	ohci_dbg(ohci, "ohci_m828xx_start, ohci:%p", ohci);

	if ((ret = ohci_init(ohci)) < 0)
		return ret;

	if ((ret = ohci_run(ohci)) < 0) {
		err("can't start %s", hcd->self.bus_name);
		ohci_stop(hcd);
		return ret;
	}

	return 0;
}

/*-------------------------------------------------------------------------*/

static const struct hc_driver ohci_m828xx_hc_driver = {
	.description =		hcd_name,
	.product_desc =		"M828xx OHCI",
	.hcd_priv_size =	sizeof(struct ohci_hcd),

	/*
	 * generic hardware linkage
	 */
	.irq =			ohci_irq,
	.flags =		HCD_USB11,

	/*
	 * basic lifecycle operations
	 */
	.start =		ohci_m828xx_start,
	.stop =			ohci_stop,
	.shutdown =		ohci_shutdown,

	/*
	 * managing i/o requests and associated device resources
	 */
	.urb_enqueue =		ohci_urb_enqueue,
	.urb_dequeue =		ohci_urb_dequeue,
	.endpoint_disable =	ohci_endpoint_disable,

	/*
	 * scheduling support
	 */
	.get_frame_number =	ohci_get_frame,

	/*
	 * root hub support
	 */
	.hub_status_data =	ohci_hub_status_data,
	.hub_control =		ohci_hub_control,
	.hub_irq_enable =	ohci_rhsc_enable,

	.start_port_reset =	ohci_start_port_reset,
};

/*-------------------------------------------------------------------------*/

static int ohci_hcd_m828xx_drv_probe(struct platform_device *pdev)
{
	if (usb_disabled())
		return -ENODEV;

	return usb_hcd_m828xx_probe(&ohci_m828xx_hc_driver, pdev);
}

static int ohci_hcd_m828xx_drv_remove(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);

	usb_hcd_m828xx_remove(hcd, pdev);

	platform_set_drvdata(pdev, NULL);

	return 0;
}


static struct platform_driver ohci_hcd_m828xx_driver = {
	.probe		= ohci_hcd_m828xx_drv_probe,
	.remove		= ohci_hcd_m828xx_drv_remove,
	.shutdown	= usb_hcd_platform_shutdown,
	.driver		= {
		.name	= "comcerto-ohci",
		.owner	= THIS_MODULE
	},
};
