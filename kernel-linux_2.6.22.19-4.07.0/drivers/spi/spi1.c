#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/sched.h>

#include <linux/spi/spi.h>
#include <linux/spi/eeprom.h>
/*add header for spidev*/
#include <asm/uaccess.h>
#include <linux/ioctl.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>

/*add header for character device*/
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/kdev_t.h>
#include <linux/uaccess.h>
#include <linux/io.h>
/*********************************/
/* spidev */
//#define SPIDEV_MAJOR			159	/* assigned */
//#define N_SPI_MINORS			32	/* ... up to 256 */
//static unsigned long	minors[N_SPI_MINORS / BITS_PER_LONG];
/* Bit masks for spi_device.mode management */
#define SPI_MODE_MASK			(SPI_CPHA | SPI_CPOL)

/* define SPI1 register */
#define SPI1_BASE 				0x1009A000
#define SPI1_END  				0x1009BFFF
#define COMCERTO_SPI_CTRLR0     0x00
#define COMCERTO_SPI_CTRLR1     0x04
#define COMCERTO_SPI_SSIENR     0x08
#define COMCERTO_SPI_MWCR       0x0C
#define COMCERTO_SPI_SER        0x10
#define COMCERTO_SPI_BAUDR      0x14
#define COMCERTO_SPI_TXFTLR     0x18
#define COMCERTO_SPI_RXFTLR     0x1C
#define COMCERTO_SPI_TXFLR      0x20
#define COMCERTO_SPI_RXFLR      0x24
#define COMCERTO_SPI_SR         0x28
#define COMCERTO_SPI_IMR        0x2C
#define COMCERTO_SPI_ISR        0x30
#define COMCERTO_SPI_RISR       0x34
#define COMCERTO_SPI_TXOICR     0x38
#define COMCERTO_SPI_RXOICR     0x3C
#define COMCERTO_SPI_RXUICR     0x40
#define COMCERTO_SPI_MSTICR     0x44
#define COMCERTO_SPI_ICR        0x48
#define COMCERTO_SPI_IDR        0x58
#define COMCERTO_SPI_DR         0x60

#define SLAVE_SELECT_ENABLE 	0x1000
#define SLAVE_SELECT			0x1004

static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);
static unsigned bufsiz = 4096;
module_param(bufsiz, uint, S_IRUGO);
MODULE_PARM_DESC(bufsiz, "data bytes in biggest supported SPI message");
/*-------------------------------------------*/

/*struct spi1_data {
	struct spi_device	*spi;
	struct mutex		lock;
	struct spi_eeprom	chip;
	struct bin_attribute	bin;
	unsigned		addrlen;

	struct cdev   c_dev; // tao ra character device
	dev_t dev_num;
	struct class  *class_p; // tao ra class
	struct device *device_p; // da co spi_device thi can them cai nay ko ? co can de thuc hien phan device file
	
	struct mutex buf_lock;
	unsigned 	users;
	u8 			*buffer;
};*/
static dev_t  dev;
static struct cdev   c_dev;
static struct class  *class_p;
static struct device *device_p;

static void __iomem *io;

static u8 buf[10000] = {1,2,3,4,5,6,7,8,9,10};
static void __iomem *io;

/*
struct spi1_data    *spidev;
spidev = kzalloc(sizeof *spi0, GFP_KERNEL);
spidev->spi = spi;
dev_set_drvdata(spi, spidev);
*/

/* tao ra file ops de thao tac voi device file */
static int spi1_open(struct inode *inodep, struct file *file)
{
	printk(KERN_INFO "======protocol_spi1: %s\n", __FUNCTION__);
	/* disable SPI SSI*/
	io = ioremap(SPI1_BASE, SPI1_END - SPI1_BASE);
	if (io == NULL) {
		printk("====== error ioremap in %s\n", __FUNCTION__);
		return -1;
	}
//	__raw_writel(0x0, io + COMCERTO_SPI_SSIENR);
	iowrite32(0x00, (io + COMCERTO_SPI_SSIENR));
	printk("====== disable SSIENR success in %s\n", __FUNCTION__);
	return 0;
}
static int spi1_release(struct inode *inodep, struct file *file)
{
	printk(KERN_INFO "======protocol_spi1: %s\n", __FUNCTION__);
	return 0;
}
/*static void write(platform device *spi)
{
	spi_write(spi, buf, 10);
}*/
static int spi1_write(struct inode *inodep, struct file *file)
{
	printk(KERN_INFO "======protocol_spi1: %s\n", __FUNCTION__);
	io = ioremap(SPI1_BASE, SPI1_END - SPI1_BASE);
	if (io == NULL) {
		printk("====== error ioremap in %s\n", __FUNCTION__);
		return -1;
	}
	__raw_writel(0x0, io + COMCERTO_SPI_SER);
	printk("====== enable SER success in %s\n", __FUNCTION__);
/*	__raw_writel(0x1, io + COMCERTO_SPI_SSIENR);
	printk("====== enable SSIENR success in %s\n", __FUNCTION__);*/
	return 0;
}
static int spi1_read(struct inode *inodep, struct file *file)
{
	printk(KERN_INFO "======protocol_spi1: %s\n", __FUNCTION__);
	io = ioremap(SPI1_BASE, SPI1_END - SPI1_BASE);
	if (io == NULL) {
		printk("====== error ioremap in %s\n", __FUNCTION__);
		return -1;
	}
/*	iowrite32(0x7, io + SLAVE_SELECT);
	printk("====== enable SER success in %s\n", __FUNCTION__);*/
	iowrite32(0x1, (io + SLAVE_SELECT_ENABLE));
	printk("====== enable SSIENR success in %s\n", __FUNCTION__);
	iowrite32(0x7, (io + SLAVE_SELECT));
	return 0;
}
static struct file_operations fops = {
	.open = spi1_open,
	.release = spi1_release,
	.write = spi1_write,
	.read = spi1_read
};
/*----------------------------------------------------------------*/
static int __init spi1_init(void)
{
	int res = -1;
	printk("===== %s\n", __FUNCTION__);
	res = alloc_chrdev_region(&dev, 0, 1, "spi1_dev");

	if (res < 0) {
		pr_info("======= error occur, can not register major number in function %s\n", __FUNCTION__);
		goto fail;
	} else {
		res = -1;
	}
	class_p = class_create(THIS_MODULE, "spi1_class");
	if (class_p == NULL) {
		pr_info("====== error occur, can not create class device %s\n", __FUNCTION__);
		goto fail;
	}
	device_p = device_create(class_p, NULL, dev, "spi1_device_file");
	if (device_p == NULL) {
		pr_info("====== can not create device %s\n",__FUNCTION__);
		goto fail;
	}

	//return spi_register_driver(&spi1_driver);
	cdev_init(&c_dev, &fops);
	c_dev.owner = THIS_MODULE;
	c_dev.dev = dev;
	res = cdev_add(&c_dev, dev, 1);
	if (res) {
		printk("====== error occur when add properties for struct cdev %s\n", __FUNCTION__);
		goto fail;
	} else {
		res = -1;
	}
	return 0;
fail:
    printk("====== fail in %s\n",__FUNCTION__);
//    dev_dbg(&spi->dev, "======probe err %d\n", err);
//    kfree(spi1);
    return -1;
}
module_init(spi1_init);

static void __exit spi1_exit(void)
{
	//	spi_unregister_driver(&spi1_driver);
	cdev_del(&c_dev);
	device_destroy(class_p, dev);
	class_destroy(class_p);
	unregister_chrdev_region(dev, 1);

}
module_exit(spi1_exit);

MODULE_DESCRIPTION("Driver SPI1");
MODULE_AUTHOR("Dicom");
MODULE_LICENSE("GPL");

