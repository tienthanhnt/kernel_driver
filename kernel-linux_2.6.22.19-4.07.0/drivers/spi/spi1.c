#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/sched.h>
#include <linux/spi/spi.h>
#include <linux/spi/eeprom.h>
#include <asm/uaccess.h>
#include <linux/ioctl.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/kdev_t.h>
#include <linux/uaccess.h>
#include <linux/io.h>

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

// name for codec register, used for CPC5750
#define MODE_CONTROL	0xB000
#define TX_PATH_GAIN	0xB100
#define TX_GAIN_STEP	0xB200
#define RX_PATH_GAIN	0xB300
#define RX_GAIN_STEP	0xB400
#define DRX_TIME_SLOT	0xB500
#define DRX_BIT_DELAY	0xB600
#define DTX_TIME_SLOT	0xB700
#define DTX_BIT_DELAY	0xB800
#define GPIO_CONTROL	0xBA00

#define SLAVE_SELECT_ENABLE 	0x1000
#define SLAVE_SELECT			0x1004

static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);
static unsigned bufsiz = 4096;
module_param(bufsiz, uint, S_IRUGO);
MODULE_PARM_DESC(bufsiz, "data bytes in biggest supported SPI message");

static dev_t  dev;
static struct cdev   c_dev;
static struct class  *class_p;
static struct device *device_p;
static void __iomem *io;

static void __iomem *io;

static int spi1_open(struct inode *inodep, struct file *file)
{
	io = ioremap(SPI1_BASE, SPI1_END - SPI1_BASE); 
	if (io == NULL) {
		printk("error ioremap in %s\n", __FUNCTION__);
		return -1;
	}
	iowrite32(0x00, (io + COMCERTO_SPI_SSIENR));
	printk("disable SSIENR success in %s\n", __FUNCTION__);
	return 0;
}

static int spi1_release(struct inode *inodep, struct file *file)
{
	printk(KERN_INFO "protocol_spi1: %s\n", __FUNCTION__);
	return 0;
}

static int spi1_write(struct file *file, const char __user *buf, size_t len, loff_t *offset)
{
	long int temp;
	char *kernel_buf = NULL;
	kernel_buf = kzalloc(len, GFP_KERNEL);
	if (kernel_buf == NULL)
		goto malloc_fail;
	temp = copy_from_user(kernel_buf, buf, len);
	if (temp != 0)
		goto get_data_fail;
	io = ioremap(SPI0_BASE, SPI0_END - SPI0_BASE);
	if (io == NULL) {
		printk("error ioremap in %s\n", __FUNCTION__);
		goto get_data_fail;
	}
	iowrite32(0x1, (io + SLAVE_SELECT_ENABLE));
	iorwrite32(0x0F10, (io + COMCERTO_SPI_CTRLR0));
	iowrite32(0x00A1, (io + COMCERTO_SPI_BAUDR));
	iowrite32(0xF0, (io + COMCERTO_SPI_TXFTLR));
	iowrite32(0xB0, (io + COMCERTO_SPI_RXFTLR));
	iowrite32(0xF, (io + COMCERTO_SPI_SER));
	iowrite32(0x3F, (io + COMCERTO_SPI_IMR));
	iowrite32(0x00, (io + COMCERTO_SPI_SSIENR));
}

static int spi1_read(struct inode *inodep, struct file *file)
{
	printk(KERN_INFO "protocol_spi1: %s\n", __FUNCTION__);
	return 0;
}

static struct file_operations fops = {
	.open = spi1_open,
	.release = spi1_release,
	.write = spi1_write,
	.read = spi1_read
};

static int __init spi0_init(void)
{
	if (alloc_chrdev_region(&dev, 0, 1, "spi1_dev") < 0) {
		pr_info("Error occur, can not register major number\n");
		return ERROR;
	}
	class_p = class_create(THIS_MODULE, "class_spi1");
	if (class_p == NULL) {
		pr_info("Error occur, can not create class device\n");
		return ERROR;
	}
	device_p = device_create(class_p, NULL, dev, "devspi1");
	if (device_p == NULL) {
		pr_info("Can not create device\n");
		return ERROR;
	}
	cdev_init(&c_dev, &fops);
	c_dev.owner = THIS_MODULE;
	c_dev.dev = dev; 
	cdev_add(&c_dev, dev, 1);

	return 0;
}
module_init(spi1_init);

static void __exit spi1_exit(void)
{
    cdev_del(&c_dev);
    device_destroy(class_p, dev);
    class_destroy(class_p);
    unregister_chrdev_region(dev, 1);
}
module_exit(spi1_exit);

MODULE_DESCRIPTION("Driver SPI1");
MODULE_AUTHOR("Dicom");
MODULE_LICENSE("GPL");

