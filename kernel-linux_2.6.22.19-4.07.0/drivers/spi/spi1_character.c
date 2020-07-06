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

#define SPI1_BASE               0x10098000
#define SPI1_END                0x10099FFF
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


#define SLAVE_SELECT_ENABLE     0x1000
#define SLAVE_SELECT            0x1004
#define COMCERTO_SPI0_SIZE      (SPI0_END - SPI0_BASE)

#define ENABLE_ALL_PIN          49
#define ENABLE_PIN_2_3_4        50
#define ENABLE_PIN_1_3_4        51
#define ENABLE_PIN_3_4          52
#define ENABLE_PIN_1_2_4        53
#define ENABLE_PIN_2_4          54
#define ENABLE_PIN_1_4          55
#define ENABLE_PIN_4            56
#define ENABLE_PIN_1_2_3        57
#define ENABLE_PIN_2_3          58
#define ENABLE_PIN_3            59
#define ENABLE_PIN_1_2          60
#define ENABLE_PIN_2            61
#define ENABLE_PIN_1            62
#define DISABLE_ALL             63

#define ERROR					-1

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

static int spi1_open(struct inode *inodep, struct file *file)
{
    io = ioremap(SPI0_BASE, SPI0_END - SPI0_BASE);
    if (io == NULL)
        return ERROR;
    iowrite32(0x00, (io + COMCERTO_SPI_SSIENR));
    iounmap(io);
    return 0;
}
static int spi1_release(struct inode *inodep, struct file *file)
{
    return 0;
}

static int spi1_write(struct file *file, const char __user *buf, size_t len, loff_t *offset)
{
    printk(KERN_INFO "\n ......protocol_spi1: %s......\n", __FUNCTION__);
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
		printk("====== error ioremap in %s\n", __FUNCTION__);
		goto get_data_fail;
	}
	iowrite32(0x1, (io + SLAVE_SELECT_ENABLE));
	switch (*kernel_buf) {
		case ENABLE_ALL_PIN:
			printk("ENABLE_ALL_PIN \n");
			iowrite32(0x0, (io + SLAVE_SELECT));
			break;
		case ENABLE_PIN_2_3_4:
			printk("ENABLE_PIN_2_3_4 \n");
			iowrite32(0x1, (io + SLAVE_SELECT));
			break;
		case ENABLE_PIN_1_3_4:
			printk("ENABLE_PIN_1_3_4 \n");
			iowrite32(0x2, (io + SLAVE_SELECT));
			break;
		case ENABLE_PIN_3_4:
			printk("ENABLE_PIN_3_4 \n");
			iowrite32(0x3, (io + SLAVE_SELECT));
			break;
		case ENABLE_PIN_1_2_4:
			printk("ENABLE_PIN_1_2_4 \n");
			iowrite32(0x4, (io + SLAVE_SELECT));
			break;
		case ENABLE_PIN_2_4:
			printk("ENABLE_PIN_2_4 \n");
			iowrite32(0x5, (io + SLAVE_SELECT));
			break;
		case ENABLE_PIN_1_4:
			printk("ENABLE_PIN_1_4 \n");
			iowrite32(0x6, (io + SLAVE_SELECT));
			break;
		case ENABLE_PIN_4:
			printk("ENABLE_PIN_4 \n");
			iowrite32(0x7, (io + SLAVE_SELECT));
			break;
		case ENABLE_PIN_1_2_3:
			printk("ENABLE_PIN_1_2_3 \n");
			iowrite32(0x8, (io + SLAVE_SELECT));
			break;
		case ENABLE_PIN_2_3:
			printk("ENABLE_PIN_2_3 \n");
			iowrite32(0x9, (io + SLAVE_SELECT));
			break;
		case ENABLE_PIN_3:
			printk("ENABLE_PIN_3 \n");
			iowrite32(0xA, (io + SLAVE_SELECT));
			break;
		case ENABLE_PIN_1_2:
			printk("ENABLE_PIN_1_2 \n");
			iowrite32(0xB, (io + SLAVE_SELECT));
			break;
		case ENABLE_PIN_2:
			printk("ENABLE_PIN_2 \n");
			iowrite32(0xC, (io + SLAVE_SELECT));
			break;
		case ENABLE_PIN_1:
			printk("ENABLE_PIN_1 \n");
			iowrite32(0xD, (io + SLAVE_SELECT));
			break;
		case DISABLE_ALL:
			printk("DISABLE_ALL \n");
			iowrite32(0xE, (io + SLAVE_SELECT));
			break;
		default:
			printk("invalid value \n");
			goto malloc_fail;
	}
	iounmap(io);
	return len;
get_data_fail:
    kfree(kernel_buf);
    kernel_buf = NULL;
malloc_fail:
    return ERROR;

}

static int spi1_read(struct inode *inodep, struct file *file)
{
    printk(KERN_INFO "\n ......protocol_spi1: %s......\n", __FUNCTION__);
	return 0;
}

static struct file_operations fops = {
    .open = spi1_open,
    .release = spi1_release,
    .write = spi1_write,
    .read = spi1_read
};

static int __init spi1_init(void)
{
    if (alloc_chrdev_region(&dev, 0, 1, "spi1_dev") < 0) { /* dang ky device number bien dev kieu dev_t dung de luu bo so MAJOR, MINOR*/
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
//    return spi_register_driver(&spi1_driver);
}

static void __exit spi1_exit(void)
{
    cdev_del(&c_dev);
    device_destroy(class_p, dev);
    class_destroy(class_p);
    unregister_chrdev_region(dev, 1);
//    spi_unregister_driver(&spi1_driver);
}

module_init(spi1_init);
module_exit(spi1_exit);

MODULE_DESCRIPTION("Driver SPI1");
MODULE_AUTHOR("Dicom");
MODULE_LICENSE("GPL");
