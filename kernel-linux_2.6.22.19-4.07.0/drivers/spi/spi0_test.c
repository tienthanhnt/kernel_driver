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
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/kdev_t.h>

#define SPI0_BASE               0x10098000
#define SPI0_END                0x10099FFF
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

#define SPI_MODE_MASK			(SPI_CPHA | SPI_CPOL)

static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);
static unsigned bufsiz = 4096;
module_param(bufsiz, uint, S_IRUGO);
MODULE_PARM_DESC(bufsiz, "data bytes in biggest supported SPI message");

struct spi0_data {
	struct spi_device	*spi;
	struct mutex		lock;
	struct spi_eeprom	chip;
	struct bin_attribute	bin;
	unsigned		addrlen;
	struct cdev   c_dev; 
	dev_t dev_num;
	struct class  *class_p;
	struct device *device_p;
	void __iomem *io;
	unsigned 	users;
	u8 			*buffer;
};
static struct spi0_data *spi0;

static int spi0_open(struct inode *inodep, struct file *file)
{
	printk(KERN_INFO "======protocol_spi1: %s\n", __FUNCTION__);
	spi0->io = ioremap(SPI0_BASE, SPI0_END - SPI0_BASE);
	if (spi0->io == NULL)
		return ERROR;
	iowrite32(0x00, (spi0->io + COMCERTO_SPI_SSIENR));
	iounmap(spi0->io);
	return 0;
}
static int spi0_release(struct inode *inodep, struct file *file)
{
	return 0;
}
static int spi0_write(struct file *file, const char __user *buf, size_t len, loff_t *offset)
{
	printk(KERN_INFO "======protocol_spi1: %s\n", __FUNCTION__);
	long int temp;
	char *kernel_buf = NULL;
	kernel_buf = kzalloc(len, GFP_KERNEL);
	if (kernel_buf == NULL)
		goto malloc_fail;
	temp = copy_from_user(kernel_buf, buf, len);
	if (temp != 0)
		goto get_data_fail;
	spi0->io = ioremap(SPI0_BASE, SPI0_END - SPI0_BASE);
	if (spi0->io == NULL) {
		printk("====== error ioremap in %s\n", __FUNCTION__);
		goto get_data_fail;
	}
	iowrite32(0x1, (spi0->io + SLAVE_SELECT_ENABLE));
	switch (*kernel_buf) {
		case ENABLE_ALL_PIN:
			printk("ENABLE_ALL_PIN \n");
			iowrite32(0x0, (spi0->io + SLAVE_SELECT));
			break;
		case ENABLE_PIN_2_3_4:
			printk("ENABLE_PIN_2_3_4 \n");
			iowrite32(0x1, (spi0->io + SLAVE_SELECT));
			break;
		case ENABLE_PIN_1_3_4:
			printk("ENABLE_PIN_1_3_4 \n");
			iowrite32(0x2, (spi0->io + SLAVE_SELECT));
			break;
		case ENABLE_PIN_3_4:
			printk("ENABLE_PIN_3_4 \n");
			iowrite32(0x3, (spi0->io + SLAVE_SELECT));
			break;
		case ENABLE_PIN_1_2_4:
			printk("ENABLE_PIN_1_2_4 \n");
			iowrite32(0x4, (spi0->io + SLAVE_SELECT));
			break;
		case ENABLE_PIN_2_4:
			printk("ENABLE_PIN_2_4 \n");
			iowrite32(0x5, (spi0->io + SLAVE_SELECT));
			break;
		case ENABLE_PIN_1_4:
			printk("ENABLE_PIN_1_4 \n");
			iowrite32(0x6, (spi0->io + SLAVE_SELECT));
			break;
		case ENABLE_PIN_4:
			printk("ENABLE_PIN_4 \n");
			iowrite32(0x7, (spi0->io + SLAVE_SELECT));
			break;
		case ENABLE_PIN_1_2_3:
			printk("ENABLE_PIN_1_2_3 \n");
			iowrite32(0x8, (spi0->io + SLAVE_SELECT));
			break;
		case ENABLE_PIN_2_3:
			printk("ENABLE_PIN_2_3 \n");
			iowrite32(0x9, (spi0->io + SLAVE_SELECT));
			break;
		case ENABLE_PIN_3:
			printk("ENABLE_PIN_3 \n");
			iowrite32(0xA, (spi0->io + SLAVE_SELECT));
			break;
		case ENABLE_PIN_1_2:
			printk("ENABLE_PIN_1_2 \n");
			iowrite32(0xB, (spi0->io + SLAVE_SELECT));
			break;
		case ENABLE_PIN_2:
			printk("ENABLE_PIN_2 \n");
			iowrite32(0xC, (spi0->io + SLAVE_SELECT));
			break;
		case ENABLE_PIN_1:
			printk("ENABLE_PIN_1 \n");
			iowrite32(0xD, (spi0->io + SLAVE_SELECT));
			break;
		case DISABLE_ALL:
			printk("DISABLE_ALL \n");
			iowrite32(0xE, (spi0->io + SLAVE_SELECT));
			break;
		default:
			printk("invalid value \n");
			goto malloc_fail;
	}
	iounmap(spi0->io);
	return len;
get_data_fail:
	kfree(kernel_buf);
	kernel_buf = NULL;
malloc_fail:
	return ERROR;
}
static int spi0_read(struct inode *inodep, struct file *file)
{
	printk(KERN_INFO "======protocol_spi1: %s\n", __FUNCTION__);
	return 0;
}

static struct file_operations fops = {
	.open = spi0_open,
	.release = spi0_release,
	.write = spi0_write,
	.read = spi0_read
};

static int spi0_probe(struct spi_device *spi)
{
	printk("===== %s\n", __FUNCTION__);
	int			err; 
	char res;

	if (!(spi0 = kzalloc(sizeof *spi0, GFP_KERNEL))) { 
		printk("====== kazalloc failed %s\n", __FUNCTION__);
		err = -ENOMEM;
		goto fail;
	} 
	spi0->spi = spi;
	dev_set_drvdata(spi, spi0);
	res = alloc_chrdev_region(&spi0->dev_num, 0, 1, "spi0_dev");
	if (res < 0) {
		pr_info("======= error occur, can not register major number in function %s\n", __FUNCTION__);
		goto fail;
	} else {
		res = -1;
	}
	spi0->class_p = class_create(THIS_MODULE, "spi0_class");
	if (spi0->class_p == NULL) {
		pr_info("====== error occur, can not create class device %s\n", __FUNCTION__);
		goto fail;
	} 
	spi0->device_p = device_create(spi0->class_p, NULL, spi0->dev_num, "spi0_device_file");
	if (spi0->device_p == NULL) {
		pr_info("====== can not create device %s\n",__FUNCTION__);
		goto fail;
	}
	cdev_init(&spi0->c_dev, &fops);
	spi0->c_dev.owner = THIS_MODULE;
	spi0->c_dev.dev = spi0->dev_num;
	res = cdev_add(&spi0->c_dev, spi0->dev_num, 1);
	if (res) {
		printk("====== error occur when add properties for struct cdev %s\n", __FUNCTION__);
		goto fail;
	} else {
		res = -1;
	}
 
	return 0;
fail:
	printk("====== fail in %s\n",__FUNCTION__);
	dev_dbg(&spi->dev, "======probe err %d\n", err);
	kfree(spi0);
	return err;
}
static int __devexit spi0_remove(struct spi_device *spi)
{
	spi0 = dev_get_drvdata(&spi->dev);
	cdev_del(&spi0->c_dev);
	device_destroy(spi0->class_p, spi0->dev_num);
	class_destroy(spi0->class_p);
	unregister_chrdev_region(spi0->dev_num, 1);
	kfree(spi0);
	return 0;
}
static struct spi_driver spi0_driver = {
	.driver = {
		.name		= "spi0",
		.owner		= THIS_MODULE,
	},
	.probe		= spi0_probe,
	.remove		= __devexit_p(spi0_remove),
};

static int __init spi0_init(void)
{
	printk("===== %s\n", __FUNCTION__);
	return spi_register_driver(&spi0_driver);
}
module_init(spi0_init);

static void __exit spi0_exit(void)
{
	spi_unregister_driver(&spi0_driver);
}
module_exit(spi0_exit);

MODULE_DESCRIPTION("Driver SPI0");
MODULE_AUTHOR("Dicom");
MODULE_LICENSE("GPL");

