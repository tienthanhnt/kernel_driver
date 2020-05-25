/*
 * test/controller_spi.c
 *

 */

/* header file of spi controller 
/home/macom/linux_kernel/macom/working/kernel_driver/kernel-linux_2.6.22.19-4.07.0/drivers/spi */
#include <linux/version.h>
#include <linux/autoconf.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <asm/delay.h>
#include <asm/fiq.h>
#include <asm/io.h>
#include <asm/arch/spi.h>

/* header file of wdt driver*/
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/watchdog.h>
#include <asm/bitops.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/arch/hardware.h>
#include <asm/arch/wdt.h>

#define SPI_VERSION             "1.0.0"

unsigned int len;
len = 32; // buffer len

/* data write to codec to config operation mode */
const uint8_t buf[32] = {
	0x00, 0x01, 0x02, 0x03,
	0x04, 0x05, 0x06, 0x07,
	0x00, 0x01, 0x02, 0x03,
	0x04, 0x05, 0x06, 0x07,
	0x00, 0x01, 0x02, 0x03,
	0x04, 0x05, 0x06, 0x07,
	0x00, 0x01, 0x02, 0x03,
	0x04, 0x05, 0x06, 0x07,
};

struct spi_codec {
	struct spi_device *spi;
	dev_t  dev_num;
	struct cdev   c_dev;
	struct class  *class_p;
	struct device *device_p
}

//struct spi_codec *spi1; /* su dung doi duong spi1 de truy xuat xu ly data*/
/* nhung ham quan trong dung trong ops cua spi la: .................*/

/* dinh nghia ham open cho spi 
 * co the hieu day la my_open de gan vao cho file ops
 */

static int spi1_open(struct inode *inodep, struct file *file)
{
	//struct spi_codec spi1;
	printk(KERN_INFO "======protocol_spi1: %s: loaded version %s\n", __FUNCTION__, SPI_VERSION);
	
}

static int spi1_release(struct inode *inodep, struct file *file)
{
	//struct spi_codec spi1;
	printk(KERN_INFO "======protocol_spi1: %s: loaded version %s\n", __FUNCTION__, SPI_VERSION);
	
}

static int spi1_write(struct inode *inodep, struct file *file)
{
	//struct spi_codec spi1;
	printk(KERN_INFO "======protocol_spi1: %s: loaded version %s\n", __FUNCTION__, SPI_VERSION);
	
}

static int spi1_ioctl(struct inode *inodep, struct file *file)
{
	printk(KERN_INFO "======protocol_spi1: %s: loaded version %s\n", __FUNCTION__, SPI_VERSION);
	
}

static struct file_operations fops = {
	.open = spi1_open,
	.release = spi1_release,
	.write = spi1_write,
	.unlocked_ioctl = spi1_ioctl,
};

/* ham init se khoi tao gia tri cho cac codec */
static void codec_init(struct spi_device *spi)  // o day phai lay lai duoc doi tuong spi1 o tren
{
/* lay ra  spi1*/
	spi_codec *spi1 = NULL;
	spi1->spi = spi;
/** init cho codec 1 gom cac buoc:
 * chon chip select voi bo so thich hop cho codec 1: vi du chan so 8 thi CS[0:3] = 1000
 * khoi tao clock cho codec 1 bang ham set_frequency: xem trong spi.h
 * ghi data xuong codec 1 bang raw_writel(): xem trong spi.h
 */
	int stat;
	stat = -1;
	spi->bits_per_word = 8;
	spi->mode = SPI_MODE_3;
	spi->max_speed_hz = 16000;
	spi->chip_select = 3;
	stat = spi1->spi_setup(spi);
	cpu_relax();
	if (stat < 0) {
		printk(KERN_INFO "======protocol_spi1: %s: loaded version %s\n", __FUNCTION__, SPI_VERSION);
		return -1;
	} else {
		spi1->spi->spi_write(spi, buf, len); // su dung ham chip message queue
	}
}


/* ham probe duoc goi khi co match voi platform device*/
static int my_probe(struct spi_device *spi) /* truyen vao mot con tro spi_devicei da duoc sinh ra tu controller */
{
	char res = 0;
	int major = MAJOR(dev_num); // so major
	int minor = spi->chip_select; // tai sao bo so minor lai duoc gan bang chip_select
	struct codec_spi *spi1 = NULL; // khai bao doi tuong spi1 o day
	struct device_node *np = spi->dev.of_node; // tao ra 1 device node

/******************* SPI Device Driver ****************/
	spi1 = kzalloc(sizeof(*codec_spi), GFP_KERNEL); // cap phat bo nho cho spi1
	if (spi1 == NULL)
		goto spi1_aloccate_failed;

/* lay ra data tu spi1*/
	spi1->spi = spi;
	/*
	res = spi_setup(lcd->spi); // dung ham nay de setup lai thong so baudrate cho spi
	*/

/***************** Create Device File ***************/
/* dang ky mot bo so minor va major number cho device file??? */
	spi1->dev_number = MKDEV(major, minor);
	spi1->device_p = device_create(class_p, &spi->dev, spi1->dev_number,
					spi1, "spi_%d", spi1->chip_select);
	if (spi1->device_p == NULL) {
		pr_info("Can not create device\n");
		goto create_device_failed;
	}

/* Register operations of device */
	cdev_init(&spi1->c_dev, &fops);
	spi1->c_dev.owner = THIS_MODULE;
	spi1->c_dev.dev = spi1->dev_number;

	res = cdev_add(&spi1->c_dev, spi1->dev_number, 1);
	if (res) {
		pr_err("error occur when add properties for struct cdev\n");
		goto cdev_add_fail;
	}

/*************** Management codec devices *************/
/* khoi tao thong so cho codec*/
	codec_int(spi1);
	pr_info("spi1 Init successfully\n");
	return 0;

cdev_add_fail:
	device_destroy(class_p, spi1->dev_number);
create_device_failed:
	kfree(lcd);
spi1_aloccate_failed:
	return -1;
}

static int my_remove(struct spi_device *spi)
{	
	printk(KERN_INFO "======protocol_spi1: %s: loaded version %s\n", __FUNCTION__, SPI_VERSION);
	struct spi_codec *spi1 = spi_get_drvdata(spi); /* tao doi tuong spi_codec lay du lieu tu spi */

	cdev_del(&spi1->c_dev); // tai sao lai lay dia chi cua spi1 ???
	device_destroy(class_p, spi1->dev_number); // huy bo device
	unregister_chrdev_region(spi1->dev_number, 1); // huy dang ky character device spi1

	spi_clear(spi1);
	spi_free_IOT(spi1);
	kfree(spi1); // free di doi tuong spi1

	return 0;
}


static struct spi_driver spi1_drv = { 
	.probe = spi1_probe,
	.remove = spi1_remove,
	.driver = {
		.name = "comcerto_spi", // phai dat name nay trung voi ten device spi 0 ? xem co che de co the match dung device
		.owner = THIS_MODULE,
//		.id = 1, // dung them id de match dc ko ?// khong

	},
};

static int __init protocol_spi1_init(void)
{
	printk(KERN_INFO "======protocol_spi1: %s: loaded version %s\n", __FUNCTION__, SPI_VERSION);
	return spi_reister_driver(&spi1_drv);
}

static int __exit protocol_spi1_exit(void)
{
	printk(KERN_INFO "======protocol_spi1: %s: loaded version %s\n", __FUNCTION__, SPI_VERSION);
	return spi_unreister_driver(&spi1_drv);
}

module_init(protocol_spi1_init);
module_exit(protocol_spi1_exit);

MODULE_AUTHOR("Dicom");
MODULE_DESCRIPTION("Comcerto spi1 device driver");
MODULE_LICENSE("GPL");

