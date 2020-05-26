#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/sched.h>

#include <linux/spi/spi.h>
#include <linux/spi/eeprom.h>

/*add header for character device*/
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/kdev_t.h>
/*********************************/
/* spidev */
#define SPIDEV_MAJOR			159	/* assigned */
#define N_SPI_MINORS			32	/* ... up to 256 */
static unsigned long	minors[N_SPI_MINORS / BITS_PER_LONG];
/* Bit masks for spi_device.mode management */
#define SPI_MODE_MASK			(SPI_CPHA | SPI_CPOL)
/*-------------------------------------------*/

struct spi0_data {
	struct spi_device	*spi;
	struct mutex		lock;
	struct spi_eeprom	chip;
	struct bin_attribute	bin;
	unsigned		addrlen;

	struct cdev   c_dev; // tao ra character device
	dev_t dev_num;
	struct class  *class_p; // tao ra class
	struct device *device_p; // da co spi_device thi can them cai nay ko ? co can de thuc hien phan device file

	unsigned 	users;
	u8 			*buffer;
};


/* tao ra file ops de thao tac voi device file */
static int spi1_open(struct inode *inodep, struct file *file)
{
	printk(KERN_INFO "======protocol_spi1: %s\n", __FUNCTION__);
}
static int spi1_release(struct inode *inodep, struct file *file)
{
	printk(KERN_INFO "======protocol_spi1: %s\n", __FUNCTION__);
}
static int spi1_write(struct inode *inodep, struct file *file)
{
	printk(KERN_INFO "======protocol_spi1: %s\n", __FUNCTION__);
}
static int spi1_ioctl(struct inode *inodep, struct file *file)
{
	printk(KERN_INFO "======protocol_spi1: %s\n", __FUNCTION__);
}
static struct file_operations fops = {
	.open = spi1_open,
	.release = spi1_release,
	.write = spi1_write,
//	.unlocked_ioctl = spi1_ioctl
};
/*----------------------------------------------------------------*/

static int spi0_probe(struct spi_device *spi)
{
	printk("===== %s\n", __FUNCTION__);
	struct spi0_data	*spi0 = NULL;
	unsigned long minor;
//	const struct spi_eeprom *chip;
	int			err; // bien check loi
	int			addrlen;

	/* tao character device luon */
	if (!(spi0 = kzalloc(sizeof *spi0, GFP_KERNEL))) { // cap phat bo nho cho sp0
		printk("====== kazalloc failed %s\n", __FUNCTION__);
		err = -ENOMEM;
		goto fail;
	} 
	spi0->spi = spi;
	spi0->spi = spi_dev_get(spi);
	dev_set_drvdata(&spi->dev, spi0);
	spi0->addrlen = addrlen;
	int major = MAJOR(spi0->dev_num); // so major
	char res;
	res = -1;
	minor = find_first_zero_bit(minors, N_SPI_MINORS);
	if (minor >= N_SPI_MINORS) {
		printk("====== add minor failed %s\n ", __FUNCTION__);
		goto fail;
	}
	res = alloc_chrdev_region(&spi0->dev_num, minor, 1, "spi0_dev");
	if (res < 0) {
		pr_info("======= error occur, can not register major number in function %s\n", __FUNCTION__);
		goto fail;
	} else {
		res = -1;
	}
	/* tao ra phan xu ly device file*/

//	int minor = spi->chip_select; // tai sao bo so minor lai duoc gan bang chip_select
	//	struct device_node *np = spi->dev._of_node;
	spi0->class_p = class_create(THIS_MODULE, "spi0_class");
	if (spi0->class_p == NULL) {
		pr_info("====== error occur, can not create class device %s\n", __FUNCTION__);
		goto fail;
	} else {
		res = -1;
	}
	if (spi0->dev_num == NULL) {
		printk("====== dev_num = NULL %s\n", __FUNCTION__); // so majorNCTION__);
		goto fail;
	} 
	spi0->device_p = device_create(spi0->class_p, NULL, spi0->dev_num, "spi0_device_file");
	if (spi0->device_p == NULL) {
		pr_info("====== can not create device %s\n",__FUNCTION__);
		goto fail;
	} else {
		res = -1;
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
	/**************************************************/ // create file in sysfs
	spi0->bin.attr.name = "spi0";
	spi0->bin.attr.mode = S_IRUSR;
	spi0->bin.attr.owner = THIS_MODULE;
//	spi0->bin.read = at25_bin_read;

	spi0->bin.size = spi0->chip.byte_len;
/*	if (!(chip->flags & EE_READONLY)) { 
		at25->bin.write = at25_bin_write; 
		at25->bin.attr.mode |= S_IWUSR; 
	}*/
	if ((NULL == &spi->dev.kobj) || (NULL == &spi0->bin)) {
		printk("====== fail to create spi0 bin %s\n",__FUNCTION__);
		goto fail;
	}
/*	err = sysfs_create_bin_file(&spi->dev.kobj, &spi0->bin);
	if (err) {
		printk("====== error occur when create file in sysfs %s\n", __FUNCTION__);
		goto fail;
	} else {
		printk("====== create file in sysfs success %s\n", __FUNCTION__);	
	}*/
	return 0;
fail:
	printk("====== fail in %s\n",__FUNCTION__);
	dev_dbg(&spi->dev, "======probe err %d\n", err);
	kfree(spi0);
	return err;
}
static int __devexit spi0_remove(struct spi_device *spi)
{
	struct spi0_data	*spi0;
	spi0 = dev_get_drvdata(&spi->dev);
	/*remove device file*/
	cdev_del(&spi0->c_dev);
	device_destroy(spi0->class_p, spi0->dev_num);
	class_destroy(spi0->class_p);
	unregister_chrdev_region(spi0->dev_num, 1);
	/****************/
	sysfs_remove_bin_file(&spi->dev.kobj, &spi0->bin);
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

