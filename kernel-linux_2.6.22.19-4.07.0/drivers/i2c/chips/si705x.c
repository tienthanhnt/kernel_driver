#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <asm/uaccess.h>
#include <linux/cdev.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/mutex.h>

#define I2C_ADDRESS	0x40
#define MEASURE_TEMP 0xE3
#define MEASURE_TEMP_NO_HOLD_MASTER 0xF3
#define RESET 0xFE
#define WRITE_USER_REG1	0xE6
#define READ_USER_REG1	0xE7	

struct i2c_si705x { 
	struct i2c_client *client_p; 
	struct device *device_p;
	struct cdev	c_dev;
	struct class *class_p;
	dev_t dev;
};

static struct i2c_driver i2cdev_driver; 
static struct i2c_si705x *si705x;

static ssize_t i2cdev_read (struct file *file, char __user *buf, size_t count,
		loff_t *offset)
{
	printk(" reading temperature in %s\n",__FUNCTION__);
	void *tmp;
	int ret;
	u8 *i2c_command;
	u32 i2c_data;
	struct i2c_client *client = (struct i2c_client *)file->private_data; 

	ret = -1;
	i2c_data = -1;
	tmp = kmalloc(count,GFP_KERNEL);
	if (tmp==NULL)
		return -1;
	/*send data to sensor*/
	i2c_command = kmalloc(2,GFP_KERNEL);
	if (i2c_command==NULL) {
		printk("fail to kmalloc i2c_command %s\n",__FUNCTION__);
		return -1;
	}
	*i2c_command = MEASURE_TEMP;
	ret = i2c_smbus_write_byte(client,*i2c_command);
	if (0>ret) {
		printk("send data i2c failed %s\n",__FUNCTION__);
		return -1;
	} else
		printk("send data i2c success %s\n",__FUNCTION__);

	/*	ret = i2c_master_send(client,i2c_command,1);
		if (ret>0) {
		printk("send command success %s\n",__FUNCTION__);
		} else
		printk("send command unsuccess %s\n",__FUNCTION__);*/

	/*read temp from sensor tim cach check gia tri doc ve co dung ko ? co the dua vao ham co san cua i2c? hay dung crc cua sensor?*/
	/*    ret = i2c_master_recv(client, tmp, 1);
		  if (ret >= 0) {
		  printk("tmp = %x\n", *tmp);
		  ret = copy_to_user(buf,tmp,1)?-1:ret;
		  }*/
	i2c_data = i2c_smbus_read_byte(client);
	if (0>i2c_data) {
		printk("read i2c data failed");
	} else {
		printk("i2c_data = %x\n", i2c_data);
		copy_to_user(buf,&i2c_data,1);
	}
	kfree(tmp);
	return ret;
}

static ssize_t i2cdev_write (struct file *file, const char __user *buf, size_t count,
		loff_t *offset)
{
	return 0;
}  

static int i2cdev_open(struct inode *inode, struct file *file)
{
	struct i2c_client *client; 
	client = kzalloc(sizeof(*client), GFP_KERNEL);
	if (!client) {
		printk("fail to create client in %s\n",__FUNCTION__);
		return -1;
	}
	client = si705x->client_p;
	file->private_data = client; 
	return 0;
}

static int i2cdev_release(struct inode *inode, struct file *file)
{
	struct i2c_client *client = file->private_data;
	kfree(client);
	file->private_data = NULL;
	return 0;
}

static const struct file_operations i2cdev_fops = {
	.owner      = THIS_MODULE,
	.read       = i2cdev_read,
	.write      = i2cdev_write,
	.open       = i2cdev_open,
	.release    = i2cdev_release,
};

static int si705x_i2c_probe(struct i2c_adapter *adapter, int addr, int kind)
{
	int err;
	struct i2c_client *client;
	si705x = kzalloc(sizeof(*si705x), GFP_KERNEL);
	if (si705x == NULL) {
		printk("si705x: failed to allocate memory in %s\n",__FUNCTION__);
		return -1;
	}
	err = alloc_chrdev_region(&si705x->dev, 0, 1, "i2c_dev");
	if (0>err) {
		printk("Alloc character device failed %s\n",__FUNCTION__);
		goto fail_malloc;
	}
	si705x->class_p = class_create(THIS_MODULE, "class_si705x");
	if (si705x->class_p == NULL) {
		pr_info("Error occur, can not create class device\n");
		goto fail_alloc;
	}
	si705x->device_p = device_create(si705x->class_p, NULL, si705x->dev, "led_drv");
	if (si705x->device_p == NULL) {
		pr_info("Can not create device\n");
		return -EFAULT;
	}
	cdev_init(&si705x->c_dev, &i2cdev_fops);
	si705x->c_dev.owner = THIS_MODULE;
	si705x->c_dev.dev = si705x->dev;
	cdev_add(&si705x->c_dev, si705x->dev, 1);

	client = (void*) si705x;
	strncpy(client->name, "si705x_driver", I2C_NAME_SIZE);
	client->adapter = adapter;
	client->driver = &i2cdev_driver;
	client->addr = addr;

	err = i2c_attach_client(client);
	if (err != 0) {
		dev_err(&client->dev, "failed to register device\n");
		goto err_free;
	}
	return 0;

err_free:
	kfree(si705x);
fail_malloc:
	kfree(si705x);
	return -1;
fail_alloc:
	unregister_chrdev_region(si705x->dev, 1);
	kfree(si705x);
	return -1;	
}

static int si705x_attach_adapter(struct i2c_adapter *adapter)
{
	static unsigned short ignore[]	= { I2C_CLIENT_END };
	static unsigned short normal_addr[] = { I2C_ADDRESS, I2C_CLIENT_END };
	static struct i2c_client_address_data addr_data =
	{
		.normal_i2c	= normal_addr,
		.probe		= ignore,
		.ignore		= ignore,
	};
	int ret = -1;
	ret =  i2c_probe(adapter, &addr_data, si705x_i2c_probe);
	return ret;
}

static int si705x_detach_client(struct i2c_client *client)
{
	int err;
	err = -1;
	si705x->client_p = client;
	cdev_del(&si705x->c_dev);
	device_destroy(si705x->class_p, si705x->dev);
	class_destroy(si705x->class_p);
	unregister_chrdev_region(si705x->dev, 1);
	kfree(si705x);
	err = i2c_detach_client(client);
	if (err != 0)
		kfree(client);
	return err;
}

static struct i2c_driver i2cdev_driver = {
	.driver = {
		.owner 	= THIS_MODULE,
		.name   = "si705x_driver",
	},
	.attach_adapter	= si705x_attach_adapter,
	.detach_client	= si705x_detach_client,
};

static int __init i2c_si705x_init(void)
{
	return i2c_add_driver(&i2cdev_driver);
}

static void __exit i2c_si705x_exit(void)
{
	i2c_del_driver(&i2cdev_driver);
}

module_init(i2c_si705x_init);
module_exit(i2c_si705x_exit);

MODULE_AUTHOR("DICOM");
MODULE_DESCRIPTION("I2C si705x");
MODULE_LICENSE("GPL");

