/*
 *  drivers/i2c/chips/m24512-eeprom.c
 *
 *  Copyright (C) 2008 Mindspeed Technologies, Inc.
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/mutex.h>


MODULE_AUTHOR("Mindspeed Technologies, Inc.");
MODULE_DESCRIPTION("STMicroelectronics M24256/512 I2C EEPROM driver");
MODULE_LICENSE("GPL");


#if	defined(CONFIG_ST_M24256_EEPROM)

#define M24_CHIP_SIZE		(32*1024)
#define M24_PAGE_SIZE		64

#elif	defined(CONFIG_ST_M24512_EEPROM)

#define M24_CHIP_SIZE		(64*1024)
#define M24_PAGE_SIZE		128

#else
	#error Not supported
#endif

#define M24_WRITE_TIME_MS	5

struct m24_device {
	struct i2c_client	client;

	struct mutex		lock;
	struct bin_attribute	node;

	u8			txbuf[M24_PAGE_SIZE];
};

static struct i2c_driver m24_i2c_driver;

static inline void m24_set_address(struct i2c_msg *msg, u32 addr)
{
	msg->buf[0] = addr >> 8;
	msg->buf[1] = addr & 255;
	msg->len += 2;
}

static int m24_set_pointer(struct i2c_client *client, u32 addr)
{
	int err;
	u8 buf[2];
	struct i2c_msg msg =
	{
		.addr = client->addr,
		.flags = 0,
		.len = 0,
		.buf = buf,
	};

	m24_set_address(&msg, addr);
	if ((err = i2c_transfer(client->adapter, &msg, 1)) != 1) {
		dev_err(&client->dev, "read transaction failed - couldn't set address, code: %d\n", err);
		return -EIO;
	}

	return 0;
}

static int m24_read_data(struct i2c_client *client, void *buf, int len)
{
	int err;
	struct i2c_msg msg =
	{
		.addr = client->addr,
		.flags = I2C_M_RD,
		.len = len,
		.buf = buf,
	};

	if ((err = i2c_transfer(client->adapter, &msg, 1)) != 1) {
		dev_err(&client->dev, "read transaction failed, code: %d\n", err);
		return -EIO;
	}

	return 0;
}

static int m24_write_data(struct i2c_client *client, u32 addr, void *buf, void *src, int len)
{
	int err;
	struct i2c_msg msg =
	{
		.addr = client->addr,
		.flags = 0,
		.len = len,
		.buf = buf,
	};

	/* set address and copy data just after address bytes */
	m24_set_address(&msg, addr);
	memcpy(buf + 2, src, len);

	if ((err = i2c_transfer(client->adapter, &msg, 1)) != 1) {
		dev_err(&client->dev, "write transaction failed, code: %d\n", err);
		return -EIO;
	}

	set_current_state(TASK_UNINTERRUPTIBLE);
	schedule_timeout(msecs_to_jiffies(M24_WRITE_TIME_MS) + 1);

	return 0;
}

static int m24_read_write(struct kobject *kobj, char *buf, loff_t offs, size_t len, int read)
{
	struct i2c_client *client = to_i2c_client(container_of(kobj, struct device, kobj));
	struct m24_device *m24 = (void*) client;
	int count;

	if (offs + len > M24_CHIP_SIZE)
		len = M24_CHIP_SIZE - offs;
	if (len < 0)
		return -EINVAL;
	if (len == 0)
		return 0;

	mutex_lock(&m24->lock);

	if (read) {
		count = m24_set_pointer(client, offs);	/* for the read transaction set address once at start */
		if (count == 0)
			count = m24_read_data(client, buf, len);
		if (count == 0) {
			count = len;
		}
	} else {
		u32 tmp;
		int req, err;

		/* write all data in no more than one page size transaction */
		count = 0;

		do {
			tmp = (offs + count + M24_PAGE_SIZE) & ~(M24_PAGE_SIZE - 1);
			req = tmp - offs - count;
			if (req + count > len)
				req = len - count;

			err = m24_write_data(client, offs+count, m24->txbuf, buf+count, req);
			if (err < 0) {
				count = err;
				break;
			}

			count += req;
		} while (count < len);
	}

	mutex_unlock(&m24->lock);

	if (count < 0)
		count = -EIO;

	return count;
}

static ssize_t m24_read(struct kobject *kobj, char *buf, loff_t off, size_t count)
{
	return m24_read_write(kobj, buf, off, count, 1);
}

static ssize_t m24_write(struct kobject *kobj, char *buf, loff_t off, size_t count)
{
	return m24_read_write(kobj, buf, off, count, 0);
}

static int m24_i2c_probe(struct i2c_adapter *adapter, int addr, int kind)
{
	int err;
	struct m24_device *m24;
	struct i2c_client *client;
	m24 = kzalloc(sizeof(*m24), GFP_KERNEL);
	if (m24 == NULL) {
		dev_err(&adapter->dev, "m24512: failed to allocate memory\n");
		return -ENOMEM;
	}

	client = (void*) m24;

	strncpy(client->name, "m24512-eeprom", I2C_NAME_SIZE);
	client->adapter = adapter;
	client->driver = &m24_i2c_driver;
	client->addr = addr;

	mutex_init(&m24->lock);

	err = i2c_attach_client(client);
	if (err != 0) {
		dev_err(&client->dev, "failed to register device\n");
		goto err_free;
	}

	m24->node.attr.name = client->name;
	m24->node.attr.mode = 0664;
	m24->node.attr.owner = THIS_MODULE;
	m24->node.size	= M24_CHIP_SIZE;
	m24->node.read	= m24_read;
	m24->node.write	= m24_write;
	err = sysfs_create_bin_file(&client->dev.kobj, &m24->node);
	if (err != 0) {
		dev_err(&client->dev, "failed to register sysfs node\n");
		goto err_detach;
	}

	return 0;

err_detach:
	i2c_detach_client(client);
err_free:
	kfree(m24);

	return err;
}

static int m24_attach_adapter(struct i2c_adapter *adapter)
{
	static unsigned short ignore[]	= { I2C_CLIENT_END };
	static unsigned short normal_addr[] = { 0x50,0x51,0x52,0x53,0x54,0x55,0x56,0x57, I2C_CLIENT_END }; 
										  /*0x40,0x41,0x42,0x43,0x44,0x45,0x46,0x47,*/
	static struct i2c_client_address_data addr_data =
	{
		.normal_i2c	= normal_addr,
		.probe		= ignore,
		.ignore		= ignore,
	};

	return i2c_probe(adapter, &addr_data, m24_i2c_probe);
}

static int m24_detach_client(struct i2c_client *client)
{
	int err;
	struct m24_device *m24 = (void*) client;

	sysfs_remove_bin_file(&client->dev.kobj, &m24->node);

	err = i2c_detach_client(client);
	if (err != 0)
		kfree(client);

	return err;
}

static struct i2c_driver m24_i2c_driver = {
	.driver = {
		.name	= "m24512-eeprom",
		.owner	= THIS_MODULE,
	},
	.attach_adapter	= m24_attach_adapter,
	.detach_client	= m24_detach_client,
};

static int __init m24_init(void)
{
	return i2c_add_driver(&m24_i2c_driver);
}

static void __exit m24_exit(void)
{
	i2c_del_driver(&m24_i2c_driver);
}

module_init(m24_init);
module_exit(m24_exit);
