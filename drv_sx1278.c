/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : drv_sx1278.c
 @brief  : the driver of lora sx1278 module.
 @author : pengrui
 @history:
           2018-04-13    pengrui    Created file
           ...
******************************************************************************/


#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/of_irq.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>

#include <linux/uaccess.h>
#include "drv_sx1278.h"
/*
 * This supports access to SPI devices using normal userspace I/O calls.
 * Note that while traditional UNIX/POSIX I/O semantics are half duplex,
 * and often mask message boundaries, full SPI support requires full duplex
 * transfers.  There are several kinds of internal message boundaries to
 * handle chipselect management and other protocol options.
 *
 * SPI has a character major number assigned.  We allocate minor numbers
 * dynamically using a bitmask.  You must use hotplug tools, such as udev
 * (or mdev with busybox) to create and destroy the /dev/spidevB.C device
 * nodes, since there is no fixed association of minor numbers with any
 * particular SPI bus or device.
 */
#define N_SPI_MINORS    32

//驱动名
#define DRV_NAME        "sx1278"
//主版本号
#define DRV_VER1        1                
//次版本号
#define DRV_VER2        0
//驱动主设备号
#define DRV_MAJOR       107
//次设备个数
#define DRV_MINOR       1                   


static DECLARE_BITMAP(minors, N_SPI_MINORS);


/* Bit masks for spi_device.mode management.  Note that incorrect
 * settings for some settings can cause *lots* of trouble for other
 * devices on a shared bus:
 *
 *  - CS_HIGH ... this device will be active when it shouldn't be
 *  - 3WIRE ... when active, it won't behave as it should
 *  - NO_CS ... there will be no explicit message boundaries; this
 *	is completely incompatible with the shared bus model
 *  - READY ... transfers may proceed when they shouldn't.
 *
 * REVISIT should changing those flags be privileged?
 */
#define SPI_MODE_MASK		(SPI_CPHA | SPI_CPOL | SPI_CS_HIGH \
				| SPI_LSB_FIRST | SPI_3WIRE | SPI_LOOP \
				| SPI_NO_CS | SPI_READY)



static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

static dev_t drv_dev_num = MKDEV(DRV_MAJOR, 0);
static struct cdev drv_cdev;
static struct class *drv_class;


static unsigned bufsiz = 4096;
module_param(bufsiz, uint, S_IRUGO);
MODULE_PARM_DESC(bufsiz, "data bytes in biggest supported SPI message");


static void lora_work_func(struct work_struct *w)
{
    //send mode

    //recv mode
    
}


/*
 * We can't use the standard synchronous wrappers for file I/O; we
 * need to protect against async removal of the underlying spi_device.
 */
static void spidev_complete(void *arg)
{
	complete(arg);
}

static ssize_t spidev_sync(drv_info_st_ptr drv_info_ptr, struct spi_message *message)
{
	DECLARE_COMPLETION_ONSTACK(done);
	int status;

	message->complete = spidev_complete;
	message->context = &done;

	spin_lock_irq(&drv_info_ptr->spi_lock);
	if (drv_info_ptr->spi == NULL)
		status = -ESHUTDOWN;
	else
		status = spi_async(drv_info_ptr->spi, message);
	spin_unlock_irq(&drv_info_ptr->spi_lock);

	if (status == 0) {
		wait_for_completion(&done);
		status = message->status;
		if (status == 0)
			status = message->actual_length;
	}
	return status;
}

static inline ssize_t spidev_sync_write(drv_info_st_ptr drv_info_ptr, size_t len)
{
	struct spi_transfer	t = {
			.tx_buf		= drv_info_ptr->buffer,
			.len		= len,
		};
	struct spi_message	m;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	return spidev_sync(drv_info_ptr, &m);
}

static inline ssize_t spidev_sync_read(drv_info_st_ptr drv_info_ptr, size_t len)
{
	struct spi_transfer	t = {
			.rx_buf		= drv_info_ptr->buffer,
			.len		= len,
		};
	struct spi_message	m;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	return spidev_sync(drv_info_ptr, &m);
}

/* Read-only message with current device setup */
static ssize_t drv_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	drv_info_st_ptr	drv_info_ptr;
	ssize_t			status = 0;

	/* chipselect only toggles at start or end of operation */
	if (count > bufsiz)
		return -EMSGSIZE;

	drv_info_ptr = filp->private_data;

	mutex_lock(&drv_info_ptr->buf_lock);
	status = spidev_sync_read(drv_info_ptr, count);
	if (status > 0) {
		unsigned long	missing;

		missing = copy_to_user(buf, drv_info_ptr->buffer, status);
		if (missing == status)
			status = -EFAULT;
		else
			status = status - missing;
	}
    
    wait_event_interruptible(drv_info_ptr->lora_wq, drv_info_ptr->flag);
    drv_info_ptr->flag = 0x00;
    
	mutex_unlock(&drv_info_ptr->buf_lock);

	return status;
}

/* Write-only message with current device setup */
static ssize_t drv_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
	drv_info_st_ptr	drv_info_ptr;
	ssize_t			status = 0;
	unsigned long		missing;

	/* chipselect only toggles at start or end of operation */
	if (count > bufsiz)
		return -EMSGSIZE;

	drv_info_ptr = filp->private_data;

	mutex_lock(&drv_info_ptr->buf_lock);
	missing = copy_from_user(drv_info_ptr->buffer, buf, count);
	if (missing == 0)
		status = spidev_sync_write(drv_info_ptr, count);
	else
		status = -EFAULT;
	mutex_unlock(&drv_info_ptr->buf_lock);

	return status;
}

static int spidev_message(drv_info_st_ptr drv_info_ptr, struct spi_ioc_transfer *u_xfers, unsigned n_xfers)
{
	struct spi_message	msg;
	struct spi_transfer	*k_xfers;
	struct spi_transfer	*k_tmp;
	struct spi_ioc_transfer *u_tmp;
	unsigned		n, total;
	u8			*buf;
	int			status = -EFAULT;

	spi_message_init(&msg);
	k_xfers = kcalloc(n_xfers, sizeof(*k_tmp), GFP_KERNEL);
	if (k_xfers == NULL)
		return -ENOMEM;

	/* Construct spi_message, copying any tx data to bounce buffer.
	 * We walk the array of user-provided transfers, using each one
	 * to initialize a kernel version of the same transfer.
	 */
	buf = drv_info_ptr->buffer;
	total = 0;
	for (n = n_xfers, k_tmp = k_xfers, u_tmp = u_xfers;
			n;
			n--, k_tmp++, u_tmp++) {
		k_tmp->len = u_tmp->len;

		total += k_tmp->len;
		/* Check total length of transfers.  Also check each
		 * transfer length to avoid arithmetic overflow.
		 */
		if (total > bufsiz || k_tmp->len > bufsiz) {
			status = -EMSGSIZE;
			goto done;
		}

		if (u_tmp->rx_buf) {
			k_tmp->rx_buf = buf;
			if (!access_ok(VERIFY_WRITE, (u8 __user *)
						(uintptr_t) u_tmp->rx_buf,
						u_tmp->len))
				goto done;
		}
		if (u_tmp->tx_buf) {
			k_tmp->tx_buf = buf;
			if (copy_from_user(buf, (const u8 __user *)
						(uintptr_t) u_tmp->tx_buf,
					u_tmp->len))
				goto done;
		}
		buf += k_tmp->len;

		k_tmp->cs_change = !!u_tmp->cs_change;
		k_tmp->bits_per_word = u_tmp->bits_per_word;
		k_tmp->delay_usecs = u_tmp->delay_usecs;
		k_tmp->speed_hz = u_tmp->speed_hz;
#ifdef VERBOSE
		dev_dbg(&spidev->spi->dev,
			"  xfer len %zd %s%s%s%dbits %u usec %uHz\n",
			u_tmp->len,
			u_tmp->rx_buf ? "rx " : "",
			u_tmp->tx_buf ? "tx " : "",
			u_tmp->cs_change ? "cs " : "",
			u_tmp->bits_per_word ? : drv_info_ptr->spi->bits_per_word,
			u_tmp->delay_usecs,
			u_tmp->speed_hz ? : drv_info_ptr->spi->max_speed_hz);
#endif
		spi_message_add_tail(k_tmp, &msg);
	}

	status = spidev_sync(drv_info_ptr, &msg);
	if (status < 0)
		goto done;

	/* copy any rx data out of bounce buffer */
	buf = drv_info_ptr->buffer;
	for (n = n_xfers, u_tmp = u_xfers; n; n--, u_tmp++) {
		if (u_tmp->rx_buf) {
			if (__copy_to_user((u8 __user *)
					(uintptr_t) u_tmp->rx_buf, buf,
					u_tmp->len)) {
				status = -EFAULT;
				goto done;
			}
		}
		buf += u_tmp->len;
	}
	status = total;

done:
	kfree(k_xfers);
	return status;
}

static long drv_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int			err = 0;
	int			retval = 0;
	drv_info_st_ptr	drv_info_ptr;
	struct spi_device	*spi;
	u32			tmp;
	unsigned		n_ioc;
	struct spi_ioc_transfer	*ioc;

	/* Check type and command number */
	if (_IOC_TYPE(cmd) != SPI_IOC_MAGIC)
		return -ENOTTY;

	/* Check access direction once here; don't repeat below.
	 * IOC_DIR is from the user perspective, while access_ok is
	 * from the kernel perspective; so they look reversed.
	 */
	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE,
				(void __user *)arg, _IOC_SIZE(cmd));
	if (err == 0 && _IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ,
				(void __user *)arg, _IOC_SIZE(cmd));
	if (err)
		return -EFAULT;

	/* guard against device removal before, or while,
	 * we issue this ioctl.
	 */
	drv_info_ptr = filp->private_data;
	spin_lock_irq(&drv_info_ptr->spi_lock);
	spi = spi_dev_get(drv_info_ptr->spi);
	spin_unlock_irq(&drv_info_ptr->spi_lock);

	if (spi == NULL)
		return -ESHUTDOWN;

	/* use the buffer lock here for triple duty:
	 *  - prevent I/O (from us) so calling spi_setup() is safe;
	 *  - prevent concurrent SPI_IOC_WR_* from morphing
	 *    data fields while SPI_IOC_RD_* reads them;
	 *  - SPI_IOC_MESSAGE needs the buffer locked "normally".
	 */
	mutex_lock(&drv_info_ptr->buf_lock);

	switch (cmd) {
	/* read requests */
	case SPI_IOC_RD_MODE:
		retval = __put_user(spi->mode & SPI_MODE_MASK,
					(__u8 __user *)arg);
		break;
	case SPI_IOC_RD_LSB_FIRST:
		retval = __put_user((spi->mode & SPI_LSB_FIRST) ?  1 : 0,
					(__u8 __user *)arg);
		break;
	case SPI_IOC_RD_BITS_PER_WORD:
		retval = __put_user(spi->bits_per_word, (__u8 __user *)arg);
		break;
	case SPI_IOC_RD_MAX_SPEED_HZ:
		retval = __put_user(spi->max_speed_hz, (__u32 __user *)arg);
		break;

	/* write requests */
	case SPI_IOC_WR_MODE:
		retval = __get_user(tmp, (u8 __user *)arg);
		if (retval == 0) {
			u8	save = spi->mode;

			if (tmp & ~SPI_MODE_MASK) {
				retval = -EINVAL;
				break;
			}

			tmp |= spi->mode & ~SPI_MODE_MASK;
			spi->mode = (u8)tmp;
			retval = spi_setup(spi);
			if (retval < 0)
				spi->mode = save;
			else
				dev_dbg(&spi->dev, "spi mode %02x\n", tmp);
		}
		break;
	case SPI_IOC_WR_LSB_FIRST:
		retval = __get_user(tmp, (__u8 __user *)arg);
		if (retval == 0) {
			u8	save = spi->mode;

			if (tmp)
				spi->mode |= SPI_LSB_FIRST;
			else
				spi->mode &= ~SPI_LSB_FIRST;
			retval = spi_setup(spi);
			if (retval < 0)
				spi->mode = save;
			else
				dev_dbg(&spi->dev, "%csb first\n",
						tmp ? 'l' : 'm');
		}
		break;
	case SPI_IOC_WR_BITS_PER_WORD:
		retval = __get_user(tmp, (__u8 __user *)arg);
		if (retval == 0) {
			u8	save = spi->bits_per_word;

			spi->bits_per_word = tmp;
			retval = spi_setup(spi);
			if (retval < 0)
				spi->bits_per_word = save;
			else
				dev_dbg(&spi->dev, "%d bits per word\n", tmp);
		}
		break;
	case SPI_IOC_WR_MAX_SPEED_HZ:
		retval = __get_user(tmp, (__u32 __user *)arg);
		if (retval == 0) {
			u32	save = spi->max_speed_hz;

			spi->max_speed_hz = tmp;
			retval = spi_setup(spi);
			if (retval < 0)
				spi->max_speed_hz = save;
			else
				dev_dbg(&spi->dev, "%d Hz (max)\n", tmp);
		}
		break;

	default:
		/* segmented and/or full-duplex I/O request */
		if (_IOC_NR(cmd) != _IOC_NR(SPI_IOC_MESSAGE(0))
				|| _IOC_DIR(cmd) != _IOC_WRITE) {
			retval = -ENOTTY;
			break;
		}

		tmp = _IOC_SIZE(cmd);
		if ((tmp % sizeof(struct spi_ioc_transfer)) != 0) {
			retval = -EINVAL;
			break;
		}
		n_ioc = tmp / sizeof(struct spi_ioc_transfer);
		if (n_ioc == 0)
			break;

		/* copy into scratch area */
		ioc = kmalloc(tmp, GFP_KERNEL);
		if (!ioc) {
			retval = -ENOMEM;
			break;
		}
		if (__copy_from_user(ioc, (void __user *)arg, tmp)) {
			kfree(ioc);
			retval = -EFAULT;
			break;
		}

		/* translate to spi_message, execute */
		retval = spidev_message(drv_info_ptr, ioc, n_ioc);
		kfree(ioc);
		break;
	}

	mutex_unlock(&drv_info_ptr->buf_lock);
	spi_dev_put(spi);
	return retval;
}

#ifdef CONFIG_COMPAT
static long
drv_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	return drv_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#else
#define drv_compat_ioctl NULL
#endif /* CONFIG_COMPAT */

static int drv_open(struct inode *inode, struct file *filp)
{
	drv_info_st_ptr	drv_info_ptr;
	int			status = -ENXIO;

	mutex_lock(&device_list_lock);

	list_for_each_entry(drv_info_ptr, &device_list, device_entry) {
		if (drv_info_ptr->devt == inode->i_rdev) {
			status = 0;
			break;
		}
	}
	if (status == 0) {
		if (!drv_info_ptr->buffer) {
			drv_info_ptr->buffer = kmalloc(bufsiz, GFP_KERNEL);
			if (!drv_info_ptr->buffer) {
				dev_dbg(&drv_info_ptr->spi->dev, "open/ENOMEM\n");
				status = -ENOMEM;
			}
		}
		if (status == 0) {
			drv_info_ptr->users++;
			filp->private_data = drv_info_ptr;
			nonseekable_open(inode, filp);
		}
	} else
		pr_debug("spidev: nothing for minor %d\n", iminor(inode));

	mutex_unlock(&device_list_lock);
	return status;
}

static int drv_release(struct inode *inode, struct file *filp)
{
	drv_info_st_ptr	drv_info_ptr;
	int			status = 0;

	mutex_lock(&device_list_lock);
	drv_info_ptr = filp->private_data;
	filp->private_data = NULL;

	/* last close? */
	drv_info_ptr->users--;
	if (!drv_info_ptr->users) {
		int		dofree;

		kfree(drv_info_ptr->buffer);
		drv_info_ptr->buffer = NULL;

		/* ... after we unbound from the underlying device? */
		spin_lock_irq(&drv_info_ptr->spi_lock);
		dofree = (drv_info_ptr->spi == NULL);
		spin_unlock_irq(&drv_info_ptr->spi_lock);

		if (dofree)
			kfree(drv_info_ptr);
	}
	mutex_unlock(&device_list_lock);

	return status;
}

static const struct file_operations drv_fops = {
	.owner          = THIS_MODULE,
	/* REVISIT switch to aio primitives, so that userspace
	 * gets more complete API coverage.  It'll simplify things
	 * too, except for the locking.
	 */
	.write          = drv_write,
	.read           = drv_read,
	.unlocked_ioctl = drv_ioctl,
	.compat_ioctl   = drv_compat_ioctl,
	.open           = drv_open,
	.release        = drv_release,
	.llseek         = no_llseek,
};

/*-------------------------------------------------------------------------*/

/* The main reason to have this class is to make mdev/udev create the
 * /dev/spidevB.C character device nodes exposing our userspace API.
 * It also simplifies memory management.
 */

/*-------------------------------------------------------------------------*/
irqreturn_t SX1278_OnDio0Irq(int irq, void *dev_id)
{
    return IRQ_HANDLED;
}

irqreturn_t SX1278_OnDio1Irq(int irq, void *dev_id)
{
    return IRQ_HANDLED;
}

irqreturn_t SX1278_OnDio2Irq(int irq, void *dev_id)
{
    return IRQ_HANDLED;
}

irqreturn_t SX1278_OnDio3Irq(int irq, void *dev_id)
{
    return IRQ_HANDLED;
}

irqreturn_t SX1278_OnDio4Irq(int irq, void *dev_id);
{
    return IRQ_HANDLED;
}

/*!
 * Hardware DIO IRQ callback initialization
 */
DioIrqHandler *DioIrq[] = { SX1278_OnDio0Irq, SX1278_OnDio1Irq,
                            SX1278_OnDio2Irq, SX1278_OnDio3Irq,
                            SX1278_OnDio4Irq, NULL };


static int drv_probe(struct spi_device *spi)
{
	int result = 0;
	unsigned char mode[2] = {0};
    drv_info_st_ptr drv_info_ptr;
	unsigned long		minor;
	struct device_node *np = spi->dev.of_node;

	/* Allocate driver data */
	drv_info_ptr = kzalloc(sizeof(*drv_info_ptr), GFP_KERNEL);
	if(!drv_info_ptr)
		return -ENOMEM;
    
    drv_info_ptr->sx2178_gpio.DIO0 = of_get_named_gpio(np, "dio0-gpios", 0);
    if(drv_info_ptr->sx2178_gpio.DIO0 < 0) 
    {
        printk(KERN_ERR "%s %d Get DIO0 pin error\n", __FUNCTION__, __LINE__);
        result = -EINVAL;
        goto ERR_EXIT;
    }
    
    drv_info_ptr->sx2178_gpio.DIO1 = of_get_named_gpio(np, "dio1-gpios", 0);
    if(drv_info_ptr->sx2178_gpio.DIO1 < 0) 
    {
        printk(KERN_ERR "%s %d Get DIO1 pin error\n", __FUNCTION__, __LINE__);
        result = -EINVAL;
        goto ERR_EXIT;
    }
    
    drv_info_ptr->sx2178_gpio.DIO2 = of_get_named_gpio(np, "dio2-gpios", 0);
    if(drv_info_ptr->sx2178_gpio.DIO2 < 0) 
    {
        printk(KERN_ERR "%s %d Get DIO2 pin error\n", __FUNCTION__, __LINE__);
        result = -EINVAL;
        goto ERR_EXIT;
    }
    
    drv_info_ptr->sx2178_gpio.DIO3 = of_get_named_gpio(np, "dio3-gpios", 0);
    if(drv_info_ptr->sx2178_gpio.DIO3 < 0)
    {
        printk(KERN_ERR "%s %d Get DIO3 pin error\n", __FUNCTION__, __LINE__);
        result = -EINVAL;
        goto ERR_EXIT;
    }
    
    drv_info_ptr->sx2178_gpio.DIO4 = of_get_named_gpio(np, "dio4-gpios", 0);
    if(drv_info_ptr->sx2178_gpio.DIO4 < 0) 
    {
        printk(KERN_ERR "%s %d Get DIO4 pin error\n", __FUNCTION__, __LINE__);
        result = -EINVAL;
        goto ERR_EXIT;
    }
    
    drv_info_ptr->sx2178_gpio.Reset= of_get_named_gpio(np, "reset-gpios", 0);
    if(drv_info_ptr->sx2178_gpio.Reset < 0) 
    {
        printk(KERN_ERR "%s %d Get Reset pin error\n", __FUNCTION__, __LINE__);
        result = -EINVAL;
        goto ERR_EXIT;
    }
    
	/* Initialize the driver data */
	drv_info_ptr->spi = spi;
	spin_lock_init(&drv_info_ptr->spi_lock);
	mutex_init(&drv_info_ptr->buf_lock);

	INIT_LIST_HEAD(&drv_info_ptr->device_entry);

	/* If we can allocate a minor number, hook up this device.
	 * Reusing minors is fine so long as udev or mdev is working.
	 */
	mutex_lock(&device_list_lock);
	minor = find_first_zero_bit(minors, N_SPI_MINORS);
	if (minor < N_SPI_MINORS) {
		struct device *dev;
        
		drv_info_ptr->devt = MKDEV(DRV_MAJOR, minor);
		dev = device_create(drv_class, &spi->dev, drv_info_ptr->devt,
				    drv_info_ptr, "sx1278%d.%d",
				    spi->master->bus_num, spi->chip_select);
		result = PTR_ERR_OR_ZERO(dev);
	} else {
		dev_dbg(&spi->dev, "no minor number available!\n");
		result = -ENODEV;
	}
	if (result == 0) {
		set_bit(minor, minors);
		list_add(&drv_info_ptr->device_entry, &device_list);
	}
	mutex_unlock(&device_list_lock);

	//初始化等待队列
	init_waitqueue_head(&(drv_info_ptr->lora_wq));
    INIT_DELAYED_WORK(&drv_info_ptr->lora_work, lora_work_func);

	if (result == 0)
		spi_set_drvdata(spi, drv_info_ptr);
	else
		kfree(drv_info_ptr);

    /*config spi mode*/
	spi->mode = SPI_MODE_0;
	spi_setup(spi);	

    result = SX1278_Init(drv_info_ptr->spi, &drv_info_ptr->sx2178_gpio);
	if (result < 0) 
    {
		goto ERR_EXIT1;
	}

    return result;
    
ERR_EXIT1:
    device_destroy(drv_class, drv_info_ptr->devt);
ERR_EXIT:
    if(drv_info_ptr)
        kfree(drv_info_ptr);
    
	return result;
}

static int drv_remove(struct spi_device *spi)
{
	drv_info_st_ptr drv_info_ptr = spi_get_drvdata(spi);

	/* make sure ops on existing fds can abort cleanly */
	spin_lock_irq(&drv_info_ptr->spi_lock);
	drv_info_ptr->spi = NULL;
	spin_unlock_irq(&drv_info_ptr->spi_lock);

	/* prevent new opens */
	mutex_lock(&device_list_lock);
	list_del(&drv_info_ptr->device_entry);
	device_destroy(drv_class, drv_info_ptr->devt);
	clear_bit(MINOR(drv_info_ptr->devt), minors);
	if (drv_info_ptr->users == 0)
		kfree(drv_info_ptr);
	mutex_unlock(&device_list_lock);

	return 0;
}

static const struct of_device_id drv_dt_ids[] = {
	{ .compatible = "lora,sx1278" },
	{},
};

MODULE_DEVICE_TABLE(of, drv_dt_ids);

static struct spi_driver sx1278_spi_driver = {
	.driver = {
		.name =		"sx1278",
		.owner =	THIS_MODULE,
		.of_match_table = of_match_ptr(drv_dt_ids),
	},
	.probe =	drv_probe,
	.remove =	drv_remove,

	/* NOTE:  suspend/resume methods are not necessary here.
	 * We don't do anything except pass the requests to/from
	 * the underlying controller.  The refrigerator handles
	 * most issues; the controller driver handles the rest.
	 */
};

/*-------------------------------------------------------------------------*/

static int __init drv_init(void)
{
    int result = 0;

	/* Claim our 256 reserved device numbers.  Then register a class
	 * that will key udev/mdev to add/remove /dev nodes.  Last, register
	 * the driver which manages those device numbers.
	 */
	//BUILD_BUG_ON(N_SPI_MINORS > 256);

    //申请设备号
    result = register_chrdev_region(drv_dev_num, DRV_MINOR, DRV_NAME);
    if (result)
    {
        goto ERR_EXIT;
    }
    
    //注册字符设备
    cdev_init(&drv_cdev, &drv_fops);
    drv_cdev.owner = THIS_MODULE;
    result = cdev_add(&drv_cdev, drv_dev_num, DRV_MINOR);
    if (result)   
    {        
        goto ERR_EXIT0;    
    }
    
    //创建设备节点    
    drv_class = class_create(THIS_MODULE, DRV_NAME);    
    if (drv_class == NULL) 
    {        
        result = -ENOMEM;        
        goto ERR_EXIT1;    
    }

	result = spi_register_driver(&sx1278_spi_driver);
	if (result < 0)
    {
		goto ERR_EXIT2;
	}

    return result;
ERR_EXIT2:
    class_destroy(drv_class);
ERR_EXIT1:
    cdev_del(&drv_cdev);
ERR_EXIT0:
    unregister_chrdev_region(drv_dev_num, DRV_MINOR);
ERR_EXIT:    
	return result;
}

module_init(drv_init);

static void __exit drv_exit(void)
{
	spi_unregister_driver(&sx1278_spi_driver);
    class_destroy(drv_class);
    cdev_del(&drv_cdev);
    unregister_chrdev_region(drv_dev_num, DRV_MINOR);
}
module_exit(drv_exit);

MODULE_AUTHOR("pengrui, <pengrui_2009@163.com>");
MODULE_DESCRIPTION("driver of lora sx1278 ");
MODULE_LICENSE("GPL");
MODULE_ALIAS("lora:sx1278");
