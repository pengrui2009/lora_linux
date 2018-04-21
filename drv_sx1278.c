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
#include <linux/errno.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/of_irq.h>
#include <linux/irqreturn.h>
#include <linux/interrupt.h>
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
#define RSSI_OFFSET_LF                              -164
#define RSSI_OFFSET_HF                              -157
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
 *    is completely incompatible with the shared bus model
 *  - READY ... transfers may proceed when they shouldn't.
 *
 * REVISIT should changing those flags be privileged?
 */
#define SPI_MODE_MASK        (SPI_CPHA | SPI_CPOL | SPI_CS_HIGH \
                | SPI_LSB_FIRST | SPI_3WIRE | SPI_LOOP \
                | SPI_NO_CS | SPI_READY)



static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

static dev_t drv_dev_num = MKDEV(DRV_MAJOR, 0);
static struct cdev drv_cdev;
static struct class *drv_class;

//static drv_info_st drv_info;

static unsigned bufsiz = 4096;

//extern RadioRegisters_t RadioRegsInit[];

module_param(bufsiz, uint, S_IRUGO);
MODULE_PARM_DESC(bufsiz, "data bytes in biggest supported SPI message");

int Lora_Reset(SX1278_Gpio_st_ptr sx1278_gpio_ptr);

int Lora_IoInit(SX1278_Gpio_st_ptr sx1278_gpio_ptr);

int Lora_IoDeInit(SX1278_Gpio_st_ptr sx1278_gpio_ptr);

int Lora_IoIrqInit(SX1278_Gpio_st_ptr sx1278_gpio_ptr);

int Lora_IoIrqDeInit(SX1278_Gpio_st_ptr sx1278_gpio_ptr);

static void lora_work_func(struct work_struct *w)
{
    uint8_t i,val = 0;

    struct delayed_work *dw = to_delayed_work(w);
    drv_info_st_ptr lora_ptr = container_of(dw, drv_info_st, delayed_work);
    
    switch( drv_info.sx1278_state )
    {
    case RF_RX_RUNNING:
#if 0
        //if( drv_info.sx1278_cfg.Modem == MODEM_FSK )
        {
            //drv_info.sx1278_cfg.FskPacketHandler.PreambleDetected = false;
            //drv_info.sx1278_cfg.FskPacketHandler.SyncWordDetected = false;
            //drv_info.sx1278_cfg.FskPacketHandler.NbBytes = 0;
            //drv_info.sx1278_cfg.FskPacketHandler.Size = 0;

            // Clear Irqs
            SX1278_Write_Reg( REG_IRQFLAGS1, RF_IRQFLAGS1_RSSI |
                                        RF_IRQFLAGS1_PREAMBLEDETECT |
                                        RF_IRQFLAGS1_SYNCADDRESSMATCH );
            SX1278_Write_Reg( REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN );

            if( drv_info.sx1278_cfg.Fsk.RxContinuous == true )
            {
                // Continuous mode restart Rx chain
                SX1278_Read_Reg( REG_RXCONFIG, &val);
                SX1278_Write_Reg( REG_RXCONFIG, val | RF_RXCONFIG_RESTARTRXWITHOUTPLLLOCK );
                //TimerStart( &RxTimeoutSyncWord );
            }
            else
            {
                drv_info.sx1278_cfg.State = RF_IDLE;
                //TimerStop( &RxTimeoutSyncWord );
            }
        }
        /*
        if( ( RadioEvents != NULL ) && ( RadioEvents->RxTimeout != NULL ) )
        {
            RadioEvents->RxTimeout( );
        }
        */
#endif
        break;
    case RF_TX_RUNNING:
        // Tx timeout shouldn't happen.
        // But it has been observed that when it happens it is a result of a corrupted SPI transfer
        // it depends on the platform design.
        //
        // The workaround is to put the radio in a known state. Thus, we re-initialize it.
#if 0
        // BEGIN WORKAROUND

        // Reset the radio
        Lora_Reset(&drv_info.sx1278_gpio);

        // Calibrate Rx chain
        RxChainCalibration( );

        // Initialize radio default values
        SX1278SetOpMode( RF_OPMODE_SLEEP );

        for( i = 0; i < sizeof( RadioRegsInit ) / sizeof( RadioRegisters_t ); i++ )
        {
            SX1278SetModem( RadioRegsInit[i].Modem );
            SX1278_Write_Reg( RadioRegsInit[i].Addr, RadioRegsInit[i].Value );
        }
        
        SX1278SetModem( MODEM_FSK );

        // Restore previous network type setting.
        SX1278SetPublicNetwork( drv_info.sx1278_cfg.PublicNetwork );
        // END WORKAROUND
#endif
        drv_info.sx1278_state = RF_IDLE;
        
        complete(&drv_info.lora_complete);
        /*
        if( ( RadioEvents != NULL ) && ( RadioEvents->TxTimeout != NULL ) )
        {
            RadioEvents->TxTimeout( );
        }
        */
        break;
    default:
        break;
    }

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

    spin_lock_irq(&drv_info.spi_lock);
    if (drv_info.spi == NULL)
        status = -ESHUTDOWN;
    else
        status = spi_async(drv_info.spi, message);
    spin_unlock_irq(&drv_info.spi_lock);

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
    struct spi_transfer    t = {
            .tx_buf        = drv_info.buffer,
            .len        = len,
        };
    struct spi_message    m;

    spi_message_init(&m);
    spi_message_add_tail(&t, &m);
    return spidev_sync(drv_info_ptr, &m);
}

static inline ssize_t spidev_sync_read(drv_info_st_ptr drv_info_ptr, size_t len)
{
    struct spi_transfer    t = {
            .rx_buf        = drv_info.buffer,
            .len        = len,
        };
    struct spi_message    m;

    spi_message_init(&m);
    spi_message_add_tail(&t, &m);
    return spidev_sync(drv_info_ptr, &m);
}

/* Read-only message with current device setup */
static ssize_t drv_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
    int i = 0;
    uint8_t rxcount = 0;
    int result = 0;

    if(count > 256)
    {
        rxcount = 256;
    }else{
        rxcount = count;
    }

    wait_event_interruptible(drv_info.lora_rq, drv_info.lora_recv_state);

    drv_info.lora_recv_state = 0;

    drv_info.buffer = kzalloc(count, GFP_KERNEL);
    if(!drv_info.buffer)
        goto ERR_EXIT;

    result = SX1278GetRxPacket(drv_info.buffer, rxcount);

    copy_to_user(buf, drv_info.buffer, result);

    if( drv_info.sx1278_cfg.rx_cfg.RxSingleOn == true ) // Rx single mode
    {
        //RFLRState = RFLR_STATE_RX_INIT;
        SX1278StartRx();
    }
    else // Rx continuous mode
    {
        //RFLRState = RFLR_STATE_RX_RUNNING;
    }

    

ERR_EXIT:

    return result;
#if 0    
    //drv_info_st_ptr    drv_info_ptr;
    //ssize_t            status = 0;

    /* chipselect only toggles at start or end of operation */
    //if (count > bufsiz)
    //    return -EMSGSIZE;

    //drv_info_ptr = filp->private_data;

    //mutex_lock(&drv_info.buf_lock);
    //status = spidev_sync_read(drv_info_ptr, count);
    //if (status > 0) {
    //    unsigned long    missing;

    //    missing = copy_to_user(buf, drv_info.buffer, status);
    //    if (missing == status)
    //        status = -EFAULT;
    //    else
    //        status = status - missing;
    //}
    
    mutex_lock(&drv_info.lora_mutex);
    wait_event_interruptible(drv_info.lora_wq, drv_info.flag);
    drv_info.flag = 0x00;
    //set the mutex lock
    drv_info.buffer = kzalloc(count, GFP_KERNEL);
    
    SX1278Receive( drv_info.buffer, count);
    cancel_delayed_work_sync(&drv_info.lora_work);
    //read data
    copy_to_user(buf, drv_info.buffer, result);
    //set the mutex unlock
    
    
    mutex_unlock(&drv_info.lora_mutex);
  
    return result;
#endif  
}

/* Write-only message with current device setup */
static ssize_t drv_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
    int result = 0;
    uint32_t timeout = 0;
    int i = 0;
    uint8_t senddata[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 ,11, 12, 13, 14, 15, 16, 17, 18, 19 };
    drv_info.buffer = kzalloc(count, GFP_KERNEL);
    if(!drv_info.buffer)
    {
        result= -ENOMEM;
        goto ERR_EXIT;
    }
  
    result = copy_from_user(drv_info.buffer, buf, count);
    if (result == 0)
    {
        for(i=0; i<count; i+=255)
        {
            uint8_t val = 0;
            uint8_t buffer_size = ((i + 255) < count) ? 256 : (count - i);
            //SX1278SetTxPacket(drv_info.buffer + i, buffer_size);
            SX1278SetTxPacket(senddata, 20);
            SX1278StartTx();
            drv_info.sx1278_state = RF_TX_RUNNING;
            timeout = drv_info.sx1278_cfg.tx_cfg.TxPacketTimeout;
            schedule_delayed_work(&drv_info.lora_work, timeout * HZ);

            //wait_for_completion(&drv_info.lora_complete);
            wait_event_interruptible(drv_info.lora_wq, drv_info.lora_send_state);
            drv_info.lora_send_state = 0;

            SX1278TxFinished();
            drv_info.sx1278_state = RF_IDLE;
        }
    }

    SX1278StartRx();
    drv_info.sx1278_state =RF_RX_RUNNING;
ERR_EXIT:

    if(drv_info.buffer)
        kfree(drv_info.buffer);

    return result;
#if 0
    uint32_t timeout = 0;
    //drv_info_st_ptr    drv_info_ptr = NULL;
    ssize_t            status = 0;
    //unsigned long        missing;

    /* chipselect only toggles at start or end of operation */
    //if (count > bufsiz)
    //    return -EMSGSIZE;

    //drv_info_ptr = filp->private_data;
    drv_info.buffer = kzalloc(count, GFP_KERNEL);
    if(!drv_info.buffer)
    {
        goto ERR_EXIT;
    }

    mutex_lock(&drv_info.lora_mutex);
    result = copy_from_user(drv_info.buffer, buf, count);
    if (result == 0)
    {
        //result = spidev_sync_write(drv_info_ptr, count);
        SX1278SetChannel(drv_info.sx1278_cfg.Channel);

        SX1278SetTxConfig(MODEM_LORA, drv_info.sx1278_cfg.TxLoRa.Power, 0, \
                                   drv_info.sx1278_cfg.TxLoRa.Bandwidth, drv_info.sx1278_cfg.TxLoRa.Datarate, \
                                   drv_info.sx1278_cfg.TxLoRa.Coderate, drv_info.sx1278_cfg.TxLoRa.PreambleLen, \
                                   drv_info.sx1278_cfg.TxLoRa.FixLen, drv_info.sx1278_cfg.TxLoRa.CrcOn, drv_info.sx1278_cfg.TxLoRa.FreqHopOn, \
                                   drv_info.sx1278_cfg.TxLoRa.HopPeriod, drv_info.sx1278_cfg.TxLoRa.IqInverted, drv_info.sx1278_cfg.TxLoRa.TxTimeout);

        //data split
        SX1278Send(drv_info.buffer, count);

        drv_info.sx1278_cfg.State = RF_TX_RUNNING;
        //TimerStart( &TxTimeoutTimer );
        timeout = drv_info.sx1278_cfg.TxLoRa.TxTimeout;
        schedule_delayed_work(&drv_info.lora_work, timeout * HZ / 1000);
        
        SX1278SetOpMode( RF_OPMODE_TRANSMITTER );
        
        wait_for_completion(drv_info.lora_complete);

        if(drv_info.flag)
        {
            result = -1;
            drv_info.flag = 0;
        }
    }else{
        result = -EFAULT;
    }
    mutex_unlock(&drv_info.lora_mutex);


ERR_EXIT:

    if(drv_info.buffer)
    {
        kfree(drv_info.buffer);
    }
    
    return result;
#endif

}

static int spidev_message(drv_info_st_ptr drv_info_ptr, struct spi_ioc_transfer *u_xfers, unsigned n_xfers)
{
    struct spi_message    msg;
    struct spi_transfer    *k_xfers;
    struct spi_transfer    *k_tmp;
    struct spi_ioc_transfer *u_tmp;
    unsigned        n, total;
    u8            *buf;
    int            status = -EFAULT;

    spi_message_init(&msg);
    k_xfers = kcalloc(n_xfers, sizeof(*k_tmp), GFP_KERNEL);
    if (k_xfers == NULL)
        return -ENOMEM;

    /* Construct spi_message, copying any tx data to bounce buffer.
     * We walk the array of user-provided transfers, using each one
     * to initialize a kernel version of the same transfer.
     */
    buf = drv_info.buffer;
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
            u_tmp->bits_per_word ? : drv_info.spi->bits_per_word,
            u_tmp->delay_usecs,
            u_tmp->speed_hz ? : drv_info.spi->max_speed_hz);
#endif
        spi_message_add_tail(k_tmp, &msg);
    }

    status = spidev_sync(drv_info_ptr, &msg);
    if (status < 0)
        goto done;

    /* copy any rx data out of bounce buffer */
    buf = drv_info.buffer;
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
    int            err = 0;
    int            retval = 0;
    drv_info_st_ptr    drv_info_ptr;
    struct spi_device    *spi;
    u32            tmp;
    unsigned        n_ioc;
    struct spi_ioc_transfer    *ioc;

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
    spin_lock_irq(&drv_info.spi_lock);
    spi = spi_dev_get(drv_info.spi);
    spin_unlock_irq(&drv_info.spi_lock);

    if (spi == NULL)
        return -ESHUTDOWN;

    /* use the buffer lock here for triple duty:
     *  - prevent I/O (from us) so calling spi_setup() is safe;
     *  - prevent concurrent SPI_IOC_WR_* from morphing
     *    data fields while SPI_IOC_RD_* reads them;
     *  - SPI_IOC_MESSAGE needs the buffer locked "normally".
     */
    mutex_lock(&drv_info.buf_lock);

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
            u8    save = spi->mode;

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
            u8    save = spi->mode;

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
            u8    save = spi->bits_per_word;

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
            u32    save = spi->max_speed_hz;

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

    mutex_unlock(&drv_info.buf_lock);
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

static unsigned int drv_poll(struct file *file, poll_table *wait)  
{      
    unsigned int mask = 0;        
    /* 该函数，只是将进程挂在button_waitq队列上，而不是立即休眠 */      
    poll_wait(file, &drv_info.lora_wq, wait);        
    /* 当没有按键按下时，即不会进入按键中断处理函数，此时ev_press = 0       
     * 当按键按下时，就会进入按键中断处理函数，此时ev_press被设置为1      
     */      
    if(drv_info.lora_recv_state)      
    {          
        mask |= POLLIN | POLLRDNORM;  /* 表示有数据可读 */      
    }        
    /* 如果有按键按下时，mask |= POLLIN | POLLRDNORM,否则mask = 0 */      
    return mask;    
}  


static int drv_open(struct inode *inode, struct file *filp)
{
    drv_info_st_ptr    drv_info_ptr;
    int            status = -ENXIO;

    mutex_lock(&device_list_lock);

    list_for_each_entry(drv_info_ptr, &device_list, device_entry) {
        if (drv_info.devt == inode->i_rdev) {
            status = 0;
            break;
        }
    }
    if (status == 0) {
        if (!drv_info.buffer) {
            drv_info.buffer = kmalloc(bufsiz, GFP_KERNEL);
            if (!drv_info.buffer) {
                dev_dbg(&drv_info.spi->dev, "open/ENOMEM\n");
                status = -ENOMEM;
            }
        }
        if (status == 0) {
            drv_info.users++;
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
    drv_info_st_ptr    drv_info_ptr;
    int            status = 0;

    mutex_lock(&device_list_lock);
    drv_info_ptr = filp->private_data;
    filp->private_data = NULL;

    /* last close? */
    drv_info.users--;
    if (!drv_info.users) {
        int        dofree;

        kfree(drv_info.buffer);
        drv_info.buffer = NULL;

        /* ... after we unbound from the underlying device? */
        spin_lock_irq(&drv_info.spi_lock);
        dofree = (drv_info.spi == NULL);
        spin_unlock_irq(&drv_info.spi_lock);

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
    .poll           = drv_poll,
};

/*-------------------------------------------------------------------------*/

/* The main reason to have this class is to make mdev/udev create the
 * /dev/spidevB.C character device nodes exposing our userspace API.
 * It also simplifies memory management.
 */

/*-------------------------------------------------------------------------*/
irqreturn_t SX1278_OnDio0Irq(int irq, void *dev_id)
{
    printk(KERN_ERR "irq0\n");
    switch(drv_info.sx1278_state)
    {
    case RF_RX_RUNNING:
        {
            SX1278RxClearIrq();
            drv_info.lora_recv_state = 1;
            wake_up_interruptible(&drv_info.lora_rq); 
            break;
        }
    case RF_TX_RUNNING:
        {
            //SX1278TxFinished();
            drv_info.sx1278_state = RF_IDLE;
            //complete(&drv_info.lora_complete);
            drv_info.lora_send_state = 1;
            wake_up_interruptible(&drv_info.lora_wq); 
            break;
        }
    }
#if 0
    uint8_t val = 0;
    volatile uint8_t irqFlags = 0;

    switch( drv_info.sx1278_cfg.State )
    {
    case RF_RX_RUNNING:
        //TimerStop( &RxTimeoutTimer );
        // RxDone interrupt
        switch( drv_info.sx1278_cfg.Modem )
        {
        case MODEM_FSK:
            if( drv_info.sx1278_cfg.Fsk.CrcOn == true )
            {
                SX1278_Read_Reg( REG_IRQFLAGS2, &irqFlags);
                if( ( irqFlags & RF_IRQFLAGS2_CRCOK ) != RF_IRQFLAGS2_CRCOK )
                {
                    // Clear Irqs
                    SX1278_Write_Reg( REG_IRQFLAGS1, RF_IRQFLAGS1_RSSI |
                                                RF_IRQFLAGS1_PREAMBLEDETECT |
                                                RF_IRQFLAGS1_SYNCADDRESSMATCH );
                    SX1278_Write_Reg( REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN );

                    //TimerStop( &RxTimeoutTimer );

                    if( drv_info.sx1278_cfg.Fsk.RxContinuous == false )
                    {
                        //TimerStop( &RxTimeoutSyncWord );
                        drv_info.sx1278_cfg.State = RF_IDLE;
                    }
                    else
                    {
                        // Continuous mode restart Rx chain
                        SX1278_Read_Reg( REG_RXCONFIG, &val );
                        SX1278_Write_Reg( REG_RXCONFIG, val | RF_RXCONFIG_RESTARTRXWITHOUTPLLLOCK );
                        //TimerStart( &RxTimeoutSyncWord );
                    }
                    /*
                    if( ( RadioEvents != NULL ) && ( RadioEvents->RxError != NULL ) )
                    {
                        RadioEvents->RxError( );
                    }
                    */
                    drv_info.sx1278_cfg.FskPacketHandler.PreambleDetected = false;
                    drv_info.sx1278_cfg.FskPacketHandler.SyncWordDetected = false;
                    drv_info.sx1278_cfg.FskPacketHandler.NbBytes = 0;
                    drv_info.sx1278_cfg.FskPacketHandler.Size = 0;
                    break;
                }
            }

            // Read received packet size
            if( ( drv_info.sx1278_cfg.FskPacketHandler.Size == 0 ) && ( drv_info.sx1278_cfg.FskPacketHandler.NbBytes == 0 ) )
            {
                if( drv_info.sx1278_cfg.Fsk.FixLen == false )
                {
                    SX1278_Read_FIFO( ( uint8_t* )&drv_info.sx1278_cfg.FskPacketHandler.Size, 1 );
                }
                else
                {
                    SX1278_Read_Reg( REG_PAYLOADLENGTH, &drv_info.sx1278_cfg.FskPacketHandler.Size);
                }
                SX1278_Read_FIFO( drv_info.buffer + drv_info.sx1278_cfg.FskPacketHandler.NbBytes, drv_info.sx1278_cfg.FskPacketHandler.Size - drv_info.sx1278_cfg.FskPacketHandler.NbBytes );
                drv_info.sx1278_cfg.FskPacketHandler.NbBytes += ( drv_info.sx1278_cfg.FskPacketHandler.Size - drv_info.sx1278_cfg.FskPacketHandler.NbBytes );
            }
            else
            {
                SX1278_Read_FIFO( drv_info.buffer + drv_info.sx1278_cfg.FskPacketHandler.NbBytes, drv_info.sx1278_cfg.FskPacketHandler.Size - drv_info.sx1278_cfg.FskPacketHandler.NbBytes );
                drv_info.sx1278_cfg.FskPacketHandler.NbBytes += ( drv_info.sx1278_cfg.FskPacketHandler.Size - drv_info.sx1278_cfg.FskPacketHandler.NbBytes );
            }

            //TimerStop( &RxTimeoutTimer );

            if( drv_info.sx1278_cfg.Fsk.RxContinuous == false )
            {
                drv_info.sx1278_cfg.State = RF_IDLE;
                //TimerStop( &RxTimeoutSyncWord );
            }
            else
            {
                // Continuous mode restart Rx chain
        SX1278_Read_Reg(REG_RXCONFIG, &val);
                SX1278_Write_Reg( REG_RXCONFIG, val | RF_RXCONFIG_RESTARTRXWITHOUTPLLLOCK );
                //TimerStart( &RxTimeoutSyncWord );
            }
            /*
            if( ( RadioEvents != NULL ) && ( RadioEvents->RxDone != NULL ) )
            {
                RadioEvents->RxDone( RxTxBuffer, SX1278.Settings.FskPacketHandler.Size, SX1278.Settings.FskPacketHandler.RssiValue, 0 );
            }
            */
            drv_info.sx1278_cfg.FskPacketHandler.PreambleDetected = false;
            drv_info.sx1278_cfg.FskPacketHandler.SyncWordDetected = false;
            drv_info.sx1278_cfg.FskPacketHandler.NbBytes = 0;
            drv_info.sx1278_cfg.FskPacketHandler.Size = 0;
            break;
        case MODEM_LORA:
            {
                int8_t snr = 0;

                // Clear Irq
                SX1278_Write_Reg( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXDONE );

                SX1278_Read_Reg( REG_LR_IRQFLAGS, &irqFlags );
                if( ( irqFlags & RFLR_IRQFLAGS_PAYLOADCRCERROR_MASK ) == RFLR_IRQFLAGS_PAYLOADCRCERROR )
                {
                    // Clear Irq
                    SX1278_Write_Reg( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_PAYLOADCRCERROR );

                    if( drv_info.sx1278_cfg.RxLoRa.RxContinuous == false )
                    {
                        drv_info.sx1278_cfg.State = RF_IDLE;
                    }
                    //TimerStop( &RxTimeoutTimer );
                    /*
                    if( ( RadioEvents != NULL ) && ( RadioEvents->RxError != NULL ) )
                    {
                        RadioEvents->RxError( );
                    }
                    */
                    break;
                }

                SX1278_Read_Reg( REG_LR_PKTSNRVALUE, &drv_info.sx1278_cfg.LoRaPacketHandler.SnrValue);
                if( drv_info.sx1278_cfg.LoRaPacketHandler.SnrValue & 0x80 ) // The SNR sign bit is 1
                {
                    // Invert and divide by 4
                    snr = ( ( ~drv_info.sx1278_cfg.LoRaPacketHandler.SnrValue + 1 ) & 0xFF ) >> 2;
                    snr = -snr;
                }
                else
                {
                    // Divide by 4
                    snr = ( drv_info.sx1278_cfg.LoRaPacketHandler.SnrValue & 0xFF ) >> 2;
                }

                int16_t rssi;
                SX1278_Read_Reg( REG_LR_PKTRSSIVALUE, &rssi );
                if( snr < 0 )
                {
                    if( drv_info.sx1278_cfg.Channel > RF_MID_BAND_THRESH )
                    {
                        drv_info.sx1278_cfg.LoRaPacketHandler.RssiValue = RSSI_OFFSET_HF + rssi + ( rssi >> 4 ) +
                                                                      snr;
                    }
                    else
                    {
                        drv_info.sx1278_cfg.LoRaPacketHandler.RssiValue = RSSI_OFFSET_LF + rssi + ( rssi >> 4 ) +
                                                                      snr;
                    }
                }
                else
                {
                    if( drv_info.sx1278_cfg.Channel > RF_MID_BAND_THRESH )
                    {
                        drv_info.sx1278_cfg.LoRaPacketHandler.RssiValue = RSSI_OFFSET_HF + rssi + ( rssi >> 4 );
                    }
                    else
                    {
                        drv_info.sx1278_cfg.LoRaPacketHandler.RssiValue = RSSI_OFFSET_LF + rssi + ( rssi >> 4 );
                    }
                }

                SX1278_Read_Reg( REG_LR_RXNBBYTES, &drv_info.sx1278_cfg.LoRaPacketHandler.Size );
                SX1278_Read_Reg( REG_LR_FIFORXCURRENTADDR, &val);
                SX1278_Write_Reg( REG_LR_FIFOADDRPTR,  val);
                SX1278_Read_FIFO( drv_info.buffer, drv_info.sx1278_cfg.LoRaPacketHandler.Size );

                if( drv_info.sx1278_cfg.RxLoRa.RxContinuous == false )
                {
                    drv_info.sx1278_cfg.State = RF_IDLE;
                }
                //TimerStop( &RxTimeoutTimer );
                /*
                if( ( RadioEvents != NULL ) && ( RadioEvents->RxDone != NULL ) )
                {
                    RadioEvents->RxDone( RxTxBuffer, SX1278.Settings.LoRaPacketHandler.Size, SX1278.Settings.LoRaPacketHandler.RssiValue, SX1278.Settings.LoRaPacketHandler.SnrValue );
                }
                */
            }
            break;
        default:
            break;
        }
        break;
    case RF_TX_RUNNING:
        //TimerStop( &TxTimeoutTimer );
        cancel_delayed_work_sync(&drv_info.lora_work);
        // TxDone interrupt
        switch( drv_info.sx1278_cfg.Modem )
        {
        case MODEM_LORA:
            // Clear Irq
            SX1278_Write_Reg( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_TXDONE );
            // Intentional fall through
            complete(&drv_info.lora_complete);
        case MODEM_FSK:
        default:
            drv_info.sx1278_cfg.State = RF_IDLE;
            /*
            if( ( RadioEvents != NULL ) && ( RadioEvents->TxDone != NULL ) )
            {
                RadioEvents->TxDone( );
            }
            */
            break;
        }
        break;
    default:
        break;
    }
#endif    
    return IRQ_HANDLED;
}

irqreturn_t SX1278_OnDio1Irq(int irq, void *dev_id)
{
    printk(KERN_ERR "irq1\n");
#if 0
    switch( drv_info.lora_state )
    {
        case RF_RX_RUNNING:
            switch( drv_info.sx1278_cfg.Modem )
            {
            case MODEM_FSK:
                // FifoLevel interrupt
                // Read received packet size
                if( ( drv_info.sx1278_cfg.FskPacketHandler.Size == 0 ) && ( drv_info.sx1278_cfg.FskPacketHandler.NbBytes == 0 ) )
                {
                    if( drv_info.sx1278_cfg.Fsk.FixLen == false )
                    {
                        SX1278_Read_FIFO( ( uint8_t* )&drv_info.sx1278_cfg.FskPacketHandler.Size, 1 );
                    }
                    else
                    {
                        SX1278_Read_Reg( REG_PAYLOADLENGTH, &drv_info.sx1278_cfg.FskPacketHandler.Size );
                    }
                }

                if( ( drv_info.sx1278_cfg.FskPacketHandler.Size - drv_info.sx1278_cfg.FskPacketHandler.NbBytes ) > drv_info.sx1278_cfg.FskPacketHandler.FifoThresh )
                {
                    SX1278_Read_FIFO( ( drv_info.buffer + drv_info.sx1278_cfg.FskPacketHandler.NbBytes ), drv_info.sx1278_cfg.FskPacketHandler.FifoThresh );
                    drv_info.sx1278_cfg.FskPacketHandler.NbBytes += drv_info.sx1278_cfg.FskPacketHandler.FifoThresh;
                }
                else
                {
                    SX1278_Read_FIFO( ( drv_info.buffer + drv_info.sx1278_cfg.FskPacketHandler.NbBytes ), drv_info.sx1278_cfg.FskPacketHandler.Size - drv_info.sx1278_cfg.FskPacketHandler.NbBytes );
                    drv_info.sx1278_cfg.FskPacketHandler.NbBytes += ( drv_info.sx1278_cfg.FskPacketHandler.Size - drv_info.sx1278_cfg.FskPacketHandler.NbBytes );
                }
                break;
            case MODEM_LORA:
                // Sync time out
                //TimerStop( &RxTimeoutTimer );
                // Clear Irq
                SX1278_Write_Reg( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXTIMEOUT );

                drv_info.sx1278_cfg.State = RF_IDLE;
                /*
                if( ( RadioEvents != NULL ) && ( RadioEvents->RxTimeout != NULL ) )
                {
                    RadioEvents->RxTimeout( );
                }
                */
                break;
            default:
                break;
            }
            break;
        case RF_TX_RUNNING:
            switch( drv_info.sx1278_cfg.Modem )
            {
            case MODEM_FSK:
                // FifoEmpty interrupt
                if( ( drv_info.sx1278_cfg.FskPacketHandler.Size - drv_info.sx1278_cfg.FskPacketHandler.NbBytes ) > drv_info.sx1278_cfg.FskPacketHandler.ChunkSize )
                {
                    SX1278_Write_FIFO( ( drv_info.buffer + drv_info.sx1278_cfg.FskPacketHandler.NbBytes ), drv_info.sx1278_cfg.FskPacketHandler.ChunkSize );
                    drv_info.sx1278_cfg.FskPacketHandler.NbBytes += drv_info.sx1278_cfg.FskPacketHandler.ChunkSize;
                }
                else
                {
                    // Write the last chunk of data
                    SX1278_Write_FIFO( drv_info.buffer + drv_info.sx1278_cfg.FskPacketHandler.NbBytes, drv_info.sx1278_cfg.FskPacketHandler.Size - drv_info.sx1278_cfg.FskPacketHandler.NbBytes );
                    drv_info.sx1278_cfg.FskPacketHandler.NbBytes += drv_info.sx1278_cfg.FskPacketHandler.Size - drv_info.sx1278_cfg.FskPacketHandler.NbBytes;
                }
                break;
            case MODEM_LORA:
                break;
            default:
                break;
            }
            break;
        default:
            break;
    }
#endif
    return IRQ_HANDLED;
}

irqreturn_t SX1278_OnDio2Irq(int irq, void *dev_id)
{
    uint8_t val = 0, val1 = 0;
    printk(KERN_ERR "irq2\n");
#if 0    
    switch( drv_info.sx1278_cfg.State )
    {
        case RF_RX_RUNNING:
            switch( drv_info.sx1278_cfg.Modem )
            {
            case MODEM_FSK:
                // Checks if DIO4 is connected. If it is not PreambleDetected is set to true.
                //if( SX1278.DIO4.port == NULL )
                //{
                //    SX1278.Settings.FskPacketHandler.PreambleDetected = true;
                //}

                if( ( drv_info.sx1278_cfg.FskPacketHandler.PreambleDetected == true ) && ( drv_info.sx1278_cfg.FskPacketHandler.SyncWordDetected == false ) )
                {
                    //TimerStop( &RxTimeoutSyncWord );

                    drv_info.sx1278_cfg.FskPacketHandler.SyncWordDetected = true;

                    SX1278_Read_Reg( REG_RSSIVALUE, &val );
                    drv_info.sx1278_cfg.FskPacketHandler.RssiValue = -( val >> 1 );

                    SX1278_Read_Reg( REG_AFCMSB, &val);
                    SX1278_Read_Reg( REG_AFCLSB, &val1);
                    drv_info.sx1278_cfg.FskPacketHandler.AfcValue = ( int32_t )( double )( ( ( uint16_t )val << 8 ) |
                                                                           ( uint16_t )val1 ) *
                                                                           ( double )FREQ_STEP;
                    SX1278_Read_Reg( REG_LNA, &val );
                    drv_info.sx1278_cfg.FskPacketHandler.RxGain = ( val >> 5 ) & 0x07;
                }
                break;
            case MODEM_LORA:
                if( drv_info.sx1278_cfg.RxLoRa.FreqHopOn == true )
                {
                    // Clear Irq
                    SX1278_Write_Reg( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL );
                    /*
                    if( ( RadioEvents != NULL ) && ( RadioEvents->FhssChangeChannel != NULL ) )
                    {
                        RadioEvents->FhssChangeChannel( ( SX1278Read( REG_LR_HOPCHANNEL ) & RFLR_HOPCHANNEL_CHANNEL_MASK ) );
                    }
                    */
                }
                break;
            default:
                break;
            }
            break;
        case RF_TX_RUNNING:
            switch( drv_info.sx1278_cfg.Modem )
            {
            case MODEM_FSK:
                break;
            case MODEM_LORA:
                if( drv_info.sx1278_cfg.TxLoRa.FreqHopOn == true )
                {
                    // Clear Irq
                    SX1278_Write_Reg( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL );
                    /*
                    if( ( RadioEvents != NULL ) && ( RadioEvents->FhssChangeChannel != NULL ) )
                    {
                        RadioEvents->FhssChangeChannel( ( SX1278Read( REG_LR_HOPCHANNEL ) & RFLR_HOPCHANNEL_CHANNEL_MASK ) );
                    }
                    */
                }
                break;
            default:
                break;
            }
            break;
        default:
            break;
    }
#endif
    return IRQ_HANDLED;
}

irqreturn_t SX1278_OnDio3Irq(int irq, void *dev_id)
{
    uint8_t val = 0;
    printk(KERN_ERR "irq3\n");
#if 0
    switch( drv_info.sx1278_cfg.Modem )
    {
    case MODEM_FSK:
        break;
    case MODEM_LORA:
        SX1278_Read_Reg( REG_LR_IRQFLAGS , &val);
        
        if( ( val & RFLR_IRQFLAGS_CADDETECTED ) == RFLR_IRQFLAGS_CADDETECTED )
        {
            // Clear Irq
            SX1278_Write_Reg( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_CADDETECTED | RFLR_IRQFLAGS_CADDONE );
            /*
            if( ( RadioEvents != NULL ) && ( RadioEvents->CadDone != NULL ) )
            {
                RadioEvents->CadDone( true );
            }
            */
        }
        else
        {
            // Clear Irq
            SX1278_Write_Reg( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_CADDONE );
            /*
            if( ( RadioEvents != NULL ) && ( RadioEvents->CadDone != NULL ) )
            {
                RadioEvents->CadDone( false );
            }
            */
        }
        break;
    default:
        break;
    }
#endif
    return IRQ_HANDLED;
}

irqreturn_t SX1278_OnDio4Irq(int irq, void *dev_id)
{
    printk(KERN_ERR "irq4\n");
#if 0
    switch( drv_info.sx1278_cfg.Modem )
    {
    case MODEM_FSK:
        {
            if( drv_info.sx1278_cfg.FskPacketHandler.PreambleDetected == false )
            {
                drv_info.sx1278_cfg.FskPacketHandler.PreambleDetected = true;
            }
        }
        break;
    case MODEM_LORA:
        break;
    default:
        break;
    }
#endif
    return IRQ_HANDLED;
}

/*!
 * Hardware DIO IRQ callback initialization
 */

int Lora_IoInit(SX1278_Gpio_st_ptr sx1278_gpio_ptr)
{
    int result = 0;
    
    if(NULL == sx1278_gpio_ptr)
    {
        result = -ENOMEM;
        goto ERR_EXIT;
    }
    
    gpio_request(sx1278_gpio_ptr->DIO0, "DIO0");
    gpio_request(sx1278_gpio_ptr->DIO1, "DIO1");
    gpio_request(sx1278_gpio_ptr->DIO2, "DIO2");
    gpio_request(sx1278_gpio_ptr->DIO3, "DIO3");
    gpio_request(sx1278_gpio_ptr->DIO4, "DIO4");
    gpio_request(sx1278_gpio_ptr->Reset, "RESET");


    gpio_direction_input(sx1278_gpio_ptr->DIO0);
    gpio_direction_input(sx1278_gpio_ptr->DIO1);
    gpio_direction_input(sx1278_gpio_ptr->DIO2);
    gpio_direction_input(sx1278_gpio_ptr->DIO3);
    gpio_direction_input(sx1278_gpio_ptr->DIO4);
    gpio_direction_output(sx1278_gpio_ptr->Reset, 1);

ERR_EXIT:
    
    return result;
}

int Lora_IoIrqInit(SX1278_Gpio_st_ptr sx1278_gpio_ptr)
{
    int irq = 0;
    int result = 0;

    if(NULL == sx1278_gpio_ptr)
    {
        result = -ENOMEM;
        goto ERR_EXIT0;
    }

    irq = gpio_to_irq(sx1278_gpio_ptr->DIO0);
    result = request_irq(irq , SX1278_OnDio0Irq, IRQF_TRIGGER_RISING, "DI0_IRQ", NULL);
    if(result)     
    {       
        printk(KERN_ERR "%s %d request_irq ERROR\n", __FUNCTION__, __LINE__);
        result = -EFAULT;
        goto ERR_EXIT0; 
    }

    irq = gpio_to_irq(sx1278_gpio_ptr->DIO1);
    result = request_irq(irq , SX1278_OnDio1Irq, IRQF_TRIGGER_RISING, "DI1_IRQ", NULL);
    if(result)     
    {       
        printk(KERN_ERR "%s %d request_irq ERROR\n", __FUNCTION__, __LINE__);
        result = -EFAULT;
        goto ERR_EXIT1; 
    }

    irq = gpio_to_irq(sx1278_gpio_ptr->DIO2);
    result = request_irq(irq , SX1278_OnDio2Irq, IRQF_TRIGGER_RISING, "DI2_IRQ", NULL);
    if(result)     
    {       
        printk(KERN_ERR "%s %d request_irq ERROR\n", __FUNCTION__, __LINE__);
        result = -EFAULT;
        goto ERR_EXIT2; 
    }

    irq = gpio_to_irq(sx1278_gpio_ptr->DIO3);
    result = request_irq(irq , SX1278_OnDio3Irq, IRQF_TRIGGER_RISING, "DI3_IRQ", NULL);
    if(result)     
    {       
        printk(KERN_ERR "%s %d request_irq ERROR\n", __FUNCTION__, __LINE__);
        result = -EFAULT;
        goto ERR_EXIT3; 
    }

    irq = gpio_to_irq(sx1278_gpio_ptr->DIO4);
    result = request_irq(irq , SX1278_OnDio4Irq, IRQF_TRIGGER_RISING, "DI4_IRQ", NULL);
    if(result)     
    {       
        printk(KERN_ERR "%s %d request_irq ERROR\n", __FUNCTION__, __LINE__);
        result = -EFAULT;
        goto ERR_EXIT4; 
    }

    return result;

ERR_EXIT4:
    irq = gpio_to_irq(sx1278_gpio_ptr->DIO3);
    free_irq(irq, NULL);    
ERR_EXIT3:
    irq = gpio_to_irq(sx1278_gpio_ptr->DIO2);
    free_irq(irq, NULL);
ERR_EXIT2:
    irq = gpio_to_irq(sx1278_gpio_ptr->DIO1);
    free_irq(irq, NULL);
ERR_EXIT1:
    irq = gpio_to_irq(sx1278_gpio_ptr->DIO0);
    free_irq(irq, NULL);
ERR_EXIT0:
    return result;
}

int Lora_IoDeInit(SX1278_Gpio_st_ptr sx1278_gpio_ptr)
{
    int result = 0;

    if(NULL == sx1278_gpio_ptr)
    {
        result = -ENOMEM;
        goto ERR_EXIT;
    }

    gpio_free(sx1278_gpio_ptr->DIO0);
    gpio_free(sx1278_gpio_ptr->DIO1);
    gpio_free(sx1278_gpio_ptr->DIO2);
    gpio_free(sx1278_gpio_ptr->DIO3);
    gpio_free(sx1278_gpio_ptr->DIO4);
    gpio_free(sx1278_gpio_ptr->Reset);

ERR_EXIT:

    return result;
}

int Lora_IoIrqDeInit(SX1278_Gpio_st_ptr sx1278_gpio_ptr)
{
    int irq = 0;
    int result = 0;

    if(NULL == sx1278_gpio_ptr)
    {
        result = -ENOMEM;
        goto ERR_EXIT;
    }

    irq = gpio_to_irq(sx1278_gpio_ptr->DIO0);
    free_irq(irq, NULL);

    irq = gpio_to_irq(sx1278_gpio_ptr->DIO1);
    free_irq(irq, NULL);

    irq = gpio_to_irq(sx1278_gpio_ptr->DIO2);
    free_irq(irq, NULL);

    irq = gpio_to_irq(sx1278_gpio_ptr->DIO3);
    free_irq(irq, NULL);

    irq = gpio_to_irq(sx1278_gpio_ptr->DIO4);
    free_irq(irq, NULL);

ERR_EXIT:
    
    return result;
}


int Lora_Reset(SX1278_Gpio_st_ptr sx1278_gpio_ptr)
{
    int result = 0;

    if(NULL == sx1278_gpio_ptr)
    {
        result = -ENOMEM;
        goto ERR_EXIT;
    }

    // Set RESET pin to 0
    gpio_direction_output(sx1278_gpio_ptr->Reset, 0);

    // Wait 1 ms
    mdelay(1);

    // Configure RESET as input
    gpio_direction_input(sx1278_gpio_ptr->Reset);

    // Wait 6 ms
    mdelay(6);

ERR_EXIT:

    return result;
}


static int drv_probe(struct spi_device *spi)
{
    int result = 0;
    unsigned char mode[2] = {0};
    drv_info_st_ptr drv_info_ptr;
    lora_info_st_ptr lora_ptr = NULL;
    unsigned long        minor;
    struct device_node *np = spi->dev.of_node;

    /* Allocate driver data */
    lora_ptr = kzalloc(sizeof(*lora_ptr), GFP_KERNEL);
    if(!lora_ptr)
        return -ENOMEM;
    
    drv_info.sx1278_gpio.DIO0 = of_get_named_gpio(np, "dio0-gpios", 0);
    if(drv_info.sx1278_gpio.DIO0 < 0) 
    {
        printk(KERN_ERR "%s %d Get DIO0 pin error\n", __FUNCTION__, __LINE__);
        result = -EINVAL;
        goto ERR_EXIT;
    }
    
    drv_info.sx1278_gpio.DIO1 = of_get_named_gpio(np, "dio1-gpios", 0);
    if(drv_info.sx1278_gpio.DIO1 < 0) 
    {
        printk(KERN_ERR "%s %d Get DIO1 pin error\n", __FUNCTION__, __LINE__);
        result = -EINVAL;
        goto ERR_EXIT;
    }
    
    drv_info.sx1278_gpio.DIO2 = of_get_named_gpio(np, "dio2-gpios", 0);
    if(drv_info.sx1278_gpio.DIO2 < 0) 
    {
        printk(KERN_ERR "%s %d Get DIO2 pin error\n", __FUNCTION__, __LINE__);
        result = -EINVAL;
        goto ERR_EXIT;
    }
    
    drv_info.sx1278_gpio.DIO3 = of_get_named_gpio(np, "dio3-gpios", 0);
    if(drv_info.sx1278_gpio.DIO3 < 0)
    {
        printk(KERN_ERR "%s %d Get DIO3 pin error\n", __FUNCTION__, __LINE__);
        result = -EINVAL;
        goto ERR_EXIT;
    }
    
    drv_info.sx1278_gpio.DIO4 = of_get_named_gpio(np, "dio4-gpios", 0);
    if(drv_info.sx1278_gpio.DIO4 < 0) 
    {
        printk(KERN_ERR "%s %d Get DIO4 pin error\n", __FUNCTION__, __LINE__);
        result = -EINVAL;
        goto ERR_EXIT;
    }
    
    drv_info.sx1278_gpio.Reset= of_get_named_gpio(np, "reset-gpios", 0);
    if(drv_info.sx1278_gpio.Reset < 0) 
    {
        printk(KERN_ERR "%s %d Get Reset pin error\n", __FUNCTION__, __LINE__);
        result = -EINVAL;
        goto ERR_EXIT;
    }
    
    /* Initialize the driver data */
    drv_info.spi = spi;
    spin_lock_init(&drv_info.spi_lock);
    mutex_init(&drv_info.buf_lock);
    mutex_init(&drv_info.lora_mutex);

    INIT_LIST_HEAD(&drv_info.device_entry);

    /* If we can allocate a minor number, hook up this device.
     * Reusing minors is fine so long as udev or mdev is working.
     */
    mutex_lock(&device_list_lock);
    minor = find_first_zero_bit(minors, N_SPI_MINORS);
    if (minor < N_SPI_MINORS) {
        struct device *dev;

        drv_info.devt = MKDEV(DRV_MAJOR, minor);
        dev = device_create(drv_class, &spi->dev, drv_info.devt,
                            drv_info_ptr, "sx1278%d.%d",
                            spi->master->bus_num, spi->chip_select);
    result = PTR_ERR_OR_ZERO(dev);
    } else {
        dev_dbg(&spi->dev, "no minor number available!\n");
        result = -ENODEV;
    }
    if (result == 0) 
    {
        set_bit(minor, minors);
        list_add(&drv_info.device_entry, &device_list);
    }
    mutex_unlock(&device_list_lock);

    //初始化等待队列
    init_completion(&drv_info.lora_complete);
    init_waitqueue_head(&(drv_info.lora_wq));
    INIT_DELAYED_WORK(&drv_info.lora_work, lora_work_func);
    INIT_WORK(&drv_info.lora_work, lora_work_func);

    if (result == 0)
        spi_set_drvdata(spi, drv_info_ptr);
    else
        kfree(drv_info_ptr);

    /*config spi mode*/
    spi->mode = SPI_MODE_0;
    spi_setup(spi);    

    result = Lora_IoInit(&drv_info.sx1278_gpio);
    if (result < 0) 
    {
        goto ERR_EXIT1;
    }

    result = Lora_IoIrqInit(&drv_info.sx1278_gpio);
    
    if (result < 0) 
    {
        goto ERR_EXIT2;
    }

    result = Lora_Reset(&drv_info.sx1278_gpio);
    if (result < 0) 
    {
        goto ERR_EXIT3;
    }

    drv_info.sx1278_cfg.rx_cfg.RFFrequency          = 470300000;
    drv_info.sx1278_cfg.rx_cfg.SignalBw             = 0;
    drv_info.sx1278_cfg.rx_cfg.SpreadingFactor      = 7;
    drv_info.sx1278_cfg.rx_cfg.ErrorCoding          = 2;
    drv_info.sx1278_cfg.rx_cfg.CrcOn                = true;
    drv_info.sx1278_cfg.rx_cfg.ImplicitHeaderOn     = false;
    drv_info.sx1278_cfg.rx_cfg.RxSingleOn           = 1;
    drv_info.sx1278_cfg.rx_cfg.FreqHopOn            = 0;
    drv_info.sx1278_cfg.rx_cfg.HopPeriod            = 4;
    drv_info.sx1278_cfg.rx_cfg.RxPacketTimeout      = 5;
    drv_info.sx1278_cfg.rx_cfg.PayloadLength        = 128;

    drv_info.sx1278_cfg.tx_cfg.RFFrequency          = 470300000;
    drv_info.sx1278_cfg.tx_cfg.Power                = 20;
    drv_info.sx1278_cfg.tx_cfg.SignalBw             = 0;
    drv_info.sx1278_cfg.tx_cfg.SpreadingFactor      = 7;
    drv_info.sx1278_cfg.tx_cfg.ErrorCoding          = 2;
    drv_info.sx1278_cfg.tx_cfg.CrcOn                = true;
    drv_info.sx1278_cfg.tx_cfg.ImplicitHeaderOn     = false;
    drv_info.sx1278_cfg.tx_cfg.FreqHopOn            = 0;
    drv_info.sx1278_cfg.tx_cfg.HopPeriod            = 4;
    drv_info.sx1278_cfg.tx_cfg.TxPacketTimeout      = 5;
    drv_info.sx1278_cfg.tx_cfg.PayloadLength        = 128;

    SX1278Init(drv_info.spi, &drv_info.sx1278_cfg);
    if (result < 0) 
    {
        goto ERR_EXIT1;
    }

    return result;
    
ERR_EXIT3:
    Lora_IoIrqDeInit(&drv_info.sx1278_gpio);
ERR_EXIT2:
    Lora_IoDeInit(&drv_info.sx1278_gpio);
ERR_EXIT1:
    device_destroy(drv_class, drv_info.devt);
ERR_EXIT:
    if(drv_info_ptr)
        kfree(drv_info_ptr);
    
    return result;
}

static int drv_remove(struct spi_device *spi)
{
    //drv_info_st_ptr drv_info_ptr = spi_get_drvdata(spi);

    /* make sure ops on existing fds can abort cleanly */
    spin_lock_irq(&drv_info.spi_lock);
    drv_info.spi = NULL;
    spin_unlock_irq(&drv_info.spi_lock);

    /* prevent new opens */
    //mutex_lock(&drv_info.device_list_lock);
    list_del(&drv_info.device_entry);
    device_destroy(drv_class, drv_info.devt);
    clear_bit(MINOR(drv_info.devt), minors);
    //if (drv_info.users == 0)
    //    kfree(drv_info_ptr);
    //mutex_unlock(&drv_info.device_list_lock);

    return 0;
}

static const struct of_device_id drv_dt_ids[] = {
    { .compatible = "lora,sx1278" },
    {},
};

MODULE_DEVICE_TABLE(of, drv_dt_ids);

static struct spi_driver sx1278_spi_driver = {
    .driver = {
        .name = "sx1278",
        .owner = THIS_MODULE,
        .of_match_table = of_match_ptr(drv_dt_ids),
    },
    .probe  = drv_probe,
    .remove = drv_remove,

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
