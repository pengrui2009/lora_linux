/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : drv_lora.c
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
#include "drv_lora.h"
#include "drv_lib.h"
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

//驱动名
#define DRV_NAME        "lora"
//主版本号
#define DRV_VER1        1                
//次版本号
#define DRV_VER2        0
//驱动主设备号
#define DRV_MAJOR       107
//次设备个数
#define DRV_MINOR       1                   

#define N_SPI_MINORS    DRV_MINOR

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

/*!
 * Hardware DIO IRQ callback initialization
 */
irqreturn_t SX1278_OnDio0Irq(int irq, void *dev_id)
{
    lora_info_st_ptr lora_ptr = NULL;
    SX1278_Gpio_st_ptr sx1278_gpio_ptr = (SX1278_Gpio_st_ptr)dev_id;

    lora_ptr = container_of(sx1278_gpio_ptr, lora_info_st, gpio);

    printk(KERN_ERR "irq0 state:%d\n", lora_ptr->state); 
    schedule_work(&lora_ptr->work);
#if 0
    switch(lora_ptr->state)
    {
    case RF_RX_RUNNING:
        {   
            //
            //SX1278RxClearIrq();
            //drv_info.lora_recv_state = 1;
            //wake_up_interruptible(&drv_info.lora_rq); 
            schedule_work(&lora_ptr->work);
            break;
        }
    case RF_TX_RUNNING:
        {
#if 0            
            //SX1278TxFinished();
            drv_info.sx1278_state = RF_IDLE;
            //complete(&drv_info.lora_complete);
            drv_info.lora_send_state = 1;
#endif           
            schedule_work(&lora_ptr->lora_work);
            
            break;
        }
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
//    uint8_t val = 0, val1 = 0;
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
    lora_info_st_ptr lora_ptr = NULL;
    SX1278_Gpio_st_ptr sx1278_gpio_ptr = (SX1278_Gpio_st_ptr)dev_id;

    lora_ptr = container_of(sx1278_gpio_ptr, lora_info_st, gpio);
    printk(KERN_ERR "irq3\n");

    schedule_work(&lora_ptr->work);
    
    return IRQ_HANDLED;
}

irqreturn_t SX1278_OnDio4Irq(int irq, void *dev_id)
{
    printk(KERN_ERR "irq4\n");
#if 0
    switch( drv_info.sx1278_cfg.Modem )
    {
    case MODEM_FSK:
      :w
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

int lora_IoInit(SX1278_Gpio_st_ptr sx1278_gpio_ptr)
{
    int result = 0;
    
    if(NULL == sx1278_gpio_ptr)
    {
        result = -ENOMEM;
        goto ERR_EXIT;
    }
   
    if(gpio_is_valid(sx1278_gpio_ptr->DIO0))
    {
        result = gpio_request(sx1278_gpio_ptr->DIO0, "DIO0");
        if(result < 0)
        {
            printk(KERN_ERR "gpio_request DIO0 error\n");
            goto ERR_EXIT;
        }
    }

    if(gpio_is_valid(sx1278_gpio_ptr->DIO1))
    {
        result = gpio_request(sx1278_gpio_ptr->DIO1, "DIO1");
        if(result < 0)
        {
            printk(KERN_ERR "gpio_request DIO1 error\n");
            goto ERR_EXIT1;
        }
    }

    if(gpio_is_valid(sx1278_gpio_ptr->DIO2))
    {
        result = gpio_request(sx1278_gpio_ptr->DIO2, "DIO2");
        if(result < 0)
        {
            printk(KERN_ERR "gpio_request DIO2 error\n");
            goto ERR_EXIT2;
        }
    }

    if(gpio_is_valid(sx1278_gpio_ptr->DIO3))
    {
        result = gpio_request(sx1278_gpio_ptr->DIO3, "DIO3");
        if(result < 0)
        {
            printk(KERN_ERR "gpio_request DIO3 error\n");
            goto ERR_EXIT3;
        }
    }

    if(gpio_is_valid(sx1278_gpio_ptr->DIO4))
    {
        result = gpio_request(sx1278_gpio_ptr->DIO4, "DIO4");
        if(result < 0)
        {
            printk(KERN_ERR "gpio_request DIO4 error\n");
            goto ERR_EXIT4;
        }
    }

    if(gpio_is_valid(sx1278_gpio_ptr->Reset))
    {
        result = gpio_request(sx1278_gpio_ptr->Reset, "RESET");
        if(result < 0)
        {
            printk(KERN_ERR "gpio_request RESET error\n");
            goto ERR_EXIT5;
        }
    }

    return result;
ERR_EXIT5:
    if(gpio_is_valid(sx1278_gpio_ptr->DIO4))
        gpio_free(sx1278_gpio_ptr->DIO4);
ERR_EXIT4:
    if(gpio_is_valid(sx1278_gpio_ptr->DIO3))
        gpio_free(sx1278_gpio_ptr->DIO3);
ERR_EXIT3:
    if(gpio_is_valid(sx1278_gpio_ptr->DIO2))
        gpio_free(sx1278_gpio_ptr->DIO2);
ERR_EXIT2:
    if(gpio_is_valid(sx1278_gpio_ptr->DIO1))
        gpio_free(sx1278_gpio_ptr->DIO1);
ERR_EXIT1:
    if(gpio_is_valid(sx1278_gpio_ptr->DIO0))
        gpio_free(sx1278_gpio_ptr->DIO0);
ERR_EXIT:
    return result;
}

int lora_IoIrqInit(SX1278_Gpio_st_ptr sx1278_gpio_ptr)
{
    int irq = 0;
    int result = 0;

    if(NULL == sx1278_gpio_ptr)
    {
        result = -ENOMEM;
        goto ERR_EXIT0;
    }

    if(gpio_is_valid(sx1278_gpio_ptr->DIO0))
    {
        irq = gpio_to_irq(sx1278_gpio_ptr->DIO0);
        result = request_irq(irq , SX1278_OnDio0Irq, IRQF_TRIGGER_RISING, "DIO0_IRQ", (void *)sx1278_gpio_ptr);
        if(result)     
        {       
            printk(KERN_ERR "%s %d request_irq ERROR\n", __FUNCTION__, __LINE__);
            result = -EFAULT;
            goto ERR_EXIT0; 
        }
    }

    if(gpio_is_valid(sx1278_gpio_ptr->DIO1))
    {
        irq = gpio_to_irq(sx1278_gpio_ptr->DIO1);
        result = request_irq(irq , SX1278_OnDio1Irq, IRQF_TRIGGER_RISING, "DIO1_IRQ", (void *)sx1278_gpio_ptr);
        if(result)     
        {       
            printk(KERN_ERR "%s %d request_irq ERROR\n", __FUNCTION__, __LINE__);
            result = -EFAULT;
            goto ERR_EXIT1; 
        }
    }

    if(gpio_is_valid(sx1278_gpio_ptr->DIO2))
    {
        irq = gpio_to_irq(sx1278_gpio_ptr->DIO2);
        result = request_irq(irq , SX1278_OnDio2Irq, IRQF_TRIGGER_RISING, "DIO2_IRQ", (void *)sx1278_gpio_ptr);
        if(result)     
        {       
            printk(KERN_ERR "%s %d request_irq ERROR\n", __FUNCTION__, __LINE__);
            result = -EFAULT;
            goto ERR_EXIT2; 
        }
    }

    if(gpio_is_valid(sx1278_gpio_ptr->DIO3))
    {
        irq = gpio_to_irq(sx1278_gpio_ptr->DIO3);
        result = request_irq(irq , SX1278_OnDio3Irq, IRQF_TRIGGER_RISING, "DIO3_IRQ", (void *)sx1278_gpio_ptr);
        if(result)     
        {       
            printk(KERN_ERR "%s %d request_irq ERROR\n", __FUNCTION__, __LINE__);
            result = -EFAULT;
            goto ERR_EXIT3; 
        }
    }

    if(gpio_is_valid(sx1278_gpio_ptr->DIO4))
    {
        irq = gpio_to_irq(sx1278_gpio_ptr->DIO4);
        result = request_irq(irq , SX1278_OnDio0Irq, IRQF_TRIGGER_RISING, "DIO4_IRQ", (void *)sx1278_gpio_ptr);
        if(result)     
        {       
            printk(KERN_ERR "%s %d request_irq ERROR\n", __FUNCTION__, __LINE__);
            result = -EFAULT;
            goto ERR_EXIT4; 
        }
    }

    return result;

ERR_EXIT4:
    if(gpio_is_valid(sx1278_gpio_ptr->DIO3))
    {
        irq = gpio_to_irq(sx1278_gpio_ptr->DIO3);
        free_irq(irq, sx1278_gpio_ptr);    
    }
ERR_EXIT3:
    if(gpio_is_valid(sx1278_gpio_ptr->DIO2))
    {
        irq = gpio_to_irq(sx1278_gpio_ptr->DIO2);
        free_irq(irq, NULL);
    }
ERR_EXIT2:
    if(gpio_is_valid(sx1278_gpio_ptr->DIO1))
    {
        irq = gpio_to_irq(sx1278_gpio_ptr->DIO1);
        free_irq(irq, NULL);
    }
ERR_EXIT1:
    if(gpio_is_valid(sx1278_gpio_ptr->DIO0))
    {
        irq = gpio_to_irq(sx1278_gpio_ptr->DIO0);
        free_irq(irq, NULL);
    }
ERR_EXIT0:
    return result;
}

int lora_IoDeInit(SX1278_Gpio_st_ptr sx1278_gpio_ptr)
{
    int result = 0;

    if(NULL == sx1278_gpio_ptr)
    {
        result = -ENOMEM;
        goto ERR_EXIT;
    }

    if(gpio_is_valid(sx1278_gpio_ptr->DIO0))
    {
        gpio_free(sx1278_gpio_ptr->DIO0);
    }
    
    if(gpio_is_valid(sx1278_gpio_ptr->DIO1))
    {
        gpio_free(sx1278_gpio_ptr->DIO1);
    }

    if(gpio_is_valid(sx1278_gpio_ptr->DIO2))
    {
        gpio_free(sx1278_gpio_ptr->DIO2);
    }

    if(gpio_is_valid(sx1278_gpio_ptr->DIO3))
    {
        gpio_free(sx1278_gpio_ptr->DIO3);
    }

    if(gpio_is_valid(sx1278_gpio_ptr->DIO4))
    {
        gpio_free(sx1278_gpio_ptr->DIO4);
    }
   
    if(gpio_is_valid(sx1278_gpio_ptr->Reset))
    {
        gpio_free(sx1278_gpio_ptr->Reset);
    }
ERR_EXIT:

    return result;
}

int lora_IoIrqDeInit(SX1278_Gpio_st_ptr sx1278_gpio_ptr)
{
    int irq = 0;
    int result = 0;

    if(NULL == sx1278_gpio_ptr)
    {
        result = -ENOMEM;
        goto ERR_EXIT;
    }

    if(gpio_is_valid(sx1278_gpio_ptr->DIO1))
    {
        irq = gpio_to_irq(sx1278_gpio_ptr->DIO0);
        free_irq(irq, sx1278_gpio_ptr);
    }

    if(gpio_is_valid(sx1278_gpio_ptr->DIO1))
    {
       irq = gpio_to_irq(sx1278_gpio_ptr->DIO1);
       free_irq(irq, sx1278_gpio_ptr);
    }

    if(gpio_is_valid(sx1278_gpio_ptr->DIO2))
    {
        irq = gpio_to_irq(sx1278_gpio_ptr->DIO2);
        free_irq(irq, sx1278_gpio_ptr);
    }

    if(gpio_is_valid(sx1278_gpio_ptr->DIO3))
    {
        irq = gpio_to_irq(sx1278_gpio_ptr->DIO3);
        free_irq(irq, sx1278_gpio_ptr);
    }
    
    if(gpio_is_valid(sx1278_gpio_ptr->DIO4))
    {
        irq = gpio_to_irq(sx1278_gpio_ptr->DIO4);
        free_irq(irq, sx1278_gpio_ptr);
    }
ERR_EXIT:
    
    return result;
}


int lora_Reset(SX1278_Gpio_st_ptr sx1278_gpio_ptr)
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

static int lora_parse_dt(struct device *dev_ptr, SX1278_Gpio_st_ptr gpio_ptr)
{
    int result = 0;
    struct device_node *node_ptr = dev_ptr->of_node;

    if (!node_ptr)
    {
        result = -ENOMEM;
        goto ERR_EXIT;
    }
    
    gpio_ptr->DIO0 = of_get_named_gpio(node_ptr, "dio0-gpios", 0);
    gpio_ptr->DIO1 = of_get_named_gpio(node_ptr, "dio1-gpios", 0);
    gpio_ptr->DIO2 = of_get_named_gpio(node_ptr, "dio2-gpios", 0);
    gpio_ptr->DIO3 = of_get_named_gpio(node_ptr, "dio3-gpios", 0);
    gpio_ptr->DIO4 = of_get_named_gpio(node_ptr, "dio4-gpios", 0);
    gpio_ptr->Reset= of_get_named_gpio(node_ptr, "reset-gpios", 0);

ERR_EXIT:

    return result;
}

static void lora_delaywork_func(struct work_struct *w)
{
    //int result = 0;
    struct delayed_work *dw = to_delayed_work(w);
    lora_info_st_ptr lora_ptr = container_of(dw, lora_info_st, delaywork);

    switch(lora_ptr->state)
    {
    case RF_RX_RUNNING:
    {
        //uint32_t timeout = 0; 
        //Clear Irq
        //lora_ptr->statu = RF_RX_TIMEOUT;
        //lora_ptr->state = RF_RX_RUNNING;
        //printk(KERN_ERR "rx timeout\n");
        //wake_up_interruptible(&lora_ptr->wq);
        //result = SX1278LoRaGetRxPacketTimeout(&lora_ptr->sx1278, &timeout);
        //if(result < 0)
        //{
        //    goto ERR_EXIT;
        //}

        //schedule_delayed_work(&lora_ptr->delaywork, timeout * HZ);
        break;
    }
    case RF_TX_RUNNING:
    {
        //uint32_t timeout = 0;
        lora_ptr->state = RF_IDLE;
        lora_ptr->statu = RF_TX_TIMEOUT;
        //printk(KERN_ERR "tx timeout\n");
        wake_up_interruptible(&lora_ptr->wq);

        //begin rx 
        
        //lora_ptr->statu = RF_ERROR;
        
        //result = SX1278SetRxConfig(&lora_ptr->sx1278);
        //if (result < 0) 
        //{
        //    goto ERR_EXIT;
        //}

        //result = SX1278StartRx(&lora_ptr->sx1278);
        //if (result < 0) 
        //{
        //    goto ERR_EXIT;
        //}

        //lora_ptr->state = RF_RX_RUNNING;

        //timeout handler
        //result = SX1278LoRaGetRxPacketTimeout(&lora_ptr->sx1278, &timeout);
        //if (result < 0) 
        //{
        //    goto ERR_EXIT;
        //}
        
        //schedule_delayed_work(&lora_ptr->delaywork, timeout * HZ);
        
        //result = SX1278SetOpMode(&lora_ptr->sx1278, RF_OPMODE_RECEIVER);
        //if (result < 0) 
        //{
        //    goto ERR_EXIT;
        //}
        break;
    }
    default:
        break;
    }

//ERR_EXIT:

    return ;
}

static void lora_work_func(struct work_struct *dw)
{
    int result = 0;
    uint8_t regval = 0;
    uint8_t irqFlags = 0;
    lora_info_st_ptr lora_ptr = container_of(dw, lora_info_st, work);
    
    result = SX1278_Read_Reg(lora_ptr->sx1278.spi_ptr, REG_LR_IRQFLAGS, &irqFlags);
    if(result < 0)
    {
        goto ERR_EXIT;
    }
    printk(KERN_ERR "0x%0x\n", irqFlags);
    switch(lora_ptr->state)
    {
    case RF_RX_RUNNING:
        //RXDONE IRQ
        mutex_lock(&lora_ptr->mutex); 
        if(irqFlags & RFLR_IRQFLAGS_RXDONE)
        {
            int8_t snr = 0;
            int16_t rssi = 0;
            //uint32_t timeout = 0;
            //Clear Irq
            result = SX1278LoRaClearIrq(&lora_ptr->sx1278, RFLR_IRQFLAGS_RXDONE);
            if(result < 0)
            {
                goto ERR_EXIT;
            }

            //valid crc flag
            if( ( irqFlags & RFLR_IRQFLAGS_PAYLOADCRCERROR_MASK ) == RFLR_IRQFLAGS_PAYLOADCRCERROR )
            {
                // Clear Irq
                result = SX1278LoRaClearIrq(&lora_ptr->sx1278, RFLR_IRQFLAGS_PAYLOADCRCERROR );
                if(result < 0)
                {
                    goto ERR_EXIT;
                }

                if(lora_ptr->sx1278.cfg.rx_cfg.RxSingleOn == true )
                {
                    lora_ptr->state = RF_IDLE;
                }
                //cancel_delayed_work(&lora_ptr->delaywork);

                break;
            }
            
            //read snr and rssi
            result = SX1278LoRaGetPacketSnrandRssi(&lora_ptr->sx1278, lora_ptr->sx1278.cfg.rx_cfg.RFFrequency, &snr, &rssi);
            if(result < 0)
            {
                goto ERR_EXIT;
            }
            lora_ptr->rxdata.snr = snr;
            lora_ptr->rxdata.rssi = rssi;
            
            if(lora_ptr->sx1278.cfg.rx_cfg.RxSingleOn == true) // Rx single mode
            {
                uint8_t RegFifoAddrPtr = 0x00;
                result = SX1278_Write_Reg(lora_ptr->sx1278.spi_ptr, REG_LR_FIFOADDRPTR, RegFifoAddrPtr);
                if(result < 0)
                {
                    goto ERR_EXIT;
                }
                
                if(lora_ptr->sx1278.cfg.rx_cfg.ImplicitHeaderOn == true)
                {
                    mutex_lock(&lora_ptr->buf_mutex);
                    result = SX1278_Read_FIFO(lora_ptr->sx1278.spi_ptr, lora_ptr->rxdata.data, lora_ptr->rxdata.recv_len);
                    if(result < 0)
                    {
                        goto ERR_EXIT;
                    }
                    mutex_unlock(&lora_ptr->buf_mutex);
                }
                else
                {
                    uint8_t RegNbRxBytes = 0;
                    result = SX1278_Read_Reg(lora_ptr->sx1278.spi_ptr, REG_LR_RXNBBYTES, &RegNbRxBytes);
                    if(result < 0)
                    {
                        goto ERR_EXIT;
                    }
                    //spin_lock(&lora_ptr->spinlock);
                    mutex_lock(&lora_ptr->buf_mutex);
                    lora_ptr->rxdata.data_len = RegNbRxBytes;
                    result = SX1278_Read_FIFO(lora_ptr->sx1278.spi_ptr, lora_ptr->rxdata.data, RegNbRxBytes);
                    if(result < 0)
                    {
                        goto ERR_EXIT;
                    }
                    //spin_unlock(&lora_ptr->spinlock);
                    mutex_unlock(&lora_ptr->buf_mutex);
                }

                lora_ptr->state = RF_IDLE;
                lora_ptr->statu = RF_RX_DONE;
            }
            else // Rx continuous mode
            {
                uint8_t RegFifoRxCurrentAddr = 0;
                result = SX1278_Read_Reg(lora_ptr->sx1278.spi_ptr, REG_LR_FIFORXCURRENTADDR, &RegFifoRxCurrentAddr);
                if(result < 0)
                {
                    goto ERR_EXIT;
                }

                if(lora_ptr->sx1278.cfg.rx_cfg.ImplicitHeaderOn == true)
                {            
                    result = SX1278_Write_Reg(lora_ptr->sx1278.spi_ptr, REG_LR_FIFOADDRPTR, RegFifoRxCurrentAddr);
                    if(result < 0)
                    {
                        goto ERR_EXIT;
                    }
                    //spin_lock(&lora_ptr->spinlock);
                    mutex_lock(&lora_ptr->buf_mutex);
                    result = SX1278_Read_FIFO(lora_ptr->sx1278.spi_ptr, lora_ptr->rxdata.data, lora_ptr->rxdata.recv_len);
                    if(result < 0)
                    {
                        goto ERR_EXIT;
                    }
                    //spin_unlock(&lora_ptr->spinlock);
                    mutex_unlock(&lora_ptr->buf_mutex);
                }
                else
                {
                    uint8_t RegNbRxBytes = 0;
                    result = SX1278_Read_Reg(lora_ptr->sx1278.spi_ptr, REG_LR_RXNBBYTES, &RegNbRxBytes);
                    if(result < 0)
                    {
                        goto ERR_EXIT;
                    }
                    
                    result = SX1278_Write_Reg(lora_ptr->sx1278.spi_ptr, REG_LR_FIFOADDRPTR, RegFifoRxCurrentAddr);
                    if(result < 0)
                    {
                        goto ERR_EXIT;
                    }
                    //spin_lock(&lora_ptr->spinlock);
                    mutex_lock(&lora_ptr->buf_mutex);
                    lora_ptr->rxdata.data_len = RegNbRxBytes;
                    result = SX1278_Read_FIFO(lora_ptr->sx1278.spi_ptr, lora_ptr->rxdata.data, RegNbRxBytes);
                    if(result < 0)
                    {
                        goto ERR_EXIT;
                    }
                    //spin_unlock(&lora_ptr->spinlock);
                    mutex_unlock(&lora_ptr->buf_mutex);
                }

                lora_ptr->statu = RF_RX_DONE;
            }

            //result = SX1278LoRaGetRxPacketTimeout(&lora_ptr->sx1278, &timeout);
            //if(result < 0)
            //{
            //    goto ERR_EXIT;
            //}
            
            //cancel_delayed_work(&lora_ptr->delaywork);
            //schedule_delayed_work(&lora_ptr->delaywork, timeout * HZ);
            
            wake_up_interruptible(&lora_ptr->wq);
        }
        else
        {
            
        }
        mutex_lock(&lora_ptr->mutex); 
        break;
    case RF_TX_RUNNING:
        if(irqFlags & RFLR_IRQFLAGS_TXDONE)
        {
            //tx done
            //Clear Irq
            result = SX1278LoRaClearIrq(&lora_ptr->sx1278, RFLR_IRQFLAGS_TXDONE);
            if(result < 0)
            {
                goto ERR_EXIT;
            }
            lora_ptr->txdata.done_len += lora_ptr->txdata.doing_len;  
            //jude we need to send other data
            if(lora_ptr->txdata.done_len < lora_ptr->txdata.data_len)
            {
                int leftlen = 0;
                uint8_t txlen = 0;
                leftlen = lora_ptr->txdata.data_len - lora_ptr->txdata.done_len;
                txlen = (uint8_t)((leftlen > FIFO_BUFFER_SIZE) ? FIFO_BUFFER_SIZE : leftlen);
                lora_ptr->txdata.doing_len = txlen;
                
                // Initializes the payload size
                result = SX1278_Write_Reg(lora_ptr->sx1278.spi_ptr, REG_LR_PAYLOADLENGTH, txlen);
                if(result < 0)
                {
                    goto ERR_EXIT;
                }

                // Full buffer used for Tx
                result = SX1278_Write_Reg(lora_ptr->sx1278.spi_ptr, REG_LR_FIFOTXBASEADDR, 0);
                if(result < 0)
                {
                    goto ERR_EXIT;
                }
        
                result = SX1278_Write_Reg(lora_ptr->sx1278.spi_ptr, REG_LR_FIFOADDRPTR, 0);
                if(result < 0)
                {
                    goto ERR_EXIT;
                }

                // Write payload buffer
                result = SX1278_Write_FIFO(lora_ptr->sx1278.spi_ptr, (lora_ptr->txdata.data_ptr + lora_ptr->txdata.done_len), txlen);
                if(result < 0)
                {
                    goto ERR_EXIT;
                }

                if(lora_ptr->sx1278.cfg.tx_cfg.FreqHopOn == true)
                {
                    result = SX1278_Write_Reg(lora_ptr->sx1278.spi_ptr, REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
                                                                   RFLR_IRQFLAGS_RXDONE |
                                                                   RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                                                   RFLR_IRQFLAGS_VALIDHEADER |
                                                                   //RFLR_IRQFLAGS_TXDONE |
                                                                   RFLR_IRQFLAGS_CADDONE |
                                                                   //RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                                                   RFLR_IRQFLAGS_CADDETECTED );
                    if(result < 0)
                    {
                        goto ERR_EXIT;
                    }
                
                    result = SX1278_Read_Reg(lora_ptr->sx1278.spi_ptr, REG_DIOMAPPING1, &regval);
                    if(result < 0)
                    {
                        goto ERR_EXIT;
                    }
        
                    // DIO0=TxDone, DIO2=FhssChangeChannel
                    result = SX1278_Write_Reg(lora_ptr->sx1278.spi_ptr, REG_DIOMAPPING1, (regval&RFLR_DIOMAPPING1_DIO0_MASK&RFLR_DIOMAPPING1_DIO2_MASK)|RFLR_DIOMAPPING1_DIO0_01|RFLR_DIOMAPPING1_DIO2_00);
                    if(result < 0)
                    {
                        goto ERR_EXIT;
                    }
               }
               else
               {
                    result = SX1278_Write_Reg(lora_ptr->sx1278.spi_ptr, REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
                                                                   RFLR_IRQFLAGS_RXDONE |
                                                                   RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                                                   RFLR_IRQFLAGS_VALIDHEADER |
                                                                   //RFLR_IRQFLAGS_TXDONE |
                                                                   RFLR_IRQFLAGS_CADDONE |
                                                                   RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                                                   RFLR_IRQFLAGS_CADDETECTED );
                   if(result < 0)
                   {
                       goto ERR_EXIT;
                   }        

                   result = SX1278_Read_Reg(lora_ptr->sx1278.spi_ptr, REG_DIOMAPPING1, &regval);
                   if(result < 0)
                   {
                       goto ERR_EXIT;
                   }

                   // DIO0=TxDone
                   result = SX1278_Write_Reg(lora_ptr->sx1278.spi_ptr, REG_DIOMAPPING1, (regval& RFLR_DIOMAPPING1_DIO0_MASK ) | RFLR_DIOMAPPING1_DIO0_01);
                   if(result < 0)
                   {
                       goto ERR_EXIT;
                   }            
               }
               
               result = SX1278StartTx(&lora_ptr->sx1278);
               if(result < 0)
               {
                   goto ERR_EXIT;
               }
               lora_ptr->state = RF_TX_RUNNING;
            }
            else
            {
                wake_up_interruptible(&lora_ptr->wq);
                cancel_delayed_work(&lora_ptr->delaywork); 
                lora_ptr->state = RF_IDLE;
                lora_ptr->statu = RF_TX_DONE;

                result = SX1278SetRxConfig(&lora_ptr->sx1278);
                if(result < 0)
                {
                    goto ERR_EXIT;
                }

                result = SX1278StartRx(&lora_ptr->sx1278);
                if(result < 0)
                {
                    goto ERR_EXIT;
                }
                 
                lora_ptr->state = RF_RX_RUNNING;
                lora_ptr->statu = RF_ERROR;

                result = SX1278SetOpMode(&lora_ptr->sx1278, RF_OPMODE_RECEIVER);
                if (result < 0) 
                {
                    printk(KERN_ERR "%s %d\n", __FUNCTION__, __LINE__);
                    goto ERR_EXIT;
                }
            }
        }
         
        break;
    case RF_CAD:
    {
        result = SX1278CadDone(&lora_ptr->sx1278);
        if(result < 0)
        {
            goto ERR_EXIT;
        }

        lora_ptr->state = RF_IDLE;
        
        if(result == 1)
        {
            lora_ptr->statu = RF_CAD_DONE;
        }

        wake_up_interruptible(&lora_ptr->wq);
        break;
    }
    default:
        break;
    }

ERR_EXIT:

    return;
}

static void debug(struct spi_device *spi_ptr, int line, uint8_t addr)
{
#if 0
    uint8_t regval = 0;
    SX1278_Read_Reg(spi_ptr, addr, &regval);
    printk(KERN_ERR "[%d] addr 0x%0x:0x%0x\n", line, addr, regval);
#endif
}

static int drv_open(struct inode *inode, struct file *filp)
{
    int result = 0;
    lora_info_st_ptr lora_ptr = NULL;

    mutex_lock(&device_list_lock);
    list_for_each_entry(lora_ptr, &device_list, device_entry) {
        if (lora_ptr->devt == inode->i_rdev) {
            result = 0;
            break;
        }
    }
    if (result == 0) {
        /*
        if (!lora_ptr->buffer) {
            lora_ptr->buffer = kmalloc(bufsiz, GFP_KERNEL);
            if (!lora_ptr->buffer) {
                dev_dbg(&lora_ptr->spi->dev, "open/ENOMEM\n");
                result = -ENOMEM;
            }
        }
        */
            
        lora_ptr->users++;
        filp->private_data = lora_ptr;
        nonseekable_open(inode, filp);
    }
    else
    {
        printk(KERN_ERR "spidev: nothing for minor %d\n", iminor(inode));
        goto ERR_EXIT;
    }
    
    if(!atomic_dec_and_test(&lora_ptr->opened)) 
    {
        printk(KERN_ERR "%s %d\n", __FUNCTION__, __LINE__);
        result = -EBUSY;
        goto ERR_EXIT;
    }
    
    result = lora_IoInit(&lora_ptr->gpio);
    if (result < 0) 
    {
        printk(KERN_ERR "%s %d\n", __FUNCTION__, __LINE__);
        goto ERR_EXIT1;
    }
    
    result = lora_Reset(&lora_ptr->gpio);
    if (result < 0) 
    {
        printk(KERN_ERR "%s %d\n", __FUNCTION__, __LINE__);
        goto ERR_EXIT3;
    }
   
    debug(lora_ptr->sx1278.spi_ptr, __LINE__, 1);   

    result = SX1278RxChainCalibration(&lora_ptr->sx1278);
    if(result < 0)
    {
        printk(KERN_ERR "%s %d\n", __FUNCTION__, __LINE__);
        goto ERR_EXIT;
    }

    debug(lora_ptr->sx1278.spi_ptr, __LINE__, 1);   
    result = SX1278SetOpMode(&lora_ptr->sx1278, RF_OPMODE_SLEEP);
    if(result < 0)
    {
        printk(KERN_ERR "%s %d\n", __FUNCTION__, __LINE__);
        goto ERR_EXIT;
    }

    debug(lora_ptr->sx1278.spi_ptr, __LINE__, 1);   
    result = lora_IoIrqInit(&lora_ptr->gpio);
    if (result < 0) 
    {
        printk(KERN_ERR "%s %d\n", __FUNCTION__, __LINE__);
        goto ERR_EXIT2;
    }

    lora_ptr->sx1278.cfg.rx_cfg.RFFrequency          = 470300000;
    lora_ptr->sx1278.cfg.rx_cfg.SignalBw             = 7;
    lora_ptr->sx1278.cfg.rx_cfg.SpreadingFactor      = 12;
    lora_ptr->sx1278.cfg.rx_cfg.ErrorCoding          = 1;
    lora_ptr->sx1278.cfg.rx_cfg.PreambleLen          = 8;
    lora_ptr->sx1278.cfg.rx_cfg.CrcOn                = true;
    lora_ptr->sx1278.cfg.rx_cfg.ImplicitHeaderOn     = false;
    //rx signle must be false
    lora_ptr->sx1278.cfg.rx_cfg.RxSingleOn           = 0;
    //FreqHop on must be false
    lora_ptr->sx1278.cfg.rx_cfg.FreqHopOn            = 0;
    lora_ptr->sx1278.cfg.rx_cfg.IqInverted           = false;
    lora_ptr->sx1278.cfg.rx_cfg.HopPeriod            = 4;
    //no timeout
    lora_ptr->sx1278.cfg.rx_cfg.RxPacketTimeout      = 5;
    lora_ptr->sx1278.cfg.rx_cfg.PayloadLength        = 128;

    lora_ptr->sx1278.cfg.tx_cfg.RFFrequency          = 470300000;
    lora_ptr->sx1278.cfg.tx_cfg.Power                = 14;
    lora_ptr->sx1278.cfg.tx_cfg.SignalBw             = 7;
    lora_ptr->sx1278.cfg.tx_cfg.SpreadingFactor      = 12;
    lora_ptr->sx1278.cfg.tx_cfg.ErrorCoding          = 1;
    lora_ptr->sx1278.cfg.tx_cfg.PreambleLen          = 8;
    lora_ptr->sx1278.cfg.tx_cfg.CrcOn                = true;
    lora_ptr->sx1278.cfg.tx_cfg.ImplicitHeaderOn     = false;
    lora_ptr->sx1278.cfg.tx_cfg.FreqHopOn            = 0;
    lora_ptr->sx1278.cfg.tx_cfg.IqInverted           = false;
    lora_ptr->sx1278.cfg.tx_cfg.HopPeriod            = 4;
    lora_ptr->sx1278.cfg.tx_cfg.TxPacketTimeout      = 5;
    lora_ptr->sx1278.cfg.tx_cfg.PayloadLength        = 128;

    result = SX1278Init(&lora_ptr->sx1278);
    if (result < 0) 
    {
        printk(KERN_ERR "%s %d\n", __FUNCTION__, __LINE__);
        goto ERR_EXIT3;
    }

    lora_ptr->state = RF_IDLE;
    lora_ptr->statu = RF_ERROR;
    
    //result = SX1278SetRxConfig(&lora_ptr->sx1278);
    //if (result < 0) 
    //{
    //    printk(KERN_ERR "%s %d\n", __FUNCTION__, __LINE__);
    //    goto ERR_EXIT3;
    //}

    //result = SX1278StartRx(&lora_ptr->sx1278);
    //if (result < 0) 
    //{
    //    printk(KERN_ERR "%s %d\n", __FUNCTION__, __LINE__);
    //    goto ERR_EXIT3;
    //}

    //lora_ptr->state = RF_RX_RUNNING;

    //timeout handler
    //result = SX1278LoRaGetRxPacketTimeout(&lora_ptr->sx1278, &timeout);
    //if (result < 0) 
    //{
    //    printk(KERN_ERR "%s %d\n", __FUNCTION__, __LINE__);
    //    goto ERR_EXIT3;
    //}
    
    //schedule_delayed_work(&lora_ptr->delaywork, timeout * HZ);
    
    //result = SX1278SetOpMode(&lora_ptr->sx1278, RF_OPMODE_RECEIVER);
    //if (result < 0) 
    //{
    //    printk(KERN_ERR "%s %d\n", __FUNCTION__, __LINE__);
    //    goto ERR_EXIT3;
    //}

    mutex_unlock(&device_list_lock);

    return result;
ERR_EXIT3:
    lora_IoIrqDeInit(&lora_ptr->gpio); 
ERR_EXIT2:
    lora_IoDeInit(&lora_ptr->gpio);
ERR_EXIT1:
ERR_EXIT:    
    atomic_inc(&lora_ptr->opened);
    return result;
}

/* Read-only message with current device setup */
static ssize_t drv_read(struct file *filp, char __user *buffer_ptr, size_t buffer_len, loff_t *f_pos)
{
    int result = 0;
    uint8_t actualrxsize = 0;
    lora_info_st_ptr lora_ptr = NULL;

    if((NULL == filp) ||(NULL == buffer_ptr) || (buffer_len <= 0))
    {
        result = -EINVAL;
        goto ERR_EXIT;
    }

    lora_ptr = filp->private_data;

    //mutex_lock(&lora_ptr->mutex); 

    lora_ptr->rxdata.data_len = 0;
    lora_ptr->rxdata.recv_len = (buffer_len > FIFO_BUFFER_SIZE) ? FIFO_BUFFER_SIZE : buffer_len;
    lora_ptr->rxdata.snr = 0;
    lora_ptr->rxdata.rssi = 0;
    //wait_event_interruptible(lora_ptr->lora_rq, lora_ptr->lora_recv_state);
    interruptible_sleep_on(&(lora_ptr->wq));

    //spin_lock(&lora_ptr->spinlock);
    mutex_lock(&lora_ptr->buf_mutex);
    actualrxsize = lora_ptr->rxdata.data_len;
    result = put_user(lora_ptr->rxdata.snr, buffer_ptr);
    if(result < 0)
    {
        goto ERR_EXIT;
    }

    result = put_user(lora_ptr->rxdata.rssi, (buffer_ptr + sizeof(lora_ptr->rxdata.snr)));
    if(result < 0)
    {
        goto ERR_EXIT;
    }

    result = copy_to_user((buffer_ptr + sizeof(lora_ptr->rxdata.rssi)), lora_ptr->rxdata.data, actualrxsize);
    if(result < 0)
    {
        goto ERR_EXIT;
    }
    //spin_unlock(&lora_ptr->spinlock);
    //mutex_unlock(&lora_ptr->buf_mutex);
    
    mutex_unlock(&lora_ptr->mutex);
    printk(KERN_ERR "kernel rx:%d\n", actualrxsize );
    return (actualrxsize + sizeof(lora_ptr->rxdata.rssi) + sizeof(lora_ptr->rxdata.snr));
ERR_EXIT:

    mutex_unlock(&lora_ptr->mutex);
    return result;
}

/* Write-only message with current device setup */
static ssize_t drv_write(struct file *filp, const char __user *buffer_ptr, size_t buffer_len, loff_t *f_pos)
{
    int result = 0;
    uint8_t txlen = 0;
    uint32_t timeout = 0;
    lora_info_st_ptr lora_ptr = NULL;

    if((NULL == filp) ||(NULL == buffer_ptr) || (buffer_len <= 0))
    {
        result = -EINVAL;
        goto ERR_EXIT;
    }

    lora_ptr = filp->private_data;
    //consider rx irq interrupt
    //spin_lock_irq(&lora_ptr->spinlock);
    disable_irq_nosync(gpio_to_irq(lora_ptr->gpio.DIO0));
    mutex_lock(&lora_ptr->mutex);
    lora_ptr->txdata.data_ptr = kzalloc(buffer_len, GFP_KERNEL);
    if(NULL == lora_ptr->txdata.data_ptr)
    {
        result = -ENOMEM;
        //spin_unlock_irq(&lora_ptr->spinlock);
        enable_irq(gpio_to_irq(lora_ptr->gpio.DIO0));
        goto ERR_EXIT;
    }
    lora_ptr->txdata.data_len = buffer_len;
    lora_ptr->txdata.doing_len = 0;
    lora_ptr->txdata.done_len = 0;

    result = copy_from_user(lora_ptr->txdata.data_ptr, buffer_ptr, buffer_len);
    if (result == 0)
    {
        result = SX1278SetTxConfig(&lora_ptr->sx1278);
        if(result < 0)
        {
            goto ERR_EXIT;
        }
        
        enable_irq(gpio_to_irq(lora_ptr->gpio.DIO0));
        //spin_unlock_irq(&lora_ptr->spinlock);

        txlen = (buffer_len > FIFO_BUFFER_SIZE) ? FIFO_BUFFER_SIZE : buffer_len;
        lora_ptr->txdata.doing_len = txlen;
        result = SX1278SetTxPacket(&lora_ptr->sx1278, lora_ptr->txdata.data_ptr, txlen);
        if(result < 0)
        {
            goto ERR_EXIT;
        }

        
        result = SX1278LoRaGetTxPacketTimeout(&lora_ptr->sx1278, &timeout);
        if(result < 0)
        {
            goto ERR_EXIT;
        }

        lora_ptr->state = RF_TX_RUNNING;
        lora_ptr->statu = RF_ERROR;
        schedule_delayed_work(&lora_ptr->delaywork, timeout * HZ);
#if 0
        for(i=1; i<=0x37; i++)
        {
            uint8_t regval = 0;
            result = SX1278_Read_Reg(lora_ptr->sx1278.spi_ptr, i, &regval);
            printk(KERN_ERR "addr 0x%0x: 0x%0x\n", i, regval);
        }
#endif
        result = SX1278StartTx(&lora_ptr->sx1278);
        if(result < 0)
        {
            goto ERR_EXIT;
        }
        
        interruptible_sleep_on(&(lora_ptr->wq));
        //wait_event_interruptible(lora_ptr->wq, (lora_ptr->state == RF_IDLE));
        
        //SX1278TxFinished(&lora_ptr->sx1278);

        result = SX1278SetRxConfig(&lora_ptr->sx1278);
        if(result < 0)
        {
             goto ERR_EXIT;
        }

        result = SX1278StartRx(&lora_ptr->sx1278);
        if(result < 0)
        {
             goto ERR_EXIT;
        }
                 
        lora_ptr->state = RF_RX_RUNNING;

        result = SX1278SetOpMode(&lora_ptr->sx1278, RF_OPMODE_RECEIVER);
        if (result < 0) 
        {
            printk(KERN_ERR "%s %d\n", __FUNCTION__, __LINE__);
            goto ERR_EXIT;
        }

        if(lora_ptr->statu == RF_TX_DONE)
        {
            result = lora_ptr->txdata.done_len;
        }else{
            //tx timeout
            result = -ETIMEDOUT;
        }
        //lora_ptr->state = RF_IDLE;
        lora_ptr->statu = RF_ERROR;
         
    }else{
        enable_irq(gpio_to_irq(lora_ptr->gpio.DIO0));
        result = -EIO;
        //spin_unlock_irq(&lora_ptr->spinlock);
        goto ERR_EXIT;
    }
    
    mutex_unlock(&lora_ptr->mutex);
ERR_EXIT:

    if(lora_ptr->txdata.data_ptr)
        kfree(lora_ptr->txdata.data_ptr);

    return result;
}

static long drv_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    int result = 0;
    lora_info_st_ptr lora_ptr = filp->private_data;

    /* Check type and command number */
    if (_IOC_TYPE(cmd) != SPI_IOC_MAGIC)
    {
        result = -ENOTTY;
        goto ERR_EXIT;
    }

    switch (cmd) {
    /* read requests */
    case LORA_IOC_RD_RXCONFIG:
    {
        Rx_Config_st cfg = {0};
        cfg.RFFrequency       = lora_ptr->sx1278.cfg.rx_cfg.RFFrequency;
        cfg.SignalBw          = lora_ptr->sx1278.cfg.rx_cfg.SignalBw;
        cfg.SpreadingFactor   = lora_ptr->sx1278.cfg.rx_cfg.SpreadingFactor;
        cfg.ErrorCoding       = lora_ptr->sx1278.cfg.rx_cfg.ErrorCoding;
        cfg.CrcOn             = lora_ptr->sx1278.cfg.rx_cfg.CrcOn;
        cfg.IqInverted        = lora_ptr->sx1278.cfg.rx_cfg.IqInverted;
        cfg.HopPeriod         = lora_ptr->sx1278.cfg.rx_cfg.HopPeriod;
        cfg.PayloadLength     = lora_ptr->sx1278.cfg.rx_cfg.PayloadLength;

        result = copy_to_user((void *)arg, &cfg, SX1278_RX_CONFIG_ST_LEN);
        break;
    }
    case LORA_IOC_WD_RXCONFIG:
    {
        Rx_Config_st cfg = {0};
        result = copy_from_user(&cfg, (void *)arg, SX1278_RX_CONFIG_ST_LEN);
        if(result < 0)
        {
            goto ERR_EXIT;
        }
        
        if((cfg.RFFrequency < 137000000) || (cfg.RFFrequency > 525000000))
        {
            result = -EINVAL;
            goto ERR_EXIT;
        }

        if((cfg.RFFrequency <= 169000000) &&((8 == cfg.SignalBw) || (9 == cfg.SignalBw)))
        {
            result = -EINVAL;
            goto ERR_EXIT;
        }

        if(cfg.SignalBw > 9)
        {
            result = -EINVAL;
            goto ERR_EXIT;
        }
        
        if((cfg.SpreadingFactor < 6) || (cfg.SpreadingFactor > 12))
        {
            result = -EINVAL;
            goto ERR_EXIT;
        }

        if((0 == cfg.ErrorCoding) || (4 < cfg.ErrorCoding))
        {
            result = -EINVAL;
            goto ERR_EXIT;
        }

        lora_ptr->sx1278.cfg.rx_cfg.RFFrequency     = cfg.RFFrequency;
        lora_ptr->sx1278.cfg.rx_cfg.SignalBw        = cfg.SignalBw;
        lora_ptr->sx1278.cfg.rx_cfg.SpreadingFactor = cfg.SpreadingFactor;
        lora_ptr->sx1278.cfg.rx_cfg.ErrorCoding     = cfg.ErrorCoding;
        lora_ptr->sx1278.cfg.rx_cfg.CrcOn           = cfg.CrcOn;
        lora_ptr->sx1278.cfg.rx_cfg.IqInverted      = cfg.IqInverted;
        lora_ptr->sx1278.cfg.rx_cfg.HopPeriod       = cfg.HopPeriod;
        lora_ptr->sx1278.cfg.rx_cfg.PayloadLength   = cfg.PayloadLength;

        break;
    }
    case LORA_IOC_RD_TXCONFIG:
    {
        Tx_Config_st cfg = {0};
        cfg.RFFrequency        = lora_ptr->sx1278.cfg.tx_cfg.RFFrequency;
        cfg.Power              = lora_ptr->sx1278.cfg.tx_cfg.Power;
        cfg.SignalBw           = lora_ptr->sx1278.cfg.tx_cfg.SignalBw;
        cfg.SpreadingFactor    = lora_ptr->sx1278.cfg.tx_cfg.SpreadingFactor;
        cfg.ErrorCoding        = lora_ptr->sx1278.cfg.tx_cfg.ErrorCoding;
        cfg.CrcOn              = lora_ptr->sx1278.cfg.tx_cfg.CrcOn;
        cfg.ImplicitHeaderOn   = lora_ptr->sx1278.cfg.tx_cfg.ImplicitHeaderOn;
        cfg.FreqHopOn          = lora_ptr->sx1278.cfg.tx_cfg.FreqHopOn;
        cfg.TxPacketTimeout    = lora_ptr->sx1278.cfg.tx_cfg.TxPacketTimeout;
        cfg.PayloadLength      = lora_ptr->sx1278.cfg.tx_cfg.PayloadLength;
        result = copy_to_user((void *)arg, &cfg, SX1278_TX_CONFIG_ST_LEN);
        if(result < 0)
        {
            goto ERR_EXIT;
        }
        break; 
    }
    case LORA_IOC_WD_TXCONFIG:
    {
        Tx_Config_st cfg = {0};
        result = copy_from_user(&cfg, (void *)arg, SX1278_TX_CONFIG_ST_LEN);
        if(result < 0)
        {
            goto ERR_EXIT;
        }
        
        if((cfg.RFFrequency < 137000000) || (cfg.RFFrequency > 525000000))
        {
            result = -EINVAL;
            goto ERR_EXIT;
        }
         
        if((cfg.RFFrequency <= 169000000) && ((8 == cfg.SignalBw) || (9 == cfg.SignalBw)))
        {
            result = -EINVAL;
            goto ERR_EXIT;
        }
        
        if(cfg.SignalBw > 9)
        {
            result = -EINVAL;
            goto ERR_EXIT;
        }
         
        if((cfg.SpreadingFactor < 6) || (cfg.SpreadingFactor > 12))
        {
            result = -EINVAL;
            goto ERR_EXIT;
        }

        if((0 == cfg.ErrorCoding) || (4 < cfg.ErrorCoding))
        {
            result = -EINVAL;
            goto ERR_EXIT;
        }
        
        lora_ptr->sx1278.cfg.tx_cfg.RFFrequency      = cfg.RFFrequency;
        lora_ptr->sx1278.cfg.tx_cfg.Power            = cfg.Power;
        lora_ptr->sx1278.cfg.tx_cfg.SignalBw         = cfg.SignalBw;
        lora_ptr->sx1278.cfg.tx_cfg.SpreadingFactor  = cfg.SpreadingFactor;
        lora_ptr->sx1278.cfg.tx_cfg.ErrorCoding      = cfg.ErrorCoding;
        lora_ptr->sx1278.cfg.tx_cfg.CrcOn            = cfg.CrcOn;
        lora_ptr->sx1278.cfg.tx_cfg.ImplicitHeaderOn = cfg.ImplicitHeaderOn;
        lora_ptr->sx1278.cfg.tx_cfg.FreqHopOn        = cfg.FreqHopOn;
        lora_ptr->sx1278.cfg.tx_cfg.TxPacketTimeout  = cfg.TxPacketTimeout;
        lora_ptr->sx1278.cfg.tx_cfg.PayloadLength    = cfg.PayloadLength;
        break;
    }
    case LORA_IOC_RD_RXSIGNAL_PRAM:
    {
        //result = SX1278StartCad(&lora_ptr->sx1278);

        //irq 3 cad done

        //irq4 cad detected
        //result = __put_user(arg, &result);
        break;
    }
    case LORA_IOC_CAD:
    {
        uint8_t status = 0;
        mutex_lock(&lora_ptr->mutex);

        lora_ptr->state = RF_CAD;

        lora_ptr->statu = RF_ERROR;
        
        result = SX1278StartCad(&lora_ptr->sx1278);
        if(result < 0)
        {
            goto ERR_EXIT;
        }
        
        interruptible_sleep_on(&(lora_ptr->wq));

        
        if(lora_ptr->statu == RF_CAD_DONE)
        {
            status = 1;
        }else{
            status = 0;
        }
        result = put_user(status, (uint8_t __user *)arg);
        if(result < 0)
        {
            goto ERR_EXIT;
        }        

        mutex_unlock(&lora_ptr->mutex);
        break;
    }
    case LORA_IOC_WR_WORK:
    {
        uint8_t work_on = 0;
        result = get_user(work_on, (uint8_t __user *)arg);
        if(result)
        {
	    goto ERR_EXIT;
        }
         
        if(work_on)
        { 
            result = SX1278SetRxConfig(&lora_ptr->sx1278);
            if (result < 0) 
            {
                printk(KERN_ERR "%s %d\n", __FUNCTION__, __LINE__);
                goto ERR_EXIT;
            }

            result = SX1278StartRx(&lora_ptr->sx1278);
            if (result < 0) 
            {
                printk(KERN_ERR "%s %d\n", __FUNCTION__, __LINE__);
                goto ERR_EXIT;
            }

            lora_ptr->state = RF_RX_RUNNING;
    
            result = SX1278SetOpMode(&lora_ptr->sx1278, RF_OPMODE_RECEIVER);
            if(result < 0)
            {
                goto ERR_EXIT;
            }
        }
        break;
    }
    default:
        break;
    }

ERR_EXIT:

    return result;
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
    lora_info_st_ptr lora_ptr;
    lora_ptr = file->private_data;   
  
    poll_wait(file, &lora_ptr->wq, wait);        
     
    if(lora_ptr->statu == RF_RX_DONE)     
    {
        mask |= POLLIN | POLLRDNORM;    
    }        
        
    return mask;    
}  

static int drv_release(struct inode *inode, struct file *filp)
{
    lora_info_st_ptr    lora_ptr;
    int            status = 0;
    
    lora_ptr = filp->private_data;

    mutex_lock(&device_list_lock);

    cancel_delayed_work(&lora_ptr->delaywork);
    lora_IoIrqDeInit(&lora_ptr->gpio); 
    lora_IoDeInit(&lora_ptr->gpio);

    filp->private_data = NULL;

    if(lora_ptr)
        kfree(lora_ptr);
    mutex_unlock(&device_list_lock);
    atomic_inc(&lora_ptr->opened);

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
static int drv_probe(struct spi_device *spi_ptr)
{
    int result = 0;
    lora_info_st_ptr lora_ptr = NULL;
    unsigned long        minor;

    /* Allocate driver data */
    lora_ptr = kzalloc(sizeof(*lora_ptr), GFP_KERNEL);
    if(!lora_ptr)
        return -ENOMEM;

    result = lora_parse_dt(&spi_ptr->dev, &lora_ptr->gpio);
    if (result < 0)
    {
        goto ERR_EXIT;
    }
    
    /* Initialize the driver data */
    //spin_lock_init(&lora_ptr->spi_lock);
    //mutex_init(&lora_ptr->buf_lock);

    INIT_LIST_HEAD(&lora_ptr->device_entry);

    /* If we can allocate a minor number, hook up this device.
     * Reusing minors is fine so long as udev or mdev is working.
     */
    mutex_lock(&device_list_lock);
    minor = find_first_zero_bit(minors, N_SPI_MINORS);
    if (minor < N_SPI_MINORS) {
        struct device *dev;

        lora_ptr->devt = MKDEV(DRV_MAJOR, minor);
        dev = device_create(drv_class, &spi_ptr->dev, lora_ptr->devt, lora_ptr, "%s%d", DRV_NAME, spi_ptr->master->bus_num);
        result = PTR_ERR_OR_ZERO(dev);
        if (result == 0) 
        {
            set_bit(minor, minors);
            list_add(&lora_ptr->device_entry, &device_list);
            //Init the open count
            atomic_set(&lora_ptr->opened, 1);
            //初始化等待队列
            mutex_init(&lora_ptr->mutex);
            mutex_init(&lora_ptr->buf_mutex);
            init_completion(&lora_ptr->lora_completion);
            init_waitqueue_head(&lora_ptr->wq);
            init_waitqueue_head(&lora_ptr->lora_rq);
            INIT_DELAYED_WORK(&lora_ptr->delaywork, lora_delaywork_func);
            INIT_WORK(&lora_ptr->work, lora_work_func);
            
            spi_set_drvdata(spi_ptr, lora_ptr);
        }
    } else {
        printk(KERN_ERR "no minor number available!\n");
        result = -ENODEV;
        goto ERR_EXIT;
    }
    
    mutex_unlock(&device_list_lock);

    if(result == 0)
    {
        /*config spi mode POL=0 PHA=0*/
        spi_ptr->mode = SPI_MODE_0;
        spi_setup(spi_ptr);    

        lora_ptr->sx1278.spi_ptr = spi_ptr;
    }else{
        goto ERR_EXIT;
    }
    return result;
    
ERR_EXIT:
    
    if(lora_ptr)
    {
        kfree(lora_ptr);
    }
    
    return result;
}

static int drv_remove(struct spi_device *spi)
{
    lora_info_st_ptr lora_ptr = spi_get_drvdata(spi);

    /* make sure ops on existing fds can abort cleanly */
    //spin_lock_irq(&drv_info.spi_lock);
    //drv_info.spi = NULL;
    //spin_unlock_irq(&drv_info.spi_lock);

    /* prevent new opens */
    mutex_lock(&device_list_lock);
    list_del(&lora_ptr->device_entry);
    device_destroy(drv_class, lora_ptr->devt);
    clear_bit(MINOR(lora_ptr->devt), minors);
    //if (drv_info.users == 0)
    //    kfree(drv_info_ptr);
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
