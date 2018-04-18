/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : drv_sx1278.h
 @brief  : the head file of lora sx1278 module driver.
 @author : pengrui
 @history:
           2018-04-13    pengrui    Created file
           ...
******************************************************************************/
#ifndef __DRV_SX1278_H__
#define __DRV_SX1278_H__
#include <linux/types.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include <linux/completion.h>

#include "sx1278.h"

typedef struct _drv_info_st 
{
    char        	*drv_name;
    dev_t		devt;
    spinlock_t		spi_lock;
    struct spi_device	*spi;
    struct list_head	device_entry;

    /* buffer is NULL unless this device is open (users > 0) */
    struct mutex		buf_lock;

    struct mutex		lora_mutex;
    unsigned			users;
    uint8_t			*buffer;
    /*0: success 1:send timeout 2:rx timeout*/
    int flag;

    wait_queue_head_t lora_wq;

    struct completion *lora_complete;
    
    struct delayed_work lora_work;
    
    SX1278_Gpio_st sx2178_gpio;

    RadioSettings_t sx1278_cfg;
}drv_info_st, *drv_info_st_ptr;



#endif/*__DRV_SX1278_H__*/
