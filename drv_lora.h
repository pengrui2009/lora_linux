/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : drv_lora.h
 @brief  : the head file of lora sx1278 module driver.
 @author : pengrui
 @history:
           2018-04-13    pengrui    Created file
           ...
******************************************************************************/
#ifndef __DRV_LORA_H__
#define __DRV_LORA_H__
#include <linux/types.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include <linux/completion.h>

#include "sx1278.h"

/*!
 * Radio driver internal state machine states definition
 */
typedef enum
{
    RF_IDLE = 0,   //!< The radio is idle
    RF_RX_RUNNING, //!< The radio is in reception state
    RF_TX_RUNNING, //!< The radio is in transmission state
    RF_CAD,        //!< The radio is doing channel activity detection
}RadioState_en;

typedef struct _lora_info_st 
{
    char        	*drv_name;
    dev_t		    devt;
    spinlock_t		spi_lock;
//    struct spi_device	*spi_ptr;
    struct list_head	device_entry;

    /* buffer is NULL unless this device is open (users > 0) */
    struct mutex		buf_lock;

    struct mutex		lora_mutex;
    unsigned			users;
    uint8_t			*buffer;
    /*0: success 1:send timeout 2:rx timeout*/
    int flag;

    uint8_t           lora_recv_state;
    uint8_t           lora_send_state;

    wait_queue_head_t lora_rq;
    wait_queue_head_t lora_wq;

    struct completion lora_completion;
    /*the tx timeout handler work*/
    struct delayed_work lora_delaywork;
    /*the work of irq bottom handler*/
    struct work_struct lora_work;

    /**/
    int8_t rx_snr;
    /**/
    int16_t rx_rssi;
    
    /*the gpio pin info of sx1278*/
    SX1278_Gpio_st gpio;
    /*the spi and config info of sx1278*/
    SX1278_st sx1278;
    /*the state of lora module*/
    RadioState_en state;
}lora_info_st, *lora_info_st_ptr;



#endif/*__DRV_LORA_H__*/
