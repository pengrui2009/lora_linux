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

#define RXBUFFER_SIZE       5
#define SX1278_TX_MAX_LENGTH    256
#define SX1278_RX_MAX_LENGTH    256
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

typedef enum
{
    RF_ERROR = 0,   //!< The radio is idle
    RF_RX_DONE, //!< The radio is in reception state
    RF_TX_DONE, //!< The radio is in transmission state
    RF_CAD_DONE,        //!< The radio is doing channel activity detection
    RF_RX_TIMEOUT,
    RF_TX_TIMEOUT,
}RadioStatus_en;


typedef struct _lora_data_st_
{
    //the data buffer of tx or rx
    uint8_t *data_ptr;
    //totoal data_len we rx or tx
    uint8_t data_len;
    //finish tx or rx length
    uint8_t doing_len;
    //current tx or rx length
    uint8_t done_len;
}lora_data_st, *lora_data_st_ptr;

typedef struct _lora_rxdata_st_
{
    uint8_t data[FIFO_BUFFER_SIZE];

    uint8_t data_len;
    //need to recv length
    uint8_t recv_len;
    
    int8_t snr;

    int16_t rssi;

}lora_rxdata_st, *lora_rxdata_st_ptr;

typedef struct _lora_rxbuff_st_
{
    lora_rxdata_st rxinfo[RXBUFFER_SIZE];
    
    uint8_t read_size;

    uint8_t write_size;
}lora_rx_st, *lora_rx_st_ptr;

struct sx1278_platform_data {
    SX1278_Gpio_st gpio;

    int irq_dio0;
    int irq_dio1;
    int irq_dio2;
    int irq_dio3;
    int irq_dio4;

};

struct sx1278_transfer_buffer {
	uint8_t rx_buf[SX1278_RX_MAX_LENGTH];
	uint8_t tx_buf[SX1278_RX_MAX_LENGTH] ____cacheline_aligned;
};


struct lora_transfer_function {
	int (*write)(struct device *spi, u8 addr, int len, u8 *data);
	int (*read)(struct device *spi, u8 addr, int len, u8 *data);
};


typedef struct _lora_info_st 
{
    char        	*drv_name;
    dev_t		    devt;
    struct sx1278_platform_data plat_data;
    uint16_t bus_type;
    struct mutex lock;
    atomic_t 		opened;
    spinlock_t		spinlock;
//    struct spi_device	*spi_ptr;
    struct list_head	device_entry;

    /* buffer is NULL unless this device is open (users > 0) */
    struct mutex		buf_mutex;
    
    struct mutex		mutex;
    unsigned			users;
    uint8_t			*buffer;
    /*0: success 1:send timeout 2:rx timeout*/
    int flag;

    uint8_t           lora_recv_state;
    uint8_t           lora_send_state;

    wait_queue_head_t lora_rq;
    wait_queue_head_t wq;

    struct completion lora_completion;
    /*the tx timeout handler work*/
    struct delayed_work delaywork;
    /*the work of irq bottom handler*/
    struct work_struct work;
    /*the buffer of send*/
    lora_data_st txdata;

    lora_rxdata_st rxdata;
    /**/
    int8_t rx_snr;
    /**/
    int16_t rx_rssi;

    const struct lora_transfer_function *tf;
    struct sx1278_transfer_buffer tb;
    
    /*the gpio pin info of sx1278*/
    //SX1278_Gpio_st gpio;
    /*the spi and config info of sx1278*/
    SX1278_st sx1278;
    /*the state of lora module*/
    RadioState_en state;

    RadioStatus_en statu;
}lora_info_st, *lora_info_st_ptr;



#endif/*__DRV_LORA_H__*/
