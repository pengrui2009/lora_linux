/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : drv_lib.h
 @brief  : the interface of lora sx1278 module.
 @author : pengrui
 @history:
           2018-04-13    pengrui    Created file
           ...
******************************************************************************/
#ifndef __DRV_LIB_H__
#define __DRV_LIB_H__
#ifdef __KERNEL__
#include <linux/types.h>
#include <linux/ioctl.h>
#else
#include <stdint.h>
#include <sys/ioctl.h>
#endif

#define SX127X_IOC_MAGIC            0x3F

enum sx127x_modulation {
    SX127X_MODULATION_FSK,
    SX127X_MODULATION_OOK,
    SX127X_MODULATION_LORA,
    SX127X_MODULATION_INVALID
};

/* the last 3 modes are only valid in lora mode */
enum sx127x_opmode {
    SX127X_OPMODE_SLEEP,
    SX127X_OPMODE_STANDBY,
    SX127X_OPMODE_FSTX,
    SX127X_OPMODE_TX,
    SX127X_OPMODE_FSRX,
    SX127X_OPMODE_RX,
    SX127X_OPMODE_RXCONTINUOS,
    SX127X_OPMODE_RXSINGLE,
    SX127X_OPMODE_CAD
};

enum sx127x_pa {
    SX127X_PA_RFO,
    SX127X_PA_PABOOST
};

//SET_MODEM param
#define MODEM_FSK_M       0
#define MODEM_LORA_M      1

//SET FREQ
#define SX1278_MIN_FREQ     137000000
#define SX1278_MAX_FREQ     525000000

//SET_BW
#define BW_78KHZ             0//78kHz
#define BW_10_4KHZ           1//10.4kHz
#define BW_15_6KHZ           2//15.6kHz
#define BW_20_8KHZ           3//20.8kHz
#define BW_31_25KHZ          4//31.25kHz
#define BW_41_7KHZ           5//41.7kHz
#define BW_62_5KHZ           6//62.5kHz
#define BW_125KHZ            7//125kHz
#define BW_250KHZ            8//250kHz
#define BW_500KHZ            9//500kHz
//notice when freq <= 169Mhz, bw is not support 250kHz and 500kHz

//SET SPREAD FACTOR
#define SF_64_CHIPS_SYMBOL      6
#define SF_128_CHIPS_SYMBOL     7
#define SF_256_CHIPS_SYMBOL     8
#define SF_512_CHIPS_SYMBOL     9
#define SF_1024_CHIPS_SYMBOL    10
#define SF_2048_CHIPS_SYMBOL    11
#define SF_4096_CHIPS_SYMBOL    12

//SET CODE RATE
#define CR_4_5                  1
#define CR_4_6                  2
#define CR_4_7                  3
#define CR_4_8                  4

//SET PA OUTPUT
#define PA_RFO                  0
#define PA_BOOST                1

//SET POWER
//when pa is RFO then min power -1 , max power 14
//when pa is BOOST then min power 2 , max power 20

//SET_FDEV
//not support now

//SET_PREAMBLELEN

//SET_OPMODE
#define OPMODE_SLEEP            0
#define OPMODE_STANDBY          1
#define OPMODE_FSTX             2
#define OPMODE_TX               3
#define OPMODE_FSRX             4
#define OPMODE_RX               5
#define OPMODE_RXCONTINUOS      5
#define OPMODE_RXSINGLE         6
#define OPMODE_CAD              7

//START WORK
//start work

//START CAD
//start cad

/* Write modulation */
#define SET_MODEM                       _IO(SX127X_IOC_MAGIC, 1)

/* Write Tx carrie frequency */
#define SET_TXFREQ                      _IO(SX127X_IOC_MAGIC, 2)

/* Write Rx carrie frequency */
#define SET_RXFREQ                      _IO(SX127X_IOC_MAGIC, 3)

/* Write Tx bandwidth */
#define SET_TXBW                        _IO(SX127X_IOC_MAGIC, 4)

/* Write Rx bandwidth */
#define SET_RXBW                        _IO(SX127X_IOC_MAGIC, 5)

/*  Write Tx Spread Factor */
#define SET_TXSF                        _IO(SX127X_IOC_MAGIC, 6)

/* Write Rx Spread Factor*/
#define SET_RXSF                        _IO(SX127X_IOC_MAGIC, 7)

/* Write Tx CodeRate*/
#define SET_TXCR                        _IO(SX127X_IOC_MAGIC, 8)

/* Write Rx CodeRate*/
#define SET_RXCR                        _IO(SX127X_IOC_MAGIC, 9)

/* Write PA OUTPUT */
#define SET_PAOUTPUT                    _IO(SX127X_IOC_MAGIC, 10)

/* Write TX POWER */
#define SET_TXPOWER                     _IO(SX127X_IOC_MAGIC, 11)

//FSK mode ioctl cmd
#define SET_FDEV                        _IO(SX127X_IOC_MAGIC, 12)

//write preamble len
#define SET_PREAMBLELEN                 _IO(SX127X_IOC_MAGIC, 13)

/* Write sync word*/
#define SET_OPMODE                      _IO(SX127X_IOC_MAGIC, 14)

/* WRITE tx timeout uint:ms*/
#define SET_TXTIMEOUT                   _IO(SX127X_IOC_MAGIC, 15)

//START WORK
#define SET_RECVDATA                    _IO(SX127X_IOC_MAGIC, 16)

//arg {bit[0] : [0:nothing detected  1: detected] }
#define START_CAD                       _IO(SX127X_IOC_MAGIC, 17)

#define START_RSSI						_IO(SX127X_IOC_MAGIC, 18)
#endif
