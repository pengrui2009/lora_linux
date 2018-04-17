/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : sx1278.h
 @brief  : the head file of lora sx1278 module driver.
 @author : pengrui
 @history:
           2018-04-13    pengrui    Created file
           ...
******************************************************************************/

#ifndef __SX1278_H__
#define __SX1278_H__
#include <linux/types.h>
#include <stdbool.h>
#include <linux/spi/spi.h>
#include <linux/irqreturn.h>
#include <linux/interrupt.h>

#include "sx1278Regs-Fsk.h"
#include "sx1278Regs-LoRa.h"

/*!
 * Radio FSK modem parameters
 */
/*!
 * Radio driver supported modems
 */
typedef enum
{
    MODEM_FSK = 0,
    MODEM_LORA,
}RadioModems_en;

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

/*
 * Local types definition
 */

/*!
 * Radio registers definition
 */
typedef struct
{
    RadioModems_en Modem;
    unsigned char       Addr;
    unsigned char       Value;
}RadioRegisters_t;

/*!
 * FSK bandwidth definition
 */
typedef struct
{
    unsigned int bandwidth;
    unsigned char  RegValue;
}FskBandwidth_t;

/*!
 * SX1276 LoRa General parameters definition
 */
typedef struct _Radio_LoRa_config_st
{
    uint32_t RFFrequency;
    int8_t Power;
    uint8_t SignalBw;                   // LORA [0: 7.8 kHz, 1: 10.4 kHz, 2: 15.6 kHz, 3: 20.8 kHz, 4: 31.2 kHz,
                                        // 5: 41.6 kHz, 6: 62.5 kHz, 7: 125 kHz, 8: 250 kHz, 9: 500 kHz, other: Reserved]  
    uint8_t SpreadingFactor;            // LORA [6: 64, 7: 128, 8: 256, 9: 512, 10: 1024, 11: 2048, 12: 4096  chips]
    uint8_t ErrorCoding;                // LORA [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
    bool CrcOn;                         // [0: OFF, 1: ON]
    bool ImplicitHeaderOn;              // [0: OFF, 1: ON]
    bool RxSingleOn;                    // [0: Continuous, 1 Single]
    bool FreqHopOn;                     // [0: OFF, 1: ON]
    uint8_t HopPeriod;                  // Hops every frequency hopping period symbols
    uint32_t TxPacketTimeout;
    uint32_t RxPacketTimeout;
    uint8_t PayloadLength;
}Radio_LoRa_config_st, *Radio_LoRa_config_st_ptr;


typedef struct
{
    int8_t   Power;
    uint32_t Fdev;
    uint32_t Bandwidth;
    uint32_t BandwidthAfc;
    uint32_t Datarate;
    uint16_t PreambleLen;
    bool     FixLen;
    uint8_t  PayloadLen;
    bool     CrcOn;
    bool     IqInverted;
    bool     RxContinuous;
    uint32_t TxTimeout;
    uint32_t RxSingleTimeout;
}RadioFskSettings_st;

/*!
 * Radio FSK packet handler state
 */
typedef struct
{
    uint8_t  PreambleDetected;
    uint8_t  SyncWordDetected;
    int8_t   RssiValue;
    int32_t  AfcValue;
    uint8_t  RxGain;
    uint16_t Size;
    uint16_t NbBytes;
    uint8_t  FifoThresh;
    uint8_t  ChunkSize;
}RadioFskPacketHandler_st;

/*!
 * Radio LoRa modem parameters
 */
typedef struct
{
    int8_t   Power;
    uint32_t Bandwidth;
    uint32_t Datarate;
    bool     LowDatarateOptimize;
    uint8_t  Coderate;
    uint16_t PreambleLen;
    bool     FixLen;
    uint8_t  PayloadLen;
    bool     CrcOn;
    bool     FreqHopOn;
    uint8_t  HopPeriod;
    bool     IqInverted;
    bool     RxContinuous;
    uint32_t TxTimeout;
    bool     PublicNetwork;
}RadioLoRaSettings_st;

/*!
 * Radio LoRa packet handler state
 */
typedef struct
{
    int8_t  SnrValue;
    int16_t RssiValue;
    uint8_t Size;
}RadioLoRaPacketHandler_st;

/*!
 * Radio Settings
 */
typedef struct
{
    RadioState_en             State;
    RadioModems_en            Modem;
    uint32_t                  Channel;
    RadioFskSettings_st       Fsk;
    RadioFskPacketHandler_st  FskPacketHandler;
    RadioLoRaSettings_st      TxLoRa;
    RadioLoRaSettings_st      RxLoRa;
    RadioLoRaPacketHandler_st LoRaPacketHandler;
}RadioSettings_st;

/*!
 * Radio hardware and global parameters
 */
typedef struct _Sx1278_Gpio_st
{
    unsigned        Reset;
    unsigned        DIO0;
    unsigned        DIO1;
    unsigned        DIO2;
    unsigned        DIO3;
    unsigned        DIO4;
    unsigned        DIO5;
}Sx1278_Gpio_st, *Sx1278_Gpio_st_ptr;

/*!
 * sx1278 module info 
 */
typedef struct _Sx1278_info_st
{
    struct spi_device *spi;
    Sx1278_Gpio_st  sx1278_gpio;
    RadioSettings_st Settings;
}Sx1278_info_st, *Sx1278_info_st_ptr;

/*!
 * Hardware IO IRQ callback function definition
 */
typedef irqreturn_t (DioIrqHandler)(int irq, void *dev_id);

irqreturn_t SX1278_OnDio0Irq(int irq, void *dev_id);
irqreturn_t SX1278_OnDio1Irq(int irq, void *dev_id);
irqreturn_t SX1278_OnDio2Irq(int irq, void *dev_id);
irqreturn_t SX1278_OnDio3Irq(int irq, void *dev_id);
irqreturn_t SX1278_OnDio4Irq(int irq, void *dev_id);

int SX1278_SetChannel(Sx1278_info_st_ptr sx1278_ptr, unsigned int freq);
int SX1278_SetOpMode(struct spi_device *spi_ptr, unsigned char opMode);
int SX1278_Init(struct spi_device *spi, Sx1278_Gpio_st_ptr sx1278_gpio_ptr);



#endif /*__SX1278_H__*/
