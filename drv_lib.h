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


/*lora payload crc error*/
#define ECRC            35

typedef struct _Tx_Config_st_
{
    uint32_t RFFrequency;
    int8_t Power;
    uint8_t SignalBw;                   // LORA [0: 7.8 kHz, 1: 10.4 kHz, 2: 15.6 kHz, 3: 20.8 kHz, 4: 31.2 kHz,
                                        // 5: 41.6 kHz, 6: 62.5 kHz, 7: 125 kHz, 8: 250 kHz, 9: 500 kHz, other: Reserved]  
    uint8_t SpreadingFactor;            // LORA [6: 64, 7: 128, 8: 256, 9: 512, 10: 1024, 11: 2048, 12: 4096  chips]
    uint8_t ErrorCoding;                // LORA [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
    bool CrcOn;                         // [0: OFF, 1: ON]
    bool ImplicitHeaderOn;              // [0: OFF, 1: ON]
    bool FreqHopOn;                     // [0: OFF, 1: ON]
    bool IqInverted;                    // [0: OFF, 1: ON]
    uint8_t HopPeriod;                  // Hops every frequency hopping period symbols
    uint32_t TxPacketTimeout;
    uint8_t PayloadLength;
}Tx_Config_st, *Tx_Config_st_ptr;

typedef struct _Rx_Config_st_
{
    uint32_t RFFrequency;
    uint8_t SignalBw;                   // LORA [0: 7.8 kHz, 1: 10.4 kHz, 2: 15.6 kHz, 3: 20.8 kHz, 4: 31.2 kHz,
                                        // 5: 41.6 kHz, 6: 62.5 kHz, 7: 125 kHz, 8: 250 kHz, 9: 500 kHz, other: Reserved]  
    uint8_t SpreadingFactor;            // LORA [6: 64, 7: 128, 8: 256, 9: 512, 10: 1024, 11: 2048, 12: 4096  chips]
    uint8_t ErrorCoding;                // LORA [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
    bool CrcOn;                         // [0: OFF, 1: ON]
//    bool ImplicitHeaderOn;              // [0: OFF, 1: ON]
//    bool RxSingleOn;                    // [0: Continuous, 1 Single]
    bool FreqHopOn;                     // [0: OFF, 1: ON]
    bool IqInverted;                    // [0: OFF, 1: ON]
    uint8_t HopPeriod;                  // Hops every frequency hopping period symbols
//    uint32_t RxPacketTimeout;
    uint8_t PayloadLength;
}Rx_Config_st, *Rx_Config_st_ptr;

typedef struct _Rx_Signal_Param_st_
{
    int8_t snr;

    int16_t rssi;
}Rx_Signal_Param_st, *Rx_Signal_Param_st_ptr;

/* Read / Write of SPI mode (SPI_MODE_0..SPI_MODE_3) */
#define LORA_IOC_RD_MODE			_IOR(SPI_IOC_MAGIC, 1, __u8)
#define LORA_IOC_WR_MODE			_IOW(SPI_IOC_MAGIC, 1, __u8)

/* Read / Write SPI bit justification */
#define LORA_IOC_RD_LSB_FIRST		_IOR(SPI_IOC_MAGIC, 2, __u8)
#define LORA_IOC_WR_LSB_FIRST		_IOW(SPI_IOC_MAGIC, 2, __u8)

/* Read / Write SPI device word length (1..N) */
#define LORA_IOC_RD_BITS_PER_WORD	_IOR(SPI_IOC_MAGIC, 3, __u8)
#define LORA_IOC_WR_BITS_PER_WORD	_IOW(SPI_IOC_MAGIC, 3, __u8)

/* Read / Write SPI device default max speed hz */
#define LORA_IOC_RD_MAX_SPEED_HZ	_IOR(SPI_IOC_MAGIC, 4, __u32)
#define LORA_IOC_WR_MAX_SPEED_HZ	_IOW(SPI_IOC_MAGIC, 4, __u32)

#define LORA_IOC_RD_RXCONFIG        _IOR(SPI_IOC_MAGIC, 5, Rx_Config_st)
#define LORA_IOC_WD_RXCONFIG        _IOW(SPI_IOC_MAGIC, 5, Rx_Config_st)

#define LORA_IOC_RD_TXCONFIG        _IOR(SPI_IOC_MAGIC, 6, Tx_Config_st)
#define LORA_IOC_WD_TXCONFIG        _IOW(SPI_IOC_MAGIC, 6, Tx_Config_st)

#define LORA_IOC_RD_RXSIGNAL_PRAM   _IOR(SPI_IOC_MAGIC, 7, Rx_Signal_Param_st)

#define LORA_IOC_CAD                _IOWR(SPI_IOC_MAGIC, 8, __u8)
#endif
