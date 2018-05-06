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

#define SX127X_IOC_MAGIC			0x3F

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


typedef struct _Tx_Config_st_
{
    unsigned int RFFrequency;
    char Power;
    unsigned char SignalBw;                   // LORA [0: 7.8 kHz, 1: 10.4 kHz, 2: 15.6 kHz, 3: 20.8 kHz, 4: 31.2 kHz,
                                        // 5: 41.6 kHz, 6: 62.5 kHz, 7: 125 kHz, 8: 250 kHz, 9: 500 kHz, other: Reserved]  
    unsigned char SpreadingFactor;            // LORA [6: 64, 7: 128, 8: 256, 9: 512, 10: 1024, 11: 2048, 12: 4096  chips]
    unsigned char ErrorCoding;                // LORA [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
    bool CrcOn;                         // [0: OFF, 1: ON]
    bool ImplicitHeaderOn;              // [0: OFF, 1: ON]
    bool FreqHopOn;                     // [0: OFF, 1: ON]
    bool IqInverted;                    // [0: OFF, 1: ON]
    unsigned char HopPeriod;                  // Hops every frequency hopping period symbols
    unsigned int TxPacketTimeout;
    unsigned char PayloadLength;
}Tx_Config_st, *Tx_Config_st_ptr;

typedef struct _Rx_Config_st_
{
    unsigned int RFFrequency;
    unsigned char SignalBw;                   // LORA [0: 7.8 kHz, 1: 10.4 kHz, 2: 15.6 kHz, 3: 20.8 kHz, 4: 31.2 kHz,
                                        // 5: 41.6 kHz, 6: 62.5 kHz, 7: 125 kHz, 8: 250 kHz, 9: 500 kHz, other: Reserved]  
    unsigned char SpreadingFactor;            // LORA [6: 64, 7: 128, 8: 256, 9: 512, 10: 1024, 11: 2048, 12: 4096  chips]
    unsigned char ErrorCoding;                // LORA [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
    bool CrcOn;                         // [0: OFF, 1: ON]
//    bool ImplicitHeaderOn;              // [0: OFF, 1: ON]
//    bool RxSingleOn;                    // [0: Continuous, 1 Single]
    bool FreqHopOn;                     // [0: OFF, 1: ON]
    bool IqInverted;                    // [0: OFF, 1: ON]
    unsigned char HopPeriod;                  // Hops every frequency hopping period symbols
//    unsigned int RxPacketTimeout;
    unsigned char PayloadLength;
}Rx_Config_st, *Rx_Config_st_ptr;

typedef struct _Rx_Signal_Param_st_
{
    char snr;

    short rssi;
}Rx_Signal_Param_st, *Rx_Signal_Param_st_ptr;


struct sx127x_cfg {
    enum sx127x_modulation modulation;
    u32 freq;
    u8 sf;
    enum sx127x_pa pa;
    u8 bandwidth;
    u8 syncword;
    bool crc;
    bool invertiq;  
};

/* Read or Write modulation */
#define LORA_IOC_RD_MODULATION			_IOR(SX127X_IOC_MAGIC, 1, uint8_t)
#define LORA_IOC_WR_MODULATION			_IOW(SX127X_IOC_MAGIC, 1, uint8_t)

/* Read / Write carrie frequency */
#define LORA_IOC_RD_CARRIERFRE		    _IOR(SX127X_IOC_MAGIC, 2, uint64_t)
#define LORA_IOC_WR_CARRIERFRE		    _IOW(SX127X_IOC_MAGIC, 2, uint64_t)

/* Read / Write spreading factoe */
#define LORA_IOC_RD_SF	                _IOR(SX127X_IOC_MAGIC, 3, uint8_t)
#define LORA_IOC_WR_SF	                _IOW(SX127X_IOC_MAGIC, 3, uint8_t)

/* Read / Write operation mode */
#define LORA_IOC_RD_PAOUTPUT	        _IOR(SX127X_IOC_MAGIC, 4, uint8_t)
#define LORA_IOC_WR_PAOUTPUT          	_IOW(SX127X_IOC_MAGIC, 4, uint8_t)

/* Read / Write bandwidth */
#define LORA_IOC_RD_BANDWIDTH           _IOR(SX127X_IOC_MAGIC, 5, uint8_t)
#define LORA_IOC_WR_BANDWIDTH           _IOW(SX127X_IOC_MAGIC, 5, uint8_t)

/* Read / Write sync word*/
#define LORA_IOC_RD_SYNCWORD            _IOR(SX127X_IOC_MAGIC, 6, uint8_t)
#define LORA_IOC_WR_SYNCWORD            _IOW(SX127X_IOC_MAGIC, 6, uint8_t)

/* Read / Write crc*/
#define LORA_IOC_RD_CRC                 _IOR(SX127X_IOC_MAGIC, 7, Tx_Config_st)
#define LORA_IOC_WR_CRC                 _IOW(SX127X_IOC_MAGIC, 7, Tx_Config_st)

/* Read / Write invert iq*/
#define LORA_IOC_RD_INVERTIQ            _IOR(SX127X_IOC_MAGIC, 8, uint8_t)
#define LORA_IOC_WR_INVERTIQ            _IOW(SX127X_IOC_MAGIC, 8, uint8_t)

/* Read / Write sync word*/
#define LORA_IOC_RD_OPMODE              _IOR(SX127X_IOC_MAGIC, 9, uint8_t)
#define LORA_IOC_WR_OPMODE              _IOW(SX127X_IOC_MAGIC, 9, uint8_t)

#define LORA_IOC_WR_WORK                _IOWR(SX127X_IOC_MAGIC, 10, uint8_t)

#endif
