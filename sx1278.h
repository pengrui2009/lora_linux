#ifndef __SX1278_H__
#define __SX1278_H__

#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include "sx1276Regs-LoRa.h"

#define XTAL_FREQ                                   32000000
#define FREQ_STEP                                   6103515625
#define RSSI_OFFSET_LF                              -164
#define RSSI_OFFSET_HF                              -157

/*!
 * SX1276 LoRa General parameters definition
 */
typedef struct _SX1278_Tx_Config_st_
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
    uint8_t HopPeriod;                  // Hops every frequency hopping period symbols
    uint32_t TxPacketTimeout;
    uint8_t PayloadLength;
}SX1278_Tx_Config_st, *SX1278_Tx_Config_st_ptr;

typedef struct _SX1278_Rx_Config_st_
{
    uint32_t RFFrequency;
    uint8_t SignalBw;                   // LORA [0: 7.8 kHz, 1: 10.4 kHz, 2: 15.6 kHz, 3: 20.8 kHz, 4: 31.2 kHz,
                                        // 5: 41.6 kHz, 6: 62.5 kHz, 7: 125 kHz, 8: 250 kHz, 9: 500 kHz, other: Reserved]  
    uint8_t SpreadingFactor;            // LORA [6: 64, 7: 128, 8: 256, 9: 512, 10: 1024, 11: 2048, 12: 4096  chips]
    uint8_t ErrorCoding;                // LORA [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
    bool CrcOn;                         // [0: OFF, 1: ON]
    bool ImplicitHeaderOn;              // [0: OFF, 1: ON]
    bool RxSingleOn;                    // [0: Continuous, 1 Single]
    bool FreqHopOn;                     // [0: OFF, 1: ON]
    uint8_t HopPeriod;                  // Hops every frequency hopping period symbols
    uint32_t RxPacketTimeout;
    uint8_t PayloadLength;
}SX1278_Rx_Config_st, *SX1278_Rx_Config_st_ptr;

typedef struct _SX1278_Cfg_st_
{
    SX1278_Tx_Config_st tx_cfg;
    SX1278_Rx_Config_st rx_cfg;
}SX1278_Cfg_st, *SX1278_Cfg_st_ptr;

typedef struct _SX1278_st_
{
    struct spi_device *spi_ptr;
    
    SX1278_Cfg_st cfg;
}SX1278_st, *SX1278_st_ptr;
/*!
 * Radio hardware and global parameters
 */
typedef struct _SX1278_Gpio_st_
{
    unsigned        Reset;
    unsigned        DIO0;
    unsigned        DIO1;
    unsigned        DIO2;
    unsigned        DIO3;
    unsigned        DIO4;
    unsigned        DIO5;
}SX1278_Gpio_st, *SX1278_Gpio_st_ptr;

int SX1278_Read_Reg(SX1278_st_ptr sx1278_ptr, uint8_t addr, uint8_t *data_ptr);

int SX1278_Write_Reg(SX1278_st_ptr sx1278_ptr, uint8_t addr, uint8_t data);

int SX1278_Read_FIFO(SX1278_st_ptr sx1278_ptr, uint8_t *data_ptr, uint8_t data_len);

int SX1278_Write_FIFO(SX1278_st_ptr sx1278_ptr, uint8_t *data_ptr, uint8_t data_len);

int SX1278LoRaSetDefaults(SX1278_st_ptr sx1278_ptr);

int SX1278Reset(SX1278_st_ptr sx1278_ptr);

int SX1278SetLoRaOn(SX1278_st_ptr sx1278_ptr, bool enable);

int SX1278GetLoRaOn(SX1278_st_ptr sx1278_ptr, bool *LoRaOn);

int SX1278SetOpMode(SX1278_st_ptr sx1278_ptr, uint8_t opMode);

int SX1278GetOpMode(SX1278_st_ptr sx1278_ptr, uint8_t *opmode);

int SX1278ReadRssi(SX1278_st_ptr sx1278_ptr, int8_t *rssi);

int SX1278ReadRxGain(SX1278_st_ptr sx1278_ptr, uint8_t *rxgain);

int SX1278GetPacketRxGain(SX1278_st_ptr sx1278_ptr, uint8_t *PacketRxGain);

int SX1278GetPacketSnr(SX1278_st_ptr sx1278_ptr, int8_t *pakcetsnr);

int SX1278GetPacketRssi(SX1278_st_ptr sx1278_ptr, int8_t *packetrssi);

int SX1278GetPacketAfc(SX1278_st_ptr sx1278_ptr, uint32_t *packetafc);

int SX1278StartRx(SX1278_st_ptr sx1278_ptr);

int SX1278GetRxPacket(SX1278_st_ptr sx1278_ptr, uint8_t *buffer_ptr, uint8_t buffer_len);

int SX1278SetTxPacket(SX1278_st_ptr sx1278_ptr, const uint8_t *buffer_ptr, uint8_t buffer_len);

int SX1278GetRFState(SX1278_st_ptr sx1278_ptr, uint8_t *RFState);

int SX1278SetRFState(SX1278_st_ptr sx1278_ptr, uint8_t state);

int SX1278Process(SX1278_st_ptr sx1278_ptr);

int SX1278LoRaSetRFFrequency(SX1278_st_ptr sx1278_ptr, uint32_t freq);

int SX1278LoRaSetSpreadingFactor(SX1278_st_ptr sx1278_ptr, uint8_t factor);

int SX1278LoRaSetErrorCoding(SX1278_st_ptr sx1278_ptr, uint8_t value);

int SX1278LoRaSetPacketCrcOn(SX1278_st_ptr sx1278_ptr, bool enable);

int SX1278LoRaSetSignalBandwidth(SX1278_st_ptr sx1278_ptr, uint8_t bw);

int SX1278LoRaSetImplicitHeaderOn(SX1278_st_ptr sx1278_ptr, bool enable);

int SX1278LoRaSetSymbTimeout(SX1278_st_ptr sx1278_ptr, uint16_t value);

int SX1278LoRaSetPayloadLength(SX1278_st_ptr sx1278_ptr, uint8_t value);

int SX1278LoRaSetLowDatarateOptimize(SX1278_st_ptr sx1278_ptr, bool enable);

int SX1278LoRaSetPAOutput(SX1278_st_ptr sx1278_ptr, uint8_t outputPin);

int SX1278LoRaSetPa20dBm(SX1278_st_ptr sx1278_ptr, bool enale);

int SX1278LoRaSetRFPower(SX1278_st_ptr sx1278_ptr, int8_t power);

int SX1278LoRaSetOpMode(SX1278_st_ptr sx1278_ptr, uint8_t opMode);

int SX1278LoRaGetOpMode(SX1278_st_ptr sx1278_ptr, uint8_t *opmode);

int SX1278LoRaSetNbTrigPeaks(SX1278_st_ptr sx1278_ptr, uint8_t value);

int SX1278Init(SX1278_st_ptr sx1278_ptr);

int SX1278TxFinished(SX1278_st_ptr sx1278_ptr);

int SX1278StartTx(SX1278_st_ptr sx1278_ptr);

int SX1278RxClearIrq(SX1278_st_ptr sx1278_ptr);
#endif/*__SX1278_H__*/
