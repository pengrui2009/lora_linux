#ifndef __SX1278_H__
#define __SX1278_H__

/*!
 * SX1276 LoRa General parameters definition
 */
typedef struct _LoRa_Tx_Config_st_
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
}LoRa_Tx_Config_st, *LoRa_Tx_Config_st_ptr;

typedef struct _LoRa_Rx_Config_st_
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
}LoRa_Rx_Config_st, *LoRa_Rx_Config_st_ptr;

typedef struct _LoRa_Config_st_
{
    LoRa_Tx_Config_st tx_cfg;
    LoRa_Rx_Config_st rx_cfg;
}LoRa_Config_st, *LoRa_Config_st_ptr;


int SX1278_Read_Reg(uint8_t addr, uint8_t *data_ptr);

int SX1278_Write_Reg(uint8_t addr, uint8_t data);

int SX1278_Read_FIFO(uint8_t *data_ptr, uint8_t data_len);

int SX1278_Write_FIFO(uint8_t *data_ptr, uint8_t data_len);

void SX1278LoRaSetDefaults( void );

void SX1278Init( void );

void SX1278Reset( void );

void SX1278SetLoRaOn( bool enable );

bool SX1278GetLoRaOn( void );

void SX1278SetOpMode( uint8_t opMode );

uint8_t SX1278GetOpMode( void );

double SX1278ReadRssi( void );

uint8_t SX1278ReadRxGain( void );

uint8_t SX1278GetPacketRxGain( void );

int8_t SX1278GetPacketSnr( void );

double SX1278GetPacketRssi( void );

uint32_t SX1278GetPacketAfc( void );

void SX1278StartRx( void );

void SX1278GetRxPacket( void *buffer, uint16_t *size );

void SX1278SetTxPacket( const void *buffer, uint16_t size );

uint8_t SX1278GetRFState( void );

void SX1278SetRFState( uint8_t state );

uint32_t SX1278Process( void );

#endif/*__SX1278_H__*/