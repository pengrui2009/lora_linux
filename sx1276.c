/*!
 * \file      sx1276.c
 *
 * \brief     SX1276 driver implementation
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 *
 * \author    Wael Guibene ( Semtech )
 */
#include <math.h>
#include <string.h>
#if defined( USE_RADIO_DEBUG )
#include "board-config.h"
#endif
#include "utilities.h"
#include "timer.h"
#include "radio.h"
#include "delay.h"
#include "sx1276.h"
#include "sx1276-board.h"

/*
 * Local types definition
 */

/*!
 * Radio registers definition
 */
typedef struct
{
    RadioModems_t Modem;
    uint8_t       Addr;
    uint8_t       Value;
}RadioRegisters_t;

/*!
 * FSK bandwidth definition
 */
typedef struct
{
    uint32_t bandwidth;
    uint8_t  RegValue;
}FskBandwidth_t;


/*
 * Private functions prototypes
 */

/*!
 * Performs the Rx chain calibration for LF and HF bands
 * \remark Must be called just after the reset so all registers are at their
 *         default values
 */
static void RxChainCalibration( void );

/*!
 * \brief Sets the SX1276 in transmission mode for the given time
 * \param [IN] timeout Transmission timeout [ms] [0: continuous, others timeout]
 */
void SX1276SetTx( uint32_t timeout );

/*!
 * \brief Writes the buffer contents to the SX1276 FIFO
 *
 * \param [IN] buffer Buffer containing data to be put on the FIFO.
 * \param [IN] size Number of bytes to be written to the FIFO
 */
void SX1276WriteFifo( uint8_t *buffer, uint8_t size );

/*!
 * \brief Reads the contents of the SX1276 FIFO
 *
 * \param [OUT] buffer Buffer where to copy the FIFO read data.
 * \param [IN] size Number of bytes to be read from the FIFO
 */
void SX1276ReadFifo( uint8_t *buffer, uint8_t size );

/*!
 * \brief Sets the SX1276 operating mode
 *
 * \param [IN] opMode New operating mode
 */
void SX1276SetOpMode( uint8_t opMode );

/*
 * SX1276 DIO IRQ callback functions prototype
 */

/*!
 * \brief DIO 0 IRQ callback
 */
void SX1276OnDio0Irq( void );

/*!
 * \brief DIO 1 IRQ callback
 */
void SX1276OnDio1Irq( void );

/*!
 * \brief DIO 2 IRQ callback
 */
void SX1276OnDio2Irq( void );

/*!
 * \brief DIO 3 IRQ callback
 */
void SX1276OnDio3Irq( void );

/*!
 * \brief DIO 4 IRQ callback
 */
void SX1276OnDio4Irq( void );

/*!
 * \brief DIO 5 IRQ callback
 */
void SX1276OnDio5Irq( void );

/*!
 * \brief Tx & Rx timeout timer callback
 */
void SX1276OnTimeoutIrq( void );

/*
 * Private global constants
 */

/*!
 * Radio hardware registers initialization
 *
 * \remark RADIO_INIT_REGISTERS_VALUE is defined in sx1276-board.h file
 */
const RadioRegisters_t RadioRegsInit[] = RADIO_INIT_REGISTERS_VALUE;

/*!
 * Constant values need to compute the RSSI value
 */
#define RSSI_OFFSET_LF                              -164
#define RSSI_OFFSET_HF                              -157

/*!
 * Precomputed FSK bandwidth registers values
 */
const FskBandwidth_t FskBandwidths[] =
{
    { 2600  , 0x17 },
    { 3100  , 0x0F },
    { 3900  , 0x07 },
    { 5200  , 0x16 },
    { 6300  , 0x0E },
    { 7800  , 0x06 },
    { 10400 , 0x15 },
    { 12500 , 0x0D },
    { 15600 , 0x05 },
    { 20800 , 0x14 },
    { 25000 , 0x0C },
    { 31300 , 0x04 },
    { 41700 , 0x13 },
    { 50000 , 0x0B },
    { 62500 , 0x03 },
    { 83333 , 0x12 },
    { 100000, 0x0A },
    { 125000, 0x02 },
    { 166700, 0x11 },
    { 200000, 0x09 },
    { 250000, 0x01 },
    { 300000, 0x00 }, // Invalid Bandwidth
};

/*
 * Private global variables
 */

/*!
 * Radio callbacks variable
 */
//static RadioEvents_t *RadioEvents;

/*!
 * Reception buffer
 */
static uint8_t RxTxBuffer[RX_BUFFER_SIZE];

/*
 * Public global variables
 */

/*!
 * Radio hardware and global parameters
 */
//SX1276_t SX1276;
static SX1278_st *sx1278_ptr;
/*!
 * Hardware DIO IRQ callback initialization
 */
/*DioIrqHandler *DioIrq[] = { SX1276OnDio0Irq, SX1276OnDio1Irq,
                            SX1276OnDio2Irq, SX1276OnDio3Irq,
                            SX1276OnDio4Irq, NULL };
*/

/*!
 * Tx and Rx timers
 */
//TimerEvent_t TxTimeoutTimer;
//TimerEvent_t RxTimeoutTimer;
//TimerEvent_t RxTimeoutSyncWord;



static int SX1278_Read_Reg(unsigned char addr, unsigned char *data_ptr)
{
    int result = 0;
    unsigned char reg = (addr & 0x7F);

    if((NULL == sx1278_ptr->spi_ptr) || (NULL == data_ptr))
    {
        printk(KERN_ERR "%s %d error %d reading %x\n", __FUNCTION__, __LINE__, ret, addr);
        result = -ENOMEM;
        goto ERR_EXIT;
    }
    
    result = spi_write_then_read(sx1278_ptr->spi_ptr, &reg, 1, data, 1);
    if (result < 0)
    {
        printk(KERN_ERR "%s %d error %d reading %x\n", __FUNCTION__, __LINE__, ret, addr);
        goto ERR_EXIT;
    }

ERR_EXIT:
    
    return result;
}

static int SX1278_Write_Reg(unsigned char addr, unsigned char data)
{
    int result = 0;
    unsigned char data_buf[2] = {0};
    data_buf[0] = (addr | 0x80);
    data_buf[1] = data;

    if(NULL == sx1278_ptr->spi_ptr)
    {
        printk(KERN_ERR "%s %d error %d\n", __FUNCTION__, __LINE__, result);
        result = -ENOMEM;
        goto ERR_EXIT;
    }

    result = spi_write(sx1278_ptr->spi_ptr, (void *)data_buf, 2);
    if (result < 0)
    {
        printk(KERN_ERR "%s %d write %d\n", __FUNCTION__, __LINE__, result);
        goto ERR_EXIT;
    }
    
    return result;
}

static int SX1278_Read_FIFO(uint8_t *data_ptr, uint32_t data_len)
{
    int result = 0;
    unsigned char reg = 0x00;
    struct spi_transfer t[2] = {0};
	struct spi_message m = {0};

    if((NULL == sx1278_ptr->spi_ptr) || (NULL == data_ptr))
    {
        result = -ENOMEM;
        goto ERR_EXIT;
    }

    pdata = kzalloc((data_len + 1), GFP_KERNEL);
    if (!pdata) 
    {
		result = -ENOMEM;
		goto ERR_EXIT;
	}

    spi_message_init(&m);
    
	t[0].tx_buf = &reg;
	t[0].len = 1;

  	t[1].rx_buf = data_ptr;
	t[1].len = data_len ;
    
	spi_message_add_tail(&t, &m);

    spi_sync(sx1278_ptr->spi_ptr, &m);

    *result += m.actual_length - 1;

ERR_EXIT:
    
    return result;
}

static int SX1278_Write_FIFO(uint8_t *data_ptr, uint8_t data_len)
{
    int result = 0;
    uint8_t *pdata = NULL;
    struct spi_transfer t = {0};
	struct spi_message m = {0};

    if((NULL == sx1278_ptr->spi_ptr) || (NULL == data_ptr))
    {
        result = -ENOMEM;
        goto ERR_EXIT;
    }

    pdata = kzalloc((data_len + 1), GFP_KERNEL);
    if (!pdata) 
    {
		result = -ENOMEM;
		goto ERR_EXIT;
	}

    pdata[0] = 0x80;
    memcpy(&pdata[1], data_ptr, data_len);

    spi_message_init(&m);
    
	t.tx_buf = pdata;
	t.len = (data_len + 1);
    
	spi_message_add_tail(&t, &m);

    spi_sync(sx1278_ptr->spi_ptr, &m);

    *result += m.actual_length - 1;
    
ERR_EXIT:

    if(pdata)
        kfree(pdata);
    
    return result;
}

/*
 * Radio driver functions implementation
 */

void SX1276Init(struct spi_device *spi_ptr, RadioSettings_t *radio_cfg_ptr/*RadioEvents_t *events*/ )
{
    uint8_t i;

	sx1278_ptr.spi_ptr = spi_ptr;
	sx1278_ptr.sx1278_cfg_ptr = radio_cfg_ptr;
    //RadioEvents = events;

    // Initialize driver timeout timers
    //TimerInit( &TxTimeoutTimer, SX1276OnTimeoutIrq );
    //TimerInit( &RxTimeoutTimer, SX1276OnTimeoutIrq );
    //TimerInit( &RxTimeoutSyncWord, SX1276OnTimeoutIrq );

    //SX1276Reset( );

    RxChainCalibration( );

    SX1276SetOpMode( RF_OPMODE_SLEEP );

    //SX1276IoIrqInit( DioIrq );

    for( i = 0; i < sizeof( RadioRegsInit ) / sizeof( RadioRegisters_t ); i++ )
    {
        SX1276SetModem( RadioRegsInit[i].Modem );
        SX1276Write( RadioRegsInit[i].Addr, RadioRegsInit[i].Value );
    }

    SX1276SetModem( MODEM_FSK );

    sx1278_ptr->sx1278_cfg_ptr->State = RF_IDLE;
	
}

RadioState_t SX1276GetStatus( void )
{
    return sx1278_ptr->sx1278_cfg_ptr->State;
}

void SX1276SetChannel( uint32_t freq )
{
    sx1278_ptr->sx1278_cfg_ptr->Channel = freq;
    freq = ( uint32_t )( ( double )freq / ( double )FREQ_STEP );
    SX1278_Write_Reg( REG_FRFMSB, ( uint8_t )( ( freq >> 16 ) & 0xFF ) );
    SX1278_Write_Reg( REG_FRFMID, ( uint8_t )( ( freq >> 8 ) & 0xFF ) );
    SX1278_Write_Reg( REG_FRFLSB, ( uint8_t )( freq & 0xFF ) );
}

bool SX1276IsChannelFree( RadioModems_t modem, uint32_t freq, int16_t rssiThresh, uint32_t maxCarrierSenseTime )
{
    bool status = true;
    int16_t rssi = 0;
    uint32_t carrierSenseTime = 0;

    SX1276SetModem( modem );

    SX1276SetChannel( freq );

    SX1276SetOpMode( RF_OPMODE_RECEIVER );

    mdelay( 1 );

    carrierSenseTime = TimerGetCurrentTime( );

    // Perform carrier sense for maxCarrierSenseTime
    while( TimerGetElapsedTime( carrierSenseTime ) < maxCarrierSenseTime )
    {
        rssi = SX1276ReadRssi( modem );

        if( rssi > rssiThresh )
        {
            status = false;
            break;
        }
    }
    SX1276SetSleep( );
    return status;
}

uint32_t SX1276Random( void )
{
    uint8_t i;
    uint32_t rnd = 0;

    /*
     * Radio setup for random number generation
     */
    // Set LoRa modem ON
    SX1276SetModem( MODEM_LORA );

    // Disable LoRa modem interrupts
    SX1278_Write_Reg( REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
                  RFLR_IRQFLAGS_RXDONE |
                  RFLR_IRQFLAGS_PAYLOADCRCERROR |
                  RFLR_IRQFLAGS_VALIDHEADER |
                  RFLR_IRQFLAGS_TXDONE |
                  RFLR_IRQFLAGS_CADDONE |
                  RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                  RFLR_IRQFLAGS_CADDETECTED );

    // Set radio in continuous reception
    SX1276SetOpMode( RF_OPMODE_RECEIVER );

    for( i = 0; i < 32; i++ )
    {
        mdelay( 1 );
        // Unfiltered RSSI value reading. Only takes the LSB value
        rnd |= ( ( uint32_t )SX1276Read( REG_LR_RSSIWIDEBAND ) & 0x01 ) << i;
    }

    SX1276SetSleep( );

    return rnd;
}

/*!
 * Performs the Rx chain calibration for LF and HF bands
 * \remark Must be called just after the reset so all registers are at their
 *         default values
 */
static void RxChainCalibration( void )
{
    uint8_t regPaConfigInitVal;
    uint32_t initialFreq;
	uint8_t val0 = 0, val1 = 0, val2 = 0

    // Save context
    SX1278_Read_Reg( REG_PACONFIG, &regPaConfigInitVal);

	SX1278_Read_Reg( REG_FRFMSB, &val0);
	SX1278_Read_Reg( REG_FRFMID, &val1);
	SX1278_Read_Reg( REG_FRFLSB, &val2);
    initialFreq = ( double )( ( ( uint32_t )val0 << 16 ) |
                              ( ( uint32_t )val1 << 8 ) |
                              ( ( uint32_t )val2 ) ) * ( double )FREQ_STEP;

    // Cut the PA just in case, RFO output, power = -1 dBm
    SX1278_Write_Reg( REG_PACONFIG, 0x00 );

    // Launch Rx chain calibration for LF band
    SX1278_Read_Reg(REG_IMAGECAL, &val0);
    SX1278_Write_Reg( REG_IMAGECAL, ( val0 & RF_IMAGECAL_IMAGECAL_MASK ) | RF_IMAGECAL_IMAGECAL_START );

	SX1278_Read_Reg(REG_IMAGECAL, &val0);
    while( ( val0 & RF_IMAGECAL_IMAGECAL_RUNNING ) == RF_IMAGECAL_IMAGECAL_RUNNING )
    {
    }

    // Sets a Frequency in HF band
    SX1276SetChannel( 868000000 );

    // Launch Rx chain calibration for HF band
    SX1278_Read_Reg(REG_IMAGECAL, &val0);
    SX1276Write( REG_IMAGECAL, ( val0 & RF_IMAGECAL_IMAGECAL_MASK ) | RF_IMAGECAL_IMAGECAL_START );
	SX1278_Read_Reg(REG_IMAGECAL, &val0);
    while( ( val0 & RF_IMAGECAL_IMAGECAL_RUNNING ) == RF_IMAGECAL_IMAGECAL_RUNNING )
    {
    }

    // Restore context
    SX1278_Write_Reg( REG_PACONFIG, regPaConfigInitVal );
    SX1276SetChannel( initialFreq );
}

/*!
 * Returns the known FSK bandwidth registers value
 *
 * \param [IN] bandwidth Bandwidth value in Hz
 * \retval regValue Bandwidth register value.
 */
static uint8_t GetFskBandwidthRegValue( uint32_t bandwidth )
{
    uint8_t i;

    for( i = 0; i < ( sizeof( FskBandwidths ) / sizeof( FskBandwidth_t ) ) - 1; i++ )
    {
        if( ( bandwidth >= FskBandwidths[i].bandwidth ) && ( bandwidth < FskBandwidths[i + 1].bandwidth ) )
        {
            return FskBandwidths[i].RegValue;
        }
    }
    // ERROR: Value not found
    while( 1 );
}

void SX1276SetRxConfig( RadioModems_en modem, uint32_t bandwidth,
                         uint32_t datarate, uint8_t coderate,
                         uint32_t bandwidthAfc, uint16_t preambleLen,
                         uint16_t symbTimeout, bool fixLen,
                         uint8_t payloadLen,
                         bool crcOn, bool freqHopOn, uint8_t hopPeriod,
                         bool iqInverted, bool rxContinuous )
{
	uint8_t val = 0;

    SX1276SetModem( modem );

    switch( modem )
    {
    case MODEM_FSK:
        {
            sx1278_ptr->sx1278_cfg_ptr.Fsk.Bandwidth = bandwidth;
            sx1278_ptr->sx1278_cfg_ptr.Fsk.Datarate = datarate;
            sx1278_ptr->sx1278_cfg_ptr.Fsk.BandwidthAfc = bandwidthAfc;
            sx1278_ptr->sx1278_cfg_ptr.Fsk.FixLen = fixLen;
            sx1278_ptr->sx1278_cfg_ptr.Fsk.PayloadLen = payloadLen;
            sx1278_ptr->sx1278_cfg_ptr.Fsk.CrcOn = crcOn;
            sx1278_ptr->sx1278_cfg_ptr.Fsk.IqInverted = iqInverted;
            sx1278_ptr->sx1278_cfg_ptr.Fsk.RxContinuous = rxContinuous;
            sx1278_ptr->sx1278_cfg_ptr.Fsk.PreambleLen = preambleLen;
            sx1278_ptr->sx1278_cfg_ptr.Fsk.RxSingleTimeout = ( uint32_t )( symbTimeout * ( ( 1.0 / ( double )datarate ) * 8.0 ) * 1000 );

            datarate = ( uint16_t )( ( double )XTAL_FREQ / ( double )datarate );
            SX1278_Write_Reg( REG_BITRATEMSB, ( uint8_t )( datarate >> 8 ) );
            SX1278_Write_Reg( REG_BITRATELSB, ( uint8_t )( datarate & 0xFF ) );

            SX1278_Write_Reg( REG_RXBW, GetFskBandwidthRegValue( bandwidth ) );
            SX1278_Write_Reg( REG_AFCBW, GetFskBandwidthRegValue( bandwidthAfc ) );

            SX1278_Write_Reg( REG_PREAMBLEMSB, ( uint8_t )( ( preambleLen >> 8 ) & 0xFF ) );
            SX1278_Write_Reg( REG_PREAMBLELSB, ( uint8_t )( preambleLen & 0xFF ) );

            if( fixLen == 1 )
            {
                SX1278_Write_Reg( REG_PAYLOADLENGTH, payloadLen );
            }
            else
            {
                SX1278_Write_Reg( REG_PAYLOADLENGTH, 0xFF ); // Set payload length to the maximum
            }

			SX1278_Write_Reg(REG_PACKETCONFIG1, &val);
            SX1278_Write_Reg( REG_PACKETCONFIG1,
                         ( val &
                           RF_PACKETCONFIG1_CRC_MASK &
                           RF_PACKETCONFIG1_PACKETFORMAT_MASK ) |
                           ( ( fixLen == 1 ) ? RF_PACKETCONFIG1_PACKETFORMAT_FIXED : RF_PACKETCONFIG1_PACKETFORMAT_VARIABLE ) |
                           ( crcOn << 4 ) );
            SX1278_Write_Reg( REG_PACKETCONFIG2, ( SX1276Read( REG_PACKETCONFIG2 ) | RF_PACKETCONFIG2_DATAMODE_PACKET ) );
        }
        break;
    case MODEM_LORA:
        {
            if( bandwidth > 2 )
            {
                // Fatal error: When using LoRa modem only bandwidths 125, 250 and 500 kHz are supported
                while( 1 );
            }
            bandwidth += 7;
            sx1278_ptr->sx1278_cfg_ptr->RxLoRa.Bandwidth = bandwidth;
            sx1278_ptr->sx1278_cfg_ptr->RxLoRa.Datarate = datarate;
            sx1278_ptr->sx1278_cfg_ptr->RxLoRa.Coderate = coderate;
            sx1278_ptr->sx1278_cfg_ptr->RxLoRa.PreambleLen = preambleLen;
            sx1278_ptr->sx1278_cfg_ptr->RxLoRa.FixLen = fixLen;
            sx1278_ptr->sx1278_cfg_ptr->RxLoRa.PayloadLen = payloadLen;
            sx1278_ptr->sx1278_cfg_ptr->RxLoRa.CrcOn = crcOn;
            sx1278_ptr->sx1278_cfg_ptr->RxLoRa.FreqHopOn = freqHopOn;
            sx1278_ptr->sx1278_cfg_ptr->RxLoRa.HopPeriod = hopPeriod;
            sx1278_ptr->sx1278_cfg_ptr->RxLoRa.IqInverted = iqInverted;
            sx1278_ptr->sx1278_cfg_ptr->RxLoRa.RxContinuous = rxContinuous;

            if( datarate > 12 )
            {
                datarate = 12;
            }
            else if( datarate < 6 )
            {
                datarate = 6;
            }

            if( ( ( bandwidth == 7 ) && ( ( datarate == 11 ) || ( datarate == 12 ) ) ) ||
                ( ( bandwidth == 8 ) && ( datarate == 12 ) ) )
            {
                sx1278_ptr->sx1278_cfg_ptr->RxLoRa.LowDatarateOptimize = 0x01;
            }
            else
            {
                sx1278_ptr->sx1278_cfg_ptr->RxLoRa.LowDatarateOptimize = 0x00;
            }

			SX1278_Read_Reg(REG_LR_MODEMCONFIG1, &val);
            SX1278_Write_Reg( REG_LR_MODEMCONFIG1,
                         ( val &
                           RFLR_MODEMCONFIG1_BW_MASK &
                           RFLR_MODEMCONFIG1_CODINGRATE_MASK &
                           RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK ) |
                           ( bandwidth << 4 ) | ( coderate << 1 ) |
                           fixLen );

			SX1278_Read_Reg(REG_LR_MODEMCONFIG2, &val);
            SX1278_Write_Reg( REG_LR_MODEMCONFIG2,
                         ( val &
                           RFLR_MODEMCONFIG2_SF_MASK &
                           RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK &
                           RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK ) |
                           ( datarate << 4 ) | ( crcOn << 2 ) |
                           ( ( symbTimeout >> 8 ) & ~RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK ) );

			SX1278_Read_Reg(REG_LR_MODEMCONFIG3, &val);
            SX1278_Write_Reg( REG_LR_MODEMCONFIG3,
                         ( val &
                           RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_MASK ) |
                           ( sx1278_ptr->sx1278_cfg_ptr->RxLoRa.LowDatarateOptimize << 3 ) );

            SX1278_Write_Reg( REG_LR_SYMBTIMEOUTLSB, ( uint8_t )( symbTimeout & 0xFF ) );

            SX1278_Write_Reg( REG_LR_PREAMBLEMSB, ( uint8_t )( ( preambleLen >> 8 ) & 0xFF ) );
            SX1278_Write_Reg( REG_LR_PREAMBLELSB, ( uint8_t )( preambleLen & 0xFF ) );

            if( fixLen == 1 )
            {
                SX1278_Write_Reg( REG_LR_PAYLOADLENGTH, payloadLen );
            }

            if( sx1278_ptr->sx1278_cfg_ptr->RxLoRa.FreqHopOn == true )
            {
            	SX1278_Read_Reg(REG_LR_PLLHOP, &val);
                SX1278_Write_Reg( REG_LR_PLLHOP, ( val & RFLR_PLLHOP_FASTHOP_MASK ) | RFLR_PLLHOP_FASTHOP_ON );
                SX1278_Write_Reg( REG_LR_HOPPERIOD, sx1278_ptr->sx1278_cfg_ptr->RxLoRa.HopPeriod );
            }

            if( ( bandwidth == 9 ) && ( sx1278_ptr->sx1278_cfg_ptr->Channel > RF_MID_BAND_THRESH ) )
            {
                // ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth
                SX1278_Write_Reg( REG_LR_TEST36, 0x02 );
                SX1278_Write_Reg( REG_LR_TEST3A, 0x64 );
            }
            else if( bandwidth == 9 )
            {
                // ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth
                SX1278_Write_Reg( REG_LR_TEST36, 0x02 );
                SX1278_Write_Reg( REG_LR_TEST3A, 0x7F );
            }
            else
            {
                // ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth
                SX1278_Write_Reg( REG_LR_TEST36, 0x03 );
            }

            if( datarate == 6 )
            {
            	SX1278_Read_Reg(REG_LR_DETECTOPTIMIZE, &val);
                SX1278_Write_Reg( REG_LR_DETECTOPTIMIZE,
                             ( val &
                               RFLR_DETECTIONOPTIMIZE_MASK ) |
                               RFLR_DETECTIONOPTIMIZE_SF6 );
                SX1278_Write_Reg( REG_LR_DETECTIONTHRESHOLD,
                             RFLR_DETECTIONTHRESH_SF6 );
            }
            else
            {
            	SX1278_Read_Reg(REG_LR_DETECTOPTIMIZE, &val);
                SX1278_Write_Reg( REG_LR_DETECTOPTIMIZE,
                             ( val &
                             RFLR_DETECTIONOPTIMIZE_MASK ) |
                             RFLR_DETECTIONOPTIMIZE_SF7_TO_SF12 );
                SX1278_Write_Reg( REG_LR_DETECTIONTHRESHOLD,
                             RFLR_DETECTIONTHRESH_SF7_TO_SF12 );
            }
        }
        break;
    }
}

void SX1276SetTxConfig( RadioModems_t modem, int8_t power, uint32_t fdev,
                        uint32_t bandwidth, uint32_t datarate,
                        uint8_t coderate, uint16_t preambleLen,
                        bool fixLen, bool crcOn, bool freqHopOn,
                        uint8_t hopPeriod, bool iqInverted, uint32_t timeout )
{
	uint8_t val = 0;

    SX1276SetModem( modem );

    SX1276SetRfTxPower( power );

    switch( modem )
    {
    case MODEM_FSK:
        {
            sx1278_ptr->sx1278_cfg_ptr->Fsk.Power = power;
            sx1278_ptr->sx1278_cfg_ptr->Fsk.Fdev = fdev;
            sx1278_ptr->sx1278_cfg_ptr->Fsk.Bandwidth = bandwidth;
            sx1278_ptr->sx1278_cfg_ptr->Fsk.Datarate = datarate;
            sx1278_ptr->sx1278_cfg_ptr->Fsk.PreambleLen = preambleLen;
            sx1278_ptr->sx1278_cfg_ptr->Fsk.FixLen = fixLen;
            sx1278_ptr->sx1278_cfg_ptr->Fsk.CrcOn = crcOn;
            sx1278_ptr->sx1278_cfg_ptr->Fsk.IqInverted = iqInverted;
            sx1278_ptr->sx1278_cfg_ptr->Fsk.TxTimeout = timeout;

            fdev = ( uint16_t )( ( double )fdev / ( double )FREQ_STEP );
            SX1278_Write_Reg( REG_FDEVMSB, ( uint8_t )( fdev >> 8 ) );
            SX1278_Write_Reg( REG_FDEVLSB, ( uint8_t )( fdev & 0xFF ) );

            datarate = ( uint16_t )( ( double )XTAL_FREQ / ( double )datarate );
            SX1278_Write_Reg( REG_BITRATEMSB, ( uint8_t )( datarate >> 8 ) );
            SX1278_Write_Reg( REG_BITRATELSB, ( uint8_t )( datarate & 0xFF ) );

            SX1278_Write_Reg( REG_PREAMBLEMSB, ( preambleLen >> 8 ) & 0x00FF );
            SX1278_Write_Reg( REG_PREAMBLELSB, preambleLen & 0xFF );

			SX1278_Read_Reg(REG_PACKETCONFIG1, &val);
            SX1278_Write_Reg( REG_PACKETCONFIG1,
                         ( val &
                           RF_PACKETCONFIG1_CRC_MASK &
                           RF_PACKETCONFIG1_PACKETFORMAT_MASK ) |
                           ( ( fixLen == 1 ) ? RF_PACKETCONFIG1_PACKETFORMAT_FIXED : RF_PACKETCONFIG1_PACKETFORMAT_VARIABLE ) |
                           ( crcOn << 4 ) );
            SX1278_Write_Reg( REG_PACKETCONFIG2, ( SX1276Read( REG_PACKETCONFIG2 ) | RF_PACKETCONFIG2_DATAMODE_PACKET ) );
        }
        break;
    case MODEM_LORA:
        {
            sx1278_ptr->sx1278_cfg_ptr->TxLoRa.Power = power;
            if( bandwidth > 2 )
            {
                // Fatal error: When using LoRa modem only bandwidths 125, 250 and 500 kHz are supported
                while( 1 );
            }
            bandwidth += 7;
            sx1278_ptr->sx1278_cfg_ptr->TxLoRa.Bandwidth = bandwidth;
            sx1278_ptr->sx1278_cfg_ptr->TxLoRa.Datarate = datarate;
            sx1278_ptr->sx1278_cfg_ptr->TxLoRa.Coderate = coderate;
            sx1278_ptr->sx1278_cfg_ptr->TxLoRa.PreambleLen = preambleLen;
            sx1278_ptr->sx1278_cfg_ptr->TxLoRa.FixLen = fixLen;
            sx1278_ptr->sx1278_cfg_ptr->TxLoRa.FreqHopOn = freqHopOn;
            sx1278_ptr->sx1278_cfg_ptr->TxLoRa.HopPeriod = hopPeriod;
            sx1278_ptr->sx1278_cfg_ptr->TxLoRa.CrcOn = crcOn;
            sx1278_ptr->sx1278_cfg_ptr->TxLoRa.IqInverted = iqInverted;
            sx1278_ptr->sx1278_cfg_ptr->TxLoRa.TxTimeout = timeout;

            if( datarate > 12 )
            {
                datarate = 12;
            }
            else if( datarate < 6 )
            {
                datarate = 6;
            }
            if( ( ( bandwidth == 7 ) && ( ( datarate == 11 ) || ( datarate == 12 ) ) ) ||
                ( ( bandwidth == 8 ) && ( datarate == 12 ) ) )
            {
                sx1278_ptr->sx1278_cfg_ptr->TxLoRa.LowDatarateOptimize = 0x01;
            }
            else
            {
                sx1278_ptr->sx1278_cfg_ptr->TxLoRa.LowDatarateOptimize = 0x00;
            }

            if( sx1278_ptr->sx1278_cfg_ptr->TxLoRa.FreqHopOn == true )
            {
            	SX1278_Read_Reg( REG_LR_PLLHOP, &val);
                SX1278_Write_Reg( REG_LR_PLLHOP, ( val & RFLR_PLLHOP_FASTHOP_MASK ) | RFLR_PLLHOP_FASTHOP_ON );
                SX1278_Write_Reg( REG_LR_HOPPERIOD, sx1278_ptr->sx1278_cfg_ptr->TxLoRa.HopPeriod );
            }

			SX1278_Read_Reg( REG_LR_MODEMCONFIG1, &val);
            SX1278_Write_Reg( REG_LR_MODEMCONFIG1,
                         ( val &
                           RFLR_MODEMCONFIG1_BW_MASK &
                           RFLR_MODEMCONFIG1_CODINGRATE_MASK &
                           RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK ) |
                           ( bandwidth << 4 ) | ( coderate << 1 ) |
                           fixLen );

			SX1278_Read_Reg( REG_LR_MODEMCONFIG2, &val);
            SX1278_Write_Reg( REG_LR_MODEMCONFIG2,
                         ( val &
                           RFLR_MODEMCONFIG2_SF_MASK &
                           RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK ) |
                           ( datarate << 4 ) | ( crcOn << 2 ) );

			SX1278_Read_Reg( REG_LR_MODEMCONFIG3, &val);
            SX1278_Write_Reg( REG_LR_MODEMCONFIG3,
                         ( val &
                           RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_MASK ) |
                           ( sx1278_ptr->sx1278_cfg_ptr->TxLoRa.LowDatarateOptimize << 3 ) );

            SX1278_Write_Reg( REG_LR_PREAMBLEMSB, ( preambleLen >> 8 ) & 0x00FF );
            SX1278_Write_Reg( REG_LR_PREAMBLELSB, preambleLen & 0xFF );

            if( datarate == 6 )
            {
            	SX1278_Read_Reg( REG_LR_DETECTOPTIMIZE, &val);
                SX1278_Write_Reg( REG_LR_DETECTOPTIMIZE,
                             ( val &
                               RFLR_DETECTIONOPTIMIZE_MASK ) |
                               RFLR_DETECTIONOPTIMIZE_SF6 );
                SX1278_Write_Reg( REG_LR_DETECTIONTHRESHOLD,
                             RFLR_DETECTIONTHRESH_SF6 );
            }
            else
            {
            	SX1278_Read_Reg( REG_LR_DETECTOPTIMIZE, &val);
                SX1278_Write_Reg( REG_LR_DETECTOPTIMIZE,
                             ( val &
                             RFLR_DETECTIONOPTIMIZE_MASK ) |
                             RFLR_DETECTIONOPTIMIZE_SF7_TO_SF12 );
                SX1278_Write_Reg( REG_LR_DETECTIONTHRESHOLD,
                             RFLR_DETECTIONTHRESH_SF7_TO_SF12 );
            }
        }
        break;
    }
}

uint32_t SX1276GetTimeOnAir( RadioModems_t modem, uint8_t pktLen )
{
	uint8_t val0 = 0, val1 = 0;
    uint32_t airTime = 0;

    switch( modem )
    {
    case MODEM_FSK:
        {
			SX1278_Read_Reg( REG_SYNCCONFIG, &val0);
			SX1278_Read_Reg( REG_PACKETCONFIG1, &val1);
            airTime = round( ( 8 * ( sx1278_ptr->sx1278_cfg_ptr->Fsk.PreambleLen +
                                     ( ( val0 & ~RF_SYNCCONFIG_SYNCSIZE_MASK ) + 1 ) +
                                     ( ( sx1278_ptr->sx1278_cfg_ptr->Fsk.FixLen == 0x01 ) ? 0.0 : 1.0 ) +
                                     ( ( ( val1 & ~RF_PACKETCONFIG1_ADDRSFILTERING_MASK ) != 0x00 ) ? 1.0 : 0 ) +
                                     pktLen +
                                     ( ( sx1278_ptr->sx1278_cfg_ptr->Fsk.CrcOn == 0x01 ) ? 2.0 : 0 ) ) /
                                     sx1278_ptr->sx1278_cfg_ptr->Fsk.Datarate ) * 1000 );
        }
        break;
    case MODEM_LORA:
        {
            double bw = 0.0;
            // REMARK: When using LoRa modem only bandwidths 125, 250 and 500 kHz are supported
            switch( sx1278_ptr->sx1278_cfg_ptr->TxLoRa.Bandwidth )
            {
            //case 0: // 7.8 kHz
            //    bw = 7800;
            //    break;
            //case 1: // 10.4 kHz
            //    bw = 10400;
            //    break;
            //case 2: // 15.6 kHz
            //    bw = 15600;
            //    break;
            //case 3: // 20.8 kHz
            //    bw = 20800;
            //    break;
            //case 4: // 31.2 kHz
            //    bw = 31200;
            //    break;
            //case 5: // 41.4 kHz
            //    bw = 41400;
            //    break;
            //case 6: // 62.5 kHz
            //    bw = 62500;
            //    break;
            case 7: // 125 kHz
                bw = 125000;
                break;
            case 8: // 250 kHz
                bw = 250000;
                break;
            case 9: // 500 kHz
                bw = 500000;
                break;
            }

            // Symbol rate : time for one symbol (secs)
            double rs = bw / ( 1 << sx1278_ptr->sx1278_cfg_ptr->TxLoRa.Datarate );
            double ts = 1 / rs;
            // time of preamble
            double tPreamble = ( sx1278_ptr->sx1278_cfg_ptr->TxLoRa.PreambleLen + 4.25 ) * ts;
            // Symbol length of payload and time
            double tmp = ceil( ( 8 * pktLen - 4 * sx1278_ptr->sx1278_cfg_ptr->TxLoRa.Datarate +
                                 28 + 16 * sx1278_ptr->sx1278_cfg_ptr->TxLoRa.CrcOn -
                                 ( sx1278_ptr->sx1278_cfg_ptr->TxLoRa.FixLen ? 20 : 0 ) ) /
                                 ( double )( 4 * ( sx1278_ptr->sx1278_cfg_ptr->TxLoRa.Datarate -
                                 ( ( sx1278_ptr->sx1278_cfg_ptr->TxLoRa.LowDatarateOptimize > 0 ) ? 2 : 0 ) ) ) ) *
                                 ( sx1278_ptr->sx1278_cfg_ptr->TxLoRa.Coderate + 4 );
            double nPayload = 8 + ( ( tmp > 0 ) ? tmp : 0 );
            double tPayload = nPayload * ts;
            // Time on air
            double tOnAir = tPreamble + tPayload;
            // return ms secs
            airTime = floor( tOnAir * 1000 + 0.999 );
        }
        break;
    }
    return airTime;
}

void SX1276Send( uint8_t *buffer, uint8_t size )
{
	uint8_t val = 0;
    uint32_t txTimeout = 0;

    switch( sx1278_ptr->sx1278_cfg_ptr->Modem )
    {
    case MODEM_FSK:
        {
            sx1278_ptr->sx1278_cfg_ptr->FskPacketHandler.NbBytes = 0;
            sx1278_ptr->sx1278_cfg_ptr->FskPacketHandler.Size = size;

            if( sx1278_ptr->sx1278_cfg_ptr->Fsk.FixLen == false )
            {
                SX1278_Write_FIFO( ( uint8_t* )&size, 1 );
            }
            else
            {
                SX1278_Write_FIFO( REG_PAYLOADLENGTH, size );
            }

            if( ( size > 0 ) && ( size <= 64 ) )
            {
                sx1278_ptr->sx1278_cfg_ptr->FskPacketHandler.ChunkSize = size;
            }
            else
            {
                memcpy1( RxTxBuffer, buffer, size );
                sx1278_ptr->sx1278_cfg_ptr->FskPacketHandler.ChunkSize = 32;
            }

            // Write payload buffer
            SX1278_Write_FIFO( buffer, sx1278_ptr->sx1278_cfg_ptr->FskPacketHandler.ChunkSize );
            sx1278_ptr->sx1278_cfg_ptr->FskPacketHandler.NbBytes += sx1278_ptr->sx1278_cfg_ptr->FskPacketHandler.ChunkSize;
            txTimeout = sx1278_ptr->sx1278_cfg_ptr->Fsk.TxTimeout;
        }
        break;
    case MODEM_LORA:
        {
            if( sx1278_ptr->sx1278_cfg_ptr->TxLoRa.IqInverted == true )
            {
            	SX1278_Write_Reg( REG_LR_INVERTIQ, &val);
                SX1278_Write_Reg( REG_LR_INVERTIQ, ( ( val & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_ON ) );
                SX1278_Write_Reg( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_ON );
            }
            else
            {
            	SX1278_Write_Reg( REG_LR_INVERTIQ, &val);
                SX1278_Write_Reg( REG_LR_INVERTIQ, ( ( val & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_OFF ) );
                SX1278_Write_Reg( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_OFF );
            }

            sx1278_ptr->sx1278_cfg_ptr->LoRaPacketHandler.Size = size;

            // Initializes the payload size
            SX1278_Write_Reg( REG_LR_PAYLOADLENGTH, size );

            // Full buffer used for Tx
            SX1278_Write_Reg( REG_LR_FIFOTXBASEADDR, 0 );
            SX1278_Write_Reg( REG_LR_FIFOADDRPTR, 0 );

            // FIFO operations can not take place in Sleep mode
            SX1278_Write_Reg( REG_OPMODE, &val);
            if( ( val & ~RF_OPMODE_MASK ) == RF_OPMODE_SLEEP )
            {
                SX1276SetStby( );
                mdelay( 1 );
            }
            // Write payload buffer
            SX1278_Write_FIFO( buffer, size );
            txTimeout = sx1278_ptr->sx1278_cfg_ptr->TxLoRa.TxTimeout;
        }
        break;
    }

    SX1276SetTx( txTimeout );
}

void SX1276SetSleep( void )
{
    TimerStop( &RxTimeoutTimer );
    TimerStop( &TxTimeoutTimer );

    SX1276SetOpMode( RF_OPMODE_SLEEP );
    sx1278_ptr->sx1278_cfg_ptr->State = RF_IDLE;
}

void SX1276SetStby( void )
{
    TimerStop( &RxTimeoutTimer );
    TimerStop( &TxTimeoutTimer );

    SX1276SetOpMode( RF_OPMODE_STANDBY );
    sx1278_ptr->sx1278_cfg_ptr->State = RF_IDLE;
}

void SX1276SetRx( uint32_t timeout )
{
	uint8_t val = 0;
    bool rxContinuous = false;

    switch( sx1278_ptr->sx1278_cfg_ptr->Modem )
    {
    case MODEM_FSK:
        {
            rxContinuous = sx1278_ptr->sx1278_cfg_ptr->Fsk.RxContinuous;

            // DIO0=PayloadReady
            // DIO1=FifoLevel
            // DIO2=SyncAddr
            // DIO3=FifoEmpty
            // DIO4=Preamble
            // DIO5=ModeReady
            SX1278_Read_Reg( REG_DIOMAPPING1, &val);
            SX1278_Write_Reg( REG_DIOMAPPING1, ( val & RF_DIOMAPPING1_DIO0_MASK &
                                                                            RF_DIOMAPPING1_DIO1_MASK &
                                                                            RF_DIOMAPPING1_DIO2_MASK ) |
                                                                            RF_DIOMAPPING1_DIO0_00 |
                                                                            RF_DIOMAPPING1_DIO1_00 |
                                                                            RF_DIOMAPPING1_DIO2_11 );

			SX1278_Read_Reg( REG_DIOMAPPING2, &val);
            SX1278_Write_Reg( REG_DIOMAPPING2, ( val & RF_DIOMAPPING2_DIO4_MASK &
                                                                            RF_DIOMAPPING2_MAP_MASK ) |
                                                                            RF_DIOMAPPING2_DIO4_11 |
                                                                            RF_DIOMAPPING2_MAP_PREAMBLEDETECT );

			SX1278_Read_Reg( REG_FIFOTHRESH, &val);
            sx1278_ptr->sx1278_cfg_ptr->FskPacketHandler.FifoThresh = val & 0x3F;

            SX1278_Write_Reg( REG_RXCONFIG, RF_RXCONFIG_AFCAUTO_ON | RF_RXCONFIG_AGCAUTO_ON | RF_RXCONFIG_RXTRIGER_PREAMBLEDETECT );

            sx1278_ptr->sx1278_cfg_ptr->FskPacketHandler.PreambleDetected = false;
            sx1278_ptr->sx1278_cfg_ptr->FskPacketHandler.SyncWordDetected = false;
            sx1278_ptr->sx1278_cfg_ptr->FskPacketHandler.NbBytes = 0;
            sx1278_ptr->sx1278_cfg_ptr->FskPacketHandler.Size = 0;
        }
        break;
    case MODEM_LORA:
        {
            if( sx1278_ptr->sx1278_cfg_ptr->RxLoRa.IqInverted == true )
            {
            	SX1278_Read_Reg( REG_LR_INVERTIQ, &val);
                SX1278_Write_Reg( REG_LR_INVERTIQ, ( ( val & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_ON | RFLR_INVERTIQ_TX_OFF ) );
                SX1278_Write_Reg( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_ON );
            }
            else
            {
            	SX1278_Read_Reg( REG_LR_INVERTIQ, &val);
                SX1278_Write_Reg( REG_LR_INVERTIQ, ( ( val & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_OFF ) );
                SX1278_Write_Reg( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_OFF );
            }

            // ERRATA 2.3 - Receiver Spurious Reception of a LoRa Signal
            if( sx1278_ptr->sx1278_cfg_ptr->RxLoRa.Bandwidth < 9 )
            {
                SX1278_Write_Reg( REG_LR_DETECTOPTIMIZE, SX1276Read( REG_LR_DETECTOPTIMIZE ) & 0x7F );
                SX1278_Write_Reg( REG_LR_TEST30, 0x00 );
                switch( sx1278_ptr->sx1278_cfg_ptr->RxLoRa.Bandwidth )
                {
                case 0: // 7.8 kHz
                    SX1278_Write_Reg( REG_LR_TEST2F, 0x48 );
                    SX1276SetChannel(sx1278_ptr->sx1278_cfg_ptr->Channel + 7810 );
                    break;
                case 1: // 10.4 kHz
                    SX1278_Write_Reg( REG_LR_TEST2F, 0x44 );
                    SX1276SetChannel(sx1278_ptr->sx1278_cfg_ptr->Channel + 10420 );
                    break;
                case 2: // 15.6 kHz
                    SX1278_Write_Reg( REG_LR_TEST2F, 0x44 );
                    SX1276SetChannel(sx1278_ptr->sx1278_cfg_ptr->Channel + 15620 );
                    break;
                case 3: // 20.8 kHz
                    SX1278_Write_Reg( REG_LR_TEST2F, 0x44 );
                    SX1276SetChannel(sx1278_ptr->sx1278_cfg_ptr->Channel + 20830 );
                    break;
                case 4: // 31.2 kHz
                    SX1278_Write_Reg( REG_LR_TEST2F, 0x44 );
                    SX1276SetChannel(sx1278_ptr->sx1278_cfg_ptr->Channel + 31250 );
                    break;
                case 5: // 41.4 kHz
                    SX1278_Write_Reg( REG_LR_TEST2F, 0x44 );
                    SX1276SetChannel(sx1278_ptr->sx1278_cfg_ptr->Channel + 41670 );
                    break;
                case 6: // 62.5 kHz
                    SX1278_Write_Reg( REG_LR_TEST2F, 0x40 );
                    break;
                case 7: // 125 kHz
                    SX1278_Write_Reg( REG_LR_TEST2F, 0x40 );
                    break;
                case 8: // 250 kHz
                    SX1278_Write_Reg( REG_LR_TEST2F, 0x40 );
                    break;
                }
            }
            else
            {
            	SX1278_Read_Reg( REG_LR_DETECTOPTIMIZE, &val);
                SX1278_Write_Reg( REG_LR_DETECTOPTIMIZE, val | 0x80 );
            }

            rxContinuous = sx1278_ptr->sx1278_cfg_ptr->RxLoRa.RxContinuous;

            if( sx1278_ptr->sx1278_cfg_ptr->RxLoRa.FreqHopOn == true )
            {
                SX1278_Write_Reg( REG_LR_IRQFLAGSMASK, //RFLR_IRQFLAGS_RXTIMEOUT |
                                                  //RFLR_IRQFLAGS_RXDONE |
                                                  //RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                                  RFLR_IRQFLAGS_VALIDHEADER |
                                                  RFLR_IRQFLAGS_TXDONE |
                                                  RFLR_IRQFLAGS_CADDONE |
                                                  //RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                                  RFLR_IRQFLAGS_CADDETECTED );

                // DIO0=RxDone, DIO2=FhssChangeChannel
                SX1278_Write_Reg( REG_DIOMAPPING1, &val);
                SX1278_Write_Reg( REG_DIOMAPPING1, ( val & RFLR_DIOMAPPING1_DIO0_MASK & RFLR_DIOMAPPING1_DIO2_MASK  ) | RFLR_DIOMAPPING1_DIO0_00 | RFLR_DIOMAPPING1_DIO2_00 );
            }
            else
            {
                SX1278_Write_Reg( REG_LR_IRQFLAGSMASK, //RFLR_IRQFLAGS_RXTIMEOUT |
                                                  //RFLR_IRQFLAGS_RXDONE |
                                                  //RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                                  RFLR_IRQFLAGS_VALIDHEADER |
                                                  RFLR_IRQFLAGS_TXDONE |
                                                  RFLR_IRQFLAGS_CADDONE |
                                                  RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                                  RFLR_IRQFLAGS_CADDETECTED );

                // DIO0=RxDone
                SX1278_Write_Reg( REG_DIOMAPPING1, &val);
                SX1278_Write_Reg( REG_DIOMAPPING1, ( val & RFLR_DIOMAPPING1_DIO0_MASK ) | RFLR_DIOMAPPING1_DIO0_00 );
            }
            SX1278_Write_Reg( REG_LR_FIFORXBASEADDR, 0 );
            SX1278_Write_Reg( REG_LR_FIFOADDRPTR, 0 );
        }
        break;
    }

    memset( RxTxBuffer, 0, ( size_t )RX_BUFFER_SIZE );

    sx1278_ptr->sx1278_cfg_ptr->State = RF_RX_RUNNING;
    if( timeout != 0 )
    {
        TimerSetValue( &RxTimeoutTimer, timeout );
        TimerStart( &RxTimeoutTimer );
    }

    if( sx1278_ptr->sx1278_cfg_ptr->Modem == MODEM_FSK )
    {
        SX1276SetOpMode( RF_OPMODE_RECEIVER );

        if( rxContinuous == false )
        {
            TimerSetValue( &RxTimeoutSyncWord, SX1276.Settings.Fsk.RxSingleTimeout );
            TimerStart( &RxTimeoutSyncWord );
        }
    }
    else
    {
        if( rxContinuous == true )
        {
            SX1276SetOpMode( RFLR_OPMODE_RECEIVER );
        }
        else
        {
            SX1276SetOpMode( RFLR_OPMODE_RECEIVER_SINGLE );
        }
    }
}

void SX1276SetTx( uint32_t timeout )
{
	uint8_t val = 0;
    TimerSetValue( &TxTimeoutTimer, timeout );

    switch( sx1278_ptr->sx1278_cfg_ptr->Modem )
    {
    case MODEM_FSK:
        {
            // DIO0=PacketSent
            // DIO1=FifoEmpty
            // DIO2=FifoFull
            // DIO3=FifoEmpty
            // DIO4=LowBat
            // DIO5=ModeReady
            SX1278_Read_Reg( REG_DIOMAPPING1, &val);
            SX1278_Write_Reg( REG_DIOMAPPING1, ( val & RF_DIOMAPPING1_DIO0_MASK &
                                                                            RF_DIOMAPPING1_DIO1_MASK &
                                                                            RF_DIOMAPPING1_DIO2_MASK ) |
                                                                            RF_DIOMAPPING1_DIO1_01 );
			SX1278_Read_Reg( REG_DIOMAPPING2, &val);
            SX1278_Write_Reg( REG_DIOMAPPING2, ( val & RF_DIOMAPPING2_DIO4_MASK &
                                                                            RF_DIOMAPPING2_MAP_MASK ) );
			SX1278_Read_Reg( REG_FIFOTHRESH, &val);
            sx1278_ptr->sx1278_cfg_ptr->FskPacketHandler.FifoThresh = val & 0x3F;
        }
        break;
    case MODEM_LORA:
        {
            if( sx1278_ptr->sx1278_cfg_ptr->TxLoRa.FreqHopOn == true )
            {
                SX1278_Write_Reg( REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
                                                  RFLR_IRQFLAGS_RXDONE |
                                                  RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                                  RFLR_IRQFLAGS_VALIDHEADER |
                                                  //RFLR_IRQFLAGS_TXDONE |
                                                  RFLR_IRQFLAGS_CADDONE |
                                                  //RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                                  RFLR_IRQFLAGS_CADDETECTED );

                // DIO0=TxDone, DIO2=FhssChangeChannel
                SX1278_Read_Reg( REG_DIOMAPPING1, &val);
                SX1278_Write_Reg( REG_DIOMAPPING1, ( val & RFLR_DIOMAPPING1_DIO0_MASK & RFLR_DIOMAPPING1_DIO2_MASK ) | RFLR_DIOMAPPING1_DIO0_01 | RFLR_DIOMAPPING1_DIO2_00 );
            }
            else
            {
                SX1278_Write_Reg( REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
                                                  RFLR_IRQFLAGS_RXDONE |
                                                  RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                                  RFLR_IRQFLAGS_VALIDHEADER |
                                                  //RFLR_IRQFLAGS_TXDONE |
                                                  RFLR_IRQFLAGS_CADDONE |
                                                  RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                                  RFLR_IRQFLAGS_CADDETECTED );

                // DIO0=TxDone
                SX1278_Read_Reg( REG_DIOMAPPING1, &val);
                SX1278_Write_Reg( REG_DIOMAPPING1, ( val & RFLR_DIOMAPPING1_DIO0_MASK ) | RFLR_DIOMAPPING1_DIO0_01 );
            }
        }
        break;
    }

    sx1278_ptr->sx1278_cfg_ptr->State = RF_TX_RUNNING;
    TimerStart( &TxTimeoutTimer );
    SX1276SetOpMode( RF_OPMODE_TRANSMITTER );
}

void SX1276StartCad( void )
{
    switch( SX1276.Settings.Modem )
    {
    case MODEM_FSK:
        {

        }
        break;
    case MODEM_LORA:
        {
            SX1278_Write_Reg( REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
                                        RFLR_IRQFLAGS_RXDONE |
                                        RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                        RFLR_IRQFLAGS_VALIDHEADER |
                                        RFLR_IRQFLAGS_TXDONE |
                                        //RFLR_IRQFLAGS_CADDONE |
                                        RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL // |
                                        //RFLR_IRQFLAGS_CADDETECTED
                                        );

            // DIO3=CADDone
            SX1278_Read_Reg( REG_DIOMAPPING1, &val);
            SX1278_Write_Reg( REG_DIOMAPPING1, ( val & RFLR_DIOMAPPING1_DIO3_MASK ) | RFLR_DIOMAPPING1_DIO3_00 );

            sx1278_ptr->sx1278_cfg_ptr->State = RF_CAD;
            SX1276SetOpMode( RFLR_OPMODE_CAD );
        }
        break;
    default:
        break;
    }
}

void SX1276SetTxContinuousWave( uint32_t freq, int8_t power, uint16_t time )
{
	uint8_t val = 0;
    uint32_t timeout = ( uint32_t )( time * 1000 );

    SX1276SetChannel( freq );

    SX1276SetTxConfig( MODEM_FSK, power, 0, 0, 4800, 0, 5, false, false, 0, 0, 0, timeout );

	SX1278_Read_Reg( REG_PACKETCONFIG2, &val);
    SX1278_Write_Reg( REG_PACKETCONFIG2, ( val & RF_PACKETCONFIG2_DATAMODE_MASK ) );
    // Disable radio interrupts
    SX1278_Write_Reg( REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_11 | RF_DIOMAPPING1_DIO1_11 );
    SX1278_Write_Reg( REG_DIOMAPPING2, RF_DIOMAPPING2_DIO4_10 | RF_DIOMAPPING2_DIO5_10 );

    TimerSetValue( &TxTimeoutTimer, timeout );

    sx1278_ptr->sx1278_cfg_ptr->State = RF_TX_RUNNING;
    TimerStart( &TxTimeoutTimer );
    SX1276SetOpMode( RF_OPMODE_TRANSMITTER );
}

int16_t SX1276ReadRssi( RadioModems_t modem )
{
    int16_t rssi = 0;

    switch( modem )
    {
    case MODEM_FSK:
		SX1278_Read_Reg( REG_RSSIVALUE, &val );
        rssi = -( val >> 1 );
        break;
    case MODEM_LORA:
        if( sx1278_ptr->sx1278_cfg_ptr->Channel > RF_MID_BAND_THRESH )
        {
        	SX1278_Read_Reg( REG_LR_RSSIVALUE, &val);
            rssi = RSSI_OFFSET_HF + val;
        }
        else
        {
        	SX1278_Read_Reg( REG_LR_RSSIVALUE, &val);
            rssi = RSSI_OFFSET_LF + val;
        }
        break;
    default:
        rssi = -1;
        break;
    }
    return rssi;
}

void SX1276SetOpMode( uint8_t opMode )
{
	uint8_t val = 0;

    if( opMode == RF_OPMODE_SLEEP )
    {
        SX1276SetAntSwLowPower( true );
    }
    else
    {
        SX1276SetAntSwLowPower( false );
        SX1276SetAntSw( opMode );
    }

	SX1278_Read_Reg( REG_OPMODE, &val);
    SX1278_Write_Reg( REG_OPMODE, ( val & RF_OPMODE_MASK ) | opMode );
}

void SX1276SetModem( RadioModems_t modem )
{
	uint8_t val = 0;

	SX1276Read( REG_OPMODE, &val);
    if( ( val & RFLR_OPMODE_LONGRANGEMODE_ON ) != 0 )
    {
        sx1278_ptr->sx1278_cfg_ptr->Modem = MODEM_LORA;
    }
    else
    {
        sx1278_ptr->sx1278_cfg_ptr->Modem = MODEM_FSK;
    }

    if( sx1278_ptr->sx1278_cfg_ptr->Modem == modem )
    {
        return;
    }

    sx1278_ptr->sx1278_cfg_ptr->Modem = modem;
    switch( sx1278_ptr->sx1278_cfg_ptr->Modem )
    {
    default:
    case MODEM_FSK:
        SX1276SetSleep( );
		SX1278_Read_Reg( REG_OPMODE, &val);
        SX1278_Write_Reg( REG_OPMODE, ( val & RFLR_OPMODE_LONGRANGEMODE_MASK ) | RFLR_OPMODE_LONGRANGEMODE_OFF );

        SX1278_Write_Reg( REG_DIOMAPPING1, 0x00 );
        SX1278_Write_Reg( REG_DIOMAPPING2, 0x30 ); // DIO5=ModeReady
        break;
    case MODEM_LORA:
        SX1276SetSleep( );
		SX1278_Read_Reg( REG_OPMODE, &val);
        SX1278_Write_Reg( REG_OPMODE, ( val & RFLR_OPMODE_LONGRANGEMODE_MASK ) | RFLR_OPMODE_LONGRANGEMODE_ON );

        SX1278_Write_Reg( REG_DIOMAPPING1, 0x00 );
        SX1278_Write_Reg( REG_DIOMAPPING2, 0x00 );
        break;
    }
}

void SX1276Write( uint16_t addr, uint8_t data )
{
    SX1276WriteBuffer( addr, &data, 1 );
}

uint8_t SX1276Read( uint16_t addr )
{
    uint8_t data;
    SX1276ReadBuffer( addr, &data, 1 );
    return data;
}

void SX1276WriteBuffer( uint16_t addr, uint8_t *buffer, uint8_t size )
{
    uint8_t i;

    //NSS = 0;
    GpioWrite( &SX1276.Spi.Nss, 0 );

    SpiInOut( &SX1276.Spi, addr | 0x80 );
    for( i = 0; i < size; i++ )
    {
        SpiInOut( &SX1276.Spi, buffer[i] );
    }

    //NSS = 1;
    GpioWrite( &SX1276.Spi.Nss, 1 );
}

void SX1276ReadBuffer( uint16_t addr, uint8_t *buffer, uint8_t size )
{
    uint8_t i;

    //NSS = 0;
    GpioWrite( &SX1276.Spi.Nss, 0 );

    SpiInOut( &SX1276.Spi, addr & 0x7F );

    for( i = 0; i < size; i++ )
    {
        buffer[i] = SpiInOut( &SX1276.Spi, 0 );
    }

    //NSS = 1;
    GpioWrite( &SX1276.Spi.Nss, 1 );
}

void SX1276WriteFifo( uint8_t *buffer, uint8_t size )
{
    SX1276WriteBuffer( 0, buffer, size );
}

void SX1276ReadFifo( uint8_t *buffer, uint8_t size )
{
    SX1276ReadBuffer( 0, buffer, size );
}

void SX1276SetMaxPayloadLength( RadioModems_t modem, uint8_t max )
{
    SX1276SetModem( modem );

    switch( modem )
    {
    case MODEM_FSK:
        if( sx1278_ptr->sx1278_cfg_ptr->Fsk.FixLen == false )
        {
            SX1278_Write_Reg( REG_PAYLOADLENGTH, max );
        }
        break;
    case MODEM_LORA:
        SX1278_Write_Reg( REG_LR_PAYLOADMAXLENGTH, max );
        break;
    }
}

void SX1276SetPublicNetwork( bool enable )
{
    SX1276SetModem( MODEM_LORA );
    sx1278_ptr->sx1278_cfg_ptr->TxLoRa.PublicNetwork = enable;
    if( enable == true )
    {
        // Change LoRa modem SyncWord
        SX1278_Write_Reg( REG_LR_SYNCWORD, LORA_MAC_PUBLIC_SYNCWORD );
    }
    else
    {
        // Change LoRa modem SyncWord
        SX1278_Write_Reg( REG_LR_SYNCWORD, LORA_MAC_PRIVATE_SYNCWORD );
    }
}

uint32_t SX1276GetWakeupTime( void )
{
    return SX1276GetBoardTcxoWakeupTime( ) + RADIO_WAKEUP_TIME;
}

void SX1276OnTimeoutIrq( void )
{
	uint8_t val = 0;
    switch( sx1278_ptr->sx1278_cfg_ptr->State )
    {
    case RF_RX_RUNNING:
        if( sx1278_ptr->sx1278_cfg_ptr->Modem == MODEM_FSK )
        {
            sx1278_ptr->sx1278_cfg_ptr->FskPacketHandler.PreambleDetected = false;
            sx1278_ptr->sx1278_cfg_ptr->FskPacketHandler.SyncWordDetected = false;
            sx1278_ptr->sx1278_cfg_ptr->FskPacketHandler.NbBytes = 0;
            sx1278_ptr->sx1278_cfg_ptr->FskPacketHandler.Size = 0;

            // Clear Irqs
            SX1278_Write_Reg( REG_IRQFLAGS1, RF_IRQFLAGS1_RSSI |
                                        RF_IRQFLAGS1_PREAMBLEDETECT |
                                        RF_IRQFLAGS1_SYNCADDRESSMATCH );
            SX1278_Write_Reg( REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN );

            if( sx1278_ptr->sx1278_cfg_ptr->Fsk.RxContinuous == true )
            {
                // Continuous mode restart Rx chain
                SX1278_Read_Reg( REG_RXCONFIG, &val);
                SX1278_Write_Reg( REG_RXCONFIG, val | RF_RXCONFIG_RESTARTRXWITHOUTPLLLOCK );
                TimerStart( &RxTimeoutSyncWord );
            }
            else
            {
                sx1278_ptr->sx1278_cfg_ptr->State = RF_IDLE;
                TimerStop( &RxTimeoutSyncWord );
            }
        }
        if( ( RadioEvents != NULL ) && ( RadioEvents->RxTimeout != NULL ) )
        {
            RadioEvents->RxTimeout( );
        }
        break;
    case RF_TX_RUNNING:
        // Tx timeout shouldn't happen.
        // But it has been observed that when it happens it is a result of a corrupted SPI transfer
        // it depends on the platform design.
        //
        // The workaround is to put the radio in a known state. Thus, we re-initialize it.

        // BEGIN WORKAROUND

        // Reset the radio
        SX1276Reset( );

        // Calibrate Rx chain
        RxChainCalibration( );

        // Initialize radio default values
        SX1276SetOpMode( RF_OPMODE_SLEEP );

        for( uint8_t i = 0; i < sizeof( RadioRegsInit ) / sizeof( RadioRegisters_t ); i++ )
        {
            SX1276SetModem( RadioRegsInit[i].Modem );
            SX1278_Write_Reg( RadioRegsInit[i].Addr, RadioRegsInit[i].Value );
        }
        SX1276SetModem( MODEM_FSK );

        // Restore previous network type setting.
        SX1276SetPublicNetwork( sx1278_ptr->sx1278_cfg_ptr->TxLoRa.PublicNetwork );
        // END WORKAROUND

        sx1278_ptr->sx1278_cfg_ptr->State = RF_IDLE;
        if( ( RadioEvents != NULL ) && ( RadioEvents->TxTimeout != NULL ) )
        {
            RadioEvents->TxTimeout( );
        }
        break;
    default:
        break;
    }
}

void SX1276OnDio0Irq( void )
{
    volatile uint8_t irqFlags = 0;

    switch( SX1276.Settings.State )
    {
        case RF_RX_RUNNING:
            //TimerStop( &RxTimeoutTimer );
            // RxDone interrupt
            switch( SX1276.Settings.Modem )
            {
            case MODEM_FSK:
                if( SX1276.Settings.Fsk.CrcOn == true )
                {
                    irqFlags = SX1276Read( REG_IRQFLAGS2 );
                    if( ( irqFlags & RF_IRQFLAGS2_CRCOK ) != RF_IRQFLAGS2_CRCOK )
                    {
                        // Clear Irqs
                        SX1276Write( REG_IRQFLAGS1, RF_IRQFLAGS1_RSSI |
                                                    RF_IRQFLAGS1_PREAMBLEDETECT |
                                                    RF_IRQFLAGS1_SYNCADDRESSMATCH );
                        SX1276Write( REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN );

                        TimerStop( &RxTimeoutTimer );

                        if( SX1276.Settings.Fsk.RxContinuous == false )
                        {
                            TimerStop( &RxTimeoutSyncWord );
                            SX1276.Settings.State = RF_IDLE;
                        }
                        else
                        {
                            // Continuous mode restart Rx chain
                            SX1276Write( REG_RXCONFIG, SX1276Read( REG_RXCONFIG ) | RF_RXCONFIG_RESTARTRXWITHOUTPLLLOCK );
                            TimerStart( &RxTimeoutSyncWord );
                        }

                        if( ( RadioEvents != NULL ) && ( RadioEvents->RxError != NULL ) )
                        {
                            RadioEvents->RxError( );
                        }
                        SX1276.Settings.FskPacketHandler.PreambleDetected = false;
                        SX1276.Settings.FskPacketHandler.SyncWordDetected = false;
                        SX1276.Settings.FskPacketHandler.NbBytes = 0;
                        SX1276.Settings.FskPacketHandler.Size = 0;
                        break;
                    }
                }

                // Read received packet size
                if( ( SX1276.Settings.FskPacketHandler.Size == 0 ) && ( SX1276.Settings.FskPacketHandler.NbBytes == 0 ) )
                {
                    if( SX1276.Settings.Fsk.FixLen == false )
                    {
                        SX1276ReadFifo( ( uint8_t* )&SX1276.Settings.FskPacketHandler.Size, 1 );
                    }
                    else
                    {
                        SX1276.Settings.FskPacketHandler.Size = SX1276Read( REG_PAYLOADLENGTH );
                    }
                    SX1276ReadFifo( RxTxBuffer + SX1276.Settings.FskPacketHandler.NbBytes, SX1276.Settings.FskPacketHandler.Size - SX1276.Settings.FskPacketHandler.NbBytes );
                    SX1276.Settings.FskPacketHandler.NbBytes += ( SX1276.Settings.FskPacketHandler.Size - SX1276.Settings.FskPacketHandler.NbBytes );
                }
                else
                {
                    SX1276ReadFifo( RxTxBuffer + SX1276.Settings.FskPacketHandler.NbBytes, SX1276.Settings.FskPacketHandler.Size - SX1276.Settings.FskPacketHandler.NbBytes );
                    SX1276.Settings.FskPacketHandler.NbBytes += ( SX1276.Settings.FskPacketHandler.Size - SX1276.Settings.FskPacketHandler.NbBytes );
                }

                TimerStop( &RxTimeoutTimer );

                if( SX1276.Settings.Fsk.RxContinuous == false )
                {
                    SX1276.Settings.State = RF_IDLE;
                    TimerStop( &RxTimeoutSyncWord );
                }
                else
                {
                    // Continuous mode restart Rx chain
                    SX1276Write( REG_RXCONFIG, SX1276Read( REG_RXCONFIG ) | RF_RXCONFIG_RESTARTRXWITHOUTPLLLOCK );
                    TimerStart( &RxTimeoutSyncWord );
                }

                if( ( RadioEvents != NULL ) && ( RadioEvents->RxDone != NULL ) )
                {
                    RadioEvents->RxDone( RxTxBuffer, SX1276.Settings.FskPacketHandler.Size, SX1276.Settings.FskPacketHandler.RssiValue, 0 );
                }
                SX1276.Settings.FskPacketHandler.PreambleDetected = false;
                SX1276.Settings.FskPacketHandler.SyncWordDetected = false;
                SX1276.Settings.FskPacketHandler.NbBytes = 0;
                SX1276.Settings.FskPacketHandler.Size = 0;
                break;
            case MODEM_LORA:
                {
                    int8_t snr = 0;

                    // Clear Irq
                    SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXDONE );

                    irqFlags = SX1276Read( REG_LR_IRQFLAGS );
                    if( ( irqFlags & RFLR_IRQFLAGS_PAYLOADCRCERROR_MASK ) == RFLR_IRQFLAGS_PAYLOADCRCERROR )
                    {
                        // Clear Irq
                        SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_PAYLOADCRCERROR );

                        if( SX1276.Settings.LoRa.RxContinuous == false )
                        {
                            SX1276.Settings.State = RF_IDLE;
                        }
                        TimerStop( &RxTimeoutTimer );

                        if( ( RadioEvents != NULL ) && ( RadioEvents->RxError != NULL ) )
                        {
                            RadioEvents->RxError( );
                        }
                        break;
                    }

                    SX1276.Settings.LoRaPacketHandler.SnrValue = SX1276Read( REG_LR_PKTSNRVALUE );
                    if( SX1276.Settings.LoRaPacketHandler.SnrValue & 0x80 ) // The SNR sign bit is 1
                    {
                        // Invert and divide by 4
                        snr = ( ( ~SX1276.Settings.LoRaPacketHandler.SnrValue + 1 ) & 0xFF ) >> 2;
                        snr = -snr;
                    }
                    else
                    {
                        // Divide by 4
                        snr = ( SX1276.Settings.LoRaPacketHandler.SnrValue & 0xFF ) >> 2;
                    }

                    int16_t rssi = SX1276Read( REG_LR_PKTRSSIVALUE );
                    if( snr < 0 )
                    {
                        if( SX1276.Settings.Channel > RF_MID_BAND_THRESH )
                        {
                            SX1276.Settings.LoRaPacketHandler.RssiValue = RSSI_OFFSET_HF + rssi + ( rssi >> 4 ) +
                                                                          snr;
                        }
                        else
                        {
                            SX1276.Settings.LoRaPacketHandler.RssiValue = RSSI_OFFSET_LF + rssi + ( rssi >> 4 ) +
                                                                          snr;
                        }
                    }
                    else
                    {
                        if( SX1276.Settings.Channel > RF_MID_BAND_THRESH )
                        {
                            SX1276.Settings.LoRaPacketHandler.RssiValue = RSSI_OFFSET_HF + rssi + ( rssi >> 4 );
                        }
                        else
                        {
                            SX1276.Settings.LoRaPacketHandler.RssiValue = RSSI_OFFSET_LF + rssi + ( rssi >> 4 );
                        }
                    }

                    SX1276.Settings.LoRaPacketHandler.Size = SX1276Read( REG_LR_RXNBBYTES );
                    SX1276Write( REG_LR_FIFOADDRPTR, SX1276Read( REG_LR_FIFORXCURRENTADDR ) );
                    SX1276ReadFifo( RxTxBuffer, SX1276.Settings.LoRaPacketHandler.Size );

                    if( SX1276.Settings.LoRa.RxContinuous == false )
                    {
                        SX1276.Settings.State = RF_IDLE;
                    }
                    TimerStop( &RxTimeoutTimer );

                    if( ( RadioEvents != NULL ) && ( RadioEvents->RxDone != NULL ) )
                    {
                        RadioEvents->RxDone( RxTxBuffer, SX1276.Settings.LoRaPacketHandler.Size, SX1276.Settings.LoRaPacketHandler.RssiValue, SX1276.Settings.LoRaPacketHandler.SnrValue );
                    }
                }
                break;
            default:
                break;
            }
            break;
        case RF_TX_RUNNING:
            TimerStop( &TxTimeoutTimer );
            // TxDone interrupt
            switch( SX1276.Settings.Modem )
            {
            case MODEM_LORA:
                // Clear Irq
                SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_TXDONE );
                // Intentional fall through
            case MODEM_FSK:
            default:
                SX1276.Settings.State = RF_IDLE;
                if( ( RadioEvents != NULL ) && ( RadioEvents->TxDone != NULL ) )
                {
                    RadioEvents->TxDone( );
                }
                break;
            }
            break;
        default:
            break;
    }
}

void SX1276OnDio1Irq( void )
{
    switch( SX1276.Settings.State )
    {
        case RF_RX_RUNNING:
            switch( SX1276.Settings.Modem )
            {
            case MODEM_FSK:
                // FifoLevel interrupt
                // Read received packet size
                if( ( SX1276.Settings.FskPacketHandler.Size == 0 ) && ( SX1276.Settings.FskPacketHandler.NbBytes == 0 ) )
                {
                    if( SX1276.Settings.Fsk.FixLen == false )
                    {
                        SX1276ReadFifo( ( uint8_t* )&SX1276.Settings.FskPacketHandler.Size, 1 );
                    }
                    else
                    {
                        SX1276.Settings.FskPacketHandler.Size = SX1276Read( REG_PAYLOADLENGTH );
                    }
                }

                if( ( SX1276.Settings.FskPacketHandler.Size - SX1276.Settings.FskPacketHandler.NbBytes ) > SX1276.Settings.FskPacketHandler.FifoThresh )
                {
                    SX1276ReadFifo( ( RxTxBuffer + SX1276.Settings.FskPacketHandler.NbBytes ), SX1276.Settings.FskPacketHandler.FifoThresh );
                    SX1276.Settings.FskPacketHandler.NbBytes += SX1276.Settings.FskPacketHandler.FifoThresh;
                }
                else
                {
                    SX1276ReadFifo( ( RxTxBuffer + SX1276.Settings.FskPacketHandler.NbBytes ), SX1276.Settings.FskPacketHandler.Size - SX1276.Settings.FskPacketHandler.NbBytes );
                    SX1276.Settings.FskPacketHandler.NbBytes += ( SX1276.Settings.FskPacketHandler.Size - SX1276.Settings.FskPacketHandler.NbBytes );
                }
                break;
            case MODEM_LORA:
                // Sync time out
                TimerStop( &RxTimeoutTimer );
                // Clear Irq
                SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXTIMEOUT );

                SX1276.Settings.State = RF_IDLE;
                if( ( RadioEvents != NULL ) && ( RadioEvents->RxTimeout != NULL ) )
                {
                    RadioEvents->RxTimeout( );
                }
                break;
            default:
                break;
            }
            break;
        case RF_TX_RUNNING:
            switch( SX1276.Settings.Modem )
            {
            case MODEM_FSK:
                // FifoEmpty interrupt
                if( ( SX1276.Settings.FskPacketHandler.Size - SX1276.Settings.FskPacketHandler.NbBytes ) > SX1276.Settings.FskPacketHandler.ChunkSize )
                {
                    SX1276WriteFifo( ( RxTxBuffer + SX1276.Settings.FskPacketHandler.NbBytes ), SX1276.Settings.FskPacketHandler.ChunkSize );
                    SX1276.Settings.FskPacketHandler.NbBytes += SX1276.Settings.FskPacketHandler.ChunkSize;
                }
                else
                {
                    // Write the last chunk of data
                    SX1276WriteFifo( RxTxBuffer + SX1276.Settings.FskPacketHandler.NbBytes, SX1276.Settings.FskPacketHandler.Size - SX1276.Settings.FskPacketHandler.NbBytes );
                    SX1276.Settings.FskPacketHandler.NbBytes += SX1276.Settings.FskPacketHandler.Size - SX1276.Settings.FskPacketHandler.NbBytes;
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
}

void SX1276OnDio2Irq( void )
{
    switch( SX1276.Settings.State )
    {
        case RF_RX_RUNNING:
            switch( SX1276.Settings.Modem )
            {
            case MODEM_FSK:
                // Checks if DIO4 is connected. If it is not PreambleDetected is set to true.
                if( SX1276.DIO4.port == NULL )
                {
                    SX1276.Settings.FskPacketHandler.PreambleDetected = true;
                }

                if( ( SX1276.Settings.FskPacketHandler.PreambleDetected == true ) && ( SX1276.Settings.FskPacketHandler.SyncWordDetected == false ) )
                {
                    TimerStop( &RxTimeoutSyncWord );

                    SX1276.Settings.FskPacketHandler.SyncWordDetected = true;

                    SX1276.Settings.FskPacketHandler.RssiValue = -( SX1276Read( REG_RSSIVALUE ) >> 1 );

                    SX1276.Settings.FskPacketHandler.AfcValue = ( int32_t )( double )( ( ( uint16_t )SX1276Read( REG_AFCMSB ) << 8 ) |
                                                                           ( uint16_t )SX1276Read( REG_AFCLSB ) ) *
                                                                           ( double )FREQ_STEP;
                    SX1276.Settings.FskPacketHandler.RxGain = ( SX1276Read( REG_LNA ) >> 5 ) & 0x07;
                }
                break;
            case MODEM_LORA:
                if( SX1276.Settings.LoRa.FreqHopOn == true )
                {
                    // Clear Irq
                    SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL );

                    if( ( RadioEvents != NULL ) && ( RadioEvents->FhssChangeChannel != NULL ) )
                    {
                        RadioEvents->FhssChangeChannel( ( SX1276Read( REG_LR_HOPCHANNEL ) & RFLR_HOPCHANNEL_CHANNEL_MASK ) );
                    }
                }
                break;
            default:
                break;
            }
            break;
        case RF_TX_RUNNING:
            switch( SX1276.Settings.Modem )
            {
            case MODEM_FSK:
                break;
            case MODEM_LORA:
                if( SX1276.Settings.LoRa.FreqHopOn == true )
                {
                    // Clear Irq
                    SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL );

                    if( ( RadioEvents != NULL ) && ( RadioEvents->FhssChangeChannel != NULL ) )
                    {
                        RadioEvents->FhssChangeChannel( ( SX1276Read( REG_LR_HOPCHANNEL ) & RFLR_HOPCHANNEL_CHANNEL_MASK ) );
                    }
                }
                break;
            default:
                break;
            }
            break;
        default:
            break;
    }
}

void SX1276OnDio3Irq( void )
{
    switch( SX1276.Settings.Modem )
    {
    case MODEM_FSK:
        break;
    case MODEM_LORA:
        if( ( SX1276Read( REG_LR_IRQFLAGS ) & RFLR_IRQFLAGS_CADDETECTED ) == RFLR_IRQFLAGS_CADDETECTED )
        {
            // Clear Irq
            SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_CADDETECTED | RFLR_IRQFLAGS_CADDONE );
            if( ( RadioEvents != NULL ) && ( RadioEvents->CadDone != NULL ) )
            {
                RadioEvents->CadDone( true );
            }
        }
        else
        {
            // Clear Irq
            SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_CADDONE );
            if( ( RadioEvents != NULL ) && ( RadioEvents->CadDone != NULL ) )
            {
                RadioEvents->CadDone( false );
            }
        }
        break;
    default:
        break;
    }
}

void SX1276OnDio4Irq( void )
{
    switch( SX1276.Settings.Modem )
    {
    case MODEM_FSK:
        {
            if( SX1276.Settings.FskPacketHandler.PreambleDetected == false )
            {
                SX1276.Settings.FskPacketHandler.PreambleDetected = true;
            }
        }
        break;
    case MODEM_LORA:
        break;
    default:
        break;
    }
}

void SX1276OnDio5Irq( void )
{
    switch( SX1276.Settings.Modem )
    {
    case MODEM_FSK:
        break;
    case MODEM_LORA:
        break;
    default:
        break;
    }
}
