#include <linux/types.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include "sx1278.h"



static struct spi_device *spi_ptr = NULL;

static LoRa_Config_st_ptr sx1278_cfg_ptr;

static int8_t RxPacketSnrEstimate;
static int8_t RxPacketRssiValue;
static uint8_t RxGain = 1;
static uint32_t RxTimeoutTimer = 0;


/*!
 * Frequency hopping frequencies table
 */
const int32_t HoppingFrequencies[] =
{
    916500000,
    923500000,
    906500000,
    917500000,
    917500000,
    909000000,
    903000000,
    916000000,
    912500000,
    926000000,
    925000000,
    909500000,
    913000000,
    918500000,
    918500000,
    902500000,
    911500000,
    926500000,
    902500000,
    922000000,
    924000000,
    903500000,
    913000000,
    922000000,
    926000000,
    910000000,
    920000000,
    922500000,
    911000000,
    922000000,
    909500000,
    926000000,
    922000000,
    918000000,
    925500000,
    908000000,
    917500000,
    926500000,
    908500000,
    916000000,
    905500000,
    916000000,
    903000000,
    905000000,
    915000000,
    913000000,
    907000000,
    910000000,
    926500000,
    925500000,
    911000000,
};

int SX1278_Read_Reg(uint8_t addr, uint8_t *data_ptr)
{
    int result = 0;
    uint8_t reg = (addr & 0x7F);

    if((NULL == spi_ptr) || (NULL == data_ptr))
    {
        printk(KERN_ERR "%s %d error %d reading %x\n", __FUNCTION__, __LINE__, result, addr);
        result = -ENOMEM;
        goto ERR_EXIT;
    }
    
    result = spi_write_then_read(spi_ptr, &reg, 1, data_ptr, 1);
    if (result < 0)
    {
        printk(KERN_ERR "%s %d error %d reading %x\n", __FUNCTION__, __LINE__, result, addr);
        goto ERR_EXIT;
    }

ERR_EXIT:
    
    return result;
}

int SX1278_Write_Reg(uint8_t addr, uint8_t data)
{
    int result = 0;
    uint8_t data_buf[2] = {0};
    
    data_buf[0] = (addr | 0x80);
    data_buf[1] = data;

    if(NULL == spi_ptr)
    {
        printk(KERN_ERR "%s %d error %d\n", __FUNCTION__, __LINE__, result);
        result = -ENOMEM;
        goto ERR_EXIT;
    }

    result = spi_write(spi_ptr, (void *)data_buf, 2);
    if (result < 0)
    {
        printk(KERN_ERR "%s %d write %d\n", __FUNCTION__, __LINE__, result);
        goto ERR_EXIT;
    }

ERR_EXIT:    

    return result;
}

int SX1278_Read_FIFO(uint8_t *data_ptr, uint8_t data_len)
{
    int result = 0;
    uint8_t reg = 0x00;
    uint8_t *pdata = NULL;
    struct spi_transfer t[2] = {0};
    struct spi_message m = {0};

    if((NULL == spi_ptr) || (NULL == data_ptr))
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
    
    spi_message_add_tail(t, &m);

    spi_sync(spi_ptr, &m);

    result += m.actual_length - 1;

ERR_EXIT:
    if(pdata)
        kfree(pdata);
    return result;
}

int SX1278_Write_FIFO(uint8_t *data_ptr, uint8_t data_len)
{
    int result = 0;
    uint8_t *pdata = NULL;
    struct spi_transfer t = {0};
    struct spi_message m = {0};

    if((NULL == spi_ptr) || (NULL == data_ptr))
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

    spi_sync(spi_ptr, &m);

    result += m.actual_length - 1;
    
ERR_EXIT:

    if(pdata)
        kfree(pdata);
    
    return result;
}


void SX1278LoRaInit( void )
{
    uint8_t val = 0;

    SX1278LoRaSetDefaults( );
    
    //SX1278ReadBuffer( REG_LR_OPMODE, SX1278Regs + 1, 0x70 - 1 );

    val = RFLR_LNA_GAIN_G1;
    SX1278_Read_Reg(REG_LR_LNA, &val);

    //SX1278WriteBuffer( REG_LR_OPMODE, SX1278Regs + 1, 0x70 - 1 );

    // set the Rx RF settings 
    SX1278LoRaSetRFFrequency( sx1278_cfg_ptr->rx_cfg.RFFrequency );
    SX1278LoRaSetSpreadingFactor( sx1278_cfg_ptr->rx_cfg.SpreadingFactor ); // SF6 only operates in implicit header mode.
    SX1278LoRaSetErrorCoding( sx1278_cfg_ptr->rx_cfg.ErrorCoding );
    SX1278LoRaSetPacketCrcOn( sx1278_cfg_ptr->rx_cfg.CrcOn );
    SX1278LoRaSetSignalBandwidth( sx1278_cfg_ptr->rx_cfg.SignalBw );

    SX1278LoRaSetImplicitHeaderOn( sx1278_cfg_ptr->rx_cfg.ImplicitHeaderOn );
    SX1278LoRaSetSymbTimeout( 0x3FF );
    SX1278LoRaSetPayloadLength( sx1278_cfg_ptr->rx_cfg.PayloadLength );
    SX1278LoRaSetLowDatarateOptimize( true );

    if( sx1278_cfg_ptr->tx_cfg.RFFrequency > 860000000 )
    {
        SX1278LoRaSetPAOutput( RFLR_PACONFIG_PASELECT_PABOOST );
        SX1278LoRaSetPa20dBm( true );
        sx1278_cfg_ptr->tx_cfg.Power = 20;
        SX1278LoRaSetRFPower( sx1278_cfg_ptr->tx_cfg.Power );
    }
    else
    {
        SX1278LoRaSetPAOutput( RFLR_PACONFIG_PASELECT_RFO );
        SX1278LoRaSetPa20dBm( false );
        sx1278_cfg_ptr->tx_cfg.Power = 14;
        SX1278LoRaSetRFPower( sx1278_cfg_ptr->tx_cfg.Power );
    } 

    SX1278LoRaSetOpMode( RFLR_OPMODE_STANDBY );
}

void SX1278LoRaSetDefaults( void )
{
    // REMARK: See SX1278 datasheet for modified default values.

    //SX1278_Read_Reg( REG_LR_VERSION, &SX1278LR->RegVersion );
}
/*
void SX1278LoRaReset( void )
{
    SX1278SetReset( RADIO_RESET_ON );
    
    // Wait 1ms
    uint32_t startTick = GET_TICK_COUNT( );
    while( ( GET_TICK_COUNT( ) - startTick ) < TICK_RATE_MS( 1 ) );    

    SX1278SetReset( RADIO_RESET_OFF );
    
    // Wait 6ms
    startTick = GET_TICK_COUNT( );
    while( ( GET_TICK_COUNT( ) - startTick ) < TICK_RATE_MS( 6 ) );    
}
*/
void SX1278LoRaSetOpMode( uint8_t opMode )
{
    uint8_t RegOpMode = 0;
    static uint8_t opModePrev = RFLR_OPMODE_STANDBY;
    static bool antennaSwitchTxOnPrev = true;
    bool antennaSwitchTxOn = false;
    uint8_t val = 0;

    SX1278_Read_Reg(0x01, &RegOpMode);
    //RegOpMode = SX1278LoRaGetOpMode();
    opModePrev = RegOpMode & ~RFLR_OPMODE_MASK;

    if( opMode != opModePrev )
    {
        if( opMode == RFLR_OPMODE_TRANSMITTER )
        {
            antennaSwitchTxOn = true;
        }
        else
        {
            antennaSwitchTxOn = false;
        }
        if( antennaSwitchTxOn != antennaSwitchTxOnPrev )
        {
            antennaSwitchTxOnPrev = antennaSwitchTxOn;
            //RXTX( antennaSwitchTxOn ); // Antenna switch control
        }
        RegOpMode = ( RegOpMode & RFLR_OPMODE_MASK ) | opMode;
        SX1278_Write_Reg( REG_LR_OPMODE, RegOpMode );        
    }
    //RegOpMode = SX1278LoRaGetOpMode();
}

uint8_t SX1278LoRaGetModem(void)
{
    uint8_t RegOpMode = 0;

    SX1278_Read_Reg(REG_LR_OPMODE, &RegOpMode);
   
    return RegOpMode & ~RFLR_MODEM_MASK;
}

uint8_t SX1278LoRaGetOpMode( void )
{
    uint8_t RegOpMode = 0;
    
    SX1278_Read_Reg( REG_LR_OPMODE, &RegOpMode );
    
    return RegOpMode & ~RFLR_OPMODE_MASK;
}

uint8_t SX1278LoRaReadRxGain( void )
{
    uint8_t val = 0x00;
    
    SX1278_Read_Reg( REG_LR_LNA, &val );
    
    return( val >> 5 ) & 0x07;
}

int8_t SX1278LoRaReadRssi( void )
{
    uint8_t val = 0x00;
    
    // Reads the RSSI value
    SX1278_Read_Reg( REG_LR_RSSIVALUE, &val );

    if( sx1278_cfg_ptr->rx_cfg.RFFrequency < 860000000 )  // LF
    {
        return RSSI_OFFSET_LF + val;
    }
    else
    {
        return RSSI_OFFSET_HF + val;
    }
}

uint8_t SX1278LoRaGetPacketRxGain( void )
{
    return RxGain;
}

int8_t SX1278LoRaGetPacketSnr( void )
{
    return RxPacketSnrEstimate;
}

int8_t SX1278LoRaGetPacketRssi( void )
{
    return RxPacketRssiValue;
}

void SX1278LoRaSetRFFrequency( uint32_t freq )
{
    uint8_t val = 0;
    uint64_t temp = 0;
      
    temp = freq * 100000000;
    do_div(temp, FREQ_STEP);
    
    freq = ( uint32_t )temp;
    
    val = ( uint8_t )( ( freq >> 16 ) & 0xFF );
    SX1278_Write_Reg(REG_LR_FRFMSB, val);

    val = ( uint8_t )( ( freq >> 8 ) & 0xFF );
    SX1278_Write_Reg(REG_LR_FRFMID, val);

    val = ( uint8_t )( freq & 0xFF );
    SX1278_Write_Reg(REG_LR_FRFLSB, val);
    
}

uint32_t SX1278LoRaGetRFFrequency( void )
{
    uint8_t val = 0;
    uint8_t RegFrfMsb = 0;
    uint8_t RegFrfMid = 0;
    uint8_t RegFrfLsb = 0;
    uint32_t freq = 0;
    uint64_t temp = 0;

    SX1278_Write_Reg(REG_LR_FRFMSB, RegFrfMsb);
    SX1278_Write_Reg(REG_LR_FRFMID, RegFrfMsb);
    SX1278_Write_Reg(REG_LR_FRFLSB, RegFrfMsb);

    temp = ( ( uint32_t )RegFrfMsb << 16 ) | ( ( uint32_t )RegFrfMid << 8 ) | ( ( uint32_t )RegFrfLsb ); 
    temp *= FREQ_STEP;

    do_div(temp, 100000000);
    freq = (uint32_t)temp;
    
    return freq;
}

void SX1278LoRaSetRFPower( int8_t power )
{
    uint8_t RegPaConfig = 0;
    uint8_t RegPaDac = 0;
    
    SX1278_Read_Reg( REG_LR_PACONFIG, &RegPaConfig );
    SX1278_Read_Reg( REG_LR_PADAC, &RegPaDac );
    
    if( ( RegPaConfig & RFLR_PACONFIG_PASELECT_PABOOST ) == RFLR_PACONFIG_PASELECT_PABOOST )
    {
        if( ( RegPaDac & 0x87 ) == 0x87 )
        {
            if( power < 5 )
            {
                power = 5;
            }
            if( power > 20 )
            {
                power = 20;
            }
            RegPaConfig = (RegPaConfig & RFLR_PACONFIG_MAX_POWER_MASK ) | 0x70;
            RegPaConfig = (RegPaConfig & RFLR_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 5 ) & 0x0F );
        }
        else
        {
            if( power < 2 )
            {
                power = 2;
            }
            if( power > 17 )
            {
                power = 17;
            }
            RegPaConfig = (RegPaConfig & RFLR_PACONFIG_MAX_POWER_MASK ) | 0x70;
            RegPaConfig = (RegPaConfig & RFLR_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 2 ) & 0x0F );
        }
    }
    else
    {
        if( power < -1 )
        {
            power = -1;
        }
        if( power > 14 )
        {
            power = 14;
        }
        RegPaConfig = (RegPaConfig & RFLR_PACONFIG_MAX_POWER_MASK ) | 0x70;
        RegPaConfig = (RegPaConfig & RFLR_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power + 1 ) & 0x0F );
    }
    
    SX1278_Write_Reg( REG_LR_PACONFIG, RegPaConfig );
}

int8_t SX1278LoRaGetRFPower( void )
{
    int8_t power;
    uint8_t RegPaConfig = 0;
    uint8_t RegPaDac = 0;

    SX1278_Read_Reg( REG_LR_PACONFIG, &RegPaConfig );
    SX1278_Read_Reg( REG_LR_PADAC, &RegPaDac );

    if( ( RegPaConfig & RFLR_PACONFIG_PASELECT_PABOOST ) == RFLR_PACONFIG_PASELECT_PABOOST )
    {
        if( (RegPaDac & 0x07 ) == 0x07 )
        {
            power = 5 + (RegPaConfig & ~RFLR_PACONFIG_OUTPUTPOWER_MASK );
        }
        else
        {
            power = 2 + (RegPaConfig & ~RFLR_PACONFIG_OUTPUTPOWER_MASK );
        }
    }
    else
    {
        power = -1 + (RegPaConfig & ~RFLR_PACONFIG_OUTPUTPOWER_MASK );
    }
    return power;
}

void SX1278LoRaSetSignalBandwidth( uint8_t bw )
{
    uint8_t RegModemConfig1 = 0;
    
    SX1278_Read_Reg( REG_LR_MODEMCONFIG1, &RegModemConfig1 );
    RegModemConfig1 = (RegModemConfig1 & RFLR_MODEMCONFIG1_BW_MASK ) | ( bw << 4 );
    SX1278_Write_Reg( REG_LR_MODEMCONFIG1, RegModemConfig1 );
}

uint8_t SX1278LoRaGetSignalBandwidth( void )
{
    uint8_t SignalBw = 0;
    uint8_t RegModemConfig1 = 0;
    
    SX1278_Read_Reg(REG_LR_MODEMCONFIG1, &RegModemConfig1 );
    
    SignalBw = (RegModemConfig1 & ~RFLR_MODEMCONFIG1_BW_MASK ) >> 4;
    
    return SignalBw;
}

void SX1278LoRaSetSpreadingFactor( uint8_t factor )
{
    uint8_t RegModemConfig2 = 0;
    if( factor > 12 )
    {
        factor = 12;
    }
    else if( factor < 6 )
    {
        factor = 6;
    }

    if( factor == 6 )
    {
        SX1278LoRaSetNbTrigPeaks( 5 );
    }
    else
    {
        SX1278LoRaSetNbTrigPeaks( 3 );
    }

    SX1278_Read_Reg( REG_LR_MODEMCONFIG2, &RegModemConfig2 );
    RegModemConfig2 = (RegModemConfig2 & RFLR_MODEMCONFIG2_SF_MASK ) | ( factor << 4 );
    SX1278_Write_Reg( REG_LR_MODEMCONFIG2, RegModemConfig2 );
}

uint8_t SX1278LoRaGetSpreadingFactor( void )
{
    uint8_t SpreadingFactor = 0;
    uint8_t RegModemConfig2 = 0;
    
    SX1278_Read_Reg(REG_LR_MODEMCONFIG2, &RegModemConfig2 );   
    SpreadingFactor = (RegModemConfig2 & ~RFLR_MODEMCONFIG2_SF_MASK ) >> 4;
    return SpreadingFactor;
}

void SX1278LoRaSetErrorCoding( uint8_t value )
{
    uint8_t RegModemConfig1 = 0;
    
    SX1278_Read_Reg(REG_LR_MODEMCONFIG1, &RegModemConfig1 );
    RegModemConfig1 = (RegModemConfig1 & RFLR_MODEMCONFIG1_CODINGRATE_MASK ) | ( value << 1 );
    SX1278_Write_Reg(REG_LR_MODEMCONFIG1, RegModemConfig1 );
}

uint8_t SX1278LoRaGetErrorCoding( void )
{
    uint8_t ErrorCoding =0;
    uint8_t RegModemConfig1 = 0;
    
    SX1278_Read_Reg(REG_LR_MODEMCONFIG1, &RegModemConfig1 );
    ErrorCoding = (RegModemConfig1 & ~RFLR_MODEMCONFIG1_CODINGRATE_MASK ) >> 1;
    
    return ErrorCoding;
}

void SX1278LoRaSetPacketCrcOn( bool enable )
{
    uint8_t RegModemConfig2 = 0;
    
    SX1278_Read_Reg(REG_LR_MODEMCONFIG2, &RegModemConfig2 );
    RegModemConfig2 = (RegModemConfig2 & RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK ) | ( enable << 2 );
    SX1278_Write_Reg(REG_LR_MODEMCONFIG2, RegModemConfig2 );
}

void SX1278LoRaSetPreambleLength( uint16_t value )
{
    uint8_t RegPreambleMsb = 0;
    uint8_t RegPreambleLsb = 0;
    
    SX1278_Read_Reg( REG_LR_PREAMBLEMSB, &RegPreambleMsb);
    SX1278_Read_Reg( REG_LR_PREAMBLELSB, &RegPreambleLsb);

    RegPreambleMsb = ( value >> 8 ) & 0x00FF;
    RegPreambleLsb = value & 0xFF;
    
    SX1278_Write_Reg( REG_LR_PREAMBLEMSB, RegPreambleMsb);
    SX1278_Write_Reg( REG_LR_PREAMBLELSB, RegPreambleLsb);
}

uint16_t SX1278LoRaGetPreambleLength( void )
{
    uint8_t RegPreambleMsb = 0;
    uint8_t RegPreambleLsb = 0;
    uint16_t PreambleLen = 0;

    SX1278_Read_Reg(REG_LR_PREAMBLEMSB, &RegPreambleMsb);
    SX1278_Read_Reg(REG_LR_PREAMBLELSB, &RegPreambleLsb);

    PreambleLen = ( ( RegPreambleMsb & 0x00FF ) << 8 ) | RegPreambleLsb;
    return PreambleLen;
}

bool SX1278LoRaGetPacketCrcOn( void )
{
    bool CrcOn = false;
    uint8_t RegModemConfig2 = 0;
    
    SX1278_Read_Reg( REG_LR_MODEMCONFIG2, &RegModemConfig2 );
    CrcOn = (RegModemConfig2 & RFLR_MODEMCONFIG2_RXPAYLOADCRC_ON ) >> 1;
    
    return CrcOn;
}

void SX1278LoRaSetImplicitHeaderOn( bool enable )
{
    uint8_t RegModemConfig1 = 0;
    
    SX1278_Read_Reg( REG_LR_MODEMCONFIG1, &RegModemConfig1 );
    RegModemConfig1 = (RegModemConfig1 & RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK ) | ( enable );
    SX1278_Write_Reg( REG_LR_MODEMCONFIG1, RegModemConfig1 );
}

bool SX1278LoRaGetImplicitHeaderOn( void )
{
    bool ImplicitHeaderOn = false;
    uint8_t RegModemConfig1 =0;
    
    SX1278_Read_Reg( REG_LR_MODEMCONFIG1, &RegModemConfig1 );
    ImplicitHeaderOn = ( RegModemConfig1 & RFLR_MODEMCONFIG1_IMPLICITHEADER_ON );
    
    return ImplicitHeaderOn;
}

void SX1278LoRaSetRxSingleOn( bool enable )
{
    //LoRaSettings.RxSingleOn = enable;
}

bool SX1278LoRaGetRxSingleOn( void )
{
    //return LoRaSettings.RxSingleOn;
    return true;
}

void SX1278LoRaSetFreqHopOn( bool enable )
{
    //LoRaSettings.FreqHopOn = enable;
}

bool SX1278LoRaGetFreqHopOn( void )
{
    //return LoRaSettings.FreqHopOn;
    return true;
}

void SX1278LoRaSetHopPeriod( uint8_t value )
{
    SX1278_Write_Reg( REG_LR_HOPPERIOD, value );
}

uint8_t SX1278LoRaGetHopPeriod( void )
{
    uint8_t RegHopPeriod =0;
    uint8_t HopPeriod = 0;

    SX1278_Read_Reg( REG_LR_HOPPERIOD, &RegHopPeriod );
    HopPeriod = RegHopPeriod;
    return HopPeriod;
}

void SX1278LoRaSetTxPacketTimeout( uint32_t value )
{
    //LoRaSettings.TxPacketTimeout = value;
}

uint32_t SX1278LoRaGetTxPacketTimeout( void )
{
    //return LoRaSettings.TxPacketTimeout;
    return 0;
}

void SX1278LoRaSetRxPacketTimeout( uint32_t value )
{
    //LoRaSettings.RxPacketTimeout = value;
}

uint32_t SX1278LoRaGetRxPacketTimeout( void )
{
    //return LoRaSettings.RxPacketTimeout;
    return 0;
}

void SX1278LoRaSetPayloadLength( uint8_t value )
{
    SX1278_Write_Reg( REG_LR_PAYLOADLENGTH, value);
}

uint8_t SX1278LoRaGetPayloadLength( void )
{
    uint8_t RegPayloadLength = 0;
    
    SX1278_Read_Reg( REG_LR_PAYLOADLENGTH, &RegPayloadLength );

    return RegPayloadLength;
}

void SX1278LoRaSetPa20dBm( bool enale )
{
    uint8_t RegPaDac = 0;
    uint8_t RegPaConfig = 0;
    
    SX1278_Read_Reg( REG_LR_PADAC, &RegPaDac );
    SX1278_Read_Reg( REG_LR_PACONFIG, &RegPaConfig );

    if( (RegPaConfig & RFLR_PACONFIG_PASELECT_PABOOST ) == RFLR_PACONFIG_PASELECT_PABOOST )
    {    
        if( enale == true )
        {
            RegPaDac = 0x87;
        }
    }
    else
    {
        RegPaDac = 0x84;
    }
    
    SX1278_Write_Reg( REG_LR_PADAC, RegPaDac );
}

bool SX1278LoRaGetPa20dBm( void )
{
    bool Pa20dBm = false;
    uint8_t RegPaDac = 0;
    
    SX1278_Read_Reg( REG_LR_PADAC, &RegPaDac );

    Pa20dBm = ( ( RegPaDac & 0x07 ) == 0x07 ) ? true : false;
    
    return Pa20dBm;
}

void SX1278LoRaSetPAOutput( uint8_t outputPin )
{
    uint8_t RegPaConfig = 0;
    
    SX1278_Read_Reg( REG_LR_PACONFIG, &RegPaConfig );
    RegPaConfig = (RegPaConfig & RFLR_PACONFIG_PASELECT_MASK ) | outputPin;
    SX1278_Write_Reg( REG_LR_PACONFIG, RegPaConfig );
}

uint8_t SX1278LoRaGetPAOutput( void )
{
    uint8_t RegPaConfig = 0;
    
    SX1278_Read_Reg( REG_LR_PACONFIG, &RegPaConfig );
    return RegPaConfig & ~RFLR_PACONFIG_PASELECT_MASK;
}

void SX1278LoRaSetPaRamp( uint8_t value )
{
    uint8_t RegPaRamp = 0;
    
    SX1278_Read_Reg( REG_LR_PARAMP, &RegPaRamp );
    RegPaRamp = (RegPaRamp & RFLR_PARAMP_MASK ) | ( value & ~RFLR_PARAMP_MASK );
    SX1278_Write_Reg( REG_LR_PARAMP, RegPaRamp );
}

uint8_t SX1278LoRaGetPaRamp( void )
{
    uint8_t RegPaRamp = 0;
    SX1278_Read_Reg( REG_LR_PARAMP, &RegPaRamp );
    return RegPaRamp & ~RFLR_PARAMP_MASK;
}

void SX1278LoRaSetSymbTimeout( uint16_t value )
{
    uint8_t RegModemConfig2 = 0;
    uint8_t RegSymbTimeoutLsb = 0;
    
    SX1278_Read_Reg( REG_LR_MODEMCONFIG2, &RegModemConfig2);
    SX1278_Read_Reg( REG_LR_SYMBTIMEOUTLSB, &RegModemConfig2);

    RegModemConfig2 = ( RegModemConfig2 & RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK ) | ( ( value >> 8 ) & ~RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK );
    RegSymbTimeoutLsb = value & 0xFF;

    SX1278_Write_Reg(REG_LR_MODEMCONFIG2, RegModemConfig2);
    SX1278_Write_Reg(REG_LR_SYMBTIMEOUTLSB, RegSymbTimeoutLsb);
}

uint16_t SX1278LoRaGetSymbTimeout( void )
{
    uint8_t RegModemConfig2 = 0;
    uint8_t RegSymbTimeoutLsb = 0;
    uint16_t SymbTimeout = 0;
    
    SX1278_Read_Reg( REG_LR_MODEMCONFIG2, &RegModemConfig2);
    SX1278_Read_Reg( REG_LR_SYMBTIMEOUTLSB, &RegModemConfig2);

    SymbTimeout = ( ( RegModemConfig2 & ~RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK ) << 8 ) | RegSymbTimeoutLsb;
    return SymbTimeout;
}

void SX1278LoRaSetLowDatarateOptimize( bool enable )
{
    uint8_t RegModemConfig3 =0;
    
    SX1278_Read_Reg( REG_LR_MODEMCONFIG3, &RegModemConfig3 );
    RegModemConfig3 = ( RegModemConfig3 & RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_MASK ) | ( enable << 3 );
    SX1278_Write_Reg( REG_LR_MODEMCONFIG3, RegModemConfig3 );
}

bool SX1278LoRaGetLowDatarateOptimize( void )
{
    uint8_t RegModemConfig3 = 0;
    
    SX1278_Read_Reg( REG_LR_MODEMCONFIG3, &RegModemConfig3 );
    return ( ( RegModemConfig3 & RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_ON ) >> 3 );
}

void SX1278LoRaSetNbTrigPeaks( uint8_t value )
{
    uint8_t RegDetectOptimize = 0;
    
    SX1278_Read_Reg( 0x31, &RegDetectOptimize );
    RegDetectOptimize = ( RegDetectOptimize & 0xF8 ) | value;
    SX1278_Write_Reg( 0x31, RegDetectOptimize );
}

uint8_t SX1278LoRaGetNbTrigPeaks( void )
{
    uint8_t RegDetectOptimize = 0;
    SX1278_Read_Reg( 0x31, &RegDetectOptimize );
    return (RegDetectOptimize & 0x07 );
}


void SX1278LoRaStartRx( void )
{
    //SX1278LoRaSetRFState( RFLR_STATE_RX_INIT );
}

void SX1278LoRaGetRxPacket( void *buffer, uint16_t *size )
{
    //*size = RxPacketSize;
    //RxPacketSize = 0;
    //memcpy( ( void * )buffer, ( void * )RFBuffer, ( size_t )*size );
}

void SX1278LoRaSetTxPacket( const void *buffer, uint16_t size )
{
    //TxPacketSize = size;
    //memcpy( ( void * )RFBuffer, buffer, ( size_t )TxPacketSize ); 
    //
    //RFLRState = RFLR_STATE_TX_INIT;
}

uint8_t SX1278LoRaGetRFState( void )
{
    //return RFLRState;
    return 0;
}

void SX1278LoRaSetRFState( uint8_t state )
{
    //RFLRState = state;
}

/*!
 * \brief Process the LoRa modem Rx and Tx state machines depending on the
 *        SX1278 operating mode.
 *
 * \retval rfState Current RF state [RF_IDLE, RF_BUSY, 
 *                                   RF_RX_DONE, RF_RX_TIMEOUT,
 *                                   RF_TX_DONE, RF_TX_TIMEOUT]
 */
#if 0 
uint32_t SX1278LoRaProcess( void )
{
    uint32_t result = RF_BUSY;
    
    switch( RFLRState )
    {
    case RFLR_STATE_IDLE:
        break;
    case RFLR_STATE_RX_INIT:
        
        SX1278LoRaSetOpMode( RFLR_OPMODE_STANDBY );

        SX1278LR->RegIrqFlagsMask = RFLR_IRQFLAGS_RXTIMEOUT |
                                    //RFLR_IRQFLAGS_RXDONE |
                                    //RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                    RFLR_IRQFLAGS_VALIDHEADER |
                                    RFLR_IRQFLAGS_TXDONE |
                                    RFLR_IRQFLAGS_CADDONE |
                                    //RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                    RFLR_IRQFLAGS_CADDETECTED;
        SX1278_Write_Reg( REG_LR_IRQFLAGSMASK, SX1278LR->RegIrqFlagsMask );

        if( LoRaSettings.FreqHopOn == true )
        {
            SX1278LR->RegHopPeriod = LoRaSettings.HopPeriod;

            SX1278_Read_Reg( REG_LR_HOPCHANNEL, &SX1278LR->RegHopChannel );
            SX1278LoRaSetRFFrequency( HoppingFrequencies[SX1278LR->RegHopChannel & RFLR_HOPCHANNEL_CHANNEL_MASK] );
        }
        else
        {
            SX1278LR->RegHopPeriod = 255;
        }
        
        SX1278Write( REG_LR_HOPPERIOD, SX1278LR->RegHopPeriod );
                
                                    // RxDone                    RxTimeout                   FhssChangeChannel           CadDone
        SX1278LR->RegDioMapping1 = RFLR_DIOMAPPING1_DIO0_00 | RFLR_DIOMAPPING1_DIO1_00 | RFLR_DIOMAPPING1_DIO2_00 | RFLR_DIOMAPPING1_DIO3_00;
                                    // CadDetected               ModeReady
        SX1278LR->RegDioMapping2 = RFLR_DIOMAPPING2_DIO4_00 | RFLR_DIOMAPPING2_DIO5_00;
        SX1278WriteBuffer( REG_LR_DIOMAPPING1, &SX1278LR->RegDioMapping1, 2 );
    
        if( LoRaSettings.RxSingleOn == true ) // Rx single mode
        {

            SX1278LoRaSetOpMode( RFLR_OPMODE_RECEIVER_SINGLE );
        }
        else // Rx continuous mode
        {
            SX1278LR->RegFifoAddrPtr = SX1278LR->RegFifoRxBaseAddr;
            SX1278Write( REG_LR_FIFOADDRPTR, SX1278LR->RegFifoAddrPtr );
            
            SX1278LoRaSetOpMode( RFLR_OPMODE_RECEIVER );
        }
        
        memset( RFBuffer, 0, ( size_t )RF_BUFFER_SIZE );

        PacketTimeout = LoRaSettings.RxPacketTimeout;
        RxTimeoutTimer = GET_TICK_COUNT( );
        RFLRState = RFLR_STATE_RX_RUNNING;
        break;
    case RFLR_STATE_RX_RUNNING:
        
        if( DIO0 == 1 ) // RxDone
        {
            RxTimeoutTimer = GET_TICK_COUNT( );
            if( LoRaSettings.FreqHopOn == true )
            {
                SX1278Read( REG_LR_HOPCHANNEL, &SX1278LR->RegHopChannel );
                SX1278LoRaSetRFFrequency( HoppingFrequencies[SX1278LR->RegHopChannel & RFLR_HOPCHANNEL_CHANNEL_MASK] );
            }
            // Clear Irq
            SX1278Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXDONE  );
            RFLRState = RFLR_STATE_RX_DONE;
        }
        if( DIO2 == 1 ) // FHSS Changed Channel
        {
            RxTimeoutTimer = GET_TICK_COUNT( );
            if( LoRaSettings.FreqHopOn == true )
            {
                SX1278Read( REG_LR_HOPCHANNEL, &SX1278LR->RegHopChannel );
                SX1278LoRaSetRFFrequency( HoppingFrequencies[SX1278LR->RegHopChannel & RFLR_HOPCHANNEL_CHANNEL_MASK] );
            }
            // Clear Irq
            SX1278Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL );
            // Debug
            RxGain = SX1278LoRaReadRxGain( );
        }

        if( LoRaSettings.RxSingleOn == true ) // Rx single mode
        {
            if( ( GET_TICK_COUNT( ) - RxTimeoutTimer ) > PacketTimeout )
            {
                RFLRState = RFLR_STATE_RX_TIMEOUT;
            }
        }
        break;
    case RFLR_STATE_RX_DONE:
        SX1278Read( REG_LR_IRQFLAGS, &SX1278LR->RegIrqFlags );
        if( ( SX1278LR->RegIrqFlags & RFLR_IRQFLAGS_PAYLOADCRCERROR ) == RFLR_IRQFLAGS_PAYLOADCRCERROR )
        {
            // Clear Irq
            SX1278Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_PAYLOADCRCERROR  );
            
            if( LoRaSettings.RxSingleOn == true ) // Rx single mode
            {
                RFLRState = RFLR_STATE_RX_INIT;
            }
            else
            {
                RFLRState = RFLR_STATE_RX_RUNNING;
            }
            break;
        }
        
        {
            uint8_t rxSnrEstimate;
            SX1278Read( REG_LR_PKTSNRVALUE, &rxSnrEstimate );
            if( rxSnrEstimate & 0x80 ) // The SNR sign bit is 1
            {
                // Invert and divide by 4
                RxPacketSnrEstimate = ( ( ~rxSnrEstimate + 1 ) & 0xFF ) >> 2;
                RxPacketSnrEstimate = -RxPacketSnrEstimate;
            }
            else
            {
                // Divide by 4
                RxPacketSnrEstimate = ( rxSnrEstimate & 0xFF ) >> 2;
            }
        }
        
        SX1278Read( REG_LR_PKTRSSIVALUE, &SX1278LR->RegPktRssiValue );
    
        if( LoRaSettings.RFFrequency < 860000000 )  // LF
        {    
            if( RxPacketSnrEstimate < 0 )
            {
                RxPacketRssiValue = RSSI_OFFSET_LF + ( SX1278LR->RegPktRssiValue ) + RxPacketSnrEstimate;
            }
            else
            {
                RxPacketRssiValue = RSSI_OFFSET_LF + (uint8_t)( (uint32_t) 10666 * SX1278LR->RegPktRssiValue / 1000 );
            }
        }
        else                                        // HF
        {    
            if( RxPacketSnrEstimate < 0 )
            {
                RxPacketRssiValue = RSSI_OFFSET_HF + SX1278LR->RegPktRssiValue + RxPacketSnrEstimate;
            }
            else
            {    
                RxPacketRssiValue = RSSI_OFFSET_HF + (uint8_t)((uint32_t)10666 * SX1278LR->RegPktRssiValue / 1000 );
            }
        }

        if( LoRaSettings.RxSingleOn == true ) // Rx single mode
        {
            SX1278LR->RegFifoAddrPtr = SX1278LR->RegFifoRxBaseAddr;
            SX1278Write( REG_LR_FIFOADDRPTR, SX1278LR->RegFifoAddrPtr );

            if( LoRaSettings.ImplicitHeaderOn == true )
            {
                RxPacketSize = SX1278LR->RegPayloadLength;
                SX1278ReadFifo( RFBuffer, SX1278LR->RegPayloadLength );
            }
            else
            {
                SX1278Read( REG_LR_NBRXBYTES, &SX1278LR->RegNbRxBytes );
                RxPacketSize = SX1278LR->RegNbRxBytes;
                SX1278ReadFifo( RFBuffer, SX1278LR->RegNbRxBytes );
            }
        }
        else // Rx continuous mode
        {
            SX1278Read( REG_LR_FIFORXCURRENTADDR, &SX1278LR->RegFifoRxCurrentAddr );

            if( LoRaSettings.ImplicitHeaderOn == true )
            {
                RxPacketSize = SX1278LR->RegPayloadLength;
                SX1278LR->RegFifoAddrPtr = SX1278LR->RegFifoRxCurrentAddr;
                SX1278Write( REG_LR_FIFOADDRPTR, SX1278LR->RegFifoAddrPtr );
                SX1278ReadFifo( RFBuffer, SX1278LR->RegPayloadLength );
            }
            else
            {
                SX1278Read( REG_LR_NBRXBYTES, &SX1278LR->RegNbRxBytes );
                RxPacketSize = SX1278LR->RegNbRxBytes;
                SX1278LR->RegFifoAddrPtr = SX1278LR->RegFifoRxCurrentAddr;
                SX1278Write( REG_LR_FIFOADDRPTR, SX1278LR->RegFifoAddrPtr );
                SX1278ReadFifo( RFBuffer, SX1278LR->RegNbRxBytes );
            }
        }
        
        if( LoRaSettings.RxSingleOn == true ) // Rx single mode
        {
            RFLRState = RFLR_STATE_RX_INIT;
        }
        else // Rx continuous mode
        {
            RFLRState = RFLR_STATE_RX_RUNNING;
        }
        result = RF_RX_DONE;
        break;
    case RFLR_STATE_RX_TIMEOUT:
        RFLRState = RFLR_STATE_RX_INIT;
        result = RF_RX_TIMEOUT;
        break;
    case RFLR_STATE_TX_INIT:

        SX1278LoRaSetOpMode( RFLR_OPMODE_STANDBY );

        if( LoRaSettings.FreqHopOn == true )
        {
            SX1278LR->RegIrqFlagsMask = RFLR_IRQFLAGS_RXTIMEOUT |
                                        RFLR_IRQFLAGS_RXDONE |
                                        RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                        RFLR_IRQFLAGS_VALIDHEADER |
                                        //RFLR_IRQFLAGS_TXDONE |
                                        RFLR_IRQFLAGS_CADDONE |
                                        //RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                        RFLR_IRQFLAGS_CADDETECTED;
            SX1278LR->RegHopPeriod = LoRaSettings.HopPeriod;

            SX1278Read( REG_LR_HOPCHANNEL, &SX1278LR->RegHopChannel );
            SX1278LoRaSetRFFrequency( HoppingFrequencies[SX1278LR->RegHopChannel & RFLR_HOPCHANNEL_CHANNEL_MASK] );
        }
        else
        {
            SX1278LR->RegIrqFlagsMask = RFLR_IRQFLAGS_RXTIMEOUT |
                                        RFLR_IRQFLAGS_RXDONE |
                                        RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                        RFLR_IRQFLAGS_VALIDHEADER |
                                        //RFLR_IRQFLAGS_TXDONE |
                                        RFLR_IRQFLAGS_CADDONE |
                                        RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                        RFLR_IRQFLAGS_CADDETECTED;
            SX1278LR->RegHopPeriod = 0;
        }
        SX1278Write( REG_LR_HOPPERIOD, SX1278LR->RegHopPeriod );
        SX1278Write( REG_LR_IRQFLAGSMASK, SX1278LR->RegIrqFlagsMask );

        // Initializes the payload size
        SX1278LR->RegPayloadLength = TxPacketSize;
        SX1278Write( REG_LR_PAYLOADLENGTH, SX1278LR->RegPayloadLength );
        
        SX1278LR->RegFifoTxBaseAddr = 0x00; // Full buffer used for Tx
        SX1278Write( REG_LR_FIFOTXBASEADDR, SX1278LR->RegFifoTxBaseAddr );

        SX1278LR->RegFifoAddrPtr = SX1278LR->RegFifoTxBaseAddr;
        SX1278Write( REG_LR_FIFOADDRPTR, SX1278LR->RegFifoAddrPtr );
        
        // Write payload buffer to LORA modem
        SX1278WriteFifo( RFBuffer, SX1278LR->RegPayloadLength );
                                        // TxDone               RxTimeout                   FhssChangeChannel          ValidHeader         
        SX1278LR->RegDioMapping1 = RFLR_DIOMAPPING1_DIO0_01 | RFLR_DIOMAPPING1_DIO1_00 | RFLR_DIOMAPPING1_DIO2_00 | RFLR_DIOMAPPING1_DIO3_01;
                                        // PllLock              Mode Ready
        SX1278LR->RegDioMapping2 = RFLR_DIOMAPPING2_DIO4_01 | RFLR_DIOMAPPING2_DIO5_00;
        SX1278WriteBuffer( REG_LR_DIOMAPPING1, &SX1278LR->RegDioMapping1, 2 );

        SX1278LoRaSetOpMode( RFLR_OPMODE_TRANSMITTER );

        RFLRState = RFLR_STATE_TX_RUNNING;
        break;
    case RFLR_STATE_TX_RUNNING:
        if( DIO0 == 1 ) // TxDone
        {
            // Clear Irq
            SX1278Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_TXDONE  );
            RFLRState = RFLR_STATE_TX_DONE;   
        }
        if( DIO2 == 1 ) // FHSS Changed Channel
        {
            if( LoRaSettings.FreqHopOn == true )
            {
                SX1278Read( REG_LR_HOPCHANNEL, &SX1278LR->RegHopChannel );
                SX1278LoRaSetRFFrequency( HoppingFrequencies[SX1278LR->RegHopChannel & RFLR_HOPCHANNEL_CHANNEL_MASK] );
            }
            // Clear Irq
            SX1278Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL );
        }
        break;
    case RFLR_STATE_TX_DONE:
        // optimize the power consumption by switching off the transmitter as soon as the packet has been sent
        SX1278LoRaSetOpMode( RFLR_OPMODE_STANDBY );

        RFLRState = RFLR_STATE_IDLE;
        result = RF_TX_DONE;
        break;
    case RFLR_STATE_CAD_INIT:    
        SX1278LoRaSetOpMode( RFLR_OPMODE_STANDBY );
    
        SX1278LR->RegIrqFlagsMask = RFLR_IRQFLAGS_RXTIMEOUT |
                                    RFLR_IRQFLAGS_RXDONE |
                                    RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                    RFLR_IRQFLAGS_VALIDHEADER |
                                    RFLR_IRQFLAGS_TXDONE |
                                    //RFLR_IRQFLAGS_CADDONE |
                                    RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL; // |
                                    //RFLR_IRQFLAGS_CADDETECTED;
        SX1278Write( REG_LR_IRQFLAGSMASK, SX1278LR->RegIrqFlagsMask );
           
                                    // RxDone                   RxTimeout                   FhssChangeChannel           CadDone
        SX1278LR->RegDioMapping1 = RFLR_DIOMAPPING1_DIO0_00 | RFLR_DIOMAPPING1_DIO1_00 | RFLR_DIOMAPPING1_DIO2_00 | RFLR_DIOMAPPING1_DIO3_00;
                                    // CAD Detected              ModeReady
        SX1278LR->RegDioMapping2 = RFLR_DIOMAPPING2_DIO4_00 | RFLR_DIOMAPPING2_DIO5_00;
        SX1278WriteBuffer( REG_LR_DIOMAPPING1, &SX1278LR->RegDioMapping1, 2 );
            
        SX1278LoRaSetOpMode( RFLR_OPMODE_CAD );
        RFLRState = RFLR_STATE_CAD_RUNNING;
        break;
    case RFLR_STATE_CAD_RUNNING:
        if( DIO3 == 1 ) //CAD Done interrupt
        { 
            // Clear Irq
            SX1278Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_CADDONE  );
            if( DIO4 == 1 ) // CAD Detected interrupt
            {
                // Clear Irq
                SX1278Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_CADDETECTED  );
                // CAD detected, we have a LoRa preamble
                RFLRState = RFLR_STATE_RX_INIT;
                result = RF_CHANNEL_ACTIVITY_DETECTED;
            } 
            else
            {    
                // The device goes in Standby Mode automatically    
                RFLRState = RFLR_STATE_IDLE;
                result = RF_CHANNEL_EMPTY;
            }
        }   
        break;
    
    default:
        break;
    } 
    return result;
}
#endif
void SX1278Init(struct spi_device *spi, LoRa_Config_st_ptr cfg_ptr)
{
    bool LoRaOn = true;
    uint8_t val = 0;
    // Initialize FSK and LoRa registers structure
    spi_ptr = spi;
    sx1278_cfg_ptr = cfg_ptr;
    
    SX1278SetLoRaOn(LoRaOn);
    // Initialize LoRa modem
    SX1278LoRaInit();

}

void SX1278Reset( void )
{
    //SX1278SetReset( RADIO_RESET_ON );
    
    // Wait 1ms
    mdelay(1);    

    //SX1278SetReset( RADIO_RESET_OFF );
    
    // Wait 6ms
    mdelay(6);
}

void SX1278SetLoRaOn( bool enable )
{
    uint8_t RegDioMapping = 0;
    uint8_t LoRaOnState = 0;
    uint8_t RegOpMode = 0;

    LoRaOnState = SX1278LoRaGetModem();//SX1278LoRaGetOpMode();

    if( LoRaOnState == enable )
    {
        printk(KERN_ERR "%s %d the same mode:lora\n", __FUNCTION__, __LINE__);
        return;
    }
     
    if( enable )
    {
        SX1278LoRaSetOpMode( RFLR_OPMODE_SLEEP );

        SX1278_Read_Reg(REG_LR_OPMODE, &RegOpMode);
        RegOpMode = ( RegOpMode & RFLR_OPMODE_LONGRANGEMODE_MASK ) | RFLR_OPMODE_LONGRANGEMODE_ON;
        SX1278_Write_Reg( REG_LR_OPMODE, RegOpMode );
        SX1278LoRaSetOpMode( RFLR_OPMODE_STANDBY );
                                        // RxDone               RxTimeout                   FhssChangeChannel           CadDone
        RegDioMapping = RFLR_DIOMAPPING1_DIO0_00 | RFLR_DIOMAPPING1_DIO1_00 | RFLR_DIOMAPPING1_DIO2_00 | RFLR_DIOMAPPING1_DIO3_00;
        SX1278_Write_Reg(REG_LR_DIOMAPPING1, RegDioMapping);
                                        // CadDetected          ModeReady
        RegDioMapping = RFLR_DIOMAPPING2_DIO4_00 | RFLR_DIOMAPPING2_DIO5_00;
        SX1278_Write_Reg(REG_LR_DIOMAPPING2, RegDioMapping);
    }
    else
    {
        SX1278LoRaSetOpMode( RFLR_OPMODE_SLEEP );

        SX1278_Read_Reg(REG_LR_OPMODE, &RegOpMode);
        RegOpMode = ( RegOpMode & RFLR_OPMODE_LONGRANGEMODE_MASK ) | RFLR_OPMODE_LONGRANGEMODE_OFF;
        SX1278_Write_Reg( REG_LR_OPMODE, RegOpMode );
        
        SX1278LoRaSetOpMode( RFLR_OPMODE_STANDBY );
    }
}

bool SX1278GetLoRaOn( void )
{
    return true;
    //return LoRaOn;
}

void SX1278SetOpMode( uint8_t opMode )
{
    SX1278LoRaSetOpMode( opMode );
}

uint8_t SX1278GetOpMode( void )
{
    return SX1278LoRaGetOpMode( );
}

int8_t SX1278ReadRssi( void )
{
    return SX1278LoRaReadRssi( );
}

uint8_t SX1278ReadRxGain( void )
{
    return SX1278LoRaReadRxGain( );
}

uint8_t SX1278GetPacketRxGain( void )
{
    return SX1278LoRaGetPacketRxGain(  );
}

int8_t SX1278GetPacketSnr( void )
{
    return SX1278LoRaGetPacketSnr(  );
}

int8_t SX1278GetPacketRssi( void )
{
    return SX1278LoRaGetPacketRssi( );
}

uint32_t SX1278GetPacketAfc( void )
{
#if 0
    if( LoRaOn == false )
    {
        return SX1278FskGetPacketAfc(  );
    }
    else
    {
         while( 1 )
         {
             // Useless in LoRa mode
             // Block program here
         }
    }
#endif
    return 0;
}

void SX1278StartRx( void )
{
    uint8_t RegHopPeriod = 0;
    uint8_t RegHopChannel = 0;
    uint8_t RegIrqFlagsMask = 0;
    uint8_t RegDioMapping1 = 0;
    uint8_t RegDioMapping2 = 0;
    uint8_t RegFifoAddrPtr = 0;
    uint8_t RegFifoRxBaseAddr = 0;
    
    SX1278LoRaSetOpMode( RFLR_OPMODE_STANDBY );

    RegIrqFlagsMask = RFLR_IRQFLAGS_RXTIMEOUT |
                        //RFLR_IRQFLAGS_RXDONE |
                        //RFLR_IRQFLAGS_PAYLOADCRCERROR |
                        RFLR_IRQFLAGS_VALIDHEADER |
                        RFLR_IRQFLAGS_TXDONE |
                        RFLR_IRQFLAGS_CADDONE |
                        //RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                        RFLR_IRQFLAGS_CADDETECTED;
    SX1278_Write_Reg( REG_LR_IRQFLAGSMASK, RegIrqFlagsMask );

    if(sx1278_cfg_ptr->rx_cfg.FreqHopOn == true )
    {
        RegHopPeriod = sx1278_cfg_ptr->rx_cfg.HopPeriod;

        SX1278_Read_Reg( REG_LR_HOPCHANNEL, &RegHopChannel );
        SX1278LoRaSetRFFrequency( HoppingFrequencies[RegHopChannel & RFLR_HOPCHANNEL_CHANNEL_MASK] );
    }
    else
    {
        RegHopPeriod = 255;
    }
    
    SX1278_Write_Reg( REG_LR_HOPPERIOD, RegHopPeriod );
            
                                // RxDone                    RxTimeout                   FhssChangeChannel           CadDone
    RegDioMapping1 = RFLR_DIOMAPPING1_DIO0_00 | RFLR_DIOMAPPING1_DIO1_00 | RFLR_DIOMAPPING1_DIO2_00 | RFLR_DIOMAPPING1_DIO3_00;
                                // CadDetected               ModeReady
    RegDioMapping2 = RFLR_DIOMAPPING2_DIO4_00 | RFLR_DIOMAPPING2_DIO5_00;

    SX1278_Write_Reg( REG_LR_DIOMAPPING1, RegDioMapping1);
    SX1278_Write_Reg( REG_LR_DIOMAPPING2, RegDioMapping2);

    if( sx1278_cfg_ptr->rx_cfg.RxSingleOn == true ) // Rx single mode
    {

        SX1278LoRaSetOpMode( RFLR_OPMODE_RECEIVER_SINGLE );
    }
    else // Rx continuous mode
    {
        //problem
        RegFifoAddrPtr = RegFifoRxBaseAddr;
        SX1278_Write_Reg( REG_LR_FIFOADDRPTR, RegFifoAddrPtr );
        
        SX1278LoRaSetOpMode( RFLR_OPMODE_RECEIVER );
    }

    //memset( RFBuffer, 0, ( size_t )RF_BUFFER_SIZE );
    //PacketTimeout = LoRaSettings.RxPacketTimeout;
    //RxTimeoutTimer = GET_TICK_COUNT( );
    //RFLRState = RFLR_STATE_RX_RUNNING;
}


int SX1278GetRxPacket( uint8_t *buffer_ptr, uint8_t buffer_len)
{
    int result = 0;
    uint8_t RegIrqFlags = 0;
    uint8_t RegPktRssiValue = 0;
    uint8_t RegFifoAddrPtr = 0;
    uint8_t RegFifoRxBaseAddr = 0;
    //SX1278LoRaGetRxPacket( buffer, size );
    
    SX1278_Read_Reg( REG_LR_IRQFLAGS, &RegIrqFlags );
    if( ( RegIrqFlags & RFLR_IRQFLAGS_PAYLOADCRCERROR ) == RFLR_IRQFLAGS_PAYLOADCRCERROR )
    {
        // Clear Irq
        SX1278_Write_Reg( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_PAYLOADCRCERROR  );
        
        if( sx1278_cfg_ptr->rx_cfg.RxSingleOn == true ) // Rx single mode
        {
            //RFLRState = RFLR_STATE_RX_INIT;
            SX1278StartRx();
        }
        else
        {
            //RFLRState = RFLR_STATE_RX_RUNNING;
        }

        return 0;
    }
    
    {
        uint8_t rxSnrEstimate;
        SX1278_Read_Reg( REG_LR_PKTSNRVALUE, &rxSnrEstimate );
        if( rxSnrEstimate & 0x80 ) // The SNR sign bit is 1
        {
            // Invert and divide by 4
            RxPacketSnrEstimate = ( ( ~rxSnrEstimate + 1 ) & 0xFF ) >> 2;
            RxPacketSnrEstimate = -RxPacketSnrEstimate;
        }
        else
        {
            // Divide by 4
            RxPacketSnrEstimate = ( rxSnrEstimate & 0xFF ) >> 2;
        }
    }
    
    SX1278_Read_Reg( REG_LR_PKTRSSIVALUE, &RegPktRssiValue );

    if( sx1278_cfg_ptr->rx_cfg.RFFrequency < 860000000 )  // LF
    {    
        if( RxPacketSnrEstimate < 0 )
        {
            RxPacketRssiValue = RSSI_OFFSET_LF + RegPktRssiValue + RxPacketSnrEstimate;
        }
        else
        {
            RxPacketRssiValue = RSSI_OFFSET_LF + (uint8_t)((uint32_t)10666 * RegPktRssiValue / 10000);
        }
    }
    else                                        // HF
    {    
        if( RxPacketSnrEstimate < 0 )
        {
            RxPacketRssiValue = RSSI_OFFSET_HF + RegPktRssiValue + RxPacketSnrEstimate;
        }
        else
        {    
            RxPacketRssiValue = RSSI_OFFSET_HF + (uint8_t)((uint32_t)10666 * RegPktRssiValue / 10000 );
        }
    }

    if( sx1278_cfg_ptr->rx_cfg.RxSingleOn == true ) // Rx single mode
    {
        RegFifoAddrPtr = RegFifoRxBaseAddr;
        SX1278_Write_Reg( REG_LR_FIFOADDRPTR, RegFifoAddrPtr );

        if( sx1278_cfg_ptr->rx_cfg.ImplicitHeaderOn == true )
        {
            SX1278_Read_FIFO( buffer_ptr, buffer_len );
        }
        else
        {
            uint8_t RegNbRxBytes = 0;
            SX1278_Read_Reg( REG_LR_RXNBBYTES, &RegNbRxBytes );
            //RxPacketSize = SX1276LR->RegNbRxBytes;
            SX1278_Read_FIFO( buffer_ptr, RegNbRxBytes );
            result = RegNbRxBytes;
        }
    }
    else // Rx continuous mode
    {
        uint8_t RegFifoRxCurrentAddr = 0;
        SX1278_Read_Reg( REG_LR_FIFORXCURRENTADDR, &RegFifoRxCurrentAddr );

        if( sx1278_cfg_ptr->rx_cfg.ImplicitHeaderOn == true )
        {            
            RegFifoAddrPtr = RegFifoRxCurrentAddr;
            SX1278_Write_Reg( REG_LR_FIFOADDRPTR, RegFifoAddrPtr );
            SX1278_Read_FIFO( buffer_ptr, buffer_len );
        }
        else
        {
            uint8_t RegNbRxBytes = 0;
            SX1278_Read_Reg( REG_LR_RXNBBYTES, &RegNbRxBytes );
            //RxPacketSize = SX1276LR->RegNbRxBytes;
            RegFifoAddrPtr = RegFifoRxCurrentAddr;
            SX1278_Write_Reg( REG_LR_FIFOADDRPTR, RegFifoAddrPtr );
            SX1278_Read_FIFO( buffer_ptr, RegNbRxBytes );
        }
    }
    
    if( sx1278_cfg_ptr->rx_cfg.RxSingleOn == true ) // Rx single mode
    {
        //RFLRState = RFLR_STATE_RX_INIT;
    }
    else // Rx continuous mode
    {
        //RFLRState = RFLR_STATE_RX_RUNNING;
    }
}

void SX1278RxFinished(void)
{
    uint8_t RegHopChannel = 0;
    
    if( sx1278_cfg_ptr->rx_cfg.FreqHopOn == true )
    {
        SX1278_Read_Reg( REG_LR_HOPCHANNEL, &RegHopChannel );
        SX1278LoRaSetRFFrequency( HoppingFrequencies[RegHopChannel & RFLR_HOPCHANNEL_CHANNEL_MASK] );
    }
    // Clear Irq
    //SX1278_Write_Reg( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXDONE);
}

void SX1278RxClearIrq(void)
{
    uint8_t RegHopChannel = 0;
    
    if( sx1278_cfg_ptr->rx_cfg.FreqHopOn == true )
    {
        SX1278_Read_Reg( REG_LR_HOPCHANNEL, &RegHopChannel );
        SX1278LoRaSetRFFrequency( HoppingFrequencies[RegHopChannel & RFLR_HOPCHANNEL_CHANNEL_MASK] );
    }

    SX1278_Write_Reg( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXDONE);
}

void SX1278StartTx( void )
{
    uint8_t reg_val = 0;
    uint8_t RegDioMapping1 = 0;
    uint8_t RegDioMapping2 = 0;
                              // TxDone               RxTimeout                   FhssChangeChannel          ValidHeader  
    RegDioMapping1 = RFLR_DIOMAPPING1_DIO0_01 | RFLR_DIOMAPPING1_DIO1_00 | RFLR_DIOMAPPING1_DIO2_00 | RFLR_DIOMAPPING1_DIO3_01;
                              // PllLock              Mode Ready
    RegDioMapping2 = RFLR_DIOMAPPING2_DIO4_01 | RFLR_DIOMAPPING2_DIO5_00;
    SX1278_Write_Reg( REG_LR_DIOMAPPING1, RegDioMapping1);
    SX1278_Write_Reg( REG_LR_DIOMAPPING2, RegDioMapping2);

    SX1278LoRaSetOpMode( RFLR_OPMODE_TRANSMITTER );
}

void SX1278SetTxPacket( const uint8_t *buffer_ptr, uint8_t buffer_len )
{
    uint8_t RegPayloadLength = 0;
    uint8_t RegFifoTxBaseAddr = 0;
    uint8_t RegFifoAddrPtr = 0;
    uint8_t RegIrqFlagsMask = 0;
    uint8_t RegHopPeriod = 0;
    uint8_t RegHopChannel = 0;
    
    //SX1278LoRaSetTxPacket( buffer, size );
    
    SX1278LoRaSetOpMode( RFLR_OPMODE_STANDBY );

    if( sx1278_cfg_ptr->tx_cfg.FreqHopOn == true )
    {
        RegIrqFlagsMask = RFLR_IRQFLAGS_RXTIMEOUT |
                            RFLR_IRQFLAGS_RXDONE |
                            RFLR_IRQFLAGS_PAYLOADCRCERROR |
                            RFLR_IRQFLAGS_VALIDHEADER |
                            //RFLR_IRQFLAGS_TXDONE |
                            RFLR_IRQFLAGS_CADDONE |
                            //RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                            RFLR_IRQFLAGS_CADDETECTED;

        RegHopPeriod = sx1278_cfg_ptr->tx_cfg.HopPeriod;

        SX1278_Read_Reg( REG_LR_HOPCHANNEL, &RegHopChannel );
        SX1278LoRaSetRFFrequency( HoppingFrequencies[RegHopChannel & RFLR_HOPCHANNEL_CHANNEL_MASK] );
    }
    else
    {
        RegIrqFlagsMask = RFLR_IRQFLAGS_RXTIMEOUT |
                                RFLR_IRQFLAGS_RXDONE |
                                RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                RFLR_IRQFLAGS_VALIDHEADER |
                                //RFLR_IRQFLAGS_TXDONE |
                                RFLR_IRQFLAGS_CADDONE |
                                RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                RFLR_IRQFLAGS_CADDETECTED;
        RegHopPeriod = 0;
    }
    SX1278_Write_Reg( REG_LR_HOPPERIOD, RegHopPeriod );
    SX1278_Write_Reg( REG_LR_IRQFLAGSMASK, RegIrqFlagsMask );
    // Initializes the payload size
    RegPayloadLength = buffer_len;
    SX1278_Write_Reg( REG_LR_PAYLOADLENGTH, RegPayloadLength );
    
    RegFifoTxBaseAddr = 0x00; // Full buffer used for Tx
    SX1278_Write_Reg( REG_LR_FIFOTXBASEADDR, RegFifoTxBaseAddr );

    RegFifoAddrPtr = RegFifoTxBaseAddr;
    SX1278_Write_Reg( REG_LR_FIFOADDRPTR, RegFifoAddrPtr );
    
    // Write payload buffer to LORA modem
    SX1278_Write_FIFO( buffer_ptr, RegPayloadLength );       

}

void SX1278TxFinished(void)
{
    SX1278_Write_Reg( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_TXDONE );

    SX1278LoRaSetOpMode( RFLR_OPMODE_STANDBY );
}

void SX1278FhssChangedChannel(void)
{
    uint8_t RegHopChannel = 0;
    //problem
    if( sx1278_cfg_ptr->rx_cfg.FreqHopOn == true )
    {
        SX1278_Read_Reg( REG_LR_HOPCHANNEL, &RegHopChannel );
        SX1278LoRaSetRFFrequency( HoppingFrequencies[RegHopChannel & RFLR_HOPCHANNEL_CHANNEL_MASK] );
    }
    // Clear Irq
    SX1278_Write_Reg( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL );
    // Debug
    RxGain = SX1278LoRaReadRxGain( );
    
}

void SX1278StartCad(void)
{
    uint8_t RegIrqFlagsMask = 0;
    uint8_t RegDioMapping1 = 0;
    uint8_t RegDioMapping2 = 0;
    
    SX1278LoRaSetOpMode( RFLR_OPMODE_STANDBY );

    RegIrqFlagsMask = RFLR_IRQFLAGS_RXTIMEOUT |
                        RFLR_IRQFLAGS_RXDONE |
                        RFLR_IRQFLAGS_PAYLOADCRCERROR |
                        RFLR_IRQFLAGS_VALIDHEADER |
                        RFLR_IRQFLAGS_TXDONE |
                        //RFLR_IRQFLAGS_CADDONE |
                        RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL; // |
                        //RFLR_IRQFLAGS_CADDETECTED;
    SX1278_Write_Reg( REG_LR_IRQFLAGSMASK, RegIrqFlagsMask );
       
                                // RxDone                   RxTimeout                   FhssChangeChannel           CadDone
    RegDioMapping1 = RFLR_DIOMAPPING1_DIO0_00 | RFLR_DIOMAPPING1_DIO1_00 | RFLR_DIOMAPPING1_DIO2_00 | RFLR_DIOMAPPING1_DIO3_00;
                                // CAD Detected              ModeReady
    RegDioMapping2 = RFLR_DIOMAPPING2_DIO4_00 | RFLR_DIOMAPPING2_DIO5_00;

    SX1278_Write_Reg( REG_LR_DIOMAPPING1, RegDioMapping1);
    SX1278_Write_Reg( REG_LR_DIOMAPPING1, RegDioMapping2);
        
    SX1278LoRaSetOpMode( RFLR_OPMODE_CAD );
}

void SX1278CadFinished(void)
{
    SX1278_Write_Reg( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_CADDONE);
    //if( DIO4 == 1 ) // CAD Detected interrupt
    {
        // Clear Irq
        SX1278_Write_Reg( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_CADDETECTED  );
        // CAD detected, we have a LoRa preamble
        //RFLRState = RFLR_STATE_RX_INIT;
        //result = RF_CHANNEL_ACTIVITY_DETECTED;
    } 
    //else
    {    
        // The device goes in Standby Mode automatically    
        //RFLRState = RFLR_STATE_IDLE;
        //result = RF_CHANNEL_EMPTY;
    }
}
uint8_t SX1278GetRFState( void )
{
    return SX1278LoRaGetRFState( );
}

void SX1278SetRFState( uint8_t state )
{
    SX1278LoRaSetRFState( state );
}

uint32_t SX1278Process( void )
{
    //return SX1278LoRaProcess( );
    return 0;
}


