#include <linux/types.h>
#include <linux/delay.h>
#include "sx1278.h"


/*
static struct spi_device *spi_ptr = NULL;

static LoRa_Config_st_ptr sx1278_cfg_ptr;

static int8_t RxPacketSnrEstimate;
static int8_t RxPacketRssiValue;
static uint8_t RxGain = 1;
static uint32_t RxTimeoutTimer = 0;
*/

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

int SX1278_Read_Reg(struct spi_device *spi_ptr, uint8_t addr, uint8_t *data_ptr)
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

int SX1278_Write_Reg(struct spi_device *spi_ptr, uint8_t addr, uint8_t data)
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

int SX1278_Read_FIFO(struct spi_device *spi_ptr, uint8_t *data_ptr, uint8_t data_len)
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

int SX1278_Write_FIFO(struct spi_device *spi_ptr, uint8_t *data_ptr, uint8_t data_len)
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


int SX1278LoRaInit(SX1278_st_ptr sx1278_ptr)
{
    int result = 0;
    uint8_t regval = 0;

    if(NULL == sx1278_ptr)
    {
        result = -ENOMEM;
        goto ERR_EXIT;
    }

    result = SX1278LoRaSetDefaults(sx1278_ptr);
    if(result < 0)
    {
        goto ERR_EXIT;
    }
    
    //SX1278ReadBuffer(REG_LR_OPMODE, SX1278Regs + 1, 0x70 - 1);

    regval = RFLR_LNA_GAIN_G1;
    result = SX1278_Read_Reg(sx1278_ptr->spi_ptr, REG_LR_LNA, &regval);
    if(result < 0)
    {
        goto ERR_EXIT;
    }

    //SX1278WriteBuffer(REG_LR_OPMODE, SX1278Regs + 1, 0x70 - 1);

    // set the Rx RF settings 
    result = SX1278LoRaSetRFFrequency(sx1278_ptr, sx1278_ptr->cfg.rx_cfg.RFFrequency);
    if(result < 0)
    {
        goto ERR_EXIT;
    }
    
    result = SX1278LoRaSetSpreadingFactor(sx1278_ptr, sx1278_ptr->cfg.rx_cfg.SpreadingFactor); // SF6 only operates in implicit header mode.
    if(result < 0)
    {
        goto ERR_EXIT;
    }
    
    result = SX1278LoRaSetErrorCoding(sx1278_ptr, sx1278_ptr->cfg.rx_cfg.ErrorCoding);
    if(result < 0)
    {
        goto ERR_EXIT;
    }

    result = SX1278LoRaSetPacketCrcOn(sx1278_ptr, sx1278_ptr->cfg.rx_cfg.CrcOn);
    if(result < 0)
    {
        goto ERR_EXIT;
    }
    
    result = SX1278LoRaSetSignalBandwidth(sx1278_ptr, sx1278_ptr->cfg.rx_cfg.SignalBw);
    if(result < 0)
    {
        goto ERR_EXIT;
    }

    result = SX1278LoRaSetImplicitHeaderOn(sx1278_ptr, sx1278_ptr->cfg.rx_cfg.ImplicitHeaderOn);
    if(result < 0)
    {
        goto ERR_EXIT;
    }
    
    result = SX1278LoRaSetSymbTimeout(sx1278_ptr, 0x3FF);
    if(result < 0)
    {
        goto ERR_EXIT;
    }
    
    result = SX1278LoRaSetPayloadLength(sx1278_ptr, sx1278_ptr->cfg.rx_cfg.PayloadLength);
    if(result < 0)
    {
        goto ERR_EXIT;
    }
    
    result = SX1278LoRaSetLowDatarateOptimize(sx1278_ptr, true);
    if(result < 0)
    {
        goto ERR_EXIT;
    }

    if( sx1278_ptr->cfg.tx_cfg.RFFrequency > 860000000 )
    {
        result = SX1278LoRaSetPAOutput(sx1278_ptr, RFLR_PACONFIG_PASELECT_PABOOST);
        if(result < 0)
        {
            goto ERR_EXIT;
        }
        
        result = SX1278LoRaSetPa20dBm(sx1278_ptr, true);
        if(result < 0)
        {
            goto ERR_EXIT;
        }
        
        sx1278_ptr->cfg.tx_cfg.Power = 20;
        result = SX1278LoRaSetRFPower(sx1278_ptr, sx1278_ptr->cfg.tx_cfg.Power);
        if(result < 0)
        {
            goto ERR_EXIT;
        }        
    }
    else
    {
        result = SX1278LoRaSetPAOutput(sx1278_ptr, RFLR_PACONFIG_PASELECT_RFO);
        if(result < 0)
        {
            goto ERR_EXIT;
        }
        
        result = SX1278LoRaSetPa20dBm(sx1278_ptr, false);
        if(result < 0)
        {
            goto ERR_EXIT;
        }
        
        sx1278_ptr->cfg.tx_cfg.Power = 14;
        result = SX1278LoRaSetRFPower(sx1278_ptr, sx1278_ptr->tx_cfg.Power);
        if(result < 0)
        {
            goto ERR_EXIT;
        }        
    } 

    SX1278LoRaSetOpMode( RFLR_OPMODE_STANDBY );

ERR_EXIT:

    return result;
}

int SX1278LoRaSetDefaults(SX1278_st_ptr sx1278_ptr)
{
    int result = 0;

    if(NULL == sx1278_ptr)
    {
        goto ERR_EXIT;
    }
    // REMARK: See SX1278 datasheet for modified default values.

    //SX1278_Read_Reg( REG_LR_VERSION, &SX1278LR->RegVersion );
ERR_EXIT:

    return result;
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
int SX1278LoRaSetOpMode(SX1278_st_ptr sx1278_ptr, uint8_t opMode)
{
    int result = 0;
    uint8_t RegOpMode = 0;
    uint8_t opModePrev = RFLR_OPMODE_STANDBY;
    bool antennaSwitchTxOnPrev = true;
    bool antennaSwitchTxOn = false;
    uint8_t val = 0;

    if(NULL == sx1278_ptr)
    {
        result = -ENOMEM;
        goto ERR_EXIT;
    }
    
    result = SX1278LoRaGetOpMode(sx1278_ptr, &opModePrev);
    if(result < 0)
    {
        goto ERR_EXIT;
    }

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
        
        result = SX1278_Write_Reg(sx1278_ptr->spi_ptr, REG_LR_OPMODE, RegOpMode);   
        if(result < 0)
        {
            goto ERR_EXIT;
        }
    }

ERR_EXIT:
    
    return result;
}

int SX1278LoRaGetModem(SX1278_st_ptr sx1278_ptr, uint8_t *modem)
{
    int result = 0;
    uint8_t RegOpMode = 0;

    if((NULL == sx1278_ptr) || (NULL == modem))
    {
        result = -ENOMEM;
        goto ERR_EXIT;
    }

    result = SX1278_Read_Reg(sx1278_ptr->spi_ptr, REG_LR_OPMODE, &RegOpMode);
    if(result < 0)
    {
        goto ERR_EXIT;
    }

    *modem = RegOpMode & ~RFLR_MODEM_MASK;

ERR_EXIT:
   
    return result;
}

int SX1278LoRaGetOpMode(SX1278_st_ptr sx1278_ptr, uint8_t *opmode)
{
    int result = 0;
    uint8_t RegOpMode = 0;

    if((NULL == sx1278_ptr)||(NULL == opmode))
    {
        result = -ENOMEM;
        goto ERR_EXIT;
    }
    
    result = SX1278_Read_Reg(sx1278_ptr->spi_ptr, REG_LR_OPMODE, &RegOpMode);
    if(result < 0)
    {
        goto ERR_EXIT;
    }

    *opmode = RegOpMode & ~RFLR_OPMODE_MASK;

ERR_EXIT:
    
    return result;
}

int SX1278LoRaReadRxGain(SX1278_st_ptr sx1278_ptr, uint8_t *RxGain)
{
    int result = 0;
    uint8_t val = 0x00;
    
    if((NULL == sx1278_ptr)||(NULL == RxGain))
    {
        result = -ENOMEM;
        goto ERR_EXIT;
    }
    
    result = SX1278_Read_Reg(sx1278_ptr->spi_ptr, REG_LR_LNA, &val);
    if(result < 0)
    {
        goto ERR_EXIT;
    }
    
    *RxGain = ( val >> 5 ) & 0x07;

ERR_EXIT:

    return result;
}

int SX1278LoRaReadRssi(SX1278_st_ptr sx1278_ptr, int8_t *Rssi)
{
    int result = 0;
    uint8_t val = 0x00;

    if((NULL == sx1278_ptr)||(NULL == Rssi))
    {
        result = -ENOMEM;
        goto ERR_EXIT;
    }
    
    // Reads the RSSI value
    result = SX1278_Read_Reg(sx1278_ptr->spi_ptr, REG_LR_RSSIVALUE, &val);
    if(result < 0)
    {
        goto ERR_EXIT;
    }

    if( sx1278_ptr->cfg.rx_cfg.RFFrequency < 860000000)  // LF
    {
        *Rssi = RSSI_OFFSET_LF + val;
    }
    else
    {
        *Rssi = RSSI_OFFSET_HF + val;
    }

ERR_EXIT:

    return result;    
}
/*
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
*/
int SX1278LoRaSetRFFrequency(SX1278_st_ptr sx1278_ptr, uint32_t freq)
{
    int result = 0;
    uint8_t regval = 0;
    uint64_t temp = 0;

    if(NULL == sx1278_ptr)
    {
        result = -ENOMEM;
        goto ERR_EXIT;
    }
    
    temp = freq * 100000000;
    do_div(temp, FREQ_STEP);
    
    freq = ( uint32_t )temp;
    
    regval = ( uint8_t )( ( freq >> 16 ) & 0xFF );
    result = SX1278_Write_Reg(sx1278_ptr->spi_ptr, REG_LR_FRFMSB, regval);
    if(result < 0)
    {
        goto ERR_EXIT;
    }

    regval = ( uint8_t )( ( freq >> 8 ) & 0xFF );
    result = SX1278_Write_Reg(sx1278_ptr->spi_ptr, REG_LR_FRFMID, val);
    if(result < 0)
    {
        goto ERR_EXIT;
    }

    regval = ( uint8_t )( freq & 0xFF );
    result = SX1278_Write_Reg(sx1278_ptr->spi_ptr, REG_LR_FRFLSB, regval);
    if(result < 0)
    {
        goto ERR_EXIT;
    }

ERR_EXIT:

    return result;
}

int SX1278LoRaGetRFFrequency(SX1278_st_ptr sx1278_ptr, uint32_t *RFFrequency)
{
    int result = 0;
    uint8_t val = 0;
    uint8_t RegFrfMsb = 0;
    uint8_t RegFrfMid = 0;
    uint8_t RegFrfLsb = 0;
    uint32_t freq = 0;
    uint64_t temp = 0;

    if((NULL == sx1278_ptr) || (NULL == RFFrequency))
    {
        result = -ENOMEM;
        goto ERR_EXIT;
    }

    result = SX1278_Write_Reg(sx1278_ptr->spi_ptr, REG_LR_FRFMSB, RegFrfMsb);
    if(result < 0)
    {
        goto ERR_EXIT;
    }
    
    result = SX1278_Write_Reg(sx1278_ptr->spi_ptr, REG_LR_FRFMID, RegFrfMsb);
    if(result < 0)
    {
        goto ERR_EXIT;
    }
    
    result = SX1278_Write_Reg(sx1278_ptr->spi_ptr, REG_LR_FRFLSB, RegFrfMsb);
    if(result < 0)
    {
        goto ERR_EXIT;
    }

    temp = ((uint32_t)RegFrfMsb<<16)|((uint32_t)RegFrfMid<<8)|((uint32_t)RegFrfLsb); 
    temp *= FREQ_STEP;

    do_div(temp, 100000000);
    *freq = (uint32_t)temp;

ERR_EXIT:
    
    return result;
}

int SX1278LoRaSetRFPower(SX1278_st_ptr sx1278_ptr, int8_t power)
{
    int result = 0;
    uint8_t RegPaConfig = 0;
    uint8_t RegPaDac = 0;

    if(NULL == sx1278_ptr)
    {
        result = -ENOMEM;
        goto ERR_EXIT;
    }

    result = SX1278_Read_Reg(sx1278_ptr->spi_ptr, REG_LR_PACONFIG, &RegPaConfig);
    if(result < 0)
    {
        goto ERR_EXIT;
    }
    
    result = SX1278_Read_Reg(sx1278_ptr->spi_ptr, REG_LR_PADAC, &RegPaDac);
    if(result < 0)
    {
        goto ERR_EXIT;
    }
    
    if((RegPaConfig & RFLR_PACONFIG_PASELECT_PABOOST) == RFLR_PACONFIG_PASELECT_PABOOST)
    {
        if((RegPaDac & 0x87) == 0x87)
        {
            if( power < 5 )
            {
                power = 5;
            }
            if( power > 20 )
            {
                power = 20;
            }
            RegPaConfig = (RegPaConfig & RFLR_PACONFIG_MAX_POWER_MASK) | 0x70;
            RegPaConfig = (RegPaConfig & RFLR_PACONFIG_OUTPUTPOWER_MASK) | (uint8_t)((uint16_t)(power - 5) & 0x0F);
        }
        else
        {
            if(power < 2)
            {
                power = 2;
            }
            if(power > 17)
            {
                power = 17;
            }
            RegPaConfig = (RegPaConfig & RFLR_PACONFIG_MAX_POWER_MASK) | 0x70;
            RegPaConfig = (RegPaConfig & RFLR_PACONFIG_OUTPUTPOWER_MASK) | (uint8_t)((uint16_t)(power - 2) & 0x0F);
        }
    }
    else
    {
        if(power < -1)
        {
            power = -1;
        }
        if(power > 14)
        {
            power = 14;
        }
        RegPaConfig = (RegPaConfig & RFLR_PACONFIG_MAX_POWER_MASK) | 0x70;
        RegPaConfig = (RegPaConfig & RFLR_PACONFIG_OUTPUTPOWER_MASK) | (uint8_t)((uint16_t)(power + 1) & 0x0F);
    }
    
    result = SX1278_Write_Reg(sx1278_ptr->spi_ptr, REG_LR_PACONFIG, RegPaConfig);
    if(result < 0)
    {
        goto ERR_EXIT;
    }

ERR_EXIT:

    return result;
}

int SX1278LoRaGetRFPower(SX1278_st_ptr sx1278_ptr, int8_t *RFPower)
{
    int result = 0;
    int8_t power;
    uint8_t RegPaConfig = 0;
    uint8_t RegPaDac = 0;

    if((NULL == sx1278_ptr) || (NULL == RFPower))
    {
        result = -ENOMEM;
        goto ERR_EXIT;
    }

    result = SX1278_Read_Reg(sx1278_ptr->spi_ptr, REG_LR_PACONFIG, &RegPaConfig);
    if(result < 0)
    {
        goto ERR_EXIT;
    }
    
    result = SX1278_Read_Reg(sx1278_ptr->spi_ptr, REG_LR_PADAC, &RegPaDac);
    if(result < 0)
    {
        goto ERR_EXIT;
    }

    if((RegPaConfig & RFLR_PACONFIG_PASELECT_PABOOST) == RFLR_PACONFIG_PASELECT_PABOOST)
    {
        if((RegPaDac & 0x07) == 0x07)
        {
            *power = 5 + (RegPaConfig & ~RFLR_PACONFIG_OUTPUTPOWER_MASK);
        }
        else
        {
            *power = 2 + (RegPaConfig & ~RFLR_PACONFIG_OUTPUTPOWER_MASK);
        }
    }
    else
    {
        *power = -1 + (RegPaConfig & ~RFLR_PACONFIG_OUTPUTPOWER_MASK);
    }

ERR_EXIT:
    
    return result;
}

int SX1278LoRaSetSignalBandwidth(SX1278_st_ptr sx1278_ptr, uint8_t bw)
{
    int result = 0;
    uint8_t RegModemConfig1 = 0;

    if(NULL == sx1278_ptr)
    {
        result = -ENOMEM;
        goto ERR_EXIT;
    }
    
    result = SX1278_Read_Reg(sx1278_ptr->spi_ptr, REG_LR_MODEMCONFIG1, &RegModemConfig1);
    if(result < 0)
    {
        goto ERR_EXIT;
    }
    
    RegModemConfig1 = (RegModemConfig1&RFLR_MODEMCONFIG1_BW_MASK)|(bw<<4);
    result = SX1278_Write_Reg(sx1278_ptr->spi_ptr, REG_LR_MODEMCONFIG1, RegModemConfig1);
    if(result < 0)
    {
        goto ERR_EXIT;
    }

ERR_EXIT:

    return result;
}

int SX1278LoRaGetSignalBandwidth(SX1278_st_ptr sx1278_ptr, uint8_t *SignalBw)
{
    int result = 0;
    uint8_t SignalBw = 0;
    uint8_t RegModemConfig1 = 0;

    if((NULL == sx1278_ptr) || (NULL == SignalBw))
    {
        result = -ENOMEM;
        goto ERR_EXIT;
    }
    
    result = SX1278_Read_Reg(sx1278_ptr->spi_ptr, REG_LR_MODEMCONFIG1, &RegModemConfig1);
    if(result < 0)
    {
        goto ERR_EXIT;
    }
    
    *SignalBw = (RegModemConfig1&~RFLR_MODEMCONFIG1_BW_MASK)> 4;

ERR_EXIT:
    
    return result;
}

int SX1278LoRaSetSpreadingFactor(SX1278_st_ptr sx1278_ptr, uint8_t factor)
{
    int result = 0;
    uint8_t RegModemConfig2 = 0;

    if(NULL == sx1278_ptr)
    {
        result = -ENOMEM;
        goto ERR_EXIT;
    }
    
    if(factor > 12)
    {
        factor = 12;
    }
    else if(factor < 6)
    {
        factor = 6;
    }

    if(factor == 6)
    {
        result = SX1278LoRaSetNbTrigPeaks(sx1278_ptr, 5);
        if(result < 0)
        {
            goto ERR_EXIT;
        }
    }
    else
    {
        result = SX1278LoRaSetNbTrigPeaks(sx1278_ptr, 3);
        if(result < 0)
        {
            goto ERR_EXIT;
        }
    }

    result = SX1278_Read_Reg(sx1278_ptr->spi_ptr, REG_LR_MODEMCONFIG2, &RegModemConfig2);
    if(result < 0)
    {
        goto ERR_EXIT;
    }
    
    RegModemConfig2 = (RegModemConfig2&RFLR_MODEMCONFIG2_SF_MASK)|(factor<<4);
    result = SX1278_Write_Reg(sx1278_ptr->spi_ptr, REG_LR_MODEMCONFIG2, RegModemConfig2);
    if(result < 0)
    {
        goto ERR_EXIT;
    }

ERR_EXIT:

    return result;
}

int SX1278LoRaGetSpreadingFactor(SX1278_st_ptr sx1278_ptr, uint8_t *SpreadingFactor)
{
    int result = 0;
    uint8_t SpreadingFactor = 0;
    uint8_t RegModemConfig2 = 0;

    if((NULL == sx1278_ptr) || (NULL == SpreadingFactor))
    {
        result = -ENOMEM;
        goto ERR_EXIT;
    }

    result = SX1278_Read_Reg(sx1278_ptr->spi_ptr, REG_LR_MODEMCONFIG2, &RegModemConfig2 );
    if(result < 0)
    {
        goto ERR_EXIT;
    }
    
    *SpreadingFactor = (RegModemConfig2 & ~RFLR_MODEMCONFIG2_SF_MASK ) >> 4;

ERR_EXIT:
    
    return result;
}

int SX1278LoRaSetErrorCoding(SX1278_st_ptr sx1278_ptr, uint8_t value)
{
    int result = 0;
    uint8_t RegModemConfig1 = 0;

    if(NULL == sx1278_ptr)
    {
        result = -ENOMEM;
        goto ERR_EXIT;
    }
    
    result = SX1278_Read_Reg(sx1278_ptr->spi_ptr, REG_LR_MODEMCONFIG1, &RegModemConfig1);
    if(result < 0)
    {
        goto ERR_EXIT;
    }
    
    RegModemConfig1 = (RegModemConfig1&RFLR_MODEMCONFIG1_CODINGRATE_MASK)|(value<<1);
    result = SX1278_Write_Reg(sx1278_ptr->spi_ptr, REG_LR_MODEMCONFIG1, RegModemConfig1);
    if(result < 0)
    {
        goto ERR_EXIT;
    }

ERR_EXIT:

    return result;
}

int SX1278LoRaGetErrorCoding(SX1278_st_ptr sx1278_ptr, uint8_t *ErrorCoding)
{
    int result = 0;
    uint8_t ErrorCoding =0;
    uint8_t RegModemConfig1 = 0;

    if((NULL == sx1278_ptr) || (NULL == ErrorCoding))
    {
        result = -ENOMEM;
        goto ERR_EXIT;
    }
    
    result = SX1278_Read_Reg(sx1278_ptr->spi_ptr, REG_LR_MODEMCONFIG1, &RegModemConfig1);
    if(result < 0)
    {
        goto ERR_EXIT;
    }
    
    *ErrorCoding = (RegModemConfig1 & ~RFLR_MODEMCONFIG1_CODINGRATE_MASK) >> 1;

ERR_EXIT:
    
    return result;
}

int SX1278LoRaSetPacketCrcOn(SX1278_st_ptr sx1278_ptr, bool enable)
{
    int result = 0;
    uint8_t RegModemConfig2 = 0;

    if(NULL == sx1278_ptr)
    {
        result = -ENOMEM;
        goto ERR_EXIT;
    }
    
    result = SX1278_Read_Reg(sx1278_ptr->spi_ptr, REG_LR_MODEMCONFIG2, &RegModemConfig2 );
    if(result < 0)
    {
        goto ERR_EXIT;
    }

    RegModemConfig2 = (RegModemConfig2&RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK )|(enable<<2);
    result = SX1278_Write_Reg(sx1278_ptr->spi_ptr, REG_LR_MODEMCONFIG2, RegModemConfig2);
    if(result < 0)
    {
        goto ERR_EXIT;
    }

ERR_EXIT:

    return result; 
}

int SX1278LoRaSetPreambleLength(SX1278_st_ptr sx1278_ptr, uint16_t value)
{
    int result = 0;
    uint8_t RegPreambleMsb = 0;
    uint8_t RegPreambleLsb = 0;

    if(NULL == sx1278_ptr)
    {
        result = -ENOMEM;
        goto ERR_EXIT;
    }
    
    result = SX1278_Read_Reg(sx1278_ptr->spi_ptr, REG_LR_PREAMBLEMSB, &RegPreambleMsb);
    if(result < 0)
    {
        goto ERR_EXIT;
    }
    
    result = SX1278_Read_Reg(sx1278_ptr->spi_ptr, REG_LR_PREAMBLELSB, &RegPreambleLsb);
    if(result < 0)
    {
        goto ERR_EXIT;
    }

    RegPreambleMsb = ( value >> 8 ) & 0x00FF;
    RegPreambleLsb = value & 0xFF;
    
    result = SX1278_Write_Reg(sx1278_ptr->spi_ptr, REG_LR_PREAMBLEMSB, RegPreambleMsb);
    if(result < 0)
    {
        goto ERR_EXIT;
    }
    
    result = SX1278_Write_Reg(sx1278_ptr->spi_ptr, REG_LR_PREAMBLELSB, RegPreambleLsb);
    if(result < 0)
    {
        goto ERR_EXIT;
    }

ERR_EXIT:

    return result;
}

int SX1278LoRaGetPreambleLength(SX1278_st_ptr sx1278_ptr, uint16_t *PreambleLength)
{
    int result = 0;
    uint8_t RegPreambleMsb = 0;
    uint8_t RegPreambleLsb = 0;
    uint16_t PreambleLen = 0;

    if((NULL == sx1278_ptr) || (NULL == PreambleLength))
    {
        result = -ENOMEM;
        goto ERR_EXIT;
    }

    result = SX1278_Read_Reg(sx1278_ptr->spi_ptr, REG_LR_PREAMBLEMSB, &RegPreambleMsb);
    if(result < 0)
    {
        goto ERR_EXIT;
    }

    result = SX1278_Read_Reg(sx1278_ptr->spi_ptr, REG_LR_PREAMBLELSB, &RegPreambleLsb);
    if(result < 0)
    {
        goto ERR_EXIT;
    }

    *PreambleLen = ( ( RegPreambleMsb & 0x00FF ) << 8 ) | RegPreambleLsb;

ERR_EXIT:
    
    return result;
}

int SX1278LoRaGetPacketCrcOn(SX1278_st_ptr sx1278_ptr, bool *CrcOn)
{
    int result = 0;    
    bool CrcOn = false;
    uint8_t RegModemConfig2 = 0;

    if((NULL == sx1278_ptr) || (NULL == CrcOn))
    {
        result = -ENOMEM;
        goto ERR_EXIT;
    }
    
    result = SX1278_Read_Reg(sx1278_ptr->spi_ptr, REG_LR_MODEMCONFIG2, &RegModemConfig2);
    if(result < 0)
    {
        goto ERR_EXIT;
    }
        
    *CrcOn = (RegModemConfig2 & RFLR_MODEMCONFIG2_RXPAYLOADCRC_ON ) >> 1;

ERR_EXIT:
    
    return result;
}

int SX1278LoRaSetImplicitHeaderOn(SX1278_st_ptr sx1278_ptr, bool enable)
{
    int result = 0;
    uint8_t RegModemConfig1 = 0;

    if(NULL == sx1278_ptr)
    {
        result = -ENOMEM;
        goto ERR_EXIT;
    }
    
    result = SX1278_Read_Reg(sx1278_ptr->spi_ptr, REG_LR_MODEMCONFIG1, &RegModemConfig1);
    if(result < 0)
    {
        goto ERR_EXIT;
    }
    
    RegModemConfig1 = (RegModemConfig1&RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK)|(enable);
    result = SX1278_Write_Reg(sx1278_ptr->spi_tr, REG_LR_MODEMCONFIG1, RegModemConfig1);
    if(result < 0)
    {
        goto ERR_EXIT;
    }

ERR_EXIT:

    return result;
}

int SX1278LoRaGetImplicitHeaderOn(SX1278_st_ptr sx1278_ptr, bool *ImplicitHeaderOn)
{
    int result = 0;
    bool ImplicitHeaderOn = false;
    uint8_t RegModemConfig1 =0;

    if((NULL == sx1278_ptr) || (NULL == ImplicitHeaderOn))
    {
        result = -ENOMEM;
        goto ERR_EXIT;
    }

    result = SX1278_Read_Reg(sx1278_ptr->spi_ptr, REG_LR_MODEMCONFIG1, &RegModemConfig1);
    if(result < 0)
    {
        goto ERR_EXIT;
    }
    
    *ImplicitHeaderOn = (RegModemConfig1&RFLR_MODEMCONFIG1_IMPLICITHEADER_ON);

ERR_EXIT:
    
    return result;
}

void SX1278LoRaSetRxSingleOn(SX1278_st_ptr sx1278_ptr, bool enable)
{
    int result = 0;

    if(NULL == sx1278_ptr)
    {
        result = -ENOMEM;
        goto ERR_EXIT;
    }

    //LoRaSettings.RxSingleOn = enable;
ERR_EXIT:
    
    return result;    
}

int SX1278LoRaGetRxSingleOn(SX1278_st_ptr sx1278_ptr, bool *RxSingleOn)
{
    int result = 0;

    if((NULL == sx1278_ptr) || (NULL == RxSingleOn))
    {
        result = -ENOMEM;
        goto ERR_EXIT;
    }

    *RxSingleOn = true;
    
ERR_EXIT:
    
    return result;    
}

int SX1278LoRaSetFreqHopOn(SX1278_st_ptr sx1278_ptr, bool enable)
{
    int result = 0;

    if(NULL == sx1278_ptr)
    {
        result = -ENOMEM;
        goto ERR_EXIT;
    }

    //LoRaSettings.FreqHopOn = enable;
    
ERR_EXIT:
    
    return result;     
}

int SX1278LoRaGetFreqHopOn(SX1278_st_ptr sx1278_ptr, bool *FreqHopOn)
{
    int result = 0;

    if((NULL == sx1278_ptr) || (NULL == FreqHopOn))
    {
        result = -ENOMEM;
        goto ERR_EXIT;
    }

    //return LoRaSettings.FreqHopOn;
    *FreqHopOn = true;
ERR_EXIT:

    return result;
}

int SX1278LoRaSetHopPeriod(SX1278_st_ptr sx1278_ptr, uint8_t value)
{
    int result = 0;

    if(NULL == sx1278_ptr)
    {
        result = -ENOMEM;
        goto ERR_EXIT;
    }

    result = SX1278_Write_Reg(sx1278_ptr->spi_ptr, REG_LR_HOPPERIOD, value);
    if(result < 0)
    {
        goto ERR_EXIT;
    }
    
ERR_EXIT:

    return result;    
}

int SX1278LoRaGetHopPeriod(SX1278_st_ptr sx1278_ptr, uint8_t *HopPeriod)
{
    int result = 0;
    uint8_t RegHopPeriod =0;
    uint8_t HopPeriod = 0;

    if((NULL == sx1278_ptr) || (NULL == HopPeriod))
    {
        result = -ENOMEM;
        goto ERR_EXIT;
    }

    result = SX1278_Read_Reg(sx1278_ptr->spi_ptr, REG_LR_HOPPERIOD, &RegHopPeriod);
    if(result < 0)
    {
        goto ERR_EXIT;
    }

    *HopPeriod = RegHopPeriod;

ERR_EXIT:
    
    return result;
}

int SX1278LoRaSetTxPacketTimeout(SX1278_st_ptr sx1278_ptr, uint32_t value)
{
    int result = 0;
    //LoRaSettings.TxPacketTimeout = value;
    if(NULL == sx1278_ptr)
    {
        result = -ENOMEM;
        goto ERR_EXIT;
    }

ERR_EXIT:
    
    return result;
}

int SX1278LoRaGetTxPacketTimeout(SX1278_st_ptr sx1278_ptr, uint32_t *TxPacketTime)
{
    int result = 0;

    if((NULL == sx1278_ptr) || (NULL == TxPacketTime))
    {
        result = -ENOMEM;
        goto ERR_EXIT;
    }

    //return LoRaSettings.TxPacketTimeout;
    *TxPacketTime = 0;
    
ERR_EXIT:
    
    return result;
}

int SX1278LoRaSetRxPacketTimeout(SX1278_st_ptr sx1278_ptr, uint32_t value)
{
    int result = 0;
    //LoRaSettings.RxPacketTimeout = value;
    if(NULL == sx1278_ptr)
    {
        result = -ENOMEM;
        goto ERR_EXIT;
    }

ERR_EXIT:
    
    return result;    
}

int SX1278LoRaGetRxPacketTimeout(SX1278_st_ptr sx1278_ptr, uint32_t *RxPacketTime)
{
    int result = 0;

    if((NULL == sx1278_ptr) || (NULL == RxPacketTime))
    {
        result = -ENOMEM;
        goto ERR_EXIT;
    }

    //return LoRaSettings.RxPacketTimeout;
    *RxPacketTime = 0;
    
ERR_EXIT:
    
    return result;

}

int SX1278LoRaSetPayloadLength(SX1278_st_ptr sx1278_tr, uint8_t value)
{
    int result = 0;

    if(NULL == sx1278_tr)
    {
        result = -ENOMEM;
        goto ERR_EXIT;
    }
    
    result = SX1278_Write_Reg(sx1278_tr->spi_ptr, REG_LR_PAYLOADLENGTH, value);
    if(result < 0)
    {
        goto ERR_EXIT;
    }

ERR_EXIT:

    return result;
}

int SX1278LoRaGetPayloadLength(SX1278_st_ptr sx1278_tr, uint8_t *RegPayloadLength)
{
    int result = 0;
    uint8_t RegPayloadLength = 0;

    if((NULL == sx1278_tr) || (NULL == RegPayloadLength))
    {
        result = -ENOMEM;
        goto ERR_EXIT;
    }

    
    result = SX1278_Read_Reg(sx1278_tr->spi_ptr, REG_LR_PAYLOADLENGTH, &RegPayloadLength);
    if(result < 0)
    {
        goto ERR_EXIT;
    }

ERR_EXIT:

    return result;
}

int SX1278LoRaSetPa20dBm(SX1278_st_ptr sx1278_ptr, bool enale)
{
    int result = 0;
    uint8_t RegPaDac = 0;
    uint8_t RegPaConfig = 0;

    if(NULL == sx1278_ptr)
    {
        result = -ENOMEM;
        goto ERR_EXIT;
    }
    
    result = SX1278_Read_Reg(sx1278_ptr->spi_ptr, REG_LR_PADAC, &RegPaDac);
    if(result < 0)
    {
        goto ERR_EXIT;
    }
    
    result = SX1278_Read_Reg(sx1278_ptr->spi_ptr, REG_LR_PACONFIG, &RegPaConfig);
    if(result < 0)
    {
        goto ERR_EXIT;
    }    

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
    
    result = SX1278_Write_Reg(sx1278_ptr->spi_ptr, REG_LR_PADAC, RegPaDac);
    if(result < 0)
    {
        goto ERR_EXIT;
    }

ERR_EXIT:

    return result;
}

int SX1278LoRaGetPa20dBm(SX1278_st_ptr sx1278_ptr, bool *Pa20dBm)
{
    int result = 0;
    bool Pa20dBm = false;
    uint8_t RegPaDac = 0;

    if((NULL == sx1278_ptr) || (NULL == Pa20dBm))
    {
        result = -ENOMEM;
        goto ERR_EXIT;
    }

    result = SX1278_Read_Reg(sx1278_ptr->spi_ptr, REG_LR_PADAC, &RegPaDac);
    if(result < 0)
    {
        goto ERR_EXIT;
    }

    *Pa20dBm = ( ( RegPaDac & 0x07 ) == 0x07 ) ? true : false;

ERR_EXIT:
    
    return result;
}

void SX1278LoRaSetPAOutput(SX1278_st_ptr sx1278_ptr, uint8_t outputPin)
{
    int result = 0;
    uint8_t RegPaConfig = 0;

    if(NULL == sx1278_ptr)
    {
        result = -ENOMEM;
        goto ERR_EXIT;
    }
        
    result = SX1278_Read_Reg(sx1278_ptr->spi_ptr, REG_LR_PACONFIG, &RegPaConfig);
    if(result < 0)
    {
        goto ERR_EXIT;
    }
    
    RegPaConfig = (RegPaConfig&RFLR_PACONFIG_PASELECT_MASK)|outputPin;
    SX1278_Write_Reg(sx1278_ptr->spi_ptr, REG_LR_PACONFIG, RegPaConfig);
    if(result < 0)
    {
        goto ERR_EXIT;
    }

ERR_EXIT:

    return result;
}

int SX1278LoRaGetPAOutput(SX1278_st_ptr sx1278_ptr, uint8_t *PAOutput)
{
    int result = 0;
    uint8_t RegPaConfig = 0;

    if((NULL == sx1278_ptr) || (NULL == PAOutput))
    {
        result = -ENOMEM;
        goto ERR_EXIT;
    }

    result = SX1278_Read_Reg(sx1278_ptr->spi_ptr, REG_LR_PACONFIG, &RegPaConfig);
    if(result < 0)
    {
        goto ERR_EXIT;
    }

    *PAOutput = RegPaConfig & ~RFLR_PACONFIG_PASELECT_MASK;

ERR_EXIT:
    
    return result;
}

int SX1278LoRaSetPaRamp(SX1278_st_ptr sx1278_ptr, uint8_t value)
{
    int result = 0;
    uint8_t RegPaRamp = 0;

    if(NULL == sx1278_ptr)
    {
        result = -ENOMEM;
        goto ERR_EXIT;
    }
    
    result = SX1278_Read_Reg(sx1278_ptr->spi_ptr, REG_LR_PARAMP, &RegPaRamp);
    if(result < 0)
    {
        goto ERR_EXIT;
    }
    
    RegPaRamp = (RegPaRamp & RFLR_PARAMP_MASK ) | ( value & ~RFLR_PARAMP_MASK);
    result = SX1278_Write_Reg(sx1278_ptr->spi_ptr, REG_LR_PARAMP, RegPaRamp);
    if(result < 0)
    {
        goto ERR_EXIT;
    }

ERR_EXIT:

    return result;
}

int SX1278LoRaGetPaRamp(SX1278_st_ptr sx1278_ptr, uint8_t *PaRamp)
{
    int result = 0;
    uint8_t RegPaRamp = 0;

    if((NULL == sx1278_ptr) || (NULL == PaRamp))
    {
        result = -ENOMEM;
        goto ERR_EXIT;
    }
    
    result = SX1278_Read_Reg(sx1278_ptr->spi_ptr, REG_LR_PARAMP, &RegPaRamp);
    if(result < 0)
    {
        goto ERR_EXIT;
    }

    *PaRamp = RegPaRamp & ~RFLR_PARAMP_MASK;

ERR_EXIT:
    
    return result;
}

int SX1278LoRaSetSymbTimeout(SX1278_st_ptr sx1278_ptr, uint16_t value)
{
    int result = 0;
    uint8_t RegModemConfig2 = 0;
    uint8_t RegSymbTimeoutLsb = 0;

    if(NULL == sx1278_ptr)
    {
        result = -ENOMEM;
        goto ERR_EXIT;
    }
    
    result = SX1278_Read_Reg(sx1278_ptr->spi_ptr, REG_LR_MODEMCONFIG2, &RegModemConfig2);
    if(result < 0)
    {
        goto ERR_EXIT;
    }
    
    result = SX1278_Read_Reg(sx1278_ptr->spi_ptr, REG_LR_SYMBTIMEOUTLSB, &RegModemConfig2);
    if(result < 0)
    {
        goto ERR_EXIT;
    }

    RegModemConfig2 = (RegModemConfig2&RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK)|((value>>8)&~RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK);
    RegSymbTimeoutLsb = value&0xFF;

    result = SX1278_Write_Reg(sx1278_ptr->spi_ptr, REG_LR_MODEMCONFIG2, RegModemConfig2);
    if(result < 0)
    {
        goto ERR_EXIT;
    }
    
    result = SX1278_Write_Reg(sx1278_ptr->spi_ptr, REG_LR_SYMBTIMEOUTLSB, RegSymbTimeoutLsb);
    if(result < 0)
    {
        goto ERR_EXIT;
    }

ERR_EXIT:

    return result;
}

uint16_t SX1278LoRaGetSymbTimeout(SX1278_st_ptr sx1278_ptr, uint16_t *SymbTimeout)
{
    int result = 0;
    uint8_t RegModemConfig2 = 0;
    uint8_t RegSymbTimeoutLsb = 0;
    uint16_t SymbTimeout = 0;

    if((NULL == sx1278_ptr) || (NULL == SymbTimeout))
    {
        result = -ENOMEM;
        goto ERR_EXIT;
    }

    result = SX1278_Read_Reg(sx1278_ptr->spi_ptr, REG_LR_MODEMCONFIG2, &RegModemConfig2);
    if(result < 0)
    {
        goto ERR_EXIT;
    }

    result = SX1278_Read_Reg(sx1278_ptr->spi_ptr, REG_LR_SYMBTIMEOUTLSB, &RegModemConfig2);
    if(result < 0)
    {
        goto ERR_EXIT;
    }

    *SymbTimeout = ((RegModemConfig2&~RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK)<<8)|RegSymbTimeoutLsb;

ERR_EXIT:
    
    return result;
}

int SX1278LoRaSetLowDatarateOptimize(SX1278_st_ptr sx1278_ptr, bool enable)
{
    int result = 0;
    uint8_t RegModemConfig3 =0;

    if(NULL == sx1278_ptr)
    {
        result = -ENOMEM;
        goto ERR_EXIT;
    }
    
    result= SX1278_Read_Reg(sx1278_ptr->spi_ptr, REG_LR_MODEMCONFIG3, &RegModemConfig3);
    if(result < 0)
    {
        goto ERR_EXIT;
    }
    
    RegModemConfig3 = (RegModemConfig3&RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_MASK)|(enable<<3);
    result= SX1278_Write_Reg(sx1278_ptr->spi_ptr, REG_LR_MODEMCONFIG3, RegModemConfig3);
    if(result < 0)
    {
        goto ERR_EXIT;
    }

ERR_EXIT:

    return result;
}

int SX1278LoRaGetLowDatarateOptimize(SX1278_st_ptr sx1278_ptr, bool *LowDatarateOptimize)
{
    int result = 0;
    uint8_t RegModemConfig3 = 0;
    if((NULL == sx1278_ptr) || (NULL == LowDatarateOptimize))
    {
        result = -ENOMEM;
        goto ERR_EXIT;
    }
    
    result = SX1278_Read_Reg(sx1278_ptr->spi_ptr, REG_LR_MODEMCONFIG3, &RegModemConfig3);
    if(result < 0)
    {
        goto ERR_EXIT;
    }

    *LowDatarateOptimize = ((RegModemConfig3&RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_ON)>>3) ? true : false;

ERR_EXIT:
    
    return result;
}

int SX1278LoRaSetNbTrigPeaks(SX1278_st_ptr sx1278_ptr, uint8_t value)
{
    int result = 0;
    uint8_t RegDetectOptimize = 0;

    if(NULL == sx1278_ptr)
    {
        result = -ENOMEM;
        goto ERR_EXIT;
    }
    
    result = SX1278_Read_Reg(sx1278_ptr->spi_ptr, 0x31, &RegDetectOptimize);
    if(result < 0)
    {
        goto ERR_EXIT;
    }
    
    RegDetectOptimize = (RegDetectOptimize & 0xF8)|value;
    result = SX1278_Write_Reg(sx1278_ptr->spi_ptr, 0x31, RegDetectOptimize);
    if(result < 0)
    {
        goto ERR_EXIT;
    }

ERR_EXIT:

    return result;
}

int SX1278LoRaGetNbTrigPeaks(SX1278_st_ptr sx1278_ptr, uint8_t *NbTrigPeaks)
{
    int result= 0;
    uint8_t RegDetectOptimize = 0;

    if((NULL == sx1278_ptr) || (NULL == NbTrigPeaks))
    {
        result = -ENOMEM;
        goto ERR_EXIT;
    }

    result = SX1278_Read_Reg(sx1278_ptr->spi_ptr, 0x31, &RegDetectOptimize);
    if(result < 0)
    {
        goto ERR_EXIT;
    }

    *NbTrigPeaks = (RegDetectOptimize & 0x07 );
    
ERR_EXIT:
    
    return result;
}


int SX1278LoRaStartRx(SX1278_st_ptr sx1278_ptr)
{
    int result= 0;

    if(NULL == sx1278_ptr)
    {
        result = -ENOMEM;
        goto ERR_EXIT;
    }

    //SX1278LoRaSetRFState( RFLR_STATE_RX_INIT );
    
ERR_EXIT:
    
    return result;    
}

int SX1278LoRaGetRxPacket(SX1278_st_ptr sx1278_ptr, void *buffer, uint16_t *size)
{
    int result= 0;

    if((NULL == sx1278_ptr) || (NULL == buffer) || (NULL == size))
    {
        result = -ENOMEM;
        goto ERR_EXIT;
    }

    //*size = RxPacketSize;
    //RxPacketSize = 0;
    //memcpy( ( void * )buffer, ( void * )RFBuffer, ( size_t )*size );
    
ERR_EXIT:
    
    return result;    
}

int SX1278LoRaSetTxPacket(SX1278_st_ptr sx1278_ptr, const void *buffer, uint16_t size)
{
    int result= 0;

    if((NULL == sx1278_ptr) || (NULL == buffer))
    {
        result = -ENOMEM;
        goto ERR_EXIT;
    }

    //TxPacketSize = size;
    //memcpy( ( void * )RFBuffer, buffer, ( size_t )TxPacketSize); 
    //
    //RFLRState = RFLR_STATE_TX_INIT;
    
ERR_EXIT:
    
    return result;    
}

int SX1278LoRaGetRFState(SX1278_st_ptr sx1278_ptr, uint8_t *RFState)
{
    int result= 0;

    if((NULL == sx1278_ptr) || (NULL == RFState))
    {
        result = -ENOMEM;
        goto ERR_EXIT;
    }

    //SX1278LoRaSetRFState( RFLR_STATE_RX_INIT );
    
ERR_EXIT:
    
    return result;    
}

int SX1278LoRaSetRFState(SX1278_st_ptr sx1278_ptr, uint8_t state)
{    
    int result= 0;

    if(NULL == sx1278_ptr)
    {
        result = -ENOMEM;
        goto ERR_EXIT;
    }

    //RFLRState = state;
    
ERR_EXIT:
    
    return result;    
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
int SX1278Init(SX1278_st_ptr sx1278_ptr)
{
    int result = 0;
    
    // Initialize LoRa registers structure
    result = SX1278SetLoRaOn(sx1278_ptr, true);
    if(result < 0)
    {
        goto ERR_EXIT;
    }
    
    // Initialize LoRa modem
    result = SX1278LoRaInit(sx1278_ptr);
    if(result < 0)
    {
        goto ERR_EXIT;
    }

ERR_EXIT:

    return result;
}

void SX1278Reset(SX1278_st_ptr sx1278_ptr)
{
    //SX1278SetReset( RADIO_RESET_ON );
    
    // Wait 1ms
    mdelay(1);    

    //SX1278SetReset( RADIO_RESET_OFF );
    
    // Wait 6ms
    mdelay(6);
}

int SX1278SetLoRaOn(SX1278_st_ptr sx1278_ptr, bool enable)
{
    int result = 0;
    uint8_t RegDioMapping = 0;
    uint8_t LoRaOnState = 0;
    uint8_t RegOpMode = 0;

    if(NULL == sx1278_ptr)
    {
        result = -ENOMEM;
        goto ERR_EXIT;
    }
    
    result = SX1278LoRaGetModem(sx1278_ptr, &LoRaOnState);
    if(result < 0)
    {
        goto ERR_EXIT;
    }

    if(LoRaOnState == enable)
    {
        goto ERR_EXIT;
    }
     
    if(enable)
    {
        result = SX1278LoRaSetOpMode(sx1278_ptr, RFLR_OPMODE_SLEEP);
        if(result < 0)
        {
            goto ERR_EXIT;
        }

        result = SX1278_Read_Reg(sx1278_ptr->spi_ptr, REG_LR_OPMODE, &RegOpMode);
        if(result < 0)
        {
            goto ERR_EXIT;
        }
        
        RegOpMode = ( RegOpMode & RFLR_OPMODE_LONGRANGEMODE_MASK ) | RFLR_OPMODE_LONGRANGEMODE_ON;

        result = SX1278_Write_Reg(sx1278_ptr->spi_ptr, REG_LR_OPMODE, RegOpMode);
        
        result = SX1278LoRaSetOpMode(sx1278_ptr, RFLR_OPMODE_STANDBY);
        if(result < 0)
        {
            goto ERR_EXIT;
        }

                                  // RxDone               RxTimeout                   FhssChangeChannel           CadDone
        RegDioMapping = RFLR_DIOMAPPING1_DIO0_00 | RFLR_DIOMAPPING1_DIO1_00 | RFLR_DIOMAPPING1_DIO2_00 | RFLR_DIOMAPPING1_DIO3_00;
        result = SX1278_Write_Reg(sx1278_ptr->spi_ptr, REG_LR_DIOMAPPING1, RegDioMapping);
        if(result < 0)
        {
            goto ERR_EXIT;
        }
                                 // CadDetected          ModeReady
        RegDioMapping = RFLR_DIOMAPPING2_DIO4_00 | RFLR_DIOMAPPING2_DIO5_00;
        result = SX1278_Write_Reg(sx1278_ptr->spi_ptr, REG_LR_DIOMAPPING2, RegDioMapping);
        if(result < 0)
        {
            goto ERR_EXIT;
        }
    }
    else
    {
        result = SX1278LoRaSetOpMode(sx1278_ptr, RFLR_OPMODE_SLEEP);
        if(result < 0)
        {
            goto ERR_EXIT;
        }

        result = SX1278_Read_Reg(sx1278_ptr->spi_ptr, REG_LR_OPMODE, &RegOpMode);
        if(result < 0)
        {
            goto ERR_EXIT;
        }
        
        RegOpMode = ( RegOpMode & RFLR_OPMODE_LONGRANGEMODE_MASK ) | RFLR_OPMODE_LONGRANGEMODE_OFF;
        result = SX1278_Write_Reg(sx1278_ptr->spi_ptr, REG_LR_OPMODE, RegOpMode);
        if(result < 0)
        {
            goto ERR_EXIT;
        }

        result = SX1278LoRaSetOpMode(sx1278_ptr, RFLR_OPMODE_STANDBY);
        if(result < 0)
        {
            goto ERR_EXIT;
        }
    }

ERR_EXIT:

    return result;
}

int SX1278GetLoRaOn(SX1278_st_ptr sx1278_ptr, bool *LoRaOn)
{
    int result = 0;

    if((NULL == sx1278_ptr) || (NULL == LoRaOn))
    {
        result = -ENOMEM;
        goto ERR_EXIT;
    }
    
ERR_EXIT:    
    
    return result;
    //return LoRaOn;
}

int SX1278SetOpMode(SX1278_st_ptr sx1278_ptr, uint8_t opMode )
{
    return SX1278LoRaSetOpMode(sx1278_ptr, opMode);        
}

int SX1278GetOpMode(SX1278_st_ptr sx1278_ptr, uint8_t *OpMode)
{
    return SX1278LoRaGetOpMode(sx1278_ptr, OpMode);       
}

int SX1278ReadRssi(SX1278_st_ptr sx1278_ptr, int8_t *rssi)
{
    return SX1278LoRaReadRssi(sx1278_ptr, rssi);
}

int SX1278ReadRxGain(SX1278_st_ptr sx1278_ptr, uint8_t *RxGain)
{
    return SX1278LoRaReadRxGain(sx1278_ptr, RxGain);
}

int SX1278GetPacketRxGain(SX1278_st_ptr sx1278_ptr, uint8_t *PacketRxGain)
{
    return SX1278LoRaGetPacketRxGain(sx1278_ptr, PacketRxGain);
}

int SX1278GetPacketSnr(SX1278_st_ptr sx1278_ptr, int8_t *PacketSnr)
{
    return SX1278LoRaGetPacketSnr(sx1278_ptr, PacketSnr);
}

int SX1278GetPacketRssi(SX1278_st_ptr sx1278_ptr, int8_t *PacketRssi)
{
    return SX1278LoRaGetPacketRssi(sx1278_ptr, PacketRssi);
}

int SX1278GetPacketAfc(SX1278_st_ptr sx1278_ptr, uint32_t *PacketAfc)
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

void SX1278StartRx(SX1278_st_ptr sx1278_ptr)
{
    int result = 0;
    uint8_t RegHopPeriod = 0;
    uint8_t RegHopChannel = 0;
    uint8_t RegIrqFlagsMask = 0;
    uint8_t RegDioMapping1 = 0;
    uint8_t RegDioMapping2 = 0;
    uint8_t RegFifoAddrPtr = 0;
    uint8_t RegFifoRxBaseAddr = 0;

    if(NULL == sx1278_ptr)
    {
        result = -ENOMEM;
        goto ERR_EXIT;
    }
    
    result = SX1278LoRaSetOpMode(sx1278_ptr, RFLR_OPMODE_STANDBY);
    if(result < 0)
    {
        goto ERR_EXIT;
    }

    RegIrqFlagsMask = RFLR_IRQFLAGS_RXTIMEOUT |
                        //RFLR_IRQFLAGS_RXDONE |
                        //RFLR_IRQFLAGS_PAYLOADCRCERROR |
                        RFLR_IRQFLAGS_VALIDHEADER |
                        RFLR_IRQFLAGS_TXDONE |
                        RFLR_IRQFLAGS_CADDONE |
                        //RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                        RFLR_IRQFLAGS_CADDETECTED;
    result = SX1278_Write_Reg(sx1278_ptr, REG_LR_IRQFLAGSMASK, RegIrqFlagsMask);
    if(result < 0)
    {
        goto ERR_EXIT;
    }

    if(sx1278_ptr->cfg.rx_cfg.FreqHopOn == true)
    {
        RegHopPeriod = sx1278_ptr->cfg.rx_cfg.HopPeriod;

        result = SX1278_Read_Reg(sx1278_ptr, REG_LR_HOPCHANNEL, &RegHopChannel);
        if(result < 0)
        {
            goto ERR_EXIT;
        }
        
        result = SX1278LoRaSetRFFrequency(sx1278_ptr, HoppingFrequencies[RegHopChannel & RFLR_HOPCHANNEL_CHANNEL_MASK]);
        if(result < 0)
        {
            goto ERR_EXIT;
        }        
    }
    else
    {
        RegHopPeriod = 255;
    }
    
    result = SX1278_Write_Reg(sx1278_ptr, REG_LR_HOPPERIOD, RegHopPeriod);
    if(result < 0)
    {
        goto ERR_EXIT;
    }              
                                // RxDone                    RxTimeout                   FhssChangeChannel           CadDone
    RegDioMapping1 = RFLR_DIOMAPPING1_DIO0_00 | RFLR_DIOMAPPING1_DIO1_00 | RFLR_DIOMAPPING1_DIO2_00 | RFLR_DIOMAPPING1_DIO3_00;
                                // CadDetected               ModeReady
    RegDioMapping2 = RFLR_DIOMAPPING2_DIO4_00 | RFLR_DIOMAPPING2_DIO5_00;

    result = SX1278_Write_Reg(sx1278_ptr, REG_LR_DIOMAPPING1, RegDioMapping1);
    if(result < 0)
    {
        goto ERR_EXIT;
    }

    result = SX1278_Write_Reg(sx1278_ptr, REG_LR_DIOMAPPING2, RegDioMapping2);
    if(result < 0)
    {
        goto ERR_EXIT;
    }

    if(sx1278_ptr->cfg.rx_cfg.RxSingleOn == true) // Rx single mode
    {
        result = SX1278LoRaSetOpMode(sx1278_ptr, RFLR_OPMODE_RECEIVER_SINGLE);
        if(result < 0)
        {
            goto ERR_EXIT;
        }        
    }
    else // Rx continuous mode
    {
        //problem
        RegFifoAddrPtr = RegFifoRxBaseAddr;
        result = SX1278_Write_Reg(sx1278_ptr, REG_LR_FIFOADDRPTR, RegFifoAddrPtr);
        if(result < 0)
        {
            goto ERR_EXIT;
        }
        
        result = SX1278LoRaSetOpMode(sx1278_ptr, RFLR_OPMODE_RECEIVER );
        if(result < 0)
        {
            goto ERR_EXIT;
        }        
    }

    //memset( RFBuffer, 0, ( size_t )RF_BUFFER_SIZE );
    //PacketTimeout = LoRaSettings.RxPacketTimeout;
    //RxTimeoutTimer = GET_TICK_COUNT( );
    //RFLRState = RFLR_STATE_RX_RUNNING;
ERR_EXIT:

    return result;
}


int SX1278GetRxPacket(SX1278_st_ptr *sx1278_ptr, uint8_t *buffer_ptr, uint8_t buffer_len)
{
    int result = 0;
    uint8_t RegIrqFlags = 0;
    uint8_t RegPktRssiValue = 0;
    uint8_t RegFifoAddrPtr = 0;
    uint8_t RegFifoRxBaseAddr = 0;
    //SX1278LoRaGetRxPacket( buffer, size );
    
    result = SX1278_Read_Reg(sx1278_ptr, REG_LR_IRQFLAGS, &RegIrqFlags);
    if(result < 0)
    {
        goto ERR_EXIT;
    }
    
    if((RegIrqFlags & RFLR_IRQFLAGS_PAYLOADCRCERROR) == RFLR_IRQFLAGS_PAYLOADCRCERROR)
    {
        // Clear Irq
        result = SX1278_Write_Reg(sx1278_ptr, REG_LR_IRQFLAGS, RFLR_IRQFLAGS_PAYLOADCRCERROR );
        if(result < 0)
        {
            goto ERR_EXIT;
        }
        
        if( sx1278_ptr->cfg.rx_cfg.RxSingleOn == true ) // Rx single mode
        {
            //RFLRState = RFLR_STATE_RX_INIT;
            result = SX1278StartRx(sx1278_ptr);
            if(result < 0)
            {
                goto ERR_EXIT;
            }
        }
        else
        {
            //RFLRState = RFLR_STATE_RX_RUNNING;
        }

        return result;
    }
    
    {
        uint8_t rxSnrEstimate;
        result = SX1278_Read_Reg(sx1278_ptr, REG_LR_PKTSNRVALUE, &rxSnrEstimate);
        if(result < 0)
        {
            goto ERR_EXIT;
        }
        
        if(rxSnrEstimate & 0x80) // The SNR sign bit is 1
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
    
    result = SX1278_Read_Reg(sx1278_ptr, REG_LR_PKTRSSIVALUE, &RegPktRssiValue);
    if(result < 0)
    {
        goto ERR_EXIT;
    }

    if(sx1278_ptr->cfg.rx_cfg.RFFrequency < 860000000)  // LF
    {    
        if(RxPacketSnrEstimate < 0)
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
        if(RxPacketSnrEstimate < 0)
        {
            RxPacketRssiValue = RSSI_OFFSET_HF + RegPktRssiValue + RxPacketSnrEstimate;
        }
        else
        {    
            RxPacketRssiValue = RSSI_OFFSET_HF + (uint8_t)((uint32_t)10666 * RegPktRssiValue / 10000);
        }
    }

    if(sx1278_ptr->cfg.rx_cfg.RxSingleOn == true) // Rx single mode
    {
        RegFifoAddrPtr = RegFifoRxBaseAddr;
        result = SX1278_Write_Reg(sx1278_ptr, REG_LR_FIFOADDRPTR, RegFifoAddrPtr);
        if(result < 0)
        {
            goto ERR_EXIT;
        }
        
        if(sx1278_ptr->cfg.rx_cfg.ImplicitHeaderOn == true)
        {
            result = SX1278_Read_FIFO(sx1278_ptr, buffer_ptr, buffer_len);
            if(result < 0)
            {
                goto ERR_EXIT;
            }
        }
        else
        {
            uint8_t RegNbRxBytes = 0;
            result = SX1278_Read_Reg(sx1278_ptr, REG_LR_RXNBBYTES, &RegNbRxBytes);
            if(result < 0)
            {
                goto ERR_EXIT;
            }
            
            //RxPacketSize = SX1276LR->RegNbRxBytes;
            result = SX1278_Read_FIFO(sx1278_ptr, buffer_ptr, RegNbRxBytes);
            if(result < 0)
            {
                goto ERR_EXIT;
            }
            
            result = RegNbRxBytes;
        }
    }
    else // Rx continuous mode
    {
        uint8_t RegFifoRxCurrentAddr = 0;
        result = SX1278_Read_Reg(sx1278_ptr, REG_LR_FIFORXCURRENTADDR, &RegFifoRxCurrentAddr);
        if(result < 0)
        {
            goto ERR_EXIT;
        }

        if(sx1278_ptr->cfg.rx_cfg.ImplicitHeaderOn == true)
        {            
            RegFifoAddrPtr = RegFifoRxCurrentAddr;
            result = SX1278_Write_Reg(sx1278_ptr, REG_LR_FIFOADDRPTR, RegFifoAddrPtr);
            if(result < 0)
            {
                goto ERR_EXIT;
            }
            
            result = SX1278_Read_FIFO(sx1278_ptr, buffer_ptr, buffer_len);
            if(result < 0)
            {
                goto ERR_EXIT;
            }            
        }
        else
        {
            uint8_t RegNbRxBytes = 0;
            result = SX1278_Read_Reg(sx1278_ptr, REG_LR_RXNBBYTES, &RegNbRxBytes);
            if(result < 0)
            {
                goto ERR_EXIT;
            }
            
            //RxPacketSize = SX1276LR->RegNbRxBytes;
            RegFifoAddrPtr = RegFifoRxCurrentAddr;
            result = SX1278_Write_Reg(sx1278_ptr, REG_LR_FIFOADDRPTR, RegFifoAddrPtr );
            if(result < 0)
            {
                goto ERR_EXIT;
            }
            
            result = SX1278_Read_FIFO(sx1278_ptr, buffer_ptr, RegNbRxBytes );
            if(result < 0)
            {
                goto ERR_EXIT;
            }            
        }
    }
    
    if( sx1278_ptr->cfg.rx_cfg.RxSingleOn == true ) // Rx single mode
    {
        //RFLRState = RFLR_STATE_RX_INIT;
    }
    else // Rx continuous mode
    {
        //RFLRState = RFLR_STATE_RX_RUNNING;
    }
}

int SX1278RxFinished(SX1278_st_ptr sx1278_ptr)
{
    int result = 0;
    uint8_t RegHopChannel = 0;

    if(NULL == sx1278_ptr)
    {
        result = -ENOMEM;
        goto ERR_EXIT;
    }
    
    if(sx1278_ptr->cfg.rx_cfg.FreqHopOn == true)
    {
        result = SX1278_Read_Reg(sx1278_ptr, REG_LR_HOPCHANNEL, &RegHopChannel);
        if(result < 0)
        {
            goto ERR_EXIT;
        }
        
        result = SX1278LoRaSetRFFrequency(sx1278_ptr, HoppingFrequencies[RegHopChannel & RFLR_HOPCHANNEL_CHANNEL_MASK]);
        if(result < 0)
        {
            goto ERR_EXIT;
        }    
    }
    // Clear Irq
    //SX1278_Write_Reg( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXDONE);
ERR_EXIT:

    return result;
}

int SX1278RxClearIrq(SX1278_st_ptr sx1278_ptr)
{
    int result = 0;
    uint8_t RegHopChannel = 0;
    
    if(NULL == sx1278_ptr)
    {
        result = -ENOMEM;
        goto ERR_EXIT;
    }
    
    if( sx1278_ptr->cfg.rx_cfg.FreqHopOn == true)
    {
        result = SX1278_Read_Reg(sx1278_ptr, REG_LR_HOPCHANNEL, &RegHopChannel);
        if(result < 0)
        {
            goto ERR_EXIT;
        }
        
        result = SX1278LoRaSetRFFrequency(sx1278_ptr, HoppingFrequencies[RegHopChannel & RFLR_HOPCHANNEL_CHANNEL_MASK]);
        if(result < 0)
        {
            goto ERR_EXIT;
        }         
    }

    result = SX1278_Write_Reg(sx1278_ptr, REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXDONE);
    if(result < 0)
    {
        goto ERR_EXIT;
    }
        
ERR_EXIT:

    return result;
}

int SX1278StartTx(SX1278_st_ptr sx1278_ptr)
{
    int result = 0;
    uint8_t reg_val = 0;
    uint8_t RegDioMapping1 = 0;
    uint8_t RegDioMapping2 = 0;

    if(NULL == sx1278_ptr)
    {
        result = -ENOMEM;
        goto ERR_EXIT;
    }    
                              // TxDone               RxTimeout                   FhssChangeChannel          ValidHeader  
    RegDioMapping1 = RFLR_DIOMAPPING1_DIO0_01 | RFLR_DIOMAPPING1_DIO1_00 | RFLR_DIOMAPPING1_DIO2_00 | RFLR_DIOMAPPING1_DIO3_01;
                              // PllLock              Mode Ready
    RegDioMapping2 = RFLR_DIOMAPPING2_DIO4_01 | RFLR_DIOMAPPING2_DIO5_00;
    result = SX1278_Write_Reg(sx1278_ptr, REG_LR_DIOMAPPING1, RegDioMapping1);
    if(result < 0)
    {
        goto ERR_EXIT;
    }
    
    result = SX1278_Write_Reg(sx1278_ptr, REG_LR_DIOMAPPING2, RegDioMapping2);
    if(result < 0)
    {
        goto ERR_EXIT;
    }

    result = SX1278LoRaSetOpMode(sx1278_ptr, RFLR_OPMODE_TRANSMITTER );
    if(result < 0)
    {
        goto ERR_EXIT;
    }

ERR_EXIT:

    return result;
}

int SX1278SetTxPacket(SX1278_st_ptr sx1278_ptr, const uint8_t *buffer_ptr, uint8_t buffer_len )
{
    int result = 0;
    uint8_t RegPayloadLength = 0;
    uint8_t RegFifoTxBaseAddr = 0;
    uint8_t RegFifoAddrPtr = 0;
    uint8_t RegIrqFlagsMask = 0;
    uint8_t RegHopPeriod = 0;
    uint8_t RegHopChannel = 0;

    if((NULL == sx1278_ptr) || (NULL == buffer_ptr))
    {
        result = -ENOMEM;
        goto ERR_EXIT;
    }
    //SX1278LoRaSetTxPacket(sx1278_ptr, buffer, size);
    
    result = SX1278LoRaSetOpMode(sx1278_ptr, RFLR_OPMODE_STANDBY);
    if(result < 0)
    {
        goto ERR_EXIT;
    }

    if(sx1278_ptr->cfg.tx_cfg.FreqHopOn == true)
    {
        RegIrqFlagsMask = RFLR_IRQFLAGS_RXTIMEOUT |
                            RFLR_IRQFLAGS_RXDONE |
                            RFLR_IRQFLAGS_PAYLOADCRCERROR |
                            RFLR_IRQFLAGS_VALIDHEADER |
                            //RFLR_IRQFLAGS_TXDONE |
                            RFLR_IRQFLAGS_CADDONE |
                            //RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                            RFLR_IRQFLAGS_CADDETECTED;

        RegHopPeriod = sx1278_ptr->cfg.tx_cfg.HopPeriod;

        result = SX1278_Read_Reg(sx1278_ptr, REG_LR_HOPCHANNEL, &RegHopChannel);
        if(result < 0)
        {
            goto ERR_EXIT;
        }
        
        result = SX1278LoRaSetRFFrequency(sx1278_ptr, HoppingFrequencies[RegHopChannel & RFLR_HOPCHANNEL_CHANNEL_MASK]);
        if(result < 0)
        {
            goto ERR_EXIT;
        }        
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
    result = SX1278_Write_Reg(sx1278_ptr, REG_LR_HOPPERIOD, RegHopPeriod);
    if(result < 0)
    {
        goto ERR_EXIT;
    }
    
    result = SX1278_Write_Reg(sx1278_ptr, REG_LR_IRQFLAGSMASK, RegIrqFlagsMask);
    if(result < 0)
    {
        goto ERR_EXIT;
    }    
    // Initializes the payload size
    RegPayloadLength = buffer_len;
    result = SX1278_Write_Reg(sx1278_ptr, REG_LR_PAYLOADLENGTH, RegPayloadLength);
    if(result < 0)
    {
        goto ERR_EXIT;
    }
    
    RegFifoTxBaseAddr = 0x00; // Full buffer used for Tx
    result = SX1278_Write_Reg(sx1278_ptr, REG_LR_FIFOTXBASEADDR, RegFifoTxBaseAddr);
    if(result < 0)
    {
        goto ERR_EXIT;
    }    

    RegFifoAddrPtr = RegFifoTxBaseAddr;
    result = SX1278_Write_Reg(sx1278_ptr, REG_LR_FIFOADDRPTR, RegFifoAddrPtr);
    if(result < 0)
    {
        goto ERR_EXIT;
    }
    
    // Write payload buffer to LORA modem
    result = SX1278_Write_FIFO(sx1278_ptr, buffer_ptr, RegPayloadLength);       
    if(result < 0)
    {
        goto ERR_EXIT;
    }

ERR_EXIT:
    
    return result;
}

int SX1278TxFinished(SX1278_st_ptr sx1278_ptr)
{
    int result = 0;

    if(NULL == sx1278_ptr)
    {
        result = -ENOMEM;
        goto ERR_EXIT;
    }
    
    result = SX1278_Write_Reg(sx1278_ptr, REG_LR_IRQFLAGS, RFLR_IRQFLAGS_TXDONE);
    if(result < 0)
    {
        goto ERR_EXIT;
    }
    
    result = SX1278LoRaSetOpMode(sx1278_ptr, RFLR_OPMODE_STANDBY);
    if(result < 0)
    {
        goto ERR_EXIT;
    }

ERR_EXIT:

    return result;
}

int SX1278FhssChangedChannel(SX1278_st_ptr sx1278_ptr)
{
    int result = 0;
    uint8_t RegHopChannel = 0;

    if(NULL == sx1278_ptr)
    {
        result = -ENOMEM;
        goto ERR_EXIT;
    }
    //problem
    if( sx1278_cfg_ptr->rx_cfg.FreqHopOn == true )
    {
        result = SX1278_Read_Reg(sx1278_ptr, REG_LR_HOPCHANNEL, &RegHopChannel);
        if(result < 0)
        {
            goto ERR_EXIT;
        }
        
        result = SX1278LoRaSetRFFrequency(sx1278_ptr, HoppingFrequencies[RegHopChannel & RFLR_HOPCHANNEL_CHANNEL_MASK]);
        if(result < 0)
        {
            goto ERR_EXIT;
        }
    }
    // Clear Irq
    result = SX1278_Write_Reg(sx1278_ptr, REG_LR_IRQFLAGS, RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL);
    if(result < 0)
    {
        goto ERR_EXIT;
    }    
    // Debug
    result = SX1278LoRaReadRxGain(sx1278_ptr, &RxGain);
    if(result < 0)
    {
        goto ERR_EXIT;
    }
   
ERR_EXIT:

    return result;
}

int SX1278StartCad(SX1278_st_ptr sx1278_ptr)
{
    int result = 0;
    uint8_t RegIrqFlagsMask = 0;
    uint8_t RegDioMapping1 = 0;
    uint8_t RegDioMapping2 = 0;

    if(NULL == sx1278_ptr)
    {
        result = -ENOMEM;
        goto ERR_EXIT;
    }
    
    result = SX1278LoRaSetOpMode(sx1278_ptr, RFLR_OPMODE_STANDBY);
    if(result < 0)
    {
        goto ERR_EXIT;
    }

    RegIrqFlagsMask = RFLR_IRQFLAGS_RXTIMEOUT |
                        RFLR_IRQFLAGS_RXDONE |
                        RFLR_IRQFLAGS_PAYLOADCRCERROR |
                        RFLR_IRQFLAGS_VALIDHEADER |
                        RFLR_IRQFLAGS_TXDONE |
                        //RFLR_IRQFLAGS_CADDONE |
                        RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL; // |
                        //RFLR_IRQFLAGS_CADDETECTED;
    result = SX1278_Write_Reg(sx1278_ptr, REG_LR_IRQFLAGSMASK, RegIrqFlagsMask);
    if(result < 0)
    {
        goto ERR_EXIT;
    }
                                // RxDone                   RxTimeout                   FhssChangeChannel           CadDone
    RegDioMapping1 = RFLR_DIOMAPPING1_DIO0_00 | RFLR_DIOMAPPING1_DIO1_00 | RFLR_DIOMAPPING1_DIO2_00 | RFLR_DIOMAPPING1_DIO3_00;
                                // CAD Detected              ModeReady
    RegDioMapping2 = RFLR_DIOMAPPING2_DIO4_00 | RFLR_DIOMAPPING2_DIO5_00;

    result = SX1278_Write_Reg(sx1278_ptr, REG_LR_DIOMAPPING1, RegDioMapping1);
    if(result < 0)
    {
        goto ERR_EXIT;
    }
    
    result = SX1278_Write_Reg(sx1278_ptr, REG_LR_DIOMAPPING1, RegDioMapping2);
    if(result < 0)
    {
        goto ERR_EXIT;
    }

    result = SX1278LoRaSetOpMode(sx1278_ptr, RFLR_OPMODE_CAD);
    if(result < 0)
    {
        goto ERR_EXIT;
    }

ERR_EXIT:

    return result;
}

int SX1278CadFinished(SX1278_st_ptr sx1278_ptr)
{
    int result = 0;

    if(NULL == sx1278_ptr)
    {
        result = -ENOMEM;
        goto ERR_EXIT;
    }
    
    result = SX1278_Write_Reg(sx1278_ptr, REG_LR_IRQFLAGS, RFLR_IRQFLAGS_CADDONE);
    if(result < 0)
    {
        goto ERR_EXIT;
    }
    //if( DIO4 == 1 ) // CAD Detected interrupt
    {
        // Clear Irq
        result = SX1278_Write_Reg(sx1278_ptr, REG_LR_IRQFLAGS, RFLR_IRQFLAGS_CADDETECTED);
        if(result < 0)
        {
            goto ERR_EXIT;
        }        
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

ERR_EXIT:

    return result;
}
int SX1278GetRFState(SX1278_st_ptr sx1278_ptr, uint8_t *RFState)
{
    return SX1278LoRaGetRFState(sx1278_ptr, RFState);
}

int SX1278SetRFState(SX1278_st_ptr sx1278_ptr, uint8_t state)
{
    SX1278LoRaSetRFState(sx1278_ptr, state);
}

int SX1278Process(SX1278_st_ptr sx1278_ptr)
{
    //return SX1278LoRaProcess( );
    return 0;
}


