#include <linux/init.h>
#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/poll.h>
#include <linux/of_irq.h>
#include <linux/of_device.h>
#include <linux/spi/spi.h>
#include <linux/gpio/consumer.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/kfifo.h>
#include <linux/wait.h>
#include <linux/uaccess.h>

#include "sx127x.h"
#include "sx127xlib.h"

#define DRV_NAME        "lora"
#define DRIVER_VER1		0					//
#define DRIVER_VER2		1					//
#define DRV_MAJOR		104 				//
#define DRV_MINOR	 	1					//

//debug macro
#undef	DEBUG
//#define DEBUG
#ifdef DEBUG
#define	DPRINTK( x... )		printk(DRV_NAME":" x)
#else
#define DPRINTK( x... )
#endif



#define SX127X_DRIVERNAME	"sx127x"
#define SX127X_CLASSNAME	"sx127x"
#define SX127X_DEVICENAME	"sx127x%d"

#define SX127X_LORAREG_MASK BIT(8)
#define SX127X_LORAREG(addr) (addr | SX127X_LORAREG_MASK)
#define SX127X_FSKOOKREG_MASK BIT(9)
#define SX127X_FSKOOKREG(addr) (addr | SX127X_FSKOOK_MASK)

#define SX127X_REGADDR(reg) (reg & 0x7f)
#define SX127X_WRITEADDR(addr) (addr | (1 << 7))

#define SX127X_REG_FIFO													0x00

#define SX127X_REG_OPMODE												0x01
#define SX127X_REG_OPMODE_LONGRANGEMODE									BIT(7)
#define SX127X_REG_OPMODE_FSKOOK_MODULATIONTYPE							(BIT(6) | BIT(5))
#define SX127X_REG_OPMODE_FSKOOK_MODULATIONTYPE_FSK						0
#define SX127X_REG_OPMODE_FSKOOK_MODULATIONTYPE_OOK						BIT(5)
#define SX127X_REG_OPMODE_LOWFREQUENCYMODEON							BIT(3)
#define SX127X_REG_OPMODE_MODE											(BIT(2) | BIT(1) | BIT(0))
#define SX127X_REG_OPMODE_MODE_SLEEPMODE								0
#define SX127X_REG_OPMODE_MODE_STDBYMODE								BIT(0)

#define SX127X_REG_FSKOOK_BITRATEMSB									SX127X_FSKOOKREG(0x02)
#define SX127X_REG_FSKOOK_BITRATELSB									SX127X_FSKOOKREG(0x03)
#define SX127X_REG_FSKOOK_FDEVMSB										SX127X_FSKOOKREG(0x04)
#define SX127X_REG_FSKOOK_FDEVLSB										SX127X_FSKOOKREG(0x05)
#define SX127X_REG_FRFMSB												0x06
#define SX127X_REG_FRFMLD												0x07
#define SX127X_REG_FRFLSB												0x08

#define SX127X_REG_PACONFIG												0x09
#define SX127X_REG_PACONFIG_PASELECT									BIT(7)
#define SX127X_REG_PACONFIG_MAXPOWER									(BIT(6) | BIT(5) | BIT(4))
#define SX127X_REG_PACONFIG_MAXPOWER_SHIFT								4
#define SX127X_REG_PACONFIG_OUTPUTPOWER									(BIT(3) | BIT(2) | BIT(1) | BIT(0))

#define SX127X_REG_PARAMP												0x0a
#define SX127X_REG_OCP													0x0b
#define SX127X_REG_LNA													0x0c
#define SX127X_REG_FSKOOK_RXCONFIG										SX127X_FSKOOKREG(0x0d)
#define SX127X_REG_LORA_FIFOADDRPTR										SX127X_LORAREG(0x0d)
#define SX127X_REG_FSKOOK_RSSICONFIG									SX127X_FSKOOKREG(0x0e)
#define SX127X_REG_LORA_FIFOTXBASEADDR									SX127X_LORAREG(0x0e)
#define SX127X_REG_FSKOOK_RSSICOLLISION									SX127X_FSKOOKREG(0x0f)
#define SX127X_REG_LORA_RXBASEADDR										SX127X_LORAREG(0x0f)
#define SX127X_REG_FSKOOK_RSSTHRESH										SX127X_FSKOOKREG(0x10)
#define SX127X_REG_LORA_RXCURRENTADDR									SX127X_LORAREG(0x10)
#define SX127X_REG_FSKOOK_RSSIVALUE										SX127X_FSKOOKREG(0x11)
#define SX127X_REG_LORA_IRQMASKFLAGS									SX127X_LORAREG(0x11)
#define SX127X_REG_FSKOOK_RXBW											SX127X_FSKOOKREG(0x12)

#define SX127X_REG_LORA_IRQFLAGS										SX127X_LORAREG(0x12)
#define SX127X_REG_LORA_IRQFLAGS_RXTIMEOUT								BIT(7)
#define SX127X_REG_LORA_IRQFLAGS_RXDONE									BIT(6)
#define SX127X_REG_LORA_IRQFLAGS_PAYLOADCRCERROR						BIT(5)
#define SX127X_REG_LORA_IRQFLAGS_TXDONE									BIT(3)
#define SX127X_REG_LORA_IRQFLAGS_CADDONE								BIT(2)
#define SX127X_REG_LORA_IRQFLAGS_CADDETECTED							BIT(0)

#define SX127X_REG_FSKOOK_AFCBW											SX127X_FSKOOKREG(0x13)
#define SX127X_REG_LORA_RXNBBYTES										SX127X_LORAREG(0x13)
#define SX127X_REG_FSKOOK_OOKPEAK										SX127X_FSKOOKREG(0x14)
#define SX127X_REG_LORA_RXHEADERCNTVALUEMSB								SX127X_LORAREG(0x14)
#define SX127X_REG_FSKOOK_OOKFIX										SX127X_FSKOOKREG(0x15)
#define SX127X_REG_LORA_RXHEADERCNTVALUELSB								SX127X_LORAREG(0x15)
#define SX127X_REG_FSKOOK_OOKAVG										SX127X_FSKOOKREG(0x16)
#define SX127X_REG_LORA_RXPACKETCNTVALUEMSB								SX127X_LORAREG(0x16)
#define SX127X_REG_LORA_RXPACKETCNTVALUELSB								SX127X_LORAREG(0x17)
#define SX127X_REG_LORA_MODEMSTAT										SX127X_LORAREG(0x18)
#define SX127X_REG_LORA_PKTSNRVALUE										SX127X_LORAREG(0x19)
#define SX127X_REG_FSKOOK_AFCFEI										SX127X_FSKOOKREG(0x1a)
#define SX127X_REG_LORA_PKTRSSIVALUE									SX127X_LORAREG(0x1a)
#define SX127X_REG_FSKOOK_AFCMSB										SX127X_FSKOOKREG(0x1b)
#define SX127X_REG_LORA_RSSIVALUE										SX127X_LORAREG(0x1b)
#define SX127X_REG_FSKOOK_AFCLSB										SX127X_FSKOOKREG(0x1c)
#define SX127X_REG_LORA_HOPCHANNEL										SX127X_LORAREG(0x1c)
#define SX127X_REG_FSKOOK_FEIMSB										SX127X_FSKOOKREG(0x1d)

#define SX127X_REG_LORA_MODEMCONFIG1									SX127X_LORAREG(0x1d)
#define SX127X_REG_LORA_MODEMCONFIG1_BW									(BIT(7) | BIT(6) | BIT(5) | BIT(4))
#define SX127X_REG_LORA_MODEMCONFIG1_BW_SHIFT							4
#define SX127X_REG_LORA_MODEMCONFIG1_BW_MAX								9
#define SX127X_REG_LORA_MODEMCONFIG1_CODINGRATE							(BIT(3) | BIT(2) | BIT(1))
#define SX127X_REG_LORA_MODEMCONFIG1_CODINGRATE_SHIFT					1
#define SX127X_REG_LORA_MODEMCONFIG1_CODINGRATE_MIN						1
#define SX127X_REG_LORA_MODEMCONFIG1_CODINGRATE_MAX						6
#define SX127X_REG_LORA_MODEMCONFIG1_IMPLICITHEADERMODEON				BIT(0)

#define SX127X_REG_FSKOOK_FEILSB										SX127X_FSKOOKREG(0x1e)

#define SX127X_REG_LORA_MODEMCONFIG2									SX127X_LORAREG(0x1e)
#define SX127X_REG_LORA_MODEMCONFIG2_SPREADINGFACTOR					(BIT(7) | BIT(6) | BIT(5) | BIT(4))
#define SX127X_REG_LORA_MODEMCONFIG2_SPREADINGFACTOR_SHIFT				4
#define SX127X_REG_LORA_MODEMCONFIG2_RXPAYLOADCRCON						BIT(2)

#define SX127X_REG_FSKOOK_PREAMBLEDETECT								SX127X_FSKOOKREG(0x1f)
#define SX127X_REG_LORA_SYMBTIMEOUTLSB									SX127X_LORAREG(0x1f)
#define SX127X_REG_FSKOOK_RXTIMEOUT1									SX127X_FSKOOKREG(0x20)
#define SX127X_REG_LORA_PREAMBLEMSB										SX127X_LORAREG(0x20)
#define SX127X_REG_FSKOOK_RXTIMEOUT2									SX127X_FSKOOKREG(0x21)
#define SX127X_REG_LORA_PREAMBLELSB										SX127X_LORAREG(0x21)
#define SX127X_REG_FSKOOK_RXTIMEOUT3									SX127X_FSKOOKREG(0x22)
#define SX127X_REG_LORA_PAYLOADLENGTH									SX127X_LORAREG(0x22)
#define SX127X_REG_FSKOOK_RXDELAY										SX127X_FSKOOKREG(0x23)
#define SX127X_REG_LORA_MAXPAYLOADLENGTH								SX127X_LORAREG(0x23)
#define SX127X_REG_FSKOOK_OSC											SX127X_FSKOOKREG(0x24)
#define SX127X_REG_LORA_HOPPERIOD										SX127X_LORAREG(0x24)
#define SX127X_REG_FSKOOK_PREAMBLEMSB									SX127X_FSKOOKREG(0x25)
#define SX127X_REG_LORA_FIFORXBYTEADDR									SX127X_LORAREG(0x25)
#define SX127X_REG_FSKOOK_PREAMBLELSB									SX127X_FSKOOKREG(0x26)

#define SX127X_REG_LORA_MODEMCONFIG3									SX127X_LORAREG(0x26)
#define SX127X_REG_LORA_MODEMCONFIG3_LOWDATARATEOPTIMIZE				BIT(3)

#define SX127X_REG_FSKOOK_SYNCCONFIG									SX127X_FSKOOKREG(0x27)
#define SX127X_REG_FSKOOK_SYNCVALUE1									SX127X_FSKOOKREG(0x28)
#define SX127X_REG_LORA_FEIMSB											SX127X_LORAREG(0x28)
#define SX127X_REG_FSKOOK_SYNCVALUE2									SX127X_FSKOOKREG(0x29)
#define SX127X_REG_LORA_FEIMID											SX127X_LORAREG(0x29)
#define SX127X_REG_FSKOOK_SYNCVALUE3									SX127X_FSKOOKREG(0x2a)
#define SX127X_REG_LORA_FEILSB											SX127X_LORAREG(0x29)
#define SX127X_REG_FSKOOK_SYNCVALUE4									SX127X_FSKOOKREG(0x2b)
#define SX127X_REG_FSKOOK_SYNCVALUE5									SX127X_FSKOOKREG(0x2c)
#define SX127X_REG_LORA_RSSIWIDEBAND									SX127X_LORAREG(0x2c)
#define SX127X_REG_FSKOOK_SYNCVALUE6									SX127X_FSKOOKREG(0x2d)
#define SX127X_REG_FSKOOK_SYNCVALUE7									SX127X_FSKOOKREG(0x2e)
#define SX127X_REG_FSKOOK_SYNCVALUE8									SX127X_FSKOOKREG(0x2f)
#define SX127X_REG_FSKOOK_PACKETCONFIG1									SX127X_FSKOOKREG(0x30)
#define SX127X_REG_FSKOOK_PACKETCONFIG2									SX127X_FSKOOKREG(0x31)

#define SX127X_REG_LORA_DETECTOPTIMIZATION								SX127X_LORAREG(0x31)
#define SX127X_REG_LORA_DETECTOPTIMIZATION_DETECTIONOPTIMIZE			(BIT(2) | BIT(1) | BIT(0))
#define SX127X_REG_LORA_DETECTOPTIMIZATION_DETECTIONOPTIMIZE_SF7SF12	0x03
#define SX127X_REG_LORA_DETECTOPTIMIZATION_DETECTIONOPTIMIZE_SF6		0x05

#define SX127X_REG_FSKOOK_PAYLOADLENGTH									SX127X_FSKOOKREG(0x32)
#define SX127X_REG_FSKOOK_NODEADRS										SX127X_FSKOOKREG(0x33)
#define SX127X_REG_LORA_INVERTIQ										SX127X_LORAREG(0x33)
#define SX127X_REG_LORA_INVERTIQ_INVERTIQ								BIT(6)

#define SX127X_REG_FSKOOK_BROADCASTADRS									SX127X_FSKOOKREG(0x34)
#define SX127X_REG_FSKOOK_FIFOTHRESH									SX127X_FSKOOKREG(0x35)
#define SX127X_REG_FSKOOK_SEQCONFIG1									SX127X_FSKOOKREG(0x36)
#define SX127X_REG_FSKOOK_SEQCONFIG2									SX127X_FSKOOKREG(0x37)
#define SX127X_REG_LORA_DETECTIONTHRESHOLD								SX127X_LORAREG(0x37)
#define SX127X_REG_FSKOOK_TIMERRESOL									SX127X_FSKOOKREG(0x38)
#define SX127X_REG_FSKOOK_TIMER1COEF									SX127X_FSKOOKREG(0x39)
#define SX127X_REG_LORA_SYNCWORD										SX127X_LORAREG(0x39)
#define SX127X_REG_FSKOOK_TIMER2COEF									SX127X_FSKOOKREG(0x3a)
#define SX127X_REG_FSKOOK_IMAGECAL										SX127X_FSKOOKREG(0x3b)
#define SX127X_REG_FSKOOK_TEMP											SX127X_FSKOOKREG(0x3c)
#define SX127X_REG_FSKOOK_LOWBAT										SX127X_FSKOOKREG(0x3d)
#define SX127X_REG_FSKOOK_IRQFLAGS1										SX127X_FSKOOKREG(0x3e)
#define SX127X_REG_FSKOOK_IRQFLAGS2										SX127X_FSKOOKREG(0x3f)

#define SX127X_REG_DIOMAPPING1											0x40
#define SX127X_REG_DIOMAPPING1_DIO0										(BIT(7) | BIT(6))
#define SX127X_REG_DIOMAPPING1_DIO0_RXDONE								0
#define SX127X_REG_DIOMAPPING1_DIO0_TXDONE								(BIT(6))
#define SX127X_REG_DIOMAPPING1_DIO0_CADDONE								(BIT(7))

#define SX127X_REG_DIOMAPPING2											0x41
#define SX127X_REG_VERSION												0x42
#define SX127X_REG_FSKOOK_PLLHOP										SX127X_FSKOOKREG(0x44)
#define SX127X_REG_TCXO													0x4b
#define SX127X_REG_PADAC												0x4d
#define SX127X_REG_FORMERTEMP											0x5b
#define SX127X_REG_FSKOOK_BITRATEFRAC									SX127X_FSKOOKREG(0x5d)
#define SX127X_REG_AGCREF												0x61
#define SX127X_REG_AGCTHRESH1											0x62
#define SX127X_REG_AGCTHRESH2											0x63
#define SX127X_REG_AGCTHRESH3											0x64
#define SX127X_REG_PLL													0x70

static int devmajor;
static struct class *devclass;
static dev_t drv_dev_num = MKDEV(DRV_MAJOR, 0);
static struct cdev drv_cdev;
static struct class *drv_class;


static const char* invalid = "invalid";
static const char* modstr[] = {"fsk", "ook", "lora"};
static const char* opmodestr[] = {"sleep", "standby", "fstx", "tx", "fsrx", "rx", "rxcontinuous", "rxsingle", "cad"};
static unsigned bwmap[] = { 7800, 10400, 15600, 20800, 31250, 41700, 62500, 125000, 250000, 500000 };
static const char* paoutput[] = {"rfo", "pa_boost"};


static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

static int sx127x_reg_read(struct spi_device *spi, u16 reg, u8* result){
	u8 addr = reg & 0xff;
	int ret = spi_write_then_read(spi,
			&addr, 1,
			result, 1);
	dev_dbg(&spi->dev, "read: @%02x %02x\n", addr, *result);
	return ret;
}

static int sx127x_reg_read16(struct spi_device *spi, u16 reg, u16* result){
	u8 addr = reg & 0xff;
	int ret = spi_write_then_read(spi,
			&addr, 1,
			result, 2);
	dev_dbg(&spi->dev, "read: @%02x %02x\n", addr, *result);
	return ret;
}

static int sx127x_reg_read24(struct spi_device *spi, u16 reg, u32* result){
	u8 addr = reg & 0xff, buf[3];
	int ret = spi_write_then_read(spi,
			&addr, 1,
			buf, 3);
	*result = (buf[0] << 16) | (buf[1] << 8) | buf[0];
	dev_dbg(&spi->dev, "read: @%02x %06x\n", addr, *result);
	return ret;
}

static int sx127x_reg_write(struct spi_device *spi, u16 reg, u8 value){
	u8 addr = SX127X_REGADDR(reg), buff[2];//, readback;
	int ret;
	buff[0] = SX127X_WRITEADDR(addr);
	buff[1] = value;
	dev_dbg(&spi->dev, "write: @%02x %02x\n", addr, value);
	ret = spi_write(spi, buff, 2);
//	ret = sx127x_reg_read(spi, reg, &readback);
//	if(readback != value){
//		dev_warn(&spi->dev, "read back does not match\n");
//	}
	return ret;
}

static int sx127x_reg_write24(struct spi_device *spi, u16 reg, u32 value){
	u8 addr = SX127X_REGADDR(reg), buff[4];
	int ret;
	buff[0] = SX127X_WRITEADDR(addr);
	buff[1] = (value >> 16) & 0xff;
	buff[2] = (value >> 8) & 0xff;
	buff[3] = value & 0xff;
	dev_dbg(&spi->dev, "write: @%02x %06x\n", addr, value);
	ret = spi_write(spi, buff, sizeof(buff));
	return ret;
}

static int sx127x_fifo_readpkt(struct spi_device *spi, void *buffer, u8 *len){
	u8 addr = SX127X_REG_FIFO, pktstart, rxbytes, off, fifoaddr;
	size_t maxtransfer = 0xFFFFFFFF;//spi_max_transfer_size(spi);
	int ret;
	unsigned readlen;
	ret = sx127x_reg_read(spi, SX127X_REG_LORA_RXCURRENTADDR, &pktstart);
	ret = sx127x_reg_read(spi, SX127X_REG_LORA_RXNBBYTES, &rxbytes);
	for(off = 0; off < rxbytes; off += maxtransfer){
		readlen = min(maxtransfer, (size_t) (rxbytes - off));
		fifoaddr = pktstart + off;
		ret = sx127x_reg_write(spi, SX127X_REG_LORA_FIFOADDRPTR, fifoaddr);
		if(ret){
			break;
		}
		dev_warn(&spi->dev, "fifo read: %02x from %02x\n", readlen, fifoaddr);
		ret = spi_write_then_read(spi,
				&addr, 1,
				buffer + off, readlen);
		if(ret){
			break;
		}

	}
	print_hex_dump_bytes("", DUMP_PREFIX_NONE, buffer, rxbytes);
	*len = rxbytes;
	return ret;
}

static int sx127x_fifo_writepkt(struct spi_device *spi, void *buffer, u8 len){
	u8 addr = SX127X_WRITEADDR(SX127X_REGADDR(SX127X_REG_FIFO));
	int ret;
	struct spi_transfer fifotransfers[] = {
			{.tx_buf = &addr, .len = 1},
			{.tx_buf = buffer, .len = len},
	};

	u8 readbackaddr = SX127X_REGADDR(SX127X_REG_FIFO);
	u8 readbackbuff[256];
	struct spi_transfer readbacktransfers[] = {
			{.tx_buf = &readbackaddr, .len = 1},
			{.rx_buf = &readbackbuff, .len = len},
	};

	ret = sx127x_reg_write(spi, SX127X_REG_LORA_FIFOTXBASEADDR, 0);
	ret = sx127x_reg_write(spi, SX127X_REG_LORA_FIFOADDRPTR, 0);
	ret = sx127x_reg_write(spi, SX127X_REG_LORA_PAYLOADLENGTH, len);

	dev_info(&spi->dev, "fifo write: %d\n", len);
	print_hex_dump(KERN_DEBUG, NULL, DUMP_PREFIX_NONE, 16, 1, buffer, len, true);
	spi_sync_transfer(spi, fifotransfers, ARRAY_SIZE(fifotransfers));

	ret = sx127x_reg_write(spi, SX127X_REG_LORA_FIFOADDRPTR, 0);
	spi_sync_transfer(spi, readbacktransfers, ARRAY_SIZE(readbacktransfers));
	if(memcmp(buffer, readbackbuff, len) != 0){
		dev_err(&spi->dev, "fifo readback doesn't match\n");
	}
	return ret;
}

int SX127XGetModem(struct sx127x *data, enum sx127x_modulation *modulation)
{
    int ret = 0;
    u8 opmode;
    
    ret = sx127x_reg_read(data->spidevice, SX127X_REG_OPMODE, &opmode);
    if(ret < 0)
    {
        goto error;
    }
    
    if(opmode & SX127X_REG_OPMODE_LONGRANGEMODE) {
		*modulation = SX127X_MODULATION_LORA;
	}
	else {
		opmode = opmode & SX127X_REG_OPMODE_FSKOOK_MODULATIONTYPE;
		if(opmode == SX127X_REG_OPMODE_FSKOOK_MODULATIONTYPE_FSK) {
			*modulation = SX127X_MODULATION_FSK;
		}
		else if(opmode == SX127X_REG_OPMODE_FSKOOK_MODULATIONTYPE_OOK) {
			*modulation = SX127X_MODULATION_OOK;
		}else{
            *modulation = SX127X_MODULATION_INVALID;
        }
	}

error:

    return ret;
}
static enum sx127x_modulation sx127x_getmodulation(struct sx127x *data) {
	int ret;
    enum sx127x_modulation modem;
    
    ret = SX127XGetModem(data, &modem);
    if(ret < 0)
    {
       goto error;
    }

    return modem;
    
error:
    
	return SX127X_MODULATION_INVALID;
}

static int sx127x_indexofstring(const char* str, const char** options, unsigned noptions){
	int i;
	for (i = 0; i < noptions; i++) {
		if (sysfs_streq(str, options[i])) {
			return i;
		}
	}
	return -1;
}

static ssize_t sx127x_modulation_show(struct device *child, struct device_attribute *attr, char *buf)
{
	struct sx127x *data = dev_get_drvdata(child);
	u8 opmode;
	enum sx127x_modulation mod;
	int ret;
	mutex_lock(&data->mutex);
	
	mod = sx127x_getmodulation(data);
	if(mod == SX127X_MODULATION_INVALID){
		ret = sprintf(buf, "%s\n", invalid);
	}
	else{
		ret = sprintf(buf, "%s\n", modstr[mod]);
	}
	mutex_unlock(&data->mutex);
	return ret;

}

static int SX127xSetModem(struct sx127x *data, enum sx127x_modulation modulation){
	u8 opmode;
	sx127x_reg_read(data->spidevice, SX127X_REG_OPMODE, &opmode);

	// LoRa mode bit can only be changed in sleep mode
	if(opmode & SX127X_REG_OPMODE_MODE){
		dev_warn(data->chardevice, "switching to sleep mode before changing modulation\n");
		opmode &= ~SX127X_REG_OPMODE_MODE;
		sx127x_reg_write(data->spidevice, SX127X_REG_OPMODE, opmode);
	}

	dev_warn(data->chardevice, "setting modulation to %s\n", modstr[modulation]);
	switch(modulation){
		case SX127X_MODULATION_FSK:
		case SX127X_MODULATION_OOK:
			opmode &= ~SX127X_REG_OPMODE_LONGRANGEMODE;
            data->cfg.modulation = modulation;
			data->loraregmap = 0;
			break;
		case SX127X_MODULATION_LORA:
			opmode |= SX127X_REG_OPMODE_LONGRANGEMODE;
			data->loraregmap = 1;
			break;
        default:
            break;
	}
	sx127x_reg_write(data->spidevice, SX127X_REG_OPMODE, opmode);
	return 0;
}

static int sx127x_setmodulation(struct sx127x *data, enum sx127x_modulation modulation){
    int ret = 0;

	dev_warn(data->chardevice, "setting modulation to %s\n", modstr[modulation]);
	switch(modulation){
		case SX127X_MODULATION_FSK:
		case SX127X_MODULATION_OOK:
			data->loraregmap = 0;
            data->cfg.modulation = modulation;
            ret = SX127xSetModem(data, modulation);
            if(ret < 0)
            {
                goto error;
            }
			break;
		case SX127X_MODULATION_LORA:
			data->loraregmap = 1;
            data->cfg.modulation = modulation;
            ret = SX127xSetModem(data, modulation);
            if(ret < 0)
            {
                goto error;
            }
			break;
        default:
            break;
	}

error:

    return ret;
}


static ssize_t sx127x_modulation_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count){
	struct sx127x *data = dev_get_drvdata(dev);
	int idx = sx127x_indexofstring(buf, modstr, ARRAY_SIZE(modstr));
	if(idx == -1){
		dev_warn(dev, "invalid modulation type\n");
		goto out;
	}
	mutex_lock(&data->mutex);
	sx127x_setmodulation(data, idx);
	mutex_unlock(&data->mutex);
out:
	return count;
}

static DEVICE_ATTR(modulation, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH, sx127x_modulation_show, sx127x_modulation_store);

int SX127XGetOpmode(struct sx127x *data, enum sx127x_opmode *mode)
{
    int ret = 0;
    u8 opmode = 0;
    
    mutex_lock(&data->mutex);
	ret = sx127x_reg_read(data->spidevice, SX127X_REG_OPMODE, &opmode);
    if(ret < 0)
    {
        goto error;
    }
	opmode = opmode & SX127X_REG_OPMODE_MODE;

    switch(opmode)
    {
    case SX127X_OPMODE_SLEEP:
        *mode = SX127X_OPMODE_SLEEP;
        break;
    case SX127X_OPMODE_STANDBY:
        *mode = SX127X_OPMODE_STANDBY;
        break;
    case SX127X_OPMODE_FSTX:
        *mode = SX127X_OPMODE_FSTX;
        break;
    case SX127X_OPMODE_TX:
        *mode = SX127X_OPMODE_TX;
        break;
    case SX127X_OPMODE_FSRX:
        *mode = SX127X_OPMODE_FSRX;
        break;
    case SX127X_OPMODE_RX:
        *mode = SX127X_OPMODE_RX;
        break;
    case SX127X_OPMODE_RXCONTINUOS:
        *mode = SX127X_OPMODE_RXCONTINUOS;
        break;
    case SX127X_OPMODE_RXSINGLE:
        *mode = SX127X_OPMODE_RXSINGLE;
        break;
    case SX127X_OPMODE_CAD:
        *mode = SX127X_OPMODE_CAD;
        break;
    default:
        break;
    }
    
	mutex_unlock(&data->mutex);

error:

    return ret;
}

static ssize_t sx127x_opmode_show(struct device *child, struct device_attribute *attr, char *buf)
{
    int ret = 0;
	struct sx127x *data = dev_get_drvdata(child);
	u8 opmode;
    enum sx127x_opmode mode;
	

    ret = SX127XGetOpmode(data, &mode);
    if(ret < 0)
    {
        goto error;
    }
    
	if(mode > 4){
		ret = sprintf(buf, "%s\n", opmodestr[mode + 1]);
	}
	else {
		ret =sprintf(buf, "%s\n", opmodestr[mode]);
	}

error:
    
	return ret;
}

static int sx127x_toggletxrxen(struct sx127x *data, bool tx){
	if(data->gpio_txen){
		if(tx){
			dev_warn(data->chardevice, "enabling tx\n");
		}
		gpiod_set_value(data->gpio_txen, tx);
	}
	if(data->gpio_rxen){
		if(!tx){
			dev_warn(data->chardevice, "enabling rx\n");
		}
		gpiod_set_value(data->gpio_rxen, !tx);
	}
	return 0;
}

int SX127XSetOpmode(struct sx127x *data, enum sx127x_opmode mode)
{
    int ret = 0;
    u8 opmode, diomapping1;

    ret = sx127x_reg_read(data->spidevice, SX127X_REG_OPMODE, &opmode);
    if(mode < SX127X_OPMODE_SLEEP || mode > SX127X_OPMODE_CAD){
        ret = -EINVAL;
        dev_err(data->chardevice, "invalid opmode\n");
    }
    else if((opmode & SX127X_REG_OPMODE_LONGRANGEMODE) && (mode == SX127X_OPMODE_RX)){
        dev_err(data->chardevice, "opmode %s not valid in LoRa mode\n", opmodestr[mode]);
    }
    else if(!(opmode & SX127X_REG_OPMODE_LONGRANGEMODE) && (mode > SX127X_OPMODE_RX)) {
        dev_err(data->chardevice, "opmode %s not valid in FSK/OOK mode\n", opmodestr[mode]);
    }
    else {
        
        dev_warn(data->chardevice, "setting opmode to %s\n", opmodestr[mode]);
        sx127x_reg_read(data->spidevice, SX127X_REG_DIOMAPPING1, &diomapping1);
        diomapping1 &= ~SX127X_REG_DIOMAPPING1_DIO0;
        switch(mode){
        case SX127X_OPMODE_CAD:
            diomapping1 |= SX127X_REG_DIOMAPPING1_DIO0_CADDONE;
            sx127x_toggletxrxen(data, false);
            break;
        case SX127X_OPMODE_TX:
            diomapping1 |= SX127X_REG_DIOMAPPING1_DIO0_TXDONE;
            sx127x_toggletxrxen(data, true);
            break;
        case SX127X_OPMODE_RX:
        case SX127X_OPMODE_RXCONTINUOS:
        case SX127X_OPMODE_RXSINGLE:
            diomapping1 |= SX127X_REG_DIOMAPPING1_DIO0_RXDONE;
            sx127x_toggletxrxen(data, false);
            break;
        default:
            dev_warn(data->chardevice, "disabling rx and tx\n");
            gpiod_set_value(data->gpio_rxen, 0);
            gpiod_set_value(data->gpio_txen, 0);
            break;
        }
        opmode &= ~SX127X_REG_OPMODE_MODE;
        if(mode > SX127X_OPMODE_RX){
            mode -= 1;
        }
        opmode |= mode;
        sx127x_reg_write(data->spidevice, SX127X_REG_DIOMAPPING1, diomapping1);
        sx127x_reg_write(data->spidevice, SX127X_REG_OPMODE, opmode);
    }
    return ret;

}

static int sx127x_setopmode(struct sx127x *data, enum sx127x_opmode mode, bool retain){
    int ret = 0;

    ret = SX127XSetOpmode(data, mode);
    if(ret < 0)
    {
        goto error;
    }
    
    if(retain){
	    data->opmode = mode;
	}

error:
    
	return ret;
}

int SX127XSetSyncWord(struct sx127x *data, u8 syncword)
{
    int ret = 0;
    
    ret = sx127x_reg_write(data->spidevice, SX127X_REG_LORA_SYNCWORD, syncword);

    return ret;
}

static int sx127x_setsyncword(struct sx127x *data, u8 syncword){
    int ret = 0;
    
    dev_warn(data->chardevice, "setting syncword to %d\n", syncword);
	ret = SX127XSetSyncWord(data, syncword);
    if(ret < 0)
    {
        goto error;
    }
    data->cfg.syncword = syncword;
    
error:
    
	return ret;
}

int SX127XSetInvertIQ(struct sx127x *data, bool invert)
{
    int ret = 0;
    u8 reg;
    
    ret = sx127x_reg_read(data->spidevice, SX127X_REG_LORA_INVERTIQ, &reg);
    if(ret < 0)
    {
        goto error;
    }
    
	if(invert)
		reg |= SX127X_REG_LORA_INVERTIQ_INVERTIQ;
	else
		reg &= ~SX127X_REG_LORA_INVERTIQ_INVERTIQ;
	ret = sx127x_reg_write(data->spidevice, SX127X_REG_LORA_INVERTIQ, reg);
    if(ret < 0)
    {
        goto error;
    }

error:
    
    return ret;
}

static int sx127x_setinvertiq(struct sx127x *data, bool invert){
    int ret = 0;
    
	dev_warn(data->chardevice, "setting invertiq to %d\n", invert);
    ret = SX127XSetInvertIQ(data, invert);
    if(ret < 0)
    {
        goto error;
    }
    data->cfg.invertiq = invert;
error:
    
	return ret;
}

int SX127XSetCrc(struct sx127x *data, bool crc)
{
    int ret = 0;
    u8 reg;

    ret = sx127x_reg_read(data->spidevice, SX127X_REG_LORA_MODEMCONFIG2, &reg);
    if(ret < 0)
    {
        goto error;
    }
    
	if(crc)
		reg |= SX127X_REG_LORA_MODEMCONFIG2_RXPAYLOADCRCON;
	else
		reg &= ~SX127X_REG_LORA_MODEMCONFIG2_RXPAYLOADCRCON;
	ret = sx127x_reg_write(data->spidevice, SX127X_REG_LORA_MODEMCONFIG2, reg);
    if(ret < 0)
    {
        goto error;
    }

error:

    return ret;
}

static int sx127x_setcrc(struct sx127x *data, bool crc){
	int ret = 0;
    
	dev_warn(data->chardevice, "setting crc to %d\n", crc);

    ret = SX127XSetCrc(data, crc);
    if(ret < 0)
    {
        goto error;
    }
    data->cfg.crc = crc;
error:
    
	return ret;
}

static ssize_t sx127x_opmode_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count){
	struct sx127x *data = dev_get_drvdata(dev);
	int idx;
	enum sx127x_opmode mode;

	idx = sx127x_indexofstring(buf, opmodestr, ARRAY_SIZE(opmodestr));
	if(idx == -1){
		dev_err(dev, "invalid opmode\n");
		goto out;
	}
	mutex_lock(&data->mutex);
	mode = idx;
	sx127x_setopmode(data, mode, true);
	mutex_unlock(&data->mutex);
out:
	return count;
}

static DEVICE_ATTR(opmode, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH, sx127x_opmode_show, sx127x_opmode_store);

int SX127XGetFreq(struct sx127x *data, u32 *freq)
{
    int ret = 0;
    u8 msb, mld, lsb;
	u32 frf;
	

    mutex_lock(&data->mutex);
	ret = sx127x_reg_read(data->spidevice, SX127X_REG_FRFMSB, &msb);
    if(ret < 0)
    {
        goto error;
    }
    
	ret = sx127x_reg_read(data->spidevice, SX127X_REG_FRFMLD, &mld);
    if(ret < 0)
    {
        goto error;
    }
    
	ret = sx127x_reg_read(data->spidevice, SX127X_REG_FRFLSB, &lsb);
    if(ret < 0)
    {
        goto error;
    }
    
	frf = (msb << 16) | (mld << 8) | lsb;
	freq = ((u64)data->fosc * frf) / 524288;
	mutex_unlock(&data->mutex);

error:

    return ret;
}

static ssize_t sx127x_carrierfrequency_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int ret = 0;
    u32 freq = 0;
	struct sx127x *data = dev_get_drvdata(dev);

    ret = SX127XGetFreq(data, &freq);
	if(ret < 0)
    {
        goto error;
    }
    return sprintf(buf, "%u\n", freq);

error:

    return ret;
}

int Sx127XSetFreq(struct sx127x *data, u32 freq)
{
    int ret = 0;
    u8 opmode, newopmode;
    u64 freq_temp = 0;
    u32 reg = 0;
    
	ret = sx127x_reg_read(data->spidevice, SX127X_REG_OPMODE, &opmode);
    if(ret < 0)
    {
        goto error;
    }
    
	newopmode = opmode & ~SX127X_REG_OPMODE_LOWFREQUENCYMODEON;
	if(freq < 700000000){
		newopmode |= SX127X_REG_OPMODE_LOWFREQUENCYMODEON;
	}
	if(newopmode != opmode){
		dev_warn(data->chardevice, "toggling LF/HF bit\n");
		ret = sx127x_reg_write(data->spidevice, SX127X_REG_OPMODE, newopmode);
        if(ret < 0)
        {
            goto error;
        }
	}

	
	freq_temp = freq * 524288;

	do_div(freq_temp, data->fosc);
    reg = (u32)freq_temp;
	ret = sx127x_reg_write24(data->spidevice, SX127X_REG_FRFMSB, reg);
    if(ret < 0)
    {
        goto error;
    }
    
error:
    
	return ret;
}
    
static int sx127x_setcarrierfrequency(struct sx127x *data, u32 freq){
    int ret = 0;

    dev_warn(data->chardevice, "setting carrier frequency to %llu\n", freq);

    ret = Sx127XSetFreq(data, freq);
    if(ret < 0)
    {
        goto error;
    }
    data->cfg.freq = freq;
error:

    return ret;
}

static ssize_t sx127x_carrierfrequency_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count){
	struct sx127x *data = dev_get_drvdata(dev);
	u32 freq;

	if(kstrtou32(buf, 10, &freq)){
		goto out;
	}
	mutex_lock(&data->mutex);
	sx127x_setcarrierfrequency(data, freq);
	mutex_unlock(&data->mutex);
	out:
	return count;
}

static DEVICE_ATTR(carrierfrequency, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH, sx127x_carrierfrequency_show, sx127x_carrierfrequency_store);

static ssize_t sx127x_rssi_show(struct device *child, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", 0);
}

static DEVICE_ATTR(rssi, S_IRUSR | S_IRGRP | S_IROTH, sx127x_rssi_show, NULL);

int SX127XGetSF(struct sx127x *data, int *sf)
{
    int ret= 0;
    u8 config2;
    
    sx127x_reg_read(data->spidevice, SX127X_REG_LORA_MODEMCONFIG2, &config2);
	*sf = config2 >> SX127X_REG_LORA_MODEMCONFIG2_SPREADINGFACTOR_SHIFT;
}

static ssize_t sx127x_sf_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct sx127x *data = dev_get_drvdata(dev);
	int sf = 0;
	
	mutex_lock(&data->mutex);
	SX127XGetSF(data, &sf);
	mutex_unlock(&data->mutex);
    return sprintf(buf, "%d\n", sf);
}

int SX127XSetSF(struct sx127x *data, unsigned sf)
{
    int ret = 0;
	u8 r;
	
	// set the spreading factor
	sx127x_reg_read(data->spidevice, SX127X_REG_LORA_MODEMCONFIG2, &r);
	r &= ~SX127X_REG_LORA_MODEMCONFIG2_SPREADINGFACTOR;
	r |= sf << SX127X_REG_LORA_MODEMCONFIG2_SPREADINGFACTOR_SHIFT;
	sx127x_reg_write(data->spidevice, SX127X_REG_LORA_MODEMCONFIG2, r);

	// set the detection optimization magic number depending on the spreading factor
	sx127x_reg_read(data->spidevice, SX127X_REG_LORA_DETECTOPTIMIZATION, &r);
	r &= ~SX127X_REG_LORA_DETECTOPTIMIZATION_DETECTIONOPTIMIZE;
	if(sf == 6){
		r |= SX127X_REG_LORA_DETECTOPTIMIZATION_DETECTIONOPTIMIZE_SF6;
	}
	else{
		r |= SX127X_REG_LORA_DETECTOPTIMIZATION_DETECTIONOPTIMIZE_SF7SF12;
	}
	sx127x_reg_write(data->spidevice, SX127X_REG_LORA_DETECTOPTIMIZATION, r);

	// set the low data rate bit
	sx127x_reg_read(data->spidevice, SX127X_REG_LORA_MODEMCONFIG3, &r);
	r |= SX127X_REG_LORA_MODEMCONFIG3_LOWDATARATEOPTIMIZE;
	sx127x_reg_write(data->spidevice, SX127X_REG_LORA_MODEMCONFIG3, r);
    
	return ret;

}

static int sx127x_setsf(struct sx127x *data, unsigned sf){
    int ret = 0;
    
    dev_info(data->chardevice, "setting spreading factor to %u\n", sf);
    ret = SX127XSetSF(data, sf);
    if(ret < 0)
    {
        goto error;
    }
    data->cfg.sf = sf;
error:
    
    return ret;
}

static ssize_t sx127x_sf_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count){
	struct sx127x *data = dev_get_drvdata(dev);
	int sf;
	if(kstrtoint(buf, 10, &sf)){
		goto out;
	}
	mutex_lock(&data->mutex);
	sx127x_setsf(data, sf);
	mutex_unlock(&data->mutex);
	out:
	return count;
}

static DEVICE_ATTR(sf, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH, sx127x_sf_show, sx127x_sf_store);

int SX127XGetBW(struct sx127x *data, unsigned *bandwidth)
{
    int ret =0;
    u8 config1;
	int bw;
    
	sx127x_reg_read(data->spidevice, SX127X_REG_LORA_MODEMCONFIG1, &config1);
	bw = config1 >> SX127X_REG_LORA_MODEMCONFIG1_BW_SHIFT;
	
    
    return ret;
}

static ssize_t sx127x_bw_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int ret = 0;
    int bandwidth = 0;
	struct sx127x *data = dev_get_drvdata(dev);


	mutex_lock(&data->mutex);
    ret = SX127XGetBW(data, &bandwidth);
    if(bandwidth > SX127X_REG_LORA_MODEMCONFIG1_BW_MAX){
        ret = sprintf(buf, "invalid\n");
    }
    else {
        ret = sprintf(buf, "%d\n", bwmap[bandwidth]);
    }

	mutex_unlock(&data->mutex);
	return ret;
}

static ssize_t sx127x_bw_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count){
	//struct sx127x *data = dev_get_drvdata(dev);
	return count;
}

static DEVICE_ATTR(bw, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH, sx127x_bw_show, sx127x_bw_store);

static char* crmap[] = { NULL, "4/5", "4/6", "4/7", "4/8" };

int SX127XGetCodingRate(struct sx127x *data, unsigned *codingrate)
{
    int ret = 0;
    u8 config1;

    ret = sx127x_reg_read(data->spidevice, SX127X_REG_LORA_MODEMCONFIG1, &config1);
	*codingrate = (config1 & SX127X_REG_LORA_MODEMCONFIG1_CODINGRATE) >> SX127X_REG_LORA_MODEMCONFIG1_CODINGRATE_SHIFT;

    return ret;
}

static ssize_t sx127x_codingrate_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct sx127x *data = dev_get_drvdata(dev);
	
	int cr, ret;

	mutex_lock(&data->mutex);
	ret = SX127XGetCodingRate(data, &cr);
	if(cr < SX127X_REG_LORA_MODEMCONFIG1_CODINGRATE_MIN ||
			cr > SX127X_REG_LORA_MODEMCONFIG1_CODINGRATE_MAX){
		ret = sprintf(buf, "invalid\n");
	}
	else {
		ret = sprintf(buf, "%s\n", crmap[cr]);
	}
	mutex_unlock(&data->mutex);
    return ret;
}

int SX127XSetCodingRate(struct sx127x *data, unsigned codingrate)
{
    int ret = 0;

error:

    return ret;
}

static ssize_t sx127x_codingrate_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count){
	//struct sx127x *data = dev_get_drvdata(dev);
	return count;
}

static DEVICE_ATTR(codingrate, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH, sx127x_codingrate_show, sx127x_codingrate_store);

int SX127XGetImplicitHeader(struct sx127x *data, unsigned *implicitheader)
{
    int ret = 0;
    u8 config1;

    ret = sx127x_reg_read(data->spidevice, SX127X_REG_LORA_MODEMCONFIG1, &config1);
	*implicitheader = config1 & SX127X_REG_LORA_MODEMCONFIG1_IMPLICITHEADERMODEON;

    return ret;
}

static ssize_t sx127x_implicitheadermodeon_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int ret = 0;
	struct sx127x *data = dev_get_drvdata(dev);
	
	int hdrmodeon;

	mutex_lock(&data->mutex);
    ret = SX127XGetImplicitHeader(data, &hdrmodeon);
	mutex_unlock(&data->mutex);
    return sprintf(buf, "%d\n", hdrmodeon);
}

int SX127XSetImplicitHeader(struct sx127x *data, unsigned *implicitheader)
{
    int ret = 0;

    return ret;
}
static ssize_t sx127x_implicitheadermodeon_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count){
	//struct sx127x *data = dev_get_drvdata(dev);
	return count;
}

static DEVICE_ATTR(implicitheadermodeon, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH, sx127x_implicitheadermodeon_show, sx127x_implicitheadermodeon_store);

int SX127XGetPaoutput(struct sx127x *data, unsigned *idx)
{
    int ret = 0;
    u8 paconfig;

    sx127x_reg_read(data->spidevice, SX127X_REG_PACONFIG, &paconfig);
	*idx = (paconfig & SX127X_REG_PACONFIG_PASELECT) ? 1 : 0;

    return ret;
}

static ssize_t sx127x_paoutput_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int ret = 0;
    int idx;
	struct sx127x *data = dev_get_drvdata(dev);

	mutex_lock(&data->mutex);
	ret = SX127XGetPaoutput(data, &idx);
	mutex_unlock(&data->mutex);
    return sprintf(buf, "%s\n", paoutput[idx]);
}

static int SX127XSetPaoutput(struct sx127x *data, enum sx127x_pa pa){
	int ret = 0;
	u8 paconfig;
    
	sx127x_reg_read(data->spidevice, SX127X_REG_PACONFIG, &paconfig);
	switch(pa){
		case SX127X_PA_RFO:
			paconfig &= ~SX127X_REG_PACONFIG_PASELECT;
			break;
		case SX127X_PA_PABOOST:
			paconfig |= SX127X_REG_PACONFIG_PASELECT;
			break;
	}
	sx127x_reg_write(data->spidevice, SX127X_REG_PACONFIG, paconfig);
    
	return ret;
}

static int sx127x_setpaoutput(struct sx127x *data, enum sx127x_pa pa){
	int ret = 0;

    ret = SX127XSetPaoutput(data, pa);
	switch(pa){
		case SX127X_PA_RFO:
			data->cfg.pa = SX127X_PA_RFO;
			break;
		case SX127X_PA_PABOOST:
			data->cfg.pa = SX127X_PA_PABOOST;
			break;
	}
    
	return ret;
}


static ssize_t sx127x_paoutput_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count){
	//TODO this needs to take into account non-default values for the "padac".
	struct sx127x *data = dev_get_drvdata(dev);
	int idx = sx127x_indexofstring(buf, paoutput, ARRAY_SIZE(paoutput));
	if(idx == -1)
		goto out;
	mutex_lock(&data->mutex);
	sx127x_setpaoutput(data, idx);
	mutex_unlock(&data->mutex);
out:
	return count;
}

static DEVICE_ATTR(paoutput, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH, sx127x_paoutput_show, sx127x_paoutput_store);

int SX127XGetOutputPower(struct sx127x *data, int *outputpower)
{
    int ret = 0;
    u8 paconfig;
    int maxoutputpower = 17;
	

    sx127x_reg_read(data->spidevice, SX127X_REG_PACONFIG, &paconfig);
	if(!(paconfig & SX127X_REG_PACONFIG_PASELECT)){
		maxoutputpower = ((paconfig & SX127X_REG_PACONFIG_MAXPOWER) >> SX127X_REG_PACONFIG_MAXPOWER_SHIFT);
	}
	*outputpower = maxoutputpower - (15 - (paconfig & SX127X_REG_PACONFIG_OUTPUTPOWER));

    return ret;
}

static ssize_t sx127x_outputpower_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int ret = 0;
	struct sx127x *data = dev_get_drvdata(dev);
	int outputpower;

	mutex_lock(&data->mutex);
	SX127XGetOutputPower(data, &outputpower);
	mutex_unlock(&data->mutex);
    return sprintf(buf, "%d\n", outputpower);
}

int SX127XSetOutputPower(struct sx127x *data, int outputpower)
{
    int ret = 0;

    u8 paconfig;
	ret = sx127x_reg_read(data->spidevice, SX127X_REG_PACONFIG, &paconfig);
	ret = sx127x_reg_write(data->spidevice, SX127X_REG_PACONFIG, paconfig);

    return ret;
}

static ssize_t sx127x_outputpower_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count){
    int ret = 0;
	struct sx127x *data = dev_get_drvdata(dev);
	//int idx = sx127x_indexofstring(buf, paoutput, ARRAY_SIZE(paoutput));
	ret = SX127XSetOutputPower(data, 0);
	return count;
}

static DEVICE_ATTR(outputpower, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH,
		sx127x_outputpower_show, sx127x_outputpower_store);


static irqreturn_t sx127x_top_irq_handler(int irq, void *dev_id)
{
	//struct sx127x *data = dev_id;
    disable_irq_nosync(irq);
	//schedule_work(&data->irq_work);
	return IRQ_HANDLED;
}

static irqreturn_t sx127x_bottem_irq_thread(int irq, void *dev_id)
{
	//struct sx127x *data = container_of(work, struct sx127x, irq_work);
	struct sx127x *data = dev_id;
	u8 irqflags, buf[128], len, snr, rssi;
	u32 fei;
	struct sx127x_pkt pkt;
	mutex_lock(&data->mutex);
	sx127x_reg_read(data->spidevice, SX127X_REG_LORA_IRQFLAGS, &irqflags);
	if(irqflags & SX127X_REG_LORA_IRQFLAGS_RXDONE){
		dev_warn(data->chardevice, "reading packet\n");
		memset(&pkt, 0, sizeof(pkt));

		sx127x_fifo_readpkt(data->spidevice, buf, &len);
		sx127x_reg_read(data->spidevice, SX127X_REG_LORA_PKTSNRVALUE, &snr);
		sx127x_reg_read(data->spidevice, SX127X_REG_LORA_PKTRSSIVALUE, &rssi);
		sx127x_reg_read24(data->spidevice, SX127X_REG_LORA_FEIMSB, &fei);

		pkt.hdrlen = sizeof(pkt);
		pkt.payloadlen = len;
		pkt.len = pkt.hdrlen + pkt.payloadlen;
		pkt.snr = (__s16)(snr << 2) / 4 ;
		pkt.rssi = -157 + rssi; //TODO fix this for the LF port

		if(irqflags & SX127X_REG_LORA_IRQFLAGS_PAYLOADCRCERROR){
			dev_warn(data->chardevice, "CRC Error for received payload\n");
			pkt.crcfail = 1;
		}

		kfifo_in(&data->out, &pkt, sizeof(pkt));
		kfifo_in(&data->out, buf, len);
		wake_up(&data->readwq);
	}
	else if(irqflags & SX127X_REG_LORA_IRQFLAGS_TXDONE){
		if(data->gpio_txen){
			gpiod_set_value(data->gpio_txen, 0);
		}
		dev_warn(data->chardevice, "transmitted packet\n");
		/* after tx the chip goes back to standby so restore the user selected mode if it wasn't standby */
		if(data->opmode != SX127X_OPMODE_STANDBY){
			dev_info(data->chardevice, "restoring opmode\n");
			sx127x_setopmode(data, data->opmode, false);
		}
		data->transmitted = 1;
		wake_up(&data->writewq);
	}
	else if(irqflags & SX127X_REG_LORA_IRQFLAGS_CADDONE){
		if(irqflags & SX127X_REG_LORA_IRQFLAGS_CADDETECTED){
			dev_info(data->chardevice, "CAD done, detected activity\n");
		}
		else {
			dev_info(data->chardevice, "CAD done, nothing detected\n");
		}
	}
	else {
		dev_err(&data->spidevice->dev, "unhandled interrupt state %02x\n", (unsigned) irqflags);
	}
	sx127x_reg_write(data->spidevice, SX127X_REG_LORA_IRQFLAGS, 0xff);
	mutex_unlock(&data->mutex);
    enable_irq(irq);

    return IRQ_HANDLED;
}


static int drv_open(struct inode *inode, struct file *file)
{
	struct sx127x *data;
	int status = -ENXIO;
    int irq;
	mutex_lock(&device_list_lock);

	list_for_each_entry(data, &device_list, device_entry) {
		if (data->devt == inode->i_rdev) {
			status = 0;
			break;
		}
	}

	if(status){
		pr_debug("sx127x: nothing for minor %d\n", iminor(inode));
		goto err_notfound;
	}

	mutex_lock(&data->mutex);
	if(data->open){
		pr_debug("sx127x: already open\n");
		status = -EBUSY;
		goto err_open;
	}
	data->open = 1;
    irq = gpiod_to_irq(data->gpio_dio0);
    if (!irq) {
		dev_err(&data->spidevice->dev, "No irq in platform data\n");
		status = -EINVAL;
		goto err_open;
	}
    
    status = devm_request_threaded_irq(&data->spidevice->dev, irq, sx127x_top_irq_handler, sx127x_bottem_irq_thread, IRQF_TRIGGER_RISING, DRV_NAME, data);
    if(status < 0)
    {
        goto err_open;
    }
	mutex_unlock(&data->mutex);

	mutex_unlock(&device_list_lock);

	file->private_data = data;
	return 0;

err_open:
	mutex_unlock(&data->mutex);
err_notfound:
	mutex_unlock(&device_list_lock);
	return status;
}

static ssize_t drv_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	struct sx127x *data = filp->private_data;
	unsigned copied;
	ssize_t ret = 0;
	wait_event_interruptible(data->readwq, kfifo_len(&data->out));
	ret = kfifo_to_user(&data->out, buf, count, &copied);
	if(!ret && copied > 0){
		ret = copied;
	}
	return ret;
}

static ssize_t drv_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
    int ret = 0;
	struct sx127x *data = filp->private_data;
	size_t packetsz, offset, maxpkt = 256;
	u8 kbuf[256];
	dev_info(&data->spidevice->dev, "char device write; %d\n", count);
	for(offset = 0; offset < count; offset += maxpkt){
		packetsz = min((count - offset), maxpkt);
		mutex_lock(&data->mutex);
		ret = copy_from_user(kbuf, buf + offset, packetsz);
        if(ret < 0)
        {
            goto error;
        }
		sx127x_setopmode(data, SX127X_OPMODE_STANDBY, false);
		sx127x_fifo_writepkt(data->spidevice, kbuf, packetsz);
		data->transmitted = 0;
		sx127x_setopmode(data, SX127X_OPMODE_TX, false);
		mutex_unlock(&data->mutex);
		wait_event_interruptible_timeout(data->writewq, data->transmitted, 60 * HZ);
	}

	return count;

 error:

    return ret;
}

static int drv_release(struct inode *inode, struct file *filp)
{
	struct sx127x *data = filp->private_data;
	mutex_lock(&data->mutex);
	sx127x_setopmode(data, SX127X_OPMODE_STANDBY, true);
	data->open = 0;
	kfifo_reset(&data->out);
	mutex_unlock(&data->mutex);
	return 0;
}

static long drv_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct sx127x *data = filp->private_data;
	int ret = 0;
	
	mutex_lock(&data->mutex);
	switch(cmd){
		case LORA_IOC_WR_MODULATION:
			ret = sx127x_setmodulation(data, arg);
			break;
		case LORA_IOC_RD_MODULATION:
			ret = 0;
			break;
		case LORA_IOC_WR_CARRIERFRE:
			ret = sx127x_setcarrierfrequency(data, arg);
			break;
		case LORA_IOC_RD_CARRIERFRE:
			ret = 0;
			break;
		case LORA_IOC_WR_SF:
			ret = sx127x_setsf(data, arg);
			break;
		case LORA_IOC_RD_SF:
			ret = 0;
			break;
		case LORA_IOC_WR_OPMODE:
			ret = sx127x_setopmode(data, arg, true);
			break;
		case LORA_IOC_RD_OPMODE:
			ret = 0;
			break;
		case LORA_IOC_WR_PAOUTPUT:
			ret = sx127x_setpaoutput(data, arg);
			break;
		case LORA_IOC_RD_PAOUTPUT:
			ret = 0;
			break;
		case LORA_IOC_WR_SYNCWORD:
			ret = sx127x_setsyncword(data, arg & 0xff);
			break;
		case LORA_IOC_RD_SYNCWORD:
			ret = 0;
			break;
		case LORA_IOC_WR_CRC:
			ret = sx127x_setcrc(data, arg & 0x1);
			break;
		case LORA_IOC_RD_CRC:
			ret = 0;
			break;
		case LORA_IOC_WR_INVERTIQ:
			ret = sx127x_setinvertiq(data, arg & 1);
			break;
		case LORA_IOC_RD_INVERTIQ:
			ret = 0;
			break;
        case LORA_IOC_WR_BANDWIDTH:
            break;
        case LORA_IOC_RD_BANDWIDTH:
            break;
		default:
			ret = -EINVAL;
			break;
	}
	mutex_unlock(&data->mutex);
	return ret;
}

static struct file_operations drv_fops = {
		.open = drv_open,
		.read = drv_read,
		.write = drv_write,
		.release = drv_release,
		.unlocked_ioctl = drv_ioctl
};


static int sx127x_probe(struct spi_device *spi){
	int ret = 0;
	struct sx127x *data;
	u8 version;
	unsigned minor;

	// allocate all of the crap we need
	data = kmalloc(sizeof(*data), GFP_KERNEL);
	if(!data){
		DPRINTK("failed to allocate driver data\n");
		ret = -ENOMEM;
		goto err_allocdevdata;
	}

	data->open = 0;
    data->name = spi->modalias;
	//INIT_WORK(&data->irq_work, sx127x_irq_work_handler);
	INIT_LIST_HEAD(&data->device_entry);
	init_waitqueue_head(&data->readwq);
	init_waitqueue_head(&data->writewq);
	mutex_init(&data->mutex);
	data->fosc = 32000000; //TODO use value from DT
	data->cfg.pa = SX127X_PA_PABOOST; // TODO use value from DT
	data->spidevice = spi;
	data->opmode = SX127X_OPMODE_STANDBY;

	ret = kfifo_alloc(&data->out, PAGE_SIZE, GFP_KERNEL);
	if(ret){
		DPRINTK("failed to allocate out fifo\n");
		goto err_allocoutfifo;
	}

	// get the reset gpio and reset the chip
	data->gpio_reset = devm_gpiod_get(&spi->dev, "reset");
	if(IS_ERR(data->gpio_reset)){
		dev_err(&spi->dev, "reset gpio is required");
		ret = -ENOMEM;
		goto err_resetgpio;
	}

	gpiod_set_value(data->gpio_reset, 1);
	mdelay(100);
	gpiod_set_value(data->gpio_reset, 0);
	mdelay(100);

	// get the rev from the chip and check it's what we expect
	sx127x_reg_read(spi, SX127X_REG_VERSION, &version);
	if(version != 0x12){
		dev_err(&spi->dev, "unknown chip version %x\n", version);
		ret = -EINVAL;
		goto err_chipid;
	}
	dev_warn(&spi->dev, "chip version %x\n",(unsigned) version);

	// get the other optional gpios
	//data->gpio_txen = devm_gpiod_get_index(&spi->dev, "txrxswitch", 0, GPIOD_OUT_LOW);
	//if(IS_ERR(data->gpio_txen)){
	//	dev_warn(&spi->dev, "no tx enable\n");
	//	data->gpio_txen = NULL;
	//}

	//data->gpio_rxen = devm_gpiod_get_index(&spi->dev, "txrxswitch", 1, GPIOD_OUT_LOW);
	//if(IS_ERR(data->gpio_rxen)){
	//	dev_warn(&spi->dev, "no rx enable\n");
	//	data->gpio_rxen = NULL;
	//}

    data->gpio_dio0 = devm_gpiod_get_index(&spi->dev, "DIO0", 0);
	if(IS_ERR(data->gpio_dio0)){
		dev_warn(&spi->dev, "no DIO0 enable\n");
		data->gpio_rxen = NULL;
	}

	// get the irq
	//irq = irq_of_parse_and_map(spi->dev.of_node, 0);
    //irq = gpiod_to_irq(data->gpio_dio0);
	//if (!irq) {
	//	dev_err(&spi->dev, "No irq in platform data\n");
	//	ret = -EINVAL;
	//	goto err_irq;
	//}
	//devm_request_irq(&spi->dev, irq, sx127x_irq, 0, DRV_NAME, data);

	// create the frontend device and stash it in the spi device
	mutex_lock(&device_list_lock);
	minor = 0;
	data->devt = MKDEV(devmajor, minor);
	data->chardevice = device_create(devclass, &spi->dev, data->devt, data, "%s%d", DRV_NAME, minor);
	if(IS_ERR(data->chardevice)){
		DPRINTK("failed to create char device\n");
		ret = -ENOMEM;
		goto err_createdevice;
	}
	list_add(&data->device_entry, &device_list);
	mutex_unlock(&device_list_lock);
	spi_set_drvdata(spi, data);

	// setup sysfs nodes
	ret = device_create_file(data->chardevice, &dev_attr_modulation);
	ret = device_create_file(data->chardevice, &dev_attr_opmode);
	ret = device_create_file(data->chardevice, &dev_attr_carrierfrequency);
	ret = device_create_file(data->chardevice, &dev_attr_rssi);
	ret = device_create_file(data->chardevice, &dev_attr_paoutput);
	ret = device_create_file(data->chardevice, &dev_attr_outputpower);
	// these are LoRa specifc
	ret = device_create_file(data->chardevice, &dev_attr_sf);
	ret = device_create_file(data->chardevice, &dev_attr_bw);
	ret = device_create_file(data->chardevice, &dev_attr_codingrate);
	ret = device_create_file(data->chardevice, &dev_attr_implicitheadermodeon);

	return 0;

//err_sysfs:
//	device_destroy(devclass, data->devt);
err_createdevice:
	mutex_unlock(&device_list_lock);
//err_irq:
err_chipid:
err_resetgpio:
	kfifo_free(&data->out);
err_allocoutfifo:
	kfree(data);
err_allocdevdata:
	return ret;
}

static int sx127x_remove(struct spi_device *spi){
	struct sx127x *data = spi_get_drvdata(spi);

	device_remove_file(data->chardevice, &dev_attr_modulation);
	device_remove_file(data->chardevice, &dev_attr_opmode);
	device_remove_file(data->chardevice, &dev_attr_carrierfrequency);
	device_remove_file(data->chardevice, &dev_attr_rssi);
	device_remove_file(data->chardevice, &dev_attr_paoutput);
	device_remove_file(data->chardevice, &dev_attr_outputpower);

	device_remove_file(data->chardevice, &dev_attr_sf);
	device_remove_file(data->chardevice, &dev_attr_bw);
	device_remove_file(data->chardevice, &dev_attr_codingrate);
	device_remove_file(data->chardevice, &dev_attr_implicitheadermodeon);

	device_destroy(devclass, data->devt);

	kfifo_free(&data->out);
	kfree(data);

	return 0;
}

static const struct of_device_id sx127x_of_match[] = {
	{
		.compatible = "semtech, sx127x",
	},
	{ },
};
MODULE_DEVICE_TABLE(of, sx127x_of_match);


static struct spi_driver sx127x_driver = {
	.probe		= sx127x_probe,
	.remove		= sx127x_remove,
	.driver = {
		.name	= SX127X_DRIVERNAME,
		.of_match_table = of_match_ptr(sx127x_of_match),
		.owner = THIS_MODULE,
	},
};

static int __init sx127x_init(void)
{
	int result = 0;
    //申请设备号
    result = register_chrdev_region(drv_dev_num, DRV_MINOR, DRV_NAME);
    if (result)
    {
        goto ERR_EXIT;
    }
    
    //注册字符设备
    cdev_init(&drv_cdev, &drv_fops);
    drv_cdev.owner = THIS_MODULE;
    result = cdev_add(&drv_cdev, drv_dev_num, DRV_MINOR);
    if (result)   
    {        
        goto ERR_EXIT0;    
    }
    
    //创建设备节点    
    drv_class = class_create(THIS_MODULE, DRV_NAME);    
    if (drv_class == NULL) 
    {        
        result = -ENOMEM;        
        goto ERR_EXIT1;    
    }

    result = spi_register_driver(&sx127x_driver);
    if (result < 0)
    {
        goto ERR_EXIT2;
    }

    return result;
ERR_EXIT2:
    class_destroy(drv_class);
ERR_EXIT1:
    cdev_del(&drv_cdev);
ERR_EXIT0:
    unregister_chrdev_region(drv_dev_num, DRV_MINOR);
ERR_EXIT:    
    return result;
}
module_init(sx127x_init);

static void __exit sx127x_exit(void)
{
	spi_unregister_driver(&sx127x_driver);
    cdev_del(&drv_cdev);
    class_destroy(drv_class);
	drv_class = NULL;
	unregister_chrdev_region(drv_dev_num, DRV_MINOR);
}
module_exit(sx127x_exit);

MODULE_LICENSE("GPL");
