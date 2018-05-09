#include <stdio.h>
#include <stdbool.h>
#include <fcntl.h>
#include <unistd.h>

#include "sx127xlib.h"

typedef unsigned char u8;
typedef char s8;
typedef unsigned short u16;
typedef short s16;
typedef unsigned int u32;
typedef int s32;

struct sx127x_config{
	u32 txfreq;
	u8 txbandwidth;
	u8 txspreadfactor;
	u8 txcodingrate;
	s8 txpower;
	u32 txtimeout;
	
	u32 rxfreq;
	u8 rxbandwidth;
	u8 rxspreadfactor;
	u8 rxcodingrate;
	
	u8 paoutput;
	u16 preamblelen;
};

int main(void)
{

    int fd = 0;
    int result = 0;
    unsigned char val = 1;
	
	struct sx127x_config cfg = {0};
	
	cfg.txfreq 			= 470000000;
	cfg.txbandwidth 	= BW_125KHZ;
	cfg.txspreadfactor 	= SF_256_CHIPS_SYMBOL;
	cfg.txcodingrate 	= CR_4_5;
	cfg.txpower 		= 14;
	cfg.txtimeout 		= 2000;
	cfg.rxfreq			= 480000000;
	cfg.rxbandwidth 	= BW_250KHZ;
	cfg.rxspreadfactor	= SF_1024_CHIPS_SYMBOL;
	cfg.rxcodingrate 	= CR_4_6;
	cfg.paoutput		= PA_BOOST;
	cfg.preamblelen 	= 8;
	
    fd = open("/dev/lora0", O_RDWR);
    if(fd < 0)
    {
        printf("open error\n");
        goto ERR_EXIT;
    }
    
	
    result = ioctl(fd, SET_TXFREQ, cfg.txfreq);
    if(result < 0)
    {
		goto ERR_EXIT;
    }
    
	result = ioctl(fd, SET_RXFREQ, cfg.rxfreq);
    if(result < 0)
    {
		goto ERR_EXIT;
    }
	
	result = ioctl(fd, SET_TXBW, cfg.txbandwidth);
    if(result < 0)
    {
		goto ERR_EXIT;
    }
	
	result = ioctl(fd, SET_RXBW, cfg.rxbandwidth);
    if(result < 0)
    {
		goto ERR_EXIT;
    }
	
	result = ioctl(fd, SET_TXSF, cfg.txspreadfactor);
    if(result < 0)
    {
		goto ERR_EXIT;
    }
	
	result = ioctl(fd, SET_RXSF, cfg.rxspreadfactor);
    if(result < 0)
    {
		goto ERR_EXIT;
    }
	
	result = ioctl(fd, SET_TXCR, cfg.txcodingrate);
    if(result < 0)
    {
		goto ERR_EXIT;
    }
	
	result = ioctl(fd, SET_RXCR, cfg.rxcodingrate);
    if(result < 0)
    {
		goto ERR_EXIT;
    }
	
	result = ioctl(fd, SET_PAOUTPUT, cfg.paoutput);
    if(result < 0)
    {
		goto ERR_EXIT;
    }
	
	result = ioctl(fd, SET_TXPOWER, cfg.txpower);
    if(result < 0)
    {
		goto ERR_EXIT;
    }
	
	result = ioctl(fd, SET_PREAMBLELEN, cfg.preamblelen);
    if(result < 0)
    {
		goto ERR_EXIT;
    }
	
	result = ioctl(fd, SET_TXTIMEOUT, cfg.txtimeout);
    if(result < 0)
    {
		goto ERR_EXIT;
    }
	//start
	result = ioctl(fd, SET_RECVDATA, 1);
    if(result < 0)
    {
		goto ERR_EXIT;
    }
    //printf("val:%d\n", val);
ERR_EXIT:
    close(fd);
      
    return result;
}
