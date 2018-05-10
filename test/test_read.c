#include <stdio.h> 
#include <stdbool.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/ioctl.h> 
#include <fcntl.h>  
#include <unistd.h>   //sleep  
#include <poll.h>  
#include <fcntl.h>  
#include "sx127xlib.h"  
  

static unsigned char buffer[50];


int main(int argc ,char *argv[])  
{  
    int fd;  
    int flag; 
    int result = 0; 
    unsigned char val = 0;
	struct sx127x_pkt *pkt_ptr = NULL;
    fd_set fds;
  
    fd = open("/dev/lora0",O_RDWR);  
    if (fd < 0)  
    {  
        printf("open error\n");  
    }  
  
    val = 1;
    result = ioctl(fd, SET_RECVDATA, val);
    if (result < 0)  
    {  
        printf("ioctl error\n");  
    }  

    while(1)  
    {
        int i = 0;
		int len = 0;
        struct timeval timeout = {3, 0};
        FD_ZERO(&fds);
        FD_SET(fd, &fds);
                

        result = select(fd +1, &fds, &fds, NULL, &timeout);
		if(result)
        {
            result = read(fd, buffer, 50);
            if(result < sizeof(struct sx127x_pkt))
			{
				printf("recv data error\n");
                return 0;
			}
			
			len = result;
			while(len)
			{
				pkt_ptr = (struct sx127x_pkt *)buffer;
				len -= pkt_ptr->len;
				
				printf("pkt header len:%d pkt data len:%d\n", pkt_ptr->hdrlen, pkt_ptr->payloadlen);
				printf("recv data[0]:0x%02x\n", buffer[pkt_ptr->hdrlen]);
			}
        }
    }  
    return 0;  
}  

