#include <stdio.h> 
#include <stdbool.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/ioctl.h> 
#include <fcntl.h>  
#include <unistd.h>   //sleep  
#include <poll.h>  
#include <fcntl.h>  
#include "drv_lib.h"  
  
  
static unsigned char buffer[50]; 
int main(int argc ,char *argv[])  
{  
    int fd;  
    int flag; 
    int result = 0; 
    unsigned char val = 0;
    fd_set fds;
  
    fd = open("/dev/lora0",O_RDWR);  
    if (fd < 0)  
    {  
        printf("open error\n");  
    }  
  
    val = 1;
    result = ioctl(fd, LORA_IOC_WR_WORK, &val);
    if (result < 0)  
    {  
        printf("ioctl error\n");  
    }  

    while(1)  
    {
        int i = 0; 
        struct timeval timeout = {3, 0};
        FD_ZERO(&fds);
        FD_SET(fd, &fds);
                

        result = select(fd +1, &fds, &fds, NULL, &timeout);
	if(result)
        {
            result = read(fd, buffer, 50);
            printf("size:%d\n", result);
            if(result < 0)
                return 0;
            for(i=0; i<result;i++)
            {
                printf("0x%02x ", buffer[i]);
                if((i+1)%8 == 0)
                    printf("\n");
            }
            buffer[result] = 0;
            printf("%s\n", buffer);
        }
    }  
    return 0;  
}  

