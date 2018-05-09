#include <stdio.h>
#include <stdbool.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include "sx127xlib.h"
int main(void)
{

    int fd = 0;
    int result = 0;
    unsigned int val = 1;
    fd = open("/dev/lora0", O_RDWR);
    if(fd < 0)
    {
        printf("open error\n");
        goto ERR_EXIT;
    }
   
    result = ioctl(fd, START_CAD, &val);
    if(result < 0)
    {
		goto ERR_EXIT;
    }
    //val =1, detected signal on the channel
	if(val == 1)
	{
		printf("channel is free\n", result);
	}else{
		printf("channel is not free\n", result);
	}
    
	//if we need receive now ,we must do the following
	result = ioctl(fd, SET_RECVDATA, 1);
	if(result < 0)
	{
		goto ERR_EXIT;
	}
ERR_EXIT:
    close(fd);
      
    return result;
}
