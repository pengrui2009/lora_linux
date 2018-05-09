#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include "sx127xlib.h"

int main(void)
{
    int fd = 0;
    unsigned char val = 12;
    //unsigned char sendbuf[] = {'0', '1', 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20};
    unsigned char sendbuf[] = {'0', '1', '2', '3', '4'};//, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20};
    int result = 0;
    fd = open("/dev/lora0", O_RDWR);
    if(fd < 0)
    {
        printf("open error\n");
        goto ERR_EXIT;
    }

    result = write(fd, sendbuf, sizeof(sendbuf));
    if(result < sizeof(sendbuf))
    {
        printf("send error\n");
        goto ERR_EXIT;
    }
    //write(fd, sendbuf, sizeof(sendbuf));
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
