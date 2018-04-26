#include <stdio.h>
#include <stdbool.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include "drv_lib.h"
int main(void)
{

    int fd = 0;
    int result = 0;
    unsigned char val = 1;
    fd = open("/dev/lora0", O_RDWR);
    if(fd < 0)
    {
        printf("open error\n");
        goto ERR_EXIT;
    }
   
    result = ioctl(fd, LORA_IOC_CAD, &val);
    printf("val:%d\n", val);
ERR_EXIT:
    close(fd);
      
    return result;
}
