#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>

int main(void)
{

    int fd = 0;
    int result = 0;
    fd = open("/dev/lora0", O_RDWR);
    if(fd < 0)
    {
        printf("open error\n");
        goto ERR_EXIT;
    }

ERR_EXIT:
    close(fd);
      
    return result;
}
