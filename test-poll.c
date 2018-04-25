#include <stdio.h>  
#include <sys/types.h>  
#include <sys/stat.h>  
#include <fcntl.h>  
#include <unistd.h>   //sleep  
#include <poll.h>  
#include <signal.h>  
#include <fcntl.h>  
  
int fd;  
  
  
static unsigned char buffer[50]; 
int main(int argc ,char *argv[])  
{  
    int flag; 
    int result = 0; 
    fd_set fds;
    //signal(SIGIO,mysignal_fun);  
  
    fd = open("/dev/lora0",O_RDWR);  
    if (fd < 0)  
    {  
        printf("open error\n");  
    }  
  
    while(1)  
    {
        int i = 0; 
        struct timeval timeout = {3, 0};
        FD_ZERO(&fds);
        FD_SET(fd, &fds);
        
        /* 为了测试，主函数里，什么也不做 */  
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

