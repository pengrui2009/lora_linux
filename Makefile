INSTALLDIR := /mnt/hgfs/share/ko
#STAGING_DIR := /home/linux/openwrt/openwrt-offical/staging_dir
#CROSS_COMPILE := 

#KERNELDIR := /home/linux/openwrt/gateway/linux_3.14/
KERNELDIR := /home/linux/imx6/linux-3.14.52/
PWD := $(shell pwd)

obj-m := sx127x.o
#lora-objs := sx1278.o drv_lora.o

all:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf-
	#cp lora.ko $(INSTALLDIR)
	#cd test&&arm-linux-gnueabihf-gcc -o test_poll test_poll.c -I../
	#cd test&&arm-linux-gnueabihf-gcc -o test_send test_send.c -I../
	#cd test&&arm-linux-gnueabihf-gcc -o test_cad test_cad.c -I../
	#cp test/test_poll test/test_send test/test_cad $(INSTALLDIR)
clean:
	-rm -rf *.o *.ko *.mod.c .*.cmd .tmp_versions Module.symvers modules.order $(INSTALLDIR)/spidev.ko


