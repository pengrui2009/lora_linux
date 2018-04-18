INSTALLDIR := /mnt/hgfs/share/ko
STAGING_DIR := /home/linux/openwrt/openwrt-offical/staging_dir
#CROSS_COMPILE := 

KERNELDIR := /home/linux/openwrt/gateway/linux_3.14/
PWD := $(shell pwd)

obj-m := lora1278.o
lora1278-objs := sx1278.o drv_sx1278.o

all:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules ARCH=arm CROSS_COMPILE=arm-openwrt-linux-
	cp lora1278.ko $(INSTALLDIR)

clean:
	-rm -rf *.o *.ko *.mod.c .*.cmd .tmp_versions Module.symvers modules.order $(INSTALLDIR)/spidev.ko


