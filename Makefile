INSTALLDIR := /mnt/hgfs/share/ko
#STAGING_DIR := /home/linux/openwrt/openwrt-offical/staging_dir
#CROSS_COMPILE := 

#KERNELDIR := /home/linux/openwrt/gateway/linux_3.14/
KERNELDIR := /home/pengrui/data/kernel/linux-3.14.52/
PWD := $(shell pwd)

obj-m := lora.o
lora-objs := sx1278.o drv_lora.o

all:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf-
	cp lora.ko $(INSTALLDIR)

clean:
	-rm -rf *.o *.ko *.mod.c .*.cmd .tmp_versions Module.symvers modules.order $(INSTALLDIR)/spidev.ko


