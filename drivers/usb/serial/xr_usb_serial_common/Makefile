obj-m := xr_usb_serial_common.o

KERNELDIR ?= /lib/modules/$(shell uname -r)/build
PWD       := $(shell pwd)

EXTRA_CFLAGS	:= -DDEBUG=0

all:
	$(MAKE) ARCH=arm64 CROSS_COMPILE=aarch64-buildroot-linux-gnu- -C $(KERNELDIR) M=$(PWD)

modules_install:
	$(MAKE) ARCH=arm64 CROSS_COMPILE=aarch64-buildroot-linux-gnu- -C $(KERNELDIR) M=$(PWD) modules_install

clean:
	rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c .tmp_versions vtty
