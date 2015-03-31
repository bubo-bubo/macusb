obj-m		:= macusb.o

KERNELDIR ?= /lib/modules/$(shell uname -r)/build
PWD       := $(shell pwd)
#EXTRA_CFLAGS += -I../lib/include

all:
	$(MAKE) -C $(KERNELDIR) M=$(PWD)

clean:
	rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c .tmp_versions

install:
	cp macusb.ko /lib/modules/$(shell uname -r)/kernel/drivers/usb/misc/
	depmod
	#su sgeadmin -c "cp -a macusb.h /net/systemlv/sw/fe/include"
