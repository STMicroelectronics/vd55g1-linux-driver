ifneq ($(KERNELRELEASE),)
include Kbuild

else
KVERSION ?= `uname -r`
KDIR ?= /lib/modules/${KVERSION}/build
default:
	CONFIG_DRIVER_ST_VD55G1=m $(MAKE) -C $(KDIR) M=$$PWD

clean:
	CONFIG_DRIVER_ST_VD55G1=m $(MAKE) -C $(KDIR) M=$$PWD clean

endif
