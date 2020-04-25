ifneq ($(KERNELRELEASE),)
# kbuild part of makefile
obj-m  := clocksource-stm32.o

else
# normal makefile
KDIR = ../build

.PHONY: default clean

default:
	$(MAKE) -C $(KDIR) M=$$PWD ARCH=arm CROSS_COMPILE=arm-linux-gnu-

clean:
	$(MAKE) -C $(KDIR) M=$$PWD ARCH=arm CROSS_COMPILE=arm-linux-gnu- clean

endif
