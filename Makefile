obj-m += mx4_pic.o
mx4_pic-y = mx4-core.o mx4-gpio.o mx4-polled.o mx4-attributes.o 

PWD := $(shell pwd)

EXTRA_CFLAGS += -I$(KERNEL_SRC)/include -I$(KERNEL_SRC)/include/uapi -Wno-format

DESTDIR ?= $(INSTALL_MOD_PATH)

modules all:
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD) modules

modules_install install: all
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD) modules_install INSTALL_MOD_PATH=$(DESTDIR)

clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD) clean

