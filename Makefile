#+++                                                              +++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++                                                              +++
#+++   COPYRIGHT (c)  HostMobility AB                             +++
#+++                                                              +++
#+++ The copyright to the computer Program(s) herein is the       +++
#+++ property of HostMobility, Sweden. The program(s) may be      +++
#+++ used and or copied only with the written permission of       +++
#+++ HostMobility, or in accordance with the terms and            +++
#+++ conditions stipulated in the agreement contract under        +++
#+++ which the program(s) have been supplied                      +++
#+++                                                              +++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++                                                              +++


EXTRA_CFLAGS += $(DEBFLAGS)

mx4_pic-objs := mx4-core.o mx4-gpio.o mx4-polled.o mx4-attributes.o
obj-m	:= mx4_pic.o

modules all:
	$(MAKE) -C $(KERNEL_DIR) SUBDIRS=$(PWD) modules

clean:
	rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c .tmp_versions *.order *.symvers
