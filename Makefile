PROJECT_ROOT = $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
SRC_DIR = src
INC_DIR = include

MODNAME = cdmx
MOD_UNITS := $(SRC_DIR)/cdmx.o $(SRC_DIR)/enttec.o

obj-m += $(MODNAME).o
$(MODNAME)-objs := $(MOD_UNITS)

ccflags-y += -I$(PROJECT_ROOT)/include
ccflags-y += -DDYNAMIC_DEBUG_MODULE

CDMX_LD = 28
TEST_DEVICE = /dev/ttyAMA0
TEST_CHRDEV = /dev/cdmx000

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean

rm: 
	sudo rmmod $(MODNAME).ko

in:	
	sudo insmod $(MODNAME).ko cdmx_port_count=1
	
in4:
	sudo insmod $(MODNAME).ko

jo:
	sudo journalctl -f -o short-monotonic --no-hostname

at:
	sudo ldattach -d $(CDMX_LD) $(TEST_DEVICE)
	
de:
	sudo killall ldattach
	
t3:
	cat test/test.3.getparams >$(TEST_CHRDEV)
	cat $(TEST_CHRDEV) | hexdump -C

t6:  
	cat test/test.6.senddmx >$(TEST_CHRDEV)
t6over:
	cat test/test.6a.senddmx >$(TEST_CHRDEV)
	cat test/test.6c.senddmx >$(TEST_CHRDEV)
t6under:
	cat test/test.6a.senddmx >$(TEST_CHRDEV)
	cat test/test.6d.senddmx >$(TEST_CHRDEV)
	
t10:
	cat test/test.10.serial >$(TEST_CHRDEV)
	cat $(TEST_CHRDEV) | hexdump -C
	
t77:
	cat test/test.77.vendor >$(TEST_CHRDEV)
	cat $(TEST_CHRDEV) | hexdump -C

t78:
	cat test/test.78.getname >$(TEST_CHRDEV)
	cat $(TEST_CHRDEV) | hexdump -C

ls:
	sudo ls -lA /sys/devices/virtual/cdmx
	sudo ls -lA /sys/class/cdmx
	sudo ls -lA /sys/cdmx_p
	sudo ls -lA /dev/cdmx*

