PROJECT_ROOT = $(dir $(abspath $(lastword $(MAKEFILE_LIST))))

MODNAME = kdmx
MOD_UNITS = cdmx.o enttec.o
#MODNAME = kobject-example
#MODNAME = kset-example

obj-m += $(MODNAME).o
$(MODNAME)-objs := $(MOD_UNITS)

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
	
inp:
	sudo insmod $(MODNAME).ko cdmx_port_count=6

jo:
	sudo journalctl -f -o short-monotonic --no-hostname

# seems to be broken	
# at2:
#	sudo ldattach -d8n2 -s250000 -i BRKINT $(CDMX_LD) $(TEST_DEVICE)

at:
	sudo ldattach -d $(CDMX_LD) $(TEST_DEVICE)
	
de:
	sudo killall ldattach

	
t3:
	cat test/test.3.getparams >$(TEST_CHRDEV)
t6:  
	cat test/test.6.senddmx >$(TEST_CHRDEV)
t6over:
	cat test/test.6a.senddmx >$(TEST_CHRDEV)
	cat test/test.6c.senddmx >$(TEST_CHRDEV)
t6under:
	cat test/test.6a.senddmx >$(TEST_CHRDEV)
	cat test/test.6d.senddmx >$(TEST_CHRDEV)
t78:
	cat test/test.78.getname >$(TEST_CHRDEV)


# /sys/devices/virtual/cdmx
# /sys/class/cdmx
# /sys/module/dmx_udev/parameters/port_count


ls:
	sudo ls -lA /sys/devices/virtual/cdmx
	sudo ls -lA /sys/class/cdmx
	sudo ls -lA /sys/cdmx_p
	sudo ls -lA /dev/cdmx*
	

name:
	cat /sys/cdmx_p/tty00
	sudo echo /dev/ttyUSB0 >/sys/cdmx_p/tty00
	cat /sys/cdmx_p/tty00
	sudo echo /dev/ttyUSB0 >/sys/cdmx_p/tty00
	
	