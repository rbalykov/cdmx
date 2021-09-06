PROJECT_ROOT = $(dir $(abspath $(lastword $(MAKEFILE_LIST))))

MODNAME = cdmx

obj-m += $(MODNAME).o

N_DMX = 29
TEST_DEVICE = /dev/ttyUSB0
SEPR = "---------------------------------------------------------------"

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean

rm: 
	sudo rmmod $(MODNAME).ko

in:	
	sudo insmod $(MODNAME).ko
	
inp:
	sudo insmod $(MODNAME).ko port_count=6

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
	
	