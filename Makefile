PROJECT_ROOT = $(dir $(abspath $(lastword $(MAKEFILE_LIST))))

MODNAME = cdmx
#MODNAME = kobject-example
#MODNAME = kset-example

obj-m += $(MODNAME).o

N_DMX = 28
TEST_DEVICE = /dev/ttyUSB0
SEPR = "---------------------------------------------------------------"

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
	
	