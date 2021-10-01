#
# General setup
#
PROJECT_ROOT = $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
SRC_DIR = src
INC_DIR = include
MODNAME = cdmx

MOD_UNITS := $(SRC_DIR)/cdmx.o $(SRC_DIR)/enttec.o
obj-m += $(MODNAME).o
$(MODNAME)-objs := $(MOD_UNITS)
ccflags-y += -I$(PROJECT_ROOT)/include

#
# if have kernel with CONFIG_DYNAMIC_DEBUG_CORE 
# or CONFIG_DYNAMIC_DEBUG enabled, next line is must-have.
#
# Without DYNDBG suport, messages will be lost, then
# comment out the line to use printk.
#
ccflags-y += -DDYNAMIC_DEBUG_MODULE -DDEBUG

#
# without USE_SIMPLE_DYNDBG '+mflt' (module, file, line, time) options
# are hard-coded to message text, otherwise they are generated on-the-fly
#
ccflags-y += -DUSE_SIMPLE_DYNDBG

# Line Discipline ID
CDMX_LD = 28

# UART tty that performs actual i/o
TEST_DEVICE = /dev/ttyAMA0

# Emulation device
TEST_CHRDEV = /dev/cdmx000

# Typical line from kernel manual, nothing changed
all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

# Typical line from kernel manual, nothing changed
clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean

# Remove module
rm: 
	sudo rmmod $(MODNAME).ko
	
# Insert module with single port
in:	
	sudo insmod $(MODNAME).ko cdmx_port_count=1

# Insert module
in4:
	sudo insmod $(MODNAME).ko

# Log only kernel and cdmx messages
log:
	sudo journalctl -f -o short-monotonic --no-hostname -tkernel -t$(MODNAME)

# Attach UART to CDMX
at:
	sudo ldattach -d $(CDMX_LD) $(TEST_DEVICE)

# detach UART
de:
	sudo killall ldattach

# Make-targets aliases
attach: at
detach: de
insert: in
remove: rm

#
# Various testing stuff
#
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

################################################################################
################################################################################