#
# This file supposed to hold together install/uninstall reminders,
# so is't rough collection of commands, that can run batch.
#
# To use it, you have to ignore possible errors in automated execution
# and maybe fix that errors manually.
#
#
# modprobe.d: options used on module loading, 
# port count could also be specified there.
#
# modules-load.d: directive to load module on kernel boot.
# Requires .ko file located in /lib/modules/... and depmod
# 

PROJECT_ROOT = $(dir $(abspath $(lastword $(MAKEFILE_LIST))))

#
# make -i install
#
install:
	sudo ln -s $(PROJECT_ROOT)cdmx-modprobe.conf /etc/modprobe.d/
	sudo ln -s $(PROJECT_ROOT)cdmx-load.conf /etc/modules-load.d/
	sudo mkdir /lib/modules/`uname -r`/kernel/drivers/tty
	sudo cp $(PROJECT_ROOT)../cdmx.ko /lib/modules/`uname -r`/kernel/drivers/char
	sudo depmod
#
#
#
uninstall:
	sudo rm /etc/modprobe.d/cdmx-modprobe.conf
	sudo rm /etc/modules-load.d/cdmx-load.conf
	sudo rm /lib/modules/`uname -r`/kernel/drivers/tty/cdmx.ko
	sudo depmod

#
# install Dynamic Debug directive
#
dyndbg:
	sudo ln -s $(PROJECT_ROOT)cdmx-modprobe.conf /etc/modprobe.d/
