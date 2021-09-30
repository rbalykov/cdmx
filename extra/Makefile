PROJECT_ROOT = $(dir $(abspath $(lastword $(MAKEFILE_LIST))))

install:
	sudo ln -s $(PROJECT_ROOT)cdmx-modprobe.conf /etc/modprobe.d/
	sudo ln -s $(PROJECT_ROOT)cdmx-load.conf /etc/modules-load.d/
	sudo mkdir /lib/modules/`uname -r`/kernel/drivers/char
	sudo cp $(PROJECT_ROOT)../cdmx.ko /lib/modules/`uname -r`/kernel/drivers/char
	sudo depmod

uninstall:
	sudo rm /etc/modprobe.d/cdmx-modprobe.conf
	sudo rm /etc/modules-load.d/cdmx-load.conf
	sudo rm /lib/modules/`uname -r`/kernel/drivers/char/cdmx.ko

dyndbg:
	sudo ln -s $(PROJECT_ROOT)cdmx-modprobe.conf /etc/modprobe.d/
