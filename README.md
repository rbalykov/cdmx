# cdmx

Linux kernel module that turns any serial tty to DMX port.
(C) 2021, GNU Public license v3.

Requirements: 
- h/w support for BRKINT and non-standard baudrate.
- Linux kernel v5 (v5.10.63-longterm is recommended)

Implemented for now:
- UART RX
- /dev/cdmx00x emulates DMXKing USBDMX512-A
- /sys/cdmx/port00x/* files provide r/w access to port parameters
- variable port count, min=1, max=256

Tests performed using OLA (https://github.com/OpenLightingProject/ola)

TODO:
- implement TX
- implement RDM

Building and testing:
- apt install build-essential
- apt install linux-headers-`uname -r` or build new kernel
- apt install ola; service olad stop
- clone cdmx repository
- edit Makefile to replace TEST_DEVICE with your local TTY
- cd cdmx; make all in
- nano /etc/ola/ola-usbserial.conf (device_prefix = cdmx)
- xterm + make jo;
- service olad start
- xterm + make at
- ola_dev_info; ola_patch; ola_dmxmonitor; ola_dmxconsole

UART on RPi3:
- sudo systemctl disable hciuart
- sudo raspi-config, turn login shell off, serial hardware on
- edit /boot/config.txt, make sure it has "enable_uart=1" and "dtoverlay=pi3-disable-bt"
- use /dev/ttyAMA0

Kernel headers on RPi3:
- sudo apt install raspberrypi-kernel-headers - it should be fine
- building the kernel: https://www.raspberrypi.org/documentation/computers/linux_kernel.html
