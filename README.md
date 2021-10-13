# cdmx

Linux kernel module that turns any serial tty to DMX port.
(C) 2021, GNU Public license v3.

Requirements: 
- h/w support for BREAK detection and non-standard baudrate.
- Linux kernel v5 (they change API at speed of light, this work done with v5.10.63 LTS)

Implemented for now:

- UART TX/RX
- DMXKing USBDMX512-A emulation (Enttec UsbPro compatible)
- SysFS access to port parameters
- variable port count, 1 to 256

TODO:
- implement RDM

Known issues:
- Oscilloscope could be useful to tweak MARK-After-Frame (MAF) time. By default it's set to 1000us, works fine on PPi3's PL011. If you don't get stable MAF, increase the value.
- some IOCTLs are implemented as stubs, so you won't be able to change TTY settings while using module. Exclusive file access used in OLA is also emulated with no real action done.
- RX overflow flag is not reported to host. Faulty frames are just dropped.
- First received DMX frame remains in read() queue and blocks the rest if no host is reading it. So the best practice is start OLA, let it detect CDMX, then attach UART to CDMX line discipline (LDISC).
- Kernel API for attaching line discipline has to be called from user-space, not kernel-space, so 'ldattach' from 'util-linux' package is what you need. Alternatively, any application could open UART file and call TIOCSETD ioctl.

Tests performed using OLA (https://github.com/OpenLightingProject/ola)

Dependencies:
- apt install build-essential util-linux ola sysvinit-utils
- apt install linux-headers-\`uname -r\` or build new kernel
- service olad stop; nano /etc/ola/ola-usbserial.conf (device_prefix = cdmx); make sure other OLA plugins or ModemManager don't use your UART.

Building the module:
- clone cdmx repository
- cd cdmx; edit Makefile to replace TEST_DEVICE with your local UART
- make all insert
- service olad start
- make attach

Further module hacking:
- It's recommended to build kernel from source, since out-of-stock images are not intended for debug.
- Enable CONFIG_DYNAMIC_DEBUG_CORE and CONFIG_MODULE_FORCE_UNLOAD in kernel .config
- cd cdmx/extra; make dyndbg

UART on RPi3:
- sudo systemctl disable hciuart
- sudo raspi-config, turn login shell off, serial hardware on
- edit /boot/config.txt, make sure it has "enable_uart=1" and "dtoverlay=pi3-disable-bt"
- use /dev/ttyAMA0

Kernel headers on RPi3:
- sudo apt install raspberrypi-kernel-headers - it should be fine
- building the kernel: https://www.raspberrypi.org/documentation/computers/linux_kernel.html
