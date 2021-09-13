# cdmx

Linux kernel module that turns any serial tty to DMX port.
(C) 2021, GNU Public license v3.

Requirements: 
- h/w support for BRKINT and non-standard baudrate.
- Linux kernel v5 (v5.10.63-longterm is recommended)

Implemented for now:
- /dev/cdmx00x emulates DMXKing USBDMX512-A
- /sys/cdmx/port00x/* files provide r/w access to port parameters
- module parameter cdmx_port_count, min=1, max=256

Tested using OLA (https://github.com/OpenLightingProject/ola)
It needs '--no-use-epoll' option due using epoll() that is still not supported.

TODO:
- add epoll support
- implement UART i/o
- add RDM support

Building:
- apt install build-essential
- apt install linux-headers-`uname -r` or build new kernel
- clone repository; make; make in; make jo; make rm;
- run olad --no-use-epoll to test
- cat test/test.3.getparams | hexdump -C to test
