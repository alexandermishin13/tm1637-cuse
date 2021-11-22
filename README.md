# tm1637-cuse

Cuse based userland device driver for tm1637 4 or 6 digits displays.

## About
The ```cuse```-based driver is an alternative one to a kernel driver
[tm1637-kmod](https://gitlab.com/alexandermishin13/tm1637-kmod).
It does almost the same as a kernel driver but it does not requires kernel
sources to build it, making it easier to use on an Arm PC. For its work a
loaded kernel module ```cuse.ko``` is required.

## Description
Creates a character device on each ```-d``` option in a ***/dev/tm1637/*** tree.
Each device binded to its own two pins ```scl``` and ```sda```. It uses a bit
banged manipulation with this pins to transmit data to the ```tm1637```
display connected to them.

You can simply send a string from a shell to the device for output it onto the
display bonded to it:
- `12:30`;
- `12 30`;
- `##:##`;
- `## ##`;
- `1234`;
- `3.14` (If You have a decimal dots type of a display);
- `01.01.21` (If You have a 6 digits display);
- `----`;
- `    `.

```shell
% echo "1234" > /dev/tm1637/0
```

It can undestand a command string like:
```shell
% printf $"\x80" > /dev/tm1637/0
% printf $"\x8b" > /dev/tm1637/0
% printf $"\x44\xc1\x86" > /dev/tm1637/0
% printf $"\x40\xc0\x06\x86\x06\x06" > /dev/tm1637/0
```
(See a ```tm1637``` chip datasheet for construct a command string)

Also it support ```ioctl``` calls defined in ```/usr/local/include/tm1637d.h```
as:
```c
struct tm1637_clock_t {
    int tm_min;
    int tm_hour;
    bool tm_colon;
};

#define TM1637IOC_CLEAR			_IO('T', 1)
#define TM1637IOC_OFF			_IO('T', 2)
#define TM1637IOC_ON			_IO('T', 3)
#define TM1637IOC_SET_BRIGHTNESS	_IOW('T', 11, uint8_t)
#define TM1637IOC_SET_CLOCKPOINT	_IOW('T', 12, uint8_t)
#define TM1637IOC_SET_CLOCK		_IOW('T', 14, struct tm1637_clock_t)
```

## Installation

You do not need a kernel sources for make a ```cuse```-based driver. It is
just a daemon You can run as a system service after it is installed.

Clone that ```git``` project to ***~/src/*** directory, change a current
directory to the projects new one, build and install the daemon.

```shell
% mkdir -p ~/src
% cd ~/src
% git clone https://gitlab.com/alexandermishin13/tm1637-cuse.git
% cd ./tm1637-cuse
% make
% sudo make install
```

You can wish to copy a file ```./rc.conf.d/tm1637d``` to a directory
```/usr/local/etc/rc.conf.d/tm1637/``` and change it to suit Your needs.
Then load a system module ```cuse.ko``` and  run the service:

```shell
% kldload cuse
% service tm1637d start
```

You can add the following line to the file ```/boot/loader.conf``` in
order for the module to load automatically at system startup:
```
cuse_load="YES"
```

One or more devices will be created into ```/dev/tm1637/``` directory.
You can change permits to them by change a file ```/etc/devfs.rules```
and restart a ```devfs``` service. Read about how to do it:

```shell
% man 5 devfs.rules
```

## Status
Time tested and found ready to use.
