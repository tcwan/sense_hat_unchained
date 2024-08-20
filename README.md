Sense Hat Unchained
-------------------
A C library to communicate with the LEDs and sensors on the Raspberry Pi
Sense Hat.
![Orange Pi Lite](/animated.gif?raw=true "Sense Hat set free")

Copyright (c) 2017 BitBank Software, Inc.
written by Larry Bank
bitbank@pobox.com

The reason for 'unchained' is that the Sense Hat is often assumed to be an
accessory which only works on RPI hardware with RPI provided software.
In actuality, the Sense Hat is just a collection of I2C devices that can work
with any computer which has an I2C bus. The purpose of this library is to
provide a simple example of how to work with the Sense Hat without needing
any special software. The code here is not complete in that it doesn't expose
all modes/options of the sensors. It does provide a minimal example of how to 
initialize and read data from them. I have tested the Sense Hat on Raspberry
Pi computers (1B/3B/ZeroW) and Orange Pi boards.

KMod Compatible Version
-----------------------
Aug 20 2024

Copyright (c) 2023-2024 TC Wan

Updated for SenseHat kmod support found in most recent kernels.

Tested with Ubuntu 24.04 on RPi 5 
(should be able to autodetect LED Matrix and Joystick on RPi 4 as well).

### Device Names
This library assumes that the LED Matrix is mapped to /dev/fbX
and the Joystick is mapped to /dev/input/eventX

The autodetect logic came from the Astro-Pi Python code.

## Ubuntu Configuration
In order to program the SenseHat with Ubuntu, you will need to configure Ubuntu 
as follows:

### Force RPi SenseHat Detection
(This is needed for Ubuntu 24.04)
```
$ sudo vi /boot/firmware/config.txt
[all]
# Force RPi SenseHat Detection
dtoverlay=rpi-sense
```

### Create udev rules
To avoid having to run the program using sudo, we enable global access to the 
i2c device.
```
$ sudo vi /etc/udev/rules.d/99-i2c.rules
KERNEL=="i2c-[0-7]",MODE="0666"
```
### Blacklist SenseHat Industrial I/O Drivers
These drivers take over the I2C bus and prevents the library from talking to 
the devices directly.

```
$ sudo vi /etc/modprobe.d/blacklist-sensehat-industrialio.conf
blacklist st_magn_spi
blacklist st_pressure_spi
blacklist st_sensors_spi
blacklist st_pressure_i2c
blacklist st_magn_i2c
blacklist st_pressure
blacklist st_magn
blacklist st_sensors_i2c
blacklist st_sensors
blacklist hts221_i2c
# Don't blacklist industrialio as it is used by other drivers
# blacklist industrialio_triggered_buffer
# blacklist industrialio
```

### Verify I2C Configuration
To check that the SenseHat is accessible via I2C at the default addresses.

See [SenseHat Info](https://pinout.xyz/pinout/sense_hat#).


```
$ sudo apt-get install i2c-tools
$ i2cdetect -y 1
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:                         -- -- -- -- -- -- -- -- 
10: -- -- -- -- -- -- -- -- -- -- -- -- 1c -- -- -- 
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
40: -- -- -- -- -- -- UU -- -- -- -- -- -- -- -- -- 
50: -- -- -- -- -- -- -- -- -- -- -- -- 5c -- -- 5f 
60: -- -- -- -- -- -- -- -- -- -- 6a -- -- -- -- -- 
70: -- -- -- -- -- -- -- -- 

UU: Reserved by kernel drivers (rpisense_fb, rpisense_js, rpisense_core)
```
