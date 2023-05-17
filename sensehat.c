//
// Sense Hat Unchained
// A simple set of C functions to initialize and
// read data from the sensors on the Sense Hat
//
// These functions are not meant to be comprehensive
// but merely to be a simple example of using
// standard Linux file system drivers to talk to
// I2C devices.
//
// The Sense Hat consists of the following:
// 8x8 LED array mapped to a microcontroller at 0x46
// 5-way joystick mapped to the same microcontroller
// HTS221 humidity/temp sensor at 0x5F
// LPS25H pressure/temp sensor at 0x5C
// LSM9DS1 accel/gyro/mag mapped to 0x1C (mag) 0x6A (accel)
//
// Written by Larry Bank - 11/10/2017
// Copyright (c) 2017 BitBank Software, Inc.
// bitbank@pobox.com
//
// Modified for Ubuntu Linux with framebuffer and joystick event
// support by TC Wan - May 16, 2023
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//

#include <unistd.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

// See example of how to program the Framebuffer at
// https://kevinboone.me/linuxfbc.html?i=1
// The Framebuffer and Joystick Input occupies I2C Address 0x46

#include <linux/fb.h>
#include <sys/mman.h>

#define FBDEV "/dev/fb0"
static int framebufsize = 0;
static unsigned char *framebuffer = NULL;

// FB parameters
static int fb_width = 0;
static int fb_height = 0;
static int fb_bpp = 0;
static int fb_bytes = 0;

// Joystick support
// Code modified from experix my-hat.tgz2
// Copyright (C) 2004-2016 William Bayard McConnaughey

// Refer to the sense-hat board diagram Sense-HAT-V1_0.pdf
// The joystick is a SKRHABE010 (info. reference = http://www.alps.com/prod/
// info/E/HTML/MultiControl/Switch/SKRH/SKRHABE010.html) connected via
// COL_EN1..COL_EN5 to port D of the MCU. The MCU executes a cycle in which
// it reads the joystick state and then drives the LED columns (which are
// also connected to the COL_EN# lines; therefore these switch between input
// and output during the cycle).
//
// Driver sources are in https://github.com/raspberrypi/linux/blob/rpi-4.1.y/
// The joystick driver shows up in lsmod as rpisense_js and its source is
// drivers/input/joystick/rpisense_js.c
// It has no ioctls or fops. It #includes
// include/linux/mfd/rpisense/joystick.h
// include/linux/mfd/rpisense/core.h
//
// lsmod:  rpisense_core used by rpisense_js
// drivers/mfd/rpisense_core.c
//
// from /usr/include/linux/input.h:
// struct input_event {
//        struct timeval time;
//        __u16 type;
//        __u16 code;
//        __s32 value;
// };
// The joystick events and the corresponding keyboard events are:
//  keyboard       joystick                    code
//  ----------------------------------------------------
//  <right arrow>  toward ethernet             106  0x6a
//  <up arrow>     toward GPIO                 103  0x67
//  <left arrow>   toward camera connector     105  0x69
//  <down arrow>   toward nearest board edge   108  0x6c
//  <enter>        press down                   28  0x1c
//
// A brief joystick connection produces 4 events:
// type,code,value = 1,code,1; 0,0,0; 1,code,0; 0,0,0
// Holding the joystick or key produces these events ({block} repeats):
// type,code,value = 1,code,1; 0,0,0; { 1,code,2; 0,0,1; } 1,code,0; 0,0,0
// (Quick keyboard presses produce 6 events).

#include <linux/input.h>
#define INPUTDEV "/dev/input/event0"
static char jsname[256];


// I2C file handles
static int file_led = -1; // Framebuffer
static int file_js  = -1; // Joystick event Input
static int file_hum = -1; // humidity/temp sensor
static int file_pres = -1; // pressure sensor
static int file_acc = -1; // accelerometer/gyro
static int file_mag = -1; // magnetometer

static int i2cRead(int iHandle, unsigned char ucAddr, unsigned char *buf, int iLen);
static int i2cWrite(int iHandle, unsigned char ucAddr, unsigned char *buf, int iLen);
// humidity/temp calibration values
static int H0_rH_x2, H1_rH_x2, T0_degC_x8;
static int T1_degC_x8, H0_T0_OUT;
static int H1_T0_OUT, T0_OUT, T1_OUT;

//
// Opens file system handles to the I2C devices
//
int shInit(int iChannel)
{
unsigned char ucTemp[32];
char filename[32];
int retv;

	sprintf(filename, "/dev/i2c-%d", iChannel);
	if ((file_led = open(filename, O_RDWR)) < 0)
	{
		fprintf(stderr, "Failed to open the i2c bus; need to run as sudo?\n");
		return -1;
	}

	file_led = open(FBDEV, O_RDWR);
	if (file_led < 0)
	{
		fprintf(stderr, "Failed to open Framebuffer for LED Matrix\n");
		goto badexit;
	}

	// get screen dimensions
	struct fb_var_screeninfo vinfo;
	retv = ioctl (file_led, FBIOGET_VSCREENINFO, &vinfo);
	if (retv < 0)
	{
		fprintf(stderr, "ioctl() error for LED Matrix Framebuffer\n");
		goto badexit;
	}

	fb_width = vinfo.xres;
	fb_height = vinfo.yres;
	fb_bpp = vinfo.bits_per_pixel;
	fb_bytes = fb_bpp / 8;
	// printf("FB Width (%d) Height (%d) BPP (%d) fb_bytes (%d)\n", fb_width,fb_height,fb_bpp, fb_bytes);

	// Map screen into memory
	framebufsize = fb_width * fb_height * fb_bytes;

	framebuffer = mmap (0, framebufsize,
		PROT_READ | PROT_WRITE, MAP_SHARED, file_led, (off_t)0);
	// printf("Framebuffer Size = %d\n", framebufsize);
	if (framebuffer == NULL)
	{
		fprintf(stderr, "Failed to mmap() Framebuffer\n");
		goto badexit;
	}

	// Fill the LED with black
	memset(framebuffer, 0, framebufsize);

	file_js = open(INPUTDEV, O_RDONLY);
	if (file_js < 0)
	{
		fprintf(stderr, "Failed to open Joystick Event Input\n");
		goto badexit;
	}
	retv = ioctl(file_js, EVIOCGNAME(sizeof(jsname)), jsname);
	if (retv < 0)
	{
		fprintf(stderr, "ioctl() error for Joystick Event Input\n");
		goto badexit;
	}
	else
		fprintf(stderr, "Input device %s is %s\n", INPUTDEV, jsname);


	file_acc = open(filename, O_RDWR);
	if (ioctl(file_acc, I2C_SLAVE, 0x6a) < 0)
	{
		fprintf(stderr, "Failed to acquire bus for accelerometer\n");
		goto badexit;
	}
	file_mag = open(filename, O_RDWR);
	if (ioctl(file_mag, I2C_SLAVE, 0x1c) < 0)
	{
		fprintf(stderr, "Failed to acquire bus for magnetometer\n");
		goto badexit;
	}

	file_hum = open(filename, O_RDWR);
	if (ioctl(file_hum, I2C_SLAVE, 0x5f) < 0)
	{
		fprintf(stderr, "Failed to acquire bus for Humidity sensor\n");
		goto badexit;
	}
	file_pres = open(filename, O_RDWR);
	if (ioctl(file_pres, I2C_SLAVE, 0x5C) < 0)
	{
		fprintf(stderr, "Failed to aquire bus for Pressure sensor\n");
		goto badexit;
	}
	// Prepare humidity sensor
	i2cRead(file_hum, 0x10, ucTemp, 1); // AV_CONF
	ucTemp[0] &= 0xc0;
	ucTemp[0] |= 0x1b; // avgt=16, avgh=32
	i2cWrite(file_hum, 0x10, ucTemp, 1);

	i2cRead(file_hum, 0x20+0x80, ucTemp, 3); // get CTRL_REG 1-3
	ucTemp[0] &= 0x78; // keep reserved bits
	ucTemp[0] |= 0x81; // turn on + 1Hz sample rate
	ucTemp[1] &= 0x7c; // turn off heater + boot + one shot
	i2cWrite(file_hum, 0x20+0x80, ucTemp, 3); // turn on + set sample rate

	// Get the H/T calibration values
	i2cRead(file_hum, 0x30+0x80, ucTemp, 16);
	H0_rH_x2 = ucTemp[0];
	H1_rH_x2 = ucTemp[1];
	T0_degC_x8 = ucTemp[2];
	T1_degC_x8 = ucTemp[3];
	T0_degC_x8 |= ((ucTemp[5] & 0x3) << 8); // 2 msb bits
	T1_degC_x8 |= ((ucTemp[5] & 0xc) << 6);
	H0_T0_OUT = ucTemp[6] | (ucTemp[7] << 8);
	H1_T0_OUT = ucTemp[10] | (ucTemp[11] << 8);
	T0_OUT = ucTemp[12] | (ucTemp[13] << 8);
	T1_OUT = ucTemp[14] | (ucTemp[15] << 8);
	if (H0_T0_OUT > 32767) H0_T0_OUT -= 65536; // signed
	if (H1_T0_OUT > 32767) H1_T0_OUT -= 65536;
	if (T0_OUT > 32767) T0_OUT -= 65536;
	if (T1_OUT > 32767) T1_OUT -= 65536;

	// prepare pressure sensor
	ucTemp[0] = 0x90; // turn on and set 1Hz update
	i2cWrite(file_pres, 0x20, ucTemp, 1);
	
	// Init magnetometer
	ucTemp[0] = 0x48; // output data rate/power mode
	ucTemp[1] = 0x00; // default scale
	ucTemp[2] = 0x00; // continuous conversion
	ucTemp[3] = 0x08; // high performance mode
	i2cWrite(file_mag, 0x20+0x80, ucTemp, 4);

	// Init accelerometer/gyroscope
	ucTemp[0] = 0x60; // 119hz accel
	i2cWrite(file_acc, 0x20, ucTemp, 1);
	ucTemp[0] = 0x38; // enable gyro on all axes
	i2cWrite(file_acc, 0x1e, ucTemp, 1);
        ucTemp[0] = 0x28; // data rate + full scale + bw selection
// bits:        ODR_G2 | ODR_G1 | ODR_G0 | FS_G1 | FS_G0 | 0 | BW_G1 | BW_G0
// 0x28 = 14.9hz, 500dps
        i2cWrite(file_acc, 0x10, ucTemp, 1); // gyro ctrl_reg1

	return 1;

// problems opening the I2C handles/addresses
badexit:
	if (file_led != -1)
	{
		close(file_led);
		file_led = -1;
	}
	if (file_hum != -1)
	{
		close(file_hum);
		file_hum = -1;
	}
	if (file_pres != -1)
	{
		close(file_pres);
		file_pres = -1;
	}
	if (file_acc != -1)
	{
		close(file_acc);
		file_acc = -1;
	}
	if (file_mag != -1)
	{
		close(file_mag);
		file_mag = -1;
	}
	return 0;
} /* shInit() */

//
// Set a single pixel on the 8x8 LED Array
//
int shSetPixel(int x, int y, uint16_t color, int bUpdate)
{
int i;

	if (x >= 0 && x < 8 && y >= 0 && y < 8 && file_led >= 0)
	{
		i = (y*fb_width+x) * fb_bytes;
		framebuffer[i] = (color & 0xFF00) >> 8;
		framebuffer[i+1] = (color & 0xFF);
		return 1;
	}
	return 0;
} /* shSetPixel() */

int shGetAccel(int *Ax, int *Ay, int *Az)
{
unsigned char ucTemp[8];
int rc;

	rc = i2cRead(file_acc, 0x28+0x80, ucTemp, 6);
	if (rc == 6)
	{
		int x, y, z;

		x = ucTemp[0] + (ucTemp[1] << 8);
		y = ucTemp[2] + (ucTemp[3] << 8);
		z = ucTemp[4] + (ucTemp[5] << 8);
		// fix the signed values
		if (x > 32767) x -= 65536;
		if (y > 32767) y -= 65536;
		if (z > 32767) z -= 65536;
		*Ax = x; *Ay = y; *Az = z;
		return 1;
	}
	return 0;
} /* shGetAccel() */

int shGetGyro(int *Gx, int *Gy, int *Gz)
{
unsigned char ucTemp[8];
int rc;

	rc = i2cRead(file_acc, 0x18+0x80, ucTemp, 6);
	if (rc == 6)
	{
		*Gx = ucTemp[0] + (ucTemp[1] << 8);
		*Gy = ucTemp[2] + (ucTemp[3] << 8);
		*Gz = ucTemp[4] + (ucTemp[5] << 8);
		return 1;
	}
	return 0;
} /* shGetGyro() */

int shGetMagneto(int *Mx, int *My, int *Mz)
{
unsigned char ucTemp[8];
int rc;

	rc = i2cRead(file_mag, 0x28+0x80, ucTemp, 6);
	if (rc == 6)
	{
		int x, y, z;
		x = ucTemp[0] + (ucTemp[1] << 8);
		y = ucTemp[2] + (ucTemp[3] << 8);
		z = ucTemp[4] + (ucTemp[5] << 8);
		// fix signed values
		if (x > 32767) x -= 65536;
		if (y > 32767) y -= 65536;
		if (z > 32767) z -= 65536;
		*Mx = z; *My = y; *Mz = z;
		return 1;
	}
	return 0;
} /* shGetMagneto() */

//
// Returns the air pressure in hPa and temp in C * 10 (18.1 = 181)
//
// 1 = pressure successfully read
// 0 = failed to read pressure
//
int shGetPressure(int *Pressure, int *Temp)
{
unsigned char ucTemp[8];
int rc, P, T;

	if (file_pres != -1 && Pressure != NULL)
	{
		rc = i2cRead(file_pres, 0x28+0x80, ucTemp, 5);
		if (rc == 5)
		{
			P = ucTemp[0] + (ucTemp[1]<<8) + (ucTemp[2]<<16); 
			*Pressure = P / 4096; //hPa
			T = ucTemp[3] + (ucTemp[4] << 8);
			if (T > 32767) T -= 65536; // twos compliment
			T = 425 + (T / 48); // 42.5 + T value/480
			*Temp = T;
		}
		return 1;	
	}
	return 0;
} /* shGetPressure() */

int shGetTempHumid(int *Temp, int *Humid)
{
unsigned char ucTemp[4];
int rc;
int H_T_out, T_out, T0_degC, T1_degC;
int H0_rh, H1_rh;
int tmp;

	rc = i2cRead(file_hum, 0x28+0x80, ucTemp, 4);
	if (rc == 4)
	{
		H_T_out = ucTemp[0] + (ucTemp[1] << 8);
		T_out = ucTemp[2] + (ucTemp[3] << 8);
		if (H_T_out > 32767) H_T_out -=65536;
		if (T_out > 32767) T_out -= 65536;
		T0_degC = T0_degC_x8 / 8;
		T1_degC = T1_degC_x8 / 8;
		H0_rh = H0_rH_x2 / 2;
		H1_rh = H1_rH_x2 / 2;
		tmp = (H_T_out - H0_T0_OUT) * (H1_rh - H0_rh)*10;
		*Humid = tmp / (H1_T0_OUT - H0_T0_OUT) + H0_rh*10;
		tmp = (T_out - T0_OUT) * (T1_degC - T0_degC)*10;
		*Temp = tmp / (T1_OUT - T0_OUT) + T0_degC*10;
		return 1;
	}
	return 0; // not ready
} /* shGetTempHumid() */

void shShutdown(void)
{
	// Blank the LED array
	memset(framebuffer, 0, framebufsize);
	if (munmap(framebuffer, framebufsize) < 0)
	{
		fprintf(stderr, "Failed to munmap() Framebuffer\n");
	}

	// Close all I2C file handles
	if (file_led != -1) close(file_led);
	if (file_hum != -1) close(file_hum);
	if (file_pres != -1) close(file_pres);
	if (file_acc != -1) close(file_acc);
	if (file_mag != -1) close(file_mag);
	file_led = file_hum = file_pres = file_acc = file_mag = -1;
} /* shShutdown() */

static int i2cRead(int iHandle, unsigned char ucAddr, unsigned char *buf, int iLen)
{
int rc;

	rc = write(iHandle, &ucAddr, 1);
	if (rc == 1)
	{
		rc = read(iHandle, buf, iLen);
	}
	return rc;
} /* i2cRead() */

int i2cWrite(int iHandle, unsigned char ucAddr, unsigned char *buf, int iLen)
{
unsigned char ucTemp[512];
int rc;

	if (iLen > 511 || iLen < 1 || buf == NULL)
		return -1; // invalid write

	ucTemp[0] = ucAddr; // send the register number first 
	memcpy(&ucTemp[1], buf, iLen); // followed by the data
	rc = write(iHandle, ucTemp, iLen+1);
	return rc-1;

} /* i2cWrite() */

unsigned char shReadJoystick(void)
{
int rd;
unsigned char retval = 0;
struct input_event jsev;

    rd = read(file_js, &jsev, sizeof(struct input_event));
    if (rd < 0)
    {
		fprintf(stderr, "Failed to read Joystick Input\n");
    	return 0;
    }
    else
    {
    	fprintf(stderr, "JS Input %d bytes; ev.type=%d ev.code=%d ev.value=%d\n",
    		      rd, jsev.type, jsev.code, jsev.value);

    	// we need a state machine to track input events
    	switch (jsev.type) {
    	case 0:
    		// ignore event delimiter marker
        	retval = 0;
    		break;
    	case 1:
    		if ((jsev.value == 0)|| (jsev.value == 2))
    			// Release, or repeat (hold)
   				retval = jsev.code;
    		break;
    	default:
    		retval = 0;
    		break;
    	}

    	return retval;
    }

} /* shReadJoystick() */
