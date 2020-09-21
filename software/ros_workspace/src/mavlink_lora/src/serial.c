/****************************************************************************
# Serial driver
# Copyright (c) 2004-2018 Kjeld Jensen <kjeld@cetus.dk>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
#****************************************************************************
# Project: Serial driver for linux
# Author: Kjeld Jensen <kjeld@cetus.dk>
#
# 2004-07-06 KJ Original vesion
# 2004-08-09 KJ Header changed
# 2004-08-11 KJ Added support for USBSerial device under OSX
# 2005-02-05 KJ Added serial_flush()
# 2005-07-23 KJ Changed global.h to platform.h
# 2008-04-17 KJ Added ser_linux_baudrate, changed prefix to serb, moved info
#               from platform.h to serial.h
# 2010-03-11 KJ Added support for baudrates 460800-4000000, removed TARGET
#               defines as init has been modified
# 2011-02-06 KJ Released under MIT license
# 2011-03-28 KJ Solved a minor problem at this header causing a compiler warning.
# 2018-06-13 KJ Released under BSD license, removed OSX support, removed
#               ser_linux_baudrate function
****************************************************************************/
/* system includes */
#include <time.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/signal.h>
#include <sys/types.h>
#include <stdio.h>

/***************************************************************************/
/* application includes */
#include "../include/serial.h"

/***************************************************************************/
int ser_open (int *serRef, struct termios *oldtio, char *deviceName, long baudRate)
{
	int status = 0;
	struct termios newtio;

	/*open the device to be non-blocking*/
	*serRef = open(deviceName, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (*serRef >= 0)
	{
		/* save current port settings*/
		if (oldtio != NULL)
			tcgetattr(*serRef, oldtio);

		/* set new port settings*/
    /*
      BAUDRATE: Set bps rate. You could also use cfsetispeed and cfsetospeed.
      CRTSCTS : output hardware flow control (only used if the cable has
                all necessary lines. See sect. 7 of Serial-HOWTO)
      CS8     : 8n1 (8bit,no parity,1 stopbit)
      CLOCAL  : local connection, no modem contol
      CREAD   : enable receiving characters
      CSTOPB  : stop bit
      PARENB  : parity bit
    */
		newtio.c_cflag = CS8 | CLOCAL | CREAD;
		/* newtio.c_cflag |= CSIZE; */

		/* cfsetspeed must be called after setting newtio.c_flag */
		cfsetspeed(&newtio, baudRate); 

		newtio.c_iflag = IGNPAR;
		newtio.c_oflag = 0;
		newtio.c_lflag = 0;
		newtio.c_cc[VMIN]=0;  /* non-blocking read*/
		newtio.c_cc[VTIME]=0;
		ser_flush (*serRef);
		tcsetattr(*serRef,TCSANOW,&newtio);
	}
	else
		status = 1;

	return status;
}
/***************************************************************************/
int ser_send (int serRef, void *buffer, int numBytes)
{
	return write (serRef, buffer, numBytes);
}
/***************************************************************************/
int ser_receive (int serRef, void *buffer, int numBytes)
{
	return read(serRef, buffer, numBytes);
}
/***************************************************************************/
void ser_flush (int serRef)
{
	tcflush(serRef, TCIFLUSH);
}
/***************************************************************************/
void ser_close (int serRef, struct termios oldtio)
{
	/* restore old port settings */
	tcsetattr(serRef, TCSANOW, &oldtio);

	/* close the com port*/
	close(serRef);
}
/***************************************************************************/
