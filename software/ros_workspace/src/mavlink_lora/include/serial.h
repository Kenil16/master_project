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
# Documentation: See serial.c
# Author: Kjeld Jensen <kjeld@cetus.dk>
****************************************************************************/
/* system includes */
#include "../../../../../../../usr/include/stdlib.h"
#include "../../../../../../../usr/include/termios.h"

/***************************************************************************/
/* function prototypes */

/* initializes the serial port, returns 0 if everything is ok */
int ser_open (
	int *serRef,			/* returned reference to the port */
	struct termios *oldtio,	/* placeholder for old port settings*/
	char *devName,			/* eg. "/dev/ttyS0" */
	long baudRate);

int ser_send (int serRef, void *buffer, int numBytes);

/* retrieves up to numBytes bytes from the serial port, returns the number
   of retrieved bytes, 0 if no bytes retrieved or -1 if an error occurs */
int ser_receive (int serRef, void *buffer, int numBytes);

/* flushes the input buffer */
void ser_flush (int serRef);

void ser_close (int serRef, struct termios oldtio);

/***************************************************************************/
