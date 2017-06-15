/*
 * manifoldGPIO.h
 *
 * Copyright (c) 2015 JetsonHacks
 * www.jetsonhacks.com
 *
 * Based on Software by RidgeRun
 * Originally from:
 * https://developer.ridgerun.com/wiki/index.php/Gpio-int-test.c
 * Copyright (c) 2011, RidgeRun
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *    This product includes software developed by the RidgeRun.
 * 4. Neither the name of the RidgeRun nor the
 *    names of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY RIDGERUN ''AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL RIDGERUN BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef manifoldGPIO_H_
#define manifoldGPIO_H_

// Constants
#define SYSFS_GPIO_DIR "/sys/class/gpio"
#define POLL_TIMEOUT (3 * 1000) /* 3 seconds */
#define MAX_BUF 64

typedef unsigned int GPIO;
typedef unsigned int pinDirection;
typedef unsigned int pinValue;

enum pinDirections {

	input  = 0,
	output = 1,
};

enum pinValues {

	low = 0,
    high = 1,
};

enum manifoldGPIONumber {
     
	gpio157 = 157,
  	gpio158 = 158,
     
};


int gpioExport ( GPIO gpio ) ;
int gpioUnexport ( GPIO gpio ) ;
int gpioSetDirection ( GPIO, pinDirection out_flag ) ;
int gpioSetValue ( GPIO gpio, pinValue value ) ;
int gpioGetValue ( GPIO gpio, unsigned int *value ) ;
int gpioSetEdge ( GPIO gpio, char *edge ) ;
int gpioOpen ( GPIO gpio ) ;
int gpioClose ( int fileDescriptor ) ;
int gpioActiveLow ( GPIO gpio, unsigned int value ) ;



#endif // manifoldGPIO_H_
