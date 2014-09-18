/*! \file	serial_comms.hpp
 *  \brief	Serial Comms information for the Miricle thermal cameras
*/

#ifndef _SERIAL_COMMS_H_
#define _SERIAL_COMMS_H_

#include "core/tools.hpp"

#ifndef _WIN32
#include <termios.h>
#include <unistd.h>
#include <sys/signal.h>
#include <termio.h>
#endif

#include <stdio.h>

#include <fcntl.h>

#include <sys/types.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#define BAUDRATE 								B38400
#define MODEMDEVICE 							"/dev/ttyS0"
#define _POSIX_SOURCE 							1         // POSIX compliant source
#define FALSE 									0
#define TRUE 									1
#define SERIAL_BUFF_SIZE 						1024

#define DEFAULT_BAUD_RATE						115200
#define DEFAULT_SERIAL_COMMS_FORMAT				4
#define DEFAULT_SERIAL_COMMS_PARITY				0
#define DEFAULT_SERIAL_COMMS_STOP_BITS			1
#define DEFAULT_SERIAL_COMMS_DATA_BITS			8

#define MAX_VALID_THERMISTOR 					80.0
#define MIN_VALID_THERMISTOR 				   -20.0
#define MAX_TIME_WITHOUT_FLAGS 					45.0
#define MAX_TIME_WITHOUT_NUC_conservative		45.0

struct serialCommsDevice {
	
	volatile int STOP;

	void signal_handler_IO (int status);    		// definition of signal handler
	int wait_flag;                     		// TRUE while no signal received
	char devicename[80];
	long Baud_Rate;         				// default Baud Rate (110 through 38400)
	long BAUD;                      				// derived baud rate from command line
	long DATABITS;
	long STOPBITS;
	long PARITYON;
	long PARITY;
	int Data_Bits;              				// Number of data bits
	int Stop_Bits;              				// Number of stop bits
	int Parity;                 				// Parity as follows:
													// 00 = NONE, 01 = Odd, 02 = Even, 03 = Mark, 04 = Space
	int Format;
	FILE *input;
	FILE *output;
	int status;

	char message[90];

	serialCommsDevice();

};

// Other stuff from: http://ubuntuforums.org/showthread.php?t=1395180
#include <stdio.h>
#include <string.h>
#include <fcntl.h> 								// File control
#include <errno.h> 								// Error number def

void set_blocking (int fd, int should_block);

#endif
