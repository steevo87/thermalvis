/*! \file	ros_stream.hpp
 *  \brief	Declarations for ROS video input.
*/

#ifndef _THERMALVIS_ROS_STREAM_H_
#define _THERMALVIS_ROS_STREAM_H_

#ifdef _BUILD_FOR_ROS_
	
#include "streamer.hpp"
	
#include "time.h"

#include "cxcore.h"
#include "highgui.h"

#include <signal.h>

#include "ros_resources.hpp"
#include "tools.hpp"
#include "improc.hpp"
#include "radiometric.hpp"
#include "video.hpp"

#ifdef _BUILD_FOR_ROS_
#include <dynamic_reconfigure/server.h>
#include "streamerConfig.h"
#endif

// SERIAL COMMS STUFF
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

#define MAX_VALID_THERMISTOR 					80.0
#define MIN_VALID_THERMISTOR 				   -20.0
#define MAX_TIME_WITHOUT_FLAGS 					45.0
#define MAX_TIME_WITHOUT_NUC_conservative		45.0

volatile int STOP=FALSE;

void signal_handler_IO (int status);    		// definition of signal handler
int wait_flag=TRUE;                     		// TRUE while no signal received
char devicename[80];
long Baud_Rate = 115200;         				// default Baud Rate (110 through 38400)
long BAUD;                      				// derived baud rate from command line
long DATABITS;
long STOPBITS;
long PARITYON;
long PARITY;
int Data_Bits = 8;              				// Number of data bits
int Stop_Bits = 1;              				// Number of stop bits
int Parity = 0;                 				// Parity as follows:
												// 00 = NONE, 01 = Odd, 02 = Even, 03 = Mark, 04 = Space
int Format = 4;
FILE *input;
FILE *output;
int status;

char message[90];

// Other stuff from: http://ubuntuforums.org/showthread.php?t=1395180
#include <stdio.h>
#include <string.h>
#include <fcntl.h> 								// File control
#include <errno.h> 								// Error number def



// END SERIAL COMMS STUFF


#ifdef _BUILD_FOR_ROS_
typedef dynamic_reconfigure::Server < thermalvis::streamerConfig > Server;
#endif

bool wantsToShutdown = false;
void mySigintHandler(int sig);

//HGH




#endif
#endif