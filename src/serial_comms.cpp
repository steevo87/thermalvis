/*! \file	serial_comms.cpp
 *  \brief	Definitions for serial comms communications for Miricle cameras on Linux
*/

#include "serial_comms.hpp"

serialCommsDevice::serialCommsDevice() :
	STOP(FALSE),
	wait_flag(TRUE),
	Baud_Rate(DEFAULT_BAUD_RATE),
	Data_Bits(DEFAULT_SERIAL_COMMS_DATA_BITS),
	Stop_Bits(DEFAULT_SERIAL_COMMS_STOP_BITS),
	Parity(DEFAULT_SERIAL_COMMS_PARITY),
	Format(DEFAULT_SERIAL_COMMS_FORMAT) 
{ }

void set_blocking (int fd, int should_block) {
#ifndef _IS_WINDOWS_
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fd, &tty) != 0) ROS_ERROR("error %d getting term settings set_blocking", errno);

    tty.c_cc[VMIN]  = should_block ? 1 : 0;
    tty.c_cc[VTIME] = should_block ? 5 : 0; // 0.5 seconds read timeout

    if (tcsetattr (fd, TCSANOW, &tty) != 0) ROS_ERROR("error setting term %sblocking", should_block ? "" : "no");
#endif
}

#ifndef _IS_WINDOWS_
void displayTermiosData(termios options) {

	ROS_INFO("   Termios Summary:");
	
	ROS_INFO("c_iflag = (%d)", options.c_iflag);
	ROS_INFO("c_oflag = (%d)", options.c_oflag);
	ROS_INFO("c_cflag = (%d)", options.c_cflag);
	ROS_INFO("c_lflag = (%d)", options.c_lflag);
	ROS_INFO("c_ispeed = (%d)", options.c_ispeed);
	ROS_INFO("c_ospeed = (%d)", options.c_ospeed);
	
	for (unsigned int iii = 0; iii < NCCS; iii++) ROS_INFO("c_cc[%d] = (%d)", iii, options.c_cc[iii]);
	
	ROS_WARN("Termios Summary Complete.");
}
#endif