/*! \file	streamer.hpp
 *  \brief	Declarations for the ROS <streamer> node.
*/

#ifndef _STREAMER_H_
#define _STREAMER_H_
	
#include "ros_stream.hpp"
	
boost::shared_ptr < streamerNode > *globalNodePtr;
void set_blocking (int fd, int should_block);

#endif
