/*! \file	slam.hpp
 *  \brief	Declarations for the ROS <slam> node.
*/

#ifndef _SLAM_H_
#define _SLAM_H_
	
#include "ros_slam.hpp"
	
boost::shared_ptr < slamNode > *globalNodePtr;
void displayTermiosData(termios options);
void set_blocking (int fd, int should_block);

#endif
