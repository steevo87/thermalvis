/*! \file	slam.hpp
 *  \brief	Declarations for the ROS <slam> node.
*/

#ifndef _SLAM_H_
#define _SLAM_H_

#include "monocular_slam.hpp"

typedef dynamic_reconfigure::Server < thermalvis::slamConfig > Server;

bool wantsToShutdown = false;
void mySigintHandler(int sig);
void imageCallback(const sensor_msgs::ImageConstPtr& msg);
void parameterCallback(const sensor_msgs::CameraInfo& msg);

boost::shared_ptr < slamNode > *globalNodePtr;
void displayTermiosData(termios options);
void set_blocking (int fd, int should_block);

#endif
