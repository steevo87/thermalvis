/*! \file	ros_slam.hpp
 *  \brief	Declarations for ROS <slam> functions.
*/

#ifndef _THERMALVIS_ROS_SLAM_H_
#define _THERMALVIS_ROS_SLAM_H_

#ifdef _BUILD_FOR_ROS_

#include "monocular_slam.hpp"

typedef dynamic_reconfigure::Server < thermalvis::slamConfig > Server;

bool wantsToShutdown = false;
void mySigintHandler(int sig);
void imageCallback(const sensor_msgs::ImageConstPtr& msg);
void parameterCallback(const sensor_msgs::CameraInfo& msg);

// dummy callbacks // http://answers.ros.org/question/55126/why-does-ros-overwrite-my-sequence-number/

void connected(const ros::SingleSubscriberPublisher&) {}
void disconnected(const ros::SingleSubscriberPublisher&) {}



#endif

#endif