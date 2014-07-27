/*! \file	ros_flow.hpp
 *  \brief	Declarations for ROS <flow> functions.
*/

#ifndef _THERMALVIS_ROS_FLOW_H_
#define _THERMALVIS_ROS_FLOW_H_

#ifdef _BUILD_FOR_ROS_

#include "sparse_flow.hpp"

typedef dynamic_reconfigure::Server < thermalvis::flowConfig > Server;

bool wantsToShutdown = false;
void mySigintHandler(int sig);
void imageCallback(const sensor_msgs::ImageConstPtr& msg);
void parameterCallback(const sensor_msgs::CameraInfo& msg);

// dummy callbacks // http://answers.ros.org/question/55126/why-does-ros-overwrite-my-sequence-number/

void connected(const ros::SingleSubscriberPublisher&) {}
void disconnected(const ros::SingleSubscriberPublisher&) {}



#endif

#endif