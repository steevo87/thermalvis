/*! \file	flow.hpp
 *  \brief	Declarations for the ROS <flow> node.
*/

#ifndef _THERMALVIS_FLOW_H_
#define _THERMALVIS_FLOW_H_

#include "sparse_flow.hpp"


#ifdef _BUILD_FOR_ROS_
typedef dynamic_reconfigure::Server < thermalvis::flowConfig > Server;
#endif

#ifdef _BUILD_FOR_ROS_
bool wantsToShutdown = false;
void mySigintHandler(int sig);
void imageCallback(const sensor_msgs::ImageConstPtr& msg);
void parameterCallback(const sensor_msgs::CameraInfo& msg);
#endif

// dummy callbacks // http://answers.ros.org/question/55126/why-does-ros-overwrite-my-sequence-number/
#ifdef _BUILD_FOR_ROS_
void connected(const ros::SingleSubscriberPublisher&) {}
void disconnected(const ros::SingleSubscriberPublisher&) {}

boost::shared_ptr < featureTrackerNode > *globalNodePtr;
#endif

#endif
