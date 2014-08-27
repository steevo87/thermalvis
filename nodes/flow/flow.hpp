/*! \file	flow.hpp
 *  \brief	Declarations for the ROS <flow> node.
*/

#ifndef _THERMALVIS_FLOW_H_
#define _THERMALVIS_FLOW_H_

#include "flow/sparse_flow.hpp"

typedef dynamic_reconfigure::Server < thermalvis::flowConfig > Server;

bool wantsToShutdown = false;
void mySigintHandler(int sig);
void imageCallback(const sensor_msgs::ImageConstPtr& msg);
void parameterCallback(const sensor_msgs::CameraInfo& msg);

boost::shared_ptr < featureTrackerNode > *globalNodePtr;

#endif
