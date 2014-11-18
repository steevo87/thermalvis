/*! \file	calibrator.hpp
 *  \brief	Declarations for the ROS <calibrator> node.
*/

#ifndef _THERMALVIS_CALIBRATOR_H_
#define _THERMALVIS_CALIBRATOR_H_

#include "calibrator/calib.hpp"

typedef dynamic_reconfigure::Server < thermalvis::calibratorConfig > Server;

bool wantsToShutdown = false;
void mySigintHandler(int sig);
void imageCallback(const sensor_msgs::ImageConstPtr& msg);
void parameterCallback(const sensor_msgs::CameraInfo& msg);

boost::shared_ptr < calibratorNode > *globalNodePtr;

#endif
