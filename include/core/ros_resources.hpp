/*! \file	ros_resources.hpp
 *  \brief	Declarations for ROS inclusion requirements and ROS-related tools.
*/

#ifndef _THERMALVIS_ROS_RESOURCES_H_
#define _THERMALVIS_ROS_RESOURCES_H_

#ifdef _BUILD_FOR_ROS_

#include "tools.hpp"
#include "core/general_resources.hpp"

/***** ROS Stuff *****/
#include "ros/ros.h"
#include <std_msgs/Float32.h>
//#include <../../opt/ros/fuerte/include/ros/node_handle.h>
#include "ros/node_handle.h"
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <camera_calibration_parsers/parse_ini.h>
#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>

#include <tf/transform_broadcaster.h>

#include <visualization_msgs/MarkerArray.h>

#include <dynamic_reconfigure/server.h>

namespace enc = sensor_msgs::image_encodings;

void changemode(int);
int  kbhit(void);

// dummy callbacks // http://answers.ros.org/question/55126/why-does-ros-overwrite-my-sequence-number/
static void connected(const ros::SingleSubscriberPublisher&) {}
static void disconnected(const ros::SingleSubscriberPublisher&) {}



#endif

#endif // _THERMALVIS_ROS_RESOURCES_H_
