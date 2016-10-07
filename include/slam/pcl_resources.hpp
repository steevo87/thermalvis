/*! \file	pcl_resources.hpp
 *  \brief	For PCL inclusion requirements.
*/

#ifndef _THERMALVIS_PCL_RESOURCES_H_
#define _THERMALVIS_PCL_RESOURCES_H_

#ifdef _USE_PCL_

#include "core/general_resources.hpp"

//#include "pcl/point_types.h"
//#include "pcl_ros/io/bag_io.h"

#include "pcl/point_cloud.h" //<--Just Added

#ifdef _BUILD_FOR_ROS_
#include "sensor_msgs/point_cloud_conversion.h" //<--Just Added
#endif

//#include <pcl/ros/conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

//using namespace cv;

#endif
#endif


