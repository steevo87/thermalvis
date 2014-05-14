/*! \file	pcl_resources.hpp
 *  \brief	For PCL inclusion requirements.
*/

#ifdef _USE_PCL_
#ifndef _THERMALVIS_PCL_RESOURCES_H_
#define _THERMALVIS_PCL_RESOURCES_H_

#include "general_resources.hpp"

//#include "pcl/point_types.h"
//#include "pcl_ros/io/bag_io.h"

#include "pcl/point_cloud.h" //<--Just Added
#include "sensor_msgs/point_cloud_conversion.h" //<--Just Added

#include <pcl/ros/conversions.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

//using namespace cv;

#endif
#endif


