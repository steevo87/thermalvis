/*! \file	videoslam.hpp
 *  \brief	Declarations for the VIDEOSLAM node.
*/

#ifndef _THERMALVIS_VIDEOSLAM_H_
#define _THERMALVIS_VIDEOSLAM_H_

#ifdef _FOR_REF_ONLY_

#include "general_resources.hpp"
#include "ros_resources.hpp"
#include "opencv_resources.hpp"
#include "pcl_resources.hpp"

//#include <cv_bridge/CvBridge.h>

#include "improc.hpp"
#include "video.hpp"
#include "features.hpp"
#include "reconstruction.hpp"
#include "sba.hpp"
#include "keyframes.hpp"
#include "geometry.hpp"

#include "feature_tracks.h"
#include "pose_confidence.h"

#include "videoslamConfig.h"

#include <std_msgs/Float32.h>

typedef Eigen::Quaternion<double>   QuaternionDbl;
typedef dynamic_reconfigure::Server < thermalvis::videoslamConfig > Server;

// Drawing
#define DEFAULT_DRAWGRAPH_DECIMATION			1
#define DEFAULT_DRAWGRAPH_BICOLOR				0

// Hard Limits




typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

const char __PROGRAM__[] = "THERMALVIS_ODOMETRY";

bool wantsToShutdown = false;
void mySigintHandler(int sig);


/// \brief		Stores configuration information for the ODOMETRY routine
struct videoslamData {
	
	
	
	bool obtainStartingData(ros::NodeHandle& nh);
	
};

/// \brief		Manages the ODOMETRY procedure
class videoslamNode {
private:

	
	
public:

	videoslamNode(ros::NodeHandle& nh, videoslamData startupData);
	
	
	
};

// dummy callbacks // http://answers.ros.org/question/55126/why-does-ros-overwrite-my-sequence-number/
void connected(const ros::SingleSubscriberPublisher&) {}
void disconnected(const ros::SingleSubscriberPublisher&) {}

boost::shared_ptr < videoslamNode > *globalNodePtr;

#endif

#endif