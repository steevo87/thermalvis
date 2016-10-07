/*! \file	initialization.hpp
 *  \brief	(probably an obsolete file)
*/

#ifndef _THERMALVIS_INITIALIZATION_H_
#define _THERMALVIS_INITIALIZATION_H_

#ifdef _BUILD_FOR_ROS_

#include "core/general_resources.hpp"
#include "core/ros_resources.hpp"
#include "core/features.hpp"

#define MAXIMUM_FRAMES_TO_STORE 		2000	// careful about setting this too high for memory
#define MAXIMUM_FEATURES_PER_DETECTOR	5000
#define DEFAULT_MAX_FRAMES 				100

/// \brief		(probably obsolete..)
struct initializationData {
	string video_stream;
	int max_frame_count;
	
	bool debugMode;
	
	string detector[MAX_DETECTORS], descriptor[MAX_DETECTORS];
	double sensitivity[MAX_DETECTORS];
	string method[MAX_DETECTORS];
	bool method_match[MAX_DETECTORS];
	
	unsigned int numDetectors;
	
	string intrinsics;
	bool obtainStartingData(ros::NodeHandle& nh);   
    void initializeDetectors(cv::Ptr<cv::FeatureDetector> *det);
    void initializeDescriptors(cv::Ptr<cv::DescriptorExtractor> *desc);
	
};

#endif

#endif // _THERMALVIS_INITIALIZATION_H_
