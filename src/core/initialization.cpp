/*! \file	initialization.cpp
 *  \brief	(probably an obsolete file)
*/

#ifdef _BUILD_FOR_ROS_

#include "core/initialization.hpp"

bool initializationData::obtainStartingData(ros::NodeHandle& nh) {
	
	numDetectors = 0;
	
	nh.param<std::string>("video_stream", video_stream, "video_stream");
	
	if (video_stream != "video_stream") {
		printf("%s << Video stream (%s) selected.\n", __FUNCTION__, video_stream.c_str());
	} else {
		printf("%s << ERROR! No video stream specified.\n", __FUNCTION__);
		return false;
	}
	
	nh.param<bool>("debug_mode", debugMode, false);
	
	if (debugMode) {
		printf("%s << Running in DEBUG mode.\n", __FUNCTION__);
	} else {
		printf("%s << Running in OPTIMIZED mode.\n", __FUNCTION__);
	}
	
	nh.param<int>("max_frame_count", max_frame_count, DEFAULT_MAX_FRAMES);
	
	if (max_frame_count > MAXIMUM_FRAMES_TO_STORE) {
		printf("%s << ERROR! Maximum frames specified (%d) is too high.\n", __FUNCTION__, max_frame_count);
	} else {
		printf("%s << Maximum frames specified: %d\n", __FUNCTION__, max_frame_count);
	}
	
	for (int iii = 0; iii < MAX_DETECTORS; iii++) {
		
		char detector_tag[256], sensitivity_tag[256], method_tag[256], descriptor_tag[256];
		
		sprintf(detector_tag, "detector_%d", iii + 1);
		sprintf(sensitivity_tag, "sensitivity_%d", iii + 1);
		sprintf(method_tag, "method_%d", iii + 1);
		sprintf(descriptor_tag, "descriptor_%d", iii + 1);
		
		//printf("%s << detector_tag = %s\n", __FUNCTION__, detector_tag);
		
		if (iii == 0) {
			nh.param<std::string>(detector_tag, detector[iii], "SURF");
		} else {
			nh.param<std::string>(detector_tag, detector[iii], "NONE");
		}
		
		//printf("%s << detector[%d] = %s\n", __FUNCTION__, iii, detector[iii].c_str());
		
		if ((detector[iii] == "SURF") || (detector[iii] == "FAST") || (detector[iii] == "GFTT") || (detector[iii] == "HARRIS") || (detector[iii] == "ORB") || (detector[iii] == "STAR")) {
			printf("%s << detector [%d]: %s\n", __FUNCTION__, iii, detector[iii].c_str());
		} else if (detector[iii] == "NONE"){
			// printf("%s << No [%d] detector requested.\n", __FUNCTION__, iii);
			break;
		} else {
			printf("%s << ERROR! Detector (%s) not recognized.\n", __FUNCTION__, detector[iii].c_str());
			return false;
		}
		
		nh.param<double>(sensitivity_tag, sensitivity[iii], 30.0);
		
		printf("%s << detector [%d] sensitivity: %f\n", __FUNCTION__, iii, sensitivity[iii]);
		
		nh.param<std::string>(method_tag, method[iii], "match");
		
		if ((method[iii] == "match") || (method[iii] == "track")) {
			printf("%s << detector [%d] matching method: %s\n", __FUNCTION__, iii, method[iii].c_str());
			if (method[iii] == "match") {
				method_match[iii] = true;
			} else {
				method_match[iii] = false;
			}
		} else {
			printf("%s << ERROR! detector [%d] matching method (%s) not recognized.\n", __FUNCTION__, iii, method[iii].c_str());
			return false;
		}
		
		nh.param<std::string>(descriptor_tag, descriptor[iii], "SURF");
		
		if (method_match[iii]) {
			if ((descriptor[iii] == "SURF") || (descriptor[iii] == "ORB") || (descriptor[iii] == "BRIEF") || (descriptor[iii] == "SIFT")) {
				printf("%s << descriptor [%d]: %s\n", __FUNCTION__, iii, descriptor[iii].c_str());
			} else {
				printf("%s << ERROR! descriptor [%d] (%s) not recognized.\n", __FUNCTION__, iii, descriptor[iii].c_str());
				return false;
			}
		}
		
		numDetectors++;
	}
	
	printf("%s << [%d] detectors to be implemented.\n", __FUNCTION__, numDetectors);
	
	nh.param<std::string>("intrinsics", intrinsics, "intrinsics");
	
	if (intrinsics != "intrinsics") {
		printf("%s << Intrinsics at %s selected.\n", __FUNCTION__, intrinsics.c_str());
	} else {
		printf("%s << ERROR! No valid intrinsics file (%s) specified.\n", __FUNCTION__, intrinsics.c_str());
	}
	
	return true;
}

void initializationData::initializeDescriptors(cv::Ptr<cv::DescriptorExtractor> *desc) {
	
	for (unsigned int iii = 0; iii < numDetectors; iii++) {
		if (descriptor[iii] == "SURF") {
			ROS_ERROR("SURF has been deactivated due to copyright protection!");
			//desc[iii] = new SurfDescriptorExtractor( );
		} else if (descriptor[iii] == "SIFT") {
			ROS_ERROR("SIFT has been deactivated due to copyright protection!");
			//desc[iii] = new SiftDescriptorExtractor( );
		} else if (descriptor[iii] == "BRIEF") {
			desc[iii] = new cv::BriefDescriptorExtractor( );
		} else if (descriptor[iii] == "ORB") {
			desc[iii] = new cv::OrbDescriptorExtractor( );
		}
	}
	
}

void initializationData::initializeDetectors(cv::Ptr<cv::FeatureDetector> *det) {
	
	for (unsigned int iii = 0; iii < numDetectors; iii++) {
		if (detector[iii] == "SURF") {
			ROS_ERROR("SURF has been deactivated due to copyright protection!");
			// det[iii] = new SurfFeatureDetector( sensitivity[iii] );
		} else if (detector[iii] == "FAST") {
			det[iii] = new cv::FastFeatureDetector( sensitivity[iii] );
		} else if (detector[iii] == "GFTT") {
			det[iii] = new cv::GoodFeaturesToTrackDetector( MAXIMUM_FEATURES_PER_DETECTOR, sensitivity[iii], 1.0, 3, false );
		} else if (detector[iii] == "STAR") {
			det[iii] = new cv::StarFeatureDetector( 16, sensitivity[iii] );
		} else if (detector[iii] == "ORB") {
			det[iii] = new cv::OrbFeatureDetector( MAXIMUM_FEATURES_PER_DETECTOR );
		} else if (detector[iii] == "HARRIS") {
			det[iii] = new cv::GoodFeaturesToTrackDetector( MAXIMUM_FEATURES_PER_DETECTOR, sensitivity[iii], 1.0, 3, true );
		}
	}
	
}

#endif
