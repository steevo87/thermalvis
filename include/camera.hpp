/*! \file	camera.hpp
 *  \brief	Declarations for geometric camera parameters.
*/
 
#ifndef _THERMALVIS_CAMERA_H_
#define _THERMALVIS_CAMERA_H_

#include "opencv_resources.hpp"

#include "boost/date_time/posix_time/posix_time.hpp"

namespace ros {

struct Time {
	int sec;
	int nsec;

	double toSec();
	double toNSec();
	Time();
	Time(double input);
	static Time now();
};

struct Header {
	int seq;
	std::string frame_id;
	Time stamp;
	Header();
};

namespace sensor_msgs {

struct CameraInfo {
	Header header;
	double K[9], R[9], P[12];
	std::vector<double> D;
	int width, height;
	int binning_x, binning_y;
	std::string distortion_model;

	CameraInfo();
};

}
}

/// \brief		Stores camera calibration information in OpenCV format
struct cameraParameters {
	
	cv::Mat cameraMatrix;
	cv::Mat distCoeffs;
	cv::Mat blankCoeffs;
	cv::Mat newCamMat;
	cv::Mat imageSize;
	cv::Size cameraSize;
	
	cv::Mat Kx;
	cv::Mat expandedCamMat;
	cv::Size expandedSize;
	
	cv::Mat K, K_inv;
	
	cv::Mat R, P;
	
	cameraParameters();
	
	bool updateCameraParameters();
	bool getCameraParameters(std::string intrinsics);
	
};

#endif
