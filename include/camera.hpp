/*! \file	camera.hpp
 *  \brief	Declarations for geometric camera parameters.
*/
 
#ifndef _THERMALVIS_CAMERA_H_
#define _THERMALVIS_CAMERA_H_

#include "opencv_resources.hpp"

#include <math.h>

#include "tools.hpp"

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
