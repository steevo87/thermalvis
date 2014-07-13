/*! \file	camera.hpp
 *  \brief	Declarations for geometric camera parameters.
*/
 
#ifndef _THERMALVIS_CAMERA_H_
#define _THERMALVIS_CAMERA_H_

#include "opencv_resources.hpp"

struct cameraInfoStruct {
	double K[9], D[8];
	int width, height;
	std::string distortion_model;

	cameraInfoStruct();
};

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
