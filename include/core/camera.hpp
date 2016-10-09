/*! \file	camera.hpp
 *  \brief	Declarations for geometric camera parameters.
*/
 
#ifndef THERMALVIS_CAMERA_H
#define THERMALVIS_CAMERA_H

#include <math.h>

#include "core/tools.hpp"

#include <opencv2/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>

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

#endif // THERMALVIS_CAMERA_H
