/*! \file	camera.cpp
 *  \brief	Definitions for geometric camera parameters.
*/

#include "camera.hpp"

namespace ros {

Time::Time() : sec(0), nsec(0) { }

Time::Time(double input) {
	sec = (long int)(floor(input));
	nsec = (long int)((input - (double(sec)))*1000000000.0);
}

double Time::toSec() { return (double(sec) + (double(nsec)/1000000000.0)); }

double Time::toNSec() { return (double(sec)*1000000000.0 + double(nsec)); }

Time Time::now() {
	Time retVal;
	boost::posix_time::ptime currentTime = boost::posix_time::microsec_clock::local_time();
	retVal.sec = (long int)(currentTime.time_of_day().total_seconds());
	retVal.nsec = (long int)(currentTime.time_of_day().total_nanoseconds()) - retVal.sec*1000000000;
	return retVal;
}

Header::Header() : seq(0), frame_id("") { }

namespace sensor_msgs {

CameraInfo::CameraInfo() : 
	width(384), 
	height(288), 
	distortion_model("plumb bob"), 
	binning_x(0),
	binning_y(0)
{

	K[0] = 500.0;
	K[1] = 0.0;
	K[2] = 191.5;

	K[3] = 0.0;
	K[4] = 500.0;
	K[5] = 143.5;

	K[6] = 0.0;
	K[7] = 0.0;
	K[8] = 1.0;
	
	for (int iii = 0; iii < 12; iii++) {
		P[iii] = 0.0;
		if (iii < 9) R[iii] = 0.0;
	}
}

}

}

cameraParameters::cameraParameters() {

	imageSize = cv::Mat(1, 2, CV_16UC1);
	cameraMatrix = cv::Mat::eye(3, 3, CV_64FC1);
	distCoeffs = cv::Mat::zeros(1, 5, CV_64FC1);
	blankCoeffs = cv::Mat::zeros(1, 5, CV_64FC1);

	double alpha = 0.00;
	bool centerPrincipalPoint = true;
	cv::Rect* validPixROI = 0;

}

bool cameraParameters::updateCameraParameters() {

	double alpha = 0.00;
	bool centerPrincipalPoint = true;
	cv::Rect* validPixROI = 0;

	cameraSize = cv::Size(imageSize.at<unsigned short>(0, 0), imageSize.at<unsigned short>(0, 1));
	
	newCamMat = getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, cameraSize, alpha, cameraSize, validPixROI, centerPrincipalPoint);
	
	newCamMat.copyTo(K);
	K_inv = K.inv();
	
	if (K.rows == 0) return false;
	return true;
}

bool cameraParameters::getCameraParameters(std::string intrinsics) {
	
	double alpha = 0.00;
	bool centerPrincipalPoint = true;
	cv::Rect* validPixROI = 0;
	
	cv::FileStorage fs(intrinsics, cv::FileStorage::READ);
	fs["imageSize"] >> imageSize;
	fs["cameraMatrix"] >> cameraMatrix;
	fs["distCoeffs"] >> distCoeffs;
	blankCoeffs = cv::Mat::zeros(distCoeffs.rows, distCoeffs.cols, CV_64FC1);
	fs.release();
	
	cameraSize = cv::Size(imageSize.at<unsigned short>(0, 0), imageSize.at<unsigned short>(0, 1));
	
	newCamMat = getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, cameraSize, alpha, cameraSize, validPixROI, centerPrincipalPoint);
	newCamMat.copyTo(K);
	K_inv = K.inv();
	
	if (K.rows == 0) return false;
	return true;
}
