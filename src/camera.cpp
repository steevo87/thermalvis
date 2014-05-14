/*! \file	camera.cpp
 *  \brief	Definitions for geometric camera parameters.
*/

#include "camera.hpp"

bool cameraParameters::getCameraParameters(std::string intrinsics) {
	
	//printf("%s << Entered (%s).\n", __FUNCTION__, intrinsics.c_str());
	
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
	//newCamMat.copyTo(K);	// or some other kind of matrix...?
	K_inv = K.inv();
	
	//printf("%s << Image size = (%d, %d)\n", __FUNCTION__, cameraSize.width, cameraSize.height);
	
	//cout << __FUNCTION__ << " << K = " << endl;
	//cout << K << endl;
	
	if (K.rows == 0) {
		return false;
	}
	
	return true;
	
}
