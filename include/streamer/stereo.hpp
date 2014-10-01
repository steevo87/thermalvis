/*! \file	stereo.hpp
 *  \brief	Declarations for depth from stereo related functions.
*/

#ifndef THERMALVIS_STEREO_H
#define THERMALVIS_STEREO_H

#include "core/general_resources.hpp"

#include "core/improc.hpp"
#include "core/tools.hpp"
// #include "flow/features.hpp"

void getContours(const cv::Mat& src, vector<vector<vector<cv::Point> > >& contours);
void drawContours(const cv::Mat& src, cv::Mat& dst);

double fitFixedLine(vector<unsigned short>& x, vector<unsigned short>& y, double grad = 1.0);

void depthFromContours(vector<vector<vector<cv::Point> > >& c1, vector<vector<vector<cv::Point> > >& c2, cv::Mat& disp);

//bool findRadiometricMapping(const cv::Mat& im1, const cv::Mat& im2, double& grad, double& shift, const cv::Mat& pim1, const cv::Mat& pim2);

void drawEpipolarLines(cv::Mat& im1, cv::Mat& im2, cv::Mat& F);

double findBestAlpha(const cv::Mat& K1, const cv::Mat& K2, const cv::Mat& coeff1, const cv::Mat& coeff2, const cv::Size& camSize, const cv::Mat& R0, const cv::Mat& R1);

// configData.cameraData1.newCamMat = getOptimalNewCameraMatrix(configData.cameraData1.K, configData.cameraData1.distCoeffs, configData.cameraData1.cameraSize, alpha, configData.cameraData1.cameraSize, &roi1, centerPrincipalPoint);

bool relevelImages(const cv::Mat& im1, const cv::Mat& im2, const cv::Mat& old_depth, const cv::Mat& R1, const cv::Mat& R2, const cv::Mat& t, const cv::Mat& Q1, const cv::Mat& Q2, cv::Mat& pim1, cv::Mat& pim2, const cv::Mat& map11, const cv::Mat& map12, const cv::Mat& map21, const cv::Mat& map22);

void findRadiometricMapping2(const cv::Mat& im1, const cv::Mat& im2, const cv::Mat& depth, const cv::Mat& R1, const cv::Mat& R2, const cv::Mat& t, const cv::Mat& Q1, const cv::Mat& Q2, double& grad, double& shift);

void plotPoints(cv::Mat& dispMat, vector<unsigned short>& i1, vector<unsigned short>& i2, unsigned short median_1 = 0, unsigned short median_2 = 0, double grad = 1.0, double shift = 0.0);

#endif // THERMALVIS_STEREO_H
