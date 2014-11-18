
#ifndef THERMALVIS_EXTRINSICS_HPP
#define THERMALVIS_EXTRINSICS_HPP

#define EXTRINSICS_DEFAULT_FLAGS 0

//#include "cv_utils.hpp"
#include "core/improc.hpp"
#include "calibrator/calibration.hpp"

#define DEFAULT_EXTRINSICS_FLAGS                    CV_CALIB_RATIONAL_MODEL + CV_CALIB_FIX_INTRINSIC

/// \brief      Calculate the Extended Reprojection Error for the extrinsic case.
double calculateExtrinsicERE(int nCams,
                             std::vector<cv::Point3f>& physicalPoints,
                             std::vector< std::vector<cv::Point2f> > *corners,
                             cv::Mat *cameraMatrix,
                             cv::Mat *distCoeffs,
                             cv::Mat *R,
                             cv::Mat *T);

/// \brief      Cut down the given vectors of pointsets to those optimal for extrinsic calibration
void optimizeCalibrationSets(std::vector<cv::Size> imSize,
                             int nCams,
                             cv::Mat *cameraMatrix,
                             cv::Mat *distCoeffs,
                             std::vector<cv::Mat>& distributionMap,
                             std::vector< std::vector<cv::Point2f> > *candidateCorners,
                             std::vector< std::vector<cv::Point2f> > *testCorners,
                             std::vector<cv::Point3f> row,
                             int selection,
                             int num,
                             std::vector<std::vector<int> >& tagNames,
                             std::vector<std::vector<int> >& selectedTags,
                             int flags = EXTRINSICS_DEFAULT_FLAGS);

/// \brief      Calculate the scores for a set if pointsets in terms of their contribution to extrinsic calibration
double obtainMultisetScore(int nCams,
                           vector<cv::Mat>& distributionMap,
                           vector<cv::Mat>& binMap,
                           vector<vector<double> >& distances,
                           std::vector<std::vector<cv::Point2f> > *corners,
                           int index);

#endif
