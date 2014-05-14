/*! \file﻿  extrinsics.hpp
 *  \brief﻿  Declarations for extrinsic (geometric) calibration
 *
 * Ideally this file should only contain functions needed for extrinsic calibration, and not intrinsic calibration.
 * Functions required by both should be included in the "calibration.hpp/cpp" files.
 */

#ifndef EXTRINSICS_HPP
#define EXTRINSICS_HPP

#define EXTRINSICS_DEFAULT_FLAGS 0

//#include "cv_utils.hpp"
#include "improc.hpp"
#include "calibration.hpp"

#define DEFAULT_EXTRINSICS_FLAGS                    CV_CALIB_RATIONAL_MODEL + CV_CALIB_FIX_INTRINSIC

/// \brief      Calculate the Extended Reprojection Error for the extrinsic case.
double calculateExtrinsicERE(int nCams,
                             cv::vector<Point3f>& physicalPoints,
                             cv::vector< cv::vector<Point2f> > *corners,
                             Mat *cameraMatrix,
                             Mat *distCoeffs,
                             Mat *R,
                             Mat *T);

/// \brief      Cut down the given vectors of pointsets to those optimal for extrinsic calibration
void optimizeCalibrationSets(cv::vector<Size> imSize,
                             int nCams,
                             Mat *cameraMatrix,
                             Mat *distCoeffs,
                             cv::vector<Mat>& distributionMap,
                             cv::vector< cv::vector<Point2f> > *candidateCorners,
                             cv::vector< cv::vector<Point2f> > *testCorners,
                             cv::vector<Point3f> row,
                             int selection,
                             int num,
                             cv::vector<cv::vector<int> >& tagNames,
                             cv::vector<cv::vector<int> >& selectedTags,
                             int flags = EXTRINSICS_DEFAULT_FLAGS);

/// \brief      Calculate the scores for a set if pointsets in terms of their contribution to extrinsic calibration
double obtainMultisetScore(int nCams,
                           vector<Mat>& distributionMap,
                           vector<Mat>& binMap,
                           vector<vector<double> >& distances,
                           cv::vector<cv::vector<Point2f> > *corners,
                           int index);

#endif
