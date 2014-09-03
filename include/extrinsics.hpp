
#ifndef _THERMALVIS_EXTRINSICS_HPP_
#define _THERMALVIS_EXTRINSICS_HPP_

#define EXTRINSICS_DEFAULT_FLAGS 0

//#include "cv_utils.hpp"
#include "improc.hpp"
#include "calibration.hpp"

#define DEFAULT_EXTRINSICS_FLAGS                    CV_CALIB_RATIONAL_MODEL + CV_CALIB_FIX_INTRINSIC

/// \brief      Calculate the Extended Reprojection Error for the extrinsic case.
double calculateExtrinsicERE(int nCams,
                             std::vector<Point3f>& physicalPoints,
                             std::vector< std::vector<Point2f> > *corners,
                             Mat *cameraMatrix,
                             Mat *distCoeffs,
                             Mat *R,
                             Mat *T);

/// \brief      Cut down the given vectors of pointsets to those optimal for extrinsic calibration
void optimizeCalibrationSets(std::vector<Size> imSize,
                             int nCams,
                             Mat *cameraMatrix,
                             Mat *distCoeffs,
                             std::vector<Mat>& distributionMap,
                             std::vector< std::vector<Point2f> > *candidateCorners,
                             std::vector< std::vector<Point2f> > *testCorners,
                             std::vector<Point3f> row,
                             int selection,
                             int num,
                             std::vector<std::vector<int> >& tagNames,
                             std::vector<std::vector<int> >& selectedTags,
                             int flags = EXTRINSICS_DEFAULT_FLAGS);

/// \brief      Calculate the scores for a set if pointsets in terms of their contribution to extrinsic calibration
double obtainMultisetScore(int nCams,
                           vector<Mat>& distributionMap,
                           vector<Mat>& binMap,
                           vector<vector<double> >& distances,
                           std::vector<std::vector<Point2f> > *corners,
                           int index);

#endif
