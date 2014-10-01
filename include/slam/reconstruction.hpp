/*! \file	reconstruction.hpp
 *  \brief	Declarations for establishing 3D structure of scene.
*/

#ifndef THERMALVIS_RECONSTRUCTION_H
#define THERMALVIS_RECONSTRUCTION_H

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "core/tools.hpp"
#include "core/opencv_redefinitions.hpp"
#include "core/general_resources.hpp"
#include "core/ros_resources.hpp"
#include "core/camera.hpp"
#include "slam/geometry.hpp"
#include "slam/triangulation.hpp"
#include "core/tracks.hpp"

#ifdef _BUILD_FOR_ROS_
#include <visualization_msgs/MarkerArray.h>
#endif

// SBA Includes
#ifdef _USE_SBA_
#include <sba/sba.h>
#include <sba/sba_file_io.h>
#include <sba/visualization.h>
#endif

#define _USE_MATH_DEFINES
#include <math.h>

#ifdef _USE_SBA_
using namespace sba;
#endif

// #include <pcl/visualization/cloud_viewer.h>

#define POSITION_LIMIT				1000
#define DEFAULT_MEAN_MAX_DIST		0.20

// #define __SFM__DEBUG__

// STRUCTURE INITIALIZATION

/// \brief 		Calculate the bias towards either F-GRIC or H-GRIC
double normalizedGRICdifference(std::vector<cv::Point2f>& pts1, std::vector<cv::Point2f>& pts2, cv::Mat& F, cv::Mat& H, cv::Mat& mask_F, cv::Mat& mask_H, double& F_GRIC, double& H_GRIC);

/// \brief 		Calculate the Geometric Robust Information Criterion (GRIC) score
double calculateGRIC(std::vector<cv::Point2f>& pts1, std::vector<cv::Point2f>& pts2, cv::Mat& rel, cv::Mat& mask, int d, double k, double r, double lambda_3, int distMethod);

/// \brief 		Calculate the 'Rho' factor, used in the GRIC calculation
double calculateRho(double e, double sig, double r, double lambda_3, int d);

/// \brief 		Calculate one of several alternative geometric distances between two 2D points
double calcGeometryDistance(cv::Point2f& pt1, cv::Point2f& pt2, cv::Mat& mat, int distMethod = SAMPSON_DISTANCE);

/// \brief		http://www.ics.forth.gr/~lourakis/homest/
double lourakisSampsonError(cv::Point2f& pt1, cv::Point2f& pt2, cv::Mat& H);

/// \brief		Extract the triangulated 3D points from the tracking vector
void ExtractPointCloud(vector<cv::Point3d>& cloud, vector<featureTrack>& trackVector);

/// \brief		Find the best (out of four possible) transformations between cameras for reconstruction
bool findBestReconstruction(const cv::Mat& P0, cv::Mat& P1, cv::Mat& R, cv::Mat& t, const cv::SVD& svd, const cv::Mat& K, const vector<cv::Point2f>& pts1, const vector<cv::Point2f>& pts2, bool useDefault = false);

/// \brief		Finds the four possible transformations from the Essential matrix
void findFourTransformations(cv::Mat *C, const cv::Mat& E, const cv::Mat& K, const vector<cv::Point2f>& pts1, const vector<cv::Point2f>& pts2);

// \brief		"cheirality test"? This function is when you have the 3D points and both transformation matrices
int pointsInFront(const cv::Mat& C1, const cv::Mat& C2, const vector<cv::Point3d>& pts);

// \brief		"cheirality test"? This function is when you have the relative transformation matrix, cam matrix, and 2D locations of points..
int pointsInFront(const cv::Mat& C1, const cv::Mat& K, const vector<cv::Point2f>& pts1, const vector<cv::Point2f>& pts2);

// \brief		Calculate frame score from geometry score and number of tracked features
double calcFrameScore(double geomScore, int numFeatures, int numTracks);

// \brief		Calculate geometry score from H/F inliers and sampson errors
double calcGeometryScore(int numInliers_H, int numInliers_F, double sampsonError_H, double sampsonError_F);

// \brief		Calculate the sampson error between two vectors
double calcSampsonError(vector<cv::Point2f>& points1, vector<cv::Point2f>& points2, cv::Mat& H, cv::Mat& Hmask);

// \brief		Calculate the sampson distance between two points and the F matrix
double calcSampsonDistance(cv::Point2f& pt1, cv::Point2f& pt2, cv::Mat& F);

// \brief		Calculate the geometry distance only using the inliers
double calcInlierGeometryDistance(vector<cv::Point2f>& points1, vector<cv::Point2f>& points2, cv::Mat& mat, cv::Mat& mask, int distMethod = SAMPSON_DISTANCE);

// \brief		Overwrite the P0 matrix with the provided intrinsics
void assignIntrinsicsToP0(cv::Mat& P0, const cv::Mat& K);

// Gets the 2d locations of points that have been tracked
void getCorrespondingPoints(vector<featureTrack>& tracks, const vector<cv::Point2f>& pts1, vector<cv::Point2f>& pts2, int idx0, int idx1);

// Given some 3D co-ordinates and two image indices, find the corresponding image points that show as many of 
// these as possible (actually it should be all of them if this is an intermediate point)
void findFeaturesForPoints(vector<featureTrack>& tracks, vector<cv::Point2f>& pts1, vector<cv::Point2f>& pts2, vector<cv::Point3d>& pts3d, int idx1, int idx2);

void estimateNewPose(vector<featureTrack>& tracks, cv::Mat& K, int idx, cv::Mat& pose);

void obtainAppropriateBaseTransformation(cv::Mat& C0, vector<featureTrack>& tracks);

void findCentroidAndSpread(vector<featureTrack>& tracks, cv::Point3d& centroid, cv::Point3d& deviations);

void filterToCompleteTracks(vector<unsigned int>& dst, vector<unsigned int>& src, vector<featureTrack>& tracks, int idx1, int idx2);
void getActiveTracks(vector<unsigned int>& indices, vector<featureTrack>& tracks, int idx1, int idx2);

int countTriangulatedTracks(const vector<featureTrack>& tracks);
int countActiveTriangulatedTracks(vector<unsigned int>& indices, vector<featureTrack>& tracks);

void updateTriangulatedPoints(vector<featureTrack>& tracks, vector<unsigned int>& indices, vector<cv::Point3d>& cloud);

void getActive3dPoints(vector<featureTrack>& tracks, vector<unsigned int>& indices, vector<cv::Point3d>& cloud);

// Gets 2D locations of active tracks into simple vectors
void filterToActivePoints(vector<featureTrack>& tracks, vector<cv::Point2f>& pts1, vector<cv::Point2f>& pts2, vector<unsigned int>& indices, int idx1, int idx2);

void reduceActiveToTriangulated(vector<featureTrack>& tracks, vector<unsigned int>& indices, vector<unsigned int>& untriangulated);



int findBestCandidate(const cv::Mat *CX, const cv::Mat& K, const vector<cv::Point2f>& pts1, const vector<cv::Point2f>& pts2, cv::Mat& C);

void summarizeTransformation(const cv::Mat& C, char *summary);

void reverseTranslation(cv::Mat& C);

void removeShortTracks(vector<featureTrack>& tracks, int idx1, int idx2);

void getIndicesForTriangulation(vector<unsigned int>& dst, vector<unsigned int>& src, vector<unsigned int>& already_triangulated);

void reconstructSubsequence(vector<featureTrack>& tracks, vector<cv::Point3d>& ptCloud, int idx1, int idx2);



void findCentroid(vector<featureTrack>& tracks, cv::Point3d& centroid, cv::Point3d& stdDeviation);

// Between two images, gets all 2D point locations and 3D locations needed for BA
void getTriangulatedFullSpanPoints(vector<featureTrack>& tracks, vector<cv::Point2f>& pts1, vector<cv::Point2f>& pts2, int idx1, int idx2, vector<cv::Point3f>& points3);

bool estimatePoseFromKnownPoints(cv::Mat& dst, cameraParameters camData, vector<featureTrack>& tracks, unsigned int index, const cv::Mat& guide, unsigned minAppearances = 1, unsigned int iterCount = 100, double maxReprojErr = 4.0, double inliersPercentage = 0.9, double *reprojError = 0, double *pnpInlierProp = 0, bool debug = false);
	
double testKeyframePair(vector<featureTrack>& tracks, cameraParameters& camData, double *scorecard[], int idx1, int idx2, double *scores, cv::Mat& pose, bool evaluate = false, bool debug = false);

bool reconstructFreshSubsequencePair(vector<featureTrack>& tracks, vector<cv::Point3d>& ptCloud, vector<unsigned int>& triangulatedIndices, cv::Mat& real_C0, cv::Mat& real_C1, cameraParameters camData, int idx1, int idx2);

void subselectPoints(const vector<cv::Point2f>& src1, vector<cv::Point2f>& dst1, const vector<cv::Point2f>& src2, vector<cv::Point2f>& dst2);

#endif // THERMALVIS_RECONSTRUCTION_H

