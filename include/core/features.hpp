/*! \file	features.hpp
 *  \brief	Declarations for local feature detection, description and cv::Matching.
*/

#ifndef THERMALVIS_FEATURES_H
#define THERMALVIS_FEATURES_H

#include <algorithm>

#include "core/general_resources.hpp"
#include "core/improc.hpp"
#include "core/tools.hpp"
#include "core/camera.hpp"

#define MAX_DETECTORS				10

#define MATCHING_MODE_NN			0
#define MATCHING_MODE_NNDR			1
#define MATCHING_MODE_SVM			2

#define DEFAULT_SVM_GRADIENT		-1.42

#define DO_EPIPOLAR_CHECK			false

/// \brief		Allows sorting of KeyPoint datatypes based on feature strength
bool KeyPoint_comparison(cv::KeyPoint i, cv::KeyPoint j);

int getOpticalFlowFlags();


/// \brief		Calculates velocities of features
double calculateFeatureSpeeds(const vector<cv::Point2f>& pts1, const vector<cv::Point2f>& pts2, vector<cv::Point2f>& velocities, double time1, double time2);

void crossCheckMatching( cv::Ptr<cv::DescriptorMatcher>& descriptorMatcher,
                         const cv::Mat& descriptors1, const cv::Mat& descriptors2,
                         vector<cv::DMatch>& filteredMatches12, int knn );

void filterTrackingsByRadialProportion(vector<cv::Point2f>& pts1, vector<cv::Point2f>& pts2, cv::Mat& K, cv::Mat& newCamMat, cv::Mat& distCoeffs, cv::Size& imSize, double prop = 1.00);

double generateVirtualPointset(const vector<cv::Point2f>& pts1, const vector<cv::Point2f>& pts2, vector<cv::Point2f>& virtual_pts, double bias_fraction = 0.5);

/// \brief		Transforms points using homography
void transformPoints(vector<cv::Point2f>& pts1, cv::Mat& H);

/// \brief		Remove features beyond a desired quantity. Removes by strength, but if many of equal strength, randomness is introduced.
void reduceFeaturesToMaximum(vector<cv::KeyPoint>& keypoints, int maximumToKeep);

/// \brief      Implement optical flow algorithm and filtering
void trackPoints(const cv::Mat& im1, const cv::Mat& im2, vector<cv::Point2f>& pts1, vector<cv::Point2f>& pts2, int distanceConstraint, double thresh, vector<unsigned int>& lostIndices, cv::Mat H12 = cv::Mat(), cameraParameters camData = cameraParameters());

/// \brief      Implement optical flow algorithm and filtering for calibrating stereo cameras
void trackPoints2(const cv::Mat& im1, const cv::Mat& im2, vector<cv::Point2f>& pts1, vector<cv::Point2f>& pts2, int distanceConstraint, double thresh, vector<unsigned int>& lostIndices, const cv::Size patternSize,  double& errorThreshold);

void checkDistances(vector<cv::Point2f>& pts1, vector<cv::Point2f>& pts2, vector<uchar>& statusVec, double distanceConstraint);

/// \brief      Prints KeyPoints onto a copy of an image
void displayKeyPoints(const cv::Mat& image, const vector<cv::KeyPoint>& KeyPoints, cv::Mat& outImg, const cv::Scalar& color, int thickness = 1, bool pointsOnly = false);

/// \brief      Prints KeyPoints onto a copy of an image
void displayKeyPoints(const cv::Mat& image, const vector<cv::Point2f>& pts, cv::Mat& outImg, cv::Scalar color, int thickness = 1, bool pointsOnly = false);

/// \brief      Prints cv::Match paths onto a copy of an image, given unfiltered points and cv::Matches structure
void drawMatchPaths(cv::Mat& src, cv::Mat& dst, vector<cv::KeyPoint>& kp1, vector<cv::KeyPoint>& kp2, vector<vector<cv::DMatch> >& matches1to2);

/// \brief      Prints cv::Match paths onto a copy of an image, given vectors containing corresponding points
void drawMatchPaths(cv::Mat& src, cv::Mat& dst, vector<cv::Point2f>& kp1, vector<cv::Point2f>& kp2, const cv::Scalar& color = cv::Scalar(255, 0, 0));

/// \brief      Creates cv::Matrix representing descriptor distance between sets of features
void createMatchingMatrix(cv::Mat& MatchingMatrix, const cv::Mat& desc1, const cv::Mat& desc2);

/// \brief      cv::Matches features between images given a cv::Matching cv::Matrix
void twoWayPriorityMatching(cv::Mat& matchingMatrix, vector<vector<cv::DMatch> >& bestMatches, int mode = MATCHING_MODE_NN);

/// \brief      Uses a linear SVM to reweight the cv::Match distances so that they can be more effectively sorted
double reweightDistanceWithLinearSVM(double dist, double ratio, double gradient);

/// \brief      Checks whether there is sufficient spread of features in an image for effective tracking
bool checkSufficientFeatureSpread(vector<cv::Point2f>& pts, cv::Size matSize, int minFeaturesInImage);

/// \brief      Calculates linear SVM cv::Match distance
double calcDistance(double dist, double ratio, double *coeffs);

void sortMatches(vector<vector<cv::DMatch> >& matches1to2);
void filterMatches(vector<vector<cv::DMatch> >& matches1to2, double threshold);
void constrainMatchingMatrix(cv::Mat& MatchingMatrix, vector<cv::KeyPoint>& kp1, vector<cv::KeyPoint>& kp2, int distanceConstraint, double sizeConstraint = 0.0);

void filterVectors(vector<cv::Point2f>& pts1, vector<cv::Point2f>& pts2, vector<uchar>& statusVec, double distanceConstraint = 10.0, bool epipolarCheck = false);

void assignMinimumRadii(vector<cv::KeyPoint>& pts);

double estimateSalience(cv::Mat& img, cv::Point2f& center, double radius);
double estimateStability(cv::Mat& img, cv::Point2f& center, double radius);
bool constructPatch(cv::Mat& img, cv::Mat& patch, cv::Point2f& center, double radius, int cells = 3);
double getValueFromPatch(cv::Mat& patch);
double getPatchVariance(const cv::Mat& patch) ;
void assignEigenVectorResponses(const cv::Mat& img, vector<cv::KeyPoint>& pts);
void assignStabilityResponses(const cv::Mat& img, vector<cv::KeyPoint>& pts);
void extendKeyPoints(cv::Mat& img, vector<cv::KeyPoint>& pts, bool updateStrength, bool updateLocation);

void filterBlandKeyPoints(cv::Mat& img, vector<cv::KeyPoint>& pts, double thresh = 3.0);

double distanceBetweenPoints(const cv::KeyPoint& pt1, const cv::KeyPoint& pt2);
double distanceBetweenPoints(const cv::Point2f& pt1, const cv::Point2f& pt2);

/// \brief		Sets flags to zero for tracked features that have too high an error
void filterTrackedPoints(vector<uchar>& statusVec, vector<float>& err, double maxError);

void markStationaryPoints(vector<cv::Point2f>&pts1, vector<cv::Point2f>&pts2, vector<uchar>&statusVec);

void markAbsentPoints(vector<cv::Point2f>&pts1, vector<cv::Point2f>&pts2, vector<uchar>&statusVec, cv::Size imSize);

void initializeDrawingColors(cv::Scalar* kColors, cv::Scalar* tColors, int num);

void concatenateWithExistingPoints(vector<cv::Point2f> &pts, vector<cv::Point2f>& kps, int size, double min_sep = 3, bool debug = false);

bool checkRoomForFeature(vector<cv::Point2f>& pts, cv::Point2f& candidate, double dist = 3.0);

void reduceProximalCandidates(vector<cv::Point2f>& existing, vector<cv::Point2f>& candidates);

void reduceEdgyFeatures(vector<cv::KeyPoint>& KeyPoints, cameraParameters& camData);

/// \brief Removes any keypoint that is too close to any of the provided point locations
void proximityViolationFilter(vector<cv::KeyPoint>& kps, vector<cv::Point2f>& pts, double minSep);

void markBlandTracks(cv::Mat& img, vector<cv::Point2f>& pts, vector<uchar>& statusVec, double thresh = 3.0);

void reduceUnrefinedCandidates(vector<cv::Point2f>& candidates);

void markUnrefinedPoints(vector<cv::Point2f>& pts, vector<uchar>&statusVec);

void markEdgyTracks(vector<cv::Point2f>& pts, vector<uchar>& statusVec, cameraParameters& camData);

void reduceWeakFeatures(cv::Mat& im, vector<cv::KeyPoint>& feats, double minResponse);

#endif // THERMALVIS_FEATURES_H
