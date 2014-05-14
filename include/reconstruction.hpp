/*! \file	reconstruction.hpp
 *  \brief	Declarations for triangulation and other recovery-of-3D structure functions.
*/

#ifdef _USE_PCL_

#ifndef _RECONSTRUCTION_H_
#define _RECONSTRUCTION_H_

#include "general_resources.hpp"
#include "opencv_resources.hpp"
#include "ros_resources.hpp"
#include "camera.hpp"
#include "geometry.hpp"
#include "tracks.hpp"

#include <visualization_msgs/MarkerArray.h>

// SBA Includes
#include <sba/sba.h>
#include <sba/sba_file_io.h>
#include <sba/visualization.h>

#define _USE_MATH_DEFINES
#include <math.h>

#define DEFAULT_NEAR_RADIUS 0.3

using namespace sba;

// #include <pcl/visualization/cloud_viewer.h>

#define POSITION_LIMIT				1000
#define DEFAULT_MEAN_MAX_DIST		0.20
#define DEFAULT_MEAN_MODE			0
#define CLUSTER_MEAN_MODE			1
#define MAX_REPROJECTION_DISPARITY	20.0

#define EPSILON 0.00001
// #define __SFM__DEBUG__


void getTranslationBetweenCameras(cv::Mat& C1, cv::Mat& C2, double *translations);

void triangulateTracks(vector<featureTrack>& tracks, vector<unsigned int>& indices, cameraParameters& cameraData, cv::Mat *cameras, unsigned int earliest_index, unsigned int latest_index);

unsigned int putativelyTriangulateNewTracks(vector<featureTrack>& tracks, vector<unsigned int>& indices, cameraParameters& cameraData, cv::Mat *cameras, unsigned int earliest_index, unsigned int latest_index);
unsigned int addNewPoints(vector<featureTrack>& tracks, vector<unsigned int>& indices, cameraParameters& cameraData, cv::Mat *cameras, unsigned int earliest_index, unsigned int latest_index);

void findTriangulatableTracks(vector<featureTrack>& tracks, vector<unsigned int>& indices, vector<unsigned int>& cameras, unsigned int min_length = 10);
void findTriangulatableTracks3(vector<featureTrack>& tracks, vector<unsigned int>& indices, int latest_index = -1, unsigned int min_length = 10);

void filterNearPoints(vector<featureTrack>& featureTrackVector, double x, double y, double z, double limit = DEFAULT_NEAR_RADIUS);

// For odometry node, avoids triangulating if pairs of camera views are not sufficiently spaced..
int initialTrackTriangulationDummy(vector<featureTrack>& tracks, vector<unsigned int>& indices, cameraParameters& cameraData, geometry_msgs::PoseStamped *keyframePoses, unsigned int keyframeCount, double minSeparation = 0.2, double maxSeparation = 1.0, int minEstimates = 3, double maxStandardDev = 0.1, bool handedness = false, int xCode = 0);
int initialTrackTriangulation(vector<featureTrack>& tracks, vector<unsigned int>& indices, cameraParameters& cameraData, geometry_msgs::PoseStamped *keyframePoses, unsigned int keyframeCount, double minSeparation = 0.2, double maxSeparation = 1.0, int minEstimates = 3, double maxStandardDev = 0.1, double maxReprojectionDisparity = MAX_REPROJECTION_DISPARITY);

void findP1Matrix(cv::Mat& P1, const cv::Mat& R, const cv::Mat& t);

void reduceVectorsToTrackedPoints(const vector<cv::Point2f>& points1, vector<cv::Point2f>& trackedPoints1, const vector<cv::Point2f>& points2, vector<cv::Point2f>& trackedPoints2, vector<uchar>& statusVec);

int TriangulateNewTracks(vector<featureTrack>& trackVector, const int index1, const int index2, const cv::Mat& K, const cv::Mat& K_inv, const cv::Mat& P0, const cv::Mat& P1, bool initializeOnFocalPlane = false);
void ExtractPointCloud(vector<cv::Point3d>& cloud, vector<featureTrack>& trackVector);
void Triangulate(const cv::Point2f& pt1, const cv::Point2f& pt2, const cv::Mat& K, const cv::Mat& Kinv,	const cv::Mat& P1, const cv::Mat& P2, cv::Point3d& xyzPoint, bool debug = false);

void LinearLSTriangulation(cv::Mat& dst, const cv::Point3d& u1, const cv::Mat& P1, const cv::Point3d& u2, const cv::Mat& P2);
void Triangulate_1(const cv::Point2d& pt1, const cv::Point2d& pt2, const cv::Mat& K, const cv::Mat& Kinv, const cv::Mat& P1, const cv::Mat& P2, cv::Point3d& xyzPoint, bool iterate = true);
void IterativeLinearLSTriangulation(cv::Mat& dst, const cv::Point3d& u1, const cv::Mat& P1, const cv::Point3d& u2, const cv::Mat& P2);


bool findClusterMean(const vector<cv::Point3d>& estimatedLocations, cv::Point3d& pt3d, int mode = DEFAULT_MEAN_MODE, int minEstimates = 3, double maxStandardDev = 0.1);

void TriangulatePoints(const vector<cv::Point2f>& pt_set1,
                       const vector<cv::Point2f>& pt_set2,
                       const cv::Mat& K,
                       const cv::Mat& Kinv,
                       const cv::Mat& P,
                       const cv::Mat& P1,
                       vector<cv::Point3d>& pointcloud,
                       vector<cv::Point2f>& correspImg1Pt);
                       
void initializeP0(cv::Mat& P);
void getWandZ(cv::Mat& W, cv::Mat& Winv, cv::Mat& Z);

bool findBestReconstruction(const cv::Mat& P0, cv::Mat& P1, cv::Mat& R, cv::Mat& t, const cv::SVD& svd, const cv::Mat& K, const vector<cv::Point2f>& pts1, const vector<cv::Point2f>& pts2, bool useDefault = false);

int addToTracks(SysSBA& sys, int im1, vector<cv::Point2f>& pts1, int im2, vector<cv::Point2f>& pts2);

void addFixedCamera(SysSBA& sys, cameraParameters& cameraData, const cv::Mat& C);
void addNewCamera(SysSBA& sys, cameraParameters& cameraData, const cv::Mat& C);
void updateCameraNode_2(SysSBA& sys, const cv::Mat& R, const cv::Mat& t, int image_index);
void updateCameraNode_2(SysSBA& sys, const cv::Mat& C, int image_index);
void addBlankCamera(SysSBA& sys, cameraParameters& cameraData, bool isFixed = false);

double retrieveCameraPose(const SysSBA& sys, unsigned int idx, geometry_msgs::Pose& pose);
void retrieveCameraPose(const SysSBA& sys, unsigned int idx, cv::Mat& camera);
void retrieveAllCameras(cv::Mat *allCameraPoses, const SysSBA& sys);
void retrieveAllPoints(vector<cv::Point3d>& pts, const SysSBA& sys);
void updateTracks(vector<featureTrack>& trackVector, const SysSBA& sys);

void updateCameraNode(SysSBA& sys, cv::Mat R, cv::Mat t, int img1, int img2);
void printSystemSummary(SysSBA& sys);



float ReciprocalSqrt( float x );

// maxIndex says what's the latest frame for which tracks should be included
void assignTracksToSBA(SysSBA& sys, vector<featureTrack>& trackVector, int maxIndex);

void transfer3dPoint(const cv::Point3d& src, cv::Point3d& dst, const cv::Mat& C);

void transfer3DPoints(const vector<cv::Point3d>& src, vector<cv::Point3d>& dst, const cv::Mat& C);

void addNewPoints(SysSBA& sys, const vector<cv::Point3d>& pc);

double getRotationInDegrees(const cv::Mat& R);
double getDistanceInUnits(const cv::Mat& t);

double getQuaternionAngle(const Quaterniond& q1, const Quaterniond& q2);

void convertPoint3dToMat(const cv::Point3d& src, cv::Mat& dst);

Quaterniond defaultQuaternion();

void convertProjectionMatCVToEigen(const cv::Mat& mat, Eigen::Matrix< double, 3, 4 > m);
void convertProjectionMatEigenToCV(const Eigen::Matrix< double, 3, 4 > m, cv::Mat& mat);

double dotProduct(const cv::Mat& vec1, const cv::Mat& vec2);
double dotProduct(const Quaterniond& q1, const Quaterniond& q2);

void constrainDodgyPoints(SysSBA& sys);

//void convertFromMatToQuaternion(const Mat& mat, Quaterniond& quat);

void convertVec4dToMat(const Vector4d& vec4, cv::Mat& mat);

void findFourTransformations(cv::Mat *C, const cv::Mat& E, const cv::Mat& K, const vector<cv::Point2f>& pts1, const vector<cv::Point2f>& pts2);

// I think these are known as a cheirality test?
int pointsInFront(const cv::Mat& C1, const cv::Mat& C2, const vector<cv::Point3d>& pts);
int pointsInFront(const cv::Mat& C1, const cv::Mat& K, const vector<cv::Point2f>& pts1, const vector<cv::Point2f>& pts2);

void compileTransform(cv::Mat& c, const cv::Mat& R, const cv::Mat& t);
void combineTransforms(cv::Mat& CN, const cv::Mat& C0, const cv::Mat& C1);
void decomposeTransform(const cv::Mat& c, cv::Mat& R, cv::Mat& t);

double calcFrameScore(double geomScore, int numFeatures, int numTracks);

//double calcGeometryScore(vector<Point2f>& points1, vector<Point2f>& points2, Mat& F, Mat& Fmask, Mat& H, Mat& Hmask);
double calcGeometryScore(int numInliers_H, int numInliers_F, double sampsonError_H, double sampsonError_F);

double calcSampsonError(vector<cv::Point2f>& points1, vector<cv::Point2f>& points2, cv::Mat& H, cv::Mat& Hmask);

double calcSampsonDistance(cv::Point2f& pt1, cv::Point2f& pt2, cv::Mat& F);

double calcInlierGeometryDistance(vector<cv::Point2f>& points1, vector<cv::Point2f>& points2, cv::Mat& mat, cv::Mat& mask, int distMethod = SAMPSON_DISTANCE);

void assignIntrinsicsToP0(cv::Mat& P0, const cv::Mat& K);

// I think this will give you all of the points that were successfully tracked from idx1 to idx2
void getPointsFromTracks(vector<featureTrack>& tracks, vector<cv::Point2f>& pts1, vector<cv::Point2f>& pts2, int idx1, int idx2);
void getPoints3dFromTracks(vector<featureTrack>& tracks, vector<cv::Point3d>& cloud, int idx1 = -1, int idx2 = -1);

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

void updateSystemTracks(SysSBA& sys, vector<featureTrack>& tracks, unsigned int start_index);

int findBestCandidate(const cv::Mat *CX, const cv::Mat& K, const vector<cv::Point2f>& pts1, const vector<cv::Point2f>& pts2, cv::Mat& C);

void summarizeTransformation(const cv::Mat& C, char *summary);

void reverseTranslation(cv::Mat& C);

void removeShortTracks(vector<featureTrack>& tracks, int idx1, int idx2);

bool pointIsInFront(const cv::Mat& C, const cv::Point3d& pt);

void getIndicesForTriangulation(vector<unsigned int>& dst, vector<unsigned int>& src, vector<unsigned int>& already_triangulated);

void reconstructSubsequence(vector<featureTrack>& tracks, vector<cv::Point3d>& ptCloud, int idx1, int idx2);



void findCentroid(vector<featureTrack>& tracks, cv::Point3d& centroid, cv::Point3d& stdDeviation);

// Between two images, gets all 2D point locations and 3D locations needed for BA
void getTriangulatedFullSpanPoints(vector<featureTrack>& tracks, vector<cv::Point2f>& pts1, vector<cv::Point2f>& pts2, int idx1, int idx2, vector<cv::Point3f>& points3);

bool estimatePoseFromKnownPoints(cv::Mat& dst, cameraParameters camData, vector<featureTrack>& tracks, unsigned int index, const cv::Mat& guide, unsigned minAppearances = 1, unsigned int iterCount = 100, double maxReprojErr = 4.0, double inliersPercentage = 0.9, double *reprojError = 0, double *pnpInlierProp = 0, bool debug = false);
	
#endif

#endif