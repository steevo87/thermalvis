/*! \file	sba.hpp
 *  \brief	Declarations for sparse bundle adjustment related functions.
*/

#ifndef _THERMALSFM_SBA_H_
#define _THERMALSFM_SBA_H_

#ifdef _USE_PCL_
#ifdef _USE_SBA_

#define SBA_MEMORY								134217728

#include "general_resources.hpp"
#include "opencv_resources.hpp"
#include "ros_resources.hpp"
#include "reconstruction.hpp"
#include "keyframes.hpp"
#include "tracks.hpp"
// #include <ros/ros.h>


#include <sba/sba.h>
#include <sba/visualization.h>


#define DEFAULT_MAX_DISPLACEMENT 	0.3

// For random seed.
#include <time.h>
//#include <visualization_msgs/Marker.h>

/// \brief		Scales up system size

void normalizeSystem(SysSBA& sys, double factor = 1.0);

/// \brief		Determines distance between farthest cameras
double determineSystemSize(SysSBA& sys);

double distanceBetweenPoints(Eigen::Matrix<double, 4, 1>& pt1, Eigen::Matrix<double, 4, 1>& pt2);

void drawKeyframes(const ros::Publisher &camera_pub, const geometry_msgs::PoseStamped *keyframePoses, unsigned int keyframeCount);

bool overwritePoints(const SysSBA& sys, const vector<int>& track_indices, vector<featureTrack>& tracks, double maxDisplacement = DEFAULT_MAX_DISPLACEMENT, double *averagePointShift = 0, bool debug = false);

void rescaleSBA(SysSBA& sba, unsigned int idx1, unsigned int idx2);

void optimizeFullSystem(vector<featureTrack>& tracks, cameraParameters& camData, cv::Mat *cameras, unsigned int last_index);

void findIntermediatePoses(vector<featureTrack>& tracks, cameraParameters& camData, cv::Mat *cameras, unsigned int image_idx_1, unsigned int image_idx_2, bool fixBothEnds = true);

//void createSupersys(SysSBA &sba, vector<featureTrack>& tracks, unsigned int total_cameras);

void assignFullSystem(SysSBA &sba, vector<featureTrack>& tracks, cameraParameters& camData, cv::Mat *cameras, unsigned int start_index, unsigned int finish_index, bool dummy = false);

void assignPartialSystem(SysSBA &sba, vector<featureTrack>& tracks, cameraParameters& camData, cv::Mat *cameras, vector<unsigned int>& indices, bool assignProjections = true);

int estimatePoseBetweenCameras(cameraParameters& camData, vector<cv::Point2f>& pts1, vector<cv::Point2f>& pts2, cv::Mat& C);

double optimizeSystem(SysSBA &sba, double err = 1e-3, int iterations = 100, bool debug = false, int mode = SBA_SPARSE_CHOLESKY);

double twoViewBundleAdjustment(cameraParameters cam_data, cv::Mat& cam1, cv::Mat& cam2, vector<cv::Point3d>& cloud, vector<cv::Point2f>& pts1, vector<cv::Point2f>& pts2, int iterations = 10); 

void initializeFrameCamera(frame_common::CamParams& cam_params, const cv::Mat& newCamMat, int& maxx, int& maxy, const cv::Size& cameraSize);

void drawGraph2(const SysSBA &sba, const ros::Publisher &camera_pub, const ros::Publisher &point_pub, const ros::Publisher &path_pub, int decimation = 1, int bicolor = 0, double scale = 0.1);

void extractPointCloud(const SysSBA &sba, pcl::PointCloud<pcl::PointXYZ>& point_cloud);

void extractCameras(const SysSBA &sba, visualization_msgs::MarkerArray& cameraArray, visualization_msgs::Marker& marker_path);

void addPointsToSBA(SysSBA& sba, vector<cv::Point3d>& cloud);

void addProjectionsToSBA(SysSBA& sba, vector<cv::Point2f>& loc, int idx);

double optimizeKeyframePair(vector<featureTrack>& tracks, cameraParameters& camData, int idx1, int idx2, cv::Mat *cameras);

void addProjectionToSBA(SysSBA& sba, cv::Point2f& loc, unsigned int trackNo, unsigned int camNo);

void removePoorTracks(vector<featureTrack>& tracks, cameraParameters& camData, cv::Mat *cameras, unsigned int start_cam, unsigned int finish_cam);

double keyframeBundleAdjustment(cameraParameters& camData, vector<featureTrack>& tracks, cv::Mat *cameras, vector<unsigned int>& indices, unsigned int iterations = 100, bool allFree = false, bool allFixedExceptLast = false, unsigned fixed_cams = 0);

double keyframeBundleAdjustment(cameraParameters& camData, vector<featureTrack>& tracks, keyframeStore& kf_store, cv::Mat *cameras, unsigned int kfIndex, unsigned int iterations = 100);

double predictiveBundleAdjustment(cameraParameters& camData, vector<featureTrack>& tracks, geometry_msgs::PoseStamped *keyframePoses, bool *keyframeTypes, unsigned int keyframeCount, geometry_msgs::PoseStamped& newPose, unsigned int iterations = 100, bool debug = false, int mode = SBA_SPARSE_CHOLESKY, double err = 1e-3, int *triangulations = 0, double *averagePointShift = 0);

double odometryBundleAdjustment(cameraParameters& camData, vector<featureTrack>& tracks, geometry_msgs::PoseStamped *keyframePoses, unsigned int keyframeCount, unsigned int iterations = 100, bool debug = false);

void getActiveCameras(cv::Mat *C, vector<unsigned int>& indices, unsigned int min_index, unsigned int max_index);

void getActiveTracks(vector<featureTrack>& tracks, vector<unsigned int>& cameras, vector<unsigned int>& indices);
void assignSystem(SysSBA &sba, vector<featureTrack>& tracks, cameraParameters& camData, cv::Mat *cameras, vector<unsigned int>& camera_indices, vector<unsigned int>& track_indices);
void retrieveSystem(SysSBA &sba, vector<featureTrack>& tracks, cameraParameters& camData, cv::Mat *cameras, vector<unsigned int>& camera_indices, vector<unsigned int>& track_indices);

void retrieveFullSystem(SysSBA& sys, cv::Mat *C, vector<featureTrack>& tracks, unsigned int start_cam, unsigned int final_cam);

double adjustFullSystem(vector<featureTrack>& tracks, cameraParameters& camData, cv::Mat *cameras, unsigned int min_index, unsigned int max_index, unsigned int its = 1);

void retrievePartialSystem(SysSBA& sys, cv::Mat *C, vector<featureTrack>& tracks, vector<unsigned int>& indices);
// SBA_BLOCK_JACOBIAN_PCG   3
// SBA_DENSE_CHOLESKY   0
// SBA_GRADIENT   2
// SBA_SPARSE_CHOLESKY   1

void renormalizeSBA(SysSBA& sba, cv::Point3d& desiredCenter);

void findCentroid(SysSBA& sba, cv::Point3d& centroid, cv::Point3d& stdDeviation);

void copySys(const SysSBA& src, SysSBA& dst);

double optimizeSubsystem(cameraParameters& camData, cv::Mat *C, vector<unsigned int>& c_i, vector<featureTrack>& tracks, vector<unsigned int>& t_i, unsigned int iterations = 100);

void finishTracks(vector<featureTrack>& tracks, vector<cv::Point2f>& pts, double retainProp = 0.80, unsigned int maxOccurrences = 0);

int addToTracks(SysSBA& sys, int im1, vector<cv::Point2f>& pts1, int im2, vector<cv::Point2f>& pts2);

void addFixedCamera(SysSBA& sys, cameraParameters& cameraData, const cv::Mat& C);
void addNewCamera(SysSBA& sys, cameraParameters& cameraData, const cv::Mat& C);
void updateCameraNode_2(SysSBA& sys, const cv::Mat& R, const cv::Mat& t, int image_index);
void updateCameraNode_2(SysSBA& sys, const cv::Mat& C, int image_index);
void addBlankCamera(SysSBA& sys, cameraParameters& cameraData, bool isFixed = false);

#ifdef _BUILD_FOR_ROS_
double retrieveCameraPose(const SysSBA& sys, unsigned int idx, geometry_msgs::Pose& pose);
#endif

void retrieveCameraPose(const SysSBA& sys, unsigned int idx, cv::Mat& camera);
void retrieveAllCameras(cv::Mat *allCameraPoses, const SysSBA& sys);
void retrieveAllPoints(vector<cv::Point3d>& pts, const SysSBA& sys);
void updateTracks(vector<featureTrack>& trackVector, const SysSBA& sys);

void updateCameraNode(SysSBA& sys, cv::Mat R, cv::Mat t, int img1, int img2);
void printSystemSummary(SysSBA& sys);

void assignTracksToSBA(SysSBA& sys, vector<featureTrack>& trackVector, int maxIndex);

void addNewPoints(SysSBA& sys, const vector<cv::Point3d>& pc);
void constrainDodgyPoints(SysSBA& sys);

void updateSystemTracks(SysSBA& sys, vector<featureTrack>& tracks, unsigned int start_index);


#endif

#endif
#endif