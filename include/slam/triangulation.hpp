/*! \file	triangulation.hpp
 *  \brief	Declarations for triangulating 3D points from camera poses and 2D projections.
*/

#ifndef THERMALVIS_TRIANGULATION_H
#define THERMALVIS_TRIANGULATION_H

#include "core/tracks.hpp"
#include "slam/geometry.hpp"
#include "slam/cloudproc.hpp"

#define MIN_PAIRS_OF_PROJECTIONS_FOR_TRIANGULATION 10
#define DEFAULT_NEAR_RADIUS 0.3
#define EPSILON 0.00001
#define MAX_REPROJECTION_DISPARITY	20.0

/// \brief 		Determine 3D position estimates for tracks with provided indices
void triangulateTracks(std::vector<featureTrack>& tracks, vector<unsigned int>& indices, cameraParameters& cameraData, cv::Mat *cameras, unsigned int earliest_index, unsigned int latest_index);

/// \brief 		Possible duplicate of 'triangulateTracks'
unsigned int putativelyTriangulateNewTracks(std::vector<featureTrack>& tracks, std::vector<unsigned int>& indices, cameraParameters& cameraData, cv::Mat *cameras, unsigned int earliest_index, unsigned int latest_index);

/// \brief 		Possible duplicate of 'triangulateTracks'
unsigned int addNewPoints(std::vector<featureTrack>& tracks, std::vector<unsigned int>& indices, cameraParameters& cameraData, cv::Mat *cameras, unsigned int earliest_index, unsigned int latest_index);

/// \brief 		Finds indices of tracks that have sufficient projections to be triangulated
void findTriangulatableTracks(std::vector<featureTrack>& tracks, std::vector<unsigned int>& indices, std::vector<unsigned int>& cameras, unsigned int min_length = 10);

/// \brief 		Possible duplicate of 'findTriangulatableTracks'
void findTriangulatableTracks3(std::vector<featureTrack>& tracks, std::vector<unsigned int>& indices, int latest_index = -1, unsigned int min_length = 10);

/// \brief 		Removes triangulations within a specified distance of the provided 3D point (x,y,z)
void filterNearPoints(std::vector<featureTrack>& featureTrackVector, double x, double y, double z, double limit = DEFAULT_NEAR_RADIUS);

/// \brief 		Avoids triangulating if pairs of camera views are not sufficiently spaced..
int initialTrackTriangulation(vector<featureTrack>& tracks, vector<unsigned int>& indices, cameraParameters& cameraData, geometry_msgs::PoseStamped *keyframePoses, unsigned int keyframeCount, double minSeparation = 0.2, double maxSeparation = 1.0, int minEstimates = 3, double maxStandardDev = 0.1, double maxReprojectionDisparity = MAX_REPROJECTION_DISPARITY);
/// \brief 		Dummy version of 'initialTrackTriangulation'
int initialTrackTriangulationDummy(vector<featureTrack>& tracks, vector<unsigned int>& indices, cameraParameters& cameraData, geometry_msgs::PoseStamped *keyframePoses, unsigned int keyframeCount, double minSeparation = 0.2, double maxSeparation = 1.0, int minEstimates = 3, double maxStandardDev = 0.1, bool handedness = false, int xCode = 0);

/// \brief 		Possible duplicate of 'triangulateTracks'
int TriangulateNewTracks(vector<featureTrack>& trackVector, const int index1, const int index2, const cv::Mat& K, const cv::Mat& K_inv, const cv::Mat& P0, const cv::Mat& P1, bool initializeOnFocalPlane = false);

/// \brief 		Triangulate a single pair of 2D projections based on camera information
void Triangulate(const cv::Point2f& pt1, const cv::Point2f& pt2, const cv::Mat& K, const cv::Mat& Kinv,	const cv::Mat& P1, const cv::Mat& P2, cv::Point3d& xyzPoint, bool debug = false);

/// \brief 		Linear Least Squares Triangulation 	(https://github.com/MasteringOpenCV/code/blob/master/Chapter4_StructureFromMotion/Triangulation.cpp, http://www.morethantechnical.com/2012/01/04/simple-triangulation-with-opencv-from-harley-zisserman-w-code/)
void LinearLSTriangulation(cv::Mat& dst, const cv::Point3d& u1, const cv::Mat& P1, const cv::Point3d& u2, const cv::Mat& P2);

/// \brief 		Possible duplicate of 'Triangulate'
void Triangulate_1(const cv::Point2d& pt1, const cv::Point2d& pt2, const cv::Mat& K, const cv::Mat& Kinv, const cv::Mat& P1, const cv::Mat& P2, cv::Point3d& xyzPoint, bool iterate = true);

/// \brief 		Iterative Linear Least Squares Triangulation
void IterativeLinearLSTriangulation(cv::Mat& dst, const cv::Point3d& u1, const cv::Mat& P1, const cv::Point3d& u2, const cv::Mat& P2);

/// \brief 		Triangulate a set of 2D projection pairs with camera information
void TriangulatePoints(const vector<cv::Point2f>& pt_set1, const vector<cv::Point2f>& pt_set2, const cv::Mat& K, const cv::Mat& Kinv, const cv::Mat& P, const cv::Mat& P1, vector<cv::Point3d>& pointcloud, vector<cv::Point2f>& correspImg1Pt);

// \brief		I think this will give you all of the 3D locations for points that were successfully tracked from idx1 to idx2
void getPoints3dFromTracks(vector<featureTrack>& tracks, vector<cv::Point3d>& cloud, int idx1 = -1, int idx2 = -1);

bool pointIsInFront(const cv::Mat& C, const cv::Point3d& pt);

#endif // THERMALVIS_TRIANGULATION_H