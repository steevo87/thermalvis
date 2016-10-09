/*! \file	geometry.hpp
 *  \brief	Declarations for geometry-related calculations.
*/

#ifndef THERMALVIS_GEOMETRY_H
#define THERMALVIS_GEOMETRY_H

#ifdef _USE_EIGEN_

#include "core/tools.hpp"

#include "core/general_resources.hpp"
#include "core/ros_resources.hpp"

#include <opencv2/core.hpp>

#ifdef _USE_OPENCV_VIZ_
#include <opencv2/viz.hpp>
#endif

#include <Eigen/Geometry>

typedef Eigen::Matrix<float, 3, 3, Eigen::RowMajor> Matrix3frm;
typedef Eigen::Quaternion<double>   QuaternionDbl;
typedef Eigen::Quaternion<double>   Quaterniond;
typedef Eigen::Matrix<double, 4, 1, Eigen::DontAlign> Vector4d;

//#include <math.h>

// FLAGS for geometric distance measurement (F & H assessment)
#define SAMPSON_DISTANCE			0
#define ALGEBRAIC_DISTANCE			1
#define EPIPOLAR_DISTANCE			2
#define LOURAKIS_DISTANCE			3

#define DEFAULT_ASSIGN_MODE			0
#define MAPPER_ASSIGN_MODE			1

#define MAX_TIME_GAP_FOR_INTERP					0.5

#define MAX_RVIZ_DISPLACEMENT 					1000

#define CLUSTER_MEAN_MODE			1
#define DEFAULT_MEAN_MODE			0

void assignPose(geometry_msgs::PoseStamped& pPose, cv::Mat& C, int idx, ros::Time timestamp, int mode = DEFAULT_ASSIGN_MODE);
void convertPoseFormat(const geometry_msgs::Pose& pose, cv::Mat& t, Eigen::Quaternion<double>& Q);
void convertPoseFormat(const cv::Mat& t, const Eigen::Quaternion<double>& Q, geometry_msgs::Pose& pose);
void convertAndShiftPoseFormat(const geometry_msgs::Pose& pose, cv::Mat& t, Eigen::Quaternion<double>& Q);
void convertAndShiftPoseFormat(const cv::Mat& t, const Eigen::Quaternion<double>& Q, geometry_msgs::Pose& pose);

/// \brief 		Converts translation vector from OpenCV to Eigen format
void convertTvecToEigenvec(const cv::Mat& T_src, Eigen::Vector3f& T_dst);

/// \brief 		Converts rotation matrix from OpenCV to Eigen format
void convertRmatTo3frm(const cv::Mat& R_src, Matrix3frm& R_dst);

/// \brief 		Converts translation vector from Eigen to OpenCV format
void convertEigenvecToTvec(const Eigen::Vector3f& T_src, cv::Mat& T_dst);

#ifdef _USE_OPENCV_VIZ_
/// \brief 		Converts from 4x4 OpenCV matrix to 3x4 Affine representation
void convertMatToAffine(const cv::Mat mat, cv::Affine3f& affine);
#endif

/// \brief 		Finds centroid of 3D cloud
cv::Point3d findCentroid(vector<cv::Point3d>& cloud);

/// \brief 		Finds the length of the diagonal from extreme corners of minimum enclosing prism (perpendicular do X,Y,Z axes)
double findPrismDiagonal(vector<cv::Point3d>& cloud);

/// \brief 		Converts rotation matrix from Eigen to OpenCV format
void convert3frmToRmat(const Matrix3frm& R_src, cv::Mat& R_dst);

/// \brief 		Splits OpenCV transformation matrix into rotation matrix and translation vector
void decomposeTransform(const cv::Mat& c, cv::Mat& R, cv::Mat& t);

/// \brief		Find the average point position of the dominant cluster
bool findClusterMean(const vector<cv::Point3d>& estimatedLocations, cv::Point3d& pt3d, int mode = DEFAULT_MEAN_MODE, int minEstimates = 3, double maxStandardDev = 0.1);

/// \brief 		Possible duplicate of 'composeTransform'
void findP1Matrix(cv::Mat& P1, const cv::Mat& R, const cv::Mat& t);

/// \brief 		Combines OpenCV rotation matrix and translation vector into a transformation matrix
void composeTransform(const cv::Mat& R, const cv::Mat& t, cv::Mat& c);

/// \brief 		Converts OpenCV rotation matrix to Eigen quaternion 
void matrixToQuaternion(const cv::Mat& mat, Eigen::Quaternion<double>& quat);

/// \brief 		Converts Eigen quaternion to OpenCV rotation matrix
void quaternionToMatrix(const Eigen::Quaternion<double>& quat, cv::Mat& mat, bool handedness = false);

/// \brief 		Transform a 3D point cloud according to a different coordinate system convention
void transformPoints(std::vector<cv::Point3d>& pts, unsigned int option = 0);

/// \brief 		Transform a 3D point cloud according to a different coordinate system convention
void transformPoints(std::vector<cv::Point3d>& pts, int *options);

/// \brief 		Get minimum number of projections required to achieve the specified number of pairs
int minProjections(int pairs);

/// \brief 		Get maximum possible number of pairs able to be achieved with specified number of projections
int possiblePairs(int projections);

/// \brief 		Convert from OpenCV 4x4 transformation matrix to 3x4 projection matrix
void transformationToProjection(const cv::Mat& trans, cv::Mat& proj);

/// \brief 		Convert from OpenCV 3x4 projection matrix to 4x4 transformation matrix
void projectionToTransformation(const cv::Mat& proj, cv::Mat& trans);

/// \brief 		Extract OpenCV 3x3 rotation matrix from OpenCV 3x4 projection matrix
void projectionToRotation(const cv::Mat& src, cv::Mat& dst);

/// \brief 		Convert OpenCV 3x3 rotation matrix to OpenCV 3x4 projection matrix
void rotationToProjection(const cv::Mat& src, cv::Mat& dst);

/// \brief 		Extract just the 3 translation magnitudes (X,Y,Z) between two poses
void getTranslationBetweenCameras(cv::Mat& C1, cv::Mat& C2, double *translations);

/// \brief 		Set the 3x4 pose matrix to the equivalent of the identity pose
void initializeP0(cv::Mat& P);

/// \brief 		Get the W and Z matrices, which are useful for some geometric operations
void getWandZ(cv::Mat& W, cv::Mat& Winv, cv::Mat& Z);

/// \brief 		Shift a 3D point according to a provided transformation
void transfer3dPoint(const cv::Point3d& src, cv::Point3d& dst, const cv::Mat& C);

/// \brief 		Shift a set of 3D points according to a provided transformation
void transfer3DPoints(const std::vector<cv::Point3d>& src, std::vector<cv::Point3d>& dst, const cv::Mat& C);

/// \brief 		Determine rotation relative to identity in degrees
double getRotationInDegrees(const cv::Mat& R);

/// \brief 		Get magnitude of translation vector
double getDistanceInUnits(const cv::Mat& t);

/// \brief 		Get angle difference between two quaternions
double getQuaternionAngle(const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2);

/// \brief 		Convert 3D point from OpenCV point format to matrix format
void convertPoint3dToMat(const cv::Point3d& src, cv::Mat& dst);

/// \brief 		Create default quaternion
Quaterniond defaultQuaternion();

/// \brief 		Converts OpenCV projection matrix to Eigen projection matrix
void convertProjectionMatCVToEigen(const cv::Mat& mat, Eigen::Matrix< double, 3, 4 >& m);

/// \brief 		Converts Eigen projection matrix to OpenCV projection matrix 
void convertProjectionMatEigenToCV(const Eigen::Matrix< double, 3, 4 >& m, cv::Mat& mat);

/// \brief 		Calculate the dot product between two vectors
double dotProduct(const cv::Mat& vec1, const cv::Mat& vec2);

/// \brief 		Calculate the dot product between two quaternions
double dotProduct(const Quaterniond& q1, const Quaterniond& q2);

/// \brief 		Convert from Eigen vector to OpenCV matrix 
void convertVec4dToMat(const Vector4d& vec4, cv::Mat& mat);

// Can't find where these are actually implemented!
/*
void compileTransform(cv::Mat& c, const cv::Mat& R, const cv::Mat& t);
void combineTransforms(cv::Mat& CN, const cv::Mat& C0, const cv::Mat& C1);
void decomposeTransform(const cv::Mat& c, cv::Mat& R, cv::Mat& t);
*/

bool interpolatePose(const geometry_msgs::Pose& pose1, ros::Time time1, const geometry_msgs::Pose& pose2, ros::Time time2, geometry_msgs::Pose& finalPose, ros::Time time3);
void shiftPose(const geometry_msgs::Pose& pose_src, geometry_msgs::Pose& pose_dst, cv::Mat transformation);

#endif // THERMALVIS_GEOMETRY_H

#endif // _USE_EIGEN_