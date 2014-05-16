/*! \file	geometry.hpp
 *  \brief	Declarations for geometry-related calculations.
*/

#ifdef _USE_EIGEN_

#ifndef _THERMALVIS_GEOMETRY_H_
#define _THERMALVIS_GEOMETRY_H_

#include "general_resources.hpp"
#include "opencv_resources.hpp"
#include "pcl_resources.hpp"
#include "ros_resources.hpp"

#include <Eigen/Geometry>

typedef Eigen::Matrix<float, 3, 3, Eigen::RowMajor> Matrix3frm;
typedef Eigen::Quaternion<double>   QuaternionDbl;

//#include <math.h>

// FLAGS for geometric distance measurement (F & H assessment)
#define SAMPSON_DISTANCE			0
#define ALGEBRAIC_DISTANCE			1
#define EPIPOLAR_DISTANCE			2
#define LOURAKIS_DISTANCE			3

#define DEFAULT_ASSIGN_MODE			0
#define MAPPER_ASSIGN_MODE			1

void assignPose(geometry_msgs::PoseStamped& pPose, cv::Mat& C, int idx, ros::Time timestamp, int mode = DEFAULT_ASSIGN_MODE);

void convertTvecToEigenvec(const cv::Mat& T_src, Eigen::Vector3f& T_dst);
void convertRmatTo3frm(const cv::Mat& R_src, Matrix3frm& R_dst);

void convertEigenvecToTvec(const Eigen::Vector3f& T_src, cv::Mat& T_dst);
void convert3frmToRmat(const Matrix3frm& R_src, cv::Mat& R_dst);

void decomposeTransform(const cv::Mat& c, cv::Mat& R, cv::Mat& t);
void composeTransform(const cv::Mat& R, const cv::Mat& t, cv::Mat& c);

void matrixToQuaternion(const cv::Mat& mat, Eigen::Quaternion<double>& quat);
void quaternionToMatrix(const Eigen::Quaternion<double>& quat, cv::Mat& mat, bool handedness = false);

void transformPoints(cv::vector<cv::Point3d>& pts, unsigned int option = 0);
void transformPoints(cv::vector<cv::Point3d>& pts, int *options);

/// \brief 		Minimum number of projections required to achieve the specified number of pairs
int minProjections(int pairs);

/// \brief 		Maximum possible number of pairs able to be achieved with specified number of projections
int possiblePairs(int projections);

//void matrixToQuaternion(const cv::Mat& mat, Quaterniond& quat);
//void quaternionToMatrix(const Quaterniond& quat, cv::Mat& mat);

void convertPoseFormat(const geometry_msgs::Pose& pose, cv::Mat& t, Eigen::Quaternion<double>& Q);
void convertPoseFormat(const cv::Mat& t, const Eigen::Quaternion<double>& Q, geometry_msgs::Pose& pose);

void convertAndShiftPoseFormat(const geometry_msgs::Pose& pose, cv::Mat& t, Eigen::Quaternion<double>& Q);
void convertAndShiftPoseFormat(const cv::Mat& t, const Eigen::Quaternion<double>& Q, geometry_msgs::Pose& pose);

void transformationToProjection(const cv::Mat& trans, cv::Mat& proj);
void projectionToTransformation(const cv::Mat& proj, cv::Mat& trans);
void projectionToRotation(const cv::Mat& src, cv::Mat& dst);
void rotationToProjection(const cv::Mat& src, cv::Mat& dst);

double normalizedGRICdifference(std::vector<cv::Point2f>& pts1, std::vector<cv::Point2f>& pts2, cv::Mat& F, cv::Mat& H, cv::Mat& mask_F, cv::Mat& mask_H, double& F_GRIC, double& H_GRIC);

double calculateGRIC(std::vector<cv::Point2f>& pts1, std::vector<cv::Point2f>& pts2, cv::Mat& rel, cv::Mat& mask, int d, double k, double r, double lambda_3, int distMethod);
double calculateRho(double e, double sig, double r, double lambda_3, int d);

double calcGeometryDistance(cv::Point2f& pt1, cv::Point2f& pt2, cv::Mat& mat, int distMethod = SAMPSON_DISTANCE);

double lourakisSampsonError(cv::Point2f& pt1, cv::Point2f& pt2, cv::Mat& H);



#endif

#endif