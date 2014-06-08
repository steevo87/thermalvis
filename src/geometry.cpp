/*! \file	geometry.cpp
 *  \brief	Definitions for geometry-related calculations.
*/

#ifdef _USE_EIGEN_

#include "geometry.hpp"

/// \brief 		Minimum number of projections required to achieve the specified number of pairs
int minProjections(int pairs) {
	double retVal = ( -1.0 + pow(1.0 + 8.0 * double(pairs), 0.5) ) / 2.0;
	//printf("%s << retVal(%d) = (%f)\n", __FUNCTION__, pairs, retVal);
	return int(ceil(retVal+1.0));
}

/// \brief 		Maximum possible number of pairs able to be achieved with specified number of projections
int possiblePairs(int projections) {
	double retVal = 0.5 * double(std::max(((int)projections-1),0)) * ( double(std::max(((int)projections-1),0)) + 1 );
	return int(retVal);
}

void transformPoints(std::vector<cv::Point3d>& pts, int *options) {
	
	int signs[3];
	
	signs[0] = (options[2] == 0) - (options[2] == 1);
	signs[1] = (options[3] == 0) - (options[3] == 1);
	signs[2] = (options[4] == 0) - (options[4] == 1);
	
	for (unsigned int iii = 0; iii < pts.size(); iii++) {
		
		double x, y, z;
		
		if (options[0] == 0) {
			x = pts.at(iii).x;
			
			if (options[1] == 0) {
				y = pts.at(iii).y;
				z = pts.at(iii).z;
			} else {
				y = pts.at(iii).z;
				z = pts.at(iii).y;
			}
			
		} else if (options[0] == 1) {
			x = pts.at(iii).y;
			
			if (options[1] == 0) {
				y = pts.at(iii).x;
				z = pts.at(iii).z;
			} else {
				y = pts.at(iii).z;
				z = pts.at(iii).x;
			}
			
		} else  {
			
			if (options[0] != 2) { printf("%s << ERROR!\n", __FUNCTION__); }
			x = pts.at(iii).z;
			
			if (options[1] == 0) {
				y = pts.at(iii).x;
				z = pts.at(iii).y;
			} else {
				y = pts.at(iii).y;
				z = pts.at(iii).x;
			}
			
		}
		
		pts.at(iii).x = signs[0] * x;
		pts.at(iii).y = signs[1] * y;
		pts.at(iii).z = signs[2] * z;
			
	}
	
}

void transformPoints(std::vector<cv::Point3d>& pts, unsigned int option) {
	
	
	for (unsigned int iii = 0; iii < pts.size(); iii++) {
		
		double x, y, z;
		x = pts.at(iii).x;
		y = pts.at(iii).y;
		z = pts.at(iii).z;
		
		if (option == 0) {
			pts.at(iii).x = x;
			pts.at(iii).y = -y;
			pts.at(iii).z = z;
		} else if (option == 1) {	// This one seems to match what you do to pose!
			pts.at(iii).x = -y;
			pts.at(iii).y = -z;
			pts.at(iii).z = x;
		} else if (option == 2) {
			pts.at(iii).x = z;
			pts.at(iii).y = -x;
			pts.at(iii).z = y;
		} else if (option == 3) {
			pts.at(iii).x = y;
			pts.at(iii).y = -z;
			pts.at(iii).z = x;
		} else if (option == 4) {
			pts.at(iii).x = -y;
			pts.at(iii).y = z;
			pts.at(iii).z = x;

		}
	}
	
}

#ifdef _BUILD_FOR_ROS_
void assignPose(geometry_msgs::PoseStamped& pPose, cv::Mat& C, int idx, ros::Time timestamp, int mode) {
	
	pPose.header.seq = idx;
	pPose.header.stamp = timestamp;
   
	cv::Mat R, t;
	Eigen::Quaterniond Q;
		
	decomposeTransform(C, R, t);
	matrixToQuaternion(R, Q);
	
	if (mode == MAPPER_ASSIGN_MODE) {
		QuaternionDbl Qrot;
		
		/*
		Qrot.x() = 0;
		Qrot.y() = -0.70711;
		Qrot.z() = 0;
		Qrot.w() = 0.70711;
		
		Q = Q * Qrot;
		*/
		
	}
   
	// tried: 1,0,2; 1,2,0; 0,2,1; 2,0,1; 2,1,0; 0,1,2
	// x-corresponds to graph -x; y to graph -z; z to graph -y
	
	// TRIED: 
	// 2,-0,-1
	// 2, 0, 1 (better)
	// 2, 0,-1
	// 2,-0, 1
	// 0, 1, 2
   
	if (mode == MAPPER_ASSIGN_MODE) {
		pPose.pose.position.x = t.at<double>(0,0); //;
		pPose.pose.position.y = t.at<double>(1,0); //t.at<double>(1,0);
		pPose.pose.position.z = t.at<double>(2,0); //t.at<double>(2,0);
	
	} else {
		
		pPose.pose.position.x = t.at<double>(2,0); //;
		pPose.pose.position.y = -t.at<double>(0,0); //t.at<double>(1,0);
		pPose.pose.position.z = -t.at<double>(1,0); //t.at<double>(2,0);
	   
	}
   
	if (abs(pPose.pose.position.x) > MAX_RVIZ_DISPLACEMENT) {
			pPose.pose.position.x = 0.0;
	}

	if (abs(pPose.pose.position.y) > MAX_RVIZ_DISPLACEMENT) {
			pPose.pose.position.y = 0.0;
	}
   
	if (abs(pPose.pose.position.z) > MAX_RVIZ_DISPLACEMENT) {
			pPose.pose.position.z = 0.0;
	}
   
	//printf("%s << QUAT = (%f, %f, %f, %f)", __FUNCTION__, Q.x(), Q.y(), Q.z(), Q.w());
   
	// tried x,y,z,w
	
	if (mode == MAPPER_ASSIGN_MODE) {
		pPose.pose.orientation.x = Q.x();
		pPose.pose.orientation.y = Q.y();
		pPose.pose.orientation.z = Q.z();
		pPose.pose.orientation.w = Q.w();
	} else {
		pPose.pose.orientation.x = Q.z();
		pPose.pose.orientation.y = -Q.x();
		pPose.pose.orientation.z = -Q.y();
		pPose.pose.orientation.w = Q.w();
	}
}

void convertPoseFormat(const cv::Mat& t, const Eigen::Quaternion<double>& Q, geometry_msgs::Pose& pose) {
	
	pose.position.x = t.at<double>(0,0);
	pose.position.y = t.at<double>(1,0);
	pose.position.z = t.at<double>(2,0);
	
	pose.orientation.w = Q.w();
	pose.orientation.x = Q.x();
	pose.orientation.y = Q.y();
	pose.orientation.z = Q.z();
	
}

void convertPoseFormat(const geometry_msgs::Pose& pose, cv::Mat& t, Eigen::Quaternion<double>& Q) {
	
	t = cv::Mat(3,1, CV_64FC1);
	
	t.at<double>(0,0) = pose.position.x;
	t.at<double>(1,0) = pose.position.y;
	t.at<double>(2,0) = pose.position.z;
	
	Q = Eigen::Quaternion<double>(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
	
}

void convertAndShiftPoseFormat(const geometry_msgs::Pose& pose, cv::Mat& t, Eigen::Quaternion<double>& Q) {
	
	t = cv::Mat(3,1, CV_64FC1);
	
	t.at<double>(2,0) = pose.position.x;
	t.at<double>(0,0) = -pose.position.y;
	t.at<double>(1,0) = -pose.position.z;
	
	Q = Eigen::Quaternion<double>(pose.orientation.w, pose.orientation.x, -pose.orientation.y, -pose.orientation.z);
	
}

void convertAndShiftPoseFormat(const cv::Mat& t, const Eigen::Quaternion<double>& Q, geometry_msgs::Pose& pose) {
	
	pose.position.x = t.at<double>(2,0);
	pose.position.y = -t.at<double>(0,0);
	pose.position.z = -t.at<double>(1,0);
	
	pose.orientation.x = Q.z();
	pose.orientation.y = -Q.x();
	pose.orientation.z = -Q.y();
	pose.orientation.w = Q.w();
	
}
#endif

void convertRmatTo3frm(const cv::Mat& R_src, Matrix3frm& R_dst) {
	R_dst(0,0) = float(R_src.at<double>(0,0));
	R_dst(0,1) = float(R_src.at<double>(0,1));
	R_dst(0,2) = float(R_src.at<double>(0,2));
	
	R_dst(1,0) = float(R_src.at<double>(1,0));
	R_dst(1,1) = float(R_src.at<double>(1,1));
	R_dst(1,2) = float(R_src.at<double>(1,2));
	
	R_dst(2,0) = float(R_src.at<double>(2,0));
	R_dst(2,1) = float(R_src.at<double>(2,1));
	R_dst(2,2) = float(R_src.at<double>(2,2));
}

void convertTvecToEigenvec(const cv::Mat& T_src, Eigen::Vector3f& T_dst) {
	T_dst[0] = float(T_src.at<double>(0,0));
	T_dst[1] = float(T_src.at<double>(1,0));
	T_dst[2] = float(T_src.at<double>(2,0));
}

void convertEigenvecToTvec(const Eigen::Vector3f& T_src, cv::Mat& T_dst) {
	T_dst = cv::Mat::zeros(3,1,CV_64FC1);
	
	T_dst.at<double>(0,0) = T_src[0];
	T_dst.at<double>(1,0) = T_src[1];
	T_dst.at<double>(2,0) = T_src[2];
}

void convert3frmToRmat(const Matrix3frm& R_src, cv::Mat& R_dst) {
	R_dst = cv::Mat::eye(3,3,CV_64FC1);
	
	R_dst.at<double>(0,0) = R_src(0,0);
	R_dst.at<double>(0,1) = R_src(0,1);
	R_dst.at<double>(0,2) = R_src(0,2);
	
	R_dst.at<double>(1,0) = R_src(1,0);
	R_dst.at<double>(1,1) = R_src(1,1);
	R_dst.at<double>(1,2) = R_src(1,2);
	
	R_dst.at<double>(2,0) = R_src(2,0);
	R_dst.at<double>(2,1) = R_src(2,1);
	R_dst.at<double>(2,2) = R_src(2,2);
}

void projectionToTransformation(const cv::Mat& proj, cv::Mat& trans) {
	trans = cv::Mat::eye(4, 4, CV_64FC1);
	
	trans.at<double>(0,0) = proj.at<double>(0,0);
	trans.at<double>(0,1) = proj.at<double>(0,1);
	trans.at<double>(0,2) = proj.at<double>(0,2);
	trans.at<double>(0,3) = proj.at<double>(0,3);
	
	trans.at<double>(1,0) = proj.at<double>(1,0);
	trans.at<double>(1,1) = proj.at<double>(1,1);
	trans.at<double>(1,2) = proj.at<double>(1,2);
	trans.at<double>(1,3) = proj.at<double>(1,3);
	
	trans.at<double>(2,0) = proj.at<double>(2,0);
	trans.at<double>(2,1) = proj.at<double>(2,1);
	trans.at<double>(2,2) = proj.at<double>(2,2);
	trans.at<double>(2,3) = proj.at<double>(2,3);
	
	trans.at<double>(3,0) = 0.0;
	trans.at<double>(3,1) = 0.0;
	trans.at<double>(3,2) = 0.0;
	trans.at<double>(3,3) = 1.0;
	
}

void transformationToProjection(const cv::Mat& trans, cv::Mat& proj) {
	proj = cv::Mat::zeros(3, 4, CV_64FC1);
	
	proj.at<double>(0,0) = trans.at<double>(0,0);
	proj.at<double>(0,1) = trans.at<double>(0,1);
	proj.at<double>(0,2) = trans.at<double>(0,2);
	proj.at<double>(0,3) = trans.at<double>(0,3);
	
	proj.at<double>(1,0) = trans.at<double>(1,0);
	proj.at<double>(1,1) = trans.at<double>(1,1);
	proj.at<double>(1,2) = trans.at<double>(1,2);
	proj.at<double>(1,3) = trans.at<double>(1,3);
	
	proj.at<double>(2,0) = trans.at<double>(2,0);
	proj.at<double>(2,1) = trans.at<double>(2,1);
	proj.at<double>(2,2) = trans.at<double>(2,2);
	proj.at<double>(2,3) = trans.at<double>(2,3);
		
}

void projectionToRotation(const cv::Mat& src, cv::Mat& dst) {
	dst = cv::Mat(3, 3, CV_64FC1);
	
	dst.at<double>(0,0) = src.at<double>(0,0);
	dst.at<double>(0,1) = src.at<double>(0,1);
	dst.at<double>(0,2) = src.at<double>(0,2);

	dst.at<double>(1,0) = src.at<double>(1,0);
	dst.at<double>(1,1) = src.at<double>(1,1);
	dst.at<double>(1,2) = src.at<double>(1,2);
	
	dst.at<double>(2,0) = src.at<double>(2,0);
	dst.at<double>(2,1) = src.at<double>(2,1);
	dst.at<double>(2,2) = src.at<double>(2,2);
	
}

void rotationToProjection(const cv::Mat& src, cv::Mat& dst) {
	dst = cv::Mat(3, 4, CV_64FC1);
	
	dst.at<double>(0,0) = src.at<double>(0,0);
	dst.at<double>(0,1) = src.at<double>(0,1);
	dst.at<double>(0,2) = src.at<double>(0,2);
	
	dst.at<double>(1,0) = src.at<double>(1,0);
	dst.at<double>(1,1) = src.at<double>(1,1);
	dst.at<double>(1,2) = src.at<double>(1,2);
	
	dst.at<double>(2,0) = src.at<double>(2,0);
	dst.at<double>(2,1) = src.at<double>(2,1);
	dst.at<double>(2,2) = src.at<double>(2,2);
	
	dst.at<double>(0,3) = 0.0;	
	dst.at<double>(1,3) = 0.0;
	dst.at<double>(2,3) = 0.0;
	
}

void composeTransform(const cv::Mat& R, const cv::Mat& t, cv::Mat& c) {
	
	c = cv::Mat::zeros(4, 4, CV_64FC1);
	
	c.at<double>(0,0) = R.at<double>(0,0);
	c.at<double>(0,1) = R.at<double>(0,1);
	c.at<double>(0,2) = R.at<double>(0,2);

	c.at<double>(1,0) = R.at<double>(1,0);
	c.at<double>(1,1) = R.at<double>(1,1);
	c.at<double>(1,2) = R.at<double>(1,2);

	c.at<double>(2,0) = R.at<double>(2,0);
	c.at<double>(2,1) = R.at<double>(2,1);
	c.at<double>(2,2) = R.at<double>(2,2);

	c.at<double>(0,3) = t.at<double>(0,0);
	c.at<double>(1,3) = t.at<double>(1,0);
	c.at<double>(2,3) = t.at<double>(2,0);
	
	c.at<double>(3,0) = 0.0;
	c.at<double>(3,1) = 0.0;
	c.at<double>(3,2) = 0.0;
	c.at<double>(3,3) = 1.0;
}

void decomposeTransform(const cv::Mat& c, cv::Mat& R, cv::Mat& t) {
	
	R = cv::Mat::zeros(3, 3, CV_64FC1);
	t = cv::Mat::zeros(3, 1, CV_64FC1);

	R.at<double>(0,0) = c.at<double>(0,0);
	R.at<double>(0,1) = c.at<double>(0,1);
	R.at<double>(0,2) = c.at<double>(0,2);

	R.at<double>(1,0) = c.at<double>(1,0);
	R.at<double>(1,1) = c.at<double>(1,1);
	R.at<double>(1,2) = c.at<double>(1,2);

	R.at<double>(2,0) = c.at<double>(2,0);
	R.at<double>(2,1) = c.at<double>(2,1);
	R.at<double>(2,2) = c.at<double>(2,2);

	t.at<double>(0,0) = c.at<double>(0,3);
	t.at<double>(1,0) = c.at<double>(1,3);
	t.at<double>(2,0) = c.at<double>(2,3);

}


// http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/ethan.htm
void matrixToQuaternion(const cv::Mat& mat, Eigen::Quaternion<double>& quat) {

	// convert mat to Eigen::Matrix3d m;
	/*
	Eigen::Matrix3d m;

	m(0,0) = mat.at<double>(0,0);
	m(0,1) = mat.at<double>(0,1);
	m(0,2) = mat.at<double>(0,2);

	m(1,0) = mat.at<double>(1,0);
	m(1,1) = mat.at<double>(1,1);
	m(1,2) = mat.at<double>(1,2);

	m(2,0) = mat.at<double>(2,0);
	m(2,1) = mat.at<double>(2,1);
	m(2,2) = mat.at<double>(2,2);

	quat = Quaterniond(m);

	quat.normalize();

	return;
	*/
	// printf("%s << Entered...\n", __FUNCTION__);

	double m00 = mat.at<double>(0,0);
	double m01 = mat.at<double>(0,1);
	double m02 = mat.at<double>(0,2);

	double m10 = mat.at<double>(1,0);
	double m11 = mat.at<double>(1,1);
	double m12 = mat.at<double>(1,2);

	double m20 = mat.at<double>(2,0);
	double m21 = mat.at<double>(2,1);
	double m22 = mat.at<double>(2,2);

	double qw, qx, qy, qz;


	// ALT 3
	// J.M.P. van Waveren
	/*
	float m[9], q[4];
	m[0] = m00; m[1] = m01; m[2] = m02;
	m[3] = m10; m[4] = m11; m[5] = m12;
	m[6] = m20; m[7] = m21; m[8] = m22;

	//const float *m = jointMats[i].mat;
	if ( m[0 * 4 + 0] + m[1 * 4 + 1] + m[2 * 4 + 2] > 0.0f ) {
	float t = + m[0 * 4 + 0] + m[1 * 4 + 1] + m[2 * 4 + 2] + 1.0f;
	float s = ReciprocalSqrt( t ) * 0.5f;
	q[3] = s * t;
	q[2] = ( m[0 * 4 + 1] - m[1 * 4 + 0] ) * s;
	q[1] = ( m[2 * 4 + 0] - m[0 * 4 + 2] ) * s;
	q[0] = ( m[1 * 4 + 2] - m[2 * 4 + 1] ) * s;
	} else if ( m[0 * 4 + 0] > m[1 * 4 + 1] && m[0 * 4 + 0] > m[2 * 4 + 2] ) {
	float t = + m[0 * 4 + 0] - m[1 * 4 + 1] - m[2 * 4 + 2] + 1.0f;
	float s = ReciprocalSqrt( t ) * 0.5f;
	q[0] = s * t;
	q[1] = ( m[0 * 4 + 1] + m[1 * 4 + 0] ) * s;
	q[2] = ( m[2 * 4 + 0] + m[0 * 4 + 2] ) * s;
	q[3] = ( m[1 * 4 + 2] - m[2 * 4 + 1] ) * s;
	} else if ( m[1 * 4 + 1] > m[2 * 4 + 2] ) {
	float t = - m[0 * 4 + 0] + m[1 * 4 + 1] - m[2 * 4 + 2] + 1.0f;
	float s = ReciprocalSqrt( t ) * 0.5f;
	q[1] = s * t;
	q[0] = ( m[0 * 4 + 1] + m[1 * 4 + 0] ) * s;
	q[3] = ( m[2 * 4 + 0] - m[0 * 4 + 2] ) * s;
	q[2] = ( m[1 * 4 + 2] + m[2 * 4 + 1] ) * s;
	} else {
	float t = - m[0 * 4 + 0] - m[1 * 4 + 1] + m[2 * 4 + 2] + 1.0f;
	float s = ReciprocalSqrt( t ) * 0.5f;
	q[2] = s * t;
	q[3] = ( m[0 * 4 + 1] - m[1 * 4 + 0] ) * s;
	q[0] = ( m[2 * 4 + 0] + m[0 * 4 + 2] ) * s;
	q[1] = ( m[1 * 4 + 2] + m[2 * 4 + 1] ) * s;
	}

	qx = (double) q[0];
	qy = (double) q[1];
	qz = (double) q[2];
	qw = (double) q[3];
	*/

	// ALT 2
	// http://www.koders.com/cpp/fid38F83F165ABF728D6046FD59CF21ECF65E30E2D4.aspx
	/*
	double tmp = abs(m00 + m11 + m22 + 1);
	double s = sqrt(tmp);

	if (s > 0.0001) {
	qx = (m21 - m12) / 4*s;
	qy = (m02 - m20) / 4*s;
	qx = (m10 - m01) / 4*s;
	} else {
	int s_iNext[3] = { 2, 3, 1 };
	int i = 1;
	if ( m11 > m00 ) {
	i = 2;
	}

	if ( m22 > m11 ) {
	i = 3;
	}

	int j = s_iNext[i-1];
	int k = s_iNext[j-1];

	double fRoot = sqrt(mat.at<double>(i-1,i-1) - mat.at<double>(j-1,j-1) - mat.at<double>(k-1,k-1) + 1.0);

	double *tmp[3] = { &qx, &qy, &qz };
	*tmp[i-1] = 0.5 * fRoot;
	s = (mat.at<double>(k-1,j-1)-mat.at<double>(j-1,k-1))*fRoot;
	*tmp[j-1] = (mat.at<double>(j-1,i-1)+mat.at<double>(i-1,j-1))*fRoot;
	*tmp[k-1] = (mat.at<double>(k-1,i-1)+mat.at<double>(i-1,k-1))*fRoot;
	}

	if ((qx*qx + qy*qy + qz*qz) != 1.0) {
	qw = sqrt(1 - qx*qx - qy*qy - qz*qz);
	} else {
	qw = 0.0;
	}
	*/

	// ALT 1

	double tr1 = 1.0 + m00 - m11 - m22;
	double tr2 = 1.0 - m00 + m11 - m22;
	double tr3 = 1.0 - m00 - m11 + m22;



	//printf("%s << tr1, tr2, tr3 = (%f, %f, %f)\n", __FUNCTION__, tr1, tr2, tr3);

	if ((tr1 > tr2) && (tr1 > tr3)) {
	double S = sqrt(tr1) * 2.0; // S=4*qx
	//printf("%s << Case 1: %f\n", __FUNCTION__, S);
	qw = (m21 - m12) / S;
	qx = 0.25 * S;
	qy = (m01 + m10) / S;
	qz = (m02 + m20) / S;
	} else if ((tr2 > tr1) && (tr2 > tr3)) {
	double S = sqrt(tr2) * 2.0; // S=4*qy
	//printf("%s << Case 2: %f\n", __FUNCTION__, S);
	qw = (m02 - m20) / S;
	qx = (m01 + m10) / S;
	qy = 0.25 * S;
	qz = (m12 + m21) / S;
	} else if ((tr3 > tr1) && (tr3 > tr2)) {
	double S = sqrt(tr3) * 2.0; // S=4*qz
	//printf("%s << Case 3: %f\n", __FUNCTION__, S);
	qw = (m10 - m01) / S;
	qx = (m02 + m20) / S;
	qy = (m12 + m21) / S;
	qz = 0.25 * S;
	} else {
	//printf("%s << Case 4\n", __FUNCTION__);
	qw = 1.0;
	qx = 0.0;
	qy = 0.0;
	qz = 0.0;
	}

	



	quat.x() = qx;
	quat.y() = qy;
	quat.z() = qz;
	quat.w() = qw;

	//printf("%s << Quat = (%f, %f, %f, %f)\n", __FUNCTION__, qx, qy, qz, qw);

	quat.normalize();

	//printf("%s << Completed.\n", __FUNCTION__);
}

void quaternionToMatrix(const Eigen::Quaternion<double>& quat, cv::Mat& mat, bool handedness) {
	
	//QuaternionDbl eQuat;
	
	// eQuat.w()

	Eigen::Matrix3d eMat = quat.matrix();
	
	mat = cv::Mat::zeros(3, 3, CV_64FC1);
	
	if (handedness) {
		mat.at<double>(0,0) = eMat(0,0);
		mat.at<double>(0,1) = eMat(0,2);
		mat.at<double>(0,2) = eMat(0,1);
		
		mat.at<double>(1,0) = eMat(2,0);
		mat.at<double>(1,1) = eMat(2,2);
		mat.at<double>(1,2) = eMat(2,1);
		
		mat.at<double>(2,0) = eMat(1,0);
		mat.at<double>(2,1) = eMat(1,2);
		mat.at<double>(2,2) = eMat(1,1);
	} else {
		// Original
		mat.at<double>(0,0) = eMat(0,0);
		mat.at<double>(0,1) = eMat(0,1);
		mat.at<double>(0,2) = eMat(0,2);
		
		mat.at<double>(1,0) = eMat(1,0);
		mat.at<double>(1,1) = eMat(1,1);
		mat.at<double>(1,2) = eMat(1,2);
		
		mat.at<double>(2,0) = eMat(2,0);
		mat.at<double>(2,1) = eMat(2,1);
		mat.at<double>(2,2) = eMat(2,2);
	}
	
	
	// Quaternion.toRotationMatrix()

}

double normalizedGRICdifference(std::vector<cv::Point2f>& pts1, std::vector<cv::Point2f>& pts2, cv::Mat& F, cv::Mat& H, cv::Mat& mask_F, cv::Mat& mask_H, double& F_GRIC, double& H_GRIC) {
	
	int d, k;
	double lambda_3 = 2.00;
	double r = 4.00;	// assuming always with 2-frames
	int distMethod;
	
	// determine d & k
	d = 2;
	k = 8;
	distMethod = LOURAKIS_DISTANCE;
	H_GRIC = calculateGRIC(pts1, pts2, H, mask_H, d, k, r, lambda_3, distMethod);
	
	d = 3;
	k = 7;
	distMethod = SAMPSON_DISTANCE;
	F_GRIC = calculateGRIC(pts1, pts2, F, mask_F, d, k, r, lambda_3, distMethod);
	
	double retVal = abs(F_GRIC - H_GRIC) / H_GRIC;
	
	return retVal;
	
	
}

double calculateGRIC(std::vector<cv::Point2f>& pts1, std::vector<cv::Point2f>& pts2, cv::Mat& rel, cv::Mat& mask, int d, double k, double r, double lambda_3, int distMethod) {
	
	double retVal = 0.00;

	int n = int(pts1.size()); // countNonZero(mask);
	
	double lambda_1 = log(r);
	double lambda_2 = log(r*((double)n));
	
	double *e_vals;
	e_vals = new double[n];
	
	// gotta calculate sig and e
	
	// they calculate 'e' using:
	//		for F: point to epipolar line cost
	//		for H: symmetric transfer error
	
	double e_mean = 0.00, sig = 0.00;
	
	for (unsigned int iii = 0; iii < pts1.size(); iii++) {
		
		e_vals[iii] = calcGeometryDistance(pts1.at(iii), pts2.at(iii), rel, distMethod);
		e_mean += e_vals[iii] / ((double) n);
		
	}
	
	// Calculate variance
	
	for (unsigned int iii = 0; iii < pts1.size(); iii++) {
		sig += pow((e_vals[iii] - e_mean), 2.0) / ((double) n);
	}
	
	for (unsigned int iii = 0; iii < pts1.size(); iii++) {
		retVal += calculateRho(e_vals[iii], sig, r, lambda_3, d);
	}
	
	retVal +=  lambda_1*((double) d)*n + lambda_2*((double) k);

	return retVal;
	
}

double calculateRho(double e, double sig, double r, double lambda_3, int d) {
	
	double val1 = pow(e, 2.0) / pow(sig, 2.0);
	double val2 = lambda_3 * (r - ((double) d));
	
	return std::min(val1, val2);
	
}

double calcGeometryDistance(cv::Point2f& pt1, cv::Point2f& pt2, cv::Mat& mat, int distMethod) {
	
	cv::Mat p1, p2;
	p1 = cv::Mat::ones(3,1,CV_64FC1);
	p2 = cv::Mat::ones(3,1,CV_64FC1);
	
	p1.at<double>(0,0) = (double) pt1.x;
	p1.at<double>(1,0) = (double) pt1.y;
	
	p2.at<double>(0,0) = (double) pt2.x;
	p2.at<double>(1,0) = (double) pt2.y;
	
	cv::Mat p2t;
	transpose(p2, p2t);
	
	double dist = 0.0;
	
	double ri;
	
	cv::Mat A;
	
	A = p2t * mat * p1; 
	ri = A.at<double>(0,0); 
	
	double r = abs(ri);	// not sure if this is correct and/or needed
			
	double rx, ry, rxd, ryd;
	
	rx = mat.at<double>(0,0) * pt2.x + mat.at<double>(1,0) * pt2.y + mat.at<double>(2,0);
	ry = mat.at<double>(0,1) * pt2.x + mat.at<double>(1,1) * pt2.y + mat.at<double>(2,1);
	rxd = mat.at<double>(0,0) * pt1.x + mat.at<double>(0,1) * pt1.y + mat.at<double>(0,2);
	ryd = mat.at<double>(1,0) * pt1.x + mat.at<double>(1,1) * pt1.y + mat.at<double>(1,2);
	
	double e1, e2;
	
	e1 = r / pow( pow(rx, 2.0) + pow(ry, 2.0) , 0.5);
	e2 = r / pow( pow(rxd, 2.0) + pow(ryd, 2.0) , 0.5);
	
	double de;
	
	de = pow(pow(e1, 2.0) + pow(e2, 2.0), 0.5);
	
	double ds;
	
	ds = r * pow((1 / (pow(rx, 2.0) + pow(ry, 2.0) + pow(rxd, 2.0) + pow(ryd, 2.0))), 0.5);
	
	switch (distMethod) {
		case SAMPSON_DISTANCE:
			dist = ds;
			break;
		case ALGEBRAIC_DISTANCE: 
			dist = r;
			break;
		case EPIPOLAR_DISTANCE:
			dist = de;
			break;
		case LOURAKIS_DISTANCE:
			dist = lourakisSampsonError(pt1, pt2, mat);
			break;
		default:
			break;
	}
	
	return dist;
}

// http://www.ics.forth.gr/~lourakis/homest/
double lourakisSampsonError(cv::Point2f& pt1, cv::Point2f& pt2, cv::Mat& H) {
	
	double error = 0.0;
	
	double t1, t10, t100, t104, t108, t112, t118, t12, t122, t125, t126, t129, t13, t139, t14, t141, t144, t15, t150, t153, t161, t167, t17, t174, t18, t19, t193, t199;
	double t2, t20, t201, t202, t21, t213, t219, t22, t220, t222, t225, t23, t236, t24, t243, t250, t253, t26, t260, t27, t271, t273, t28, t29, t296;
	double t3, t30, t303, t31, t317, t33, t331, t335, t339, t34, t342, t345, t35, t350, t354, t36, t361, t365, t37, t374, t39;
	double t4, t40, t41, t42, t43, t44, t45, t46, t47, t49, t51, t57;
	double t6, t65, t66, t68, t69;
	double t7, t72, t78;
	double t8, t86, t87, t90, t95;
	
	double h[9];
	
	for (int iii = 0; iii < 3; iii++) {
		for (int jjj = 0; jjj < 3; jjj++) {
			h[3*iii + jjj] = H.at<double>(iii,jjj);
		}
	}
	
	double m1[2], m2[2];
	
	m1[0] = (double) pt1.x;
	m1[1] = (double) pt1.y;
	m2[0] = (double) pt2.x;
	m2[1] = (double) pt2.y;
	
	t1 = m2[0];
	t2 = h[6];
	t3 = t2*t1;
	t4 = m1[0];
	t6 = h[7];
	t7 = t1*t6;
	t8 = m1[1];
	t10 = h[8];
	t12 = h[0];
	t13 = t12*t4;
	t14 = h[1];
	t15 = t14*t8;
	t17 = t3*t4+t7*t8+t1*t10-t13-t15-h[2];
	t18 = m2[1];
	t19 = t18*t18;
	t20 = t2*t2;
	t21 = t19*t20;
	t22 = t18*t2;
	t23 = h[3];
	t24 = t23*t22;
	t26 = t23*t23;
	t27 = t6*t6;
	t28 = t19*t27;
	t29 = t18*t6;
	t30 = h[4];
	t31 = t29*t30;
	t33 = t30*t30;
	t34 = t4*t4;
	t35 = t20*t34;
	t36 = t2*t4;
	t37 = t6*t8;
	t39 = 2.0*t36*t37;
	t40 = t36*t10;
	t41 = 2.0*t40;
	t42 = t8*t8;
	t43 = t42*t27;
	t44 = t37*t10;
	t45 = 2.0*t44;
	t46 = t10*t10;
	t47 = t21-2.0*t24+t26+t28-2.0*t31+t33+t35+t39+t41+t43+t45+t46;
	t49 = t12*t12;
	t51 = t6*t30;
	t57 = t20*t2;
	t65 = t1*t1;
	t66 = t65*t20;
	t68 = t65*t57;
	t69 = t4*t10;
	t72 = t2*t49;
	t78 = t27*t6;
	t86 = t65*t78;
	t87 = t8*t10;
	t90 = t65*t27;
	t95 = -2.0*t49*t18*t51-2.0*t3*t12*t46-2.0*t1*t57*t12*t34-2.0*t3*t12*t33+t66
	*t43+2.0*t68*t69+2.0*t72*t69-2.0*t7*t14*t46-2.0*t1*t78*t14*t42-2.0*t7*t14*t26+
	2.0*t86*t87+t90*t35+2.0*t49*t6*t87;
	t100 = t14*t14;
	t104 = t100*t2;
	t108 = t2*t23;
	t112 = t78*t42*t8;
	t118 = t57*t34*t4;
	t122 = t10*t26;
	t125 = t57*t4;
	t126 = t10*t19;
	t129 = t78*t8;
	t139 = -2.0*t57*t34*t18*t23+2.0*t100*t6*t87+2.0*t104*t69-2.0*t100*t18*t108+
	4.0*t36*t112+6.0*t43*t35+4.0*t118*t37+t35*t28+2.0*t36*t122+2.0*t125*t126+2.0*
	t129*t126+2.0*t37*t122-2.0*t78*t42*t18*t30+t43*t21;
	t141 = t10*t33;
	t144 = t46*t18;
	t150 = t46*t19;
	t153 = t46*t10;
	t161 = t27*t27;
	t167 = 2.0*t36*t141-2.0*t144*t108+2.0*t37*t141+t66*t33+t150*t27+t150*t20+
	4.0*t37*t153+6.0*t43*t46+4.0*t112*t10+t43*t33+t161*t42*t19+t43*t26+4.0*t36*t153
	;
	t174 = t20*t20;
	t193 = 6.0*t35*t46+4.0*t10*t118+t35*t33+t35*t26+t174*t34*t19+t100*t27*t42+
	t100*t20*t34+t100*t19*t20+t90*t46+t65*t161*t42+t90*t26+t49*t27*t42+t49*t20*t34+
	t49*t19*t27;
	t199 = t34*t34;
	t201 = t12*t23;
	t202 = t14*t30;
	t213 = t42*t42;
	t219 = t66*t46+t100*t26+t46*t100+t174*t199-2.0*t201*t202-2.0*t144*t51+t46*
	t26+t65*t174*t34+t49*t33+t49*t46+t46*t33+t161*t213-2.0*t7*t14*t20*t34;
	t220 = t1*t27;
	t222 = t36*t8;
	t225 = t7*t14;
	t236 = t4*t6*t8;
	t243 = t3*t12;
	t250 = t46*t46;
	t253 = t1*t20;
	t260 = -4.0*t220*t14*t222-4.0*t225*t40-4.0*t220*t15*t10+2.0*t90*t40+2.0*
	t225*t24+2.0*t72*t236-2.0*t3*t12*t27*t42-4.0*t243*t44+2.0*t66*t44+2.0*t243*t31+
	t250+2.0*t68*t236-4.0*t253*t12*t236-4.0*t253*t13*t10;
	t271 = t4*t20;
	t273 = t8*t18;
	t296 = t10*t18;
	t303 = 2.0*t104*t236-2.0*t35*t31+12.0*t35*t44+2.0*t125*t37*t19-4.0*t271*t6*
	t273*t23+2.0*t36*t37*t26+2.0*t36*t129*t19-4.0*t36*t27*t273*t30+2.0*t36*t37*t33+
	12.0*t36*t43*t10+12.0*t36*t37*t46-4.0*t271*t296*t23+2.0*t36*t126*t27;
	t317 = t18*t14;
	t331 = t14*t2;
	t335 = t12*t18;
	t339 = t220*t18;
	t342 = t7*t30;
	t345 = t317*t6;
	t350 = -4.0*t31*t40-2.0*t43*t24+2.0*t37*t126*t20-4.0*t44*t24-4.0*t27*t8*
	t296*t30-2.0*t253*t317*t30-2.0*t65*t2*t23*t6*t30+2.0*t3*t23*t14*t30-2.0*t12*t19
	*t331*t6+2.0*t335*t331*t30-2.0*t201*t339+2.0*t201*t342+2.0*t201*t345+2.0*t86*
	t222;
	t354 = 1/(t95+t139+t167+t193+t219+t260+t303+t350);
	t361 = t22*t4+t29*t8+t296-t23*t4-t30*t8-h[5];
	t365 = t253*t18-t3*t23-t335*t2+t201+t339-t342-t345+t202;
	t374 = t66-2.0*t243+t49+t90-2.0*t225+t100+t35+t39+t41+t43+t45+t46;

	error = sqrt((t17*t47*t354-t361*t365*t354)*t17+(-t17*t365*t354+t361*t374*
	t354)*t361);

	return error;
}	
	
#endif