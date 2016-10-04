#ifndef _ROS_SUBSTITUTION_H_
#define _ROS_SUBSTITUTION_H_

#include <string>
#include <vector>
#include <stdarg.h>
#include <math.h>
#include <stdio.h>

#ifdef _USE_BOOST_
#include "boost/date_time/posix_time/posix_time.hpp"	
#endif

#ifdef _IS_WINDOWS_
#define __SHORTENED_FILE__ (strrchr(__FILE__, '\\') ? strrchr(__FILE__, '\\') + 1 : __FILE__)
#define ROS_INFO(fmt, ...)  std::printf("%s:%s << " fmt "\n",          __SHORTENED_FILE__, __FUNCTION__, __VA_ARGS__);
#define ROS_WARN(fmt, ...)  std::printf("%s:%s << WARNING! " fmt "\n", __SHORTENED_FILE__, __FUNCTION__, __VA_ARGS__);
#define ROS_ERROR(fmt, ...) std::printf("%s:%s << ERROR! " fmt "\n",   __SHORTENED_FILE__, __FUNCTION__, __VA_ARGS__);
#else
#define __SHORTENED_FILE__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)
void ROS_INFO   (char *fmt, ...);
void ROS_WARN   (char *fmt, ...);
void ROS_ERROR  (char *fmt, ...);
#endif

namespace ros {

/**
 * A structure to substitute for the ros::Time structure when ROS is not being used
 */
struct Time {
	unsigned long int sec;
	unsigned long int nsec;

	double toSec() const;
	double toNSec();
	Time();
	Time(double input);
	static Time now();
};

}

namespace std_msgs {

/**
 * A structure to substitute for the std_msgs::Header structure when ROS is not being used
 */
struct Header {
	unsigned long int seq;
	std::string frame_id;
	ros::Time stamp;
	Header();
};

}

namespace sensor_msgs {

/**
 * A structure to substitute for the ROS sensor_msgs::CameraInfo structure when ROS is not being used
 */
struct CameraInfo {
	std_msgs::Header header;
	double K[9], R[9], P[12];
	std::vector<double> D;
	int width, height;
	int binning_x, binning_y;
	std::string distortion_model;

	CameraInfo();
};

}

namespace geometry_msgs {

/**
 * A structure to substitute for the ROS geometry_msgs::Point structure when ROS is not being used
 */
struct Point {
	float x,y,z;
	Point();
};

/**
 * A structure to substitute for the ROS geometry_msgs::Point structure when ROS is not being used
 */
struct Quaternion {
	float x,y,z,w;
	Quaternion();
};

/**
 * A structure to substitute for the ROS geometry_msgs::Pose structure when ROS is not being used
 */
struct Pose {
	Point position;
	Quaternion orientation;
	Pose();
};

/**
 * A structure to substitute for the ROS geometry_msgs::PoseStamped structure when ROS is not being used
 */
struct PoseStamped {
	std_msgs::Header header;
	Pose pose;
	PoseStamped();
};

}

#endif // _ROS_SUBSTITUTION_H_
