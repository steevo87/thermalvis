#include "core/ros_substitution.hpp"

#if _IS_LINUX_
void ROS_INFO(char *fmt, ...) 
{
  va_list args;
  va_start(args, fmt);
  vprintf(fmt, args);
  printf("\n");
  va_end(args);
}

void ROS_WARN(char *fmt, ...) 
{
  va_list args;
  va_start(args, fmt);
  vprintf(fmt, args);
  printf("\n");
  va_end(args);
}

void ROS_ERROR(char *fmt, ...) 
{
  va_list args;
  va_start(args, fmt);
  vprintf(fmt, args);
  printf("\n");
  va_end(args);
}
#endif

namespace ros {

	Time::Time() : sec(0), nsec(0) { }

	Time::Time(double input) {
		sec = (long int)(floor(input));
		nsec = (long int)((input - (double(sec)))*1000000000.0);
	}

	double Time::toSec() const { return (double(sec) + (double(nsec)/1000000000.0)); }

	double Time::toNSec() { return (double(sec)*1000000000.0 + double(nsec)); }

	Time Time::now() {
		Time retVal;

#ifdef _USE_BOOST_
		boost::posix_time::ptime currentTime = boost::posix_time::microsec_clock::local_time();
		retVal.sec = (long int)(currentTime.time_of_day().total_seconds());
		retVal.nsec = (long int)(currentTime.time_of_day().total_nanoseconds()) - retVal.sec * 1000000000;
#else
		// TODO: Implement alternative way to get current system time
#endif

		return retVal;
	}

}

namespace std_msgs {

	Header::Header() : seq(0), frame_id("") { }

}

namespace sensor_msgs {

	CameraInfo::CameraInfo() : 
		width(384), 
		height(288), 
		distortion_model("plumb bob"), 
		binning_x(0),
		binning_y(0)
	{

		K[0] = 500.0;
		K[1] = 0.0;
		K[2] = 191.5;

		K[3] = 0.0;
		K[4] = 500.0;
		K[5] = 143.5;

		K[6] = 0.0;
		K[7] = 0.0;
		K[8] = 1.0;
	
		for (int iii = 0; iii < 12; iii++) {
			P[iii] = 0.0;
			if (iii < 9) R[iii] = 0.0;
		}
	}

}

namespace geometry_msgs {

	Point::Point() : x(0.0), y(0.0), z(0.0) { }
	Quaternion::Quaternion() : x(0.0), y(0.0), z(0.0), w(1.0) { }
	Pose::Pose() { }
	PoseStamped::PoseStamped() { }

}

