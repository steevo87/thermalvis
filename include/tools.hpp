/*! \file	tools.hpp
 *  \brief	Declarations for generic tools not depending on libraries such as OpenCV, PCL and ROS.
*/

#ifndef _THERMALVIS_TOOLS_H_
#define _THERMALVIS_TOOLS_H_

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
	#define NOMINMAX 1	// Don't remove this, even though it causes a warning!
	#include <windows.h>
	#include <time.h>
	#if defined(_MSC_VER) || defined(_MSC_EXTENSIONS)
	  #define DELTA_EPOCH_IN_MICROSECS  11644473600000000Ui64
	#else
	  #define DELTA_EPOCH_IN_MICROSECS  11644473600000000ULL
	#endif
#else
	#include <unistd.h>
	#include <getopt.h>
	#include <dirent.h>
	#include <sys/time.h>
	#include <sys/mman.h>
	#include <sys/ioctl.h>
	#include <termios.h>
	#include <string.h>
	#include <cstdlib>
	#include <cmath>
	#include <stdarg.h>
#endif

#include <iostream>
#include <fstream>
#include <vector>
#include <string.h>

#ifdef _BUILD_FOR_ROS_
#include <ros/ros.h>
#endif

#ifdef _USE_BOOST_
#include "boost/date_time/posix_time/posix_time.hpp"	
#endif

#define MESSAGE_NORMAL	0
#define MESSAGE_WARNING 1
#define MESSAGE_ERROR	2

#define USE_CLAHE 0

#define DEFAULT_DATA_DIRECTORY "C:\\Users\\Public\\Documents\\Data\\optris\\test_seq_001"

using namespace std;

// http://social.msdn.microsoft.com/Forums/vstudio/en-US/430449b3-f6dd-4e18-84de-eebd26a8d668/gettimeofday?forum=vcgeneral
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__) 
struct timezone 
{
  int  tz_minuteswest; /* minutes W of Greenwich */
  int  tz_dsttime;     /* type of dst correction */
};
int gettimeofday(struct timeval *tv, struct timezone *tz);
#endif

typedef std::pair<unsigned int,unsigned int> mypair;
bool comparator ( const mypair& l, const mypair& r);

/// \brief      Counts and returns a list of all specified elements within a folder
int countElementsInFolder(const char* folderName, vector<string>& elementNames, int elementType);

void convertUcharToBinary(unsigned char val, int* binaryArray);



/// \brief      Calls the relevant system function to ensure number pseudo-randomness
void initializeRandomNums();

/// \brief      Calculate the mean and standard deviation of a vector of doubles
void calcParameters(const vector<double>& v, double& mean, double& stdev);

/// \brief      Calculates perpendicular distance between two "parallel" lines
double calcLinePerpDistance(double *line1, double *line2);

void findLinearModel(double* x, double* y, int termsToConsider, double &m, double &c);

/// \brief      Calculates time elapsed since the last time the timer was reset
double timeElapsedMS(struct timeval& timer, bool reset = true);



void addUniqueToVector(vector<unsigned int>& dst, vector<unsigned int>& src);

double asymmetricGaussianValue(double score, double mean, double loVar, double hiVar);

void randomSelection(vector<unsigned int>& src, vector<unsigned int>& dst, unsigned int max_val);

#ifndef _WIN32
#define S_OK 		0
#define S_FALSE 	-1
#endif

#ifndef _BUILD_FOR_ROS_

#ifdef _WIN32
#define __SHORTENED_FILE__ (strrchr(__FILE__, '\\') ? strrchr(__FILE__, '\\') + 1 : __FILE__)
#define ROS_INFO(fmt, ...) std::printf("%s:%s << " fmt "\n", __SHORTENED_FILE__, __FUNCTION__, __VA_ARGS__);
#define ROS_WARN(fmt, ...) std::printf("%s:%s << WARNING! " fmt "\n", __SHORTENED_FILE__, __FUNCTION__, __VA_ARGS__);
#define ROS_ERROR(fmt, ...) std::printf("%s:%s << ERROR! " fmt "\n", __SHORTENED_FILE__, __FUNCTION__, __VA_ARGS__);
#else
#define __SHORTENED_FILE__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

void ROS_INFO(char *fmt, ...);
void ROS_WARN(char *fmt, ...);
void ROS_ERROR(char *fmt, ...);

namespace ros {

struct Time {
	unsigned long int sec;
	unsigned long int nsec;

	double toSec();
	double toNSec();
	Time();
	Time(double input);
	static Time now();
};

struct Header {
	unsigned long int seq;
	std::string frame_id;
	Time stamp;
	Header();
};

}

namespace sensor_msgs {

struct CameraInfo {
	ros::Header header;
	double K[9], R[9], P[12];
	std::vector<double> D;
	int width, height;
	int binning_x, binning_y;
	std::string distortion_model;

	CameraInfo();
};

}

#endif

#endif

/// \brief		Calculates Factorial of an integer
long long int factorial(int num);

/// \brief      Gets next possible combination for an exhaustive combinatorial search
void getNextCombo(vector<unsigned int>& currentIndices, int r, int n);

/// \brief      Selects the score which is minimally better than a specified proportion of all scores
double findEquivalentProbabilityScore(double* values, int quantity, double prob);



/// \brief      Converts a raw byte as an 8-bit character array of 1s and 0s
void convert_byte_to_binary_string(void* src, char* dst);



/// \brief		http://stackoverflow.com/questions/485525/round-for-float-in-c
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
	double round(double d);
#endif

#endif
