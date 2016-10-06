/*! \file	calib.hpp
 *  \brief	Declarations for the CALIBRATOR node.
*/

#ifndef THERMALVIS_CALIB_HPP
#define THERMALVIS_CALIB_HPP

#include "calibrator_config.hpp"

#include "core/tools.hpp"
#include "core/program.hpp"

#ifdef _BUILD_FOR_ROS_
#include "calibratorConfig.h"
#include "core/ros_resources.hpp"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <dynamic_reconfigure/server.h>
#endif

#include "core/general_resources.hpp"

#include <signal.h>

#include "core/launch.hpp"
#include "core/improc.hpp"
#include "core/features.hpp"
#include "calibrator/calibration.hpp"
#include "calibrator/intrinsics.hpp"
#include "calibrator/extrinsics.hpp"

#ifdef _USE_BOOST_
#include <boost/filesystem.hpp>
#include <boost/thread/thread.hpp>
#ifndef _BUILD_FOR_ROS_
#include <boost/algorithm/string/replace.hpp>
#endif
#endif

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "opencv2/calib3d/calib3d.hpp"

#define DEFAULT_TIMER_PERIOD	0.05

//HGH
#define PATTERN_CODE_INVALID -1
#define PATTERN_CODE_FIND_TRACK 0
#define PATTERN_CODE_TRACK_FIND 1
#define PATTERN_CODE_FIND 2

const char __PROGRAM__[] = "thermalvis-calibrator";

/// \brief		Stores configuration information for the streamer/image-processing routine
class calibratorData : public calibratorSharedData, public commonData, public calibratorLaunchOnlyData {
	friend class xmlParameters;
	friend class calibratorNode;
#ifndef _BUILD_FOR_ROS_
	friend class calibratorConfig;
#endif

protected:
	// ...

public:
	// Camera models
	cv::Mat K[MAX_ALLOWABLE_CAMS];
	cv::Mat distCoeffs[MAX_ALLOWABLE_CAMS];
	
	string read_addr;
	
	calibratorData();

	bool assignFromXml(xmlParameters& xP);

#ifdef _BUILD_FOR_ROS_
	bool obtainStartingData(ros::NodeHandle& nh);   
#endif
	bool processStartingData();
};

#ifndef _BUILD_FOR_ROS_
/// \brief		Substitute for ROS live configuration adjustments
class calibratorConfig : public calibratorRealtimeData {
public:
	calibratorConfig() { }
	bool assignStartingData(calibratorData& startupData);

};
#endif

/// \brief		Critical frame data regarding whether it contains a pattern valid for use in calibration
struct validPattern {
    int  validFrameID;
    int  patternID;
    bool patternFound;
    bool isValidFrame;
};


// example: http://ua-ros-pkg.googlecode.com/svn/trunk/arrg/ua_vision/object_tracking/src/object_tracker_node.cpp
/// \brief		Manages the calibration procedures
class calibratorNode : public GenericOptions {
private:

#ifdef _BUILD_FOR_ROS_
	ros::Timer timer;
	image_transport::CameraPublisher debug_pub[MAX_ALLOWABLE_CAMS];
	image_transport::CameraSubscriber camera_sub[MAX_ALLOWABLE_CAMS];
	cv_bridge::CvImagePtr cv_ptr[MAX_ALLOWABLE_CAMS];
	sensor_msgs::Image msg_debug[MAX_ALLOWABLE_CAMS];
	ros::NodeHandle private_node_handle;
	dynamic_reconfigure::Server<thermalvis::calibratorConfig> server;
	dynamic_reconfigure::Server<thermalvis::calibratorConfig>::CallbackType f;
#else
	cv::Mat cv_ptr[MAX_ALLOWABLE_CAMS];
#endif

	cv::Mat prevMat[MAX_ALLOWABLE_CAMS], grayMat[MAX_ALLOWABLE_CAMS];

	struct timeval cycle_timer;
	double elapsedTime;
	double avgTime;
	
	struct timeval tracking_timer;
	double elapsedTrackingTime;
	double frameTrackingTime;
	
	unsigned int validSets;
	
	struct timeval vacant_timer, elapsed_timer;
	double vacantInputTime, elapsedInputTime;
	
	Mat map1[MAX_ALLOWABLE_CAMS], map2[MAX_ALLOWABLE_CAMS];
	int topValidHeight, botValidHeight, leftValid[MAX_ALLOWABLE_CAMS], rightValid[MAX_ALLOWABLE_CAMS];
	
	vector<unsigned int> patternIndices[MAX_ALLOWABLE_CAMS];
	vector<Mat> displayImages[MAX_ALLOWABLE_CAMS];
	
	vector<ros::Time> times[MAX_ALLOWABLE_CAMS];
	
	vector<unsigned int> validPairs[MAX_ALLOWABLE_CAMS];
	
	bool infoProcessed[MAX_ALLOWABLE_CAMS];
	bool stillCollecting;

	double alpha;
	
	Mat default_R;
	
	Mat cameraMatrices[MAX_ALLOWABLE_CAMS];
	Mat distCoeffVecs[MAX_ALLOWABLE_CAMS];
	
	Rect roi[MAX_ALLOWABLE_CAMS];
	vector<Point2f> rectangleBounds, newRecBounds;
	Mat rectCamMat[MAX_ALLOWABLE_CAMS];
	vector<Point2f> leftLinePoints, rightLinePoints;
	
	Size imSize[MAX_ALLOWABLE_CAMS];
	Mat newCamMat[MAX_ALLOWABLE_CAMS];
	
	double reprojectionError_intrinsics[MAX_ALLOWABLE_CAMS];
	double extendedReprojectionError_intrinsics[MAX_ALLOWABLE_CAMS];
	
	vector<unsigned char> duplicateFlags[MAX_ALLOWABLE_CAMS];
	
	vector<ros::Time> patternTimestamps[MAX_ALLOWABLE_CAMS];
	
	vector<int> subselectedTags_intrinsics[MAX_ALLOWABLE_CAMS];
	
	string topic[MAX_ALLOWABLE_CAMS];
	
	unsigned int publishCount;
	
	// Extrinsics:
	Mat R_[MAX_ALLOWABLE_CAMS], P_[MAX_ALLOWABLE_CAMS];
	Mat E[MAX_ALLOWABLE_CAMS], F[MAX_ALLOWABLE_CAMS], Q[MAX_ALLOWABLE_CAMS];      // Between first camera and all other cameras
	Mat R[MAX_ALLOWABLE_CAMS], Rv[MAX_ALLOWABLE_CAMS], T[MAX_ALLOWABLE_CAMS];  // Rotations/translations between first camera and all other cameras
	Mat R2[MAX_ALLOWABLE_CAMS], T2[MAX_ALLOWABLE_CAMS];       // Rotations/translations between all other cameras
	
	double stereoError;
	double extendedExtrinsicReprojectionError;
	
	bool alphaChanged;

	calibratorData configData;
	
	sensor_msgs::CameraInfo debug_camera_info[MAX_ALLOWABLE_CAMS];

	char debug_pub_name[MAX_ALLOWABLE_CAMS][256];
	
	char nodeName[256];
	
	Mat newImage, lastImage;
	
	unsigned int checkIndex[MAX_ALLOWABLE_CAMS];
	unsigned int frameCount[MAX_ALLOWABLE_CAMS], undistortionCount, rectificationCount, totalFrameCount[2];
	
	std::vector<cv::Point3f> row;
    
	vector<vector<Point2f> > pointSets[MAX_ALLOWABLE_CAMS], candidateSets[MAX_ALLOWABLE_CAMS], testingSets[MAX_ALLOWABLE_CAMS];
	vector<vector<Point2f> > extrinsicsPointSets[MAX_ALLOWABLE_CAMS], extrinsicCandidateSets[MAX_ALLOWABLE_CAMS], extrinsicTestingSets[MAX_ALLOWABLE_CAMS];

    //HGH
    Mat lastMat[MAX_ALLOWABLE_CAMS];
    vector<Point2f> cornerSets[MAX_ALLOWABLE_CAMS];
    vector<unsigned int> frameCounts[MAX_ALLOWABLE_CAMS];
    bool  doVerify;

#ifdef _USE_BOOST_
	boost::shared_mutex _access;
#endif

	bool readyForOutput;
	
	vector<Point2f> cornerSet[MAX_ALLOWABLE_CAMS];

    //HGH
    void preprocessImage(Mat src, Mat &dst, double a, double b, bool normaliz, bool negative );

public:
	///brief	Constructor. Must receive configuration data.
#ifdef _BUILD_FOR_ROS_
	calibratorNode(ros::NodeHandle& nh, calibratorData startupData);
#else
	calibratorNode(calibratorData startupData);
#endif
	
	void performIntrinsicCalibration();
	void performExtrinsicCalibration();
	
	///brief	Initial receipt of an image WITHOUT camera info. 
#ifdef _BUILD_FOR_ROS_
	void handle_camera(const sensor_msgs::ImageConstPtr& msg_ptr, const sensor_msgs::CameraInfoConstPtr& info_msg, const unsigned int camera_index = 0);
#else
	void handle_camera(const cv::Mat& inputImage, const sensor_msgs::CameraInfo *info_msg, const unsigned int camera_index = 0);
#endif
	
	
	
	void create_virtual_pairs();
	
	void assignDebugCameraInfo();
	
//#ifdef _BUILD_FOR_ROS_
//	void publishUndistorted(const ros::TimerEvent& event);
//#else
	void publishUndistorted();
//#endif

//#ifdef _BUILD_FOR_ROS_
//	void publishRectified(const ros::TimerEvent& event);
//#else
	void publishRectified();
//#endif

	void startUndistortionPublishing();
	
	
	void startRectificationPublishing();
	
	bool isStillCollecting();
	bool wantsToUndistort();
	bool wantsToRectify();
	void writeResults();
	bool wantsIntrinsics();
	bool wantsExtrinsics();
        void updatePairs();

        //HGH
        void determineValidPairs();
        void evaluateFrames();

	void getAverageTime();
	void getFrameTrackingTime();
	void assignIntrinsics();
	
	void preparePatternSubsets();
	void prepareExtrinsicPatternSubsets();
	
	///brief	Processes online changes in node configuration and applies them.
#ifdef _BUILD_FOR_ROS_
	void serverCallback(thermalvis::calibratorConfig &config, uint32_t level);
#else
	void serverCallback(calibratorConfig &config);
#endif
	
	void updateMap();
	
	bool findPattern(const Mat& im, vector<Point2f>& dst, Mat& prev, const int cameraNumber);
	
	void updateIntrinsicMap(unsigned int idx); 
	void set_ready_for_output();
	
	void prepareForTermination();

        //HGH
        bool isVerifying();

	///brief	Periodically checks for some things... 
#ifdef _BUILD_FOR_ROS_
    void timerCallback(const ros::TimerEvent& e);
#else
	void loopCallback();
#endif
	

};

//boost::shared_ptr < calibratorNode > *globalNodePtr;

#endif
