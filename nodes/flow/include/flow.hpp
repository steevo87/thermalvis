/*! \file	flow.hpp
 *  \brief	Declarations for the FLOW node.
*/

#ifndef _THERMALVIS_FLOW_H_
#define _THERMALVIS_FLOW_H_

#include "general_resources.hpp"
#include "ros_resources.hpp"
#include "opencv_resources.hpp"
	
#include "improc.hpp"
#include "video.hpp"
#include "features.hpp"
#include "tracks.hpp"
#include "initialization.hpp"

#include "boost/filesystem.hpp"  

#ifdef _BUILD_FOR_ROS_
#include "feature_tracks.h"
#include "flowConfig.h"
#else
#include "boost/date_time/posix_time/posix_time.hpp"
#endif

const char __PROGRAM__[] = "THERMALVIS_FLOW";

#ifdef _BUILD_FOR_ROS_
typedef dynamic_reconfigure::Server < thermalvis::flowConfig > Server;
#endif

#define DEFAULT_ALPHA 	0.00
#define MS_PER_SEC 		1000.0

#define MINIMUM_GFTT_SENSITIVITY	0.001
#define MINIMUM_HARRIS_SENSITIVITY	0.0001
#define FAST_DETECTOR_SENSITIVITY_SCALING	200.0

#define TIGHT_DISTANCE_CONSTRAINT 3
#define FEATURE_DROP_TRIGGER 0.40

#define MINIMUM_PROP_OF_PREVIOUS_FEATURES 0.8

#define SKIP_WARNING_TIME_PERIOD 10.0

// DEBUG:
// Red: 	Successfully tracked points
// Green:	Recovered points
// Purple:	Detected, unmatched points
// Yellow:	Detected and matched points
// Blue:	

#define COLOR_TRACKED_POINTS 		CV_RGB(255, 0, 0)		// Red
#define COLOR_TRACKED_PATH	 		CV_RGB(255, 128, 127)	// Light Red
#define COLOR_RECOVERED_POINTS 		CV_RGB(0, 255, 0)		// Green
#define COLOR_UNMATCHED_POINTS 		CV_RGB(255, 0, 255)		// Purple	
#define COLOR_MATCHED_POINTS 		CV_RGB(255, 255, 0)		// Yellow

#define ADAPTIVE_WINDOW_CONTIGENCY_FACTOR 5.0

#define MIN_FEATURES_FOR_FEATURE_MOTION	10
#define MAX_HISTORY_FRAMES	16

#ifdef _BUILD_FOR_ROS_
bool wantsToShutdown = false;
void mySigintHandler(int sig);
void imageCallback(const sensor_msgs::ImageConstPtr& msg);
void parameterCallback(const sensor_msgs::CameraInfo& msg);
#endif

/// \brief		Stores configuration information for the sparse optical flow routine
struct trackerData {
	
	string topic, topicParent; // video_stream, video_sequence;
	string read_addr;
	
	string tracksOutputTopic;
	
	bool detectEveryFrame;
	
	bool verboseMode;
	bool adaptiveWindow;
	
	double maxVelocity;
	bool attemptMatching;
	
	bool attemptHistoricalRecovery;
	
	
	double newFeaturesPeriod;
	int multiplier[MAX_HISTORY_FRAMES];
	
	bool autoTrackManagement;
	
	string outputFolder;
	
	bool velocityPrediction;
	
	cameraParameters cameraData;
	
	
	//double alpha;
	double delayTimeout;
	double flowThreshold;
	double maxFrac;
	
	int matchingMode;
	
	int maxFeatures, minFeatures;
	//int maxProjectionsPerFeature;
	
	bool debugMode, showTrackHistory;
	bool outputTrackCount, outputFeatureMotion;
	bool normalizeFeatureVelocities;
	
	string detector[MAX_DETECTORS], descriptor[MAX_DETECTORS];
	double sensitivity[MAX_DETECTORS];
	string method[MAX_DETECTORS];
	bool method_match[MAX_DETECTORS];
	
	//int newDetectionFramecountTrigger;

	
	//string normalizationMode;
	//int minLimit, maxLimit;
	
	
	
	//int minPointsForWarning;
	
	int drawingHistory;
	
	//double historyPeriods[MAX_HISTORY_FRAMES];
	
	/*
	int nearSearchHistory;
	int farSearchHistory;
	int lossGaps[2];
	*/
	
	double minSeparation;
		
	unsigned int numDetectors;

	trackerData();

	#ifdef _BUILD_FOR_ROS_
	bool obtainStartingData(ros::NodeHandle& nh);   
	#endif

    void initializeDetectors(cv::Ptr<cv::FeatureDetector> *det, cv::Ptr<cv::FeatureDetector> *hom);
    void initializeDescriptors(cv::Ptr<cv::DescriptorExtractor> *desc, cv::Ptr<cv::DescriptorExtractor> *hom);
	
};

// example: http://ua-ros-pkg.googlecode.com/svn/trunk/arrg/ua_vision/object_tracking/src/object_tracker_node.cpp
/// \brief		Manages the optical flow procedure
class featureTrackerNode {
private:
	
	#ifdef _BUILD_FOR_ROS_
	image_transport::ImageTransport *it;
	
	ros::Timer timer, features_timer;
	sensor_msgs::CameraInfo debug_camera_info;
	ros::Publisher tracks_pub;
	image_transport::CameraPublisher debug_pub;
	image_transport::CameraSubscriber camera_sub;
	ros::Subscriber info_sub;
	image_transport::Subscriber image_sub;
	cv_bridge::CvImagePtr cv_ptr;
	image_transport::CameraPublisher pub_debug;
	sensor_msgs::Image msg_debug;
	ros::NodeHandle private_node_handle;

	ros::Time lastCycleFrameTime;
	ros::Time olderTimes[MAX_HISTORY_FRAMES];
	ros::Time info_time, image_time, previous_time, original_time, dodgeTime;
    
	dynamic_reconfigure::Server<thermalvis::flowConfig> server;
	dynamic_reconfigure::Server<thermalvis::flowConfig>::CallbackType f;
	#else
	cv::Mat bridgeReplacement;

	boost::posix_time::ptime lastCycleFrameTime;
	boost::posix_time::ptime olderTimes[MAX_HISTORY_FRAMES];
	boost::posix_time::ptime info_time, image_time, previous_time, original_time, dodgeTime;

	#endif

	int previousIndex, currentIndex;

	unsigned int cycleCount;
	bool cycleFlag;
	
	
	unsigned int skippedFrameCount, capturedFrameCount;
	
	long int lastAllocatedTrackIndex;
	
	
	ofstream trackCountStream, featureMotionStream;
	
	bool debugInitialized;
	
	int newlyDetectedFeatures;
	
	string optical_frame;
	
	bool lowPointsWarning;
	
	cv::Size ksize;
	double blurSigma;
	
	//unsigned int historicalIndices[];
	
	std::vector<unsigned int> activeTrackIndices;

	int successfullyTrackedFeatures;
	int discardedNewFeatures;
	
	vector<unsigned int> lostTrackIndices;
	int referenceFrame;
	
	bool undergoingDelay;
	bool handleDelays;
	
	cv::Mat dispMat;
	
	bool infoProcessed, infoSent;
	
	cv::Mat H12;
	
	double factor, minResponse; // minVal, maxVal, 
	
	

	trackerData configData;
	
	

	unsigned int previousTrackedPointsPeak;
	
	
	
	cv::Mat normalizedMat, blownImage, displayImage2, subscribedImage, drawImage2;
	cv::Mat newImage, grayImage, realImage, mappedImage, lastImage, olderImage;

	//gpu::GpuMat gmap1, gmap2;
	
	// http://docs.opencv.org/doc/tutorials/gpu/gpu-basics-similarity/gpu-basics-similarity.html#how-to-do-it-the-gpu
	//Mat I1;         // Main memory item - read image into with imread for example
	//gpu::GpuMat gI; // GPU matrix - for now empty
	//gI1.upload(I1); // Upload a data from the system memory to the GPU memory
	//I1 = gI1;       // Download, gI1.download(I1) will work too

	//Mat map1, map2;

	struct timeval cycle_timer, test_timer, skip_timer;
	double elapsedTime, testTime, skipTime;

	unsigned int frameCount;
	unsigned int readyFrame;
	

	

	char debug_pub_name[256];
	char tracks_pub_name[256];
	
	
	char nodeName[256];
	
	cv::Mat displayImage, drawImage, drawImage_resized;
	
	unsigned int bufferIndices[2];
	cv::Mat grayImageBuffer[2];
	
	unsigned int numHistoryFrames;
	unsigned int olderIndices[MAX_HISTORY_FRAMES];
	
	cv::Mat olderImages[MAX_HISTORY_FRAMES];
	double featuresVelocity;
	
	//cv::Mat workingImOld[MAXIMUM_FRAMES_TO_STORE], workingImNew[MAXIMUM_FRAMES_TO_STORE];
	
	cv::Mat blurredImage;
	
	
	vector<featureTrack> displayTracks;
	
	cv::Ptr<cv::FeatureDetector> keypointDetector[MAX_DETECTORS];
    cv::Ptr<cv::FeatureDetector> homographyDetector;
    
    vector<cv::KeyPoint> homogPoints[2];
    cv::Ptr<cv::DescriptorExtractor> descriptorExtractor;
    cv::Ptr<cv::DescriptorExtractor> homographyExtractor;
    
    cv::Mat homogDescriptors[2];
    
    
	
    int newlyRecoveredFeatures;
    int lostTrackCount;
    
    // Point vectors:
    vector<cv::Point2f> globalStartingPoints, globalFinishingPoints;
    
    vector<cv::Point2f> allRecoveredPoints, preservedRecoveredPoints, unmatchedPoints, matchedPoints;

    
    vector<cv::Point2f> newlySensedFeatures;
	vector<cv::Point2f> matchedFeatures;
    
    
    
    int distanceConstraint;
    
    int peakTracks;
    
	vector<featureTrack> featureTrackVector;
	
	bool freezeNextOutput;
	
	
	
	
	
	double averageTrackLength;
	
public:

#ifdef _BUILD_FOR_ROS_
	featureTrackerNode(ros::NodeHandle& nh, trackerData startupData);
	void timed_loop(const ros::TimerEvent& event);
	void features_loop(const ros::TimerEvent& event);
	void process_info(const sensor_msgs::CameraInfoConstPtr& info_msg);
	void handle_camera(const sensor_msgs::ImageConstPtr& msg_ptr, const sensor_msgs::CameraInfoConstPtr& info_msg);
	int publish_tracks(ros::Publisher *pub_message, unsigned int latestIndex);
	void serverCallback(thermalvis::flowConfig &config, uint32_t level);
#else
	featureTrackerNode(trackerData startupData);
	void handle_camera(const cv::Mat& inputImage);
	void process_info();
	void timed_loop();
	void features_loop();
#endif

	///brief	Calculates the average motion of all features successfully tracked from the previous frame.
	int calculateFeatureMotion(unsigned int idx, double& mx, double &my);
	
	void getDisplayImage(cv::Mat& dispIm) {
		drawImage_resized.copyTo(dispIm);
	}
	
	void handle_delay();
	
	void trimFeatureTrackVector();
	void trimDisplayTrackVectors();
	
	void handle_very_new();
	
	//int recoverTracks();
	void updateDistanceConstraint();
	void attemptTracking();
	void detectNewFeatures();
	void publishRoutine();
	void matchWithExistingTracks();
	
	void act_on_image();

	void updateTrackerData(trackerData newTrackerData);
	
	
	
	
	void prepareForTermination();
	
	
};

// dummy callbacks // http://answers.ros.org/question/55126/why-does-ros-overwrite-my-sequence-number/
#ifdef _BUILD_FOR_ROS_
void connected(const ros::SingleSubscriberPublisher&) {}
void disconnected(const ros::SingleSubscriberPublisher&) {}

boost::shared_ptr < featureTrackerNode > *globalNodePtr;
#endif

#endif
