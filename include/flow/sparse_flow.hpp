/*! \file	sparse_flow.hpp
 *  \brief	Declarations for sparse optical flow.
*/

#ifndef _THERMALVIS_SPARSE_FLOW_H_
#define _THERMALVIS_SPARSE_FLOW_H_

#define MINIMUM_MATCHING_RANK	10
#define DEFAULT_DESCRIPTOR_AREA	3.0

#include "core/tools.hpp"
#include "core/program.hpp"

#include "core/general_resources.hpp"
#include "core/ros_resources.hpp"
	
#include "core/launch.hpp"
#include "core/improc.hpp"
#include "core/features.hpp"
#include "core/tracks.hpp"
#include "core/initialization.hpp"

#ifdef _AVLIBS_AVAILABLE_
#include "streamer/video.hpp"
#endif

#ifdef _USE_BOOST_
#include <boost/filesystem.hpp>
#include <boost/filesystem/path.hpp>
#endif

#ifdef _BUILD_FOR_ROS_
#include "feature_tracks.h"
#include "flowConfig.h"
#endif

#include "flow_config.hpp"

#include "opencv2/core.hpp"

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <vector>
#include <iostream>

using namespace cv;
using namespace std;

/// \brief		Stores configuration information for the sparse optical flow routine
class trackerData : public flowSharedData, public commonData, public flowLaunchOnlyData {
	friend class xmlParameters;
	friend class featureTrackerNode;
#ifndef _BUILD_FOR_ROS_
	friend class flowConfig;
#endif

protected:
	
	string tracksOutputTopic;

	bool outputTrackCount, outputFeatureMotion, outputDebugImages;
	bool outputDetectedFeatures, outputTrackedFeatures;

	bool normalizeFeatureVelocities;
	
	int multiplier[MAX_HISTORY_FRAMES];
	int numDetectors;
	
	string detector[MAX_DETECTORS], descriptor[MAX_DETECTORS];
	double sensitivity[MAX_DETECTORS];
	string method[MAX_DETECTORS];
	bool method_match[MAX_DETECTORS];

public:
	trackerData();

	bool assignFromXml(xmlParameters& xP);

#ifdef _BUILD_FOR_ROS_
	bool obtainStartingData(ros::NodeHandle& nh);   
#endif

    bool initializeDetectors(cv::Ptr<cv::FeatureDetector> *det, cv::Ptr<cv::FeatureDetector> *hom);
    void initializeDescriptors(cv::Ptr<cv::DescriptorExtractor> *desc, cv::Ptr<cv::DescriptorExtractor> *hom);
	
};

#ifndef _BUILD_FOR_ROS_
/// \brief		Substitute for ROS live configuration adjustments
class flowConfig : public flowRealtimeData {
public:
	flowConfig() { }
	bool assignStartingData(trackerData& startupData);

	void setDetector1(int detector_1 = DETECTOR_FAST) {};
	void setDetector2(int detector_2 = DETECTOR_FAST) {};
	void setDetector3(int detector_3 = DETECTOR_FAST) {};

	int getDetector1() { return detector_1; }
	int getDetector2() { return detector_2; }
	int getDetector3() { return detector_3; }

};
#endif

/// \brief		Manages the optical flow procedure
class featureTrackerNode : public GenericOptions {
private:

#ifdef _USE_QT_
        QImage myImage;
        QLabel myLabel;
#endif
	
#ifdef _BUILD_FOR_ROS_

	ros::NodeHandle private_node_handle;
	
	dynamic_reconfigure::Server<thermalvis::flowConfig> server;
	dynamic_reconfigure::Server<thermalvis::flowConfig>::CallbackType f;

	ros::Publisher tracks_pub;
	ros::Subscriber info_sub;

	image_transport::ImageTransport *it;
	image_transport::CameraPublisher debug_pub, pub_debug;
	image_transport::CameraSubscriber camera_sub;
	image_transport::Subscriber image_sub;

	sensor_msgs::CameraInfo debug_camera_info;
	sensor_msgs::Image msg_debug;

	cv_bridge::CvImagePtr cv_ptr;

	ros::Timer timer, features_timer;
	
#else
	const cv::Mat *bridgeReplacement;
#endif

	ros::Time lastCycleFrameTime;
	ros::Time olderTimes[MAX_HISTORY_FRAMES];
	ros::Time info_time, image_time, previous_time, original_time, dodgeTime;

	trackerData configData;

	bool previousTimeInitialized;

	ofstream trackCountStream, featureMotionStream, detectedFeaturesStream, trackedFeaturesStream;

	string optical_frame;

	bool cycleFlag, undergoingDelay, handleDelays;
	unsigned int cycleCount, previousTrackedPointsPeak, frameCount, readyFrame;

	std::vector<unsigned int> activeTrackIndices, lostTrackIndices;
	int previousIndex, currentIndex, skippedFrameCount, capturedFrameCount, referenceFrame;
	long int lastAllocatedTrackIndex, lastAllocatedDisplayTrackIndex;
	
	bool debugInitialized, lowPointsWarning, infoProcessed, infoSent, freezeNextOutput;
	
	int newlyDetectedFeatures, successfullyTrackedFeatures, discardedNewFeatures, newlyRecoveredFeatures, lostTrackCount;
	
	cv::Size ksize;
	double blurSigma, factor, minResponse, featuresVelocity, averageTrackLength;

	
	
	cv::Mat H12, dispMat;	
	cv::Mat normalizedMat, blownImage, displayImage2, subscribedImage, drawImage2;
	cv::Mat blurredImage, newImage, grayImage, realImage, mappedImage, lastImage, olderImage;

	struct timeval cycle_timer, test_timer, skip_timer;
	double elapsedTime, testTime, skipTime;

	char *debug_pub_name, *tracks_pub_name, *nodeName;
		
	cv::Mat olderImages[MAX_HISTORY_FRAMES], grayImageBuffer[2], displayImage, drawImage, drawImage_resized;
	
	unsigned int numHistoryFrames, bufferIndices[2], olderIndices[MAX_HISTORY_FRAMES];
	
	std::vector<featureTrack> displayTracks;

	std::vector<std::string> predetectedFeatureFiles;
	
	cv::Ptr<cv::FeatureDetector> homographyDetector, keypointDetector[MAX_DETECTORS];
    vector<cv::KeyPoint> homogPoints[2], currPoints[MAX_DETECTORS];
	vector<cv::Point2f> candidates[MAX_DETECTORS];
    cv::Ptr<cv::DescriptorExtractor> homographyExtractor, descriptorExtractor;
    cv::Mat homogDescriptors[2];

    vector<cv::Point2f> globalStartingPoints, globalFinishingPoints, allRecoveredPoints, preservedRecoveredPoints, unmatchedPoints, matchedPoints, newlySensedFeatures, matchedFeatures;

    int distanceConstraint, peakTracks;
	
public:

	std::vector<featureTrack> featureTrackVector;

	///brief	Processes online changes in node configuration and applies them.
#ifdef _BUILD_FOR_ROS_
	void serverCallback(thermalvis::flowConfig &config, uint32_t level);
#else
	void serverCallback(flowConfig &config);
#endif

	///brief	Constructor. Must receive configuration data.
#ifdef _BUILD_FOR_ROS_
	featureTrackerNode(ros::NodeHandle& nh, trackerData startupData);
#else
	featureTrackerNode(trackerData startupData);
#endif

	///brief	Periodically checks for delays in data transmission, for detection of NUCs. 
#ifdef _BUILD_FOR_ROS_
	void timed_loop(const ros::TimerEvent& event);
#else
	void timed_loop();
#endif

	///brief	Main function for attempting feature tracking. 
#ifdef _BUILD_FOR_ROS_
	void features_loop(const ros::TimerEvent& event);
#else
	void features_loop();
#endif

	///brief	Processes provided configuration data to determine other calibration parameters etc. 
#ifdef _BUILD_FOR_ROS_
	void process_info(const sensor_msgs::CameraInfoConstPtr& info_msg);
#else
	void process_info(sensor_msgs::CameraInfo *info_msg);
#endif

	///brief	Initial receipt of an image. 
#ifdef _BUILD_FOR_ROS_
	void handle_camera(const sensor_msgs::ImageConstPtr& msg_ptr, const sensor_msgs::CameraInfoConstPtr& info_msg);
#else
	void handle_camera(cv::Mat& inputImage, sensor_msgs::CameraInfo *info_msg);
#endif

	///brief	Broadcasts tracking information. 
#ifdef _BUILD_FOR_ROS_
	int publish_tracks(ros::Publisher *pub_message, unsigned int latestIndex);
#else
	int publish_tracks(unsigned int latestIndex);
#endif

	///brief	Calculates the average motion of all features successfully tracked from the previous frame.
	int calculateFeatureMotion(unsigned int idx, double& mx, double &my);

	///brief	Loads predetected keypoints from a file as an alternative to detecting them from the image
	void prepareKeypointFilelist();

	///brief	Loads predetected keypoints from a file as an alternative to detecting them from the image
	void loadKeypointsFromFile(vector<cv::KeyPoint>& pts_vec);
	
	///brief	Handles the situation where a duplicate frame is received (implying a NUC interruption).
	void handle_delay();

	///brief	Handles the situation where the first new frame after a NUC interruption arrives.
	void handle_very_new();

	///brief	Clear all values stored in the candidates arrays
	void clearCandidates();
	
	///brief	Reduces the size of the feature track vector by eliminating unlikely-to-be-recovered tracks.
	void trimFeatureTrackVector();

	///brief	Reduces the size of the debug/display feature track vector by eliminating unlikely-to-be-recovered tracks.
	void trimDisplayTrackVectors();
	
	///brief	Updates the search distance radius, possibly based on factors such as feature velocity as well as user settings.
	void updateDistanceConstraint();

	///brief	Attempts to track features from preceding frame.
	void attemptTracking();

	///brief	Attempts to track features from historically recorded frames.
	void attemptHistoricalTracking();

	///brief	Attempts to detect new features on current frame.
	void detectNewFeatures();

	///brief	Uses new detection and matching results to update the feature vectors.
	void updateTrackingVectors();

	///brief	Publishes all data relating to a completely processed frame, including for debugging.
	void publishRoutine();

	///brief	Attempts to match newly detected features with existing untracked features. 
	void matchWithExistingTracks();
	
	///brief	Initial categorization (e.g. NUC/non-NUC) of an image. 
	void act_on_image();

	///brief	Updates tracker configuration data, potentially based on first received image which contains image size etc. 
	void updateTrackerData(trackerData newTrackerData) { configData = newTrackerData; }

	///brief	Updates the history/loop closure parameters, potentially based on user input. 
	void updateHistoryParameters();
	
	///brief	Sets the debug mode - if true, images and data for visualization will be displayed to the user. 
	void setDebugMode(bool val) { 
		debugMode = val;
		configData.debugMode = val; 
	}
	
	///brief	Prepares node for termination.
	void prepareForTermination();

	///brief	Displays latest debug frame
	void displayCurrentFrame();
	
};

#endif
