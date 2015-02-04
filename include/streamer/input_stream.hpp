/*! \file	input_stream.hpp
 *  \brief	Declarations for generic video input.
*/

#ifndef _THERMALVIS_INPUT_STREAM_H_
#define _THERMALVIS_INPUT_STREAM_H_

#include "core/tools.hpp"
#include "core/general_resources.hpp"

#include "time.h"

#include "cxcore.h"
#include "highgui.h"

#include "serial_comms.hpp"

#ifdef _BUILD_FOR_ROS_
#include "core/ros_resources.hpp"
#include <thermalvis/streamerConfig.h>
#include <signal.h>
#endif

#ifdef _AVLIBS_AVAILABLE_
#include "streamer/video.hpp"
#endif

//#include <opencv2/videoio/videoio.hpp>
	
#include "core/launch.hpp"
#include "core/program.hpp"
#include "core/improc.hpp"
#include "core/colormapping.hpp"
#include "streamer/radiometric.hpp"
#include "core/camera.hpp"

#ifndef _IS_WINDOWS_
#include "serial_comms.hpp"
#endif

#ifdef _USE_BOOST_
#include <boost/filesystem.hpp>
#include <boost/filesystem/path.hpp>
#ifndef _BUILD_FOR_ROS_
#include <boost/algorithm/string/replace.hpp>
#endif
#endif

#include "streamer_config.hpp"

#define _16BIT_MEDIAN_BUFFER_SIZE				256

#define DEFAULT_SERIAL_POLLING_RATE			 	0.04

#define DEFAULT_READ_RATE 						25.0
#define MAX_READ_RATE 							100.0
#define FASTEST_READ_RATE 						1000.0

#define DEFAULT_IMAGE_WIDTH 					288 // 288 for Optris PI450, 480 for Miricle 307K
#define DEFAULT_IMAGE_HEIGHT 					382 // 382 for Optris PI450, 640 for Miricle 307K

#define MAX_ROWS 								1920
#define MAX_COLS 								2560

#define MAX_THERMISTOR_READINGS_TO_STORE 		100

#define UVC_STREAM_PTS                                  (1 << 2)
#define UVC_STREAM_SCR                                  (1 << 3)

#define NO_REPUBLISH_CODE 0
#define REPUBLISH_CODE_8BIT_MONO 1
#define REPUBLISH_CODE_8BIT_COL 2
#define REPUBLISH_CODE_16BIT 3

#define SYNCMODE_HARD						0
#define SYNCMODE_SOFT						1
#define SYNCMODE_IMAGEONLY					2

#define CALIBMODE_OFF						0
#define CALIBMODE_ALT_SHUTTER				1 // Alternates shutter and sends NUCs
#define CALIBMODE_ALT_OUTPUT				2 // Alternates between RAW and INS camera outputs
#define CALIBMODE_LONG_SHUTTER				3 // Alternates shutter, but lets NUCs occur naturally

#define MIN_THERMISTOR_READNG				-20.0
#define MAX_THERMISTOR_READING				60.0

#define SERIAL_COMMS_CONFIG_DEFAULT			0	// Seems to work with the 1-1 USB serial converter
#define SERIAL_COMMS_CONFIG_1				1	// Seems to work with the USB-serial Hub

#define DEFAULT_COLORSCHEME_INDEX	3
#define DEFAULT_COLORSCHEME_CODE	CIELUV

#define DEFAULT_MIN_TEMP			25.0
#define DEFAULT_MAX_TEMP			35.0

#define IMAGE_FILTER_NONE					0
#define IMAGE_FILTER_GAUSSIAN				1
#define IMAGE_FILTER_BILATERAL				2
#define IMAGE_FILTER_ADAPTIVE_BILATERAL		3

#define OUTPUT_TYPE_CV_8UC1		0
#define OUTPUT_TYPE_CV_8UC3		1
#define OUTPUT_TYPE_CV_16UC1	2

// "hardSync", int_t, 0, "Image and camera_info topics must be fully synchronized"),
// "softSync", int_t, 1, "Image and camera_info topics do not have to be fully synchronized"),
// "imageOnly", int_t, 2, "To be used when no camera_info topic is present") ],

//int getMapIndex(string mapping);
void getMapping(int mapCode, int& fullMapCode);

#ifndef _IS_WINDOWS_
void displayTermiosData(termios options);
#endif

/// \brief		Stores configuration information for the streamer/image-processing routine
class streamerData : public streamerSharedData, public commonData, public streamerLaunchOnlyData {
	friend class xmlParameters;
	friend class streamerNode;
#ifndef _BUILD_FOR_ROS_
	friend class streamerConfig;
#endif

protected:
	bool dataValid;
	int syncMode, camera_number, desiredRows, desiredCols, temporalMemory, outputType;
	
	string radiometryFile, externalNucManagement, portAddress, source, file, folder, capture_device, intrinsics, extrinsics, topicname;
	string timeStampsAddress, republishTopic, frameID, outputFormatString, outputTimeFile, outputVideo, videoType, outputTypeString;

	bool radiometricCorrection, radiometricRaw, serialFeedback, useCurrentRosTime, alreadyCorrected, markDuplicates, outputDuplicates, smoothThermistor;
	bool radiometricInterpolation, displayThermistor, serialComms, readThermistor, forceInputGray, fixDudPixels, disableSkimming;
	
	bool loopMode, resizeImages, removeDuplicates, temporalSmoothing, pauseMode, stepChangeTempScale, readTimestamps;
	bool rectifyImages, writeImages, keepOriginalNames, writeVideo, addExtrinsics, republishNewTimeStamp, drawReticle, autoAlpha;

	int filterMode, radiometricBias, calibrationMode, alternatePeriod, dummy, serialCommsConfigurationCode, serialWriteAttempts;
	int republishSource, outputFormat, device_num, maxIntensityChange;
	double filterParam, thermistorWindow, syncDiff, writeQuality, maxThermistorDiff, alpha;
	
	vector<int> outputFileParams;

	unsigned long soft_diff_limit;

public:
	bool captureMode, readMode, loadMode, subscribeMode, resampleMode, pollMode;
	string read_addr;
	
	streamerData();

	bool assignFromXml(xmlParameters& xP);

#ifdef _BUILD_FOR_ROS_
	bool obtainStartingData(ros::NodeHandle& nh);   
#endif

};

#ifndef _BUILD_FOR_ROS_
/// \brief		Substitute for ROS live configuration adjustments
class streamerConfig : public streamerRealtimeData {
public:
	streamerConfig() { }
	bool assignStartingData(streamerData& startupData);

};
#endif

/// \brief		Stores some basic camera information in OpenCV format
struct camData_ {
	cv::Mat cameraMatrix, distCoeffs, imageSize;
	
	cv::Size cameraSize;
	cv::Mat newCamMat;
	
	cv::Mat R, T;

	camData_();
};

/// \brief		Stores some basic extrinsic camera information in OpenCV format
struct camExtrinsicsData_ {
    cv::Mat R, T;
    cv::Mat R0, R1, T0, T1, P0, P1, cameraMatrix0, cameraMatrix1, distCoeffs0, distCoeffs1;
    cv::Rect roi0, roi1;
};

/// \brief		Manages the driver / streamer
class streamerNode : public GenericOptions {
private:

#ifdef _BUILD_FOR_ROS_
	image_transport::ImageTransport *it;
	image_transport::Subscriber image_sub;
	image_transport::CameraSubscriber camera_sub;
	image_transport::Publisher pub_color_im, pub_16bit_im, pub_8bit_im, pub_republish_im;
	image_transport::CameraPublisher pub_color, pub_16bit, pub_8bit, pub_republish;

	ros::Subscriber info_sub, nuc_management_sub;
	
	
	sensor_msgs::Image msg_color, msg_16bit, msg_8bit;

	ros::Timer timer, serial_timer;
	
	ros::ServiceServer set_camera_info;
	
	dynamic_reconfigure::Server<thermalvis::streamerConfig> server;
	dynamic_reconfigure::Server<thermalvis::streamerConfig>::CallbackType f;
#endif

	ros::Time info_time, image_time, original_time, dodgeTime, lastFlagReceived, lastNucPerformed_at_the_earliest;
	sensor_msgs::CameraInfo original_camera_info, camera_info;

#ifdef _BUILD_FOR_ROS_
	cv_bridge::CvImagePtr cv_ptr;
#else
	const cv::Mat *bridgeReplacement;
#endif
	
	double lastMinDisplayTemp, lastMaxDisplayTemp;
	bool deviceCreated, currentDesiredNucProtectionMode, currentNucProtectionMode, wantsToDisableNuc, firstCall, settingsDisabled, canRadiometricallyCorrect;
	bool lastIsDuplicate, updateNucInterval, updateDetectorMode, updateUSBMode, altStatus, performingNuc;

	unsigned int recordedThermistorReadings;
	double thermistorBuffer[MAX_THERMISTOR_READINGS_TO_STORE][2];
		
	double lastDisplayed, medianPercentile, shiftDiff;
	
	rScheme radMapper;
	
	ofstream ofs;

	int alternateCounter, pastMeanIndex, fullMapCode;
	vector<double> prereadTimestamps;
	
	double pastMeans[256];
	int past16bitMedians[_16BIT_MEDIAN_BUFFER_SIZE];
	
	string callLogFile, retrieveLogFile, internalLogFile, writeLogFile, duplicatesLogFile, thermistorLogFile;
	ofstream ofs_call_log, ofs_internal_log, ofs_retrieve_log, ofs_write_log, ofs_duplicates_log, ofs_thermistor_log;
	
	cv::Mat testMat, temperatureMat;
	
	float lastThermistorReading, newThermistorReading, originalInternalTime;
	
	streamerData configData;
	
	unsigned long original_bx, original_by, internal_time;
	
#ifdef _WIN32
	signed long long int firmwareTime;
#else
	int64_t firmwareTime;
#endif
	
	bool firstServerCallbackProcessed, centerPrincipalPoint, firstFrame;
	
	double fusionFactor, lastMedian, newMedian, lastLowerLimit, lastUpperLimit, oldMaxDiff, minVal, maxVal;

	cv::Mat lastFrame, map1, map2, rzMat;
	
	cScheme colourMap;

	int writeIndex, frameCounter, lastWritten;
	
	cv::VideoWriter vid_writer;
	
	bool videoInitialized, readyToPublish, alphaChanged;

	cv::Mat _8bitMat, _16bitMat, colourMat;
	cv::Mat preFilteredMat, smoothedMat, scaled16Mat;
	cv::Mat _8bitMat_pub, _16bitMat_pub, colourMat_pub;
	cv::Mat newImage, normalizedMat, videoFrame, frame, workingFrame, undistorted;
	
	camData_ globalCameraInfo;
    camExtrinsicsData_ globalExtrinsicsData;

	bool isActuallyGray, videoValid;

	vector<string> inputList;
	int fileCount;
	
#ifdef _AVLIBS_AVAILABLE_
	streamerSource *mainVideoSource;
#endif

	cv::VideoCapture cap;
	
	// Serial Comms
	int mainfd; /* File descriptor */
	
public:

	///brief	Processes online changes in node configuration and applies them.
#ifdef _BUILD_FOR_ROS_
	void serverCallback(thermalvis::streamerConfig &config, uint32_t level);
#else
	void serverCallback(streamerConfig &config);
#endif

	///brief	Constructor. Must receive configuration data.
#ifdef _BUILD_FOR_ROS_
	streamerNode(ros::NodeHandle& nh, streamerData startupData);
#else
	streamerNode(streamerData startupData);
#endif

	///brief	Initial receipt of an image. 
#ifdef _BUILD_FOR_ROS_
	void handle_camera(const sensor_msgs::ImageConstPtr& msg_ptr, const sensor_msgs::CameraInfoConstPtr& info_msg);
#else
	void handle_camera(const cv::Mat& inputImage, const sensor_msgs::CameraInfo *info_msg);
#endif

	///brief	Initial receipt of an image WITHOUT camera info. 
#ifdef _BUILD_FOR_ROS_
	void handle_image(const sensor_msgs::ImageConstPtr& msg_ptr);
#else
	void handle_image(const cv::Mat& inputImage);
#endif

	///brief	Initial receipt of an image/camera info. 
#ifdef _BUILD_FOR_ROS_
	void handle_info(const sensor_msgs::CameraInfoConstPtr& info_msg);
#else
	void handle_info(const sensor_msgs::CameraInfo *info_msg);
#endif

	///brief	Handles a command to perform a NUC. 
#ifdef _BUILD_FOR_ROS_
	void handle_nuc_instruction(const std_msgs::Float32::ConstPtr& nuc_msg);
#else
	void handle_nuc_instruction(const bool& perform_nuc);
#endif

	///brief	Set the camera info. 
#ifdef _BUILD_FOR_ROS_
	bool setCameraInfo(sensor_msgs::SetCameraInfo::Request &req, sensor_msgs::SetCameraInfo::Response &rsp);
#else
	bool setCameraInfo();
#endif
	
	///brief	Callback for serial comms. 
#ifdef _BUILD_FOR_ROS_
	void serialCallback(const ros::TimerEvent& e);
#else
	void serialCallback();
#endif
	
	///brief	Periodically checks for some things... 
#ifdef _BUILD_FOR_ROS_
    void timerCallback(const ros::TimerEvent& e);
#else
	bool loopCallback();
#endif

	///brief	Extracts the next frame from the video file
	bool getFrameFromVideoFile();

	///brief	Extracts the next frame from the topic subscription?
	bool getFrameFromSubscription();

	bool getFrameUsingAVLibs();

	bool getFrameUsingCVLibs();

    bool setupVideoFile();
    bool closeVideoFile();

	bool getFrameFromDirectoryNONROS();
	bool getFrameFromDirectoryROS();

	void updateCameraInfo();
	void assignCameraInfo();

#ifdef _BUILD_FOR_ROS_
	void updateCameraInfoExtrinsics();
	void refreshCameraAdvertisements();
#else
	bool get8bitImage(cv::Mat& img, sensor_msgs::CameraInfo& info);
#endif

	void assignDefaultCameraInfo(int rows = DEFAULT_IMAGE_HEIGHT, int cols = DEFAULT_IMAGE_WIDTH, bool guessIntrinsics = false);
	
	bool isVideoValid();
	
	void setValidity(bool val) { videoValid = val; }
	
#ifdef _BUILD_FOR_ROS_
	void updateThermistor();
	void calibrationModeRoutine();
#endif

#ifdef _AVLIBS_AVAILABLE_
    streamerSource * getMainVideoSource() { return mainVideoSource; }
#endif

	cv::VideoCapture * getVideoCapture() { return &cap; }
	bool setupVideoForReading();

	void displayFrame(cv::Mat& frame, std::string name = "streamer_display");

	bool streamCallback(bool capture = true);
	void acceptImage(void *ptr);
	bool processImage();
	bool imageLoop();
	void publishTopics();
	void writeData();
	bool processFolder();
    bool prepareVideo();
	
    void overwriteCameraDims();

#ifdef _BUILD_FOR_ROS_
	bool runBag();
	void initializeMessages();
#endif

	bool sendSerialCommand(char *command, int max_attempts = 1);
	
	void act_on_image();
	
	bool run();
	// Source alternatives
	
	bool runRead();
	bool runLoad();
	bool runDevice();
	
	bool performNuc();
	
	void updateMap();
	
	void prepareForTermination();
	
	void markCurrentFrameAsDuplicate();
	
	bool getNucSettingsReading(int& delay, double& diff);
	
#ifndef _WIN32
	bool configureSerialComms();
#endif

	float getThermistorReading();
	double smoothThermistorReading();
	
	bool setupDevice();
	void releaseDevice();
	
	int open_port();
    void getRectification();
	
};

/// \brief		Temporary class for streaming in Windows from the Optris camera
class inputStream : public GenericOptions {

protected:
	
	int colormap_index;

	bool autoscaleTemps;

	double minTemp, maxTemp;

	short FrameWidth, FrameHeight, FrameDepth;
	double FrameRatio;
	int FrameSize;
	
	cv::Mat *rawImage, *scaledImage, *_8bitImage;
	cScheme *cMapping;

public:
	sensor_msgs::CameraInfo camera_info;

	inputStream();
	bool processFrame();
	void colorizeFrame() { cMapping->falsify_image(*_8bitImage, *displayImage); }		// THIS IS SLOWING THINGS DOWN!!
	void displayCurrentFrame();
	bool accessLatestRawFrame(cv::Mat& latestFrame);
	bool accessLatest8bitFrame(cv::Mat& latestFrame);
	bool writeImageToDisk();

	void load_standard(int mapCode, int mapParam = 0) { cMapping->load_standard(mapCode, mapParam); }
	int get_colormap_index() { return colormap_index; }
	void set_colormap_index(int index) { colormap_index = index; }
	void set_autoscaleTemps(bool setting) { autoscaleTemps = setting; }

	void assignMemoryToRawImage(int height, int width) { rawImage = new cv::Mat(height, width, CV_16UC1); }
	void assignDataToRawImage(uchar *buff) { rawImage->data = buff; }
	bool getPauseMode() { return pauseMode; }
	void switchPauseMode() { pauseMode = !pauseMode; }

	void setMinTemp(double temperature) { minTemp = temperature; }
	void setMaxTemp(double temperature) { maxTemp = temperature; }
	
};

#endif
