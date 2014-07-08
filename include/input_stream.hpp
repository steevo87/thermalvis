/*! \file	input_stream.hpp
 *  \brief	Declarations for generic video input.
*/

#ifndef _THERMALVIS_INPUT_STREAM_H_
#define _THERMALVIS_INPUT_STREAM_H_

#include "general_resources.hpp"
#include "ros_resources.hpp"
#include "opencv_resources.hpp"
	
#include "launch.hpp"
#include "program.hpp"
#include "improc.hpp"
#include "radiometric.hpp"

#include "boost/filesystem.hpp"  

#ifndef _BUILD_FOR_ROS_
#include <boost/algorithm/string/replace.hpp>
#include <boost/filesystem.hpp>
#endif

#define DEFAULT_SERIAL_POLLING_RATE			 	0.04

#define DEFAULT_READ_RATE 						25.0
#define MAX_READ_RATE 							100.0
#define FASTEST_READ_RATE 						1000.0

#define DEFAULT_IMAGE_WIDTH 					640
#define DEFAULT_IMAGE_HEIGHT 					480

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

#define NORM_MODE_STANDARD			0
#define NORM_MODE_EQUALIZE			1
#define NORM_MODE_CENTRALIZED		2

#define CONFIG_MAP_CODE_GRAYSCALE		0
#define CONFIG_MAP_CODE_CIECOMP			1
#define CONFIG_MAP_CODE_BLACKBODY		2
#define CONFIG_MAP_CODE_RAINBOW			3
#define CONFIG_MAP_CODE_IRON			4
#define CONFIG_MAP_CODE_BLUERED			5
#define CONFIG_MAP_CODE_JET				6
#define CONFIG_MAP_CODE_CIELUV			7
#define CONFIG_MAP_CODE_ICEIRON			8
#define CONFIG_MAP_CODE_ICEFIRE			9
#define CONFIG_MAP_CODE_REPEATED		10
#define CONFIG_MAP_CODE_HIGHLIGHTED		11

#define DETECTOR_MODE_RAW					0
#define DETECTOR_MODE_LUM					1
#define DETECTOR_MODE_INS					2
#define DETECTOR_MODE_RAD					3
#define DETECTOR_MODE_TMP					4

#define IMAGE_FILTER_NONE					0
#define IMAGE_FILTER_GAUSSIAN				1
#define IMAGE_FILTER_BILATERAL				2
#define IMAGE_FILTER_ADAPTIVE_BILATERAL		3

#define USB_MODE_16							1
#define USB_MODE_8							2

#define DATATYPE_8BIT 	0
#define DATATYPE_RAW  	1
#define DATATYPE_MM   	2
#define DATATYPE_DEPTH  3

#define OUTPUT_TYPE_CV_8UC1		0
#define OUTPUT_TYPE_CV_8UC3		1
#define OUTPUT_TYPE_CV_16UC1	2

// "hardSync", int_t, 0, "Image and camera_info topics must be fully synchronized"),
// "softSync", int_t, 1, "Image and camera_info topics do not have to be fully synchronized"),
// "imageOnly", int_t, 2, "To be used when no camera_info topic is present") ],

//int getMapIndex(string mapping);
void getMapping(int mapCode, int& fullMapCode);

/// \brief		Parameters that are shared between both real-time update configuration, and program launch configuration for streamer
struct streamerSharedData {
	bool output16bit, output8bit, outputColor, undistortImages, verboseMode, autoTemperature, debugMode;
	int maxReadAttempts, normMode, maxNucInterval, mapCode, extremes, inputDatatype, detectorMode, usbMode;
	double framerate, threshFactor, normFactor, fusionFactor, serialPollingRate, maxNucThreshold, minTemperature, maxTemperature, tempGrad, tempIntercept;

	streamerSharedData();
};


/// \brief		Stores configuration information for the streamer/image-processing routine
class streamerData : public streamerSharedData, public commonData {
	friend class xmlParameters;
	friend class streamerNode;
#ifndef _BUILD_FOR_ROS_
	friend class streamerConfig;
#endif

protected:
	bool dataValid;
	int syncMode, camera_number, desiredRows, desiredCols, temporalMemory, outputType;
	
	string radiometryFile, externalNucManagement, portAddress, read_addr, source, filename, folder, capture_device, intrinsics, extrinsics, topicname;
	string timeStampsAddress, republishTopic, frameID, outputFormatString, outputTimeFile, outputVideo, videoType, outputTypeString;

	bool radiometricCorrection, radiometricRaw, serialFeedback, useCurrentRosTime, alreadyCorrected, markDuplicates, outputDuplicates, smoothThermistor;
	bool radiometricInterpolation, imageDimensionsSpecified, displayThermistor, serialComms, readThermistor, forceInputGray, fixDudPixels, disableSkimming;
	bool captureMode, readMode, loadMode, subscribeMode, resampleMode, pollMode;
	bool loopMode, resizeImages, dumpTimestamps, removeDuplicates, temporalSmoothing, pauseMode, stepChangeTempScale;
	bool intrinsicsProvided, rectifyImages, writeImages, keepOriginalNames, writeVideo, addExtrinsics, republishNewTimeStamp, drawReticle, autoAlpha;

	int filterMode, radiometricBias, calibrationMode, alternatePeriod, dummy, inputWidth, inputHeight, serialCommsConfigurationCode, serialWriteAttempts;
	int republishSource, outputFormat, device_num;
	double filterParam, thermistorWindow, syncDiff, writeQuality, maxThermistorDiff, maxIntensityChange, alpha;
	
	vector<int> outputFileParams;

	unsigned long soft_diff_limit;

public:
	streamerData();

	bool assignFromXml(xmlParameters& xP);

	#ifdef _BUILD_FOR_ROS_
	bool obtainStartingData(ros::NodeHandle& nh);   
	#endif

};

#ifndef _BUILD_FOR_ROS_
/// \brief		Substitute for ROS live configuration adjustments
class streamerConfig : public streamerSharedData {
	friend class streamerNode;

protected:
    // ...

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
	sensor_msgs::CameraInfo original_camera_info, camera_info;

	ros::Timer timer, serial_timer;
	
	ros::ServiceServer set_camera_info;
	
	dynamic_reconfigure::Server<thermalvis::streamerConfig> server;
	dynamic_reconfigure::Server<thermalvis::streamerConfig>::CallbackType f;
	#endif

#ifdef _BUILD_FOR_ROS_
	ros::Time info_time, image_time, original_time, dodgeTime, lastFlagReceived, lastNucPerformed_at_the_earliest;
	cv_bridge::CvImagePtr cv_ptr;
#else
	boost::posix_time::ptime info_time, image_time, original_time, dodgeTime, lastFlagReceived, lastNucPerformed_at_the_earliest;
	const cv::Mat *bridgeReplacement;
#endif

	char nodeName[256];
	
	double lastMinDisplayTemp, lastMaxDisplayTemp;
	bool deviceCreated, currentDesiredNucProtectionMode, currentNucProtectionMode, wantsToDisableNuc, firstCall, settingsDisabled, canRadiometricallyCorrect;
	bool lastIsDuplicate, updateNucInterval, updateDetectorMode, updateUSBMode, altStatus, performingNuc;

	unsigned int recordedThermistorReadings;
	double thermistorBuffer[MAX_THERMISTOR_READINGS_TO_STORE][2];
		
	double lastDisplayed, medianPercentile, shiftDiff;
	
	rScheme radMapper;
	
	ofstream ofs;

	int alternateCounter, pastMeanIndex, fullMapCode;
	
	double pastMeans[256];
	
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
	cv::Mat newImage, normalizedMat, frame, workingFrame, undistorted;
	
	camData_ globalCameraInfo;
    camExtrinsicsData_ globalExtrinsicsData;

	bool isActuallyGray, videoValid;

	vector<string> inputList;
	int fileCount;
	
#ifndef _WIN32
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

	#ifdef _BUILD_FOR_ROS_
	#endif

	///brief	Initial receipt of an image. 
#ifdef _BUILD_FOR_ROS_
	void handle_camera(const sensor_msgs::ImageConstPtr& msg_ptr, const sensor_msgs::CameraInfoConstPtr& info_msg);
#else
	void handle_camera(const cv::Mat& inputImage, const cameraInfoStruct *info_msg);
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
	void handle_info(const cameraInfoStruct *info_msg);
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
	void timerCallback();
#endif

#ifdef _BUILD_FOR_ROS_
	void updateCameraInfo();
	void assignCameraInfo();
	
	void updateCameraInfoExtrinsics();
	void refreshCameraAdvertisements();
#else
	bool retrieveRawFrame();
	bool get8bitImage(cv::Mat& img);
#endif

	void assignDefaultCameraInfo();

	CvCapture* capture;
	
#ifndef _BUILD_FOR_ROS_
	bool streamerNode::isVideoValid() { return videoValid; }
#endif
	
	void setValidity(bool val) { videoValid = val; }
	
#ifndef _WIN32
	streamerSource * getMainVideoSource() { return mainVideoSource; }
#endif

	cv::VideoCapture * getVideoCapture() { return &cap; }

	bool streamCallback(bool capture = true);
	void acceptImage(void *ptr);
	bool processImage();
	bool imageLoop();
	void publishTopics();
	void writeData();
	bool processFolder();
	
#ifdef _BUILD_FOR_ROS_
	void overwriteCameraDims();
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

#ifndef _BUILD_FOR_ROS_
	bool streamerNode::wantsToShutdown() { return false; }
#endif
	
};

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
	inputStream();
	bool processFrame();
	void colorizeFrame() { cMapping->falsify_image(*_8bitImage, *displayImage); }		// THIS IS SLOWING THINGS DOWN!!
	void displayFrame();
	bool accessLatestRawFrame(cv::Mat& latestFrame);
	bool accessLatest8bitFrame(cv::Mat& latestFrame);
	virtual bool writeImageToDisk();

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
