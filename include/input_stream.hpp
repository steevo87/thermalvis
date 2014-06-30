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

#include "boost/filesystem.hpp"  

#define DEFAULT_COLORSCHEME_INDEX	3
#define DEFAULT_COLORSCHEME_CODE	CIELUV

#define DEFAULT_MIN_TEMP			25.0
#define DEFAULT_MAX_TEMP			35.0

#define NORM_MODE_STANDARD			0
#define NORM_MODE_EQUALIZE			1
#define NORM_MODE_CLAHE				2
#define NORM_MODE_CLAHE_ADAPTIVE	3
#define NORM_MODE_CENTRALIZED		4
#define NORM_MODE_EXPANDED			5

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

/// \brief		Parameters that are shared between both real-time update configuration, and program launch configuration for streamer
struct streamerSharedData {
	bool output16bit, output8bit, outputColor, undistortImages, verboseMode, autoTemperature, debugMode;
	int maxReadAttempts, normMode, maxNucInterval, map;
	double framerate, normFactor, fusionFactor, serialPollingRate, maxNucThreshold, minTemperature, maxTemperature;

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
	int detectorMode, usbMode, inputDatatype, syncMode, camera_number, desiredRows, desiredCols, mapCode, mapParam, temporalMemory, outputType;
	
	string radiometryFile, externalNucManagement, portAddress, read_addr, source, filename, folder, capture_device, intrinsics, extrinsics, topicname, normalizationMode;
	string timeStampsAddress, republishTopic, outputFolder, frameID, outputFormatString, outputTimeFile, outputVideo, videoType;

	bool radiometricCorrection, radiometricRaw, serialFeedback, useCurrentRosTime, alreadyCorrected, wantsToMarkDuplicates, wantsToOutputDuplicates, smoothThermistor;
	bool radiometricInterpolation, imageDimensionsSpecified, displayThermistor, serialComms, readThermistor, wantsToUndistort, forceInputGray, fixDudPixels, disableSkimming;
	bool captureMode, readMode, loadMode, subscribeMode, resampleMode, pollMode;
	bool loopMode, wantsToResize, wantsToDumpTimestamps, wantsToRemoveDuplicates, temporalSmoothing, pauseMode, extremes, stepChangeTempScale;
	bool intrinsicsProvided, wantsToRectify, wantsToWrite, wantsToKeepNames, wantsToEncode, wantsToAddExtrinsics, republishNewTimeStamp, drawReticle, autoAlpha;

	int filterMode, radiometricBias, calibrationMode, alternatePeriod, dummy, inputWidth, inputHeight, serialCommsConfigurationCode, maxNucInterval, serialWriteAttempts;
	int republishSource, outputFormat, device_num;
	double filterParam, thermistorWindow, serialPollingRate, maxNucThreshold, syncDiff, writeQuality, framerate, maxThermistorDiff, maxIntensityChange, alpha;
	
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
	streamerConfig();
	void assignStartingData(streamerData& startupData);

};
#endif

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
	void colorizeFrame() { cMapping->falsify_image(*_8bitImage, *displayImage, 0); }		// THIS IS SLOWING THINGS DOWN!!
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
