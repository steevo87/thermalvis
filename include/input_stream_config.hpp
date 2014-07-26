/*! \file	input_stream_config.hpp
 *  \brief	Declarations for video input configuration.
*/

#ifndef _THERMALVIS_INPUT_STREAM_CONFIG_H_
#define _THERMALVIS_INPUT_STREAM_CONFIG_H_

#define DETECTOR_MODE_RAW					0
#define DETECTOR_MODE_LUM					1
#define DETECTOR_MODE_INS					2
#define DETECTOR_MODE_RAD					3
#define DETECTOR_MODE_TMP					4

#define NO_FILTERING 						0
#define GAUSSIAN_FILTERING 					1
#define BILATERAL_FILTERING 				2

#define USB_MODE_16							1
#define USB_MODE_8							2

#define DATATYPE_8BIT 						0
#define DATATYPE_RAW  						1
#define DATATYPE_MM   						2
#define DATATYPE_DEPTH						3

#define NORM_MODE_FULL_STRETCHING 			0
#define NORM_MODE_EQUALIZATION 				1
#define NORM_MODE_CENTRALIZED				2
#define NORM_MODE_FIXED_TEMP_RANGE			3
#define NORM_MODE_FIXED_TEMP_LIMITS			4

#define CONFIG_MAP_CODE_GRAYSCALE			0
#define CONFIG_MAP_CODE_CIECOMP				1
#define CONFIG_MAP_CODE_BLACKBODY			2
#define CONFIG_MAP_CODE_RAINBOW				3
#define CONFIG_MAP_CODE_IRON				4
#define CONFIG_MAP_CODE_BLUERED				5
#define CONFIG_MAP_CODE_JET					6
#define CONFIG_MAP_CODE_CIELUV				7
#define CONFIG_MAP_CODE_ICEIRON				8
#define CONFIG_MAP_CODE_ICEFIRE				9
#define CONFIG_MAP_CODE_REPEATED			10
#define CONFIG_MAP_CODE_HIGHLIGHTED			11

/// \brief		Parameters that are shared between both real-time update configuration, and program launch configuration for streamer
struct streamerSharedData {
	bool output16bit, output8bit, outputColor, undistortImages, verboseMode, autoTemperature, debugMode, showExtremeColors;
	int maxReadAttempts, normMode, maxNucInterval, mapCode, inputDatatype, detectorMode, usbMode, zeroDegreesOffset;
	double framerate, threshFactor, normFactor, fusionFactor, serialPollingRate, maxNucThreshold, minTemperature, maxTemperature, degreesPerGraylevel, desiredDegreesPerGraylevel;

	streamerSharedData();
};

/// \brief		Parameters that are only changeable from real-time interface
struct streamerRealtimeOnlyData {
	bool pauseMode;

	streamerRealtimeOnlyData();
};

/// \brief		Parameters that are only changeable from launch interface
struct streamerLaunchOnlyData {
	bool specialMode;
#ifndef _BUILD_FOR_ROS_
	bool display8bit, display16bit, displayColour, displayGUI;
#endif
	streamerLaunchOnlyData();
};

/// \brief		Substitute for ROS live configuration adjustments
class streamerRealtimeData : public streamerSharedData, public streamerRealtimeOnlyData {
protected:
    // ...

public:
	streamerRealtimeData() { }
};

#endif