/*! \file	input_stream_config.hpp
 *  \brief	Declarations for video input configuration.
*/

#include "streamer_defines.hpp"

#ifndef _THERMALVIS_INPUT_STREAM_CONFIG_H_
#define _THERMALVIS_INPUT_STREAM_CONFIG_H_

/// \brief		Parameters that are shared between both real-time update configuration, and program launch configuration for streamer
struct streamerSharedData {
	bool output16bit, output8bit, outputColor, undistortImages, verboseMode, autoTemperature, debugMode, showExtremeColors, wantsToMarkDuplicates, wantsToUndistort, wantsToDumpTimestamps;
	bool wantsToAddExtrinsics, wantsToRectify;
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