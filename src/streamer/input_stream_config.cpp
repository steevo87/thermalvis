#include "streamer/input_stream_config.hpp"

streamerSharedData::streamerSharedData() : 
	detectorMode(DETECTOR_MODE_INS),
	usbMode(USB_MODE_16),
	inputDatatype(DATATYPE_RAW),
	maxReadAttempts(0), 
	framerate(-1.0),
	normMode(NORM_MODE_FIXED_TEMP_RANGE),
	normFactor(0.2),
	threshFactor(0.0), 
	output16bit(false), 
	output8bit(true),
	outputColor(false),
	map(CONFIG_MAP_CODE_CIELUV),
	fusionFactor(0.6),  
	serialPollingRate(25.0),
	maxNucInterval(45),
	maxNucThreshold(0.2),
	verboseMode(false), 
	autoTemperature(false),
	minTemperature(25.0), 
	maxTemperature(35.0),
    degreesPerGraylevel(DEFAULT_DEGREES_PER_GRAYLEVEL),
    desiredDegreesPerGraylevel(DEFAULT_DESIRED_DEGREES_PER_GRAYLEVEL),
	zeroDegreesOffset(0),
	debugMode(false),
	wantsToAddExtrinsics(false),
	wantsToUndistort(false),
	wantsToRectify(false)
{ }

streamerRealtimeOnlyData::streamerRealtimeOnlyData() :
	pauseMode(false)
{ }

streamerLaunchOnlyData::streamerLaunchOnlyData() :
#ifndef _BUILD_FOR_ROS_
	display8bit(false), 
	display16bit(false), 
	displayColour(false),
	displayGUI(false),
#endif
	specialMode(false),
	dumpTimestamps(false),
    extremes(true),
	wantsToEncode(false),
	wantsToKeepNames(false),
	wantsToResize(false)
{ }
