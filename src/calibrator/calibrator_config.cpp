#include "calibrator/calibrator_config.hpp"

calibratorSharedData::calibratorSharedData()
{ }

calibratorRealtimeOnlyData::calibratorRealtimeOnlyData()
{ }

calibratorLaunchOnlyData::calibratorLaunchOnlyData() :
#ifndef _BUILD_FOR_ROS_
	displayDebug(false),
	displayGUI(false),
	fixIntrinsics(true),
	fixPrincipalPoint(false),
	noConstraints(false),
	video_stream("video_stream"),
	ignoreDistortion("ignoreDistortion"),
	useRationalModel(false),
	calibType("calibType"),
	useUndistortedLocations(false),
	removeSpatialBias(true),
	video_stream_secondary("video_stream_secondary"),
	flowThreshold(1e-4),
	maxFrac(0.05),
	errorThreshold(0.0),
	patternType("mask"),
	setSize(DEFAULT_SET_SIZE),
	gridSize(DEFAULT_GRID_SIZE),
	maxTimeDiff(DEFAULT_MAX_TIME_DIFF),
	xCount(DEFAULT_X_COUNT),
	yCount(DEFAULT_Y_COUNT),
	optMethod("enhancedMCM"),
	undistortImages(false),
	rectifyImages(false),
	wantsIntrinsics(false),
	alpha(0.0),
	autoAlpha(true),
	stopCapturingAtLimit(false),
	patternDetectionMode("find"),
	numCams(1),
#endif
	specialMode(false)
{ 
	intrinsicsFiles[0] = "intrinsicsFile";
	intrinsicsFiles[1] = "intrinsicsFile";
}
