#include "calibrator/calibrator_config.hpp"

calibratorSharedData::calibratorSharedData() :
	autoTimeout(0.0),
	stopCapturingAtLimit(false),
	maxTime(600.0),
	flowThreshold(1e-4),
	maxFrac(0.05),
	maxCandidates(1000),
	maxTests(100),
	errorThreshold(0.0),
	setSize(DEFAULT_SET_SIZE),
	alpha(0.0),
	autoAlpha(true),
	invertPrimary(false), 
	invertSecondary(false)
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
	useRationalModel(false),
	calibType("calibType"),
	useUndistortedLocations(false),
	removeSpatialBias(true),
	video_stream_secondary("video_stream_secondary"),
	ignoreDistortion(false),
	patternType("mask"),
	gridSize(DEFAULT_GRID_SIZE),
	maxTimeDiff(DEFAULT_MAX_TIME_DIFF),
	xCount(DEFAULT_X_COUNT),
	yCount(DEFAULT_Y_COUNT),
	optMethod("enhancedMCM"),
	undistortImages(false),
	rectifyImages(false),
	wantsIntrinsics(false),
	patternDetectionMode("find"),
	numCams(1),
    numDetectors(0),
    tracksOutputTopic(""),
    outputFeatureMotion(false),
    normalizeFeatureVelocities(true),
    outputTrackCount(false),
#endif
	specialMode(false)
{ 
	intrinsicsFiles[0] = "intrinsicsFile";
	intrinsicsFiles[1] = "intrinsicsFile";
}
