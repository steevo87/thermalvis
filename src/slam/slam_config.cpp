#include "../../include/slam/slam_config.hpp"

slamSharedData::slamSharedData() :
	maxInitializationFrames(MAX_INITIALIZATION_FRAMES),
	maxInitializationSeconds(MAX_INITIALIZATION_SECONDS),  
	minInitializationConfidence(MIN_INITIALIZATION_CONFIDENCE),
	maxTestsPerFrame(MAX_TESTS_PER_FRAME)
{ }

slamRealtimeOnlyData::slamRealtimeOnlyData() :
	pauseMode(false)
{ }

slamLaunchOnlyData::slamLaunchOnlyData() :
#ifndef _BUILD_FOR_ROS_
	displayDebug(false),
	displayGUI(false),
#endif
	inspectInitialization(false),
	keyframeEvaluationMode(false),
	specialMode(false),
	logErrors(false),
	writePoses(false),
	evaluateParameters(false)
{ }
