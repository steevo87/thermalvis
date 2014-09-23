#include "../../include/slam/slam_config.hpp"

slamSharedData::slamSharedData() :
	maxInitializationFrames(MAX_INITIALIZATION_FRAMES)
{ }

slamRealtimeOnlyData::slamRealtimeOnlyData() :
	pauseMode(false)
{ }

slamLaunchOnlyData::slamLaunchOnlyData() :
#ifndef _BUILD_FOR_ROS_
	displayDebug(false),
	displayGUI(false),
#endif
	specialMode(false)
{ }
