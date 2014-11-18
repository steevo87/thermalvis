#include "../../include/calibrator/calibrator_config.hpp"

calibratorSharedData::calibratorSharedData()
{ }

calibratorRealtimeOnlyData::calibratorRealtimeOnlyData()
{ }

calibratorLaunchOnlyData::calibratorLaunchOnlyData() :
#ifndef _BUILD_FOR_ROS_
	displayDebug(false),
	displayGUI(false),
#endif
	specialMode(false)
{ }
