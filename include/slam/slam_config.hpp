/*! \file	slam.hpp
 *  \brief	Declarations for slam node configuration.
*/

#ifndef THERMALVIS_SLAM_CONFIG_H_
#define THERMALVIS_SLAM_CONFIG_H_

#include <string>

// Should have lots of defines

/// \brief		Parameters that are shared between both real-time update configuration, and program launch configuration for slam
struct slamSharedData {
	bool verboseMode, debugMode;

	slamSharedData();
};

/// \brief		Parameters that are only changeable from real-time interface
struct slamRealtimeOnlyData {
	bool pauseMode;
	slamRealtimeOnlyData();
};

/// \brief		Parameters that are only changeable from launch interface
struct slamLaunchOnlyData {
	bool specialMode;
#ifndef _BUILD_FOR_ROS_
	bool displayDebug, displayGUI;
#endif
	slamLaunchOnlyData();
};

/// \brief		Substitute for ROS live configuration adjustments
class slamRealtimeData : public slamSharedData, public slamRealtimeOnlyData {
protected:
    // ...

public:
	slamRealtimeData() { }
};

#endif
