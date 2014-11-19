/*! \file	calibrator_config.hpp
 *  \brief	Declarations for calibrator node configuration.
*/

#ifndef THERMALVIS_CALIBRATOR_CONFIG_HPP
#define THERMALVIS_CALIBRATOR_CONFIG_HPP

#include <string>

#define MAX_ALLOWABLE_CAMS		2

#define DEFAULT_SET_SIZE		10
#define DEFAULT_GRID_SIZE		10 // (mm)
#define DEFAULT_X_COUNT			12
#define DEFAULT_Y_COUNT			8

#define DEFAULT_MAX_TIME_DIFF	0.001

/// \brief		Parameters that are shared between both real-time update configuration, and program launch configuration for flow
struct calibratorSharedData {
	double autoTimeout;
	bool drawGrids;
	bool debugMode;
	bool verboseMode;

	calibratorSharedData();
};

/// \brief		Parameters that are only changeable from real-time interface
struct calibratorRealtimeOnlyData {
	// ..

	calibratorRealtimeOnlyData();
};

/// \brief		Parameters that are only changeable from launch interface
struct calibratorLaunchOnlyData {
	bool specialMode;
	bool stopCapturingAtLimit;
	std::string intrinsicsFiles[MAX_ALLOWABLE_CAMS];
	bool invert[MAX_ALLOWABLE_CAMS];
	std::string patternType, optMethod;
	int optimizationMethod;
	std::string video_stream, video_stream_secondary;
	int pattType;
	std::string calibType;
	bool wantsIntrinsics;
	bool trackingMode;
	std::string patternDetectionMode;
	unsigned int numCams;
	bool  invertPrimary, invertSecondary;
	bool fixPrincipalPoint;
	bool fixIntrinsics;
	bool noConstraints;
	bool ignoreDistortion, useRationalModel;
	double maxInterpolationTimegap, maxInterpolationMotion;

	// Course settings
	bool useUndistortedLocations;
	double alpha;
	bool autoAlpha;
	bool removeSpatialBias;
	
	// Fine settings
	double maxTimeDiff;
	int maxPatterns, maxFrames, setSize, maxCandidates, maxTests;
	double maxTime;
	double flowThreshold, errorThreshold, maxFrac;
	
	// Pattern properties
	double gridSize;
	int xCount, yCount;

	// Debug
	bool generateFigures;
	bool undistortImages;
	bool rectifyImages;

#ifndef _BUILD_FOR_ROS_
	bool displayDebug, displayGUI;
#endif
	calibratorLaunchOnlyData();
};

/// \brief		Substitute for ROS live configuration adjustments
class calibratorRealtimeData : public calibratorSharedData, public calibratorRealtimeOnlyData {
protected:
    // ...

public:
	calibratorRealtimeData() { }
};

#endif
