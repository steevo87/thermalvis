/*! \file	calibrator_config.hpp
 *  \brief	Declarations for calibrator node configuration.
*/

#ifndef THERMALVIS_CALIBRATOR_CONFIG_HPP
#define THERMALVIS_CALIBRATOR_CONFIG_HPP

#include <string>

#define MAX_ALLOWABLE_CAMS		2
#define MAX_DETECTORS           10

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
	bool stopCapturingAtLimit;

	int maxPatterns, maxFrames;
	double maxTime;
	int setSize, maxCandidates, maxTests;
	double maxInterpolationTimegap, maxInterpolationMotion;
	double alpha;
	bool autoAlpha;
	bool trackingMode;
	double flowThreshold, errorThreshold, maxFrac;
	bool  invertPrimary, invertSecondary;

	// MSER settings
	bool adjustMSER[MAX_ALLOWABLE_CAMS];
	int delta[MAX_ALLOWABLE_CAMS];
	double maxVar[MAX_ALLOWABLE_CAMS], minDiv[MAX_ALLOWABLE_CAMS];
	double areaThreshold[MAX_ALLOWABLE_CAMS];
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
	std::string intrinsicsFiles[MAX_ALLOWABLE_CAMS];
	bool invert[MAX_ALLOWABLE_CAMS];
	std::string patternType, optMethod;
	int optimizationMethod;
	std::string video_stream, video_stream_secondary;
	int pattType;
	std::string calibType;
	bool wantsIntrinsics;
	std::string patternDetectionMode;
	unsigned int numCams;
	
	bool fixPrincipalPoint;
	bool fixIntrinsics;
	bool noConstraints;
	bool ignoreDistortion, useRationalModel;

    // Stuff copied from tracker... not sure why all of it is needed
    // stuff relating to publishing the tracks might be useful for estimating camera motion
    // from a calibration sequence though.
    int numDetectors;
    std::string tracksOutputTopic;
    bool outputFeatureMotion, normalizeFeatureVelocities, outputTrackCount;
    std::string detector[MAX_DETECTORS], descriptor[MAX_DETECTORS];
    double sensitivity[MAX_DETECTORS];
    std::string method[MAX_DETECTORS];
    bool method_match[MAX_DETECTORS];

	// Course settings
	bool useUndistortedLocations;
	
	bool removeSpatialBias;
	
	// Fine settings
	double maxTimeDiff;
	
	
	
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
