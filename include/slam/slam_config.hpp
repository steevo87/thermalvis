/*! \file	slam.hpp
 *  \brief	Declarations for slam node configuration.
*/

#ifndef THERMALVIS_SLAM_CONFIG_H_
#define THERMALVIS_SLAM_CONFIG_H_

#include <string>

#define INITIALIZATION_SCORING_PARAMETERS		5

// Hard Limits
#define MAX_INITIALIZATION_FRAMES				50
#define MAX_INITIALIZATION_SECONDS				5  
#define	MIN_INITIALIZATION_CONFIDENCE			0.9
#define MAX_FRAMES								1000
#define MAX_TRACKS 								10000
#define MAX_SEGMENTS							16
#define MAX_HISTORY								10
#define MAX_POSES_TO_STORE						100
#define MAX_TESTS_PER_FRAME						3

/// \brief		Parameters that are shared between both real-time update configuration, and program launch configuration for slam
struct slamSharedData {
	bool verboseMode, debugMode, timeDebug;
	bool debugSBA, debugTriangulation, publishPoints, publishKeyframes;

	double dataTimeout;
	int timeSpacing;
	double cameraLatency;
	double terminationTime, restartTime;
	double maxPoseDelay;

	// Initialization
	int maxInitializationFrames, minStartingSeparation, maxStartingSeparation, maxTestsPerFrame, maxInitializationSeconds;
	double minInitializationConfidence;
	int initialStructureIterations;
	

	// Operation
	int minTrackedFeatures, min3dPtsForPnP;
	int adjustmentFrames, flowback, framesForTriangulation, maxGuides;
	double motionThreshold, minGeometryScore, minKeyframeScore, requiredTrackFrac;
	int keyframeSpacing, maxKeyframeSeparation, camerasPerSys;
	int poseEstimateIterations, fullSystemIterations, subsequenceIterations, keyframeIterations;
	double baStep;
	double maxAllowableError;
	bool clearTriangulations;
	int pnpIterations;
	double inliersPercentage;
	bool trimFeatureTracks;
	int pairsForTriangulation, adjustmentIterations;
	double maxReprojectionDisparity, maxDistance, minSeparation, maxSeparation, maxStandardDev;

	slamSharedData();
};

/// \brief		Parameters that are only changeable from real-time interface
struct slamRealtimeOnlyData {
	bool pauseMode;
	slamRealtimeOnlyData();
};

/// \brief		Parameters that are only changeable from launch interface
struct slamLaunchOnlyData {
	bool specialMode, logErrors, keyframeEvaluationMode, writePoses, inspectInitialization;
	std::string initializationScorecard, evaluationFile, flowSource, mapperSource, extrinsicsFile, stream, errorFile;
	int baMode, evaluateParameters;
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
