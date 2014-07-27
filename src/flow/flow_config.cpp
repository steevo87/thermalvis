#include "../../include/flow/flow_config.hpp"

flowSharedData::flowSharedData() : 
	maxFeatures(300), 
	minFeatures(30),
	flowThreshold(0.0002),
	minSeparation(3.0),
	maxVelocity(200.0), 
	maxFrac(0.05),
	verboseMode(false),
	debugMode(false),
	adaptiveWindow(true), 
	velocityPrediction(true),  
	attemptHistoricalRecovery(true),
	autoTrackManagement(true),
	attemptMatching(false),
	showTrackHistory(false), 
	detectEveryFrame(false),
	newFeaturesPeriod(0.5), 
	delayTimeout(0.0),
	drawingHistory(25),
    matchingMode(MATCHING_MODE_LDA),
	sensitivity_1(DEFAULT_SENSITIVITY), 
	sensitivity_2(DEFAULT_SENSITIVITY), 
	sensitivity_3(DEFAULT_SENSITIVITY), 
	detector_1(DETECTOR_FAST), 
	detector_2(DETECTOR_OFF), 
	detector_3(DETECTOR_OFF), 
	multiplier_1(DEFAULT_MULTIPLIER_1), 
	multiplier_2(DEFAULT_MULTIPLIER_2)
{ }

flowRealtimeOnlyData::flowRealtimeOnlyData() :
	pauseMode(false)
{ }

flowLaunchOnlyData::flowLaunchOnlyData() :
#ifndef _BUILD_FOR_ROS_
	displayDebug(false),
	displayGUI(false),
#endif
	specialMode(false)
{ }
