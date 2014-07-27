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
    matchingMode(MATCHING_MODE_LDA)
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
