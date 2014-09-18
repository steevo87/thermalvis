/*! \file	monocular_slam.hpp
 *  \brief	Declarations for monocular slam.
*/

#ifndef _THERMALVIS_MONOCULAR_SLAM_H_
#define _THERMALVIS_MONOCULAR_SLAM_H_

#include "core/program.hpp"

#include "core/general_resources.hpp"
#include "core/ros_resources.hpp"
#include "core/opencv_resources.hpp"

#include "core/tracks.hpp"
	
#include "core/launch.hpp"

#ifdef _BUILD_FOR_ROS_
#include "slamConfig.h"
#endif

#include "slam_config.hpp"

/// \brief		Stores configuration information for the monocular slam routine
class slamData : public slamSharedData, public commonData, public slamLaunchOnlyData {
	friend class xmlParameters;
	friend class featureTrackerNode;
#ifndef _BUILD_FOR_ROS_
	friend class slamConfig;
#endif

protected:
	
	// ...

public:
	slamData();

	bool assignFromXml(xmlParameters& xP);

#ifdef _BUILD_FOR_ROS_
	bool obtainStartingData(ros::NodeHandle& nh);   
#endif	
};

#ifndef _BUILD_FOR_ROS_
/// \brief		Substitute for ROS live configuration adjustments
class slamConfig : public slamRealtimeData {
public:
	slamConfig() { }
	bool assignStartingData(slamData& startupData);
};
#endif

/// \brief		Manages the monocular slam procedure
class slamNode : public GenericOptions {
private:
	
#ifdef _BUILD_FOR_ROS_
	ros::NodeHandle private_node_handle;
	dynamic_reconfigure::Server<thermalvis::slamConfig> server;
	dynamic_reconfigure::Server<thermalvis::slamConfig>::CallbackType f;
#endif

	slamData configData;
	
public:

	vector<featureTrack> *featureTrackVector;

	///brief	Processes online changes in node configuration and applies them.
#ifdef _BUILD_FOR_ROS_
	void serverCallback(thermalvis::slamConfig &config, uint32_t level);
#else
	void serverCallback(slamConfig &config);
#endif

	///brief	Constructor. Must receive configuration data.
#ifdef _BUILD_FOR_ROS_
	slamNode(ros::NodeHandle& nh, slamData startupData);
#else
	slamNode(slamData startupData);
#endif

	///brief	Sets the debug mode - if true, images and data for visualization will be displayed to the user. 
	void setDebugMode(bool val) { 
		debugMode = val;
		configData.debugMode = val; 
	}

	bool slam_loop();
	
	///brief	Prepares node for termination.
	void prepareForTermination();
	
};

#endif
