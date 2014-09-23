/*! \file	monoslam.cpp
 *  \brief	Definitions for the MONOSLAM node.
*/

#ifdef _FOR_REF_ONLY_

#include "slam/monoslam.hpp"

struct timeval cycle_timer;
//double elapsedTime;

int main(int argc, char** argv) {
	
	ROS_INFO("Node launched.");
	
	ros::init(argc, argv, "thermal_slam");
	
	ros::NodeHandle private_node_handle("~");
	
	slamData startupData;
		
	bool inputIsValid = startupData.obtainStartingData(private_node_handle);
	
	startupData.read_addr = argv[0];
	startupData.read_addr = startupData.read_addr.substr(0, startupData.read_addr.size()-12);
	
	if (!inputIsValid) {
		ROS_INFO("Configuration invalid.");
	}
		
	ROS_INFO("Startup data processed.");
	
	//ros::Rate loop_rate(25);
	
	ros::NodeHandle nh;
	
	boost::shared_ptr < slamNode > slam_node (new slamNode (nh, startupData));
	
	globalNodePtr = &slam_node;

	signal(SIGINT, mySigintHandler);
	
	ROS_INFO("Node configured.");
	
	ros::AsyncSpinner spinner(2);
	
	spinner.start();
	
	ros::waitForShutdown();
	
	ROS_INFO("Exiting.");
	
	return 0;
	
}

void mySigintHandler(int sig)
{
	wantsToShutdown = true;
	ROS_WARN("Requested shutdown... terminating feeds...");
	
	(*globalNodePtr)->prepareForTermination();
}

void slamNode::prepareForTermination() {
	return;
}





void slamNode::serverCallback(thermalvis::monoslamConfig &config, uint32_t level) {
    
    
    
    //ROS_WARN("Switched keyframe evaluation to (%d)", configData.keyframeEvaluationMode ? 1 : 0);
	
}

slamNode::slamNode(ros::NodeHandle& nh, slamData startupData) {
	
	
	
}

#endif