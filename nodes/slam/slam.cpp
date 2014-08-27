/*! \file	slam.cpp
 *  \brief	Definitions for the ROS <slam> node.
*/

#include "slam.hpp"

int main(int argc, char** argv) {
	
	ROS_INFO("Node launched.");
	ros::init(argc, argv, "slam");
	ros::NodeHandle private_node_handle("~");
	slamData startupData;
	bool inputIsValid = startupData.obtainStartingData(private_node_handle);
	
	startupData.read_addr = argv[0];
	
	if (!inputIsValid) {
		ROS_ERROR("Configuration invalid.");
		return -1;
	}
		
	ROS_INFO("Startup data processed.");
	ros::NodeHandle nh;
	boost::shared_ptr < slamNode > slam_node (new slamNode (nh, startupData));
	globalNodePtr = &slam_node;
	signal(SIGINT, mySigintHandler);
	ROS_INFO("Node configured.");
	while (!::wantsToShutdown) ros::spinOnce();
	return 0;
}

bool slamData::obtainStartingData(ros::NodeHandle& nh) {
	
	return true;
}

void mySigintHandler(int sig) {
	wantsToShutdown = true;
	ROS_WARN("Requested shutdown, terminating feeds...");
	(*globalNodePtr)->prepareForTermination();
}
