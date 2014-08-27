/*! \file	streamer.cpp
 *  \brief	Definitions for the ROS <streamer> node.
*/

#ifdef _BUILD_FOR_ROS_

#include "streamer.hpp"

int main(int argc, char **argv) {

	ros::init(argc, argv, "streamer");
	
	ros::NodeHandle private_node_handle("~");
	
	streamerData startupData;
	
	startupData.read_addr = argv[0];
	startupData.read_addr = startupData.read_addr.substr(0, startupData.read_addr.size()-12);
	bool inputIsValid = startupData.obtainStartingData(private_node_handle);
	
	if (!inputIsValid) {
		ROS_ERROR("Configuration invalid!\n");
		return -1;
	}

	ros::NodeHandle nh;
		
	if (startupData.verboseMode) { ROS_INFO("Establishing streamer node..."); }
	boost::shared_ptr < streamerNode > streamer_node (new streamerNode (nh, startupData));
	
	globalNodePtr = &streamer_node;
	
	signal(SIGINT, mySigintHandler);
	
	//streamerNode streamer_node(nh, startupData);
	
	if (wantsToShutdown) {
		ros::shutdown();
		return 0;
	}
	
	if ((startupData.subscribeMode) || (startupData.resampleMode)) streamer_node->runBag();
	if (startupData.readMode) streamer_node->runRead();
	if (startupData.loadMode) streamer_node->runLoad();
	if ((startupData.captureMode) || (startupData.pollMode)) streamer_node->runDevice();
	
	ros::shutdown();

	
	return 0;
}

void mySigintHandler(int sig)
{
	ROS_INFO("Requested shutdown... terminating feeds...");
	wantsToShutdown = true;
	(*globalNodePtr)->prepareForTermination();
}

#endif
