/*! \file	streamer.cpp
 *  \brief	Definitions for the ROS <streamer> node.
*/

#ifdef _BUILD_FOR_ROS_

#include "streamer.hpp"

int main(int argc, char **argv) {
    ROS_INFO("streamer <main> entered.");

    ros::init(argc, argv, "streamer");
    ROS_INFO("streamer <init> completed.");

	ros::NodeHandle private_node_handle("~");
    ROS_INFO("streamer <NodeHandle> (private) created.");

	streamerData startupData;
    ROS_INFO("streamer <streamerData> created.");
	
	startupData.read_addr = argv[0];
    ROS_INFO("streamer <readaddr> set to (%s).", startupData.read_addr.c_str());

	bool inputIsValid = startupData.obtainStartingData(private_node_handle);
    ROS_INFO("streamer <inputIsValid> = (%s).", inputIsValid ? "true" : "false");

	startupData.setTerminationTrigger(&wantsToShutdown);
    if (startupData.verboseMode) { ROS_INFO("streamer <setTerminationTrigger> has been set."); }

	if (!inputIsValid) {
		ROS_ERROR("Configuration invalid!\n");
		return -1;
	}
    if (startupData.verboseMode) { ROS_INFO("streamer <inputIsValid> is true."); }

	ros::NodeHandle nh;
    if (startupData.verboseMode) { ROS_INFO("streamer <NodeHandle> created."); }
		
	if (startupData.verboseMode) { ROS_INFO("Establishing streamer node..."); }
	boost::shared_ptr < streamerNode > streamer_node (new streamerNode (nh, startupData));
    if (startupData.verboseMode) { ROS_INFO("Streamer node established."); }

	globalNodePtr = &streamer_node;
	
	signal(SIGINT, mySigintHandler);
	
	if (wantsToShutdown) {
		ros::shutdown();
		return 0;
	}
	
    if (startupData.verboseMode) { ROS_INFO("Streamer node about to run..."); }
	if ((startupData.subscribeMode) || (startupData.resampleMode)) streamer_node->runBag();
	if (startupData.readMode) streamer_node->runRead();
	if (startupData.loadMode) streamer_node->runLoad();
	if ((startupData.captureMode) || (startupData.pollMode)) streamer_node->runDevice();
	
    if (startupData.verboseMode) { ROS_INFO("Streamer node shutting down."); }
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
