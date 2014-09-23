/*! \file	videoslam.cpp
 *  \brief	Definitions for the VIDEOSLAM node.
*/

#ifdef _FOR_REF_ONLY_

#include "slam/videoslam.hpp"

struct timeval cycle_timer;
//double elapsedTime;

int main(int argc, char** argv) {	
	
	ROS_INFO("Node launched.");
	
	ros::init(argc, argv, "videoslam");
	
	ros::NodeHandle private_node_handle("~");
	
	videoslamData startupData;
		
	bool inputIsValid = startupData.obtainStartingData(private_node_handle);
	
	startupData.read_addr = argv[0];
	startupData.read_addr = startupData.read_addr.substr(0, startupData.read_addr.size()-12);
	
	if (!inputIsValid) {
		ROS_INFO("Configuration invalid.");
	}
		
	ROS_INFO("Startup data processed.");
	
	//ros::Rate loop_rate(25);
	
	ros::NodeHandle nh;
	
	ROS_INFO("About to create shared pointer..");
	boost::shared_ptr < videoslamNode > videoslam_node (new videoslamNode (nh, startupData));
	ROS_INFO("Shared pointer created.");
	
	globalNodePtr = &videoslam_node;

	signal(SIGINT, mySigintHandler);
	
	ROS_INFO("Node configured.");
	
	ros::AsyncSpinner spinner(2);
	
	spinner.start();
	
	while (!wantsToShutdown) { };
	
	mySigintHandler(1);
	//ros::waitForShutdown();
	ros::shutdown();
	
	ROS_INFO("Exiting.");
	
	return 0;
	
}

bool videoslamData::obtainStartingData(ros::NodeHandle& nh) {
		
	
	
	return true;
}

void mySigintHandler(int sig)
{
	wantsToShutdown = true;
	ROS_WARN("Requested shutdown... terminating feeds...");

}

void videoslamNode::main_loop(const ros::TimerEvent& event) {
	
	// main_mutex.lock();
	// main_mutex.unlock();
	
	// ROS_INFO("Entered main loop...");
	
	// addFixedCamera(display_sys, configData.cameraData, eye4);
			
	// pose_pub.publish(currentPose);
	
}



void videoslamNode::serverCallback(thermalvis::videoslamConfig &config, uint32_t level) {
    
    
    
	
}

videoslamNode::videoslamNode(ros::NodeHandle& nh, videoslamData startupData) {
	
	ROS_INFO("X..");
	
	configData = startupData;
	
	
	
	if (configData.verboseMode) { ROS_INFO("Initializing node.."); }
	
	sprintf(nodeName, "%s", ros::this_node::getName().c_str());
	
	
	
	ROS_INFO("Establishing server callback...");
	f = boost::bind (&videoslamNode::serverCallback, this, _1, _2);
    server.setCallback (f);
	
}

#endif