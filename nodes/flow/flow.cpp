#ifdef _BUILD_FOR_ROS_

#include "flow.hpp"

void mySigintHandler(int sig) {
	wantsToShutdown = true;
	ROS_WARN("Requested shutdown... terminating feeds...");
	
	(*globalNodePtr)->prepareForTermination();
}

int main(int argc, char** argv) {
	
	ROS_INFO("Node launched.");
		
	ros::init(argc, argv, "flow");
	
	ros::NodeHandle private_node_handle("~");
	
	trackerData startupData;
		
	bool inputIsValid = startupData.obtainStartingData(private_node_handle);
	
	startupData.read_addr = argv[0];
	startupData.read_addr = startupData.read_addr.substr(0, startupData.read_addr.size()-8);
	
	if (!inputIsValid) {
		ROS_ERROR("Configuration invalid.");
		return -1;
	}
		
	ROS_INFO("Startup data processed.");
	
	ros::NodeHandle nh;
	
	boost::shared_ptr < featureTrackerNode > tracker_node (new featureTrackerNode (nh, startupData));
	
	globalNodePtr = &tracker_node;
	
	signal(SIGINT, mySigintHandler);
	
	ROS_INFO("Node configured.");
	
	while (!::wantsToShutdown) {
		ros::spinOnce();
	}
	
	
	return 0;
	
}

#endif
