#ifdef _BUILD_FOR_ROS_

#include "flow.hpp"

#define NODE_NAME "flow"

int main(int argc, char** argv) {
	
		
	ros::init(argc, argv, NODE_NAME);
	
	ros::NodeHandle private_node_handle("~");
	
	trackerData startupData;
		
	bool inputIsValid = startupData.obtainStartingData(private_node_handle);
	
	startupData.read_addr = argv[0];
	ROS_INFO("<%s> Node launched from (%s)", NODE_NAME, startupData.read_addr.c_str());
	
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

void mySigintHandler(int sig) {
	wantsToShutdown = true;
	ROS_WARN("Requested shutdown... terminating feeds...");
	
	(*globalNodePtr)->prepareForTermination();
}

bool trackerData::obtainStartingData(ros::NodeHandle& nh) {
	
	numDetectors = 0;
	
	nh.param<string>("outputFolder", outputFolder, "outputFolder");
	
	nh.param<std::string>("topic", topic, "topic");
	
	if (topic != "topic") {
		ROS_INFO("<topic> (%s) selected.", topic.c_str());
	} else {
		topic = "/thermalvis/streamer/image_mono";
		ROS_WARN("No <topic> supplied; assuming (%s)", topic.c_str());
		//return false;
	}
	
	topicParent = topic.substr(0, topic.find_last_of("/"));
	
	ROS_INFO("topicParent = (%s)", topicParent.c_str());
	
	nh.param<string>("tracksOutputTopic", tracksOutputTopic, "tracksOutputTopic");
	
	if (tracksOutputTopic != "tracksOutputTopic") {
		ROS_INFO("<tracksOutputTopic> (%s) selected.", tracksOutputTopic.c_str());
	} else {
		ROS_WARN("No <tracksOutputTopic> supplied; will publish <tracks> at <%s>", topicParent.c_str());
		//return false;
	}
	
	//nh.param<int>("maxFeatures", maxFeatures, 100);
	
	

	nh.param<bool>("outputFeatureMotion", outputFeatureMotion, false);
	nh.param<bool>("normalizeFeatureVelocities", normalizeFeatureVelocities, true);

	
	//nh.param<bool>("debug_mode", debugMode, false);
	
	nh.param<bool>("outputTrackCount", outputTrackCount, false);
	
        //HGH/*
	
	for (int iii = 0; iii < MAX_DETECTORS; iii++) {
		
		char detector_tag[256], sensitivity_tag[256], method_tag[256], descriptor_tag[256];
		
		sprintf(detector_tag, "detector_%d", iii + 1);
		sprintf(sensitivity_tag, "sensitivity_%d", iii + 1);
		sprintf(method_tag, "method_%d", iii + 1);
		sprintf(descriptor_tag, "descriptor_%d", iii + 1);
		
		//printf("%s << detector_tag = %s\n", __FUNCTION__, detector_tag);
		
		
		if (iii == 0) {
			nh.param<std::string>(detector_tag, detector[iii], "GFTT");
		} else {
			nh.param<std::string>(detector_tag, detector[iii], "NONE");
		}
		
		//printf("%s << detector[%d] = %s\n", __FUNCTION__, iii, detector[iii].c_str());
		
		if ((detector[iii] == "SURF") || (detector[iii] == "FAST") || (detector[iii] == "GFTT") || (detector[iii] == "HARRIS") || (detector[iii] == "ORB") || (detector[iii] == "STAR")) {
			ROS_INFO("detector [%d]: %s", iii, detector[iii].c_str());
		} else if (detector[iii] == "NONE"){
			// printf("%s << No [%d] detector requested.\n", __FUNCTION__, iii);
			break;
		} else {
			ROS_INFO("ERROR! Detector (%s) not recognized.", detector[iii].c_str());
			return false;
		}
		
		nh.param<double>(sensitivity_tag, sensitivity[iii], 0.2);
		
		ROS_INFO("detector [%d] sensitivity: %f", iii, sensitivity[iii]);
		
		nh.param<std::string>(method_tag, method[iii], "match");
		
		if ((method[iii] == "match") || (method[iii] == "track")) {
			ROS_INFO("detector [%d] matching method: %s", iii, method[iii].c_str());
			if (method[iii] == "match") {
				method_match[iii] = true;
			} else {
				method_match[iii] = false;
			}
		} else {
			ROS_INFO("ERROR! detector [%d] matching method (%s) not recognized.", iii, method[iii].c_str());
			return false;
		}
		
		nh.param<std::string>(descriptor_tag, descriptor[iii], "BRIEF");
		
		if (method_match[iii]) {
			if ((descriptor[iii] == "SURF") || (descriptor[iii] == "ORB") || (descriptor[iii] == "BRIEF") || (descriptor[iii] == "SIFT")) {
				ROS_INFO("descriptor [%d]: %s", iii, descriptor[iii].c_str());
			} else {
				ROS_INFO("ERROR! descriptor [%d] (%s) not recognized.", iii, descriptor[iii].c_str());
				return false;
			}
		}
		
		numDetectors++;
	}
	
	
	
	ROS_INFO("[%d] detectors to be implemented.", numDetectors);
	
        //*/
	
	return true;
}

#endif
