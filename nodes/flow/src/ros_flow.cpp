#ifdef _BUILD_FOR_ROS_

#include "flow.hpp"

void mySigintHandler(int sig) {
	wantsToShutdown = true;
	displayMessage("Requested shutdown... terminating feeds...", MESSAGE_WARNING);
	
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

int featureTrackerNode::publish_tracks(ros::Publisher *pub_message, unsigned int latestIndex) {
	
	averageTrackLength = 0.0;
	
	int publishedTrackCount = 0;
	
	thermalvis::feature_tracks msg;

	msg.source = configData.topic;
	
	if (configData.autoTrackManagement) { trimFeatureTrackVector(); }
	
	vector<unsigned int> tracksToInclude;
	for (unsigned int iii = 0; iii < featureTrackVector.size(); iii++) {
		
		//if (featureTrackVector.at(iii).locations.at(featureTrackVector.at(iii).locations.size()-1).imageIndex == (frameCount-1)) {
		//if (featureTrackVector.at(iii).locations.at(featureTrackVector.at(iii).locations.size()-1).imageIndex == ((unsigned int) currentIndex)) {

			tracksToInclude.push_back(iii);
			publishedTrackCount++;
			averageTrackLength += featureTrackVector.at(iii).locations.size();
		//}
		
	}
	
	averageTrackLength /= ((double) publishedTrackCount);
	
	//ROS_INFO("Publishing tracks: featureTrackVector.size() = (%d), averageTrackLength = (%f)", featureTrackVector.size(), averageTrackLength);
	
	unsigned int projCount = 0;
	for (unsigned int iii = 0; iii < tracksToInclude.size(); iii++) {
		projCount += featureTrackVector.at(tracksToInclude.at(iii)).locations.size();
	}
	
	//ROS_WARN("Publishing (%d) projections from (%d) tracks", projCount, tracksToInclude.size());
	
	msg.projection_count = projCount;

	
	cv::Mat trackMatrix;
	
	if (configData.showTrackHistory) {
		if (createTrackMatrix(featureTrackVector, trackMatrix)) {
                    //HGH
                    //imshow("trackMatrix", trackMatrix);
                    imshow("trackMatrix : " + configData.topic, trackMatrix);
			cv::waitKey(1);
		}		
	}
	
	
	vector<unsigned int> currentIndices;
	vector<cv::Point2f> currentProjections;
	
	msg.cameras.clear();
	msg.indices.clear();
	msg.projections_x.clear();
	msg.projections_y.clear();
	
	for (unsigned int iii = 0; iii < tracksToInclude.size(); iii++) {
		
		//unsigned int startIndex = featureTrackVector.at(tracksToInclude.at(iii)).locations.size() - std::min(configData.maxProjectionsPerFeature, ((int)featureTrackVector.at(tracksToInclude.at(iii)).locations.size()));
		
		//for (unsigned int jjj = startIndex; jjj < featureTrackVector.at(tracksToInclude.at(iii)).locations.size(); jjj++) {
		for (unsigned int jjj = 0; jjj < featureTrackVector.at(tracksToInclude.at(iii)).locations.size(); jjj++) {
			
			//msg.indices.push_back(tracksToInclude.at(iii));
			msg.indices.push_back(featureTrackVector.at(tracksToInclude.at(iii)).trackIndex);
			msg.cameras.push_back(featureTrackVector.at(tracksToInclude.at(iii)).locations.at(jjj).imageIndex);
			
			if (featureTrackVector.at(tracksToInclude.at(iii)).locations.at(jjj).imageIndex == 0) {
				//ROS_ERROR("Publishing an invalid track! [%d] (%d, %d) - why does it exist??", iii, tracksToInclude.at(iii), jjj);
			}
			
			msg.projections_x.push_back(((double) featureTrackVector.at(tracksToInclude.at(iii)).locations.at(jjj).featureCoord.x));
			msg.projections_y.push_back(((double) featureTrackVector.at(tracksToInclude.at(iii)).locations.at(jjj).featureCoord.y));
		}
		
	}
	
	// Assign header info
	
	previousIndex = currentIndex;
	msg.header.seq = currentIndex;
	previous_time = original_time;
	msg.header.stamp = original_time;
	msg.header.frame_id = optical_frame;
	pub_message->publish(msg);
	
	return publishedTrackCount;
}

void featureTrackerNode::serverCallback(thermalvis::flowConfig &config, uint32_t level) {
      
    configData.minSeparation = config.minSeparation;
    
    configData.adaptiveWindow = config.adaptiveWindow;
    configData.autoTrackManagement = config.autoTrackManagement;
    configData.attemptMatching = config.attemptMatching;
    configData.maxVelocity = config.maxVelocity;
    
    configData.detectEveryFrame = config.detectEveryFrame;
    
    configData.matchingMode = config.matchingMode;
    
    configData.showTrackHistory = config.showTrackHistory;
      
    configData.sensitivity[0] = config.sensitivity_1;
	configData.sensitivity[1] = config.sensitivity_2;
	configData.sensitivity[2] = config.sensitivity_3;
	
	configData.velocityPrediction = config.velocityPrediction;
	
	// ROS_INFO("Sens = (%f, %f, %f)", configData.sensitivity[0], configData.sensitivity[1], configData.sensitivity[2]);
	
	if (config.maxFeatures > configData.maxFeatures) {
		previousTrackedPointsPeak = config.maxFeatures;
	}
	configData.maxFeatures = config.maxFeatures;
	
	
	configData.minFeatures = config.minFeatures;
	
	int det_array[3];
	det_array[0] = config.detector_1;
	det_array[1] = config.detector_2;
	det_array[2] = config.detector_3;
	
	configData.newFeaturesPeriod = config.newFeaturesPeriod;
	configData.attemptHistoricalRecovery = config.attemptHistoricalRecovery;
	
	if (configData.attemptHistoricalRecovery) {
		configData.multiplier[0] = 1;
		numHistoryFrames = 1;
	} else {
		configData.multiplier[0] = 0;
		numHistoryFrames = 0;
	}
	
	
	configData.multiplier[1] = config.multiplier_1;
	if ((numHistoryFrames == 1) && (configData.multiplier[1] > 0)) {
		numHistoryFrames++;
	}
	
	configData.multiplier[2] = config.multiplier_2;
	if ((numHistoryFrames == 2) && (configData.multiplier[2] > 0)) {
		numHistoryFrames++;
	}
	
	//ROS_INFO("Selected detectors: (%d, %d, %d)", config.detector_1, config.detector_2, config.detector_3);
	
	for (unsigned int iii = 0; iii < 3; iii++) {
		
		if (det_array[iii] == 1) {
			configData.detector[iii] = "GFTT";
		} else if (det_array[iii] == 2) {
			configData.detector[iii] = "FAST";
		} else if (det_array[iii] == 3) {
			configData.detector[iii] = "HARRIS";
		}
		
	}
	
	if (config.detector_1 > 0) {
		
		if (config.detector_2 > 0) {
			
			if (config.detector_3 > 0) {
				configData.numDetectors = 3;
				
			} else {
				configData.numDetectors = 2;
			}
			
		} else {
			configData.numDetectors = 1;
		}
		
	} else {
		configData.numDetectors = 0;
	}
    
    configData.verboseMode = config.verboseMode;
    
    
	configData.debugMode = config.debugMode;
	
	if ((configData.debugMode) && !debugInitialized) {
		
		// In order to prevent ROS from overriding sequence number
		//ros::AdvertiseOptions op_debug = ros::AdvertiseOptions::create<sensor_msgs::CameraInfo>(debug_pub_name, 1, &connected, &disconnected, ros::VoidPtr(), NULL);
		//op_debug.has_header = false;

		//debug_pub = it.advertise(op_debug);
		
		debug_pub = it->advertiseCamera(debug_pub_name, 1);
		
		debugInitialized = true;
		
		ROS_INFO("Advertising tracker debugging video (%s)", debug_pub_name);
	}
	
	configData.flowThreshold = config.flowThreshold;
	
	configData.maxFrac = config.maxFrac;
	
	configData.delayTimeout = config.delayTimeout;
	
	if (configData.delayTimeout == 0) {
		handleDelays = false;
	} else {
		handleDelays = true;
	}
	
	//configData.newDetectionFramecountTrigger = config.newDetectionFramecountTrigger;
	
	//configData.maxProjectionsPerFeature = config.maxProjectionsPerFeature;
	
	// TRACK RECOVERY - OLD
	/*
	configData.nearSearchHistory = config.nearSearchHistory;
	configData.farSearchHistory = config.farSearchHistory;
	configData.lossGaps[0] = configData.nearSearchHistory;
	configData.lossGaps[1] = configData.farSearchHistory;
	*/
	
	// TRACK RECOVERY - NEW
	
	
	
	
	
	configData.initializeDetectors(keypointDetector, &homographyDetector);
	
	configData.drawingHistory = config.drawingHistory;
	
	//configData.minPointsForWarning = config.minPointsForWarning;
	
}
#endif
