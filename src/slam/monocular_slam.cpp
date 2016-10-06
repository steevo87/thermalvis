/*! \file	monocular_slam.cpp
 *  \brief	Definitions for monocular slam.
*/

#include "slam/monocular_slam.hpp"

#define NULL_SCORE    0.25*numeric_limits<double>::max()

#ifdef _USE_OPENCV_VIZ_
void keyboard_callback (const cv::viz::KeyboardEvent &e, void *cookie) {
	
	keyboardCommands* kC = reinterpret_cast<keyboardCommands*> (cookie);

	unsigned char key = e.code;

	// Known reserved letters:
	// Q (exit) - causes crash
  
	switch (key) {
		case (int)'t': case (int)'T': kC->toggleCamera = true; break;
		case (int)'e': case (int)'E': kC->exit = true; break;
		default: break;
	}    
}
#endif

#ifndef _BUILD_FOR_ROS_
bool slamConfig::assignStartingData(slamData& startupData) {

	verboseMode = startupData.verboseMode;
	debugMode = startupData.debugMode;
	
	return true;
}
#endif

slamData::slamData()
{
	
}

#ifdef _USE_BOOST_ 
#ifndef _BUILD_FOR_ROS_
bool slamData::assignFromXml(xmlParameters& xP) 
{

	int countOfNodes = 0;

	BOOST_FOREACH(boost::property_tree::ptree::value_type &v, xP.pt.get_child("launch")) {
		if (v.first.compare("node")) continue; // only progresses if its a "node" tag
		if (!v.second.get_child("<xmlattr>.type").data().compare("slam")) countOfNodes++;
	}

	if (countOfNodes == 0) 
  {
		ROS_ERROR("No relevant nodes found in XML config!");
		return false;
	}

	if (countOfNodes > 1) 
  {
		ROS_ERROR("More than 1 relevant node found in XML config! This functionality is not supported in Windows..");
		return false;
	}

	BOOST_FOREACH(boost::property_tree::ptree::value_type &v, xP.pt.get_child("launch")) { // Within tree (pt), finds launch, and loops all tags within it
		if (v.first.compare("node")) continue;
		if (v.second.get_child("<xmlattr>.type").data().compare("slam")) {
			if ((!v.second.get_child("<xmlattr>.type").data().compare("reconfigure_gui")) || (!v.second.get_child("<xmlattr>.type").data().compare("rqt_reconfigure"))) {
				if (!v.second.get_child("<xmlattr>.args").data().compare("slam")) displayGUI = true;
				if (!v.second.get_child("<xmlattr>.args").data().compare("/slam")) displayGUI = true;
			}
			continue;
		}

		BOOST_FOREACH(boost::property_tree::ptree::value_type &v2, v.second) { // Traverses the subtree...
			if (v2.first.compare("param")) continue;

			if (!v2.second.get_child("<xmlattr>.name").data().compare("debugMode")) debugMode = !v2.second.get_child("<xmlattr>.value").data().compare("true");
			if (!v2.second.get_child("<xmlattr>.name").data().compare("verboseMode")) verboseMode = !v2.second.get_child("<xmlattr>.value").data().compare("true");
			if (!v2.second.get_child("<xmlattr>.name").data().compare("inspectInitialization")) inspectInitialization = !v2.second.get_child("<xmlattr>.value").data().compare("true");

			if (!v2.second.get_child("<xmlattr>.name").data().compare("maxInitializationFrames")) maxInitializationFrames = atoi(v2.second.get_child("<xmlattr>.value").data().c_str());
        }

		// Substitute tildes if in Windows
#ifdef _WIN32
    CleanAndSubstitutePath( outputFolder );
#endif

	}

	return true;

}
#endif
#endif

#ifdef _BUILD_FOR_ROS_
void slamNode::serverCallback(thermalvis::slamConfig &config, uint32_t level) {
#else
void slamNode::serverCallback(slamConfig &config) {
#endif

	// from monoslam
	configData.flowback = config.flowback;
    configData.verboseMode = config.verboseMode;
    configData.timeSpacing = config.timeSpacing;
    configData.poseEstimateIterations = config.poseEstimateIterations;
    configData.minInitializationConfidence = config.minInitializationConfidence;
    configData.adjustmentFrames = config.adjustmentFrames;
    configData.motionThreshold = config.motionThreshold;
    configData.timeDebug = config.timeDebug;
    configData.keyframeIterations = config.keyframeIterations;
    configData.framesForTriangulation = config.framesForTriangulation;
    configData.maxKeyframeSeparation = config.maxKeyframeSeparation;
    configData.min3dPtsForPnP = config.min3dPtsForPnP;
    configData.camerasPerSys = config.camerasPerSys;
    configData.minTrackedFeatures = config.minTrackedFeatures;
    configData.minGeometryScore = config.minGeometryScore;
    configData.minKeyframeScore = config.minKeyframeScore;
    configData.requiredTrackFrac = config.requiredTrackFrac;
    configData.fullSystemIterations = config.fullSystemIterations;
    configData.subsequenceIterations = config.subsequenceIterations;
    configData.keyframeSpacing = config.keyframeSpacing;
    configData.maxGuides = config.maxGuides;
    configData.initialStructureIterations = config.initialStructureIterations;

	// From videoslam
	configData.cameraLatency = config.cameraLatency;
    configData.adjustmentFrames = config.adjustmentFrames;
    configData.debugMode = config.debugMode;
    configData.pairsForTriangulation = config.pairsForTriangulation;
    configData.dataTimeout = config.dataTimeout;
    configData.adjustmentIterations = config.adjustmentIterations;
    configData.verboseMode = config.verboseMode;
    configData.maxDistance = config.maxDistance;
    configData.minSeparation = config.minSeparation;
    configData.maxSeparation = config.maxSeparation;
    configData.maxStandardDev = config.maxStandardDev;
    configData.debugSBA = config.debugSBA;
    configData.debugTriangulation = config.debugTriangulation;
    configData.trimFeatureTracks = config.trimFeatureTracks;
    //configData.baMode = config.baMode;
    configData.baStep = config.baStep;
    configData.maxAllowableError = config.maxAllowableError;
    configData.pnpIterations = config.pnpIterations;
    configData.inliersPercentage = config.inliersPercentage;
    
    configData.maxReprojectionDisparity = config.maxReprojectionDisparity;

}

bool slamNode::updatePotentialInitializationFrames() {
	// This is based on minimum amount of feature motion from ALL other collected frames in the potential startup set. 
	// This will be used to prevent testing of frames that are very similar to existing ones, which is particularly important if the 
	// camera is staying in the same place, or undergoing a very limited motion that is not providing enough viewpoint variation. 
	// In this case, if you don't ignore redundant frames, you risk "randomly" selecting potential frames from a very long frame history 
	// that is near-identical. You did something similar for the videoslam keyframe selection system, right?

	if (initialization_store.keyframes.size() == 0) {
        cv::Mat dummyMat;
        initialization_store.addKeyframe(latestFrame, dummyMat);
		return false;
	}

	if (sufficientMotionForInitializationFrame()) {

		// May also want to hard-code a maximum number of these potential frames to store for ongoing structure initialization tests, 
		// so that even if motion is sufficient, the oldest test frames (except if they constitute the top scoring pair so far) drop 
		// off as new ones are accumulated - especially since it is likely that they don't contain common points anyway.
		int idx = 0;
		while (initialization_store.keyframes.size() >= MAX_INITIALIZATION_CANDIDATES) {
			if (initialization_store.keyframes.at(idx).idx == bestInitializationIndices[0]) {
				idx++;
				continue;
			} else if (initialization_store.keyframes.at(idx).idx == bestInitializationIndices[1]) {
				idx++;
				continue;
			}
			initialization_store.keyframes.erase(initialization_store.keyframes.begin()+idx);
		}

        cv::Mat dummyMat;
        initialization_store.addKeyframe(latestFrame, dummyMat);
		return true;
	}

	return false;
}

bool slamNode::sufficientMotionForInitializationFrame() {

	// For each frame already in the initialization store...
	for (unsigned int iii = 0; iii < initialization_store.keyframes.size(); iii++) {
		// Find all common features shared with the current frame...
		vector<unsigned int> shared_track_indices;
		getActiveTracks(shared_track_indices, *featureTrackVector, initialization_store.keyframes.at(iii).idx, latestFrame);

		double featureMotion = getFeatureMotion(*featureTrackVector, shared_track_indices, initialization_store.keyframes.at(iii).idx, latestFrame);
		if (featureMotion < MIN_FEATURE_MOTION_THRESHOLD) return false;
	}
	
	return true;
}

void slamNode::testInitializationWithCurrentFrame() {

	// For real-time implementation, for each new potential keyframe it does as many tests with random preceding potential frames 
	// as it can fit in until next frame arrives.
	
	// For offline implementation, should for the moment do a fixed number of maximum tests, based on what is not too sluggish.

	double keyframe_scores[5];
	cv::Mat startingTrans;
	cv::Mat blankMat = cv::Mat::zeros(80, 640, CV_8UC3);
	
	vector<int> startersToTest;
	for (unsigned int iii = 0; iii < initialization_store.keyframes.size()-1; iii++) startersToTest.push_back(initialization_store.keyframes.at(iii).idx);
		
	while (((int)startersToTest.size()) > configData.maxTestsPerFrame) {
		unsigned int randIndex = rand() % startersToTest.size();
		startersToTest.erase(startersToTest.begin() + randIndex);
	}
		
	for (unsigned int iii = 0; iii < startersToTest.size(); iii++) {
			
		vector<unsigned int> activeTracks;
		getActiveTracks(activeTracks, *featureTrackVector, startersToTest.at(iii), latestFrame);

		startingTrans = cv::Mat();
		double score = testKeyframePair(*featureTrackVector, configData.cameraData, scorecardParams, startersToTest.at(iii), latestFrame, keyframe_scores, startingTrans, true /*configData.keyframeEvaluationMode*/, true);
				
		ROS_INFO("Keyframe pair (%03d, %03d) initialization score = [%f] {%1.2f, %1.2f, %1.2f, %1.2f, %1.2f}", startersToTest.at(iii), latestFrame, score, keyframe_scores[0], keyframe_scores[1], keyframe_scores[2], keyframe_scores[3], keyframe_scores[4]);

		startupVector_score.push_back(score);
		startupVector_idx1.push_back(startersToTest.at(iii));
		startupVector_idx2.push_back(latestFrame);
		startupVector_trans.push_back(startingTrans);
	}
}

bool slamNode::selectBestInitializationPair() {

	int best_idx1 = -1, best_idx2 = -1;
	cv::Mat best_trans;

	double bestScore = -1.0;
	for (unsigned int iii = 0; iii < startupVector_score.size(); iii++) {
		if (startupVector_score.at(iii) > bestScore) {
			bestScore = startupVector_score.at(iii);
			best_idx1 = startupVector_idx1.at(iii);
			best_idx2 = startupVector_idx2.at(iii);
			best_trans = startupVector_trans.at(iii);
		}
	}

	if (bestScore <= 0.0) {
		ROS_ERROR("No suitable startup pair was found, so structure cannot be initialized.");
		return false;
	}

	ROS_INFO("Best startup pair found: score = (%f), indices = (%d, %d)", bestScore, best_idx1, best_idx2);
	assignStartingFrames(best_idx1, best_idx2, best_trans);
	return true;
}

#ifdef _BUILD_FOR_ROS_
void slamNode::main_loop(const ros::TimerEvent& event) {
#else
void slamNode::main_loop(sensor_msgs::CameraInfo& info_msg, const vector<featureTrack>& msg) {
#endif

#ifdef _BUILD_FOR_ROS_	
	if (!infoProcessed) return;
#else
	if (!infoProcessed) handle_info(&info_msg);
	handle_tracks(msg);
	ROS_INFO("featureTrackVector->size() = (%zu)", featureTrackVector->size());
#endif
	
	if (configData.keyframeEvaluationMode && evaluationCompleted) return;
	
	if (firstIteration) {
#ifdef _USE_BOOST_
		main_mutex.lock();
		update_display();
		main_mutex.unlock();
#endif
		firstIteration = false;
	}
	
	

	if (configData.keyframeEvaluationMode) {
		if (latestFrame >= configData.maxInitializationFrames) {
#ifdef _USE_BOOST_
			main_mutex.lock();
			performKeyframeEvaluation();
			main_mutex.unlock();
#endif
			wantsToShutdown = true;
		}
		return;
	}

	if (!structureFormed) {
		
		if (updatePotentialInitializationFrames()) testInitializationWithCurrentFrame();

		if ((latestFrame >= configData.maxInitializationFrames) || (bestInitializationScore >= configData.minInitializationConfidence)) { // elapsedTime > configData.maxInitializationSeconds
			// select best pair
			if (!selectBestInitializationPair()) return;
#ifdef _USE_BOOST_
			main_mutex.lock();
			structureFormed = formInitialStructure();
			putativelyEstimatedFrames = currentPoseIndex-1;
			main_mutex.unlock();
#endif
		}

		if (!structureFormed) return;
	} else {
		//while ((currentPoseIndex < latestFrame) && (keyframe_store.keyframes.size() <= 7)) {
		while (currentPoseIndex < latestFrame) {
#ifdef _USE_BOOST_
			main_mutex.lock();
			processNextFrame();
			main_mutex.unlock();
#endif
		}
	}
		
	
}

#ifdef _BUILD_FOR_ROS_
slamNode::slamNode(ros::NodeHandle& nh, slamData startupData) : 
#else
slamNode::slamNode(slamData startupData) : 
#endif
	evaluationCompleted(false),
	putativelyEstimatedFrames(0),
	firstIteration(true),
	repetitionNoted(false),
	currentPoseIndex(-1),
	isTracking(true),
	baseConnectionNum(0),
	lastBasePose(-1),
	latestFrame(-1),
	bestInitializationScore(0.0),
	structureFormed(false), 
	infoProcessed(false),
	framesArrived(0),
	framesProcessed(0),
	pnpSuccesses(0),
	baSuccesses(0),
	baAverage(0.0),
	dsAverage(0.0),	
	hasTerminatedFeed(false),	
	latestReceivedPoseProcessed(false),
	extrinsicCalib_R(cv::Mat::eye(3,3,CV_64FC1)),
	extrinsicCalib_T(cv::Mat::zeros(3,1,CV_64FC1)),
	current_view_index(-1)
{
	configData = startupData;

	featureTrackVector = new std::vector<featureTrack>;

	bestInitializationIndices[0] = -1;
	bestInitializationIndices[1] = -1;

	scorecardParams = new double*[INITIALIZATION_SCORING_PARAMETERS];
	
	for (int iii = 0; iii < INITIALIZATION_SCORING_PARAMETERS; iii++) scorecardParams[iii] = new double[3];
	processScorecard();
	
#ifdef _BUILD_FOR_ROS_
	sprintf(nodeName, "%s", ros::this_node::getName().c_str());
#endif

#ifdef _IS_WINDOWS_
	srand((unsigned int)time(NULL));
#else
	srand(time(NULL));
#endif
	
#ifdef _USE_BOOST_
	boost::mutex cam_mutex;
	boost::mutex tracks_mutex;
	boost::mutex keyframes_mutex;
#endif

	char timeString[256];
	
    sprintf(timeString, "%010d.%09d", int(ros::Time::now().sec), int(ros::Time::now().nsec));
	//stringstream convert;
	//convert << ros::Time::now().sec << "." << ros::Time::now().nsec;
	
	//configData.evaluationFile = configData.read_addr + "nodes/monoslam/log/" + convert.str() + ".txt";
#ifdef _BUILD_FOR_ROS_
	configData.evaluationFile = configData.read_addr + "nodes/monoslam/log/" + timeString + "-" + ros::this_node::getName().substr(1,ros::this_node::getName().size()-1) + ".txt";
#else
	configData.evaluationFile = configData.read_addr + "nodes/monoslam/log/" + timeString + "-" + "slam" + ".txt";
#endif

	if (configData.keyframeEvaluationMode) ROS_INFO("evaluationFile = (%s)", configData.evaluationFile.c_str());
	
#ifdef _USE_SBA_
	display_sys.tracks.clear();
	display_sys.nodes.clear();
	sys.verbose = 0;
	display_sys.verbose = 0;
#endif

	eye4 = cv::Mat::eye(4, 4, CV_64FC1);
	
#ifdef _BUILD_FOR_ROS_
	ROS_INFO("Setting up node.");
	path_pub = nh.advertise<visualization_msgs::Marker>( "path", 1 );
	camera_pub = nh.advertise<visualization_msgs::Marker>( "cameras", 1 );
	points_pub = nh.advertise<visualization_msgs::Marker>( "points", 1);
	
	std::string topic_tracks = configData.stream + "tracks";
	
	ROS_INFO("Connecting to tracks topic. %s", topic_tracks.c_str());
    tracks_sub = nh.subscribe(topic_tracks, 1, &slamNode::handle_tracks, this); // <thermalvis::feature_tracks>
	
	std::string topic_info = configData.stream + "camera_info";
	
	ROS_INFO("Connecting to camera_info. %s", topic_info.c_str());
	
	info_sub = nh.subscribe<sensor_msgs::CameraInfo>(topic_info, 1, &slamNode::handle_info, this);

	ROS_INFO("Node setup.");

	sprintf(pose_pub_name, "thermalvis%s/pose", nodeName);
	ROS_INFO("Configuring pose topic. %s", pose_pub_name);
	
	currentPose.header.frame_id = "/pgraph"; //pose_pub_name;
	pose_pub = nh.advertise<geometry_msgs::PoseStamped>(pose_pub_name, 1);
	
	timer = nh.createTimer(ros::Duration(0.01), &slamNode::main_loop, this);

	if (configData.logErrors) {
		configData.errorFile = configData.read_addr + "nodes/monoslam/log/stability/" + timeString + "-" + ros::this_node::getName().substr(1,ros::this_node::getName().size()-1) + ".txt";
		ROS_INFO("Current debug filename: %s", configData.errorFile.c_str());
		error_file.open(configData.errorFile.c_str());
	}

#endif
	
	keyframeTestScores = cv::Mat::zeros(configData.maxInitializationFrames, configData.maxInitializationFrames, CV_64FC1);
	keyframeTestFlags = cv::Mat::zeros(configData.maxInitializationFrames, configData.maxInitializationFrames, CV_8UC1);
	

	// from videoslam:

	if (configData.writePoses) {
		// http://stackoverflow.com/questions/8478851/suppressing-cout-output-with-in-a-function
		ROS_WARN("writePoses == true, therefore suppressing cout messages from SBA libraries..");
		lStream.open( "garbage.txt" );
		lBufferOld = std::cout.rdbuf();
	}
	
	
	
	
	currentPose.pose.position.x = std::numeric_limits<float>::max();
	
	
	
	if ((configData.extrinsicsFile != "extrinsicsFile") && (configData.extrinsicsFile != "")) {
		
		if (configData.verboseMode) { ROS_INFO("Reading extrinsics file (%s)...", configData.extrinsicsFile.c_str()); }
		
		try {
			cv::FileStorage fs(configData.extrinsicsFile, cv::FileStorage::READ);
			fs["R1"] >> extrinsicCalib_R;
			fs["T1"] >> extrinsicCalib_T;
			fs.release();

			ROS_INFO("Extrinsics data read.");

			if (extrinsicCalib_R.empty()){
				ROS_ERROR("Extrinsics file (%s) invalid! Please check path and filecontent...", configData.extrinsicsFile.c_str());
				wantsToShutdown = true;
			}

		} catch (...) {
			ROS_ERROR("Some failure in reading in the extrinsics (%s).", configData.extrinsicsFile.c_str());
			wantsToShutdown = true;
		}
	} else {
		if (configData.verboseMode) { ROS_INFO("No extrinsics provided."); }
	}
	
	//matrixToQuaternion(extrinsicCalib_R, extrinsicCalib_quat);
	composeTransform(extrinsicCalib_R, extrinsicCalib_T, extrinsicCalib_P);
	
	if (configData.verboseMode) { ROS_INFO("Transforms complete."); }
	
	// To Apply
	// actualThermal_quat = extrinsicCalib_quat * virtualDepth_quat;
	// actualThermal_P = virtualDepth_P * extrinsicCalib_P;
	
	frameProcessedCounter = 0;
	frameHeaderHistoryCounter = 0;
	poseHistoryCounter = 0;
	
	//latestHandledTracks = -1;
	
	distanceTravelled = 0.0;
	
	storedPosesCount = 0;
	
#ifdef _USE_SBA_
	sys.verbose = 0;
#endif

	eye4 = cv::Mat::eye(4, 4, CV_64FC1);
	
	lastTestedFrame = -1;
	
	if (configData.verboseMode) { ROS_INFO("Initializing topics.."); }
	
	/*
	char path_pub_name[256];
	sprintf(path_pub_name, "/thermalvis%s/path", nodeName);
	path_pub = nh.advertise<visualization_msgs::Marker>( path_pub_name, 1 );
	*/
	
#ifdef _BUILD_FOR_ROS_
	char camera_pub_name[256];
	sprintf(camera_pub_name, "/thermalvis%s/cameras", nodeName);
	camera_pub = nh.advertise<visualization_msgs::Marker>( camera_pub_name, 1 );
	
	char points_pub_name[256];
	sprintf(points_pub_name, "/thermalvis%s/points", nodeName);
	points_pub = nh.advertise<sensor_msgs::PointCloud2>(points_pub_name, 1);
	
	char confidence_pub_name[256];
	sprintf(confidence_pub_name, "/thermalvis%s/confidence", nodeName);
	
	//confidence_pub  = nh.advertise<std_msgs::Float32>(confidence_pub_name, 1);
	
	ROS_INFO("Publishing confidence data at (%s)", confidence_pub_name);
	ros::AdvertiseOptions op = ros::AdvertiseOptions::create<thermalvis::pose_confidence>(confidence_pub_name, 1, &connected, &disconnected, ros::VoidPtr(), NULL);
	op.has_header = false;
	confidence_pub = nh.advertise(op);
	
	publishPoints(ros::Time::now(), 0);

	ROS_INFO("Setting up node.");
	
	infoProcessed = false;
	
	ROS_INFO("Connecting to tracks topic. %s", topic_tracks.c_str());
    tracks_sub = nh.subscribe(topic_tracks, 1, &slamNode::handle_tracks, this); // <thermalvis::feature_tracks>
	
	ROS_INFO("Connecting to camera_info. %s", topic_info.c_str());
    info_sub = nh.subscribe<sensor_msgs::CameraInfo>(topic_info, 1, &slamNode::handle_info, this);
	
	std::string topic_pose = configData.mapperSource + "pose";
	ROS_INFO("Connecting to pose topic. %s", topic_pose.c_str());
	pose_sub = nh.subscribe(topic_pose, 1, &slamNode::handle_pose, this);
	
	ROS_INFO("Node setup.");

	sprintf(pose_pub_name, "/thermalvis%s/pose", nodeName);
	ROS_INFO("Configuring pose topic. %s", pose_pub_name);
#endif	
	currentPose.header.frame_id = "/world"; //pose_pub_name;
	
	//pose_pub = nh.advertise<geometry_msgs::PoseStamped>(pose_pub_name, 1);
	//pose_pub = nh.advertise(op);
	
#ifdef _BUILD_FOR_ROS_
	ros::AdvertiseOptions op1 = ros::AdvertiseOptions::create<geometry_msgs::PoseStamped>(pose_pub_name, 1, &connected, &disconnected, ros::VoidPtr(), NULL);
	op1.has_header = false;
	pose_pub = nh.advertise(op1);
	
	timer = nh.createTimer(ros::Duration(0.05), &slamNode::main_loop, this);

	ROS_INFO("Establishing server callback...");
	f = boost::bind (&slamNode::serverCallback, this, _1, _2);
    server.setCallback (f);
#endif

}

bool slamNode::processScorecard() {
	
	if (configData.initializationScorecard == "") {

		// MODIFIED
		// Convergence
		scorecardParams[0][0] = 1.67;
		scorecardParams[0][1] = 1.25;
		scorecardParams[0][2] = 4.34;

		// GRIC Ratio
		scorecardParams[1][0] = 2.00;
		scorecardParams[1][1] = 0.33;
		scorecardParams[1][2] = NULL_SCORE;

		// Points in front
		scorecardParams[2][0] = 1.00;
		scorecardParams[2][1] = 0.08;
		scorecardParams[2][2] = NULL_SCORE;

		// Translation score
		scorecardParams[3][0] = 1.00;
		scorecardParams[3][1] = NULL_SCORE;
		scorecardParams[3][2] = NULL_SCORE;

		// Angle score
		scorecardParams[4][0] = 1.00;
		scorecardParams[4][1] = NULL_SCORE;
		scorecardParams[4][2] = NULL_SCORE;

		/*
		// Convergence
		scorecardParams[0][0] = 1.67;
		scorecardParams[0][1] = 1.25;
		scorecardParams[0][2] = 4.34;

		// GRIC Ratio
		scorecardParams[1][0] = 1.32;
		scorecardParams[1][1] = 0.18;
		scorecardParams[1][2] = 0.10;

		// Points in front
		scorecardParams[2][0] = 1.00;
		scorecardParams[2][1] = 0.08;
		scorecardParams[2][2] = 0.00;

		// Translation score
		scorecardParams[3][0] = 27.3;
		scorecardParams[3][1] = 13.6;
		scorecardParams[3][2] = 35.7;

		// Angle score
		scorecardParams[4][0] = 8.64;
		scorecardParams[4][1] = 2.67;
		scorecardParams[4][2] = 9.70;
		*/
	} else {
		if ((configData.initializationScorecard[0] == '.') && (configData.initializationScorecard[1] == '.')) {
			configData.initializationScorecard = configData.read_addr + "nodes/monoslam/config/" + configData.initializationScorecard;
		}
	
		ifstream ifs(configData.initializationScorecard.c_str());

		if (!ifs.is_open()) return false;
		for (int jjj = 0; jjj < 2; jjj++) { for (int iii = 0; iii < INITIALIZATION_SCORING_PARAMETERS; iii++) ifs >> scorecardParams[iii][jjj]; }
		ifs.close();
	}

	for ( int iii = 0; iii < INITIALIZATION_SCORING_PARAMETERS; iii++ ) 
  {
    char cScore1[8];
    char cScore2[8];
    char cScore3[8];
    
    ( scorecardParams[iii][0] == NULL_SCORE ) ? snprintf( cScore1, 8, "-.--" ) : snprintf( cScore1, 8, "%1.2f", scorecardParams[iii][0] );
    ( scorecardParams[iii][1] == NULL_SCORE ) ? snprintf( cScore2, 8, "-.--" ) : snprintf( cScore2, 8, "%1.2f", scorecardParams[iii][1] );
    ( scorecardParams[iii][2] == NULL_SCORE ) ? snprintf( cScore3, 8, "-.--" ) : snprintf( cScore3, 8, "%1.2f", scorecardParams[iii][2] );
    
    ROS_INFO( "Criteria (%d) = (%s, %s, %s)", iii, cScore1, cScore2, cScore3 );
  }
	return true;
}

#ifdef _BUILD_FOR_ROS_
bool slamData::obtainStartingData(ros::NodeHandle& nh) {
	
	nh.param<std::string>("stream", stream, "null");
	
	if (stream != "null") {
		ROS_INFO("Tracker stream (%s) selected.", stream.c_str());
	} else {
		ROS_ERROR("No tracker stream specified.");
		return false;
	}
	
	nh.param<bool>("logErrors", logErrors, true);
	
	nh.param<bool>("keyframeEvaluationMode", keyframeEvaluationMode, false);
	
	nh.param<int>("maxTestsPerFrame", maxTestsPerFrame, 10);
	
	nh.param<int>("maxInitializationFrames", maxInitializationFrames, 50);
	
	nh.param<int>("minStartingSeparation", minStartingSeparation, 4);
	nh.param<int>("maxStartingSeparation", maxStartingSeparation, 12);
	
	
	nh.param<string>("initializationScorecard", initializationScorecard, "../config/default_scorecard.txt");



	// FROM VIDEOSLAM

	nh.param<std::string>("extrinsicsFile", extrinsicsFile, "extrinsicsFile");
	
	if (extrinsicsFile == "extrinsicsFile") {
		ROS_ERROR("No extrinsics specified! Please provide extrinsic calibration data.");
	} else {
		ROS_INFO("Extrinsics at (%s) selected.", extrinsicsFile.c_str());
	}
	
	nh.param<bool>("writePoses", writePoses, false);
	nh.param<bool>("clearTriangulations", clearTriangulations, false);
	
	nh.param<double>("maxPoseDelay", maxPoseDelay, 1.0);
	nh.param<double>("terminationTime", terminationTime, -1.0);
	nh.param<double>("restartTime", restartTime, -1.0);
	//nh.param<bool>("evaluationMode", evaluationMode, false);
	
	
	
	nh.param<int>("evaluateParameters", evaluateParameters, 0);
	
	nh.param<std::string>("flowSource", flowSource, "/thermalvis/flow/");
	nh.param<std::string>("mapperSource", mapperSource, "/thermalvis/mapper"); // /pose
	
	nh.param<bool>("publishPoints", publishPoints, false);
	nh.param<bool>("publishKeyframes", publishKeyframes, false);

	
	return true;
}
#endif

bool slamNode::checkForKeyframe() {
	unsigned int image_idx_2 = currentPoseIndex + 1;
	unsigned int previousKeyframe = keyframe_store.keyframes.at(keyframe_store.keyframes.size()-1).idx;
	bool keyframeAdded = false;
	bool hasReverted = false;
	
	// IF CURRENT FRAME IS NOT A KEYFRAME, CHECK TO SEE IF IT SHOULD BE
	if (keyframe_store.keyframes.at(keyframe_store.keyframes.size()-1).idx != image_idx_2) {
		
		vector<unsigned int> tempIndices, maintainedIndices;

		getActiveTracks(tempIndices, *featureTrackVector, previousKeyframe, image_idx_2);
		filterToCompleteTracks(maintainedIndices, tempIndices, *featureTrackVector, previousKeyframe, image_idx_2);

		double motionScore = getFeatureMotion(*featureTrackVector, maintainedIndices, previousKeyframe, image_idx_2);		
		
		vector<unsigned int> untriangulatedIndices;
		int trackedSinceLast = int(maintainedIndices.size());
		reduceActiveToTriangulated(*featureTrackVector, maintainedIndices, untriangulatedIndices);
		
		//printf("%s::%s << Tracked since last keyframe (%d, %d): %d / %d", __PROGRAM__, __FUNCTION__, previousKeyframe, image_idx_2, trackedSinceLast, startingTracksCount);
		
		bool lowProportionTracked = false;
		if (trackedSinceLast < ((int) (((double) startingTracksCount) * configData.requiredTrackFrac))) {
			lowProportionTracked = true;
			//printf("%s::%s << Low proportion tracked: (%d / %d).", __PROGRAM__, __FUNCTION__, trackedSinceLast, startingTracksCount);
		}
		
		// Low feature count will only trigger new keyframe selection if the PROPORTION is also low...
		
		if (motionScore > configData.motionThreshold) {
			keyframe_store.addKeyframe(image_idx_2-1, blank);
			keyframeAdded = true;
			currentPoseIndex--;
			hasReverted = true;
			//printf("%s::%s << Marking (%d) as keyframe (large feature motion).", __PROGRAM__, __FUNCTION__, image_idx_2, motionScore);
		} else if ((image_idx_2 - previousKeyframe) == configData.maxKeyframeSeparation) {
			keyframe_store.addKeyframe(image_idx_2, blank);
			keyframeAdded = true;
			//printf("%s::%s << Marking (%d) as keyframe (max frames between).", __PROGRAM__, __FUNCTION__, image_idx_2);
		} else if (lowProportionTracked && (((int)trackedSinceLast) < configData.minTrackedFeatures) && (keyframe_store.keyframes.at(keyframe_store.keyframes.size()-1).idx != (image_idx_2-1))) {
			// test to see if too few features, and if previous frame is NOT a keyframe, then it should be set as one
			// ((initialPoints2.size() < configData.minTrackedFeatures) && (keyframe_store.keyframes.at(keyframe_store.count-1).idx != (iii-1))) {
			keyframe_store.addKeyframe(image_idx_2-1, blank);
			currentPoseIndex--;
			hasReverted = true;
			//printf("%s::%s << Marking (%d) [prev] as keyframe (too few features).", __PROGRAM__, __FUNCTION__, image_idx_2-1);
			keyframeAdded = true;
			//return false;
			//isKeyframe = true;
			//iii--;
			//keyframe_type = KF_TYPE_EXHAUSTED;
		} else if (lowProportionTracked && (((int)trackedSinceLast) < configData.minTrackedFeatures)) {
			keyframe_store.addKeyframe(image_idx_2, blank);
			keyframeAdded = true;
			// test to see if too few features since last keyframe, and if previous frame IS a keyframe, then current should be set as one
			//(initialPoints2.size() < configData.minTrackedFeatures) {
			//isKeyframe = true;
			//keyframe_type = KF_TYPE_WEAK;
			//printf("%s::%s << Marking (%d) as keyframe (too few features).", __PROGRAM__, __FUNCTION__, image_idx_2);
		} else if ((((int)maintainedIndices.size()) < configData.min3dPtsForPnP) && (keyframe_store.keyframes.at(keyframe_store.keyframes.size()-1).idx != (image_idx_2-1))) {
			// If there literally are not enough points to perform PNP for next frame
			keyframe_store.addKeyframe(image_idx_2-1, blank);
			currentPoseIndex--;
			hasReverted = true;
			keyframeAdded = true;
			//printf("%s::%s << Marking (%d) [prev] as keyframe (too few 3d points).", __PROGRAM__, __FUNCTION__, image_idx_2-1);
			//return false; 
		} else if (((int)maintainedIndices.size()) < configData.min3dPtsForPnP) {
			//printf("%s::%s << ERROR! Too few 3d points for PnP but previous frame already added...", __PROGRAM__, __FUNCTION__);

		}
	
	} else {
		
	}
	
	return hasReverted;
	
	if (hasReverted) {
		// Return true if the current frame is a keyframe
		return true;
	}
	
	if (keyframeAdded) {
		ROS_INFO("Added frame (%d) as Keyframe [%d]", keyframe_store.keyframes.at(keyframe_store.keyframes.size()-1).idx, currentPoseIndex);
		//return true;
	} else {
		//currentPoseIndex++;
		//printf("%s << Skipping...", __FUNCTION__);
		//return false;
	}
	
	return false;
}

void slamNode::show_poses() {

	for (int iii = 0; iii < currentPoseIndex; iii++) {
		if (ACM[iii].rows == 4) {
			
			cv::Mat temp;
			
			estimatePoseFromKnownPoints(temp, configData.cameraData, *featureTrackVector, iii, eye4);
			
			ROS_INFO("EST[%d] = ", iii);
			cout << temp << endl;
			
			estimatePoseFromKnownPoints(temp, configData.cameraData, *featureTrackVector, iii, ACM[iii]);
			
			ROS_INFO("ES2[%d] = ", iii);
			cout << temp << endl;
			
			ROS_INFO("ACM[%d] = ", iii);
			cout << ACM[iii] << endl;
		}
	}
	
}

void slamNode::getGuidingPose(cv::Mat *srcs, cv::Mat& dst, unsigned int idx) {
	
	cv::Mat eye4 = cv::Mat::eye(4, 4, CV_64FC1);
	
	unsigned int maxGuides = std::min(((int) idx), configData.maxGuides);
	
	if (srcs[idx].rows == 4) {
		// Already has an estimate
		srcs[idx].copyTo(dst);
	} else if (idx == 0) {
		// First frame so use identity
		eye4.copyTo(dst);
	} else if (srcs[idx-1].rows != 4) {
		// No previous frame at all so just use identity as guide
		eye4.copyTo(dst);
	} else {
		// Weighted combo of previous frames
		
		//printf("%s::%s << Estimating frame (%d) using previous frame poses...", __PROGRAM__, __FUNCTION__, idx);
		
		cv::Mat R_dev, t_dev;
		R_dev = cv::Mat::zeros(3, 1, CV_64FC1);
		t_dev = cv::Mat::zeros(3, 1, CV_64FC1);
		
		double total_contrib = 0.0;
		
		for (unsigned int iii = idx-maxGuides; iii < idx-1; iii++) {
			
			double contrib = ((double) idx) - ((double) iii);
			total_contrib += contrib;
			
			//printf("%s::%s << Using frame (%d:%d) [contrib = %f]", __PROGRAM__, __FUNCTION__, iii, iii+1, contrib);
			
			// For each pair of frames (iii & iii+1)
			
			cv::Mat R1, R2, t1, t2, rv1, rv2, rd, td;
			
			// Decompose into R-vec and t-vec
			decomposeTransform(srcs[iii], R1, t1);
			Rodrigues(R1, rv1);
			decomposeTransform(srcs[iii+1], R2, t2);
			Rodrigues(R2, rv2);
			
			// Find the derivative
			rd = rv2-rv1;
			
			// Need to make sure derivative is represented as minimum angular change
			for (unsigned int jjj = 0; jjj < 3; jjj++) {
				
				while (rd.at<double>(jjj, 0) > M_PI) {
					rd.at<double>(jjj, 0) -= 2*M_PI;
				}
				
				while (rd.at<double>(jjj, 0) < -M_PI) {
					rd.at<double>(jjj, 0) += 2*M_PI;
				}
			}
			
			//cout << "Deriv rotation = " << rd << endl;
			
			td = t2-t1;
			
			//cout << "Deriv translation = " << td << endl;
			
			// Add a weighted version of the derivative to R_dev & t_dev
			R_dev += pow(0.5, contrib) * rd;
			t_dev += pow(0.5, contrib) * td;
			
		}
		
		// Re-weight R_dev and t_dev
		R_dev /= total_contrib;
		t_dev /= total_contrib;
		
		//cout << "Weighted rotation = " << R_dev << endl;
		//cout << "Weighted translation = " << t_dev << endl;
		
		// Add R_dev & t_dev to the previous pose
		cv::Mat R_n, t_n, rv_n;
		decomposeTransform(srcs[idx-1], R_n, t_n);
		Rodrigues(R_n, rv_n);
		
		rv_n += R_dev;
		Rodrigues(rv_n, R_n);
		t_n += t_dev;
		
		//cout << "New rotation = " << rv_n << endl;
		//cout << "New translation = " << t_n << endl;
		
		cv::Mat T_n;
		composeTransform(R_n, t_n, T_n);
		transformationToProjection(T_n, dst);
		//compileTransform(dst, R_n, t_n);
		
		//cout << "Compiled transform = " << dst << endl;

	}
}

void slamNode::estimatePose(vector<unsigned int>& basisNodes, unsigned int idx) {

	//bool alreadyEstimated = false;
	
	//ROS_INFO("Estimating pose (%d)...", idx);
	
	if (ACM[idx].rows != 4) {
		
		if (configData.timeDebug) poseEstimationTime.startRecording();
		
		cv::Mat guidePose;
		getGuidingPose(ACM, guidePose, idx);
		
		
		
		
		//ROS_INFO("Guided pose gathered (%d)", idx);
		estimatePoseFromKnownPoints(ACM[idx], configData.cameraData, *featureTrackVector, idx, guidePose);
		//cout << "guidePose = " << guidePose << endl;
		
		if (configData.timeDebug) poseEstimationTime.stopRecording();
		
		if (ACM[idx-1].rows != 4) {
			//printf("%s::%s << Estimating pose (%d) with no guidance...", __PROGRAM__, __FUNCTION__, currentPoseIndex+1);
			//estimatePoseFromKnownPoints(ACM[idx], configData.cameraData, featureTrackVector, idx, eye4);
		} else {
			//printf("%s::%s << Estimating pose (%d) with (%d) to guide...", __PROGRAM__, __FUNCTION__, currentPoseIndex+1, currentPoseIndex);
			
			
			
			//estimatePoseFromKnownPoints(ACM[idx], configData.cameraData, featureTrackVector, idx, ACM[idx-1]);
		}
		
	} else {
		//printf("%s::%s << Pose (%d) already estimated...", __PROGRAM__, __FUNCTION__, currentPoseIndex+1);
		//alreadyEstimated = true;
		//return;
	}
	
	//cout << "estPose = " << ACM[idx] << endl;
	
	// Some kind of subsystem BA to go here...
	
#ifdef _USE_SBA_
	double avgError;
		
	if (configData.timeDebug) bundleAdjustmentTime.startRecording();
	if (basisNodes.size() <= ((unsigned int) (configData.adjustmentFrames/2))) {
		//ROS_INFO("Basis nodes size is low: %d (%d)", basisNodes.size(), configData.flowback);
		avgError = keyframeBundleAdjustment(configData.cameraData, featureTrackVector, ACM, basisNodes, configData.poseEstimateIterations, false, false, 1);
	} else if (basisNodes.size()-configData.flowback > 0) {
		//for (unsigned int iii = 0; iii < basisNodes.size(); iii++) {
			//cout << basisNodes.at(iii) << endl;
		//}
		//ROS_INFO("Basis nodes size is a bit larger: %d (%d)", basisNodes.size(), configData.flowback);
		avgError = keyframeBundleAdjustment(configData.cameraData, featureTrackVector, ACM, basisNodes, configData.poseEstimateIterations, false, false, basisNodes.size()-configData.flowback);
	} else {
		//printf("Basis nodes size is poor: %d (%d)", basisNodes.size(), configData.flowback);
		avgError = keyframeBundleAdjustment(configData.cameraData, featureTrackVector, ACM, basisNodes, configData.poseEstimateIterations, false, false, 1);
	}
	if (configData.timeDebug) bundleAdjustmentTime.stopRecording();
	//cout << "refinedPose = " << ACM[idx] << endl;
		
	//printf("Error adjusting latest pose (%d) estimate: %f", idx, avgError);
	
	if (configData.logErrors) {
		error_file << idx << " " << avgError << endl;
	}
#endif
	
}

void slamNode::getBasisNodes(vector<unsigned int>& basisNodes, unsigned int idx) {
	
	//printf("%s << ENTERED.\n", __FUNCTION__);
	
	basisNodes.clear();
	
	if (idx < keyframe_store.keyframes.at(0).idx) {
		ROS_WARN("ERROR! IDX too low (%d / %d)", idx, keyframe_store.keyframes.at(0).idx);
		return;
	}
	
	int activeCameras = ((int) idx) + 1 - ((int) keyframe_store.keyframes.at(0).idx);
	
	if (activeCameras < 2) {
		return;
	}
	
	//printf("%s << DEBUG [%0d]\n", __FUNCTION__, 0);
	
	if (activeCameras <= configData.adjustmentFrames) {
		
		//printf("%s << DEBUG [%0d]\n", __FUNCTION__, 1);
		
		unsigned int firstNode = keyframe_store.keyframes.at(0).idx;
		
		for (unsigned int iii = firstNode; iii <= idx; iii++) {
			basisNodes.push_back(iii);
		}
		
		//printf("%s << DEBUG [%0d]\n", __FUNCTION__, 2);
		
		
	} else {
		// Need to come up with some kind of subsequence to help estimate latest pose
		
		// Want to use same number of cameras, but spread them out a bit
		
		// How many MORE cameras than acceptable are there?
		
		//printf("%s << DEBUG [%0d]\n", __FUNCTION__, 3);
		
		unsigned int surplus = activeCameras - configData.adjustmentFrames;
		unsigned int spread = surplus / configData.keyframeSpacing;
		
		spread = std::min(spread, ((unsigned int) (configData.adjustmentFrames-configData.flowback)));
		
		//printf("%s::%s << idx = %d; activeCameras = %d; adjFrames = %d; surplus = %d; spread = %d", __PROGRAM__, __FUNCTION__, idx, activeCameras, configData.adjustmentFrames, surplus, spread);
		
		// For every 6 in surplus, stretch out the early ones
		unsigned int starting_cam = idx - configData.adjustmentFrames + spread + 2; // this will be first used frame
		starting_cam = starting_cam - (starting_cam % configData.keyframeSpacing); // going down to nearest'5' mult
		starting_cam -= configData.keyframeSpacing*(spread-1);
		
		//printf("%s::%s << starting_cam = %d; spread = %d; absolute limit = %d", __PROGRAM__, __FUNCTION__, starting_cam, spread, keyframe_store.keyframes.at(0).idx);
		
		//printf("%s << DEBUG (%04d)\n", __FUNCTION__, 4);
		
		for (unsigned int iii = starting_cam; iii < (idx - configData.adjustmentFrames + spread); iii += configData.keyframeSpacing) {
			//printf("%s << node: %d", __FUNCTION__, iii);
			basisNodes.push_back(iii);
			
		}
		
		//printf("%s << DEBUG (%04d)\n", __FUNCTION__, 5);
		
		//idx - (configData.adjustmentFrames - spread)
		starting_cam = idx - configData.adjustmentFrames + 1 + spread + 1;
		for (unsigned int iii = starting_cam; iii <= idx; iii++) {
			//printf("%s << node: %d", __FUNCTION__, iii);
			basisNodes.push_back(iii);
		}
		
		//printf("%s << DEBUG (%04d)\n", __FUNCTION__, 6);
		
	}
	
	//printf("%s << EXITING.\n", __FUNCTION__);
	
}

void slamNode::update_display() {
	
#ifdef _USE_SBA_
	if (currentPoseIndex != -1) {
		if (ACM[currentPoseIndex].rows == 4) {
			assignPose(currentPose, ACM[currentPoseIndex]);
			pose_pub.publish(currentPose);
		}
		
		vector<unsigned int> displayIndices;
		for (int iii = 0; iii <= currentPoseIndex; iii++) {
			if (ACM[iii].rows == 4) {
				displayIndices.push_back(iii);
			}
		}
		
		for (unsigned int iii = 0; iii < displayIndices.size(); iii++) {
			//cout << "ACM[" << displayIndices.at(iii) << "] = " << ACM[displayIndices.at(iii)] << endl;
		}
		
		assignPartialSystem(display_sys, featureTrackVector, configData.cameraData, ACM, displayIndices, false);
	
		
		
	} else {
		assignPose(currentPose, eye4);
		pose_pub.publish(currentPose);
		
		display_sys.nodes.clear();
		display_sys.tracks.clear();
	}
	
	drawGraph2(display_sys, camera_pub, points_pub, path_pub, decimation, bicolor);
#endif

}

void slamNode::update_cameras_to_pnp() {
	
	#pragma omp parallel for
	for (int iii = 0; iii < currentPoseIndex; iii++) {
		if (ACM[iii].rows == 4) {
			estimatePoseFromKnownPoints(ACM[iii], configData.cameraData, *featureTrackVector, iii, eye4);
		}
	}
	
}

bool slamNode::evaluationSummaryAndTermination() {
#ifdef _BUILD_FOR_ROS_
	if ((configData.evaluateParameters > 0) && (framesArrived > configData.evaluateParameters)) {
		// print summary
		ROS_ERROR("Reached evaluation point, shutting down...");
		ROS_WARN("Summary(1): (%d, %d, %d, %d)", framesArrived, framesProcessed, pnpSuccesses, baSuccesses);
		ROS_WARN("Summary(2): (%f, %f, %f, %f)", double(pnpSuccesses)/double(framesProcessed), double(baSuccesses)/double(framesProcessed), baAverage, dsAverage);
		
		wantsToShutdown = true;
		//mySigintHandler(1);
		
		return true;
	}
#endif
	return false;
}

void slamNode::videoslamPoseProcessing() {
#ifdef _BUILD_FOR_ROS_
	publishPose();
		
	if (configData.publishPoints) {	publishPoints(currentPose.header.stamp, currentPose.header.seq); }
		
	double elapsed = latestTracksTime - poseHistoryBuffer[(poseHistoryCounter-1) % MAX_HISTORY].header.stamp.toSec();
		
	//ROS_INFO("elapsed = (%f) vs (%f) & (%f)", elapsed, latestTracksTime, poseHistoryBuffer[(poseHistoryCounter-1) % MAX_HISTORY].header.stamp.toSec());
		
	if (elapsed > configData.maxPoseDelay) {

		if (configData.verboseMode) { ROS_INFO("Considering video-based pose estimate as a keyframe..."); }
			
			
		main_mutex.lock();
		bool updated = updateKeyframePoses(currentPose, false);
		lastTestedFrame = currentPose.header.seq;
		if (configData.publishKeyframes) { 
#ifdef _USE_SBA_
			drawKeyframes(camera_pub, keyframePoses, storedPosesCount); 
#endif
		}
		main_mutex.unlock();
			
		if (updated) {
			
			main_mutex.lock();
			if (configData.clearTriangulations) {
				for (unsigned int iii = 0; iii < featureTrackVector->size(); iii++) {
					featureTrackVector->at(iii).isTriangulated = false;
				}
			}
			triangulatePoints();
			main_mutex.unlock();
				
		}
			
	}
#endif
}

#ifdef _BUILD_FOR_ROS_
void slamNode::handle_tracks(const thermalvis::feature_tracks& msg) { // FROM MONOSLAM
#else
void slamNode::handle_tracks(const vector<featureTrack>& msg) {
#endif

	if (wantsToShutdown) return;
	if (evaluationSummaryAndTermination()) return;
	if (!infoProcessed) return;
	framesArrived++;

#ifdef _BUILD_FOR_ROS_
	if (msg.indices.size() == 0) return;
	latestTracksTime = msg.header.stamp.toSec();
    frameHeaderHistoryBuffer[frameHeaderHistoryCounter % MAX_HISTORY] = msg.header;
	frameHeaderHistoryCounter++;
#else
	if (msg.size() == 0) return;
#endif	
        	
	if (configData.timeDebug) trackHandlingTime.startRecording();

#ifdef _USE_BOOST_
	main_mutex.lock();
	integrateNewTrackMessage(msg);
	main_mutex.unlock();

	main_mutex.lock();
	if (configData.trimFeatureTracks) trimFeatureTrackVector(); 	
	main_mutex.unlock();
#endif

	if (determinePose()) videoslamPoseProcessing();

	if (configData.timeDebug) trackHandlingTime.stopRecording();
}

void slamNode::processNextFrame() {
	
	//double avgError;
	
	//ROS_INFO("About to process frame (%d)", currentPoseIndex);
	
	//ROS_INFO("Checking first frame (%d)", keyframe_store.keyframes.at(0).idx);
	
	if (!matricesAreEqual(ACM[keyframe_store.keyframes.at(0).idx], eye4)) {
		//ROS_ERROR("First camera (%d) corrupted...", keyframe_store.keyframes.at(0).idx);
		while (1) {}
	}
	
	vector<unsigned int> basisNodes;
	
	
	// Try triangulating points and fixing new camera at the same time...
	if (1) {
		
		if (configData.timeDebug) triangulationTime.startRecording();
		unsigned int triangulationIndex = currentPoseIndex-configData.flowback;
		
		ROS_INFO("Getting basis nodes...");
		getBasisNodes(basisNodes, triangulationIndex);
		
		if (basisNodes.size() > 0) {
			
			ROS_INFO("Finding relevant indices...");
			
			vector<unsigned int> triangulatedIndices, untriangulatedIndices;
			findRelevantIndices(*featureTrackVector, triangulatedIndices, untriangulatedIndices, basisNodes.at(0), triangulationIndex);
					
			// unsigned int points_in_3d;
			if (untriangulatedIndices.size() > 0) {
				
				vector<unsigned int> triangulatableIndices;
				findTriangulatableTracks3(*featureTrackVector, triangulatableIndices, triangulationIndex, configData.framesForTriangulation);
				
				if (triangulatableIndices.size() > 0) {
					//ROS_INFO("About to triangulate (%d) new tracks...", triangulatableIndices.size());
					triangulateTracks(*featureTrackVector, triangulatableIndices, configData.cameraData, ACM, basisNodes.at(0), triangulationIndex);
					//ROS_INFO("Tracks triangulated.");
					//double avgError = keyframeBundleAdjustment(configData.cameraData, featureTrackVector, ACM, basisNodes, configData.keyframeIterations, false, false, basisNodes.size()-3);
					//printf("%s::%s << F(%d) Adjustment error with newly triangulated points = %f (k = %d)", __PROGRAM__, __FUNCTION__, currentPoseIndex, avgError, basisNodes.size());
					
					if (currentPoseIndex > 60) {
						//while (1) {}
					}
					
				}
				
				
			}
			
			if (configData.timeDebug) triangulationTime.stopRecording();
			
			
			ROS_INFO("Performing adjustment... (%d)", ((int)basisNodes.size()));
			
#ifdef _USE_SBA_
			if (configData.timeDebug) bundleAdjustmentTime.startRecording();
			double avgError = keyframeBundleAdjustment(configData.cameraData, *featureTrackVector, ACM, basisNodes, configData.keyframeIterations, false, false, basisNodes.size());
			ROS_INFO("F(%d) Adjustment error with newly triangulated points = %f (k = %d)", currentPoseIndex, avgError, ((int)basisNodes.size()));
			if (configData.timeDebug) bundleAdjustmentTime.stopRecording();
#endif
		}
	
		
	}
	
	
	ROS_INFO("Getting more basis nodes...");
	getBasisNodes(basisNodes, currentPoseIndex);
	ROS_INFO("Estimating pose...");
	estimatePose(basisNodes, currentPoseIndex);
	
	if (configData.timeDebug) {
		if ((currentPoseIndex % configData.timeSpacing) == 0) {
			//double factor;
			trackHandlingTime.calcParameters();
			triangulationTime.calcParameters();
			poseEstimationTime.calcParameters();
			bundleAdjustmentTime.calcParameters();							// * ((double) trackHandlingTime.cycles) / ((double) currentPoseIndex)
			ROS_WARN("Showing timing summary:");
			ROS_INFO("Track summary: (%d, %f, %f)", trackHandlingTime.cycles, trackHandlingTime.average * ((double) trackHandlingTime.cycles) / ((double) currentPoseIndex), trackHandlingTime.sigma* ((double) trackHandlingTime.cycles) / ((double) currentPoseIndex));
			ROS_INFO("Triangulation summary: (%d, %f, %f)", triangulationTime.cycles, triangulationTime.average* ((double) triangulationTime.cycles) / ((double) currentPoseIndex), triangulationTime.sigma* ((double) triangulationTime.cycles) / ((double) currentPoseIndex));
			ROS_INFO("Pose summary: (%d, %f, %f)", poseEstimationTime.cycles, poseEstimationTime.average* ((double) poseEstimationTime.cycles) / ((double) currentPoseIndex), poseEstimationTime.sigma* ((double) poseEstimationTime.cycles) / ((double) currentPoseIndex));
			ROS_INFO("Bundle summary: (%d, %f, %f)", bundleAdjustmentTime.cycles, bundleAdjustmentTime.average* ((double) bundleAdjustmentTime.cycles) / ((double) currentPoseIndex), bundleAdjustmentTime.sigma* ((double) bundleAdjustmentTime.cycles) / ((double) currentPoseIndex));
			
		}
	}
	
	
	if (1) {
		update_display();
		currentPoseIndex++;
		return;
	}
	
	if (0) {
		vector<unsigned int> triangulatedIndices, untriangulatedIndices;
		findRelevantIndices(*featureTrackVector, triangulatedIndices, untriangulatedIndices, keyframe_store.keyframes.at(0).idx, currentPoseIndex);
				
		//unsigned int points_in_3d;
		if (untriangulatedIndices.size() > 0) {
			
			vector<unsigned int> triangulatableIndices;
			findTriangulatableTracks3(*featureTrackVector, triangulatableIndices, currentPoseIndex, configData.framesForTriangulation);
			
			if (triangulatableIndices.size() > 0) {
				//ROS_INFO("About to triangulate (%d) new tracks...", triangulatableIndices.size());
				triangulateTracks(*featureTrackVector, triangulatableIndices, configData.cameraData, ACM, keyframe_store.keyframes.at(0).idx, currentPoseIndex);
#ifdef _USE_SBA_
				double avgError = keyframeBundleAdjustment(configData.cameraData, featureTrackVector, ACM, basisNodes, configData.keyframeIterations, false, false, basisNodes.size()-3);
				if (configData.verboseMode) { ROS_INFO("F(%d) Adjustment error with newly triangulated points = %f (k = %d)", currentPoseIndex, avgError, ((int)basisNodes.size())); };
#endif
			}
			
			
		}
	}
	
	
	
	// Checks next frame to determine keyframe
	if (checkForKeyframe()) {
		
		if ((currentPoseIndex+1) == keyframe_store.keyframes.at(keyframe_store.keyframes.size()-1).idx) {
			// reverted
			//ROS_INFO("Keyframe found (reverted); processing (%d)", currentPoseIndex+1);
		} else {
			// Not a keyframe
			update_display();
			currentPoseIndex++;
			return;
		}
		
		
	} else {
		
		if ((currentPoseIndex+1) == keyframe_store.keyframes.at(keyframe_store.keyframes.size()-1).idx) {
			// not reverted
			//ROS_INFO("Keyframe found (non-reverted); processing (%d)", currentPoseIndex+1);
			basisNodes.clear();
			getBasisNodes(basisNodes, currentPoseIndex+1);
			estimatePose(basisNodes, currentPoseIndex+1);
		} else {
			// Not a keyframe
			update_display();
			currentPoseIndex++;
			return;
		}
		
	}
	
	if (0) {
		update_display();
		currentPoseIndex++;
		return;
	}
	
	// This block is used to get a decent initial estimate of the latest pose
	
	
	unsigned int image_idx_1 = keyframe_store.keyframes.at(keyframe_store.keyframes.size()-2).idx;
	unsigned int image_idx_2 = keyframe_store.keyframes.at(keyframe_store.keyframes.size()-1).idx;
	
	//ROS_INFO("Currently processing frame (%d:%d) as Keyframe", image_idx_1, image_idx_2);

	vector<cv::Point2f> pts1, pts2;
	vector<cv::Point3f> objectPoints;
	getTriangulatedFullSpanPoints(*featureTrackVector, pts1, pts2, image_idx_1, image_idx_2, objectPoints);

	
	//estimatePoseFromKnownPoints(ACM[image_idx_2], configData.cameraData, featureTrackVector, image_idx_2, eye4);
	
	
	vector<unsigned int> adjustableKeyframeIndices;
	/*
	for (unsigned int iii = 0; iii < keyframe_store.keyframes.size(); iii++) {
		adjustableKeyframeIndices.push_back(keyframe_store.keyframes.at(iii).idx);
	}
	
	while (adjustableKeyframeIndices.size() > (MAX_CAMERAS_FOR_POSE_EST_BA / 2)) {
		adjustableKeyframeIndices.erase(adjustableKeyframeIndices.begin());
	}
	* */
	
	vector<unsigned int> keyframe_indices, tmp_indices, subseq_indices;
				
	for (unsigned int iii = 0; iii < keyframe_store.keyframes.size()-1; iii++) {
		keyframe_indices.push_back(keyframe_store.keyframes.at(iii).idx);
	}
	
	// Risky mod
	//while (keyframe_indices.size() > (DEFAULT_ADJUSTMENT_FRAMES / 2)) {
	while (int(keyframe_indices.size()) > (configData.adjustmentFrames / 2)) keyframe_indices.erase(keyframe_indices.begin());
	
	for (int iii = keyframe_store.keyframes.at(keyframe_store.keyframes.size()-1).idx+1; iii < currentPoseIndex+1; iii++) tmp_indices.push_back(iii);
	
	// Risky mod
	//SelectRandomSubset(tmp_indices, subseq_indices, (DEFAULT_ADJUSTMENT_FRAMES / 2));
	SelectRandomSubset(tmp_indices, subseq_indices, (configData.adjustmentFrames / 2));
	
	
	//unsigned int fixed_cameras = keyframe_indices.size();
	
	adjustableKeyframeIndices.insert(adjustableKeyframeIndices.end(), keyframe_indices.begin(), keyframe_indices.end());
	adjustableKeyframeIndices.insert(adjustableKeyframeIndices.end(), subseq_indices.begin(), subseq_indices.end());
	adjustableKeyframeIndices.push_back(image_idx_2);
	
	//cout << "Estimated pose: " << ACM[image_idx_2] << endl;
	
	//printf("%s::%s << DEBUG [%d]", __PROGRAM__, __FUNCTION__, 5);
	
	//unsigned int startingPoseForFullSys = ((unsigned int) std::max(((int) image_idx_2)-CAMERAS_PER_SYS, ((int) lastBasePose)));
	
	//unsigned int startingPoseForFullSys =  keyframe_store.keyframes.at(0).idx;
	
	//printf("%s::%s << DEBUG [%d]", __PROGRAM__, __FUNCTION__, 51);
	
	// DETERMINE NUMBER OF TRIANGULATED POINTS CURRENTLY BEING TRACKED
	//numTriangulatedPoints = countActiveTriangulatedTracks(fullSpanIndices, featureTrackVector);
	
	// ADJUST THE MINIMUM NUMBER OF FRAMES SO THAT YOU GET A SUFFICIENT AMOUNT
	
	//avgError = keyframeBundleAdjustment(configData.cameraData, featureTrackVector, ACM, basisNodes, configData.keyframeIterations, false, true);
	//avgError = keyframeBundleAdjustment(configData.cameraData, featureTrackVector, ACM, adjustableKeyframeIndices, configData.keyframeIterations, false, false, fixed_cameras);
	
	if (ACM[currentPoseIndex+2].rows == 4) {
		//printf("%s::%s << Pose (%d) somehow estimated at this point [%d]", __PROGRAM__, __FUNCTION__, currentPoseIndex+2, 2);
	}
		
	//ROS_INFO("F(%d) Initial adjustment error = %f (k = %d)", image_idx_2, avgError, adjustableKeyframeIndices.size());
	
	if (0) {
		update_display();
		currentPoseIndex++;
		return;
	}
	
	//cout << "After kF adjustment: " << ACM[image_idx_2] << endl;
	/*
	for (unsigned int iii = 0; iii < keyframe_store.keyframes.size(); iii++) {
		
		cout << "(" << iii << ") Actual pose: " << ACM[keyframe_store.keyframes.at(iii).idx] << endl;
		
		Mat tmp;
		estimatePoseFromKnownPoints(tmp, configData.cameraData, featureTrackVector, keyframe_store.keyframes.at(iii).idx, ACM[keyframe_store.keyframes.at(iii).idx]);
		
		cout << "(" << iii << ") Re-estimated pose: " << tmp << endl;
	}
	*/
	
	vector<unsigned int> triangulatedIndices, untriangulatedIndices;
	
	//reduceActiveToTriangulated(featureTrackVector, triangulatedIndices, untriangulatedIndices);
	
	findRelevantIndices(*featureTrackVector, triangulatedIndices, untriangulatedIndices, image_idx_1, image_idx_2);
			
	//printf("%s::%s << untriangulatedIndices.size() = %d vs %d", __PROGRAM__, __FUNCTION__, untriangulatedIndices.size(), triangulatedIndices.size());
	
	//unsigned int points_in_3d;
	if (untriangulatedIndices.size() > 0) {

		vector<unsigned int> triangulatableIndices;
		findTriangulatableTracks3(*featureTrackVector, triangulatableIndices, image_idx_2, configData.framesForTriangulation); // (image_idx_2-image_idx_1)/2
		
		if (triangulatableIndices.size() > 0) {
			//ROS_INFO("About to triangulate (%d) new tracks...", triangulatableIndices.size());
			triangulateTracks(*featureTrackVector, triangulatableIndices, configData.cameraData, ACM, image_idx_1, image_idx_2);
			//avgError = keyframeBundleAdjustment(configData.cameraData, featureTrackVector, ACM, adjustableKeyframeIndices, configData.keyframeIterations, false, true);
			//ROS_INFO("F(%d) Adjustment error with newly triangulated points = %f (k = %d)", image_idx_2, avgError, adjustableKeyframeIndices.size());
		}
		
		
	}
		
	if (1) {
		update_display();
		currentPoseIndex++;
		return;
	}
	
	ROS_INFO("Establishing new adjustment indices...");
	
	vector<unsigned int> newAdjustmentIndices;
	for (unsigned int iii = 0; iii < keyframe_store.keyframes.size(); iii++) {
		
		newAdjustmentIndices.push_back(keyframe_store.keyframes.at(iii).idx);
		
		if (keyframe_store.keyframes.at(iii).idx == image_idx_1) {
			for (unsigned int jjj = image_idx_1+1; jjj < image_idx_2; jjj++) {
				newAdjustmentIndices.push_back(jjj);
			}
		}
	}
	
	update_display();
	currentPoseIndex++;
}

#ifdef TEMPORARILY_DEACTIVATED
#ifdef _BUILD_FOR_ROS_
void slamNode::handle_info(const sensor_msgs::CameraInfoConstPtr& info_msg) { // FROM VIDEOSLAM
#else
void slamNode::handle_info(sensor_msgs::CameraInfo *info_msg) {
#endif
	
	if (wantsToShutdown) return;
	
	if (!infoProcessed) {
		
		ROS_INFO("Handling camera info...");
		
		try	{
			
			configData.cameraData.K = cv::Mat::eye(3, 3, CV_64FC1);
			
			for (unsigned int mmm = 0; mmm < 3; mmm++) {
				for (unsigned int nnn = 0; nnn < 3; nnn++) {
					configData.cameraData.K.at<double>(mmm, nnn) = info_msg.K[3*mmm + nnn];
				}
			}
			
			cout << configData.cameraData.K << endl;
			
			configData.cameraData.K_inv = configData.cameraData.K.inv();
			
			configData.cameraData.cameraSize.width = info_msg.width;
			configData.cameraData.cameraSize.height = info_msg.height;
		
			unsigned int maxDistortionIndex;
			
			if (info_msg.distortion_model == "rational_polynomial") {
				maxDistortionIndex = 8;
			} else { /*if (info_msg.distortion_model == "plumb_bob") {*/
				maxDistortionIndex = 5;
			}
			
			configData.cameraData.distCoeffs = cv::Mat::zeros(1, maxDistortionIndex, CV_64FC1);
			configData.cameraData.blankCoeffs = cv::Mat::zeros(1, maxDistortionIndex, CV_64FC1);
			
			for (unsigned int iii = 0; iii < maxDistortionIndex; iii++) {
				configData.cameraData.distCoeffs.at<double>(0, iii) = info_msg.D[iii];
			}
			
			cout << "Distortion: " << configData.cameraData.distCoeffs << endl;
			
			configData.cameraData.newCamMat = cv::Mat::zeros(3, 3, CV_64FC1);
			
			cv::Rect* validPixROI = 0;
			
			double alpha = 0.00;
			bool centerPrincipalPoint = true;
			
			configData.cameraData.newCamMat = getOptimalNewCameraMatrix(configData.cameraData.K, configData.cameraData.distCoeffs, configData.cameraData.cameraSize, alpha, configData.cameraData.cameraSize, validPixROI, centerPrincipalPoint);
			
			cout << configData.cameraData.newCamMat << endl;
			
			infoProcessed = true;
			
		} catch (...) /*(sensor_msgs::CvBridgeException& e)*/ {
			ROS_ERROR("Some failure in reading in the camera parameters...");
		}
		
		ROS_INFO("Camera information processed.");
		
	} 
	
}
#endif

#ifdef _BUILD_FOR_ROS_
void slamNode::handle_info(const sensor_msgs::CameraInfoConstPtr& info_msg) { // FROM MONOSLAM
#else
void slamNode::handle_info(sensor_msgs::CameraInfo *info_msg) {
#endif

#ifdef _USE_SBA_
	drawGraph2(display_sys, camera_pub, points_pub, path_pub, decimation, bicolor);
#endif

	if (!infoProcessed) {
		ROS_INFO("Handling camera info.");
		try	{
			
			configData.cameraData.K = cv::Mat::eye(3, 3, CV_64FC1);
			
			for (unsigned int mmm = 0; mmm < 3; mmm++) {
                for (unsigned int nnn = 0; nnn < 3; nnn++) configData.cameraData.K.at<double>(mmm, nnn) = info_msg->K[3*mmm + nnn];
			}
			
			cout << configData.cameraData.K << endl;

			configData.cameraData.K_inv = configData.cameraData.K.inv();

			configData.cameraData.cameraSize.width = info_msg->width;
			configData.cameraData.cameraSize.height = info_msg->height;
		
			unsigned int maxDistortionIndex;
			if (info_msg->distortion_model == "plumb_bob") {
				maxDistortionIndex = 5;
			} else {
				if (info_msg->distortion_model != "rational_polynomial") ROS_ERROR("Unfamiliar with <info_msg.distortion_model> of (%s)", info_msg->distortion_model.c_str());
				maxDistortionIndex = 8;
			}
			
			configData.cameraData.distCoeffs = cv::Mat::zeros(1, maxDistortionIndex, CV_64FC1);
			configData.cameraData.blankCoeffs = cv::Mat::zeros(1, maxDistortionIndex, CV_64FC1);
			
			for (unsigned int iii = 0; iii < maxDistortionIndex; iii++) configData.cameraData.distCoeffs.at<double>(0, iii) = info_msg->D[iii];
			
			cout << configData.cameraData.distCoeffs << endl;
			
			configData.cameraData.newCamMat = cv::Mat::zeros(3, 3, CV_64FC1);
			
			cv::Rect* validPixROI = 0;
			
			double alpha = 0.00;
			bool centerPrincipalPoint = true;
			
			configData.cameraData.newCamMat = getOptimalNewCameraMatrix(configData.cameraData.K, configData.cameraData.distCoeffs, configData.cameraData.cameraSize, alpha, configData.cameraData.cameraSize, validPixROI, centerPrincipalPoint);
			
			cout << configData.cameraData.newCamMat << endl;
			
			infoProcessed = true;
			
#ifdef _USE_SBA_
			addFixedCamera(display_sys, configData.cameraData, eye4);
			drawGraph2(display_sys, camera_pub, points_pub, path_pub, decimation, bicolor);
#endif
			assignPose(currentPose, eye4);

#ifdef _BUILD_FOR_ROS_
			pose_pub.publish(currentPose);
#endif
		} catch (...) { ROS_ERROR("Some failure in reading in the camera parameters..."); }
		
		ROS_INFO("Camera information processed.");	
	} 
}

bool slamNode::performKeyframeEvaluation() {

	bool foundStartingPair = false;
	
	if (latestFrame < 1) return foundStartingPair;

	double keyframe_scores[5];
	cv::Mat startingTrans;
	
	cv::Mat blankMat = cv::Mat::zeros(80, 640, CV_8UC3);
	
	if ((configData.keyframeEvaluationMode) && (!evaluationStream.is_open())) {
		ROS_WARN("Opening evaluation summary stream...");
		evaluationStream.open(configData.evaluationFile.c_str(), ios::out | ios::app);
	}

	for (int jjj = configData.minStartingSeparation; jjj < min(latestFrame+1, configData.maxInitializationFrames); jjj++) {
		
		vector<unsigned int> startersToTest;
		
		for (int iii = max(0, ((int)jjj)-configData.maxStartingSeparation); iii < jjj-configData.minStartingSeparation; iii++) startersToTest.push_back(iii);
		
		while (((int)startersToTest.size()) > configData.maxTestsPerFrame) {
			unsigned int randIndex = rand() % startersToTest.size();
			startersToTest.erase(startersToTest.begin() + randIndex);
		}
		
		//#pragma omp parallel for
		for (unsigned int iii = 0; iii < startersToTest.size(); iii++) {
			
			if ((foundStartingPair) && (!configData.keyframeEvaluationMode)) break;
			
			if (keyframeTestFlags.at<unsigned char>(startersToTest.at(iii),jjj) == 0) {

				vector<unsigned int> activeTracks;
				getActiveTracks(activeTracks, *featureTrackVector, startersToTest.at(iii), jjj);

				//ROS_ERROR("About to test frames (%d) & (%d)", startersToTest.at(iii), jjj);
				startingTrans = cv::Mat();
				keyframeTestScores.at<double>(startersToTest.at(iii),jjj) = testKeyframePair(*featureTrackVector, configData.cameraData, scorecardParams, startersToTest.at(iii), jjj, keyframe_scores, startingTrans, true /*configData.keyframeEvaluationMode*/, true);
				//ROS_INFO("Frames tested");
				
				if (configData.keyframeEvaluationMode) {
					
					clearSystem();
					
					bool result = true;
					char outputString[256];
					
					ROS_INFO("Assigning as starting frames (%d, %d)...", startersToTest.at(iii), jjj);
					assignStartingFrames(startersToTest.at(iii), jjj, keyframe_scores, startingTrans);
					ROS_INFO("Starting frames assigned.");
					
					if (startingTrans.rows > 0) {
						
						//cout << "startingTrans = " << startingTrans << endl;
						formInitialStructure();
						
						ROS_INFO("Initial structure formed.");
						
						char response = ' ';
						
						blankMat = cv::Mat(blankMat.size(), blankMat.type(), CV_COLOR_GREEN);
						
						//while (0) {
						while ((response != 'y') && (response != 'n')) {
							cv::imshow("readybar", blankMat);
							response = cv::waitKey();
						}
						
						blankMat = cv::Mat(blankMat.size(), blankMat.type(), CV_COLOR_RED);
						cv::imshow("readybar", blankMat);
						cv::waitKey(1);
						
						(response == 'y') ? result = true: result = false;
						
					} else {
						ROS_INFO("Initial starting transformation invalid, returning failure.");
						result = false;
					}
					
					ROS_INFO("Preparing string...");
					sprintf(outputString, "%06d %06d %+02.2f %+02.2f %+02.2f %+02.2f %+02.2f %1.2f %d", startersToTest.at(iii), jjj, keyframe_scores[0], keyframe_scores[1], keyframe_scores[2], keyframe_scores[3], keyframe_scores[4], keyframeTestScores.at<double>(startersToTest.at(iii),jjj), (result ? 1 : 0));
					ROS_INFO("String written.");
					evaluationStream << outputString << endl;
				}
				
				
				keyframeTestFlags.at<unsigned char>(startersToTest.at(iii),jjj) = 1;

				if (keyframeTestScores.at<double>(startersToTest.at(iii),jjj) >= configData.minInitializationConfidence) foundStartingPair = true;
				ROS_INFO("Keyframe pair (%03d, %03d) initialization score = [%f] {%1.2f, %1.2f, %1.2f, %1.2f, %1.2f}", startersToTest.at(iii), jjj, keyframeTestScores.at<double>(startersToTest.at(iii),jjj), keyframe_scores[0], keyframe_scores[1], keyframe_scores[2], keyframe_scores[3], keyframe_scores[4]);
			}
		}
		
		if ((foundStartingPair) && (!configData.keyframeEvaluationMode)) break;
	}
	
	if (configData.keyframeEvaluationMode) {
		evaluationStream.close();
		evaluationCompleted = true;
		ROS_INFO("Keyframe evaluation results finalize. (CTRL + C) to terminate.");
		
		blankMat = cv::Mat(blankMat.size(), blankMat.type(), CV_COLOR_BLUE);
		cv::imshow("readybar", blankMat);
		cv::waitKey();
		return false;
	}
	
	if (foundStartingPair) {
		
		double a, b;
		cv::Point min_coord, max_coord;
		unsigned int best_iii, best_jjj;
		minMaxLoc(keyframeTestScores, &a, &b, &min_coord, &max_coord); //, keyframeTestFlags);
		
		best_iii = max_coord.y;
		best_jjj = max_coord.x;
		
		assignStartingFrames(best_iii, best_jjj, keyframe_scores, startingTrans);
	}
	
	return foundStartingPair;
}

void slamNode::clearSystem() {
	
	keyframe_store.connections.clear();
	keyframe_store.keyframes.clear();
	//keyframe_store.count = 0;
	
	for (unsigned int iii = 0; iii < featureTrackVector->size(); iii++) featureTrackVector->at(iii).isTriangulated = false;
	
	int finalIndex;
	
	(configData.keyframeEvaluationMode) ? finalIndex = configData.maxInitializationFrames + 1 : finalIndex = latestFrame + 1;
	
	for (int iii = 0; iii < finalIndex; iii++) ACM[iii] = cv::Mat();
}

void slamNode::assignStartingFrames(unsigned int idx1, unsigned int idx2, cv::Mat trans) {

	keyframe_store.clearAll();

	keyframe_store.addKeyframe(idx1, blank);
	keyframe_store.addKeyframe(idx2, blank);

	ACM[idx1] = cv::Mat::eye(4, 4, CV_64FC1);
	trans.copyTo(ACM[idx2]);

	cout << ACM[idx1] << endl;
	cout << ACM[idx2] << endl;
	
	keyframe_store.addConnection(0, 1, KF_CONNECTION_GEOMETRIC, F_arr[idx1]);
}


double slamNode::assignStartingFrames(unsigned int best_iii, unsigned int best_jjj, double* keyframe_scores, cv::Mat& startingTrans) {
	
	printf("%s << Adding (%d) & (%d) to keyframe store... (already (%d) large)", __FUNCTION__, best_iii, best_jjj, ((int)keyframe_store.keyframes.size()));
	
	keyframe_store.addKeyframe(best_iii, blank);
	keyframe_store.addKeyframe(best_jjj, blank);
	
	double kfScore = testKeyframePair(*featureTrackVector, configData.cameraData, scorecardParams, best_iii, best_jjj, keyframe_scores, startingTrans, configData.keyframeEvaluationMode, true);
	
	ACM[best_iii] = cv::Mat::eye(4, 4, CV_64FC1);
	startingTrans.copyTo(ACM[best_jjj]);
	
	ROS_INFO("kfScore = %f (%d, %d)", kfScore, best_iii, best_jjj);
	cout << ACM[best_iii] << endl;
	cout << ACM[best_jjj] << endl;
	
	keyframe_store.addConnection(int(keyframe_store.keyframes.size())-2, int(keyframe_store.keyframes.size())-1, KF_CONNECTION_GEOMETRIC, F_arr[best_iii]);
	
	return kfScore;
	
}

bool slamNode::formInitialStructure() {

	// In terms of selecting the arbitary scale, you should make it so that the centroid of the 
	// 3D points in front of the camera (those used for initial structure) are 1m in front of the first camera (in terms of their Z coordinates).
	
	//ROS_INFO("Entered <formInitialStructure> (%d).", baseConnectionNum);
	
	//cout << "ACM[0] = " << ACM[0] << endl;
	
	double keyframe_scores[5];
	cv::Mat startingTrans;
	bool formedInitialStructure = false;

	int keyframe_idx_1 = keyframe_store.connections.at(baseConnectionNum).idx1;
	int keyframe_idx_2 = keyframe_store.connections.at(baseConnectionNum).idx2;
	
	//printf("%s << keyframe_idx = (%d, %d) [%d]", __FUNCTION__, keyframe_idx_1, keyframe_idx_2, keyframe_store.connections.size());
	
	unsigned int image_idx_1 = keyframe_store.keyframes.at(keyframe_idx_1).idx;
	unsigned int image_idx_2 = keyframe_store.keyframes.at(keyframe_idx_2).idx;
	
	//printf("%s << image_idx = (%d, %d)", __FUNCTION__, image_idx_1, image_idx_2);
	
	lastBasePose = image_idx_1;
	
	vector<cv::Point2f> pts1, pts2;
	getPointsFromTracks(*featureTrackVector, pts1, pts2, image_idx_1, image_idx_2);
	
	vector<unsigned int> activeTrackIndices, fullSpanIndices, triangulatedIndices;
		
	getActiveTracks(activeTrackIndices, *featureTrackVector, image_idx_1, image_idx_2);
	filterToCompleteTracks(fullSpanIndices, activeTrackIndices, *featureTrackVector, image_idx_1, image_idx_2);
	
	startingTracksCount = (unsigned int)(fullSpanIndices.size());
	
	//printf("%s << Active indices for this subsequence: %d", __FUNCTION__, activeTrackIndices.size());
	//printf("%s << Full-span indices for this subsequence: %d", __FUNCTION__, fullSpanIndices.size());
	
	//int numTriangulatedPoints = countActiveTriangulatedTracks(fullSpanIndices, featureTrackVector);
	
	//printf("%s << Number of triangulated full-span tracks for this subsequence = %d", __FUNCTION__, numTriangulatedPoints);
		
	vector<cv::Point3d> ptsInCloud;
	cv::Mat P1, R1, t1; //, CX[4];
	cv::Mat Rvec, C;
	vector<cv::Point2f> correspPoints;

	//ROS_INFO("Reconstructing initialization pair with (%d) & (%d)...", image_idx_1, image_idx_2);
	
	//cout << "ACM[0] = " << ACM[0] << endl;
	
	bool worked = reconstructFreshSubsequencePair(*featureTrackVector, ptsInCloud, triangulatedIndices, ACM[image_idx_1], ACM[image_idx_2], configData.cameraData, image_idx_1, image_idx_2);
	
	//ROS_INFO("Initial fresh cameras:");
	//cout << ACM[image_idx_1] << endl;
	//cout << ACM[image_idx_2] << endl;
	
	if (!worked) {
		ROS_ERROR("2-Frame reconstruction didn't work.");
	}
	
	//cam_mutex.unlock();
	
	//printf("%s::%s << DEBUG [%d]", __PROGRAM__, __FUNCTION__, 0);
	
	double keyframeError;
	keyframeError = testKeyframePair(*featureTrackVector, configData.cameraData, scorecardParams, image_idx_1, image_idx_2, keyframe_scores, startingTrans, configData.keyframeEvaluationMode);
	
	//printf("%s::%s << DEBUG [%d]", __PROGRAM__, __FUNCTION__, 1);
	
	//cout << "(" << keyframeError << ") = " << startingTrans << endl;
	startingTrans.copyTo(ACM[image_idx_2]);
	//cam_mutex.lock();
	//printf("%s << Pre-optimized initialization cameras:", __FUNCTION__);
	//ROS_INFO("Initial keyframe cameras:");
	//cout << ACM[image_idx_1] << endl;
	//cout << ACM[image_idx_2] << endl;
	
	//cout << "ACM[0] = " << ACM[0] << endl;
	/*
	keyframeError = optimizeKeyframePair(featureTrackVector, configData.cameraData, image_idx_1, image_idx_2, ACM);
	printf("%s << Freshly reconstructed initial cameras:", __FUNCTION__);
	cout << ACM[image_idx_1] << endl;
	cout << ACM[image_idx_2] << endl;
	*/
	//cam_mutex.unlock();
	
	//double twoViewError;
	
	vector<unsigned int> adjustableKeyframeIndices;
	
	adjustableKeyframeIndices.push_back(image_idx_1);
	adjustableKeyframeIndices.push_back(image_idx_2);
	
	//cam_mutex.lock();
	
#ifdef _USE_SBA_
	keyframeError = keyframeBundleAdjustment(configData.cameraData, featureTrackVector, ACM, adjustableKeyframeIndices, configData.initialStructureIterations, false, false, 1);
	ROS_INFO("Adjusted error: %f", keyframeError);
#endif

	//cout << ACM[image_idx_1] << endl;
	//cout << ACM[image_idx_2] << endl;
	
	//cam_mutex.unlock();

	
	//real_C0.copyTo(ACM[image_idx_1]);
	//real_C1.copyTo(ACM[image_idx_2]);
						
	if (!worked) {
		ROS_ERROR("Reconstruction of fresh subsequence pair failed.");
		return false;
	}
	
	ROS_INFO("Getting active 3d points...");
	
	//printf("%s << ptsInCloud.size() (pre-update) = %d", __FUNCTION__, ptsInCloud.size());
	getActive3dPoints(*featureTrackVector, triangulatedIndices, ptsInCloud);
	//printf("%s << ptsInCloud.size() (post-update) = %d", __FUNCTION__, ptsInCloud.size());

#ifdef _USE_OPENCV_VIZ_
	if (configData.inspectInitialization) {

		/*
		cv::viz::WLine x_axis(cv::Point3f(0.0f,0.0f,0.0f), cv::Point3f(1.0f,0.0f,0.0f));
		x_axis.setRenderingProperty(cv::viz::LINE_WIDTH, 2.0);
		cv::viz::WLine y_axis(cv::Point3f(0.0f,0.0f,0.0f), cv::Point3f(0.0f,1.0f,0.0f));
		y_axis.setRenderingProperty(cv::viz::LINE_WIDTH, 1.0);
		cv::viz::WLine z_axis(cv::Point3f(0.0f,0.0f,0.0f), cv::Point3f(0.0f,0.0f,1.0f));
		z_axis.setRenderingProperty(cv::viz::LINE_WIDTH, 4.0);
    
		viz.showWidget("X-Axis Widget", x_axis);
		viz.showWidget("Y-Axis Widget", y_axis);
		viz.showWidget("Z-Axis Widget", z_axis);
		*/

		cv::viz::WCloud ptCloud(ptsInCloud);
		viz.showWidget("3D Cloud", ptCloud); // z flipped

		/// Let's assume camera has the following properties
		cv::Point3d cam_pos, cam_focal_point, cam_y_dir(0.0f,1.0f,0.0f);

		cam_focal_point = findCentroid(ptsInCloud);
		double cloudExtent = findPrismDiagonal(ptsInCloud);

		cv::Point3d cam_pos_1(cam_pose_1.matrix(0,3), cam_pose_1.matrix(1,3), cam_pose_1.matrix(2,3));
		cv::Point3d cam_pos_2(cam_pose_2.matrix(0,3), cam_pose_2.matrix(1,3), cam_pose_2.matrix(2,3));

		cam_pos = cam_focal_point + 1.5*cloudExtent*(0.5*(cam_pos_1 + cam_pos_2) - cam_focal_point)*(1.0/norm(0.5*(cam_pos_1 + cam_pos_2) - cam_focal_point));
		cam_pos.y -= 0.5*cloudExtent;

		/// We can get the pose of the cam using makeCameraPose
		floating_pose = cv::viz::makeCameraPose(cam_pos, cam_focal_point, cam_y_dir);
		//cv::viz::Camera(...)
		viz.setViewerPose(floating_pose);

		convertMatToAffine(ACM[image_idx_1], cam_pose_1);
		convertMatToAffine(ACM[image_idx_2], cam_pose_2);

		/// We can get the transformation matrix from camera coordinate system to global using
		/// - makeTransformToGlobal. We need the axes of the camera
		cv::Affine3f transform = cv::viz::makeTransformToGlobal(cv::Vec3f(0.0f,-1.0f,0.0f), cv::Vec3d(-1.0f,0.0f,0.0f), cv::Vec3d(0.0f,0.0f,-1.0f), cam_pos);

		cv::viz::WCameraPosition cpw(0.05); // Coordinate axes
		cv::viz::WCameraPosition cpw_frustum1(cv::Vec2f(0.9f, 0.5f)), cpw_frustum2(cv::Vec2f(0.9f, 0.5f)); // Camera frustum // 0.889484, 0.523599
    
		//viz.showWidget("CPW_1", cpw, cam_pose_1);
		viz.showWidget("CPW_FRUSTUM_1", cpw_frustum1, cam_pose_1);
	
		//viz.showWidget("CPW_2", cpw, cam_pose_2);
		viz.showWidget("CPW_FRUSTUM_2", cpw_frustum2, cam_pose_2);

		viz.registerKeyboardCallback(keyboard_callback, (void*)(&kC));
		//cloud_viewer->registerKeyboardCallback (keyboard_callback, (void*)(&kC));

		while(!viz.wasStopped()) {
			if (kC.toggleCamera) {
				viz.showWidget("CoordinateAxis", cv::viz::WCoordinateSystem());
				switch (current_view_index) {
				case -1:
					current_view_index++;
					viz.removeWidget("CoordinateAxis");
					viz.setViewerPose(cam_pose_1);
					break;
				case 0:
					current_view_index++;
					viz.removeWidget("CoordinateAxis");
					viz.setViewerPose(cam_pose_2);
					break;
				case 1:
					current_view_index = -1;
					viz.setViewerPose(floating_pose);
					break;
				default:
					break;
				}
				kC.toggleCamera = false;
				if (kC.exit) break;
			}
			viz.spinOnce(1, true);
		}
	}
#endif
	
	ROS_INFO("Points acquired. GT.");
	
	int relativeIndex = image_idx_2-image_idx_1;
	
	printf("%s << DEBUG (%04d)\n", __FUNCTION__, 0);
	
#ifdef _USE_SBA_
	SysSBA subsys;
	addPointsToSBA(subsys, ptsInCloud);
	
	printf("%s << DEBUG (%04d)\n", __FUNCTION__, 1);
	
	//cam_mutex.lock();
	addFixedCamera(subsys, configData.cameraData, ACM[image_idx_1]);
	
	printf("%s << DEBUG (%04d)\n", __FUNCTION__, 2);
	
	addNewCamera(subsys, configData.cameraData, ACM[image_idx_2]);
	
	//cam_mutex.unlock();
	
	printf("%s << DEBUG (%04d)\n", __FUNCTION__, 3);
	
	subsys.nFixed = 1;
	
	addProjectionsToSBA(subsys, pts1, 0);	// keyframe_idx_1
	
	printf("%s << DEBUG (%04d)\n", __FUNCTION__, 4);
	
	addProjectionsToSBA(subsys, pts2, 1);
	
	ROS_INFO("Going through relative indices...");
	
	for (int jjj = 1; jjj < relativeIndex; jjj++) {
		
		vector<cv::Point2f> latestPoints;
		
		printf("%s << About to get corresponding points (%d)...\n", __FUNCTION__, jjj);
		getCorrespondingPoints(featureTrackVector, pts1, latestPoints, image_idx_1, image_idx_1+jjj);
		printf("%s << Corresponding points acquired (%d)\n", __FUNCTION__, jjj);
		
		
		vector<cv::Point3f> objectPoints;
		cv::Point3f tmpPt;
		
		for (unsigned int kkk = 0; kkk < ptsInCloud.size(); kkk++) {
			tmpPt = cv::Point3f((float) ptsInCloud.at(kkk).x, (float) ptsInCloud.at(kkk).y, (float) ptsInCloud.at(kkk).z);
			objectPoints.push_back(tmpPt);
		}
		
		cv::Mat t, R, Rvec;
		
		printf("%s << DEBUG (%03d)\n", __FUNCTION__, 0);
		
		printf("%s << Solving PnP... (%d) (oP.size() = %d; lP.size() = %d)\n", __FUNCTION__, jjj, ((int)objectPoints.size()), ((int)latestPoints.size()));
		
		if (objectPoints.size() != latestPoints.size()) {
			ROS_ERROR("Unable to find proper corresponding points!");
			continue;
		}
		//solvePnPRansac(InputArray objectPoints, InputArray imagePoints, InputArray cameraMatrix, InputArray distCoeffs, OutputArray rvec, OutputArray tvec, bool useExtrinsicGuess=false, int iterationsCount=100, float reprojectionError=8.0, int minInliersCount=100, OutputArray inliers=noArray() );
		solvePnPRansac(objectPoints, latestPoints, configData.cameraData.K, configData.cameraData.blankCoeffs, Rvec, t);
		
		printf("%s << DEBUG (%03d)\n", __FUNCTION__, 1);
		
		Rodrigues(Rvec, R);
		
		//cout << __FUNCTION__ << " << [" << jjj << "] R = " << R << endl;
		//cout << __FUNCTION__ << " << [" << jjj << "] t = " << t << endl;
		
		
		cv::Mat newCam;
		cv::Mat T;
		composeTransform(R, t, T);
		transformationToProjection(T, newCam);
		// compileTransform(newCam, R, t);
		
		// Inverting camera "learned" by PnP since it always seems to need to be inverted when passed to 
		// and from SBA...
		
		cv::Mat newCamC;
		projectionToTransformation(newCam, newCamC);
		newCamC = newCamC.inv();
		transformationToProjection(newCamC, newCam);
		
		addNewCamera(subsys, configData.cameraData, newCam);
		
		addProjectionsToSBA(subsys, latestPoints, jjj+1);
		
		printf("%s << DEBUG (%03d)\n", __FUNCTION__, 2);
		
		//avgError = optimizeSystem(subsys, 1e-4, 10);
		//printf("%s << Progressive subsequence BA error = %f.", __FUNCTION__, avgError);
		
		//drawGraph2(subsys, camera_pub, points_pub, path_pub, decimation, bicolor);
		
	}
	
	//printf("%s << About to optimize subsystem... (nFixed = %d)", __FUNCTION__, subsys.nFixed);
	//ROS_INFO("About to optimize starting sequence...");
	
	//cout << "ACM[0] = " << ACM[0] << endl;
	//if (iii == 0) {
		
	subsys.nFixed = 1;
	ROS_INFO("About to rescale (1)");
	rescaleSBA(subsys, 0, 1);
	ROS_INFO("About to optimize between rescalings...");
	double avgError = optimizeSystem(subsys, 1e-4, configData.subsequenceIterations);
	ROS_INFO("About to rescale (2)");
	rescaleSBA(subsys, 0, 1);
	ROS_INFO("Rescaling done");

	
	if (avgError < 0.0) {
		ROS_ERROR("Subsystem optimization failed to converge..");
	}
	
	//ROS_INFO("Subsystem optimized; err = %f", avgError);
	
	
	//cout << "ACM[0] = " << ACM[0] << endl;
	//printf("%s << DEBUG [%d] looking for crash...", __FUNCTION__, 0);
	
	//cam_mutex.lock();
	retrieveCameraPose(subsys, 0, ACM[image_idx_1]);
	retrieveCameraPose(subsys, 1, ACM[image_idx_2]);
	
	//ROS_INFO("Starting sequence cameras =");
	//cout << ACM[image_idx_1] << endl;
	
	
	//printf("%s << DEBUG [%d] looking for crash...", __FUNCTION__, 1);
	
	#pragma omp parallel for
	for (unsigned int ttt = 1; ttt < image_idx_2-image_idx_1; ttt++) {
		retrieveCameraPose(subsys, ttt+1, ACM[image_idx_1+ttt]);
		//cout << "ACM[0]" << ACM[image_idx_1+ttt] << endl;
	}
	
	//cout << ACM[image_idx_2] << endl;
	
	//cout << "ACM[0] = " << ACM[0] << endl;
	
	//cam_mutex.unlock();

	
	ptsInCloud.clear();
	retrieveAllPoints(ptsInCloud, subsys);
	
	updateTriangulatedPoints(featureTrackVector, triangulatedIndices, ptsInCloud);
	
	vector<unsigned int> basisNodes;
	
	for (unsigned int iii = image_idx_1; iii <= image_idx_2; iii++) {
		basisNodes.push_back(iii);
	}
												
												keyframeError = keyframeBundleAdjustment(configData.cameraData, featureTrackVector, ACM, basisNodes, configData.keyframeIterations, false, false, 1);
												//ROS_INFO("Full-sys error 1: %f", keyframeError);
												// So by here you've done all basic triangulation..
												/*
												assignFullSystem(sys, featureTrackVector, configData.cameraData, ACM, image_idx_1, image_idx_2);
												printf("%s << SYS: nodes = %d; tracks = %d", __FUNCTION__, sys.nodes.size(), sys.tracks.size());
												sys.nFixed = 1;
												rescaleSBA(sys, 0, sys.nodes.size()-1);
												avgError = optimizeSystem(sys, 1e-4, 10);
												rescaleSBA(sys, 0, sys.nodes.size()-1);
												printf("%s << retrieving full system with image (%d) to start with", __FUNCTION__, image_idx_1);
												retrieveFullSystem(sys, ACM, featureTrackVector, image_idx_1, image_idx_2);
												sys.nodes.clear();
												sys.tracks.clear();
												//currentPoseIndex = image_idx_2;
												//formedInitialStructure = true;
												//printf("%s::%s << Initial structure formed.", __PROGRAM__, __FUNCTION__);
												//return formedInitialStructure;
												*/
				
	
	
	//getBasisNodes(basisNodes, currentPoseIndex);
#endif

	if (1) {
		
		vector<unsigned int> triangulatedIndices, untriangulatedIndices;
		findRelevantIndices(*featureTrackVector, triangulatedIndices, untriangulatedIndices, image_idx_1, image_idx_2);
				
		//unsigned int points_in_3d;
		if (untriangulatedIndices.size() > 0) {
			
			vector<unsigned int> triangulatableIndices;
			findTriangulatableTracks3(*featureTrackVector, triangulatableIndices, image_idx_2, configData.framesForTriangulation);
			
			if (triangulatableIndices.size() > 0) {
				//ROS_INFO("About to triangulate (%d) new tracks...", triangulatableIndices.size());
				triangulateTracks(*featureTrackVector, triangulatableIndices, configData.cameraData, ACM, image_idx_1, image_idx_2);
				//ROS_INFO("Tracks triangulated.");
				//double avgError = keyframeBundleAdjustment(configData.cameraData, featureTrackVector, ACM, basisNodes, configData.keyframeIterations, false, false, basisNodes.size()-3);
				//printf("%s::%s << F(%d) Adjustment error with newly triangulated points = %f (k = %d)", __PROGRAM__, __FUNCTION__, currentPoseIndex, avgError, basisNodes.size());
				
				if (currentPoseIndex > 60) {
					//while (1) {}
				}
				
			}
			
			
		}
	}
	
	//cout << "ACM[0] = " << ACM[0] << endl;
	
#ifdef _USE_SBA_
	keyframeError = keyframeBundleAdjustment(configData.cameraData, featureTrackVector, ACM, basisNodes, configData.keyframeIterations, false, false, 1);
	//ROS_INFO("Full-sys error 2: %f", keyframeError);
	/*
	assignFullSystem(sys, featureTrackVector, configData.cameraData, ACM, image_idx_1, image_idx_2);
	//printf("%s << SYS: nodes = %d; tracks = %d", __FUNCTION__, sys.nodes.size(), sys.tracks.size());
	printf("%s::%s << Optimizing with triangulated points...", __PROGRAM__, __FUNCTION__);
	sys.nFixed = 1;
	rescaleSBA(sys, 0, sys.nodes.size()-1);
	avgError = optimizeSystem(sys, 1e-4, 10);
	rescaleSBA(sys, 0, sys.nodes.size()-1);
	printf("%s::%s << Final error = %f", __PROGRAM__, __FUNCTION__, avgError);
												
	cout << "ACM[0] = " << ACM[0] << endl;
	//printf("%s << retrieving full system with image (%d) to start with", __FUNCTION__, image_idx_1);
	retrieveFullSystem(sys, ACM, featureTrackVector, image_idx_1, image_idx_2);
	*/
	currentPoseIndex = image_idx_2;
	formedInitialStructure = true;
	//ROS_INFO("Initial structure formed.");
	//cout << "ACM[0] = " << ACM[0] << endl;
												
	for (unsigned int iii = image_idx_1; iii <= image_idx_2; iii++) {
		//cout << "ACM[" << iii << "] = " << ACM[iii] << endl;
	}
												
												
	if (1) {
		update_display();
	}
												
	//while (1) { }
#else
	formedInitialStructure = true;
#endif
												
	return formedInitialStructure;	
							

}

void slamNode::assignPose(geometry_msgs::PoseStamped& pPose, cv::Mat& C) {
	pPose.header.seq = currentPoseIndex;
	pPose.header.stamp = ros::Time::now();
	
	cv::Mat R, t;
	Quaterniond Q;
	decomposeTransform(C, R, t);
	matrixToQuaternion(R, Q);
	
	// tried: 1,0,2; 1,2,0; 0,2,1; 2,0,1; 2,1,0; 0,1,2
	// x-corresponds to graph -x; y to graph -z; z to graph -y
	
	pPose.pose.position.x = float(t.at<double>(2,0)); //;
	pPose.pose.position.y = float(-t.at<double>(0,0)); //t.at<double>(1,0);
	pPose.pose.position.z = float(-t.at<double>(1,0)); //t.at<double>(2,0);
	
	if (abs(pPose.pose.position.x) > MAX_RVIZ_DISPLACEMENT) {
		pPose.pose.position.x = 0.0;
	}

	if (abs(pPose.pose.position.y) > MAX_RVIZ_DISPLACEMENT) {
		pPose.pose.position.y = 0.0;
	}
	
	if (abs(pPose.pose.position.z) > MAX_RVIZ_DISPLACEMENT) {
		pPose.pose.position.z = 0.0;
	}
	
	//printf("%s << QUAT = (%f, %f, %f, %f)", __FUNCTION__, Q.x(), Q.y(), Q.z(), Q.w());
	
	// tried x,y,z,w
	pPose.pose.orientation.x = float(Q.z());
	pPose.pose.orientation.y = float(-Q.x());
	pPose.pose.orientation.z = float(-Q.y());
	pPose.pose.orientation.w = float(Q.w());
}


void slamNode::refreshPoses() {
	
	/*
	for (unsigned int iii = keyframe_store.keyframes.at(0).idx; iii < keyframe_store.keyframes.at(keyframe_store.keyframes.size()-1).idx; iii++) {
		
		//estimatePoseFromKnownPoints(ACM[iii], configData.cameraData, featureTrackVector, iii, eye4);
		
		if (ACM[iii].rows == 4) {
			Mat test;
			estimatePoseFromKnownPoints(test, configData.cameraData, featureTrackVector, iii, eye4);
			
			//cout << "est[" << iii << "] = " << test << endl;
			//cout << "acm[" << iii << "] = " << ACM[iii] << endl;
		}
		
	}
	*/
	
	/*
	for (unsigned int iii = 0; iii < currentPoseIndex; iii++) {
		
		estimatePoseFromKnownPoints(ACM[iii], configData.cameraData, featureTrackVector, iii, eye4);
		
		if (ACM[iii].rows == 4) {
			Mat test;
			estimatePoseFromKnownPoints(test, configData.cameraData, featureTrackVector, iii, eye4);
			
			//cout << "est[" << iii << "] = " << test << endl;
			//cout << "acm[" << iii << "] = " << ACM[iii] << endl;
		}
		
	}
	*/
	for (unsigned int iii = 0; iii < keyframe_store.keyframes.size(); iii++) {
		//estimatePoseFromKnownPoints(ACM[keyframe_store.keyframes.at(iii).idx], configData.cameraData, featureTrackVector, keyframe_store.keyframes.at(iii).idx, eye4);
	}
	
}

bool slamNode::checkConnectivity(unsigned int seq) {
	
	//main_mutex.lock();
	
	
	
	unsigned int projectionsCount = 0;
	
	unsigned int longTracksCount = 0;
	
	for (unsigned int iii = 0; iii < featureTrackVector->size(); iii++) {
		
		if (featureTrackVector->at(iii).locations.size() < 2) {
			continue;
		}
		
		longTracksCount++;
		
		for (unsigned int jjj = 0; jjj < featureTrackVector->at(iii).locations.size(); jjj++) {
			
			//printf("%s << featureTrackVector.at(%d).locations.at(%d).imageIndex = (%d) vs (%d)\n", __FUNCTION__, iii, jjj, featureTrackVector.at(iii).locations.at(jjj).imageIndex, seq);
			
			if (featureTrackVector->at(iii).locations.at(jjj).imageIndex == seq) {
				projectionsCount++;
			}
		}
	}
	
	if (configData.verboseMode) { ROS_INFO("Projections for current tracks #(%d) = (%d, %d)", seq, projectionsCount, longTracksCount); }
	
	//main_mutex.unlock();
	
	if (projectionsCount < 8) return false;
	
	return true;
	
}

bool slamNode::updateKeyframePoses(const geometry_msgs::PoseStamped& pose_msg, bool fromICP) {
	
	if (configData.verboseMode) { ROS_INFO("Entered <%s>", __FUNCTION__); }
	
	// First, some kind of check to make sure that the tracks/connections for this frame are decent..
	
	if ( (storedPosesCount > 0) && !checkConnectivity(pose_msg.header.seq) ) {
		if (configData.verboseMode) { ROS_WARN("Frame (%d) has insufficient connectivity...", pose_msg.header.seq); }
		return false;
	}
	
	if (configData.verboseMode) { ROS_WARN("Considering tracks frame (%d) as a keyframe (has sufficient connectivity)", pose_msg.header.seq); }
	
	if (fromICP) {
		
		unsigned int floatersCleared = 0;
		
		for (unsigned int iii = 0; iii < storedPosesCount; iii++) {
			
			if (!keyframeTypes[iii]) {
			
				for (unsigned int jjj = iii; jjj < storedPosesCount-1; jjj++) {
					keyframePoses[jjj] = keyframePoses[jjj+1];
					keyframeTypes[jjj] = keyframeTypes[jjj+1];
				}
				
				storedPosesCount--;
				iii--;
				floatersCleared++;
				
			}
			
			
		}
		
		if (configData.verboseMode && (floatersCleared > 0)) { ROS_INFO("Cleared (%d) poses (%d remain) because now have received ICP-based estimate.", floatersCleared, storedPosesCount); }
	}
	
	if (((int)storedPosesCount) < configData.adjustmentFrames) {
		keyframePoses[storedPosesCount] = pose_msg;
		keyframeTypes[storedPosesCount] = fromICP;
		if (configData.verboseMode) { ROS_INFO("Adding keyframe pose with index (%d)", pose_msg.header.seq); }
		storedPosesCount++;
		return true;
	} /* else if (fromICP) {
		
		for (unsigned int iii = 0; iii < storedPosesCount; iii++) {
			
			if (!keyframeTypes[iii]) {
			
				for (unsigned int jjj = iii; jjj < storedPosesCount-1; jjj++) {
					keyframePoses[jjj] = keyframePoses[jjj+1];
					keyframeTypes[jjj] = keyframeTypes[jjj+1];
				}
				
				storedPosesCount--;
				
				if (configData.verboseMode) { ROS_WARN("Appending most recent frame (%d) to keyframes to replace video-based frame", pose_msg.header.seq); }
				keyframePoses[storedPosesCount] = pose_msg;
				keyframeTypes[storedPosesCount] = fromICP;
				storedPosesCount++;
				return true;
				
			}
		}
	}*/
	
	while (((int)storedPosesCount) >= ((int)configData.adjustmentFrames)) {

		double maxDistance = 0.0;
		unsigned int maxIndex = 0;
		
		for (unsigned int iii = 0; iii < storedPosesCount; iii++) {
			double dist = pow(pose_msg.pose.position.x - keyframePoses[iii].pose.position.x, 2.0) + pow(pose_msg.pose.position.y - keyframePoses[iii].pose.position.y, 2.0) + pow(pose_msg.pose.position.z - keyframePoses[iii].pose.position.z, 2.0);
			
			if (dist >= maxDistance) {
				maxDistance = dist;
				maxIndex = iii;
			}
			
		}
		
		maxDistance = pow(maxDistance, 0.5);
		
		if ( (maxDistance <= configData.maxDistance) && (configData.maxDistance != 0.0) ) {
			break;
		}
		
		if (configData.verboseMode) { ROS_INFO("Removing frame (%d) because of distance (%f) > (%f)", keyframePoses[maxIndex].header.seq, maxDistance, configData.maxDistance); }
		
		for (unsigned int iii = maxIndex; iii < storedPosesCount-1; iii++) {
			keyframePoses[iii] = keyframePoses[iii+1];
			keyframeTypes[iii] = keyframeTypes[iii+1];
		}
		
		storedPosesCount--;
		
		if (((int)storedPosesCount) == ((int)configData.adjustmentFrames-1)) {
			if (configData.verboseMode) { ROS_WARN("Appending most recent frame (%d) to keyframes in max loop", pose_msg.header.seq); }
			keyframePoses[storedPosesCount] = pose_msg;
			keyframeTypes[storedPosesCount] = fromICP;
			storedPosesCount++;
			return true;
		}
		
	}
	
	// Otherwise, need to find the least informative... (want to include newest, too!)
	
	while (((int)storedPosesCount) >= ((int)configData.adjustmentFrames)) {
		
		double minDistance = std::numeric_limits<double>::max();
		unsigned int minIndex = 0;
		
		for (unsigned int iii = 0; iii < storedPosesCount; iii++) {
		
			double dist = 0.0;
			double div = 0.0;
			
			if (iii > 0) {
				dist += pow(keyframePoses[iii].pose.position.x - keyframePoses[iii-1].pose.position.x, 2.0) + pow(keyframePoses[iii].pose.position.y - keyframePoses[iii-1].pose.position.y, 2.0) + pow(keyframePoses[iii].pose.position.z - keyframePoses[iii-1].pose.position.z, 2.0);
				div += 1.0;
			} 
			
			if(iii < (storedPosesCount-1)) {
				dist += pow(keyframePoses[iii].pose.position.x - keyframePoses[iii+1].pose.position.x, 2.0) + pow(keyframePoses[iii].pose.position.y - keyframePoses[iii+1].pose.position.y, 2.0) + pow(keyframePoses[iii].pose.position.z - keyframePoses[iii+1].pose.position.z, 2.0);
				div += 1.0;
			}
			
			if (div == 0.0) {
				ROS_ERROR("Trying to update keyframe list but only one seems to exist!");
			}
			
			dist /= div;
			
			if (dist <= minDistance) {
				minDistance = dist;
				minIndex = iii;
			}
			
		}
		
		minDistance = pow(minDistance, 0.5);
		
		if (configData.verboseMode) { ROS_WARN("Removing %dth frame (%d) which is least distant (%f)", minIndex, keyframePoses[minIndex].header.seq, minDistance); }
		
		for (unsigned int iii = minIndex; iii < storedPosesCount-1; iii++) {
			keyframePoses[iii] = keyframePoses[iii+1];
			keyframeTypes[iii] = keyframeTypes[iii+1];
		}
		
		storedPosesCount--;
		
	}
	
	if (configData.verboseMode) { ROS_WARN("Adding keyframe (%d) at end of function", pose_msg.header.seq); }
	keyframePoses[storedPosesCount] = pose_msg;
	keyframeTypes[storedPosesCount] = fromICP;
	storedPosesCount++;
	
	return true;
	
	
}

void slamNode::trimFeatureTrackVector() {
	
	unsigned int jjj;
	int preservationBuffer = 0;
	int newestSafeIndex = max(lastTestedFrame-preservationBuffer, 0);
	
	for (int iii = 0; iii < ((int)featureTrackVector->size()); iii++) {
		jjj = 0;
		
		while (jjj < featureTrackVector->at(iii).locations.size()) {
			
			bool validImage = false;
			if (((int)featureTrackVector->at(iii).locations.at(jjj).imageIndex) >= newestSafeIndex) {
				validImage = true;
				jjj++;
				continue;
			} 
			
			
			for (unsigned int kkk = 0; kkk < storedPosesCount; kkk++) {
				if (keyframePoses[kkk].header.seq == featureTrackVector->at(iii).locations.at(jjj).imageIndex) {
					validImage = true;
					jjj++;
					break;
				} 
			}
			
			if (validImage) continue;

			for (unsigned int kkk = 0; kkk < initialization_store.keyframes.size(); kkk++) {
				if (initialization_store.keyframes.at(kkk).idx == featureTrackVector->at(iii).locations.at(jjj).imageIndex) {
					validImage = true;
					jjj++;
					break;
				} 
			}
			
			// If image is invalid (not needed)
			featureTrackVector->at(iii).locations.erase(featureTrackVector->at(iii).locations.begin()+jjj);
		}

		if (featureTrackVector->at(iii).locations.size() == 0) {
			featureTrackVector->erase(featureTrackVector->begin()+iii);
			iii--;
		}

	}
}





bool slamNode::findNearestPoses(int& index1, int& index2, ros::Time& targetTime) {
	
	int minPosInd = -1, minNegInd = -1, twoPosInd = -1, twoNegInd = -1;
	
	int posAssigned = 0, negAssigned = 0;
	
	
	double minPosDiff = std::numeric_limits<double>::max(), twoPosDiff = std::numeric_limits<double>::max(), minNegDiff = std::numeric_limits<double>::max(), twoNegDiff = std::numeric_limits<double>::max();
	
	//ROS_WARN("Searching for time (%f) with a total of (%d) poses to check", targetTime.toSec(), poseHistoryCounter);
	
	for (int iii = 0; iii < min(int(poseHistoryCounter), MAX_HISTORY); iii++) {
		
		//ROS_WARN("Testing time (%f) with (%f)", targetTime.toSec(), poseHistoryBuffer[iii % MAX_HISTORY].header.stamp.toSec());
		
		if (poseHistoryBuffer[iii % MAX_HISTORY].header.stamp.toSec() == 0.0) {
			continue;
		}
		
		double diff = targetTime.toSec() - poseHistoryBuffer[iii % MAX_HISTORY].header.stamp.toSec();
		
		//ROS_WARN("diff = (%f)", diff);
		
		//ROS_INFO("diff = (%f); (%f & (%d)%f)", diff, latestTracksTime, iii, poseHistoryBuffer[iii % MAX_HISTORY].header.stamp.toSec());
		
		if (diff > 0.0) {
			if (abs(diff) < minPosDiff) {
				twoPosDiff = minPosDiff;
				minPosDiff = abs(diff);
				twoPosInd = minPosInd;
				minPosInd = iii;
				//ROS_INFO("updating pos index 1: (%d)", iii);
				posAssigned++;
			} else if (posAssigned < 2) {
				twoPosDiff = abs(diff);
				twoPosInd = iii;
				posAssigned++;
				//ROS_INFO("updating pos index 2: (%d)", iii);
			}
		} else if (diff < 0.0) {
			if (abs(diff) < minNegDiff) {
				twoNegDiff = minNegDiff;
				minNegDiff = abs(diff);
				twoNegInd = minNegInd;
				minNegInd = iii;
				//ROS_INFO("updating neg indices");
			} else if (negAssigned < 2) {
				twoNegDiff = abs(diff);
				twoNegInd = iii;
				negAssigned++;
			}
		} else {
			twoNegDiff = minNegDiff;
			minNegDiff = 0.0;
			twoNegInd = minNegInd;
			minNegInd = iii;
			
			twoPosDiff = minPosDiff;
			minPosDiff = 0.0;
			twoPosInd = minPosInd;
			minPosInd = iii;
			//ROS_INFO("updating ALL indices");
		}
		
	}
	
	if ( (minPosInd >= 0) && (minNegInd >= 0) ) { // && (minPosDiff <= twoNegDiff) && (minNegDiff <= twoPosDiff)
		
		index1 = minPosInd;
		index2 = minNegInd;
		
		//ROS_WARN("minPosDiff = (%f), twoPosDiff = (%f), minNegDiff = (%f), twoNegDiff = (%f)", minPosDiff, twoPosDiff, minNegDiff, twoNegDiff);
		
		return true;
		 
	} else {
		
		//ROS_WARN("Entering non-surrounded segment...");
		
		if (minPosDiff >= twoNegDiff) {
			
			index1 = minNegInd;
			index2 = twoNegInd;
			//ROS_WARN("Two negatives... (%d, %d)", index1, index2);
		} else if (minNegDiff >= twoPosDiff) {
			
			index1 = twoPosInd;
			index2 = minPosInd;
			//ROS_WARN("Two positives... (%d, %d)", index1, index2);
		} else {
			ROS_ERROR("Shouldn't be in here!!");
		}
		
	}
	
	return false;
	
}

bool slamNode::updateLocalPoseEstimates() {
	
	double depthTime = poseHistoryBuffer[(poseHistoryCounter-1) % MAX_HISTORY].header.stamp.toSec();
	
	if (0) { ROS_INFO("%s << depthTime = (%f)", __FUNCTION__, depthTime); }
	
	if (0) { ROS_INFO("%s << depthTime = (%d) (%d) (%d)", __FUNCTION__, frameProcessedCounter, frameHeaderHistoryCounter, MAX_HISTORY); }
	
	frameProcessedCounter = max(frameProcessedCounter, frameHeaderHistoryCounter - MAX_HISTORY);
	
	for (unsigned int jjj = frameProcessedCounter; jjj < frameHeaderHistoryCounter; jjj++) { 
	//for (unsigned int jjj = max(frameProcessedCounter, int(frameHeaderHistoryCounter)-MAX_HISTORY+1); jjj < frameHeaderHistoryCounter; jjj++) { 
		
		if (0) { ROS_WARN("Considering with (%d) and frameProcessedCounter (%d)", jjj, frameProcessedCounter); }
		
		int minPosInd = -1, minNegInd = -1;
		if (!findNearestPoses(minPosInd, minNegInd, frameHeaderHistoryBuffer[jjj % MAX_HISTORY].stamp)) {
			if (0) { ROS_WARN("Surrounding depth frames were not able to be found!"); }
			return false;
		}
		
		if (0) { ROS_WARN("Progressing.."); }
		
		geometry_msgs::PoseStamped tracksFrameInterpolatedPose, tracksFrameShiftedPose;
		tracksFrameInterpolatedPose.header = frameHeaderHistoryBuffer[jjj % MAX_HISTORY];
		
		if (0) { ROS_INFO("Considering frame (%d) with time = (%f)", jjj, frameHeaderHistoryBuffer[jjj % MAX_HISTORY].stamp.toSec()); }
		if (0) { ROS_INFO("Best matching indices were (%d) and (%d) : [%f] {%f} [%f]:", minPosInd, minNegInd, poseHistoryBuffer[minPosInd % MAX_HISTORY].header.stamp.toSec(), frameHeaderHistoryBuffer[jjj % MAX_HISTORY].stamp.toSec(), poseHistoryBuffer[minNegInd % MAX_HISTORY].header.stamp.toSec()); }
		
		if (!interpolatePose(poseHistoryBuffer[minPosInd % MAX_HISTORY].pose, poseHistoryBuffer[minPosInd % MAX_HISTORY].header.stamp, poseHistoryBuffer[minNegInd % MAX_HISTORY].pose, poseHistoryBuffer[minNegInd % MAX_HISTORY].header.stamp, tracksFrameInterpolatedPose.pose, frameHeaderHistoryBuffer[jjj % MAX_HISTORY].stamp)) {
			if (configData.verboseMode) { ROS_WARN("Pose unable to be interpolated."); }
			return false;
		} else {
			if (configData.verboseMode) { ROS_WARN("Pose was interpolated."); }
		}
		
		shiftPose(tracksFrameInterpolatedPose.pose, tracksFrameShiftedPose.pose, extrinsicCalib_P);
		
		tracksFrameShiftedPose.header = tracksFrameInterpolatedPose.header;
		
		bool updated = false;
#ifdef _USE_BOOST_
		main_mutex.lock();
		updated = updateKeyframePoses(tracksFrameShiftedPose, true);
		lastTestedFrame = tracksFrameShiftedPose.header.seq;

#if defined(_BUILD_FOR_ROS_) && defined(_USE_SBA_)
		if (configData.publishKeyframes) { drawKeyframes(camera_pub, keyframePoses, storedPosesCount); }
#endif
		main_mutex.unlock();
#endif
		
		if (updated) {
			
#ifdef _USE_BOOST_
			main_mutex.lock();
			if (configData.clearTriangulations) {
				for (unsigned int iii = 0; iii < featureTrackVector->size(); iii++) {
					featureTrackVector->at(iii).isTriangulated = false;
				}
			}
			triangulatePoints();
			main_mutex.unlock();
#endif
		}

		if (0) { ROS_INFO("Updated.. (%d)", updated); }
		frameProcessedCounter++;
	
	}
	
	return true;
	
}

void slamNode::triangulatePoints() {
	if (storedPosesCount >= 2) {
		vector<unsigned int> triangulatableIndices;
		vector<unsigned int> cameraIndices;
		
		for (unsigned int iii = 0; iii < storedPosesCount; iii++) {
			// ROS_INFO("Camera indices to consider includes [%d] : (%u)", iii, keyframePoses[iii].header.seq);
			cameraIndices.push_back(keyframePoses[iii].header.seq);
		}
	
		// ROS_INFO("Finding triangulatable tracks with latest frame of (%d)", keyframePoses[storedPosesCount-1].header.seq);
		//main_mutex.lock();
		
		//maxPairs = itriangle(minProjections-1);
		
		int minProjections_ = minProjections(configData.pairsForTriangulation);
		
		//unsigned int minAppearances = triangle(configData.pairsForTriangulation-1);
		findTriangulatableTracks(*featureTrackVector, triangulatableIndices, cameraIndices, minProjections_);
		//findTriangulatableTracks3(featureTrackVector, triangulatableIndices, keyframePoses[storedPosesCount-1].header.seq, configData.pairsForTriangulation);
		
		
		// unsigned int triangulatableCount = ((int)triangulatableIndices.size());
		
		int minimumlyProjectedTracks = 0;
		for (unsigned int zzz = 0; zzz < featureTrackVector->size(); zzz++) {
			if (int(featureTrackVector->at(zzz).locations.size()) >= minProjections_) { minimumlyProjectedTracks++; }
		}
		
		unsigned int actuallyTriangulated = 0;
		if (triangulatableIndices.size() > 0) {	
			actuallyTriangulated = initialTrackTriangulation(*featureTrackVector, triangulatableIndices, configData.cameraData, keyframePoses, storedPosesCount, configData.minSeparation, configData.maxSeparation, configData.pairsForTriangulation, configData.maxStandardDev, configData.maxReprojectionDisparity);
		}
		
		
		
		if (configData.verboseMode) { ROS_INFO("Keyframe (%d): (%d) points triangulated out of (%d) valid, (%d) potential and (%d) total", keyframePoses[storedPosesCount-1].header.seq, actuallyTriangulated, ((int)triangulatableIndices.size()), minimumlyProjectedTracks, ((int)featureTrackVector->size())); }
		
		//main_mutex.unlock();
	}
}

bool slamNode::determinePose() {
	
	// TO PUBLISH POSE FOR LATEST RECEIVED TRACKS
		// MAY NEED TO EXTRAPOLATE IF THESE TRACKS POSES HAVE NOT BEEN INTERPOLATED
		
	// if (configData.verboseMode) { ROS_INFO("<%s> entered..", __FUNCTION__); } // , frameHeaderHistoryBuffer[(frameHeaderHistoryCounter-1) % MAX_HISTORY].stamp.toSec(), poseHistoryBuffer[(poseHistoryCounter-1) % MAX_HISTORY].header.stamp.toSec());
	
	int idx1, idx2;
	
	bool surrounded = findNearestPoses(idx1, idx2, frameHeaderHistoryBuffer[(frameHeaderHistoryCounter-1) % MAX_HISTORY].stamp);
	
	//ROS_INFO("Found indices (%d, %d) : (%f, %f)", idx1, idx2, poseHistoryBuffer[idx1 % MAX_HISTORY].header.stamp.toSec(), poseHistoryBuffer[idx2 % MAX_HISTORY].header.stamp.toSec());
	
	
	
	if (frameHeaderHistoryBuffer[(frameHeaderHistoryCounter-1) % MAX_HISTORY].stamp.toSec() < poseHistoryBuffer[(poseHistoryCounter-1) % MAX_HISTORY].header.stamp.toSec()) {
		if (surrounded == false) ROS_WARN("No surrounding poses exist for tracks message, however, its timestamp is old, so assuming bag is being looped..");
	} else {
		if (surrounded == true) ROS_WARN("Surrounding poses exist for tracks message, but more recently received pose has very new timestamp, so assuming bag is being looped.."); 
	}
	
	currentPose.header = frameHeaderHistoryBuffer[(frameHeaderHistoryCounter-1) % MAX_HISTORY];
	currentPose.header.frame_id = "/world";
	
	if (surrounded) {
		interpolatePose(poseHistoryBuffer[idx1 % MAX_HISTORY].pose, poseHistoryBuffer[idx1 % MAX_HISTORY].header.stamp, poseHistoryBuffer[idx2 % MAX_HISTORY].pose, poseHistoryBuffer[idx2 % MAX_HISTORY].header.stamp, currentPose.pose, frameHeaderHistoryBuffer[(frameHeaderHistoryCounter-1) % MAX_HISTORY].stamp);	
	} else {
		if (0) { ROS_WARN("No appropriate surrounding frames were found.."); }
		
		if ( (idx1 < 0) || (idx2 < 0) ) {
			if ((configData.verboseMode) && (poseHistoryCounter > 1)) { 
				ROS_ERROR("No pair of indices were able to be found to estimate a position!"); 
				cin.get();
			}
			
			if (currentPose.pose.position.x == std::numeric_limits<float>::max()) {
				return false;
			} 
			//
		} else {
			if (configData.verboseMode) { ROS_WARN("Estimating a position, but not based on surrounding poses..."); }
			interpolatePose(poseHistoryBuffer[idx1 % MAX_HISTORY].pose, poseHistoryBuffer[idx1 % MAX_HISTORY].header.stamp, poseHistoryBuffer[idx2 % MAX_HISTORY].pose, poseHistoryBuffer[idx2 % MAX_HISTORY].header.stamp, currentPose.pose, frameHeaderHistoryBuffer[(frameHeaderHistoryCounter-1) % MAX_HISTORY].stamp);	
		}
		
		// Just use the previous "currentPose" as the best estimate for the current one..
			
	}
	
	savedPose = currentPose;
	
	// Convert: currentPose.pose into usable format
	cv::Mat estimatedPose, t, R, c; 
	Eigen::Quaternion<double> Q;
	
	convertPoseFormat(currentPose.pose, t, Q);
	quaternionToMatrix(Q, R);
	composeTransform(R, t, c);
	
	
	// ROS_ERROR("About to attempt to estimate pose for index (%d) using PnP", frameHeaderHistoryBuffer[(frameHeaderHistoryCounter-1) % MAX_HISTORY].seq);
	
	// int minProjections_ = minProjections(configData.pairsForTriangulation);
	
	framesProcessed++;
	
	pnpError = -1.0;
	pnpInlierProp = -1.0;
	
	bool res = false;
#ifdef _USE_BOOST_
	main_mutex.lock();
	//ROS_WARN("about to <estimatePoseFromKnownPoints> with seq = (%d), lastTestedFrame = (%d), latestHandledTracks = (%d)", frameHeaderHistoryBuffer[(frameHeaderHistoryCounter-1) % MAX_HISTORY].seq, lastTestedFrame, latestHandledTracks);
	res = estimatePoseFromKnownPoints(estimatedPose, configData.cameraData, *featureTrackVector, frameHeaderHistoryBuffer[(frameHeaderHistoryCounter-1) % MAX_HISTORY].seq, c, 1, configData.pnpIterations, configData.maxReprojectionDisparity, configData.inliersPercentage, &pnpError, &pnpInlierProp, configData.debugTriangulation);
	main_mutex.unlock();
#endif

	predictiveError = configData.maxAllowableError;
	
	if (res) {
		
		pnpSuccesses++;
		
	
		
		//cout << "guide = " << c << endl;
		//cout << "estimated = " << estimatedPose << endl << endl;
		
		cv::Mat R_, t_; 
		Eigen::Quaternion<double> Q_;
		
		decomposeTransform(estimatedPose, R_, t_);
		matrixToQuaternion(R_, Q_);
		convertPoseFormat(t_, Q_, currentPose.pose);
		
		pnpPose = currentPose;
		
		if (configData.verboseMode) { ROS_INFO("Pose for index (%d) able to be estimated accurately using PnP... (%f, %f, %f)", frameHeaderHistoryBuffer[(frameHeaderHistoryCounter-1) % MAX_HISTORY].seq, currentPose.pose.position.x, currentPose.pose.position.y, currentPose.pose.position.z); }
		
	} else {
		predictiveError = std::numeric_limits<double>::max();
		if (configData.verboseMode) { ROS_WARN("Pose for index (%d) unable to be estimated accurately using PnP", frameHeaderHistoryBuffer[(frameHeaderHistoryCounter-1) % MAX_HISTORY].seq); }
	}
	
	
	//ROS_ERROR("Base pose #1 (%f, %f, %f) [%f, %f, %f, %f]", poseHistoryBuffer[idx1 % MAX_HISTORY].pose.position.x, poseHistoryBuffer[idx1 % MAX_HISTORY].pose.position.y, poseHistoryBuffer[idx1 % MAX_HISTORY].pose.position.z, poseHistoryBuffer[idx1 % MAX_HISTORY].pose.orientation.w, poseHistoryBuffer[idx1 % MAX_HISTORY].pose.orientation.x, poseHistoryBuffer[idx1 % MAX_HISTORY].pose.orientation.y, poseHistoryBuffer[idx1 % MAX_HISTORY].pose.orientation.z);
	//ROS_ERROR("Base pose #2 (%f, %f, %f) [%f, %f, %f, %f]", poseHistoryBuffer[idx2 % MAX_HISTORY].pose.position.x, poseHistoryBuffer[idx2 % MAX_HISTORY].pose.position.y, poseHistoryBuffer[idx2 % MAX_HISTORY].pose.position.z, poseHistoryBuffer[idx2 % MAX_HISTORY].pose.orientation.w, poseHistoryBuffer[idx2 % MAX_HISTORY].pose.orientation.x, poseHistoryBuffer[idx2 % MAX_HISTORY].pose.orientation.y, poseHistoryBuffer[idx2 % MAX_HISTORY].pose.orientation.z);
	//ROS_ERROR("About to publish pose of (%f, %f, %f) [%f, %f, %f, %f]", currentPose.pose.position.x, currentPose.pose.position.y, currentPose.pose.position.z, currentPose.pose.orientation.w, currentPose.pose.orientation.x, currentPose.pose.orientation.y, currentPose.pose.orientation.z);
	
	
	bundleTransShift = -1.0;
	bundleRotShift = -1.0;
	
	usedTriangulations = -1;
	pointShift = -1.0;
	
#ifdef _USE_SBA_
	if ( (configData.adjustmentIterations > 0) && (storedPosesCount >= 2)) {
		//ROS_INFO("currentPose.seq = (%d); pos = (%f, %f, %f), q = (%f, %f, %f, %f)", currentPose.header.seq, currentPose.pose.position.x, currentPose.pose.position.y, currentPose.pose.position.z, currentPose.pose.orientation.w, currentPose.pose.orientation.x, currentPose.pose.orientation.y, currentPose.pose.orientation.z);

		// ROS_INFO("Have (%d) stored poses.", storedPosesCount);

		main_mutex.lock();
		if (configData.verboseMode) { ROS_WARN("About to perform <predictiveBundleAdjustment> with (%d) cameras...", storedPosesCount); }
		
		if (configData.writePoses) { std::cout.rdbuf( lStream.rdbuf() ); }
		predictiveError = predictiveBundleAdjustment(configData.cameraData, featureTrackVector, keyframePoses, keyframeTypes, storedPosesCount, currentPose, configData.adjustmentIterations, configData.debugSBA, configData.baMode, configData.baStep, &usedTriangulations, &pointShift);
		if (configData.writePoses) { std::cout.rdbuf( lBufferOld ); }
		main_mutex.unlock();

		bundleTransShift = pow(pow(currentPose.pose.position.x-pnpPose.pose.position.x, 2.0) + pow(currentPose.pose.position.y-pnpPose.pose.position.y, 2.0) + pow(currentPose.pose.position.z-pnpPose.pose.position.z, 2.0), 0.5);
		bundleRotShift = pow(pow(currentPose.pose.orientation.w-pnpPose.pose.orientation.w, 2.0) + pow(currentPose.pose.orientation.x-pnpPose.pose.orientation.x, 2.0) + pow(currentPose.pose.orientation.y-pnpPose.pose.orientation.y, 2.0) + pow(currentPose.pose.orientation.z-pnpPose.pose.orientation.z, 2.0), 0.5);
		
		main_mutex.lock();
		filterNearPoints(*featureTrackVector, currentPose.pose.position.x, currentPose.pose.position.y, currentPose.pose.position.z);
		main_mutex.unlock();
		
		
		
		//odometryBundleAdjustment(configData.cameraData, featureTrackVector, keyframePoses, storedPosesCount, configData.adjustmentIterations, configData.debugSBA);

	}
#endif
	
	if (configData.verboseMode) { 
		if (predictiveError > configData.maxAllowableError) {
			ROS_INFO("predictiveError = ( PNP-FAIL )"); 
		} else if (predictiveError == -1.0) {
			ROS_INFO("predictiveError = ( SBA-FAIL )");

		} else if (predictiveError == configData.maxAllowableError) {
			ROS_INFO("predictiveError = ( PNP-ONLY )"); 
		} else {
			ROS_INFO("predictiveError = ( %8.5f )", predictiveError); 
		}	
			

	}
	
	if ((predictiveError < configData.maxAllowableError) && (predictiveError > -1.0)) {
		baAverage *= double(baSuccesses);
		dsAverage *= double(baSuccesses);
		baAverage += predictiveError;
		dsAverage += pow(pow(currentPose.pose.position.x-savedPose.pose.position.x, 2.0)+pow(currentPose.pose.position.y-savedPose.pose.position.y, 2.0)+pow(currentPose.pose.position.z-savedPose.pose.position.z, 2.0),0.5);
		baSuccesses++;
		baAverage /= double(baSuccesses);
		dsAverage /= double(baSuccesses);
	}

	if ( ( (predictiveError > 0.0) && (predictiveError <= configData.maxAllowableError) ) ) { //   || (configData.adjustmentIterations == 0)
		
		return true;
		
	} else {
		return false;
	}
	
	
	
	

}

#ifdef _BUILD_FOR_ROS_
void slamNode::publishPoints(ros::Time stamp, unsigned int seq) {
//void videoslamNode::publishPoints(const geometry_msgs::PoseStamped& pose_msg) {
	vector<cv::Point3d> testPoints3d;
	
	if (0) { ROS_INFO("About to try and extract 3D points.."); }
	
	//main_mutex.lock();
	getPoints3dFromTracks(*featureTrackVector, testPoints3d);
	//main_mutex.unlock();
	
	//transformPoints(testPoints3d, 4);
	//transformPoints(testPoints3d, configData.transformationCode);
	
	
	cv::Point3d midPoint(0.0, 0.0, 0.0);
	for (unsigned int iii = 0; iii < testPoints3d.size(); iii++) {
		//pointsToPublish.push_back(pcl::PointXYZ(testPoints3d.at(iii).x, testPoints3d.at(iii).y, testPoints3d.at(iii).z));
		
		/*
		testPoints3d.at(iii).x = testPoints3d.at(iii).y;
		testPoints3d.at(iii).y = 3.0 - testPoints3d.at(iii).x;
		testPoints3d.at(iii).z = -testPoints3d.at(iii).z;
		*/
		
		// ROS_INFO("Point(%d) = (%f, %f, %f)", iii, testPoints3d.at(iii).x,testPoints3d.at(iii).y, testPoints3d.at(iii).z);
		
		double x, y, z;
		x = testPoints3d.at(iii).x;
		y = testPoints3d.at(iii).y;
		z = testPoints3d.at(iii).z;
		
		midPoint.x += x / double(testPoints3d.size());
		midPoint.y += y / double(testPoints3d.size());
		midPoint.z += z / double(testPoints3d.size());
		
	}
	
	if (0) { ROS_INFO("Cloud midPoint = (%f, %f, %f)", midPoint.x, midPoint.y, midPoint.z); }
	
	/*
	sys.tracks.clear();
	sys.nodes.clear();
	
	SysSBA sys_temp;
	addPointsToSBA(sys_temp, testPoints3d);
	*/

	
	
	
	
	


	pcl::PointCloud<pcl::PointXYZ> pointsToPublish;
	
	for (unsigned int iii = 0; iii < testPoints3d.size(); iii++) {
		pointsToPublish.push_back(pcl::PointXYZ(testPoints3d.at(iii).x, testPoints3d.at(iii).y, testPoints3d.at(iii).z));
	}
	
	//pointsToPublish.push_back(pcl::PointXYZ(configData.x, configData.y, configData.z));
	
	if (configData.verboseMode) { ROS_INFO("Publishing (%d) points", ((int)pointsToPublish.size())); }
	
    //pcl::toROSMsg(pointsToPublish, pointCloud_message);
    //fromPCLPointCloud2(pointsToPublish, pointCloud_message);
    pcl::PCLPointCloud2 tmpCloud;
    toPCLPointCloud2(pointsToPublish, tmpCloud);
    pcl_conversions::fromPCL(tmpCloud, pointCloud_message);
    //pcl::fromPCLPointCloud2(pcl_pc, cloud);
	
    pointCloud_message.header.frame_id = "/world";
    pointCloud_message.header.stamp = stamp; // pose_msg.header.stamp;
	pointCloud_message.header.seq = seq; // pose_msg.header.seq;
	
	//ROS_ERROR("pointCloud_message.size() = (%d, %d)", pointCloud_message.height, pointCloud_message.width);
	
	points_pub.publish(pointCloud_message);
}
#endif

void slamNode::handle_pose(const geometry_msgs::PoseStamped& pose_msg) {
	
	if (wantsToShutdown) return;
	
	//if (configData.verboseMode) { ROS_WARN("Handling mapper pose (%d) at (%f)", pose_msg.header.seq, pose_msg.header.stamp.toSec()); }
	
	if (configData.terminationTime != -1.0) {
		
		if (pose_msg.header.stamp.toSec() > configData.terminationTime) {
			
			if ( (pose_msg.header.stamp.toSec() < configData.restartTime) || (configData.restartTime == -1.0) ) {
				
				if (!hasTerminatedFeed && !configData.writePoses) {
					ROS_ERROR("Terminating feed: incoming poses timestamped after (%f)", configData.terminationTime);
				}
				
				hasTerminatedFeed = true;
				return;
				
			} else {
				
				if (hasTerminatedFeed && !configData.writePoses) {
					ROS_ERROR("Restarting feed: incoming poses timestamped after (%f)", configData.restartTime);
				}
				
				hasTerminatedFeed = false;
			}

			
		}
	}
	
	latestReceivedPoseProcessed = false;
	
	poseHistoryBuffer[poseHistoryCounter % MAX_HISTORY] = pose_msg;
	poseHistoryCounter++;
	
	if (updateLocalPoseEstimates()) {
		if (0) { ROS_INFO("Successfully updated pose estimates..?"); }
	}
	
	latestReceivedPoseProcessed = true;
	
	if (0) { ROS_WARN("frameProcessedCounter = (%d) vs frameHeaderHistoryCounter (%d)", frameProcessedCounter, frameHeaderHistoryCounter); }

}

#ifdef _BUILD_FOR_ROS_
void slamNode::integrateNewTrackMessage(const thermalvis::feature_tracks& msg) {
#else
void slamNode::integrateNewTrackMessage(const vector<featureTrack>& msg) {
#endif

	featureTrack blankTrack;
	unsigned int addedTracks = 0, addedProjections = 0;
	
#ifdef _BUILD_FOR_ROS_
	for (unsigned int iii = 0; iii < msg.projection_count; iii++) {
#else
	for (unsigned int iii = 0; iii < msg.size(); iii++) {
		for (unsigned int jjj = 0; jjj < msg.at(iii).locations.size(); jjj++) {
#endif

#ifdef _BUILD_FOR_ROS_
			int track_idx = msg.indices[iii];
			int camera_idx = msg.cameras.at(iii);
			float proj_x = msg.projections_x.at(iii);
			float proj_y = msg.projections_y.at(iii);
#else
			int track_idx = msg.at(iii).trackIndex;
			int camera_idx = msg.at(iii).locations.at(jjj).imageIndex;
			float proj_x = msg.at(iii).locations.at(jjj).featureCoord.x;
			float proj_y = msg.at(iii).locations.at(jjj).featureCoord.y;
#endif

			latestFrame = max(latestFrame, camera_idx);
		
			int trackPos = findTrackPosition(*featureTrackVector, track_idx);

			if (trackPos == -1) {
			
				// Put the track in, but at the correct spot..
				unsigned int jjj = 0;
			
				if (featureTrackVector->size() > 0) {

					while (featureTrackVector->at(jjj).trackIndex < track_idx) {
						jjj++;
						if (jjj >= featureTrackVector->size()) break;
					}
				}			
			
				blankTrack.trackIndex = track_idx;
				featureTrackVector->insert(featureTrackVector->begin()+jjj, blankTrack);
				addedTracks++;
			
				trackPos = jjj;
			}

			// Now you have the track in place for this projection...
			bool alreadyAdded = false;
			for (unsigned int jjj = 0; jjj < featureTrackVector->at(trackPos).locations.size(); jjj++) {
								
				if (featureTrackVector->at(trackPos).locations.at(jjj).imageIndex == camera_idx) {
					alreadyAdded = true;
					break;
				}		
			}
		
			if (!alreadyAdded) {
				cv::Point2f proj(proj_x, proj_y);		
				indexedFeature newFeature(camera_idx, proj);
			
				//ROS_INFO("Adding point...");
				featureTrackVector->at(trackPos).addFeature(newFeature);
				addedProjections++;
			}
#ifndef _BUILD_FOR_ROS_
		}
#endif
	}
	
	if (configData.debugMode) {
		cv::Mat trackMatrix;
		if (createTrackMatrix(*featureTrackVector, trackMatrix)) {
			cv::imshow("trackMatrix", trackMatrix);
			cv::waitKey(1);
		}		
	}
}

#ifdef _BUILD_FOR_ROS_
void slamNode::publishPose() {
	
	thermalvis::pose_confidence confidence_msg;
	confidence_msg.source = configData.flowSource;
	confidence_msg.header = frameHeaderHistoryBuffer[(frameHeaderHistoryCounter-1) % MAX_HISTORY];
	
	pose_pub.publish(currentPose);
	if (configData.verboseMode) { ROS_INFO("Publishing currentPose (%d) of (%f, %f, %f)", currentPose.header.seq, currentPose.pose.position.x, currentPose.pose.position.y, currentPose.pose.position.z); }
	
	confidence_msg.metric_count = 7;
	
	// First will publish the PnP convergence error
	
	// Second will publish the SBA convergence error (if any)
	if (predictiveError == configData.maxAllowableError) {
		confidence_msg.scores.push_back(-1.0);
		//confidence_msg.data = 0.0;	// This means SBA failed, so PnP estimate only
	} else {
		confidence_msg.scores.push_back(predictiveError);
		//confidence_msg.data = 1.0 - min(predictiveError/3.0,1.0);
	}
	
	confidence_msg.scores.push_back(bundleTransShift);
	confidence_msg.scores.push_back(bundleRotShift);
	confidence_msg.scores.push_back(float(usedTriangulations)); // 4th - not very useful...
	confidence_msg.scores.push_back(pointShift);
	confidence_msg.scores.push_back(pnpError);
	confidence_msg.scores.push_back(pnpInlierProp);
	
	//confidence_msg.header = currentPose.header;
	confidence_pub.publish(confidence_msg);
	
	if (configData.writePoses) { 
		
		printf("%f %d %8.5f %8.5f %8.5f %8.5f %8.5f %8.5f %8.5f", currentPose.header.stamp.toSec(), currentPose.header.seq, currentPose.pose.position.x, currentPose.pose.position.y, currentPose.pose.position.z, currentPose.pose.orientation.w, currentPose.pose.orientation.x, currentPose.pose.orientation.y, currentPose.pose.orientation.z); 
		
		for (unsigned int iii = 0; iii < confidence_msg.metric_count; iii++) {
			printf(" %8.5f", confidence_msg.scores.at(iii));
		}
		
		printf("\n");
		
		}
	
}
#endif
