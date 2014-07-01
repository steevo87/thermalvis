#include "input_stream.hpp"

streamerSharedData::streamerSharedData() : 
	detectorMode(DETECTOR_MODE_INS),
	usbMode(USB_MODE_16),
	inputDatatype(DATATYPE_RAW),
	maxReadAttempts(0), 
	framerate(-1.0),
	normMode(NORM_MODE_STANDARD),
	normFactor(0.0),
	output16bit(false), 
	output8bit(true),
	outputColor(false),
	map(CONFIG_MAP_CODE_CIELUV),
	undistortImages(false), 
	fusionFactor(0.6),  
	serialPollingRate(25.0),
	maxNucInterval(45),
	maxNucThreshold(0.2),
	verboseMode(false), 
	autoTemperature(false),
	minTemperature(20.0), 
	maxTemperature(40.0),
	debugMode(false)
{ }
	
streamerData::streamerData() : 
	filterMode(IMAGE_FILTER_NONE), 
	filterParam(2.0),
	outputType(OUTPUT_TYPE_CV_16UC1)
{ }

bool streamerData::assignFromXml(xmlParameters& xP) {

	int countOfFlowNodes = 0;

	BOOST_FOREACH(boost::property_tree::ptree::value_type &v, xP.pt.get_child("launch")) {
		if (v.first.compare("node")) continue; // only progresses if its a "node" tag
		if (!v.second.get_child("<xmlattr>.type").data().compare("flow")) countOfFlowNodes++;
	}

	if (countOfFlowNodes == 0) {
		ROS_ERROR("No flow nodes found in XML config!");
		return false;
	}

	if (countOfFlowNodes > 1) {
		ROS_ERROR("More than 1 flow node found in XML config! This functionality is not supported in Windows..");
		return false;
	}

	BOOST_FOREACH(boost::property_tree::ptree::value_type &v, xP.pt.get_child("launch")) { // Within tree (pt), finds launch, and loops all tags within it
		if (v.first.compare("node")) continue;
		if (v.second.get_child("<xmlattr>.type").data().compare("flow")) continue;

		BOOST_FOREACH(boost::property_tree::ptree::value_type &v2, v.second) { // Traverses the subtree...
			if (v2.first.compare("param")) continue;

			if (!v2.second.get_child("<xmlattr>.name").data().compare("debugMode")) debugMode = !v2.second.get_child("<xmlattr>.value").data().compare("true");
			if (!v2.second.get_child("<xmlattr>.name").data().compare("verboseMode")) verboseMode = !v2.second.get_child("<xmlattr>.value").data().compare("true");
			
			// ...

        }
	}

	return true;

}

#ifndef _BUILD_FOR_ROS_
streamerConfig::streamerConfig() { }

void streamerConfig::assignStartingData(streamerData& startupData) {

	/*
	maxFeatures = startupData.maxFeatures;
	minFeatures = startupData.minFeatures;
	drawingHistory = startupData.drawingHistory;
	matchingMode = startupData.matchingMode;

	maxFrac = startupData.maxFrac;
	flowThreshold = startupData.flowThreshold;
	minSeparation = startupData.minSeparation;
	maxVelocity = startupData.maxVelocity;
	newFeaturesPeriod = startupData.newFeaturesPeriod;
	delayTimeout = startupData.delayTimeout;
	
	verboseMode = startupData.verboseMode;
	debugMode = startupData.debugMode;
	showTrackHistory = startupData.showTrackHistory;

	adaptiveWindow = startupData.adaptiveWindow;
	velocityPrediction = startupData.velocityPrediction;
	attemptHistoricalRecovery = startupData.attemptHistoricalRecovery;
	autoTrackManagement = startupData.autoTrackManagement;
	attemptMatching = startupData.attemptMatching;
	detectEveryFrame = startupData.detectEveryFrame;

	sensitivity_1 = startupData.sensitivity[0];
	sensitivity_2 = startupData.sensitivity[1];
	sensitivity_3 = startupData.sensitivity[2];

	if ((!startupData.detector[0].compare("FAST")) || (!startupData.detector[0].compare("fast"))) {
		detector_1 = DETECTOR_FAST;
	} else if ((!startupData.detector[0].compare("GFTT")) || (!startupData.detector[0].compare("gftt"))) {
		detector_1 = DETECTOR_GFTT;
	} else if ((!startupData.detector[0].compare("HARRIS")) || (!startupData.detector[0].compare("harris"))) {
		detector_1 = DETECTOR_HARRIS;
	} else {
		ROS_ERROR("Could not identify provided detector..");
		detector_1 = DETECTOR_FAST;
	}

	if ((!startupData.detector[1].compare("FAST")) || (!startupData.detector[1].compare("fast"))) {
		detector_2 = DETECTOR_FAST;
	} else if ((!startupData.detector[1].compare("GFTT")) || (!startupData.detector[1].compare("gftt"))) {
		detector_2 = DETECTOR_GFTT;
	} else if ((!startupData.detector[1].compare("HARRIS")) || (!startupData.detector[1].compare("harris"))) {
		detector_2 = DETECTOR_HARRIS;
	} else {
		detector_2 = DETECTOR_OFF;
	}

	if ((!startupData.detector[2].compare("FAST")) || (!startupData.detector[2].compare("fast"))) {
		detector_3 = DETECTOR_FAST;
	} else if ((!startupData.detector[2].compare("GFTT")) || (!startupData.detector[2].compare("gftt"))) {
		detector_3 = DETECTOR_GFTT;
	} else if ((!startupData.detector[2].compare("HARRIS")) || (!startupData.detector[2].compare("harris"))) {
		detector_3 = DETECTOR_HARRIS;
	} else {
		detector_3 = DETECTOR_OFF;
	}
	*/

}
#endif

inputStream::inputStream() {
	cMapping = new cScheme(DEFAULT_COLORSCHEME_CODE);
	colormap_index = DEFAULT_COLORSCHEME_CODE;

	minTemp = DEFAULT_MIN_TEMP; 
	maxTemp = DEFAULT_MAX_TEMP;

	autoscaleTemps = true;

	rawImage = new cv::Mat();
	scaledImage = new cv::Mat();
	_8bitImage = new cv::Mat();
}

bool inputStream::accessLatestRawFrame(cv::Mat& latestFrame) {
	if (rawImage->rows == 0) return false;
	latestFrame = *rawImage;
	return true;
}

bool inputStream::accessLatest8bitFrame(cv::Mat& latestFrame) {
	if (_8bitImage->rows == 0) return false;
	latestFrame = *_8bitImage;
	return true;
}

void inputStream::displayFrame() {
	if (displayImage->rows != 0) {
		!pauseMode ? cv::imshow("display", *displayImage) : 0;
	} else if (_8bitImage->rows != 0) {
		!pauseMode ? cv::imshow("display", *_8bitImage) : 0;
	} else {
		!pauseMode ? cv::imshow("display", *rawImage) : 0;
	}
	
	char key = cv::waitKey(1);
	if (key == 'q') isValid = false;
}

bool inputStream::processFrame() {

	if (scaledImage->rows == 0) {
		scaledImage = new cv::Mat(rawImage->rows, rawImage->cols, CV_16UC1);
		_8bitImage = new cv::Mat(rawImage->rows, rawImage->cols, CV_8UC1);
		displayImage = new cv::Mat(rawImage->rows, rawImage->cols, CV_8UC3);
	}

	if (autoscaleTemps) {
		normalize_16(*scaledImage, *rawImage);
	} else {
		double minLevel = minTemp * 100.0;
		double maxLevel = maxTemp * 100.0;
		normalize_16(*scaledImage, *rawImage, minLevel, maxLevel);
	}
	
	down_level(*_8bitImage, *scaledImage);

	return 0;
}

bool inputStream::writeImageToDisk() {
	if (wantsToOutput) {
		char imFilename[256];
		sprintf(imFilename, "%s/frame%06d.png", output_directory, FrameCounter1);
		std::string imageFilename(imFilename);

		if (writeInColor && (displayImage->rows != 0)) {
			cv::imwrite(imageFilename, *displayImage);
		} else if (writeInColor && (_8bitImage->rows != 0)) {
			cv::imwrite(imageFilename, *_8bitImage);
		} else {
			cv::imwrite(imageFilename, *rawImage);
		}

		return true;
	}
	return false;
}

#ifdef _BUILD_FOR_ROS_
streamerNode::streamerNode(ros::NodeHandle& nh, streamerData startupData) :
#else
streamerNode::streamerNode(streamerData startupData) :
#endif
	canRadiometricallyCorrect(false),
	deviceCreated(false),
	recordedThermistorReadings(0), 
	settingsDisabled(false), 
	currentNucProtectionMode(false), 
	wantsToDisableNuc(false), 
	lastIsDuplicate(true), 
	firstCall(true), 
	updateNucInterval(true), 
	updateDetectorMode(true), 
	updateUSBMode(true), 
	altStatus(true), 
	performingNuc(false), 
	alphaChanged(true), 
	readyToPublish(false), 
	isActuallyGray(false), 
	videoInitialized(false), 
	videoValid(true), 
	firstFrame(true), 
    centerPrincipalPoint(true), 
	firstServerCallbackProcessed(false), 
	pastMeanIndex(-1), 
	medianPercentile(0.50), 
	alternateCounter(0), 
	fusionFactor(0.8), 
	fileCount(0), 
	writeIndex(0), 
	frameCounter(0), 
	lastWritten(-1), 
	lastThermistorReading(-99.0), 
	lastMedian(-1.0), 
	lastLowerLimit(-1.0), 
	lastUpperLimit(-1.0), 
	oldMaxDiff(-1.0), 
	minVal(65535.0), 
	maxVal(0.0)
{
	
	configData = startupData;
	
#ifdef _BUILD_FOR_ROS_
	dodgeTime.sec = 0;
	dodgeTime.nsec = 0;
#endif

	if (configData.wantsToAddExtrinsics) {
		getRectification();

#ifdef _BUILD_FOR_ROS_
		updateCameraInfoExtrinsics();
#endif
	}
	
	#ifdef _BUILD_FOR_ROS_
	sprintf(nodeName, "%s", ros::this_node::getName().substr(1).c_str());
	#else
	sprintf(nodeName, "/%s", "THERMALVIS_STREAMER");
	#endif

	if (configData.verboseMode) { ROS_INFO("Initializing node (%s)", nodeName); }
	
	if (configData.radiometryFile != "radiometryFile") {
		// Does file exist?
		
		cv::Mat sensorLimits, graylevelLimits, mappingMatrix;
		
		if (configData.verboseMode) { ROS_INFO("About to read from (%s)", configData.radiometryFile.c_str()); }
		
		cv::FileStorage fs(configData.radiometryFile, cv::FileStorage::READ);
		fs["sensorLimits"] >> sensorLimits;
		fs["graylevelLimits"] >> graylevelLimits;
		fs["mappingMatrix"] >> mappingMatrix;
		fs.release();
		
		if ((sensorLimits.rows == 0) || (graylevelLimits.rows == 0) || (mappingMatrix.rows == 0)) {
			ROS_ERROR("Provided radiometry file is invalid. Please check.");
			setValidity(false);
			prepareForTermination();
			return;
		}
		
		radMapper.update(mappingMatrix, sensorLimits.at<double>(0,0), sensorLimits.at<double>(0,1), graylevelLimits.at<double>(0,0), graylevelLimits.at<double>(0,1));
		
		canRadiometricallyCorrect = true;
	
	}
	
#ifndef _WIN32
	if (configData.serialComms) {
		if (configData.verboseMode) { ROS_INFO("Configuring serial comms..."); }
		if (!configureSerialComms()) {
			ROS_ERROR("Serial comms configuration failed. Shutting down.");
			setValidity(false);
			prepareForTermination();
			return;
		}
	}
#endif

	if (configData.verboseMode) { ROS_INFO("configData.serialPollingRate = (%f)", configData.serialPollingRate); }
	
#ifdef _BUILD_FOR_ROS_
	if (configData.serialPollingRate > 0.1) {
		if (configData.verboseMode) { ROS_INFO("Initializing serial timer at (%f)", 1.0 / ((double) configData.serialPollingRate)); }
		serial_timer = nh.createTimer(ros::Duration(1.0 / ((double) configData.serialPollingRate)), &streamerNode::serialCallback, this);
	} else {
		if (configData.verboseMode) { ROS_INFO("Initializing serial timer at (%f)", DEFAULT_SERIAL_POLLING_RATE); }
		serial_timer = nh.createTimer(ros::Duration(DEFAULT_SERIAL_POLLING_RATE), &streamerNode::serialCallback, this);
	}
	
	it = new image_transport::ImageTransport(nh);
#endif

	string info_name = configData.topicname.substr(0, configData.topicname.find_last_of("/") + 1) + "camera_info";
	if (configData.verboseMode) { ROS_INFO("configData.topicname = (%s)", info_name.c_str()); }
	
	if (configData.calibrationMode) {
#ifdef _BUILD_FOR_ROS_
		ros::Duration(1.0).sleep(); // wait a little bit to ensure that the serial comms has been configured..
#endif
	}
	
	if (configData.verboseMode) { ROS_INFO("Initializing camera (%s)", configData.topicname.c_str()); }
	
#ifndef _WIN32
	mainVideoSource = new streamerSource;
#endif

	std::string camera_name;
	
	if (configData.outputFolder.size() == 0) {
		configData.outputFolder = configData.read_addr + configData.outputFolder;
	} else if (configData.outputFolder[0] != '/') {
		configData.outputFolder = configData.read_addr + configData.outputFolder;
	}
	
	configData.outputTimeFile = configData.outputFolder + "-timestamps.txt";
	
	if (configData.wantsToDumpTimestamps) {
		ofs.open(configData.outputTimeFile.c_str());
	}
	
	if (configData.intrinsicsProvided) {
		
		if (configData.verboseMode) { ROS_INFO("Reading in calibration data"); }
		
		if (configData.intrinsics[0] != '/') {
			configData.intrinsics = configData.read_addr + configData.intrinsics;
		}
		
		if (configData.verboseMode) { ROS_INFO("intrinsics = %s", configData.intrinsics.c_str()); }
		
		// http://www.mathpirate.net/log/2009/11/26/when-in-doubt-link-debug/
		cv::Mat dummyImage(300, 300, CV_8UC1);
		GaussianBlur(dummyImage, dummyImage, cv::Size(5, 5), 1.5);
		
		
		cv::FileStorage fs(configData.intrinsics, cv::FileStorage::READ);
		fs["imageSize"] >> globalCameraInfo.imageSize;
		fs["cameraMatrix"] >> globalCameraInfo.cameraMatrix;
		fs["distCoeffs"] >> globalCameraInfo.distCoeffs;
		fs["newCamMat"] >> globalCameraInfo.newCamMat;
		fs.release();
		
		if (configData.verboseMode) { ROS_INFO("Calibration data read."); }

			//HGH
			if (globalCameraInfo.cameraMatrix.empty()){
				ROS_ERROR("Intrinsics file %s invalid! Please check path and filecontent...\n", configData.intrinsics.c_str());
			}
		
		if (configData.verboseMode) { ROS_INFO("Establishing size (%d, %d).", globalCameraInfo.imageSize.at<unsigned short>(0, 0), globalCameraInfo.imageSize.at<unsigned short>(0, 1)); }
		
		globalCameraInfo.cameraSize = cv::Size(globalCameraInfo.imageSize.at<unsigned short>(0, 0), globalCameraInfo.imageSize.at<unsigned short>(0, 1));
		
		//globalCameraInfo.newCamMat = getOptimalNewCameraMatrix(globalCameraInfo.cameraMatrix, globalCameraInfo.distCoeffs, globalCameraInfo.cameraSize, configData.alpha, globalCameraInfo.cameraSize, validPixROI, centerPrincipalPoint);
		
		
		if (configData.verboseMode) { ROS_INFO("Global params determined."); }
		
		
	} else if (configData.imageDimensionsSpecified) {
		
		if (configData.verboseMode) { ROS_INFO("Assigning default intrinsics but with specified image size"); }
		
#ifdef _BUILD_FOR_ROS_
		assignDefaultCameraInfo();
#endif

		globalCameraInfo.imageSize.at<unsigned short>(0, 1) = configData.inputHeight;
		globalCameraInfo.imageSize.at<unsigned short>(0, 0) = configData.inputWidth;
		globalCameraInfo.cameraSize = cv::Size(globalCameraInfo.imageSize.at<unsigned short>(0, 0), globalCameraInfo.imageSize.at<unsigned short>(0, 1));
		
		// ROS_INFO("Assigned.");
		
	} else {
		if (configData.verboseMode) { ROS_INFO("Assigning default intrinsics..."); }

#ifdef _BUILD_FOR_ROS_
		assignDefaultCameraInfo();
#endif

		if (configData.verboseMode) { ROS_INFO("Default intrinsics assigned."); }
	}
	
	
	//HGH
	if (configData.autoAlpha) {
		
		if (configData.intrinsicsProvided) {
			if (configData.verboseMode) { ROS_INFO("Finding auto alpha..."); }
			configData.alpha = findBestAlpha(globalCameraInfo.cameraMatrix, globalCameraInfo.distCoeffs, globalCameraInfo.cameraSize);
			//HGH
			if (configData.verboseMode) { ROS_INFO("Optimal alpha: (%f)", configData.alpha); }
		} else {
			ROS_WARN("Cannot estimate an appropriate alpha coefficient because no intrinsics were provided.");
		}
		
	}

	
	if (configData.wantsToAddExtrinsics) {
		if (configData.verboseMode) { ROS_INFO("Reading in extrinsics..."); }

			char rotation_name[256], translation_name[256];

			sprintf(rotation_name, "R_%d", configData.camera_number);
			sprintf(translation_name, "T%d", configData.camera_number);

			// http://www.mathpirate.net/log/2009/11/26/when-in-doubt-link-debug/
			cv::Mat dummyImage(300, 300, CV_8UC1);
			GaussianBlur(dummyImage, dummyImage, cv::Size(5, 5), 1.5);

			cv::FileStorage fs(configData.extrinsics, cv::FileStorage::READ);

			//HGH
			fs[rotation_name] >> globalCameraInfo.R;
			fs[translation_name] >> globalCameraInfo.T;

			//used to calculate the individual rotation and projection matrices for rectification depending on alpha
			fs["R"] >> globalExtrinsicsData.R;
			fs["T"] >> globalExtrinsicsData.T;

			//we need also both camera matrices and distortion coefficients
			fs["cameraMatrix0"] >> globalExtrinsicsData.cameraMatrix0;
			fs["cameraMatrix1"] >> globalExtrinsicsData.cameraMatrix1;
			fs["distCoeffs0"] >> globalExtrinsicsData.distCoeffs0;
			fs["distCoeffs1"] >> globalExtrinsicsData.distCoeffs1;

	fs.release();

			if (globalExtrinsicsData.R.empty()){
				ROS_ERROR("Extrinsics file %s invalid! Please check path and filecontent...\n", configData.extrinsics.c_str());
			}

			globalCameraInfo.cameraSize = cv::Size(globalCameraInfo.imageSize.at<unsigned short>(0, 0), globalCameraInfo.imageSize.at<unsigned short>(0, 1));

			getRectification();

	} else {

		globalCameraInfo.R = cv::Mat::eye(3, 3, CV_64FC1);
		globalCameraInfo.T = cv::Mat::zeros(3, 1, CV_64FC1);
	}

#ifdef _BUILD_FOR_ROS_
	if (configData.verboseMode) { ROS_INFO("calling assignCameraInfo() from streamerNode()..."); }
	assignCameraInfo();
#endif

	if (configData.verboseMode) { ROS_INFO("Initializing video source..."); }

#ifndef _WIN32
	mainVideoSource->initialize_video_source();
#endif

	colourMap.load_standard(configData.mapCode, configData.mapParam);
	
#ifdef _BUILD_FOR_ROS_
	refreshCameraAdvertisements();
#endif

	char cameraInfoName[256];
	sprintf(cameraInfoName, "thermalvis/%s/set_camera_info", nodeName);

#ifdef _BUILD_FOR_ROS_
	ros::ServiceServer set_camera_info = nh.advertiseService(cameraInfoName, &streamerNode::setCameraInfo, this);
#endif

	if (configData.framerate > 0.0) {
		configData.pauseMode = false;
#ifdef _BUILD_FOR_ROS_
		timer = nh.createTimer(ros::Duration(1.0 / ((double) configData.framerate)), &streamerNode::timerCallback, this);
#endif
	} else {
		configData.pauseMode = true;
#ifdef _BUILD_FOR_ROS_
		timer = nh.createTimer(ros::Duration(1.0 / 1.0), &streamerNode::timerCallback, this);
#endif
	}
	
	// Configure log files
	callLogFile = configData.read_addr + "/nodes/streamer/log/call_log.txt";
	retrieveLogFile = configData.read_addr + "/nodes/streamer/log/retrieve_log.txt";
	internalLogFile = configData.read_addr + "/nodes/streamer/log/internal_log.txt";
	writeLogFile = configData.read_addr + "/nodes/streamer/log/write_log.txt";
	duplicatesLogFile = configData.outputFolder + "-duplicates.txt";
	thermistorLogFile = configData.outputFolder + "-thermistor.txt";
	
#ifdef _BUILD_FOR_ROS_
	if (configData.verboseMode) { ROS_INFO("Establishing server callback..."); }
	f = boost::bind (&streamerNode::serverCallback, this, _1, _2);
    server.setCallback (f);
#endif

#ifdef _BUILD_FOR_ROS_
    if (configData.subscribeMode || configData.resampleMode) {
		if (configData.verboseMode) { ROS_INFO("syncMode: (%d)", configData.syncMode); }
		if (configData.syncMode == SYNCMODE_HARD) {
			camera_sub = it->subscribeCamera(configData.topicname, 1, &streamerNode::handle_camera, this);
		} else if (configData.syncMode == SYNCMODE_SOFT) {
			info_sub = nh.subscribe<sensor_msgs::CameraInfo>(info_name, 1, &streamerNode::handle_info, this);
			image_sub = it->subscribe(configData.topicname, 1, &streamerNode::handle_image, this);
		} else if (configData.syncMode == SYNCMODE_IMAGEONLY) {
			image_sub = it->subscribe(configData.topicname, 1, &streamerNode::handle_image, this);
		}
	}
#endif
	
	if (configData.externalNucManagement != "") {
		
#ifdef _BUILD_FOR_ROS_
		lastFlagReceived = ros::Time::now();
#else
		lastFlagReceived = boost::posix_time::microsec_clock::local_time();
#endif

#ifdef _BUILD_FOR_ROS_
		ROS_INFO("Subscribing to external nuc management flag (%s)", configData.externalNucManagement.c_str());
		nuc_management_sub = nh.subscribe<std_msgs::Float32>(configData.externalNucManagement, 1, &streamerNode::handle_nuc_instruction, this);
#endif
	}
    
    // Wait for the first server callback to be processed before continuing..
    while (!firstServerCallbackProcessed) { };
    
    if (configData.wantsToDumpTimestamps) {
		ofs_call_log.open(callLogFile.c_str());
		ofs_retrieve_log.open(retrieveLogFile.c_str());
		ofs_internal_log.open(internalLogFile.c_str());
		ofs_write_log.open(writeLogFile.c_str());
	}
	
	if (configData.wantsToOutputDuplicates) {
		if (configData.verboseMode) { ROS_INFO("Outputting duplicates to (%s)", duplicatesLogFile.c_str()); }
		ofs_duplicates_log.open(duplicatesLogFile.c_str());
	}
}

void streamerNode::prepareForTermination() {
	if (ofs.is_open()) ofs.close();
	if (ofs_call_log.is_open()) ofs_call_log.close();
	if (ofs_retrieve_log.is_open()) ofs_retrieve_log.close();
	if (ofs_internal_log.is_open()) ofs_internal_log.close();
	if (ofs_write_log.is_open()) ofs_write_log.close();
	if (ofs_duplicates_log.is_open()) ofs_duplicates_log.close();
	if (ofs_thermistor_log.is_open()) ofs_thermistor_log.close();
	
#ifndef _WIN32
	if (configData.serialComms) close(mainfd);
#endif
	
	releaseDevice();	
}

#ifndef _WIN32
bool streamerNode::configureSerialComms() {
	
	
	struct termios options;
   
	mainfd = open_port();
	
	//sleep(2); //required to make flush work, for some reason
	//tcflush(mainfd,TCIOFLUSH);
	
	int fcntl_ret = fcntl(mainfd, F_SETFL, FNDELAY); /* Configure port reading */
	//int fcntl_ret = fcntl(mainfd, F_SETFL, 0);
	
	if (fcntl_ret != 0) { ROS_ERROR("fcntl_ret = (%d)", fcntl_ret); }
	
	//sleep(2); //required to make flush work, for some reason
	//tcflush(mainfd,TCIOFLUSH);
	
	if (configData.verboseMode) { displayTermiosData(options); }
	
	//int tcgetattr_ret = tcgetattr(mainfd, &options); 					/* Get the current options for the port */
	//ROS_ERROR("tcgetattr_ret = (%d)", tcgetattr_ret);
	
	if (0) { // Attempt to write with 1-1 converter
		
        bzero(&options, sizeof(options));
        options.c_cflag = B115200 | CRTSCTS | CS8 | CLOCAL | CREAD; // cflag is common flag?
        options.c_iflag = IGNPAR;
        options.c_oflag = 0;
        
        /* set input mode (non-canonical, no echo,...) */
        options.c_lflag = 0;
         
        options.c_cc[VTIME]    = 0;   /* inter-character timer unused */
        options.c_cc[VMIN]     = 11;   /* blocking read until 5 chars received */
        
        tcflush(mainfd, TCIFLUSH);
        tcsetattr(mainfd,TCSANOW,&options);
        tcflush(mainfd, TCIFLUSH);
	} else {
		// Works for the Serial Hub, and writes properly with 1-1 converter
		
		//bzero(&options, sizeof(options));
		
		//options.c_cflag &= B115200;
		
		int cfsetispeed_ret = cfsetispeed(&options, B115200); /* Set the baud rates to 115200 */ // can change "i" to 2400 and it still works, not so with "o" ...
		if (cfsetispeed_ret != 0) { ROS_ERROR("cfsetispeed_ret = (%d)", cfsetispeed_ret); }
		int cfsetospeed_ret = cfsetospeed(&options, B115200);
		if (cfsetospeed_ret != 0) { ROS_ERROR("cfsetospeed_ret = (%d)", cfsetospeed_ret); }
		
		
		
		// should have 1 stop bit, 115200 baud, 8 bits, no parity, no flow control
		
									   /* Enable the receiver and set local mode */
	    
		
	
		
		// Hack: additional settings to align with GTKTERM:
		
		if (configData.serialCommsConfigurationCode == SERIAL_COMMS_CONFIG_DEFAULT) {
			
			//ROS_ERROR("Assigning here...");
			
			if (1) {
				
				bzero(&options, sizeof(options));
				
				options.c_iflag = 0;
				options.c_oflag = 0;
				options.c_cflag = 0;
				options.c_lflag = 0;
				
				// http://man7.org/linux/man-pages/man3/termios.3.html
				
				//options.c_cflag &= B115200;
				
				cfsetispeed(&options, B115200);
				cfsetospeed(&options, B115200);
		
				//int cfsetispeed_ret = cfsetispeed(&options, B115200); /* Set the baud rates to 115200 */ // can change "i" to 2400 and it still works, not so with "o" ...
				//ROS_ERROR("cfsetispeed_ret = (%d)", cfsetispeed_ret);
				//int cfsetospeed_ret = cfsetospeed(&options, B115200);
				//ROS_ERROR("cfsetospeed_ret = (%d)", cfsetospeed_ret);
				
				
				
				// c_iflag : http://www.delorie.com/gnu/docs/glibc/libc_362.html
				options.c_iflag &= ~INPCK;
				options.c_iflag |= IGNPAR;
				options.c_iflag &= ~PARMRK;
				options.c_iflag &= ~ISTRIP;
				options.c_iflag |= IGNBRK;
				options.c_iflag &= ~BRKINT;
				options.c_iflag &= ~IGNCR;
				options.c_iflag &= ~ICRNL;
				options.c_iflag &= ~INLCR;
				options.c_iflag &= ~IXOFF;
				options.c_iflag &= ~IXON;
				options.c_iflag &= ~IXANY;
				options.c_iflag &= ~IMAXBEL;
				// UNUSED:
				options.c_iflag &= ~IUCLC;
				options.c_iflag &= ~IUTF8;
				
				
				// c_oflag : http://www.delorie.com/gnu/docs/glibc/libc_363.html
				options.c_oflag &= ~OPOST;
				options.c_oflag &= ~ONLCR;
				// UNUSED: oxtabs, onoeot
				options.c_oflag &= ~OLCUC;
				options.c_oflag &= ~OCRNL;
				options.c_oflag &= ~ONOCR;
				options.c_oflag &= ~ONLRET;
				options.c_oflag &= ~OFILL;
				options.c_oflag &= ~OFDEL;
				
				options.c_oflag |= NL0;
				options.c_oflag |= CR0;
				options.c_oflag |= TAB0;
				options.c_oflag |= BS0;
				options.c_oflag |= VT0;
				options.c_oflag |= FF0;
				
				
				// c_cflag : http://www.delorie.com/gnu/docs/glibc/libc_364.html
				options.c_cflag |= CLOCAL;
				options.c_cflag &= ~HUPCL;
				options.c_cflag |= CREAD;
				options.c_cflag &= ~CSTOPB; 
				options.c_cflag &= ~PARENB;
				options.c_cflag &= ~PARODD;
				options.c_cflag &= ~CSIZE;
				options.c_cflag |= CS8;
				// UNUSED: cs5, cs6, cs7, ccts_oflow, crts_iflow, mdmbuf, cignore
				//options.c_cflag &= ~CRTSCTS;
				options.c_cflag &= ~CRTSCTS;
				
				
				// c_lflag : http://www.delorie.com/gnu/docs/glibc/libc_365.html
				options.c_lflag &= ~ICANON;
				options.c_lflag &= ~ECHO;
				options.c_lflag &= ~ECHOE;
				options.c_lflag &= ~ECHOPRT;
				options.c_lflag &= ~ECHOK;
				options.c_lflag &= ~ECHOKE;
				options.c_lflag &= ~ECHONL;
				options.c_lflag &= ~ECHOCTL;
				options.c_lflag &= ~ISIG;
				options.c_lflag &= ~IEXTEN;
				options.c_lflag &= ~NOFLSH;
				options.c_lflag &= ~TOSTOP;
				// UNUSED: altwerase, flusho, nokerninfo, pendin
				options.c_lflag &= ~XCASE;		
				
				options.c_ispeed = B115200;
				options.c_ospeed = B115200;
				
				
				options.c_cc[0] = 79;
				options.c_cc[1] = 215;
				options.c_cc[2] = 212;
				options.c_cc[3] = 255;
				options.c_cc[4] = 127;
				
				options.c_cc[5] = 0;
				options.c_cc[6] = 0;
				options.c_cc[7] = 88;
				options.c_cc[8] = 42;
				options.c_cc[9] = 121;
				
				options.c_cc[10] = 106;
				options.c_cc[11] = 96;
				options.c_cc[12] = 127;
				options.c_cc[13] = 0;
				options.c_cc[14] = 0;
				
				options.c_cc[15] = 104;
				options.c_cc[16] = 41;
				options.c_cc[17] = 121;
				options.c_cc[18] = 106;
				options.c_cc[19] = 96;
				
				options.c_cc[20] = 127;
				options.c_cc[21] = 0;
				options.c_cc[22] = 0;
				options.c_cc[23] = 16;
				options.c_cc[24] = 16;
				
				options.c_cc[25] = 121;
				options.c_cc[26] = 106;
				options.c_cc[27] = 96;
				options.c_cc[28] = 127;
				options.c_cc[29] = 0;
				
				options.c_cc[30] = 0;
				options.c_cc[31] = 144;
				
				
				//options.c_cc[VTIME]    = 0;   /* inter-character timer unused */
				//options.c_cc[VMIN]     = 1;   /* blocking read until 1 char received */

				tcflush(mainfd, TCOFLUSH );
				tcflush(mainfd, TCIFLUSH);
				
				int yes = 1;
				
				ioctl(mainfd, TIOCCONS, &yes);
				
				// Taken from : http://tldp.org/HOWTO/Serial-Programming-HOWTO/x115.html
						
			} else {
				
				//ROS_ERROR("Old ones...");
				
				options.c_cflag |= (CLOCAL | CREAD);
				options.c_cflag &= ~PARENB; /* Mask the character size to 8 bits, no parity */ // changed to PARENB (no effect in streamer)
				options.c_cflag |= CSTOPB;
				options.c_cflag &= ~CSIZE;
				options.c_cflag |=  CS8; /* Select 8 data bits */ // changing to 7 breaks...
				options.c_cflag &= ~CRTSCTS; /* Disable hardware flow control */  
				
				// Try to set some of these...
				options.c_oflag = 0;
			 
											 /* Enable data to be processed as raw input */
				options.c_lflag &= ~(ICANON | ECHO | ISIG);
				//options.c_lflag = 0;
				
				options.c_cflag &= ~CSTOPB;
				options.c_cflag &= ~HUPCL;
				
				
				options.c_cflag &= IGNBRK;
				options.c_cflag &= IGNPAR;	
				//options.c_cflag |= IGNBRK;
				//options.c_cflag |= IGNPAR;	
				
				options.c_cflag &= ~ICRNL;
				options.c_cflag &= ~IXON;
				options.c_cflag &= ~IEXTEN;
				options.c_cflag &= ~ECHOE;
				options.c_cflag &= ~ECHOK;
				options.c_cflag &= ~ECHOCTL;
				options.c_cflag &= ~ECHOKE;
				
				
			}
			

			
					
		} else {
			if (configData.verboseMode) { ROS_INFO("Using modified serial comms configuration settings..."); }
		}
		
		if (configData.verboseMode) { displayTermiosData(options); }
		   
		/* Set the new options for the port */
		//int tcsetattr_ret = tcsetattr(mainfd, TCSAFLUSH, &options);
		int tcsetattr_ret = tcsetattr(mainfd, TCSANOW, &options);
		
		
		if (tcsetattr_ret != 0) { ROS_ERROR("tcsetattr_ret = (%d)", tcsetattr_ret); }
		
		//tcsetattr(mainfd, TCSAFLUSH, &options);
		//sleep(2); //required to make flush work, for some reason
		tcflush(mainfd,TCIOFLUSH);
		
	}
	
	
	
	
	
	{
		if (configData.verboseMode) { ROS_INFO("SerialComms: Opening shutter..."); }
		char localCommand[256];
		sprintf(localCommand, "open");
		if (!sendSerialCommand(localCommand, configData.serialWriteAttempts)) ROS_WARN("Serial command (%s) failed after (%d) attempts", localCommand, configData.serialWriteAttempts);
	}
	
	{
		// Trying to clear serial comms buffer...
		int n;
		char buff[SERIAL_BUFF_SIZE];	
		n = read(mainfd, buff, SERIAL_BUFF_SIZE);
		
		if (configData.verboseMode) { ROS_INFO("Cleared (%d) characters from buffer: (%s)", n, buff); }
		
	}
	
	int statusNum = 0;
	
	
	
	if (configData.disableSkimming) {
		// Check current SKIMACTIVE status
		char localCommand[256];
		sprintf(localCommand, "SKIMACTIVE");
		
		if (1) { ROS_INFO("SerialComms: Disabling mean-shifting..."); }
		if (!sendSerialCommand(localCommand, 1)) ROS_WARN("Serial command (%s) failed after (%d) attempts", localCommand, 1);
		
		ros::Duration(1.0).sleep();
		
		char buff[SERIAL_BUFF_SIZE];	
		read(mainfd, buff, SERIAL_BUFF_SIZE);
		//ROS_WARN("Message received = (%s)\n", buff);
		//printf("%s\n", buff);
		
		//istringstream ss_buff( &buff[103]);
		
		statusNum = atof(&buff[17]);
		if (configData.verboseMode) { ROS_INFO("SKIMACTIVE status = (%d)", statusNum); }
		
	}
	
	ros::Duration(1.0).sleep();
					
	if (configData.disableSkimming) {
		char localCommand[256];
		sprintf(localCommand, "SKIMACTIVE 0");
		
		if (configData.verboseMode) { ROS_INFO("SerialComms: Verifying disabling of mean-shifting..."); }
		if (!sendSerialCommand(localCommand, 1)) ROS_WARN("Serial command (%s) failed after (%d) attempts", localCommand, 1);

	}
	
	{
		char localCommand[256];
		sprintf(localCommand, "SAVE");
		
		if (configData.verboseMode) { ROS_INFO("SerialComms: Saving current settings..."); }
		if (!sendSerialCommand(localCommand, 1)) ROS_WARN("Serial command (%s) failed after (%d) attempts", localCommand, 1);
		
		
	}
	
	int n;
	char buff[SERIAL_BUFF_SIZE];
	
	n = read(mainfd, buff, SERIAL_BUFF_SIZE);
	
	// if has 5 characters, is probably just " DSP:> "
	if ((configData.verboseMode) && (n != 0)) { ROS_WARN("read(): n = (%d)", n); }
	
	sleep(2); //required to make flush work, for some reason
	tcflush(mainfd,TCIOFLUSH);

	//ROS_WARN("Attempting initial reading of thermistor, 1 failure is expected..");
	// getThermistorReading();
	
	//sleep(2); //required to make flush work, for some reason
	//tcflush(mainfd,TCIOFLUSH);
	
	//sleep(2); //required to make flush work, for some reason
	// tcflush(mainfd,TCIOFLUSH);
	
	// Ensure shutter is open...
	
	//n = read(mainfd, buff, SERIAL_BUFF_SIZE);
	
	//sleep(2); //required to make flush work, for some reason
	//tcflush(mainfd,TCIOFLUSH);
	
	//getThermistorReading();
	
	//sleep(2); //required to make flush work, for some reason
	//tcflush(mainfd,TCIOFLUSH);
	
	// Ensure shutter is open...
	
	//n = read(mainfd, buff, SERIAL_BUFF_SIZE);
	
	if (configData.disableSkimming) {
		if (statusNum == 1) {
			ROS_ERROR("Camera must now be powercycled: SKIMACTIVE was not originally off.");
			return false;
		} else {
			ROS_WARN("Assuming camera has been powercycled since SKIMACTIVE was saved as off.");
			return true;
		}		
	} else {
		return true;
	}

}
#endif

void streamerNode::getRectification(){

    cv::Mat Q;
    cv::stereoRectify(globalExtrinsicsData.cameraMatrix0, globalExtrinsicsData.distCoeffs0, globalExtrinsicsData.cameraMatrix1, globalExtrinsicsData.distCoeffs1,
                                globalCameraInfo.cameraSize,
                                globalExtrinsicsData.R, globalExtrinsicsData.T,
                                globalExtrinsicsData.R0, globalExtrinsicsData.R1, globalExtrinsicsData.P0, globalExtrinsicsData.P1,
                                Q,
                                cv::CALIB_ZERO_DISPARITY, configData.alpha, globalCameraInfo.cameraSize, &globalExtrinsicsData.roi0, &globalExtrinsicsData.roi1);

}

void streamerNode::releaseDevice() {
	
	if (!deviceCreated) { return; }
	
	setValidity(false);
	
	if ((configData.inputDatatype == DATATYPE_8BIT) || (configData.inputDatatype == DATATYPE_MM)) {
		getVideoCapture()->release();
	} else if (configData.inputDatatype == DATATYPE_RAW) {
		
		#ifdef _BUILD_FOR_ROS_
		getMainVideoSource()->close_video_capture();
		#endif
	}
	
	deviceCreated = false;
}

#ifdef _BUILD_FOR_ROS_
void streamerNode::serverCallback(thermalvis::streamerConfig &config, uint32_t level) {
#else
void streamerNode::serverCallback(streamerConfig &config) {
#endif

	if (configData.verboseMode){ ROS_INFO("Processing reconfigure request..."); }
	
	//configData.radiometricCorrection = config.radiometricCorrection;
	//configData.radiometricRaw = config.radiometricRaw;
	configData.minTemperature = config.minTemperature;
	configData.maxTemperature = config.maxTemperature;
	//configData.radiometricBias = config.radiometricBias;
	
	//configData.alreadyCorrected = config.alreadyCorrected;
	
	//configData.stepChangeTempScale = config.stepChangeTempScale;
	
	if (configData.autoTemperature != config.autoTemperature) {
		lastMinDisplayTemp = -9e99, lastMaxDisplayTemp = 9e99;
		configData.autoTemperature = config.autoTemperature;
	}
	
	
	
	//configData.smoothThermistor = config.smoothThermistor;
	//configData.thermistorWindow = config.thermistorWindow;
	
	//configData.radiometricInterpolation = config.radiometricInterpolation;
	
	if (configData.detectorMode != config.detectorMode) {
		
		configData.detectorMode = config.detectorMode;
		
		if (!configData.serialComms) {
			ROS_WARN("Selecting a detector mode has no effect unless serial comms are enabled.");
		} else {
			updateDetectorMode = true;
		}
		
		
	}
	
	
	configData.usbMode = config.usbMode;
	if (!configData.serialComms) {
		if (configData.usbMode != config.usbMode) {
			ROS_WARN("Selecting a USB mode has no effect unless serial comms are enabled.");
		}
	} else {
		
		updateUSBMode = true;
		
		if (configData.usbMode == USB_MODE_16) {
			if (configData.verboseMode) { ROS_INFO("changing <configData.inputDatatype> to DATATYPE_RAW"); }
			config.inputDatatype = DATATYPE_RAW;
			if (strcmp(configData.outputTypeString.c_str(), "CV_8UC3")) { configData.outputTypeString = "CV_16UC1"; }
		} else if (configData.usbMode == USB_MODE_8) {
			config.inputDatatype = DATATYPE_8BIT;
			if (strcmp(configData.outputTypeString.c_str(), "CV_8UC3")) { configData.outputTypeString = "CV_8UC1"; }
		}
	
		
	}
	
	if (configData.serialPollingRate != config.serialPollingRate) {
		configData.serialPollingRate = config.serialPollingRate;
		if (configData.verboseMode) { ROS_INFO("Updating period with rate (%f)", configData.serialPollingRate); }
		
#ifdef _BUILD_FOR_ROS_
		if (configData.serialPollingRate > 0.1) {
			serial_timer.setPeriod(ros::Duration(1.0 / configData.serialPollingRate));
		} else {
			serial_timer.setPeriod(ros::Duration(1.0 / 1.0));
		}
	
#endif
	}

	if (configData.maxNucInterval != config.maxNucInterval) {
		
		configData.maxNucInterval = config.maxNucInterval;
		ROS_WARN("NUC interval changed by config...");
		updateNucInterval = true;
	}
	
	if (configData.maxNucThreshold != config.maxNucThreshold) {
		
		configData.maxNucThreshold = config.maxNucThreshold;
		updateNucInterval = true;
	}
	
	
	if (!configData.temporalSmoothing) {
		lastMedian = -1.0;
	}
	
	configData.verboseMode = config.verboseMode;
	
	configData.normFactor = config.normFactor;

	if (config.framerate != 0.0) {
		configData.pauseMode = false;
	}
		
	if (alphaChanged) {
		if (configData.verboseMode) { ROS_INFO("Updating map..."); }
		updateMap();
	}
	
	fusionFactor = config.fusionFactor;
	
	//ROS_WARN("Progressing...");
            
	if (config.framerate < 0.0) {
		
		if (configData.pollMode) {
		
			ROS_WARN("Invalid framerate (%f) so switching to capture mode.", config.framerate);
			configData.framerate = DEFAULT_READ_RATE;
			configData.pollMode = false;
			configData.captureMode = true;
			
		} else if (configData.readMode) {
			
			ROS_WARN("Invalid framerate (%f) so defaulting to (%f).", config.framerate, DEFAULT_READ_RATE);
			configData.framerate = DEFAULT_READ_RATE;

		} else if (configData.loadMode) {
			
			ROS_WARN("Invalid framerate (%f) so defaulting to (%f).", config.framerate, DEFAULT_READ_RATE);
			configData.framerate = DEFAULT_READ_RATE;
			
		} else if (configData.resampleMode) {
			
			ROS_WARN("Invalid framerate (%f) so switching to subscribe mode.", config.framerate);
			configData.framerate = DEFAULT_READ_RATE;
			configData.resampleMode = false;
			configData.subscribeMode = true;
			
		}

#ifdef _BUILD_FOR_ROS_
		timer.setPeriod(ros::Duration(1.0 / DEFAULT_READ_RATE));
#endif
	} else {
		
		if (configData.captureMode) {
		
			ROS_INFO("Specified framerate (%f) so switching to poll mode.", config.framerate);
			configData.framerate = config.framerate;
			configData.captureMode = false;
			configData.pollMode = true;
			
		} else if (configData.subscribeMode) {
			
			ROS_INFO("Specified framerate (%f) so switching to resample mode.", config.framerate);
			configData.framerate = config.framerate;
			configData.subscribeMode = false;
			configData.resampleMode = true;
			
		}
		
		if (config.framerate > 0.0) {
#ifdef _BUILD_FOR_ROS_
			timer.setPeriod(ros::Duration(1.0 / config.framerate));
#endif
		} else {
			configData.pauseMode = true;
#ifdef _BUILD_FOR_ROS_
			timer.setPeriod(ros::Duration(1.0 / 1.0));
#endif
		}
		
		
	}
	
	if (config.inputDatatype != configData.inputDatatype) {
		if (configData.readMode) {
			ROS_WARN("You cannot change the bit-reading mode mid-stream. Consider restarting.");
		} else if ((configData.captureMode) || (configData.pollMode)) {
			if (configData.verboseMode) { ROS_INFO("Resetting device in order to change streaming mode to (%d)...", config.inputDatatype); }
			releaseDevice();
			if (configData.verboseMode) { ROS_INFO("device released..."); }
			configData.inputDatatype = config.inputDatatype;
			if (configData.verboseMode) { ROS_INFO("Changing to (%d)", configData.inputDatatype); }
			setupDevice();
			if (configData.verboseMode) { ROS_INFO("Set up done."); }
		} else {
			configData.inputDatatype = config.inputDatatype;
		}

	}
	
	//configData.filterMode = config.filterMode;
	//configData.filterParam = config.filterParam;
	
	//configData.maxReadAttempts = config.maxReadAttempts;
	
	configData.normMode = config.normMode;
	
	bool wantsToRefreshCameras = false;
	
	if (config.output16bit && !configData.output16bitFlag) {
		configData.output16bitFlag = true;
		wantsToRefreshCameras = true;
	} else if (!config.output16bit && configData.output16bitFlag) {
		configData.output16bitFlag = false;
		wantsToRefreshCameras = true;
	}
	
	if (config.output8bit && !configData.output8bitFlag) {
		configData.output8bitFlag = true;
		wantsToRefreshCameras = true;
	} else if (!config.output8bit && configData.output8bitFlag){
		configData.output8bitFlag = false;
		wantsToRefreshCameras = true;
	}
	
	if (config.outputColor && !configData.outputColorFlag) {
		configData.outputColorFlag = true;
		wantsToRefreshCameras = true;
	} else if (!config.outputColor && configData.outputColorFlag) {
		configData.outputColorFlag = false;
		wantsToRefreshCameras = true;
	}
	
#ifdef _BUILD_FOR_ROS_
	if (wantsToRefreshCameras) refreshCameraAdvertisements();
#endif
            
	getMapping(config.map, configData.extremes, configData.mapCode, configData.mapParam);
	colourMap.load_standard(configData.mapCode, configData.mapParam);
    
	if (configData.outputFolder == "outputFolder") {
		
		ROS_WARN("No valid output folder specified...");
		configData.wantsToWrite = false;
		configData.wantsToDumpTimestamps = false;
	}
	
       //HGH
        //configData.wantsToRectify = config.rectifyImages;

        if (config.undistortImages) {
                //if (map1.rows == 0) {
                    //HGH  initUndistortRectifyMap(globalCameraInfo.cameraMatrix, globalCameraInfo.distCoeffs, globalCameraInfo.R, globalCameraInfo.newCamMat, globalCameraInfo.cameraSize, CV_32FC1, map1, map2);
                    //HGH
                    if (configData.wantsToAddExtrinsics){
                        if (configData.wantsToRectify){
                            if (configData.camera_number == 0){
                                cv::initUndistortRectifyMap(globalCameraInfo.cameraMatrix, globalCameraInfo.distCoeffs, globalExtrinsicsData.R0, globalExtrinsicsData.P0, globalCameraInfo.cameraSize, CV_32FC1,  map1, map2);
                            }else if (configData.camera_number == 1){
                                cv::initUndistortRectifyMap(globalCameraInfo.cameraMatrix, globalCameraInfo.distCoeffs, globalExtrinsicsData.R1, globalExtrinsicsData.P1, globalCameraInfo.cameraSize, CV_32FC1,  map1, map2);
                            }
                            //initUndistortRectifyMap(globalCameraInfo.cameraMatrix, globalCameraInfo.distCoeffs, globalCameraInfo.R, globalCameraInfo.newCamMat, globalCameraInfo.cameraSize, CV_16SC2/*CV_32FC1*/, map1, map2);
                        }else{
                            cv::initUndistortRectifyMap(globalCameraInfo.cameraMatrix, globalCameraInfo.distCoeffs,  cv::Mat(), globalCameraInfo.newCamMat, globalCameraInfo.cameraSize, CV_32FC1, map1, map2);
                        }
                    }else{
                        cv::initUndistortRectifyMap(globalCameraInfo.cameraMatrix, globalCameraInfo.distCoeffs,  cv::Mat(), globalCameraInfo.newCamMat, globalCameraInfo.cameraSize, CV_32FC1, map1, map2);
                    }
                  //  }
		}

        configData.wantsToUndistort = config.undistortImages;
	
	
	if (!firstServerCallbackProcessed) {
		firstServerCallbackProcessed = true;
	}
	
	if (configData.verboseMode) { ROS_INFO("Reconfigure request complete..."); }
}


void getMapping(int mapIndex, bool extremes, int& mapCode, int& mapParam) {
	
	switch (mapIndex) {
	case 0:
		mapCode = GRAYSCALE;
		break;
	case 1:
		mapCode = CIECOMP;
		break;
	case 2:
		mapCode = BLACKBODY;
		break;
	case 3:
		mapCode = RAINBOW;
		break;
	case 4:
		mapCode = IRON;
		break;
	case 5:
		mapCode = BLUERED;
		break;
	case 6:
		mapCode = JET;
		break;
	case 7:
		mapCode = CIELUV;
		break;
	case 8:
		mapCode = ICEIRON;
		break;
	case 9:
		mapCode = ICEFIRE;
		break;
	case 10:
		mapCode = REPEATED;
		break;
	case 11:
		mapCode = HIGHLIGHTED;
		break;
	case 12:
		mapCode = GRAYSCALE;
		break;
	default:
		mapCode = CIELUV;
	}
		
	extremes ? mapParam = 0 : mapParam = 1;
	
}

bool streamerNode::setupDevice() {
	
	if (configData.verboseMode) { ROS_INFO("Entered setupDevice()..."); }
	
	if (deviceCreated) { return false; }
	
	//ROS_WARN("%d ; %s", configData.device_num, configData.capture_device.c_str());
	
	colourMat.release();
	
	if ((configData.inputDatatype == DATATYPE_8BIT) || (configData.inputDatatype == DATATYPE_MM)) {
		
		if (configData.verboseMode) { ROS_INFO("Setting up device in 8-bit/MM mode..."); }
		getVideoCapture()->open(configData.device_num);
	} else if (configData.inputDatatype == DATATYPE_RAW) {
		if (configData.verboseMode) { ROS_INFO("Setting up device in 16-bit mode..."); }
		
		#ifdef _BUILD_FOR_ROS_
		int deviceWidth, deviceHeight;
		getMainVideoSource()->setup_video_capture(configData.capture_device.c_str(), deviceWidth, deviceHeight, configData.verboseMode);
		
		// Now use this opportunity to test/correct?
		
		if ((!configData.imageDimensionsSpecified) && (!configData.intrinsicsProvided)) {
			// Use these values..
			ROS_INFO("Using image size learned from device initialization (%d, %d)", deviceWidth, deviceHeight);
			globalCameraInfo.imageSize.at<unsigned short>(0, 0) = deviceWidth;
			globalCameraInfo.imageSize.at<unsigned short>(0, 1) = deviceHeight;
			globalCameraInfo.cameraSize = cv::Size(globalCameraInfo.imageSize.at<unsigned short>(0, 0), globalCameraInfo.imageSize.at<unsigned short>(0, 1));
	
			overwriteCameraDims();
	
		} else {
			// Compare the values to what you are using, and produce a warning if necessary...
			
			if ((globalCameraInfo.cameraSize.width != deviceWidth) || (globalCameraInfo.cameraSize.height != deviceHeight)) {
				ROS_WARN("Device-based image size estimates (%d, %d) conflict with provided image size (%d, %d)", deviceWidth, deviceHeight, globalCameraInfo.cameraSize.width, globalCameraInfo.cameraSize.height);
			}
			
		}
		#endif
	}
	
	if (configData.verboseMode) { ROS_INFO("Device set up!"); }
	
	setValidity(true);
	deviceCreated = true;
	return true;
}

void streamerNode::updateMap() {
	
	if (configData.verboseMode) { ROS_INFO("Updating map..."); }
	
	if (configData.autoAlpha) {
		configData.alpha = findBestAlpha(globalCameraInfo.cameraMatrix, globalCameraInfo.distCoeffs, globalCameraInfo.cameraSize);
		
                if (configData.verboseMode) { ROS_INFO("Optimal alpha: (%f)", configData.alpha); }

	}
	
	cv::Rect* validPixROI = 0;
	globalCameraInfo.newCamMat = getOptimalNewCameraMatrix(globalCameraInfo.cameraMatrix, globalCameraInfo.distCoeffs, globalCameraInfo.cameraSize, configData.alpha, globalCameraInfo.cameraSize, validPixROI, centerPrincipalPoint);
	
        //HGH initUndistortRectifyMap(globalCameraInfo.cameraMatrix, globalCameraInfo.distCoeffs, globalCameraInfo.R, globalCameraInfo.newCamMat, globalCameraInfo.cameraSize, CV_32FC1, map1, map2);
        //HGH
        if (configData.wantsToAddExtrinsics){

            //update camera info extrinsics...
            getRectification();

#ifdef _BUILD_FOR_ROS_
            updateCameraInfoExtrinsics();
#endif
            if (configData.wantsToRectify){
                if (configData.camera_number == 0){
                    cv::initUndistortRectifyMap(globalCameraInfo.cameraMatrix, globalCameraInfo.distCoeffs, globalExtrinsicsData.R0, globalExtrinsicsData.P0, globalCameraInfo.cameraSize, CV_32FC1,  map1, map2);
                }else if (configData.camera_number == 1){
                    cv::initUndistortRectifyMap(globalCameraInfo.cameraMatrix, globalCameraInfo.distCoeffs, globalExtrinsicsData.R1, globalExtrinsicsData.P1, globalCameraInfo.cameraSize, CV_32FC1,  map1, map2);
                }
            }else{
                cv::initUndistortRectifyMap(globalCameraInfo.cameraMatrix, globalCameraInfo.distCoeffs,  cv::Mat(), globalCameraInfo.newCamMat, globalCameraInfo.cameraSize, CV_32FC1, map1, map2);
            }

        }else{
            cv::initUndistortRectifyMap(globalCameraInfo.cameraMatrix, globalCameraInfo.distCoeffs,  cv::Mat() , globalCameraInfo.newCamMat, globalCameraInfo.cameraSize, CV_32FC1, map1, map2);
        }

	alphaChanged = false;
	
	if (configData.verboseMode) { ROS_INFO("Map updated."); }
}

