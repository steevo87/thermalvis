#include "streamer/input_stream.hpp"

#ifdef _BUILD_FOR_ROS_

bool streamerNode::runBag() {
	
	if (configData.subscribeMode) {
		ROS_INFO("Subscription started...");
	} else if (configData.resampleMode) ROS_INFO("Resampling subscription started...");

	while (isVideoValid()) ros::spinOnce();
	
	if (configData.subscribeMode) {
		ROS_INFO("Subscription terminating...");
	} else if (configData.resampleMode) ROS_INFO("Resampling subscription terminating...");
	
	return true;
}

void streamerNode::updateCameraInfoExtrinsics() {
    if (configData.verboseMode) { ROS_INFO("Updating camera info..."); }
    if (configData.camera_number == 0){
        for (unsigned int iii = 0; iii < 3; iii++) {
            for (unsigned int jjj = 0; jjj < 4; jjj++) camera_info.P[iii*4 + jjj] = globalExtrinsicsData.P0.at<double>(iii,jjj);
        }
    } else if (configData.camera_number == 1){
        for (unsigned int iii = 0; iii < 3; iii++) {
            for (unsigned int jjj = 0; jjj < 4; jjj++) camera_info.P[iii*4 + jjj] = globalExtrinsicsData.P1.at<double>(iii,jjj);
        }
    }
    if (configData.verboseMode) { ROS_INFO("Camera info updated."); }
}

bool streamerNode::setCameraInfo(sensor_msgs::SetCameraInfo::Request &req, sensor_msgs::SetCameraInfo::Response &rsp) {

	camera_info = req.camera_info;

	if (configData.verboseMode) { ROS_INFO("calling assignCameraInfo() from setCameraInfo()..."); }
	assignCameraInfo();

	if (camera_calibration_parsers::writeCalibrationIni("../camera_parameters.txt", "gscam", camera_info)) {
		ROS_INFO("Camera information written to camera_parameters.txt");
		return true;
	} else {
		ROS_ERROR("Could not write camera_parameters.txt");
		return false;
	}
}

void streamerNode::acceptImage(void *ptr) {
	if ((temperatureMat.rows == 0) && canRadiometricallyCorrect) temperatureMat = cv::Mat::zeros(camera_info.height, camera_info.width, CV_32FC1);
	
	if (configData.inputDatatype == DATATYPE_RAW) {
		if ((frame.rows == 0) || (frame.type() != CV_16UC1)) frame = cv::Mat::zeros(camera_info.height, camera_info.width, CV_16UC1);

		if (configData.verboseMode){ ROS_INFO("Copying image data to internal matrix... (%d, %d)", camera_info.width, camera_info.height); }
		memcpy(&(frame.at<unsigned char>(0,0)), ptr, camera_info.width*camera_info.height*2);
		if (configData.verboseMode){ ROS_INFO("Data copied."); }

	} else if (configData.inputDatatype == DATATYPE_8BIT) {

		if ((frame.rows == 0) || (frame.type() != CV_8UC1)) frame = cv::Mat::zeros(camera_info.height, camera_info.width, CV_8UC1);
		
		if (configData.verboseMode){ ROS_INFO("Copying image data to internal matrix.... (%d, %d)", camera_info.width, camera_info.height); }
		memcpy(&(frame.at<unsigned char>(0,0)), ptr, camera_info.width*camera_info.height);
		if (configData.verboseMode){ ROS_INFO("Data copied."); }
		
	} else if (configData.inputDatatype == DATATYPE_MM){
		if ((frame.rows == 0) || (frame.type() != CV_8UC3)) frame = cv::Mat::zeros(camera_info.height, camera_info.width, CV_8UC3);
		
		if (configData.verboseMode){ ROS_INFO("Copying image data to internal matrix... (%d, %d)", camera_info.width, camera_info.height); }
		memcpy(&(frame.at<unsigned char>(0,0)), ptr, camera_info.width*camera_info.height*3);
		if (configData.verboseMode){ ROS_INFO("Data copied."); }
	}

	if (configData.fixDudPixels) {
		if (configData.verboseMode){ ROS_INFO("Fixing dud pixels..."); }
		fix_bottom_right(frame);
		if (configData.verboseMode){ ROS_INFO("Dud pixels fixed."); }
	}
}

void streamerNode::initializeMessages() {
	
	if (configData.output16bit) {
		if (msg_16bit.width == 0) {
			if (_16bitMat.rows != 0) {
				msg_16bit.width = _16bitMat.cols; 
				msg_16bit.height = _16bitMat.rows;
				msg_16bit.encoding = "mono16";
				msg_16bit.is_bigendian = false;
				msg_16bit.step = _16bitMat.cols*2;
				msg_16bit.data.resize(_16bitMat.cols*_16bitMat.rows*2);
			}			
		}
	}
	
	if (configData.output8bit) {
		if (msg_8bit.width == 0) {
			if (_8bitMat.rows != 0) {
				msg_8bit.width = _8bitMat.cols; 
				msg_8bit.height = _8bitMat.rows;
				msg_8bit.encoding = "mono8";
				msg_8bit.is_bigendian = false;
				msg_8bit.step = _8bitMat.cols*1;
				msg_8bit.data.resize(_8bitMat.cols*_8bitMat.rows*1);
			}
		}
	}
	
	if (configData.outputColor) {
		if (msg_color.width == 0) {
			if (colourMat.rows != 0) {
				msg_color.width = colourMat.cols; 
				msg_color.height = colourMat.rows;
				msg_color.encoding = "bgr8";
				msg_color.is_bigendian = false;
				msg_color.step = colourMat.cols*3;
				msg_color.data.resize(colourMat.cols*colourMat.rows*3);
			}
		}
		
	}
}

bool streamerNode::performNuc() {
	
	if (configData.verboseMode) { ROS_INFO("Sending NUC command..."); }
	
	char localCommand[256];
	sprintf(localCommand, "nuc");
	if (!sendSerialCommand(localCommand, configData.serialWriteAttempts)) ROS_WARN("Serial command (%s) failed after (%d) attempts", localCommand, configData.serialWriteAttempts);
	
	return true;
}

void streamerNode::updateThermistor() {
	newThermistorReading = getThermistorReading();
	if (!((newThermistorReading < MAX_VALID_THERMISTOR) && (newThermistorReading > MIN_VALID_THERMISTOR))) return;
	
	if (!((lastThermistorReading == -99.0) || ( (lastThermistorReading != -99.0) && (abs(lastThermistorReading - newThermistorReading) < configData.maxThermistorDiff) ) )) return;
	
	// Update thermistor reading buffer
	thermistorBuffer[recordedThermistorReadings % MAX_THERMISTOR_READINGS_TO_STORE][0] = ros::Time::now().toSec();
	thermistorBuffer[recordedThermistorReadings % MAX_THERMISTOR_READINGS_TO_STORE][1] = newThermistorReading;
	recordedThermistorReadings++;
	
	if (configData.smoothThermistor) {
		double smoothReading = smoothThermistorReading();
		
		if ((smoothReading < MAX_VALID_THERMISTOR) && (smoothReading > MIN_VALID_THERMISTOR)) {
			if ((abs(smoothReading - newThermistorReading) < configData.maxThermistorDiff)) newThermistorReading = smoothReading;
		}
	}

	lastThermistorReading = newThermistorReading;

}

void streamerNode::calibrationModeRoutine() {
	if (!(alternateCounter >= configData.alternatePeriod)) {
		alternateCounter++;
		return;
	}
		
	char localCommand[256];

	if (configData.calibrationMode == CALIBMODE_ALT_OUTPUT) {
		(altStatus) ? sprintf(localCommand, "vidout raw") : sprintf(localCommand, "vidout ins");
		altStatus = !altStatus;
	} else if (configData.calibrationMode == CALIBMODE_ALT_SHUTTER) {
		if (performingNuc) {
			if (configData.verboseMode) { ROS_INFO("About to switch shutter"); }
			(altStatus) ? sprintf(localCommand, "close") : sprintf(localCommand, "open");
			altStatus = !altStatus;
		} else {
			if (configData.verboseMode) { ROS_INFO("About to perform a NUC"); }
			sprintf(localCommand, "nuc");
		}
		performingNuc = !performingNuc;
	} else if (configData.calibrationMode == CALIBMODE_LONG_SHUTTER) {
		
		if (configData.verboseMode) { ROS_INFO("About to switch shutter"); }
		(altStatus) ? sprintf(localCommand, "close") : sprintf(localCommand, "open");
		altStatus = !altStatus;
	}
	
	if (configData.verboseMode) { ROS_INFO("Sending SPECIAL command: (%s)", localCommand); }
	if (!sendSerialCommand(localCommand, configData.serialWriteAttempts)) ROS_WARN("Serial command (%s) failed after (%d) attempts", localCommand, configData.serialWriteAttempts);
								
	alternateCounter = 0;
}

void streamerNode::serialCallback(const ros::TimerEvent&) {
	
	if (!configData.serialComms) return;
	
	if (configData.serialPollingRate == 0.0) {
		if (configData.verboseMode) { ROS_INFO("Skipping serial callback because desired frequency is zero (0)."); }
		return;
	}
	
	if (configData.readThermistor) updateThermistor(); 
	if (configData.calibrationMode > 0) calibrationModeRoutine();
	
	bool wantsToPerformNuc = false;
	
	if (currentNucProtectionMode && currentDesiredNucProtectionMode && (abs(ros::Time::now().toSec() - lastFlagReceived.toSec()) > MAX_TIME_WITHOUT_FLAGS) ) {
		if (configData.verboseMode) { ROS_INFO("Timer exceeded for disabled NUC mode, preparing to restore NUC settings..."); }
		currentDesiredNucProtectionMode = false;
		updateNucInterval = true;
		wantsToPerformNuc = true;
	}
	
	if (currentNucProtectionMode && !currentDesiredNucProtectionMode) {
		if (abs(ros::Time::now().toSec() - lastNucPerformed_at_the_earliest.toSec()) > MAX_TIME_WITHOUT_NUC_conservative) wantsToPerformNuc = true;
	}
	
	if (wantsToPerformNuc) {
		lastNucPerformed_at_the_earliest = ros::Time::now();
		performNuc();
	}
	
	if (updateDetectorMode) {
		
		string detectorCode;
		
		switch (configData.detectorMode) {
			case DETECTOR_MODE_RAW:
				detectorCode = "raw";
				break;
			case DETECTOR_MODE_LUM:
				detectorCode = "lum";
				break;
			case DETECTOR_MODE_INS:
				detectorCode = "ins";
				break;
			case DETECTOR_MODE_RAD:
				detectorCode = "rad";
				break;
			case DETECTOR_MODE_TMP:
				detectorCode = "t";
				break;
		}
		
		if (configData.verboseMode) { ROS_INFO("Changing detector output..."); }
		char localCommand[256];
		sprintf(localCommand, "vidout %s", detectorCode.substr(0,3).c_str());
		if (!sendSerialCommand(localCommand, configData.serialWriteAttempts)) ROS_WARN("Serial command (%s) failed after (%d) attempts", localCommand, configData.serialWriteAttempts);

		char buff[SERIAL_BUFF_SIZE];
		int n = read(mainfd, buff, SERIAL_BUFF_SIZE);
		
		updateDetectorMode = false;
	}
	
	if (updateUSBMode) {
		
		string usbCode;

		if (configData.usbMode == USB_MODE_16) {
			usbCode = "1";
		} else if (configData.usbMode == USB_MODE_8) usbCode = "2";
		
		
		if (configData.verboseMode) { ROS_INFO("Changing usb output with command..."); }
		char localCommand[256];
		sprintf(localCommand, "usb %s", usbCode.substr(0,usbCode.size()).c_str());
		if (!sendSerialCommand(localCommand, configData.serialWriteAttempts)) ROS_WARN("Serial command (%s) failed after (%d) attempts", localCommand, configData.serialWriteAttempts);
		
		char buff[SERIAL_BUFF_SIZE];
		int n = read(mainfd, buff, SERIAL_BUFF_SIZE);
		
		// ROS_INFO("Returned is <%s>", buff);
		
		updateUSBMode = false;
	}
	
	if (updateNucInterval) {
		
		//ROS_ERROR("ENTERED AGAIN!");
		
		// Value is in 100ths of a degree centigrade, so pass 20 to trigger change every 0.2 C
		
		double valid_thermLimit;
		int maxInterval;
		
		if (configData.verboseMode) { ROS_INFO("Currently in protection mode (%d), want to be in mode (%d)", currentNucProtectionMode, currentDesiredNucProtectionMode); }
		
		if (currentDesiredNucProtectionMode && !currentNucProtectionMode) {
			valid_thermLimit = 0.0;
			maxInterval = 0;
			if (configData.verboseMode) { ROS_WARN("Disabling NUC settings..."); }
			
		} else if (currentDesiredNucProtectionMode && currentNucProtectionMode) {
			ROS_ERROR("Shouldn't be in here! (%d, %d)", currentNucProtectionMode, currentDesiredNucProtectionMode);
		} else {
			valid_thermLimit = float(int(round(100*configData.maxNucThreshold)))/100.0;
			maxInterval = configData.maxNucInterval;
			if (configData.verboseMode) { ROS_INFO("Updating NUC settings... (%d, %f)", maxInterval, valid_thermLimit); }
			settingsDisabled = false;
		}
		
		if (configData.verboseMode) { ROS_INFO("Sending NUCSET command..."); }
		char localCommand[256];
		sprintf(localCommand, "nucset %d %d %d %.2f", maxInterval, 0, 0, valid_thermLimit);
		if (!sendSerialCommand(localCommand, configData.serialWriteAttempts)) ROS_WARN("Serial command (%s) failed after (%d) attempts", localCommand, configData.serialWriteAttempts);
		
		char buff[SERIAL_BUFF_SIZE];
		int n = read(mainfd, buff, SERIAL_BUFF_SIZE);
		
		currentNucProtectionMode = currentDesiredNucProtectionMode;

		updateNucInterval = false;
	}
}

void streamerNode::overwriteCameraDims() {
	
	camera_info.height = globalCameraInfo.imageSize.at<unsigned short>(0, 1);
	camera_info.width = globalCameraInfo.imageSize.at<unsigned short>(0, 0);
	
}

void streamerNode::handle_nuc_instruction(const std_msgs::Float32::ConstPtr& nuc_msg) {
	
	if (configData.verboseMode) { ROS_INFO("Handling NUC instruction: (%f)", nuc_msg->data); }
	lastFlagReceived = ros::Time::now();
	currentDesiredNucProtectionMode = (nuc_msg->data < 0.5);
	if (currentDesiredNucProtectionMode != currentNucProtectionMode) updateNucInterval = true;
}

void streamerNode::handle_info(const sensor_msgs::CameraInfoConstPtr& info_msg) {
	
	if (configData.syncMode != SYNCMODE_SOFT) return;
	if ((!configData.subscribeMode) && (!configData.resampleMode)) return;
	
	if (configData.verboseMode) { ROS_INFO("Copying camera info over..."); }
	
	original_camera_info = *info_msg;
	
	info_time = ros::Time::now();
	original_time = info_msg->header.stamp;
	originalInternalTime = info_msg->R[0];
	
	original_bx = info_msg->binning_x;
	original_by = info_msg->binning_y;
	
	if (std::abs(image_time.toNSec() - info_time.toNSec()) < configData.soft_diff_limit) {
		image_time = dodgeTime;
		act_on_image();
	}
}

void streamerNode::refreshCameraAdvertisements() {

	char colorPubName[256], _16bitPubName[256], _8bitPubName[256];

	sprintf(colorPubName, "thermalvis/%s/image_col", nodeName);
	sprintf(_16bitPubName, "thermalvis/%s/image_raw", nodeName);
	sprintf(_8bitPubName, "thermalvis/%s/image_mono", nodeName);

	bool cameraAdvertised = false;

	// Once all copies of the returned Publisher object are destroyed, the topic
	// will be automatically unadvertised.

	pub_color.shutdown();
	pub_color_im.shutdown();
	pub_16bit.shutdown();
	pub_16bit_im.shutdown();
	pub_8bit.shutdown();
	pub_8bit_im.shutdown();

	//HGH
	pub_republish.shutdown();
	pub_republish_im.shutdown();

	char _republishPubName[256];
	sprintf(_republishPubName, "%s", configData.republishTopic.c_str());

	if (configData.output16bit) {

	//HGH
	if (configData.republishSource==REPUBLISH_CODE_16BIT) pub_republish = it->advertiseCamera(_republishPubName, 1);
		if (!cameraAdvertised) {
			pub_16bit = it->advertiseCamera(_16bitPubName, 1);
			cameraAdvertised = true;
		} else pub_16bit_im = it->advertise(_16bitPubName, 1);
	}

	if (configData.output8bit) {
		//HGH
		if (configData.republishSource==REPUBLISH_CODE_8BIT_MONO) pub_republish = it->advertiseCamera(_republishPubName, 1);
		if (!cameraAdvertised) {
				pub_8bit = it->advertiseCamera(_8bitPubName, 1);
				cameraAdvertised = true;
		} else pub_8bit_im = it->advertise(_8bitPubName, 1);
	}


	if (configData.outputColor) {
		//HGH
		if (configData.republishSource==REPUBLISH_CODE_8BIT_COL) pub_republish = it->advertiseCamera(_republishPubName, 1);
		if (!cameraAdvertised) {
			pub_color = it->advertiseCamera(colorPubName, 1);
			cameraAdvertised = true;
		} else pub_color_im = it->advertise(colorPubName, 1);
	}
}

void streamerNode::handle_image(const sensor_msgs::ImageConstPtr& msg_ptr) {
	
	if (configData.syncMode == SYNCMODE_HARD) return;
	if ((!configData.subscribeMode) && (!configData.resampleMode)) return;
	
	original_time = msg_ptr->header.stamp;
	
	image_time = ros::Time::now();
	cv_ptr = cv_bridge::toCvCopy(msg_ptr, enc::BGR8); // For some reason it reads as BGR, not gray
	
	if (configData.syncMode == SYNCMODE_SOFT) {
		if (std::abs(image_time.toNSec() - info_time.toNSec()) < configData.soft_diff_limit) {
			image_time = dodgeTime;
			act_on_image();
		}
	} else act_on_image();
}

bool streamerData::obtainStartingData(ros::NodeHandle& nh) {

    nh.param<bool>("verboseMode", verboseMode, false);
	
	nh.param<std::string>("source", source, "dev");
	
	nh.param<std::string>("file", file, "file");
	nh.param<std::string>("dev", capture_device, "/dev/video0");
	device_num = atoi(&(capture_device.c_str(), capture_device.at(capture_device.length()-1)));
	
	nh.param<std::string>("topic", topicname, "/thermalvis/streamer/image_raw");
	nh.param<std::string>("externalNucManagement", externalNucManagement, "");
	
	
	nh.param<bool>("serialFeedback", serialFeedback, false);
	
	nh.param<double>("maxThermistorDiff", maxThermistorDiff, 0.5);
	
	nh.param<int>("outputFormat", outputFormat, 4);
	
	nh.param<int>("temporalMemory", temporalMemory, 5);
	
	nh.param<int>("serialWriteAttempts", serialWriteAttempts, 1);
	//nh.param<int>("serialReadAttempts", serialReadAttempts, 1);
	
	nh.param<int>("alternatePeriod", alternatePeriod, 5);
	
	switch (outputFormat) {
		case 0:
			outputFormatString = "jpg";
			break;
		case 1:
			outputFormatString = "pgm";
			break;
		case 2:
			outputFormatString = "bmp";
			break;
		case 3:
			outputFormatString = "ppm";
			break;
		case 4:
			outputFormatString = "png";
			break;
	}
	
    //std::string outputTypeString;
	
    nh.param<int>("outputType", outputType, 2);
	
    //if ((outputTypeString == "CV_8UC3") && (outputFormatString == "pgm")) ROS_WARN("PGM cannot write as CV_8UC3...");
    //if ((outputTypeString == "CV_16UC1") && ((outputFormatString == "jpg") || (outputFormatString == "bmp"))) ROS_WARN("JPG/BMP cannot write as CV_16UC1...");
    //if ((outputTypeString != "CV_8UC3") && (outputTypeString != "CV_8UC1") && (outputTypeString != "CV_16UC1")) ROS_WARN("Unrecognized output format (%s)", outputTypeString.c_str());
	
	nh.param<int>("maxIntensityChange", maxIntensityChange, 2);

    nh.param<double>("degreesPerGraylevel", degreesPerGraylevel, DEFAULT_DEGREES_PER_GRAYLEVEL);
    nh.param<double>("desiredDegreesPerGraylevel", desiredDegreesPerGraylevel, DEFAULT_DESIRED_DEGREES_PER_GRAYLEVEL);

	nh.param<bool>("drawReticle", drawReticle, false);
	nh.param<bool>("displayThermistor", displayThermistor, false);
	nh.param<bool>("outputDuplicates", outputDuplicates, false);
	nh.param<bool>("fixDudPixels", fixDudPixels, true);
	nh.param<bool>("forceInputGray", forceInputGray, false);
	nh.param<bool>("stepChangeTempScale", stepChangeTempScale, false);
	nh.param<bool>("extremes", extremes, true);
	nh.param<bool>("temporalSmoothing", temporalSmoothing, false);
	nh.param<bool>("markDuplicates", wantsToMarkDuplicates, false);
	nh.param<bool>("smoothThermistor", smoothThermistor, false);
	nh.param<double>("thermistorWindow", thermistorWindow, 5.0);
	
	nh.param<bool>("readThermistor", readThermistor, true);
	
	nh.param<int>("maxReadAttempts", maxReadAttempts, 0);
	
	nh.param<int>("filterMode", filterMode, 0);
	nh.param<double>("filterParam", filterParam, 2.0);
	
	nh.param<std::string>("radiometryFile", radiometryFile, "radiometryFile");
	
	nh.param<double>("syncDiff", syncDiff, 0.005);
	
	soft_diff_limit = (unsigned long) (syncDiff * 1000000000.0);
	
	nh.param<std::string>("portAddress", portAddress, "/dev/ttyUSB0");
	
	nh.param<bool>("autoAlpha", autoAlpha, true);
	nh.param<double>("alpha", alpha, 0.00);
	
	nh.param<int>("serialCommsConfigurationCode", serialCommsConfigurationCode, SERIAL_COMMS_CONFIG_DEFAULT);

	nh.param<bool>("useCurrentRosTime", useCurrentRosTime, false);

	//nh.param<int>("map", map, 0);
	//nh.param<bool>("extremes", extremes, true);
	nh.param<std::string>("intrinsics", intrinsics, "intrinsics");
	
	nh.param<int>("inputWidth", inputWidth, 0);
	nh.param<int>("inputHeight", inputHeight, 0);
	
	nh.param<std::string>("extrinsics", extrinsics, "extrinsics");
	
	//nh.param<int>("inputDatatype", inputDatatype, 1);
	
    nh.param<int>("camera_number", camera_number, 0);
	nh.param<double>("framerate", framerate, -1.0);
	
	nh.param<double>("writeQuality", writeQuality, 1.0);
	
	nh.param<bool>("output16bit", output16bit, true);
	nh.param<bool>("output8bit", output8bit, false);
	nh.param<bool>("outputColor", outputColor, true);
	
	nh.param<bool>("loopMode", loopMode, false);
	
	nh.param<bool>("disableSkimming", disableSkimming, true);
	
	nh.param<std::string>("outputFolder", outputFolder, "outputFolder");
	
	nh.param<bool>("serialComms", serialComms, false);
	nh.param<bool>("radiometricCorrection", radiometricCorrection, true);
	nh.param<bool>("alreadyCorrected", alreadyCorrected, false);
	nh.param<bool>("radiometricRaw", radiometricRaw, false);
	nh.param<bool>("radiometricInterpolation", radiometricInterpolation, true);
	
	nh.param<int>("radiometricBias", radiometricBias, 0);
	
	nh.param<int>("calibrationMode", calibrationMode, CALIBMODE_OFF);
	
	ROS_INFO("calibrationMode = (%d)", calibrationMode);
	
    if ((outputFolder.size() > 0) && (outputFolder != "outputFolder")) {
        if (outputFolder.at(outputFolder.size()-1) == '/') outputFolder = outputFolder.substr(0, outputFolder.size()-1);
        writeImages = true;
        if (verboseMode) ROS_INFO("Will write to outputFolder = (%s)", outputFolder.c_str());
        char folderCommand[256];
        sprintf(folderCommand, "mkdir -p %s", outputFolder.c_str());
        if (verboseMode && (system(folderCommand) == 0)) ROS_WARN("Failed to create necessary output folder... perhaps it already exists!");
    }

    nh.param<bool>("writeImages", writeImages, writeImages); // Still check if user doesn't want to write images from the very beginning
	
	outputFileParams.clear();
	int val;
	if (outputFormatString == "png") {
		outputFileParams.push_back(CV_IMWRITE_PNG_COMPRESSION);
		val = int((1.0-writeQuality) * 9.0);
		outputFileParams.push_back(val);
	} else if (outputFormatString == "jpg") {
		outputFileParams.push_back(CV_IMWRITE_JPEG_QUALITY);
		val = int(writeQuality * 100.0);
		outputFileParams.push_back(val);
	}
		
	nh.param<bool>("keepOriginalNames", wantsToKeepNames, false);
	nh.param<bool>("loopMode", loopMode, false);
	
	nh.param<bool>("writeVideo", wantsToEncode, false);
	nh.param<std::string>("outputVideo", outputVideo, "outputVideo");
	nh.param<std::string>("videoType", videoType, "videoType");
		
	nh.param<bool>("wantsToUndistort", wantsToUndistort, false);
        //HGH
        nh.param<bool>("rectifyImages", wantsToRectify, false);
	
	nh.param<bool>("removeDuplicates", removeDuplicates, false);
	
	if (outputFolder != "outputFolder") {
		nh.param<bool>("dumpTimestamps", dumpTimestamps, false);
	} else dumpTimestamps = false;
	
	nh.param<bool>("resizeMode", wantsToResize, false);
	
	nh.param<int>("syncMode", syncMode, SYNCMODE_HARD);

	// "hardSync", int_t, 0, "Image and camera_info topics must be fully synchronized"),
	// "softSync", int_t, 1, "Image and camera_info topics do not have to be fully synchronized"),
	// "imageOnly", int_t, 2, "To be used when no camera_info topic is present") ],

	nh.param<int>("rows", desiredRows, -1);
	nh.param<int>("cols", desiredCols, -1);
	
	if (wantsToResize) {
		if ((desiredRows < 1) || (desiredRows > MAX_ROWS) || (desiredCols < 1) || (desiredCols > MAX_COLS)) {
			ROS_ERROR("Resizing values (%d, %d) invalid.",desiredCols, desiredRows);
			return false;
			
		} else ROS_INFO("Resizing to (%d x %d)", desiredCols, desiredRows);
	}
	
	if (wantsToUndistort) { ROS_INFO("Undistorting images..."); }
	
	nh.param<int>("normMode", normMode, NORM_MODE_FIXED_TEMP_RANGE);
	nh.param<double>("normFactor", normFactor, 0.0);
	
	if (wantsToEncode) {
        if (verboseMode) { ROS_INFO("Has chosen to encode."); }
		
		if (outputVideo == "outputVideo") {
			ROS_ERROR("outputVideo incorrectly specified...");
			return false;
		} else ROS_INFO("outputVideo = (%s)", outputVideo.c_str());
		
		if (verboseMode) { ROS_INFO("Image format = (%s); image type = (%s)", "avi", videoType.c_str()); }
	}
	
	if (verboseMode) { 
		switch (inputDatatype) {
			case DATATYPE_8BIT:
				ROS_INFO("Streaming mode: 8-bit");
				break;
			case DATATYPE_RAW:
				ROS_INFO("Streaming mode: 16-bit");
				break;
			case DATATYPE_MM:
				ROS_INFO("Streaming mode: multimodal");
				break;
		}
	}
	
	if (source == "dev") {
		
		if ((framerate < 0) || (framerate > MAX_READ_RATE)) {
			if (verboseMode) { ROS_INFO("Framerate set to (%f) so therefore defaulting to capture mode.", framerate); }
			framerate = MAX_READ_RATE;
			captureMode = true;
		} else {
			if (verboseMode) { ROS_INFO("Requested framerate (%f) - polling mode", framerate); }
			pollMode = true;
		}
		
		ROS_INFO("Reading from device (%s)", capture_device.c_str());
		
	} else if (source == "file") {
		readMode = true;
		
		if ((framerate < 0.0) || (framerate > MAX_READ_RATE)) {
			ROS_INFO("Invalid framerate (%f) so defaulting to (%f).", framerate, DEFAULT_READ_RATE);
			framerate = DEFAULT_READ_RATE;
		} else ROS_INFO("Requested framerate = %f", framerate);
		
		ROS_INFO("Reading from a file (%s)", file.c_str());
	} else if ((source == "folder") || (source == "directory")) {
		
		loadMode = true;
		
		nh.param<std::string>("folder", folder, "folder");
		nh.param<std::string>("directory", folder, "folder");
		
		if (folder[0] == '~') {
			folder = folder.substr(1, folder.size()-1);
			folder = _USERPROFILE_ + folder;
		}
		
		for (int iii = 0; iii < folder.size(); iii++) if (folder[iii] == '\\') folder[iii] = '/';
		
		if ((framerate < 0.0) || (framerate > MAX_READ_RATE)) {
			ROS_WARN("Invalid framerate (%f) so defaulting to (%f).", framerate, DEFAULT_READ_RATE);
			framerate = DEFAULT_READ_RATE;
		} else ROS_INFO("Requested framerate = %f", framerate);
		
		ROS_INFO("Loading images from a folder (%s)", folder.c_str());
		
	} else if (source == "topic") {
		
		ROS_INFO("Subscribing to topic (%s) ", topicname.c_str());
		
		if ((framerate < 0.0) || (framerate > MAX_READ_RATE)) {
			ROS_WARN("Invalid framerate (%f) so defaulting to subscribe mode.", framerate);
			framerate = DEFAULT_READ_RATE;
			subscribeMode = true;
		} else {
			ROS_INFO("Requested framerate = %f (resample mode)", framerate);
			resampleMode = true;
		}
		
	}
	
	//maxIntensityChange = 100 / DEFAULT_READ_RATE;
	
	if (loopMode == true) ROS_INFO("Option to loop has been selected.");
	
	if (intrinsics != "intrinsics") {
		intrinsicsProvided = true;
		if (verboseMode) { ROS_INFO("Intrinsics at (%s) selected.", intrinsics.c_str()); }
		
		if ((inputWidth != 0) && (inputHeight != 0)) ROS_WARN("Provided image dimensions will be ignored because of provided intrinsics file.");
	} else {
		
		if ((inputWidth != 0) && (inputHeight != 0)) {
			ROS_INFO("Provided image dimensions (%d, %d) will be used.", inputWidth, inputHeight);
			imageDimensionsSpecified = true;
		} else ROS_WARN("No intrinsics or image dimensions provided. Will attempt to estimate camera size...");
		
		//intrinsics = read_addr + "data/calibration/csiro-aslab/miricle-1.yml";
		//ROS_ERROR("No intrinsics supplied. Defaulting to (%s)", intrinsics.c_str());
	}
	
	if (extrinsics != "extrinsics") {
		ROS_INFO("Extrinsics at %s selected.", extrinsics.c_str());

		//HGH
		//wantsToRectify = false;

		//HGH
		wantsToAddExtrinsics = true;
		if (camera_number < 0) {
			ROS_WARN("Invalid camera number selected (%d) so defaulting to (0).", camera_number);
			camera_number = 0;
		} else ROS_INFO("Camera number (%d).", camera_number);	
	}

	//getMapping(map, extremes, mapCode, mapParam);
		
	int modeCount = 0;
	
	if (captureMode) modeCount++;
	if (pollMode) modeCount++;
	if (readMode) modeCount++;
	if (loadMode) modeCount++;
	if (subscribeMode) modeCount++;
	if (resampleMode) modeCount++;
	
	if (modeCount == 0) {
		ROS_ERROR("%s << Either a device, file or topic should be specified for streaming.", __FUNCTION__);
		return false;
	} else if (modeCount > 1) {
		ROS_ERROR("Either a device, file or topic should be specified - not more than one.");
		return false;
	}
	
	if ((framerate < -1.0) || (framerate > MAX_READ_RATE)) framerate = DEFAULT_READ_RATE;
	

	//HGH
	nh.param<int>("republishSource", republishSource, NO_REPUBLISH_CODE);
	nh.param<std::string>("republishTopic", republishTopic, "specifyTopic/image_raw");
	//check for valid republishSource
	switch(republishSource){
		case REPUBLISH_CODE_8BIT_MONO:
			ROS_INFO("Republishing mono image as %s", republishTopic.c_str() );
			break;
		case REPUBLISH_CODE_8BIT_COL:
			ROS_INFO("Republishing color image as %s", republishTopic.c_str() );
			break;
		case REPUBLISH_CODE_16BIT:
			ROS_INFO("Republishing 16bit image as %s", republishTopic.c_str() );
			break;
		default:
			republishSource = NO_REPUBLISH_CODE;
			break;
	}
	if (republishSource != NO_REPUBLISH_CODE){
		ROS_INFO("Republish Code: %d", republishSource);
	}

	nh.param<bool>("republishNewTimeStamp",republishNewTimeStamp,false);

	nh.param<std::string>("frameID", frameID, ""); //specify the frameID

	nh.param<bool>("drawReticle",drawReticle, false);
	
	return true;
}
	
int getMapIndex(string mapping) {
	
	int map = 0;
	
	if (mapping == "GRAYSCALE") map = 0;
	if (mapping == "CIECOMP") map = 1;
	if (mapping == "BLACKBODY") map = 2;
	if (mapping == "RAINBOW") map = 3;
	if (mapping == "IRON") map = 4; 
	if (mapping == "BLUERED") map = 5;
	if (mapping == "JET") map = 6;
	if (mapping == "CIELUV") map = 7;
	if (mapping == "ICEIRON") map = 8;
	if (mapping == "ICEFIRE") map = 9;
	if (mapping == "REPEATED") map = 10;
	if (mapping == "HIGHLIGHTED") map = 11;
	
	return map;
}

double streamerNode::smoothThermistorReading() {
	
	int history = min((int)recordedThermistorReadings, MAX_THERMISTOR_READINGS_TO_STORE);
	
	double smoothedTemperature = 0.0;
	
	double x[MAX_THERMISTOR_READINGS_TO_STORE], y[MAX_THERMISTOR_READINGS_TO_STORE];
	
	unsigned int termsToConsider = 0;
	
	for (int iii = 0; iii < history; iii++) {
		
		if ( ( ros::Time::now().toSec() - thermistorBuffer[(recordedThermistorReadings-iii-1) % MAX_THERMISTOR_READINGS_TO_STORE][0] ) < configData.thermistorWindow) {
			x[termsToConsider] = thermistorBuffer[(recordedThermistorReadings-iii-1) % MAX_THERMISTOR_READINGS_TO_STORE][0];
			y[termsToConsider] = thermistorBuffer[(recordedThermistorReadings-iii-1) % MAX_THERMISTOR_READINGS_TO_STORE][1];
			termsToConsider++;
		}
	}
	
	// Simple average:
	if (0) {
		for (unsigned int iii = 0; iii < termsToConsider; iii++) {
			//ROS_INFO("Considering reading (%f, %f)...", x[iii], y[iii]);
			smoothedTemperature += y[iii];
		}
		
		smoothedTemperature /= double(termsToConsider);
	}
	
	//ROS_INFO("termsToConsider = (%d), (%f, %f)", termsToConsider, x[0], y[0]);
	
	// Linear Regression
	if (1) {
		double m, c;
		
		findLinearModel(x, y, termsToConsider, m, c);
		
		// Use the latest
		smoothedTemperature = m * thermistorBuffer[(recordedThermistorReadings-1) % MAX_THERMISTOR_READINGS_TO_STORE][0] + c;
		
	}
	
	//ROS_INFO("smoothedTemperature = (%f)", smoothedTemperature);
	
	return smoothedTemperature;
}

bool streamerNode::getNucSettingsReading(int& delay, double& diff) {
	
	bool writeSucceeded = false;
	
	char localCommand[256];
	sprintf(localCommand, "NUCSET");
	if (!sendSerialCommand(localCommand, configData.serialWriteAttempts)) ROS_WARN("Serial command (%s) failed after (%d) attempts", localCommand, configData.serialWriteAttempts);
	
	char buff[SERIAL_BUFF_SIZE];
	
	memset(&buff[0], 0, sizeof(buff));
	
	int n = read(mainfd, buff, SERIAL_BUFF_SIZE);
	
	
	bool readSucceeded = false;
	
	//ROS_INFO("read size = (%d)", n);
	
	if (n < 0) {
		ROS_WARN("NUC settings were unable to be read!");
	} else if (n == 0) {
		ROS_WARN("0 characters from NUC settings...");
	} else readSucceeded = true;
	
	ROS_INFO("Read: (%s)", buff);
	
	return (writeSucceeded && readSucceeded);
	
}

float streamerNode::getThermistorReading() {
	
	double retVal;
	
	bool writeSucceeded = false;
	
	char localCommand[256];
	sprintf(localCommand, "THERM");
	if (!sendSerialCommand(localCommand, configData.serialWriteAttempts)) {
		if (configData.verboseMode) { ROS_WARN("Serial command (%s) failed after (%d) attempts", localCommand, configData.serialWriteAttempts); }
		return -9e99;
	}
	
	char buff[SERIAL_BUFF_SIZE];
	memset(&buff[0], 0, sizeof(buff));
	
	int n = read(mainfd, buff, SERIAL_BUFF_SIZE);
	
	//n = fread(buff, 1, 256, mainfd);
	
	bool readSucceeded = false;
	
	//ROS_INFO("read size = (%d)", n);
	
	if (n < 0) {
		ROS_WARN("Thermistor value was unable to be read!");
	} else if (n == 0) {
		ROS_WARN("Zero characters returned from attempted thermistor reading.");
	} else readSucceeded = true;
	
	if (!readSucceeded) return -9e99;
	
	for (unsigned int iii = 0; iii < 512; iii++) {
		if (buff[iii] == '\r') buff[iii] = '-';
	}
	
	//ROS_ERROR("getThermistorReading(): Line = [%d] (%s)", n, buff);
	
	//istringstream ss_buff( &buff[101]); // used to be 103
	
	retVal = atof(&buff[103]);
	
	return retVal;
}

#endif
