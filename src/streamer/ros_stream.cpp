#include "streamer/ros_stream.hpp"

#ifdef _BUILD_FOR_ROS_

void mySigintHandler(int sig)
{
	// Do some custom action.
	// For example, publish a stop message to some other nodes.

	ROS_INFO("Requested shutdown... terminating feeds...");
	wantsToShutdown = true;
	
#ifdef _BUILD_FOR_ROS_
	(*globalNodePtr)->prepareForTermination();
#endif
	

	//shutdown();

	// All the default sigint handler does is call shutdown()
  
}

bool streamerNode::runBag() {
	
	if (configData.subscribeMode) {
		ROS_INFO("Subscription started...");
	} else if (configData.resampleMode) {
		ROS_INFO("Resampling subscription started...");
	}

	while (isVideoValid()) {
		#ifdef _BUILD_FOR_ROS_
		ros::spinOnce();
		#endif
	}
	
	if (configData.subscribeMode) {
		ROS_INFO("Subscription terminating...");
	} else if (configData.resampleMode) {
		ROS_INFO("Resampling subscription terminating...");
	}
	
	return true;
}

#ifdef _BUILD_FOR_ROS_
//HGH
void streamerNode::updateCameraInfoExtrinsics() {
    if (configData.verboseMode) { ROS_INFO("Updating camera info..."); }
    if (configData.camera_number == 0){
        for (unsigned int iii = 0; iii < 3; iii++) {
            for (unsigned int jjj = 0; jjj < 4; jjj++) {
                camera_info.P[iii*4 + jjj] = globalExtrinsicsData.P0.at<double>(iii,jjj);
            }
        }
    }else if (configData.camera_number == 1){
        for (unsigned int iii = 0; iii < 3; iii++) {
            for (unsigned int jjj = 0; jjj < 4; jjj++) {
                camera_info.P[iii*4 + jjj] = globalExtrinsicsData.P1.at<double>(iii,jjj);
            }
        }
    }
    if (configData.verboseMode) { ROS_INFO("Camera info updated."); }
}
#endif

#ifdef _BUILD_FOR_ROS_
bool streamerNode::setCameraInfo(sensor_msgs::SetCameraInfo::Request &req, sensor_msgs::SetCameraInfo::Response &rsp) {

	camera_info = req.camera_info;

	if (configData.verboseMode) { ROS_INFO("calling assignCameraInfo() from setCameraInfo()..."); }
	assignCameraInfo();

	if (camera_calibration_parsers::writeCalibrationIni("../camera_parameters.txt", "gscam", camera_info)) {
		ROS_INFO("Camera information written to camera_parameters.txt");
		return true;
	}
	else {
		ROS_ERROR("Could not write camera_parameters.txt");
		return false;
	}
}
#endif

void streamerNode::acceptImage(void *ptr) {
	#ifdef _BUILD_FOR_ROS_
	if (temperatureMat.rows == 0) {
		if (canRadiometricallyCorrect) {
			temperatureMat = cv::Mat::zeros(camera_info.height, camera_info.width, CV_32FC1);
		}
	}
	
	if (configData.inputDatatype == DATATYPE_RAW) {
		if ((frame.rows == 0) || (frame.type() != CV_16UC1)) {

			frame = cv::Mat::zeros(camera_info.height, camera_info.width, CV_16UC1);
		}
		

		if (configData.verboseMode){ ROS_INFO("Copying image data to internal matrix... (%d, %d)", camera_info.width, camera_info.height); }
		memcpy(&(frame.at<unsigned char>(0,0)), ptr, camera_info.width*camera_info.height*2);
		if (configData.verboseMode){ ROS_INFO("Data copied."); }

		
		
	} else if (configData.inputDatatype == DATATYPE_8BIT) {

		if ((frame.rows == 0) || (frame.type() != CV_8UC1)) {
			frame = cv::Mat::zeros(camera_info.height, camera_info.width, CV_8UC1);
		}
		
		if (configData.verboseMode){ ROS_INFO("Copying image data to internal matrix.... (%d, %d)", camera_info.width, camera_info.height); }
		memcpy(&(frame.at<unsigned char>(0,0)), ptr, camera_info.width*camera_info.height);
		if (configData.verboseMode){ ROS_INFO("Data copied."); }
		
	} else if (configData.inputDatatype == DATATYPE_MM){
		if ((frame.rows == 0) || (frame.type() != CV_8UC3)) {
			frame = cv::Mat::zeros(camera_info.height, camera_info.width, CV_8UC3);
		}
		
		if (configData.verboseMode){ ROS_INFO("Copying image data to internal matrix... (%d, %d)", camera_info.width, camera_info.height); }
		memcpy(&(frame.at<unsigned char>(0,0)), ptr, camera_info.width*camera_info.height*3);
		if (configData.verboseMode){ ROS_INFO("Data copied."); }
	}

	if (configData.fixDudPixels) {
		if (configData.verboseMode){ ROS_INFO("Fixing dud pixels..."); }
		fix_bottom_right(frame);
		if (configData.verboseMode){ ROS_INFO("Dud pixels fixed."); }
	}
	#endif
}

void streamerNode::initializeMessages() {
	
	
	if (configData.output16bitFlag) {
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
	
	if (configData.output8bitFlag) {
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
	
	if (configData.outputColorFlag) {
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

void streamerNode::serialCallback(const ros::TimerEvent&) {
	
	if (!configData.serialComms) {
		return;
	}
	
	if (configData.serialPollingRate == 0.0) {
		if (configData.verboseMode) { ROS_INFO("Skipping serial callback because desired frequency is zero (0)."); }
		return;
	} else {
		//ROS_WARN("Polling serial device...");
		
		//struct termios options;
		//int tcgetattr_ret = tcgetattr(mainfd, &options);
		//displayTermiosData(options);
		
		if (configData.readThermistor) {
			
			newThermistorReading = getThermistorReading();
			//ROS_INFO("Original reading (%f)...", newThermistorReading);
			
			if ((newThermistorReading < MAX_VALID_THERMISTOR) && (newThermistorReading > MIN_VALID_THERMISTOR)) {
				
				//ROS_INFO("DEBUG (%d) : (%f) (%f)", 0, lastThermistorReading, newThermistorReading);
				
				if ((lastThermistorReading == -99.0) || ( (lastThermistorReading != -99.0) && (abs(lastThermistorReading - newThermistorReading) < configData.maxThermistorDiff) ) ) {
				
				//if ((!isinf(-lastThermistorReading)) || ( (isinf(-lastThermistorReading)) && (abs(lastThermistorReading - newThermistorReading) < configData.maxThermistorDiff))) {
				
				//bool initTest = (lastThermistorReading < MAX_VALID_THERMISTOR) && (lastThermistorReading > MIN_VALID_THERMISTOR);
					
				//if (isinf(-lastThermistorReading) || (abs(lastThermistorReading - newThermistorReading) < configData.maxThermistorDiff)) {
					
					//ROS_INFO("DEBUG (%d)", 1);
					
					if (newThermistorReading == 0.0) {
						//ROS_ERROR("newThermistorReading = (%f), lastThermistorReading = (%f)", newThermistorReading, lastThermistorReading);
					}
					
					// Update thermistor reading buffer
					thermistorBuffer[recordedThermistorReadings % MAX_THERMISTOR_READINGS_TO_STORE][0] = ros::Time::now().toSec();
					thermistorBuffer[recordedThermistorReadings % MAX_THERMISTOR_READINGS_TO_STORE][1] = newThermistorReading;
					recordedThermistorReadings++;
					
					//ofs_thermistor_log << newThermistorReading;
					
					if (configData.smoothThermistor) {
						double smoothReading = smoothThermistorReading();
						
						if ((smoothReading < MAX_VALID_THERMISTOR) && (smoothReading > MIN_VALID_THERMISTOR)) {
							
							//ROS_INFO("DEBUG (%d)", 2);
							
							if ((abs(smoothReading - newThermistorReading) < configData.maxThermistorDiff)) {
								newThermistorReading = smoothReading;
								//ROS_INFO("Smoothed reading (%f)...", newThermistorReading);
								
								//ofs_thermistor_log << " " << newThermistorReading;
							} else {
								//ROS_ERROR("smoothReading vs newThermistorReading = (%f) vs (%f)", smoothReading, newThermistorReading);
							}
							
						} else {
							//ROS_ERROR("smoothReading = (%f)", smoothReading);
						}
						
						
					}
	

	
					//ofs_thermistor_log << endl;
					lastThermistorReading = newThermistorReading;
					
					
					
				}
				
				
				
			}
			
			//ROS_INFO("newVal = (%f) vs oldVal = (%f)", newThermistorReading, lastThermistorReading);
			
			// If new value is valid
			//if ((newThermistorReading > MIN_THERMISTOR_READNG) && (newThermistorReading < MAX_THERMISTOR_READING)) {
				
				// If the historical value is valid
				//if (lastThermistorReading != 0.0) {
					
					
				//	if (abs(newThermistorReading - lastThermistorReading) < configData.maxThermistorDiff) {
				//		lastThermistorReading = newThermistorReading;
				//	}
					
					
				//} else {
					//lastThermistorReading = newThermistorReading;
				//}
				
			//}
			
			//ROS_WARN("Thermistor value = (%f)", lastThermistorReading);
		}
		
		if (configData.calibrationMode > 0) {
			if (alternateCounter >= configData.alternatePeriod) {
				
				char localCommand[256];
	
				if (configData.calibrationMode == CALIBMODE_ALT_OUTPUT) {
					if (altStatus) {
						sprintf(localCommand, "vidout raw");
					} else {
						sprintf(localCommand, "vidout ins");
					}
					altStatus = !altStatus;
				} else if (configData.calibrationMode == CALIBMODE_ALT_SHUTTER) {
					if (performingNuc) {
						
						if (configData.verboseMode) { ROS_INFO("About to switch shutter"); }
						
						if (altStatus) {
							sprintf(localCommand, "close");
						} else {
							sprintf(localCommand, "open");
						}
						altStatus = !altStatus;
						
					} else {
						if (configData.verboseMode) { ROS_INFO("About to perform a NUC"); }
						sprintf(localCommand, "nuc");
					}
					
					performingNuc = !performingNuc;
				} else if (configData.calibrationMode == CALIBMODE_LONG_SHUTTER) {
					
					if (configData.verboseMode) { ROS_INFO("About to switch shutter"); }
					
					if (altStatus) {
						sprintf(localCommand, "close");
					} else {
						sprintf(localCommand, "open");
					}
					
					altStatus = !altStatus;
				}
				
				if (configData.verboseMode) { ROS_INFO("Sending SPECIAL command: (%s)", localCommand); }
				if (!sendSerialCommand(localCommand, configData.serialWriteAttempts)) ROS_WARN("Serial command (%s) failed after (%d) attempts", localCommand, configData.serialWriteAttempts);
											
				alternateCounter = 0;
			}
			alternateCounter++;
		}
		
	}
	
	bool wantsToPerformNuc = false;
	
	if (currentNucProtectionMode && currentDesiredNucProtectionMode && (abs(ros::Time::now().toSec() - lastFlagReceived.toSec()) > MAX_TIME_WITHOUT_FLAGS) ) {
		
		if (configData.verboseMode) { ROS_INFO("Timer exceeded for disabled NUC mode, preparing to restore NUC settings..."); }
		
		currentDesiredNucProtectionMode = false;
		updateNucInterval = true;
		wantsToPerformNuc = true;
		
	}
	
	if (currentNucProtectionMode && !currentDesiredNucProtectionMode) {
		
		if (abs(ros::Time::now().toSec() - lastNucPerformed_at_the_earliest.toSec()) > MAX_TIME_WITHOUT_NUC_conservative) {
			wantsToPerformNuc = true;
		}
		
	}
	
	if (wantsToPerformNuc) {
		lastNucPerformed_at_the_earliest = ros::Time::now();
		performNuc();
		
	}
	
	if (updateDetectorMode) {
		
		string detectorCode;
		
		if (configData.detectorMode == DETECTOR_MODE_RAW) {
			detectorCode = "raw";
		} else if (configData.detectorMode == DETECTOR_MODE_LUM) {
			detectorCode = "lum";
		} else if (configData.detectorMode == DETECTOR_MODE_INS) {
			detectorCode = "ins";
		} else if (configData.detectorMode == DETECTOR_MODE_RAD) {
			detectorCode = "rad";
		} else if (configData.detectorMode == DETECTOR_MODE_TMP) {
			detectorCode = "t";
		}
		
		if (configData.verboseMode) { ROS_INFO("Changing detector output..."); }
		char localCommand[256];
		sprintf(localCommand, "vidout %s", detectorCode.substr(0,3).c_str());
		if (!sendSerialCommand(localCommand, configData.serialWriteAttempts)) ROS_WARN("Serial command (%s) failed after (%d) attempts", localCommand, configData.serialWriteAttempts);

		char buff[SERIAL_BUFF_SIZE];
		int n = read(mainfd, buff, SERIAL_BUFF_SIZE);
		
		// ROS_INFO("Returned is <%s>", buff);
		
		updateDetectorMode = false;
	}
	
	if (updateUSBMode) {
		
		string usbCode;
		
		
		
		if (configData.usbMode == USB_MODE_16) {
			usbCode = "1";
		} else if (configData.usbMode == USB_MODE_8) {
			usbCode = "2";
		}
		
		
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
		
		// ROS_INFO("Returned is (%s)", buff);
		
		/*
		int testDelay;
		double testDiff;
		getNucSettingsReading(testDelay, testDiff);
		ROS_WARN("Current nuc settings = (%d, %f)", testDelay, testDiff);
		*/
		
		currentNucProtectionMode = currentDesiredNucProtectionMode;

		updateNucInterval = false;

	}
	
}

bool streamerNode::sendSerialCommand(char *command, int max_attempts) {
	
	int bytesToWrite = 0;
	
	// string(serialCommand).substr(0,bytesToWrite-1).c_str(), bytesToWrite

	while (command[bytesToWrite] != '\0') {
		
		bytesToWrite++;
	}
	bytesToWrite++;
	
	char *ext_command;
	ext_command = new char[bytesToWrite+1];
	
	for (int iii = 0; iii < bytesToWrite-1; iii++) {
		ext_command[iii] = command[iii];
	}
	
	ext_command[bytesToWrite-1] = '\r';
	ext_command[bytesToWrite] = '\0';
	
	//printf("%s << sending (%s) ...\n", __FUNCTION__, ext_command);
	
	int attempts = 0, n = 0;
	while ((max_attempts == 0) || (attempts < max_attempts)) {
		
		n = write(mainfd, ext_command, bytesToWrite);
		
		if (n == bytesToWrite) {
			if (configData.serialFeedback) { ROS_INFO("Serial write of (%s) was successful.", command); }
			return true;
		}
		attempts++;
	}
	
	if ((n < 0) && (errno == EINTR)) {
		ROS_WARN("write() failed 1");
	} else if ( n < 0 ) {
		ROS_WARN("write() failed 2");
	} else {
		ROS_WARN("write() failed 3");
	}
	
	return false;
}

void streamerNode::timerCallback(const ros::TimerEvent&) {
	
	if (configData.pauseMode) {
		return;
	}
	
	if (configData.captureMode || configData.subscribeMode) {
		return;
	}
	
	if (configData.pollMode || configData.resampleMode) {
		
		if (configData.pollMode) {
			
			//ROS_ERROR("About to call stream");
			
			streamCallback();
			
			//ROS_ERROR("Stream called");
		}
		
		if (processImage()) {
			
			//ROS_ERROR("Processed");
			
			if (readyToPublish) {
				if (configData.resampleMode) {
					//updateCameraInfo();
				}
				publishTopics();
				
				//ROS_ERROR("Published");
				
				writeData();
				
				//ROS_ERROR("Written");
				
			}
			
		}
		
		//ROS_ERROR("All done.");
		

		return;
	}
	
	if (configData.loadMode) {
		
		string filename;
		
		if (configData.folder.at(configData.folder.length()-1) == '/') {
			filename = configData.folder + inputList.at(frameCounter);
		} else {
			filename = configData.folder + "/" + inputList.at(frameCounter);
		}

		frame = cv::imread(filename, -1);
		
		//ROS_INFO("Read frame (%s)", filename.c_str());
		
		if (processImage()) {
			//ROS_INFO("Image process is true...");
			if (readyToPublish) {
				//ROS_INFO("Publishing...");
				updateCameraInfo();
				publishTopics();
				writeData();
			}
			
		}
		
		if (frameCounter >= fileCount) {
			if (configData.verboseMode) { ROS_INFO("setValidity(false) : (frameCounter >= fileCount)"); }
			setValidity(false);
		}
		
		return;
	}

	bool validFrameRead = true;
	
	// If in read mode...
	if (configData.inputDatatype == DATATYPE_RAW) {

		do { 
			//ROS_WARN("Attempting to read frame...");
			if (av_read_frame(mainVideoSource->pIFormatCtx, &(mainVideoSource->packet)) != 0) {
				ROS_WARN("Frame invalid...");
				validFrameRead = false;
				break;
			}
			//ROS_WARN("Frame valid!");
		} while (mainVideoSource->packet.stream_index != mainVideoSource->videoStream);

		
		if (validFrameRead) {

			
			avcodec_decode_video2(mainVideoSource->pCodecCtx, mainVideoSource->pFrame, &(mainVideoSource->frameFinished), &(mainVideoSource->packet));

			acceptImage((void*) *(mainVideoSource->pFrame->data));

			if (processImage()) {
				if (readyToPublish) {
					updateCameraInfo();
					publishTopics();
					writeData();
				}
			}

			

			av_free_packet(&(mainVideoSource->packet));
			
		} else {
			if (configData.verboseMode) { ROS_INFO("setValidity(false) : (validFrameRead)"); }
			setValidity(false);
		}
		
	} else if ((configData.inputDatatype == DATATYPE_8BIT) || (configData.inputDatatype == DATATYPE_MM)) {
		
		if (configData.verboseMode) { ROS_INFO("About to read in frame..."); }
		
		if(!cap.isOpened()) {
			ROS_ERROR("Actually, device isn't open...");
			return;
		} else {
			if (configData.verboseMode) { ROS_INFO("Device is open..."); }
		}
		
		if (configData.verboseMode) { ROS_INFO("framecount = (%f)", cap.get(CV_CAP_PROP_FRAME_COUNT)); }
		
		if (configData.verboseMode) { ROS_INFO("dim = (%f, %f)", cap.get(CV_CAP_PROP_FRAME_WIDTH), cap.get(CV_CAP_PROP_FRAME_HEIGHT)); }
		
		cap >> frame;
		//IplImage* tmp_im = cvQueryFrame(capture);
		//frame = tmp_im;
		
		if (configData.verboseMode) { ROS_INFO("Frame read"); }
		
		//imshow("test", frame);
		//waitKey();
		
		if (&frame == NULL) {
			if (configData.verboseMode) { ROS_INFO("setValidity(false) : (&frame == NULL)"); }
			setValidity(false);
			
		} else {
			if (processImage()) {
				if (readyToPublish) {
					updateCameraInfo();
					publishTopics();
					writeData();
				}
			}


		}
		
	}
}

void streamerNode::overwriteCameraDims() {
	
	camera_info.height = globalCameraInfo.imageSize.at<unsigned short>(0, 1);
	camera_info.width = globalCameraInfo.imageSize.at<unsigned short>(0, 0);
	
}

void streamerNode::handle_nuc_instruction(const std_msgs::Float32::ConstPtr& nuc_msg) {
//void streamerNode::handle_nuc_instruction(const std_msgs::Float32& nuc_msg) {
	
	if (configData.verboseMode) { ROS_INFO("Handling NUC instruction: (%f)", nuc_msg->data); }
	
	lastFlagReceived = ros::Time::now();
	
	currentDesiredNucProtectionMode = (nuc_msg->data < 0.5);
	
	if (currentDesiredNucProtectionMode != currentNucProtectionMode) { 
		updateNucInterval = true;
	}
	
	
}

void streamerNode::handle_info(const sensor_msgs::CameraInfoConstPtr& info_msg) {
	
	//ROS_ERROR("handle_info");
	
	if (configData.syncMode != SYNCMODE_SOFT) {
		return;
	}
	
	
	if ((!configData.subscribeMode) && (!configData.resampleMode)) {
		return;
	}
	
	if (configData.verboseMode) { ROS_INFO("Copying camera info over..."); }
	
	
	original_camera_info = *info_msg;
	
	
	
	info_time = ros::Time::now();
	original_time = info_msg->header.stamp;
	originalInternalTime = info_msg->R[0];
	
	original_bx = info_msg->binning_x;
	original_by = info_msg->binning_y;
	
	//cout << "Handling info..." << info_msg->header.stamp.toNSec() << ", " << info_time.toNSec() << endl;
	
	if (std::abs(image_time.toNSec() - info_time.toNSec()) < configData.soft_diff_limit) {
		//ROS_ERROR("handle_info : image_time == info_time");
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



        if (configData.output16bitFlag) {

            //HGH
            if (configData.republishSource==REPUBLISH_CODE_16BIT){
                pub_republish = it->advertiseCamera(_republishPubName, 1);
            }

                if (!cameraAdvertised) {
                        pub_16bit = it->advertiseCamera(_16bitPubName, 1);

                        cameraAdvertised = true;
                } else {
                        pub_16bit_im = it->advertise(_16bitPubName, 1);

                }

        }

        if (configData.output8bitFlag) {

            //HGH
            if (configData.republishSource==REPUBLISH_CODE_8BIT_MONO){
                pub_republish = it->advertiseCamera(_republishPubName, 1);
            }

                if (!cameraAdvertised) {
                        pub_8bit = it->advertiseCamera(_8bitPubName, 1);

                        cameraAdvertised = true;
                } else {
                        pub_8bit_im = it->advertise(_8bitPubName, 1);

                }

        }


        if (configData.outputColorFlag) {
            //HGH
            if (configData.republishSource==REPUBLISH_CODE_8BIT_COL){
                pub_republish = it->advertiseCamera(_republishPubName, 1);
            }
                if (!cameraAdvertised) {
                        pub_color = it->advertiseCamera(colorPubName, 1);

                        cameraAdvertised = true;
                } else {
                        pub_color_im = it->advertise(colorPubName, 1);

                }
        }

}



void streamerNode::handle_image(const sensor_msgs::ImageConstPtr& msg_ptr) {
	
	//ROS_ERROR("handle_image");
	
	if (configData.syncMode == SYNCMODE_HARD) {
		return;
	}
	
	if ((!configData.subscribeMode) && (!configData.resampleMode)) {
		return;
	}

	//cout << "Handling image..." << msg_ptr->header.stamp.toNSec() << ", " << image_time.toNSec() << endl;
	
	original_time = msg_ptr->header.stamp;
	
	//ROS_INFO("original_time = (%f)", original_time.toSec());
	
	image_time = ros::Time::now();
	cv_ptr = cv_bridge::toCvCopy(msg_ptr, enc::BGR8);					// For some reason it reads as BGR, not gray
	
	if (configData.syncMode == SYNCMODE_SOFT) {
		if (std::abs(image_time.toNSec() - info_time.toNSec()) < configData.soft_diff_limit) {
			// ROS_ERROR("handle_image : image_time == info_time");
			
			image_time = dodgeTime;
			act_on_image();
		}
	} else {
		act_on_image();
	}
	
	
	//ROS_ERROR("image header = (%d)", msg_ptr->header.stamp.toNSec());
		
	

	
}

streamerData::streamerData() {
	
	intrinsicsProvided = false;
        //wantsToRectify = false;
        //HGH
        wantsToAddExtrinsics = false;
	wantsToWrite = false;
	wantsToEncode = false;
	wantsToKeepNames = false;
	wantsToUndistort = false;
	wantsToResize = false;
	loopMode = false;
	imageDimensionsSpecified = false;
	
	
	
	mapCode = CIECOMP;
	mapParam = 0;
	
	framerate = -1.0;
	
	soft_diff_limit = 5000000;
	
}

bool streamerData::obtainStartingData(ros::NodeHandle& nh) {
	
	nh.param<std::string>("source", source, "dev");
	
	nh.param<std::string>("file", filename, "file");
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
	
	if (outputFormat == 0) {
		outputFormatString = "jpg";
	} else if (outputFormat == 1) {
		outputFormatString = "pgm";
	} else if (outputFormat == 2) {
		outputFormatString = "bmp";
	} else if (outputFormat == 3) {
		outputFormatString = "ppm";
	} else if (outputFormat == 4) {
		outputFormatString = "png";
	}
	
	nh.param<std::string>("outputType", outputType, "CV_8UC3");
	
	if (outputType == "CV_8UC3") {
		if (outputFormatString == "pgm") {
			ROS_WARN("PGM cannot write as CV_8UC3...");
		}
	} else if (outputType == "CV_8UC1") { 
		// ...
	} else if (outputType == "CV_16UC1") {
		if ((outputFormatString == "jpg") || (outputFormatString == "bmp")) {
			ROS_WARN("JPG/BMP cannot write as CV_16UC1...");
		}
	} else {
		ROS_WARN("Unrecognized output format (%s)", outputType.c_str());
	}

	nh.param<double>("maxIntensityChange", maxIntensityChange, 2.0);

	
	nh.param<bool>("drawReticle", drawReticle, false);
	nh.param<bool>("displayThermistor", displayThermistor, false);
	nh.param<bool>("dumpDuplicates", wantsToOutputDuplicates, false);
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
	
	nh.param<bool>("output16bit", output16bitFlag, true);
	nh.param<bool>("output8bit", output8bitFlag, false);
	nh.param<bool>("outputColor", outputColorFlag, true);
	
	nh.param<bool>("loopMode", loopMode, false);
	
	
	nh.param<bool>("disableSkimming", disableSkimming, true);
	
	nh.param<bool>("writeImages", wantsToWrite, false);
	nh.param<std::string>("outputFolder", outputFolder, "outputFolder");
	
	nh.param<bool>("serialComms", serialComms, false);
	nh.param<bool>("radiometricCorrection", radiometricCorrection, true);
	nh.param<bool>("alreadyCorrected", alreadyCorrected, false);
	nh.param<bool>("radiometricRaw", radiometricRaw, false);
	nh.param<bool>("radiometricInterpolation", radiometricInterpolation, true);
	
	nh.param<int>("radiometricBias", radiometricBias, 0);
	
	nh.param<int>("calibrationMode", calibrationMode, CALIBMODE_OFF);
	
	ROS_INFO("calibrationMode = (%d)", calibrationMode);
	
	if (outputFolder.size() > 0) {
		if (outputFolder.at(outputFolder.size()-1) == '/') {
			outputFolder = outputFolder.substr(0, outputFolder.size()-1);
		}
	} else {
		ROS_WARN("Specified output folder is 0 characters long - ignoring.");
		outputFolder = "outputFolder";
	}
	
	
	if ((wantsToWrite) && (outputFolder.size() > 0)) {
		// Create necessary folders
		char folderCommand[256];
		sprintf(folderCommand, "mkdir -p %s", outputFolder.c_str());
		
		int res = system(folderCommand);
		
		if (res == 0) {
			ROS_WARN("system() call returned 0...");
		}
	}
	
	
	if (verboseMode) { ROS_INFO("outputFolder = (%s)", outputFolder.c_str()); }
	
	
	
	nh.param<std::string>("outputType", outputType, "CV_16UC1");
	
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
		
	nh.param<bool>("undistortImages", wantsToUndistort, false);
        //HGH
        nh.param<bool>("rectifyImages", wantsToRectify, false);
	
	nh.param<bool>("removeDuplicates", wantsToRemoveDuplicates, false);
	
	if (outputFolder != "outputFolder") {
		nh.param<bool>("dumpTimestamps", wantsToDumpTimestamps, false);
	} else {
		wantsToDumpTimestamps = false;
	}
	
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
			
		} else {
			ROS_INFO("Resizing to (%d x %d)", desiredCols, desiredRows);
		}
	}
	
	if (wantsToUndistort) { ROS_INFO("Undistorting images..."); }
	
	nh.param<int>("normMode", normMode, 0);
	nh.param<std::string>("normalizationMode", normalizationMode, "normalizationMode");
	
	nh.param<double>("normFactor", normFactor, 0.0);
	
	if (normalizationMode == "standard") {
		normMode = 0;
	} else if (normalizationMode == "low_contrast") {
		normMode = 1;
	}
	
	/*
	if (wantsToWrite) {
		ROS_INFO("Has chosen to write.");
		
		if (outputFolder == "outputFolder") {
			ROS_ERROR("outputFolder incorrectly specified...");
			return false;
		} else {
			ROS_INFO("outputFolder = (%s)", outputFolder.c_str());
			
		}
		
		ROS_INFO("Image format = (%d); image type = (%s)", outputFormat, outputType.c_str());
		
		if (wantsToKeepNames) {
			ROS_INFO("(retaining original names)");
		} else {
			ROS_INFO("(not retaining original names)");
		}
		
	}
	*/
	
	if (wantsToEncode) {
		if (verboseMode) { ROS_INFO("Has chosen to encode."); }
		
		if (outputVideo == "outputVideo") {
			ROS_ERROR("outputVideo incorrectly specified...");
			return false;
		} else {
			ROS_INFO("outputVideo = (%s)", outputVideo.c_str());
			
		}
		
		if (verboseMode) { ROS_INFO("Image format = (%s); image type = (%s)", "avi", videoType.c_str()); }
		
	}
	
	if (inputDatatype == DATATYPE_8BIT) {
		if (verboseMode) { ROS_INFO("Streaming mode: 8-bit"); }
	} else if (inputDatatype == DATATYPE_RAW) {
		if (verboseMode) { ROS_INFO("Streaming mode: 16-bit"); }
	} else if (inputDatatype == DATATYPE_MM) {
		if (verboseMode) { ROS_INFO("Streaming mode: multimodal"); }
	}

	// ROS_INFO("Source = %s", source.c_str());
	
	readMode = false;
	loadMode = false;
	captureMode = false;
	pollMode = false;
	subscribeMode = false;
	resampleMode = false;
	
	
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
		} else {
			ROS_INFO("Requested framerate = %f", framerate);
		}
		
		ROS_INFO("Reading from a file (%s)", filename.c_str());
	} else if (source == "folder") {
		
		loadMode = true;
		
		if ((framerate < 0.0) || (framerate > MAX_READ_RATE)) {
			ROS_WARN("Invalid framerate (%f) so defaulting to (%f).", framerate, DEFAULT_READ_RATE);
			framerate = DEFAULT_READ_RATE;
		} else {
			ROS_INFO("Requested framerate = %f", framerate);
		}
		
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
	
	if (loopMode == true) {
		ROS_INFO("Option to loop has been selected.");
	}
	
	if (intrinsics != "intrinsics") {
		intrinsicsProvided = true;
		if (verboseMode) { ROS_INFO("Intrinsics at (%s) selected.", intrinsics.c_str()); }
		
		if ((inputWidth != 0) && (inputHeight != 0)) {
			ROS_WARN("Provided image dimensions will be ignored because of provided intrinsics file.");
		}
	} else {
		
		if ((inputWidth != 0) && (inputHeight != 0)) {
			ROS_INFO("Provided image dimensions (%d, %d) will be used.", inputWidth, inputHeight);
			imageDimensionsSpecified = true;
		} else {
			ROS_WARN("No intrinsics or image dimensions provided. Will attempt to estimate camera size...");	
		}
		
			
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
			
		} else {
			ROS_INFO("Camera number (%d).", camera_number);
		}
		
	}

	getMapping(map, extremes, mapCode, mapParam);
		
	int modeCount = 0;
	
	if (captureMode) modeCount++;
	if (pollMode) modeCount++;
	if (readMode) modeCount++;
	if (loadMode) modeCount++;
	if (subscribeMode) modeCount++;
	if (resampleMode) modeCount++;
	
	if (modeCount == 0) {
		ROS_ERROR("Either a device, file or topic should be specified for streaming.");
		return false;
	} else if (modeCount > 1) {
		ROS_ERROR("Either a device, file or topic should be specified - not more than one.");
		return false;
	}
	
	if ((framerate < -1.0) || (framerate > MAX_READ_RATE)) {
		framerate = DEFAULT_READ_RATE;
	}
	
	 //ROS_INFO("normalization mode = (%d)", normMode);

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

bool streamerNode::isVideoValid() {
	
	if (::wantsToShutdown) {
		if (configData.verboseMode){ ROS_INFO("Wants to shut down.."); }
		setValidity(false);
	}	
		
	return videoValid;
	
}
	
int getMapIndex(string mapping) {
	
	int map;
	
	if (mapping == "GRAYSCALE") {
		map = 0;
	} else if (mapping == "CIECOMP") {
		map = 1;
	} else if (mapping == "BLACKBODY") {
		map = 2;
	} else if (mapping == "RAINBOW") {
		map = 3;
	} else if (mapping == "IRON") {
		map = 4;
	} else if (mapping == "BLUERED") {
		map = 5;
	} else if (mapping == "JET") {
		map = 6;
	} else if (mapping == "CIELUV") {
		map = 7;
	} else if (mapping == "ICEIRON") {
		map = 8;
	} else if (mapping == "ICEFIRE") {
		map = 9;
	} else if (mapping == "REPEATED") {
		map = 10;
	} else if (mapping == "HIGHLIGHTED") {
		map = 11;
	} else {
		map = 0;
	}
	
	return map;
}

/*
int streamerNode::mygetch() {
  struct termios oldt,newt;
  int ch;
  tcgetattr( STDIN_FILENO, &oldt );
  newt = oldt;
  newt.c_lflag &= ~( ICANON | ECHO );
  tcsetattr( STDIN_FILENO, TCSANOW, &newt );
  ch = getchar();
  tcsetattr( STDIN_FILENO, TCSANOW, &oldt );
  return ch;
}
*/

void set_blocking (int fd, int should_block)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
			ROS_ERROR("error %d getting term settings set_blocking", errno);
			//return;
        }

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = should_block ? 5 : 0; // 0.5 seconds read timeout

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
                ROS_ERROR("error setting term %sblocking", should_block ? "" : "no");
}

int streamerNode::open_port() {
   int fd;                                   /* File descriptor for the port */


   //fd = open(configData.portAddress.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
   //fd = open(configData.portAddress.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK); 
   //fd = open(configData.portAddress.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
   fd = open(configData.portAddress.c_str(), O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK | O_SYNC);
   
   set_blocking (fd, 0);   // disable reads blocked when no input ready

	char buf [10000];
	int n;
	do {
			n = read (fd, buf, sizeof buf);
	} while (n > 0);

	//set_blocking (fd, 1);  // enable read blocking (if desired)
   
   usleep(200000);
   tcflush(fd, TCIOFLUSH);

   if (fd == -1) {                                              /* Could not open the port */
     ROS_ERROR("open_port(): Unable to open (%s) (%s)", configData.portAddress.c_str(), strerror(errno));
   } else {
	   //ROS_ERROR("port (%s) opened", configData.portAddress.c_str());
	   //fcntl(fd, F_SETFL, 0);
	   
   }
   return (fd);
}
 
void displayTermiosData(termios options) {
	
	ROS_INFO("   Termios Summary:");
	
	ROS_INFO("c_iflag = (%d)", options.c_iflag);
	ROS_INFO("c_oflag = (%d)", options.c_oflag);
	ROS_INFO("c_cflag = (%d)", options.c_cflag);
	ROS_INFO("c_lflag = (%d)", options.c_lflag);
	ROS_INFO("c_ispeed = (%d)", options.c_ispeed);
	ROS_INFO("c_ospeed = (%d)", options.c_ospeed);
	
	for (unsigned int iii = 0; iii < NCCS; iii++) {
		ROS_INFO("c_cc[%d] = (%d)", iii, options.c_cc[iii]);
	}
	
	ROS_WARN("Termios Summary Complete.");
	
	/*
	tcflag_t c_iflag;
	tcflag_t c_oflag;
	tcflag_t c_cflag;
	tcflag_t c_lflag;
	cc_t c_cc[NCCS];
	speed_t c_ispeed;
	speed_t c_ospeed
	*/
	
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
	} else {
		readSucceeded = true;
		
	}
	
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
	} else {
		readSucceeded = true;
	}
	
	if (!readSucceeded) {
		return -9e99;
	}
	
	for (unsigned int iii = 0; iii < 512; iii++) {
		if (buff[iii] == '\r') {
			buff[iii] = '-';
		}
	}
	
	//ROS_ERROR("getThermistorReading(): Line = [%d] (%s)", n, buff);
	
	//istringstream ss_buff( &buff[101]); // used to be 103
	
	retVal = atof(&buff[103]);
	
	/*
	char arrayChar[11];
	
	for (unsigned int iii = 0; iii < 10; iii++) {
		arrayChar[iii] = buff[100+iii];
	}
	
	arrayChar[10] = '\0';
	*/
	
	// if (1) /*(configData.verboseMode)*/ { ROS_INFO("Temperature = (%f), (%s)", retVal, arrayChar); }
	
	return retVal;
	
}

#endif

