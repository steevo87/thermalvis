#include "ros_stream.hpp"

#ifdef _BUILD_FOR_ROS_

void mySigintHandler(int sig) {
	wantsToShutdown = true;
	displayMessage("Requested shutdown... terminating feeds...", MESSAGE_WARNING);
	
	(*globalNodePtr)->prepareForTermination();
}

void streamerNode::prepareForTermination() {
	if (ofs.is_open()) {
		ofs.close();
	}
	
	if (ofs_call_log.is_open()) {
		ofs_call_log.close();
	}
	
	if (ofs_retrieve_log.is_open()) {
		ofs_retrieve_log.close();
	}
	
	if (ofs_internal_log.is_open()) {
		ofs_internal_log.close();
	}
	
	if (ofs_write_log.is_open()) {
		ofs_write_log.close();
	}
	
	if (ofs_duplicates_log.is_open()) ofs_duplicates_log.close();
	
	if (ofs_thermistor_log.is_open()) ofs_thermistor_log.close();
	
	if (configData.serialComms) {
		#ifndef _WIN32
		close(mainfd);
		#endif
	}
	
	releaseDevice();
	
}

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
		
		int deviceWidth, deviceHeight;

		#ifdef _BUILD_FOR_ROS_
		getMainVideoSource()->setup_video_capture(configData.capture_device.c_str(), deviceWidth, deviceHeight, configData.verboseMode);
		#endif

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
		
	}
	
	if (configData.verboseMode) { ROS_INFO("Device set up!"); }
	
	setValidity(true);
	deviceCreated = true;
	return true;
}

void streamerNode::releaseDevice() {
	
	if (!deviceCreated) { return; }
	
	if ((isVideoValid()) && (configData.verboseMode)) { ROS_INFO("setValidity(false) : releaseDevice()"); }
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


bool streamerNode::runDevice() {
	
	if (configData.captureMode) {
		ROS_INFO("Video stream (device: %d) started...", configData.device_num);
	} else if (configData.pollMode) {
		ROS_INFO("Video polling (device: %d) started...", configData.device_num);
	}
	
	setupDevice();
	
	while (isVideoValid()) {
		
		

		if (configData.verboseMode){ ROS_INFO("Starting loop.."); }
		if (configData.captureMode) {
			
			#ifdef _BUILD_FOR_ROS_
			if (configData.verboseMode){ ROS_INFO("About to spin"); }
			ros::spinOnce();
			if (configData.verboseMode){ ROS_INFO("Spun"); }
			#endif
			
			if (streamCallback()) {
				
				
				
				if (configData.verboseMode){ ROS_INFO("About to process"); }
				
				
				if (processImage()) {
				
					if (configData.verboseMode){ ROS_INFO("Processed image."); }
					
					if (readyToPublish && (!configData.serialComms || !updateUSBMode)) {
						if (configData.verboseMode){ ROS_INFO("About to publish topics..."); }
						publishTopics();		
						if (configData.verboseMode){ ROS_INFO("Topics published."); }
						writeData();
						if (configData.verboseMode){ ROS_INFO("Data written."); }
					}
				}
			}
			
			
			
		} else if (configData.pollMode) {
			
			// Want to keep on calling streamCallback until the time is right to capture the frame
			streamCallback(false);
			#ifdef _BUILD_FOR_ROS_
			ros::spinOnce();
			#endif
		}
		
		if (configData.verboseMode){ ROS_INFO("Ending loop."); }
	}
	
	ROS_INFO("About to release device here...");
	releaseDevice();
	
	if (configData.captureMode) {
		ROS_INFO("Video capture terminating...");
	} else if (configData.pollMode) {
		ROS_INFO("Video polling terminating...");
	}
	
	return true;
}

bool streamerNode::runLoad() {
	ROS_INFO("Image load started...");
	
	if (!processFolder()) {
		ROS_ERROR("Processing of folder failed...");
		return false;
	}
	
	do {
		
		frameCounter = 0;
		setValidity(true);
		
		while (isVideoValid()) {
			#ifdef _BUILD_FOR_ROS_
			ros::spinOnce();		
			#endif
		}
		
	} while (configData.loopMode && !wantsToShutdown);
	
	return true;
	
}

bool streamerNode::runRead() {
	
	ROS_INFO("Video reading started...");
		
	do {
		
		setValidity(true);
		
		if (configData.verboseMode) { ROS_INFO("Opening file (%s)...", configData.filename.c_str()); }
		if (configData.inputDatatype == DATATYPE_RAW) {
			#ifdef _BUILD_FOR_ROS_
			if (getMainVideoSource()->setup_video_file(configData.filename) < 0) {
				ROS_ERROR("Source configuration failed.");
				return false;
			}
			#endif
		} else if ((configData.inputDatatype == DATATYPE_8BIT) || (configData.inputDatatype == DATATYPE_MM)) {
			getVideoCapture()->open(configData.filename.c_str());
			
			if(!cap.isOpened()) { // check if we succeeded
				ROS_ERROR("File open failed (using OpenCV).");
				return false;
			}
			//capture = cvCaptureFromAVI(configData.filename.c_str());
		}	
		
		if (configData.verboseMode) { ROS_INFO("Source configured."); }
		
		while (isVideoValid()) {
			#ifdef _BUILD_FOR_ROS_
			ros::spinOnce();
			#endif
		}
		
		if (configData.verboseMode) { ROS_INFO("Video complete."); }
		
		if (configData.inputDatatype == DATATYPE_RAW) {
			#ifdef _BUILD_FOR_ROS_
			getMainVideoSource()->close_video_file(configData.filename);
			#endif
		} else if ((configData.inputDatatype == DATATYPE_8BIT) || (configData.inputDatatype == DATATYPE_MM)) {
			getVideoCapture()->release();
		}

	} while (configData.loopMode && !wantsToShutdown);
	
	if (configData.verboseMode) { ROS_INFO("Video reading terminating..."); }
	
	return true;
	
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


bool streamerNode::processFolder() {
	
#ifndef _WIN32
	DIR * dirp;
	struct dirent * entry;
	
	dirp = opendir(configData.folder.c_str());
	
	if (dirp == NULL) {
		ROS_ERROR("Opening of directory (%s) failed.", configData.folder.c_str());
		return false;
	}

	while ((entry = readdir(dirp)) != NULL) {
		if (entry->d_type == DT_REG) { // If the entry is a regular file
			inputList.push_back(string(entry->d_name));
			fileCount++;
		}
	}
	closedir(dirp);
#else
	printf("%s << ERROR! Not implemented yet!\n", __FUNCTION__);
	return false;
#endif
	

	sort(inputList.begin(), inputList.end());

	if(fileCount == -1)	{
		ROS_ERROR("File counting error.\n");
		return false;
	}

	ROS_INFO("No. of images in folder = %d\n", fileCount);

	if (fileCount == 0) {
		ROS_ERROR("Returning, because no images are in folder.\n");
		return false;
	}
	
	return true;
}

void streamerNode::markCurrentFrameAsDuplicate() {
	
#ifdef _BUILD_FOR_ROS
	//camera_info.binning_x = 1; // this one is used for thermistor
	camera_info.binning_y = 1; // this one is to be used for duplicates
#endif
}

#ifdef _BUILD_FOR_ROS_
void streamerNode::updateCameraInfo() {
	
	//if (configData.verboseMode) { ROS_INFO("Updating camera info..."); }
	
	
	if ((configData.subscribeMode) || (configData.resampleMode)) {
		
		
		
		if (configData.syncMode == SYNCMODE_IMAGEONLY) {
			
			
			
			
			if (configData.useCurrentRosTime) {

				ros::Time currTime = ros::Time::now();
				camera_info.header.stamp = currTime;
			} else {
				camera_info.header.stamp = original_time;
			}
			
			
			
			//camera_info.header.frame_id = "thermalvis_streamer_optical_frame";
			//camera_info.height = cv_ptr->image.rows;
			//camera_info.width = cv_ptr->image.cols;
			
			
		} else {
			camera_info = original_camera_info;
		}
		
		//ROS_WARN("(%f) -> (%f)", original_camera_info.binning_x, camera_info.binning_x);
		
		memcpy(&newThermistorReading, &camera_info.binning_x, sizeof(float)); // monkey, lastThermistorReading
		

		//ROS_WARN("2: (%f) -> (%f)", original_camera_info.binning_x, camera_info.binning_x);
		
		//if (configData.verboseMode) { ROS_INFO("seq = (%d, %d)", original_camera_info.header.seq, camera_info.header.seq); }
		
	} else {

		
		ros::Time currTime = ros::Time::now();
		
		camera_info.header.stamp = currTime;
		
		if ((configData.captureMode || configData.pollMode) && configData.readThermistor) {
			memcpy(&camera_info.binning_x, &lastThermistorReading, sizeof(float));
			//ROS_WARN("Thermistor value = (%f, %d)", lastThermistorReading, camera_info.binning_x);
		}
		
		
		
		// Internal time..
		memcpy(&camera_info.R[0], &firmwareTime, sizeof(float));
		
	}
	
	if (configData.wantsToMarkDuplicates && lastIsDuplicate) {
		camera_info.binning_y = 1;
	} else if (configData.wantsToMarkDuplicates) {
		camera_info.binning_y = 0;
	}
	
	if ((globalCameraInfo.cameraMatrix.rows == 3) && (configData.wantsToUndistort)) {
		for (unsigned int iii = 0; iii < 3; iii++) {
			for (unsigned int jjj = 0; jjj < 3; jjj++) {
				camera_info.K[iii*3 + jjj] = globalCameraInfo.newCamMat.at<double>(iii,jjj);
			}
		}	
		
		for (unsigned int iii = 0; iii < camera_info.D.size(); iii++) {
			camera_info.D.at(iii) = 0.0;
		}	
	}
	
	msg_color.header = camera_info.header;
	msg_16bit.header = camera_info.header;
	msg_8bit.header = camera_info.header;
	

}
#endif

#ifdef _BUILD_FOR_ROS_
void streamerNode::assignCameraInfo() {
	
	if (configData.verboseMode) { ROS_INFO("Entered assignCameraInfo()..."); }
	
	char frame_id[256];
	sprintf(frame_id, "%s_%s_%s", "thermalvis", nodeName, "optical_frame");
	camera_info.header.frame_id = string(frame_id);
	msg_color.header.frame_id = string(frame_id);
	msg_16bit.header.frame_id = string(frame_id);
	msg_8bit.header.frame_id = string(frame_id);
	
	camera_info.height = globalCameraInfo.imageSize.at<unsigned short>(0, 1); // DEFAULT_IMAGE_HEIGHT
	camera_info.width = globalCameraInfo.imageSize.at<unsigned short>(0, 0); // DEFAULT_IMAGE_WIDTH


	/* use data from extrinsics file
	if (globalExtrinsicsData.distCoeffs0.cols != 5) { //*/

	// /* use data from intrinsics file
	if (globalCameraInfo.distCoeffs.cols != 5) { //*/

		camera_info.distortion_model = "rational_polynomial";
		if (configData.verboseMode) { ROS_INFO("Camera model : RATIONAL POLYNOMIAL - (%d) coeffs", globalCameraInfo.distCoeffs.cols); }
	} else {
		camera_info.distortion_model = "plumb_bob";
		if (configData.verboseMode) { ROS_INFO("Camera model : PLUMB BOB - (%d) coeffs", globalCameraInfo.distCoeffs.cols); }
	}

	camera_info.D.clear();


        // /* use data from intrinsics file
	for (int iii = 0; iii < globalCameraInfo.distCoeffs.cols; iii++) { //orig
			camera_info.D.push_back(globalCameraInfo.distCoeffs.at<double>(0, iii)); //orig
	}

        /* use data from extrinsics file
        for (int iii = 0; iii < globalExtrinsicsData.distCoeffs0.cols; iii++) {
            if (configData.camera_number==0){
                camera_info.D.push_back(globalExtrinsicsData.distCoeffs0.at<double>(0, iii));
            }else if(configData.camera_number==1){
                camera_info.D.push_back(globalExtrinsicsData.distCoeffs1.at<double>(0, iii));
            }
        }
        //*/


	// Assign camera info from intrinsics file
	if (globalCameraInfo.cameraMatrix.rows == 3) {
		for (unsigned int iii = 0; iii < 3; iii++) {
			for (unsigned int jjj = 0; jjj < 3; jjj++) {
				if (configData.wantsToUndistort) {
					camera_info.K[iii*3 + jjj] = globalCameraInfo.newCamMat.at<double>(iii,jjj); //orig
				} else {
					camera_info.K[iii*3 + jjj] = globalCameraInfo.cameraMatrix.at<double>(iii,jjj); //orig
				}
			}
		}		
	}
	
	 // */

        /* use data from extrinsics file
        if (globalCameraInfo.cameraMatrix.rows == 3) {
                for (unsigned int iii = 0; iii < 3; iii++) {
                        for (unsigned int jjj = 0; jjj < 3; jjj++) {
                            if (configData.camera_number==0){
                                camera_info.K[iii*3 + jjj] = globalExtrinsicsData.cameraMatrix0.at<double>(iii,jjj);
                            }else if(configData.camera_number==1){
                                camera_info.K[iii*3 + jjj] = globalExtrinsicsData.cameraMatrix1.at<double>(iii,jjj);
                            }
                        }
                }
        } // */



        //HGH
        if (configData.wantsToAddExtrinsics){
            if (configData.camera_number == 0){
                for (unsigned int iii = 0; iii < 3; iii++) {
                    for (unsigned int jjj = 0; jjj < 3; jjj++) {
                        camera_info.R[iii*3 + jjj] = globalExtrinsicsData.R0.at<double>(iii,jjj);
                    }
                }
                for (unsigned int iii = 0; iii < 3; iii++) {
                    for (unsigned int jjj = 0; jjj < 4; jjj++) {
                        camera_info.P[iii*4 + jjj] = globalExtrinsicsData.P0.at<double>(iii,jjj);
                    }
                }
            }else if (configData.camera_number == 1){
                for (unsigned int iii = 0; iii < 3; iii++) {
                    for (unsigned int jjj = 0; jjj < 3; jjj++) {
                        camera_info.R[iii*3 + jjj] = globalExtrinsicsData.R1.at<double>(iii,jjj);
                    }
                }
                for (unsigned int iii = 0; iii < 3; iii++) {
                    for (unsigned int jjj = 0; jjj < 4; jjj++) {
                        camera_info.P[iii*4 + jjj] = globalExtrinsicsData.P1.at<double>(iii,jjj);
                    }
                }
            }

        } else {
                if (globalCameraInfo.R.rows == 3) {
                        for (unsigned int iii = 0; iii < 3; iii++) {
                                for (unsigned int jjj = 0; jjj < 3; jjj++) {
                                        camera_info.R[iii*3 + jjj] = globalCameraInfo.R.at<double>(iii,jjj);
                                }
                        }
                }

                if (globalCameraInfo.newCamMat.rows == 3) {
                        for (unsigned int iii = 0; iii < 3; iii++) {
                                for (unsigned int jjj = 0; jjj < 3; jjj++) {
                                            camera_info.P[iii*4 + jjj] = globalCameraInfo.newCamMat.at<double>(iii,jjj);
                                }
                        }
                }
        }


        if (configData.wantsToUndistort) {
                if ((globalCameraInfo.cameraMatrix.rows == 3) && (globalCameraInfo.distCoeffs.cols > 0) && (globalCameraInfo.newCamMat.rows == 3)) {
                    //HGH initUndistortRectifyMap(globalCameraInfo.cameraMatrix, globalCameraInfo.distCoeffs, globalCameraInfo.R, globalCameraInfo.newCamMat, globalCameraInfo.cameraSize, CV_32FC1, map1, map2);
                    //HGH
                    if (configData.wantsToAddExtrinsics){
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
                        cv::initUndistortRectifyMap(globalCameraInfo.cameraMatrix, globalCameraInfo.distCoeffs,  cv::Mat(), globalCameraInfo.newCamMat, globalCameraInfo.cameraSize, CV_32FC1, map1, map2);
                    }

                }
        }
	
	if (configData.verboseMode) { ROS_INFO("Camera info assigned."); }
	
}
#endif

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

bool streamerNode::streamCallback(bool capture) {
	
	int currAttempts = 0;
	
	#ifdef _BUILD_FOR_ROS_
	if (configData.verboseMode) { 
		ros::Time callbackTime = ros::Time::now();
		ROS_INFO("Entered <streamCallback> at (%f)", callbackTime.toSec());
	}
	#endif

	if ((configData.inputDatatype == DATATYPE_8BIT) || (configData.inputDatatype == DATATYPE_MM)) {
		
		#ifdef _BUILD_FOR_ROS_
		if (configData.verboseMode){ ROS_INFO("Updating camera info..."); }
		updateCameraInfo();
		if (configData.verboseMode){ ROS_INFO("Camera info updated."); }
		ofs_call_log << ros::Time::now().toNSec() << endl;
		#endif

		if (configData.verboseMode){ ROS_INFO("Capturing frame (8-bit/MM)..."); }
		cap >> frame;
		
		if (configData.fixDudPixels) {
			if (configData.verboseMode){ ROS_INFO("Fixing dud pixels..."); }
			fix_bottom_right(frame);
			if (configData.verboseMode){ ROS_INFO("Dud pixels fixed."); }
		}
		
		if (configData.verboseMode){ ROS_INFO("Frame captured."); }
		#ifdef _BUILD_FOR_ROS_
		ofs_retrieve_log << ros::Time::now().toNSec() << endl;
		#endif
	} else if (configData.inputDatatype == DATATYPE_RAW) {
		
		#ifdef _BUILD_FOR_ROS_
		ros::Time callTime, retrieveTime;
		
		callTime = ros::Time::now();
		if (configData.verboseMode) { ROS_INFO("Capturing frame (16-bit)... (%f)", callTime.toSec()); }
		#endif

		//bool frameRead = false;
		while ((configData.maxReadAttempts == 0) || (currAttempts < configData.maxReadAttempts)) {
			// Keep on looping, but if max defined, only until curr attempts is high enough
			
			if (configData.verboseMode){ ROS_INFO("Attempting to capture frame..."); }
			
			#ifndef _WIN32
			if (av_read_frame(mainVideoSource->pIFormatCtx, &(mainVideoSource->oPacket)) != 0) {
				if (configData.verboseMode){ ROS_WARN("av_read_frame() failed."); }
				currAttempts++;
				continue;
			}
			#endif
			
			if (configData.verboseMode){ ROS_INFO("Frame captured successfully."); }
			
			#ifdef _BUILD_FOR_ROS_
			retrieveTime = ros::Time::now();
			firmwareTime = mainVideoSource->oPacket.pts;
			

			if (configData.verboseMode){ ROS_INFO("Updating camera info..."); }
			updateCameraInfo();
			if (configData.verboseMode){ ROS_INFO("Camera info updated."); }
			
			if (mainVideoSource->bRet < 0) {
				if (configData.verboseMode) { ROS_WARN("(mainVideoSource->bRet < 0) failed."); }
				currAttempts++;
				continue;
			}
			
			if (mainVideoSource->oPacket.stream_index != mainVideoSource->ixInputStream) {
				if (configData.verboseMode) { ROS_WARN("(mainVideoSource->oPacket.stream_index != mainVideoSource->ixInputStream) failed."); }
				currAttempts++;
				continue;
			}
			if (configData.verboseMode){ ROS_INFO("Decoding frame..."); }
			avcodec_decode_video2(mainVideoSource->pICodecCtx, mainVideoSource->pFrame, &(mainVideoSource->fFrame),&(mainVideoSource->oPacket));
			if (configData.verboseMode){ ROS_INFO("Frame decoded."); }
			
			av_free_packet(&(mainVideoSource->oPacket));
			if (configData.verboseMode){ ROS_INFO("Frame freed."); }
			
			
			if (!(mainVideoSource->fFrame)) {
				if (configData.verboseMode) { ROS_WARN("(!(mainVideoSource->fFrame)) failed."); }
				currAttempts++;
				continue;
			}
			
			if (capture) {
				if (configData.verboseMode){ ROS_INFO("Accepting image..."); }
				acceptImage((void*) *(mainVideoSource->pFrame->data));
				if (configData.verboseMode){ ROS_INFO("Image accepted."); }
				
				char outbuff[256];
				
				uint32_t internal_sec, internal_nsec;
		
				internal_sec = firmwareTime / 1000000;
				internal_nsec = (firmwareTime % 1000000) * 1000;
				
				//sprintf(outbuff, "%016d000", firmwareTime);
				sprintf(outbuff, "%010d.%09d", internal_sec, internal_nsec);
				
				if (configData.wantsToDumpTimestamps) {
					ofs_internal_log << outbuff << endl;
					//ofs_retrieve_log << retrieveTime.toNSec() << endl;
					//ofs_retrieve_log << retrieveTime.sec << "." << retrieveTime.nsec << endl;
					char output_time[256];
					sprintf(output_time,"%010d.%09d", retrieveTime.sec, retrieveTime.nsec);
					ofs_retrieve_log << output_time << endl;
					ofs_call_log << callTime.toNSec() << endl;
				}
				
			}
			#endif

			// Want to exit loop if it actually worked
			if (configData.verboseMode){ ROS_INFO("Frame read."); }
			break;
			
		}

		if ((currAttempts >= configData.maxReadAttempts) && (configData.maxReadAttempts != 0))  {
			ROS_ERROR("Frame reading failed after (%d) attempts.", currAttempts);
			setValidity(false);
			return false;
		}
		
		// If it got here, it means it succeeded so one attempt was not recorded...
		if (currAttempts >= 1) {
			ROS_WARN("Frame reading required (%d) attempts.", currAttempts+1);
		}
	}
	
	// processImage();
	
	//ROS_INFO("Exiting callback.");
	if (configData.verboseMode){ ROS_INFO("Exiting callback..."); }
	
	return true;
}

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

bool streamerNode::processImage() {
	
	if (frame.rows == 0) {
		return false;
	}
	
	//ros::Time preTime, postTime;
	//preTime = ros::Time::now();
	
	if (configData.wantsToResize) {
		resize(frame, rzMat, cv::Size(configData.desiredCols, configData.desiredRows));
		frame = cv::Mat(rzMat);
	}
	
	
	if (configData.wantsToRemoveDuplicates || configData.wantsToMarkDuplicates || configData.wantsToOutputDuplicates) {
		
		if (matricesAreEqual(frame, lastFrame)) {
			
			#ifdef _BUILD_FOR_ROS_
			lastNucPerformed_at_the_earliest = ros::Time::now();		
			#endif

			if (configData.wantsToMarkDuplicates) {
				// markCurrentFrameAsDuplicate();
				// ROS_ERROR("DUPLICATED!");
				lastIsDuplicate = true;
			}
			
			//ROS_INFO("SHOULD GET HERE A.");
			if (configData.wantsToOutputDuplicates) {
				//ROS_INFO("IS IT WRITING HERE? A");
				ofs_duplicates_log << "1" << endl;
			} else {
				//ROS_INFO("WHY HERE? A");
			}
			//
			
			if (configData.wantsToRemoveDuplicates) {
				if (configData.loadMode) {
					frameCounter++;
				}
				return false;
			}
			
		} else {
			//ROS_ERROR("Not omitting...");
			
			if (configData.wantsToMarkDuplicates) {
				// markCurrentFrameAsDuplicate();
				lastIsDuplicate = false;
			}
			
			//ROS_INFO("SHOULD GET HERE.");
			if (configData.wantsToOutputDuplicates) {
				//ROS_INFO("IS IT WRITING HERE?");
				ofs_duplicates_log << "0" << endl;
			} else {
				//ROS_INFO("WHY HERE? B");
			}
			
		}
		
		frame.copyTo(lastFrame);
		
	}

	//postTime = ros::Time::now();
	//ROS_WARN("1. Elapsed time = (%f)", postTime.toSec()-preTime.toSec());
	
	if (configData.readThermistor) {
		//ROS_WARN("Writing thermistor value... (%f)", lastThermistorReading);
		//ofs_thermistor_log << lastThermistorReading << endl;
	}
	

        //HGH
         //ROS_INFO("Processing image (%d)...", frameCounter);
        if (configData.verboseMode){ROS_INFO("Processing image (%d)...", frameCounter);}

	
	if (pastMeanIndex >= (configData.temporalMemory-1)) {
		pastMeanIndex = 0;
	} else {
		pastMeanIndex++;
	}
	
	if (configData.inputDatatype == DATATYPE_RAW) {
		
		_16bitMat = cv::Mat(frame);
		
		/*
		if (configData.verboseMode) {
			double currMin, currMax;
			minMaxLoc(_16bitMat, &currMin, &currMax);
			ROS_WARN("Current 16-bit image limits = (%f, %f)", currMin, currMax);
		}
		*/
		
		//postTime = ros::Time::now();
		//ROS_WARN("1a. Elapsed time = (%f)", postTime.toSec()-preTime.toSec());
		
		if (configData.normFactor > 0.0) {
					
			//double currMin, currMax;
			double percentile_levels[2];
			percentile_levels[0] = (configData.normFactor / 2.0);
			percentile_levels[1] = 1.0 - (configData.normFactor / 2.0);
			double percentile_values[2];
			
			findPercentiles(_16bitMat, percentile_values, percentile_levels, 2);
			
			thresholdRawImage(_16bitMat, percentile_values);
			
		}

		//postTime = ros::Time::now();
		//ROS_WARN("1b. Elapsed time = (%f)", postTime.toSec()-preTime.toSec());
		
		if (canRadiometricallyCorrect && configData.radiometricCorrection) {
			
			// ROS_WARN("Applying radiometric correction with (%f, %f, %f)", lastThermistorReading, configData.minTemperature, configData.maxTemperature);
			
			if (configData.radiometricBias != 0) {
				_16bitMat += configData.radiometricBias;
			}
			
			radMapper.apply(_16bitMat, temperatureMat, lastThermistorReading, configData.radiometricInterpolation);
			
		}

		//postTime = ros::Time::now();
		//ROS_WARN("1c. Elapsed time = (%f)", postTime.toSec()-preTime.toSec());
		
		if ((configData.outputColorFlag) || (configData.output8bitFlag) || ((configData.wantsToWrite) && ((configData.outputType == "CV_8UC3") || (configData.outputType == "CV_8UC1")) )) {
			//printf("%s << Applying adaptiveDownsample ... (%d)\n", __FUNCTION__, configData.normMode);
			
			if (configData.verboseMode) { ROS_INFO("Entering here x123..."); }
			
			if ((canRadiometricallyCorrect && configData.radiometricCorrection) || configData.alreadyCorrected) {
				
				if (configData.verboseMode) { ROS_INFO("Entering here x124..."); }
				
				if (configData.alreadyCorrected) {
					convertToTemperatureMat(_16bitMat, temperatureMat);
				}
				
				
				if (configData.autoTemperature) {
					
					if (configData.verboseMode) { ROS_INFO("Entering here x125..."); }
					
					if (configData.verboseMode) {
						double currMin, currMax;
						minMaxLoc(temperatureMat, &currMin, &currMax);
						ROS_WARN("Current temp image limits = (%f) : (%f, %f)", abs(currMax-currMin), currMin, currMax);
					}
					
					double currMin, currMax, currRange, newMin, newMax;
					//double percentile_levels[2];
					//percentile_levels[0] = (configData.normFactor / 2.0);
					//percentile_levels[1] = 1.0 - (configData.normFactor / 2.0);
					//double percentile_values[2];
					
					minMaxLoc(temperatureMat, &currMin, &currMax);
					
					//findPercentiles(temperatureMat, percentile_values, percentile_levels, 2);
					
					//if (configData.verboseMode) { ROS_INFO("Temperature limits of (%f, %f) = (%f, %f) C", percentile_levels[0], percentile_levels[1], percentile_values[0], percentile_values[1]); }
					
					//currMin = percentile_values[0];
					//currMax = percentile_values[1];
					
					
					// minMaxLoc(temperatureMat, &currMin, &currMax);
					
					if (configData.stepChangeTempScale) {
						
						currRange = currMax - currMin;
					
						if (currRange < 20.0) {
							newMin = floor(currMin);
							newMax = ceil(currMax);
						} else if (currRange < 50.0) {
							newMin = 2*floor(currMin/2);
							newMax = 2*ceil(currMax/2);
						} else if (currRange < 100.0) {
							newMin = 5*floor(currMin/5);
							newMax = 5*ceil(currMax/5);
						} else if (currRange < 200.0) { 
							newMin = 10*floor(currMin/5);
							newMax = 10*ceil(currMax/5);
						} else if (currRange < 500.0) {
							newMin = 20*floor(currMin/5);
							newMax = 20*ceil(currMax/5);
						} else if (currRange < 1000.0) {
							newMin = 50*floor(currMin/5);
							newMax = 50*ceil(currMax/5);
						} else { 
							newMin = 100*floor(currMin/5);
							newMax = 100*ceil(currMax/5);
						}
						
						//newMin = max(-50.0, newMin);
						//newMax = min(200.0, newMax);
						
						//ROS_ERROR("currRange = (%f), lastMin/Max = (%f, %f), newMin/Max = (%f, %f)", currRange, lastMinDisplayTemp, lastMaxDisplayTemp, newMin, newMax);
						
						if ( (abs(newMin - lastMinDisplayTemp) > 0.1*currRange) || (abs(newMax - lastMaxDisplayTemp) > 0.1*currRange) ) {
							// So if the change in min or max is more than 30% of the current range, then update..
							
							ROS_WARN("MIN/MAX for display changed to = (%f, %f)", newMin, newMax);
							
							lastMinDisplayTemp = newMin;
							lastMaxDisplayTemp = newMax;
							
						}
						
					} else {
						lastMinDisplayTemp = currMin;
						lastMaxDisplayTemp = currMax;
					}
					
					
					
					
					
					
					
					temperatureDownsample(temperatureMat, preFilteredMat, lastMinDisplayTemp, lastMaxDisplayTemp);
					
					//if (configData.verboseMode) { ROS_INFO("Temperature limits of (%f, %f) C", lastMinDisplayTemp, lastMaxDisplayTemp); }
					
			
				} else {
					if (configData.verboseMode) { ROS_INFO("Downsampling with (%f, %f)", configData.minTemperature, configData.maxTemperature); }
					temperatureDownsample(temperatureMat, preFilteredMat, configData.minTemperature, configData.maxTemperature);
				}
				
				
				
				
			} else {
				if (configData.verboseMode) { ROS_INFO("Entering here x126..."); }
				adaptiveDownsample(_16bitMat, preFilteredMat, configData.normMode, configData.normFactor); //, configData.filterMode);
			}
		

		} 
		
		//postTime = ros::Time::now();
		//ROS_WARN("1d. Elapsed time = (%f)", postTime.toSec()-preTime.toSec());
		
		if (configData.output16bitFlag || (configData.wantsToWrite && (configData.outputType == "CV_16UC1"))) {
			
			if (canRadiometricallyCorrect && configData.radiometricCorrection && configData.radiometricRaw) {
				
				temperatureDownsample16(temperatureMat, scaled16Mat);

				
			} else {
				
				scaled16Mat = _16bitMat;
				
			}
			
		}

		//postTime = ros::Time::now();
		//ROS_WARN("1e. Elapsed time = (%f)", postTime.toSec()-preTime.toSec());
		
	} else if (configData.inputDatatype == DATATYPE_8BIT) {

		
		//ROS_WARN("Entered.. 2");
		
		if (frame.channels() == 3) {
			
			//ROS_WARN("Entered.. X");
			
			if (firstFrame) {
				isActuallyGray = checkIfActuallyGray(frame);
				//ROS_WARN("isGray =(%d)", isActuallyGray);
				firstFrame = false;
				
				if (configData.forceInputGray) {
					isActuallyGray = true;
					if (configData.verboseMode) { ROS_INFO("Forcing input images to be considered gray."); }
				}
			}
			
		}
		
		if ((frame.channels() == 1) || (isActuallyGray)) {
			
			if (frame.channels() == 1) {
				workingFrame = cv::Mat(frame);
			} else if (isActuallyGray) {
				if (((configData.outputColorFlag) || (configData.outputType == "CV_8UC3")) || ((configData.output8bitFlag) || (configData.outputType == "CV_8UC1"))) {
					cvtColor(frame, workingFrame, CV_RGB2GRAY);
				}
			}

			//ROS_WARN("Processing...");
			
			// Need to normalize if appropriate
			process8bitImage(workingFrame, preFilteredMat, configData.normMode, configData.normFactor);
			
		} else if (frame.channels() == 3) {
			
			colourMat = cv::Mat(frame);
			
			//ROS_WARN("Processing... @");
			
			if ((configData.output8bitFlag) || (configData.outputType == "CV_8UC1")) {
				cvtColor(colourMat, preFilteredMat, CV_RGB2GRAY);
			}
			
		}

	} else if (configData.inputDatatype == DATATYPE_MM) {
		
		//ROS_WARN("Entered.. 3");
		
		if (frame.channels() != 3) {
			
			ROS_ERROR("Frames must have 3 channels to be used for multi-modal fusion.");
			
			if (firstFrame) {
				isActuallyGray = false;
				firstFrame = false;
			}
			
		} else {
			
			//ROS_ERROR("Processing 3-channel image as an MM...");
		
			// This is where you'll break the image apart and perform fusion...
			
			cv::Mat thermal, visible;
			
			splitMultimodalImage(frame, thermal, visible);
			
			double fusion_params[2];
			fusion_params[0] = max(min(0.5, 0.5 - (fusionFactor/2.0)), 0.0);
			fusion_params[1] = max(min(0.5, 1.0 - (fusionFactor/2.0)), 1.0);

			colourMap.fuse_image(thermal, visible, colourMat, fusion_params);

			
			if ((configData.output8bitFlag) || (configData.outputType == "CV_8UC1")) {
				cvtColor(colourMat, preFilteredMat, CV_RGB2GRAY);
			}
			
		}

	}
	
	//postTime = ros::Time::now();
	//ROS_WARN("2. Elapsed time = (%f)", postTime.toSec()-preTime.toSec());

	
	if (configData.filterMode > 0) {
		applyFilter(preFilteredMat, smoothedMat, configData.filterMode, configData.filterParam);
	} else {
		smoothedMat = preFilteredMat;
	}
	
	if (configData.temporalSmoothing) {
		if (((configData.outputColorFlag) || ((configData.wantsToWrite) && (configData.outputType == "CV_8UC3"))) || ((configData.output8bitFlag) || ((configData.wantsToWrite) && (configData.outputType == "CV_8UC1")))) {
			// If you want to output any kind of 8-bit format...
			
			cv::Scalar means = mean(smoothedMat);
		
			pastMeans[pastMeanIndex] = means[0];
		
			//ROS_WARN("mean = (%f)", pastMeans[pastMeanIndex]);
			
			double temporalMean = 0.0;
			
			for (int iii = 0; iii < min(frameCounter+1, configData.temporalMemory); iii++) {
				
				temporalMean += pastMeans[iii];
				
			}
			
			temporalMean /= min(frameCounter+1, configData.temporalMemory);
			
			//ROS_WARN("temporalMean = (%f)", temporalMean);

			// sign gives temporal vs mean
			// 
			shiftDiff = (((temporalMean - pastMeans[pastMeanIndex]) > 0.0) ? 1.0 : (((temporalMean - pastMeans[pastMeanIndex]) < 0.0) ? -1.0 : 0.0)) * min(abs((pastMeans[pastMeanIndex] - temporalMean)), configData.maxIntensityChange);
			
			_8bitMat = smoothedMat + shiftDiff;
			
			//cv::Scalar meanA = mean(_8bitMat);
			//ROS_WARN("8-bit mean = (%f)", meanA[0]);
			
		}
		
	} else {
		_8bitMat = smoothedMat;
	}

	if ((configData.inputDatatype != DATATYPE_MM) && ((configData.inputDatatype != DATATYPE_8BIT) || (frame.channels() != 3) || isActuallyGray)) {
	//if (1) {
		// If it wasn't a 3-channel 8-bit image to start with
		//ROS_ERROR("There...");
		//imshow("x", _8bitMat);
		//waitKey(1);
		
		if ((configData.outputColorFlag) || ((configData.wantsToWrite) && (configData.outputType == "CV_8UC3"))) {
			// And if you want color output
			
			//ROS_INFO("Entered...");

			colourMap.falsify_image(_8bitMat, colourMat, configData.mapParam);

		}
	}
	
	//postTime = ros::Time::now();
	//ROS_WARN("3. Elapsed time = (%f)", postTime.toSec()-preTime.toSec());

	
	if (configData.displayThermistor) { 
		
		// IMPLEMENTING HYSTERESIS
		
		double displayVal;
		
		// Determine if higher than last displayed..
		if (newThermistorReading > lastDisplayed) {
			// Increasing...
			displayVal = newThermistorReading-0.015;
			//ROS_INFO("Increasing from (%f) to (%f), last displayed was (%f), now displaying (%f)", lastThermistorReading, newThermistorReading, lastDisplayed, displayVal);
		} else {
			// Decreasing...
			displayVal = newThermistorReading+0.015;
			//ROS_INFO("Decreasing from (%f) to (%f), last displayed was (%f), now displaying (%f)", lastThermistorReading, newThermistorReading, lastDisplayed, displayVal);
			
		}
		
		displayVal = round(20.0*displayVal)/20.0;
		
		ROS_INFO("Temperature = (%.2f)", displayVal); 
		
		lastDisplayed = displayVal;
	}
	
	frameCounter++;
	
	if (!readyToPublish) {
		readyToPublish = true;
	}
	
	return true;

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

void streamerNode::publishTopics() {


	initializeMessages();


	/*
	if (alphaChanged) {
			updateMap();
	}
	*/

	bool cameraPublished = false;

	if (configData.wantsToDumpTimestamps) {
			//ROS_WARN("outputFile = (%s)", configData.outputTimeFile.c_str());
			ofs << camera_info.header.stamp.toNSec() << endl;
	}

	//cout << camera_info.header.stamp.toNSec() << endl;

	if ((configData.output16bitFlag) || (configData.wantsToWrite &&  (configData.outputType == "CV_16UC1"))) {

			//_16bitMat_pub.copyTo(testMat);

			if (configData.wantsToUndistort) {
					if (scaled16Mat.data == _16bitMat_pub.data) {
							_16bitMat_pub = cv::Mat(scaled16Mat.size(), scaled16Mat.type());
					}

					remap(scaled16Mat, _16bitMat_pub, map1, map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT, 0);
			} else {
					_16bitMat_pub = cv::Mat(scaled16Mat);
			}

			//if (matricesAreEqual(publishTopics() testMat, _16bitMat_pub)) {
					//ROS_ERROR("Violation!!");
			//} else {
					//ROS_WARN("Non-Violation!!");
			//}

			if (configData.output16bitFlag) {

					std::copy(&(_16bitMat_pub.at<char>(0,0)), &(_16bitMat_pub.at<char>(0,0))+(_16bitMat_pub.cols*_16bitMat_pub.rows*2), msg_16bit.data.begin());



					if (!cameraPublished) {

							pub_16bit.publish(msg_16bit, camera_info);
							cameraPublished = true;
					} else {

							pub_16bit_im.publish(msg_16bit);
					}


					//HGH
					if (configData.republishSource==REPUBLISH_CODE_16BIT){

						if (configData.republishNewTimeStamp){
							//republish with new time stamps
							pub_republish.publish(msg_16bit, camera_info, ros::Time::now());
						} else {
							pub_republish.publish(msg_16bit, camera_info);
						}
					}

			}

	}

	if ((configData.output8bitFlag) || (configData.wantsToWrite &&  (configData.outputType == "CV_8UC1"))) {

			if (configData.wantsToUndistort) {
					if (_8bitMat.data == _8bitMat_pub.data) {
							_8bitMat_pub = cv::Mat(_8bitMat.size(), _8bitMat.type());
					}

					remap(_8bitMat, _8bitMat_pub, map1, map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT, 0);
			} else {
					_8bitMat_pub = cv::Mat(_8bitMat);
			}

			if (configData.output8bitFlag) {

				//HGH
				if (configData.drawReticle){
					line(_8bitMat_pub, cv::Point(0,_8bitMat_pub.rows/2), cv::Point(_8bitMat_pub.cols, _8bitMat_pub.rows/2), cv::Scalar(0),1,8);
					line(_8bitMat_pub, cv::Point(_8bitMat_pub.cols/2,0), cv::Point(_8bitMat_pub.cols/2, _8bitMat_pub.rows), cv::Scalar(0),1,8);
				}

					std::copy(&(_8bitMat_pub.at<unsigned char>(0,0)), &(_8bitMat_pub.at<unsigned char>(0,0))+(_8bitMat_pub.cols*_8bitMat_pub.rows), msg_8bit.data.begin());


					if (!cameraPublished) {
							pub_8bit.publish(msg_8bit, camera_info);

							cameraPublished = true;
					} else {
							pub_8bit_im.publish(msg_8bit);

					}

					//HGH
					 if (configData.republishSource==REPUBLISH_CODE_8BIT_MONO){

						 if (configData.republishNewTimeStamp){
							 //republish with new time stamps
							 pub_republish.publish(msg_8bit, camera_info, ros::Time::now());
						 } else{
							 pub_republish.publish(msg_8bit, camera_info);
						 }
					 }

			}

	}

	if ((configData.outputColorFlag) || (configData.wantsToWrite &&  (configData.outputType == "CV_8UC3"))) {


			if (colourMat.rows > 0) {


					if (configData.wantsToUndistort) {
							if (colourMat.data == colourMat_pub.data) {
									colourMat_pub = cv::Mat(colourMat.size(), colourMat.type());
							}

							remap(colourMat, colourMat_pub, map1, map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT, 0);
					} else {
							colourMat_pub = cv::Mat(colourMat);
					}

					if (configData.outputColorFlag) {

						//HGH
						if (configData.drawReticle){
							line(colourMat_pub, cv::Point(0,colourMat_pub.rows/2), cv::Point(colourMat_pub.cols, colourMat_pub.rows/2), cv::Scalar(0,255,0),1,8);
							line(colourMat_pub, cv::Point(colourMat_pub.cols/2,0), cv::Point(colourMat_pub.cols/2, colourMat_pub.rows), cv::Scalar(0,255,0),1,8);
//                                //overlay lines:
//                                for( int j = 0; j < colourMat_pub.rows; j += 32 ){
//                                    line(colourMat_pub, cv::Point(0, j), cv::Point(colourMat_pub.cols, j), cv::Scalar(0, 255, 0), 1, 8);
//                                }
						}

							std::copy(&(colourMat_pub.at<cv::Vec3b>(0,0)[0]), &(colourMat_pub.at<cv::Vec3b>(0,0)[0])+(colourMat_pub.cols*colourMat_pub.rows*3), msg_color.data.begin());

							//if (configData.verboseMode) { ROS_INFO("%s << seq = (%d, %d)", __FUNCTION__, camera_info.header.seq, msg_color.header.seq); }
							
							if (!cameraPublished) {
									if (configData.verboseMode) { ROS_INFO("%s << Publishing the whole camera...", __FUNCTION__); }
									pub_color.publish(msg_color, camera_info);
									cameraPublished = true;
							} else {
									//if (configData.verboseMode) { ROS_INFO("%s << Publishing just the image...", __FUNCTION__); }
									pub_color_im.publish(msg_color);
							}

							//HGH
							 if (configData.republishSource==REPUBLISH_CODE_8BIT_COL){

								 if (configData.republishNewTimeStamp){
									 //republish with new time stamps
									 pub_republish.publish(msg_color, camera_info, ros::Time::now());
								 }else{
									 pub_republish.publish(msg_color, camera_info);
								 }

							 }
					}


					/*
					imshow("display", colourMat);
					waitKey(1);
					*/
			}
	}



	readyToPublish = false;

}

void streamerNode::writeData() {
	
	if (configData.wantsToWrite) {
		
		//ROS_ERROR("wantsToWrite == true");
				
		if ((frameCounter-1) != lastWritten) {
			
			//ROS_ERROR("Entered write routine...");
		
			char *outputFilename;
			outputFilename = (char*) malloc(256);
			
			if (configData.loadMode && configData.wantsToKeepNames) {
				
				size_t findDot = inputList.at(frameCounter-1).rfind(".");
				
				string partialName;
				partialName = inputList.at(frameCounter-1).substr(0, findDot);
				
				sprintf(outputFilename, "%s/%s.%s", configData.outputFolder.c_str(), partialName.c_str(), configData.outputFormatString.c_str());
				
			} else {
				sprintf(outputFilename, "%s/frame%06d.%s", configData.outputFolder.c_str(), frameCounter-1, configData.outputFormatString.c_str());
			}
			
			if (configData.verboseMode) { ROS_INFO("Output name = (%s), outputType = (%s)", outputFilename, configData.outputType.c_str()); }
			
			if (configData.outputType == "CV_16UC1") {
				if (scaled16Mat.rows > 0) {
					
					if (configData.outputFormatString == "png") {
						imwrite(outputFilename, _16bitMat_pub, configData.outputFileParams);
					} else if ((configData.outputFormatString == "pgm") || (configData.outputFormatString == "ppm")) {
						imwrite(outputFilename, _16bitMat_pub);
					}
				}
			} else if (configData.outputType == "CV_8UC3") {
				if (colourMat.rows > 0) {
					
					//ROS_ERROR("Actually writing...");
					
					if ((configData.outputFormatString == "png") || (configData.outputFormatString == "jpg")) {
						imwrite(outputFilename, colourMat_pub, configData.outputFileParams);
					} else if ((configData.outputFormatString == "bmp") || (configData.outputFormatString == "ppm")) {
						imwrite(outputFilename, colourMat_pub);
					}
				}
			} else if (configData.outputType == "CV_8UC1") {
				if (_8bitMat.rows > 0) {
					if ((configData.outputFormatString == "png") || (configData.outputFormatString == "jpg")) {
						imwrite(outputFilename, _8bitMat_pub, configData.outputFileParams);
					} else if ((configData.outputFormatString == "bmp") || (configData.outputFormatString == "pgm") || (configData.outputFormatString == "ppm")) {
						imwrite(outputFilename, _8bitMat_pub);
					}
				}
			}			
			lastWritten = frameCounter - 1;
		}
	}
	
	if (configData.wantsToEncode) {
		if (!videoInitialized) {
			if (configData.videoType == "CV_16UC1") {
				// 0 writes uncompressed, 1 gives user option
				vid_writer.open(configData.outputVideo, CV_FOURCC('P','I','M','1'), ((int) configData.framerate), scaled16Mat.size(), true);
			} else if (configData.videoType == "CV_8UC3") {
				vid_writer.open(configData.outputVideo, CV_FOURCC('P','I','M','1'), ((int) configData.framerate), colourMat.size()); // , true);
			} else if (configData.videoType == "CV_8UC1") {
				vid_writer.open(configData.outputVideo, CV_FOURCC('X', 'V', 'I', 'D'), ((int) configData.framerate), _8bitMat.size(), false);
			}
			
			videoInitialized = true;
		}
		
		if (configData.videoType == "CV_16UC1") {
			vid_writer << scaled16Mat;
		} else if (configData.videoType == "CV_8UC3") {
			vid_writer << colourMat;
		} else if (configData.videoType == "CV_8UC1") {
			vid_writer << _8bitMat;
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

void streamerNode::assignDefaultCameraInfo() {
	
	globalCameraInfo.imageSize = cv::Mat(1, 2, CV_16UC1);
	globalCameraInfo.imageSize.at<unsigned short>(0, 1) = DEFAULT_IMAGE_HEIGHT;
	globalCameraInfo.imageSize.at<unsigned short>(0, 0) = DEFAULT_IMAGE_WIDTH;
	
	globalCameraInfo.cameraMatrix = cv::Mat::eye(3, 3, CV_64FC1);
	globalCameraInfo.distCoeffs = cv::Mat::zeros(1, 8, CV_64FC1);
	globalCameraInfo.newCamMat = cv::Mat::eye(3, 3, CV_64FC1);
	globalCameraInfo.cameraSize = cv::Size(globalCameraInfo.imageSize.at<unsigned short>(0, 0), globalCameraInfo.imageSize.at<unsigned short>(0, 1));
	
}

void streamerNode::overwriteCameraDims() {
	
	camera_info.height = globalCameraInfo.imageSize.at<unsigned short>(0, 1);
	camera_info.width = globalCameraInfo.imageSize.at<unsigned short>(0, 0);
	
}

streamerNode::streamerNode(ros::NodeHandle& nh, streamerData startupData) {
	
	configData = startupData;
	
	if (configData.wantsToAddExtrinsics) {
		getRectification();
		updateCameraInfoExtrinsics();
	}
	
	sprintf(nodeName, "%s", ros::this_node::getName().substr(1).c_str());
	
	if (configData.verboseMode) { ROS_INFO("Initializing node (%s)", nodeName); }
	
	canRadiometricallyCorrect = false;
	
	deviceCreated = false;
	
	recordedThermistorReadings = 0;
	
	settingsDisabled = false;
	
	currentNucProtectionMode = false; 
	wantsToDisableNuc = false;
	
	lastIsDuplicate = true;
	firstCall = true;
	updateNucInterval = true;
	updateDetectorMode = true;
	updateUSBMode = true;
	altStatus = true;
	performingNuc = false;
	alphaChanged = true;
	readyToPublish = false;
	isActuallyGray = false;
	videoInitialized = false;
	setValidity(true);
	firstFrame = true;
    centerPrincipalPoint = true;
	firstServerCallbackProcessed = false;
	pastMeanIndex = -1;
	medianPercentile = 0.50;
	alternateCounter = 0;
	dodgeTime.sec = 0;
	dodgeTime.nsec = 0;
	fusionFactor = 0.8;
	fileCount = 0;
	writeIndex = 0;
	frameCounter = 0;
	lastWritten = -1;
	lastThermistorReading = -99.0;
	lastMedian = -1.0;
	lastLowerLimit = -1.0;
	lastUpperLimit = -1.0;
	oldMaxDiff = -1.0;
	minVal = 65535.0;
	maxVal = 0.0;
	
	
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
			wantsToShutdown = true;
			prepareForTermination();
			return;
		}
		
		/*
		cv::Mat testX, testX2;
		
		cv::normalize(mappingMatrix, testX, 0, 255, cv::NORM_MINMAX);
		
		testX.convertTo(testX2, CV_8UC1);
		
		cv::imshow("mapping", testX2);
		cv::waitKey(1);
		
		double minIntensity, maxIntensity;
		
		minMaxLoc(sensorLimits, &minIntensity, &maxIntensity);
		ROS_ERROR("%f, %f", minIntensity, maxIntensity);
		
		minMaxLoc(mappingMatrix, &minIntensity, &maxIntensity);
		ROS_ERROR("%f, %f", minIntensity, maxIntensity);
		
		minMaxLoc(testX, &minIntensity, &maxIntensity);
		ROS_ERROR("%f, %f", minIntensity, maxIntensity);
		
		minMaxLoc(testX2, &minIntensity, &maxIntensity);
		ROS_ERROR("%f, %f", minIntensity, maxIntensity);
		*/
		
		radMapper.update(mappingMatrix, sensorLimits.at<double>(0,0), sensorLimits.at<double>(0,1), graylevelLimits.at<double>(0,0), graylevelLimits.at<double>(0,1));
		
		canRadiometricallyCorrect = true;
		
		/*
		cout << "sensorLimits = " << sensorLimits << endl;
		cout << "graylevelLimits = " << graylevelLimits << endl;
		cout << "mappingMatrix = " << mappingMatrix << endl;
		*/
	}
	
	if (configData.serialComms) {
		if (configData.verboseMode) { ROS_INFO("Configuring serial comms..."); }
		if (!configureSerialComms()) {
			ROS_ERROR("Serial comms configuration failed. Shutting down.");
			wantsToShutdown = true;
			prepareForTermination();
			return;
		}
	}
	
	if (configData.verboseMode) { ROS_INFO("configData.serialPollingRate = (%f)", configData.serialPollingRate); }
	
	if (configData.serialPollingRate > 0.1) {
		if (configData.verboseMode) { ROS_INFO("Initializing serial timer at (%f)", 1.0 / ((double) configData.serialPollingRate)); }
		serial_timer = nh.createTimer(ros::Duration(1.0 / ((double) configData.serialPollingRate)), &streamerNode::serialCallback, this);
	} else {
		if (configData.verboseMode) { ROS_INFO("Initializing serial timer at (%f)", DEFAULT_SERIAL_POLLING_RATE); }
		serial_timer = nh.createTimer(ros::Duration(DEFAULT_SERIAL_POLLING_RATE), &streamerNode::serialCallback, this);
	}
	
	it = new image_transport::ImageTransport(nh);
	
	string info_name = configData.topicname.substr(0, configData.topicname.find_last_of("/") + 1) + "camera_info";
	if (configData.verboseMode) { ROS_INFO("configData.topicname = (%s)", info_name.c_str()); }
	
	if (configData.calibrationMode) {
		ros::Duration(1.0).sleep(); // wait a little bit to ensure that the serial comms has been configured..
	}
	
	if (configData.verboseMode) { ROS_INFO("Initializing camera (%s)", configData.topicname.c_str()); }
	
	mainVideoSource = new streamerSource;

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
		
		assignDefaultCameraInfo();
		
		globalCameraInfo.imageSize.at<unsigned short>(0, 1) = configData.inputHeight;
		globalCameraInfo.imageSize.at<unsigned short>(0, 0) = configData.inputWidth;
		globalCameraInfo.cameraSize = cv::Size(globalCameraInfo.imageSize.at<unsigned short>(0, 0), globalCameraInfo.imageSize.at<unsigned short>(0, 1));
		
		// ROS_INFO("Assigned.");
		
	} else {
		if (configData.verboseMode) { ROS_INFO("Assigning default intrinsics..."); }
		
		assignDefaultCameraInfo();
		
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

	if (configData.verboseMode) { ROS_INFO("calling assignCameraInfo() from streamerNode()..."); }
	assignCameraInfo();

	if (configData.verboseMode) { ROS_INFO("Initializing video source..."); }
	mainVideoSource->initialize_video_source();

	colourMap.load_standard(configData.mapCode, configData.mapParam);
	
	refreshCameraAdvertisements();
	
	char cameraInfoName[256];
	sprintf(cameraInfoName, "thermalvis/%s/set_camera_info", nodeName);
	ros::ServiceServer set_camera_info = nh.advertiseService(cameraInfoName, &streamerNode::setCameraInfo, this);


	if (configData.framerate > 0.0) {
		configData.pauseMode = false;
		timer = nh.createTimer(ros::Duration(1.0 / ((double) configData.framerate)), &streamerNode::timerCallback, this);
	} else {
		configData.pauseMode = true;
		timer = nh.createTimer(ros::Duration(1.0 / 1.0), &streamerNode::timerCallback, this);
	}
	
	// Configure log files
	callLogFile = configData.read_addr + "/nodes/streamer/log/call_log.txt";
	retrieveLogFile = configData.read_addr + "/nodes/streamer/log/retrieve_log.txt";
	internalLogFile = configData.read_addr + "/nodes/streamer/log/internal_log.txt";
	writeLogFile = configData.read_addr + "/nodes/streamer/log/write_log.txt";
	duplicatesLogFile = configData.outputFolder + "-duplicates.txt";
	thermistorLogFile = configData.outputFolder + "-thermistor.txt";

	// &PGRCameraNode::publishImage, this, _1
	//f = boost::bind(&streamerNode::serverCallback, this, NULL);
	//f = boost::bind(serverCallback, _1, _2);
	//server.setCallback(f);
	
	if (configData.verboseMode) { ROS_INFO("Establishing server callback..."); }
	f = boost::bind (&streamerNode::serverCallback, this, _1, _2);
    server.setCallback (f);
    
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
	
	if (configData.externalNucManagement != "") {
		
		lastFlagReceived = ros::Time::now();
		
		ROS_INFO("Subscribing to external nuc management flag (%s)", configData.externalNucManagement.c_str());
		nuc_management_sub = nh.subscribe<std_msgs::Float32>(configData.externalNucManagement, 1, &streamerNode::handle_nuc_instruction, this);

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

//HGH
void streamerNode::getRectification(){

    cv::Mat Q;

        cv::stereoRectify(globalExtrinsicsData.cameraMatrix0, globalExtrinsicsData.distCoeffs0, globalExtrinsicsData.cameraMatrix1, globalExtrinsicsData.distCoeffs1,
                                  globalCameraInfo.cameraSize,
                                  globalExtrinsicsData.R, globalExtrinsicsData.T,
                                  globalExtrinsicsData.R0, globalExtrinsicsData.R1, globalExtrinsicsData.P0, globalExtrinsicsData.P1,
                                  Q,
                                  cv::CALIB_ZERO_DISPARITY, configData.alpha, globalCameraInfo.cameraSize, &globalExtrinsicsData.roi0, &globalExtrinsicsData.roi1);

//        ROS_WARN("----- CAM Number %d",configData.camera_number);
//        cout << "R0 = " << globalExtrinsicsData.R0 << endl;
//        cout << "P0 = "  << globalExtrinsicsData.P0 << endl;
//        cout << "R1 = " << globalExtrinsicsData.R1 << endl;
//        cout << "P1 = "  << globalExtrinsicsData.P1 << endl;
//        cout << "newCamMat = " << globalCameraInfo.newCamMat << endl;
//        printf("roi0 (%d, %d, %d, %d) ", int(globalExtrinsicsData.roi0.x), int(globalExtrinsicsData.roi0.y), int(globalExtrinsicsData.roi0.width), int(globalExtrinsicsData.roi0.height));
//        printf("roi1 (%d, %d, %d, %d) ", int(globalExtrinsicsData.roi1.x), int(globalExtrinsicsData.roi1.y), int(globalExtrinsicsData.roi1.width), int(globalExtrinsicsData.roi1.height));
//        cout << "configData.alpha = " << configData.alpha << endl;

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


void streamerNode::handle_camera(const sensor_msgs::ImageConstPtr& msg_ptr, const sensor_msgs::CameraInfoConstPtr& info_msg) {
	
	//ROS_ERROR("handle_camera");
	
	if (configData.syncMode != SYNCMODE_HARD) {
		return;
	}
	
	if ((!configData.subscribeMode) && (!configData.resampleMode)) {
		return;
	}
	
	
	
	
	//ROS_ERROR("Handling an image...");
	
	//cout << "Handling camera..." << msg_ptr->header.stamp.toNSec() << ", " << image_time.toNSec() << endl;
	
	if (configData.verboseMode) { ROS_INFO("Copying camera info over..."); }
	
	original_camera_info = *info_msg;
	
	if (configData.verboseMode) { ROS_INFO("original_camera_info.header.seq = (%d)", original_camera_info.header.seq); }
	
	original_time = info_msg->header.stamp;
	
	//ROS_ERROR("original_bx = (%f)", info_msg->binning_x);
	
	memcpy(&lastThermistorReading, &info_msg->binning_x, sizeof(float));
	
	
	
	//original_bx = 0;
	//original_by = 0;
	
	if (configData.inputDatatype == DATATYPE_DEPTH) {
		cv_ptr = cv_bridge::toCvCopy(msg_ptr, "16UC1");
	} else {
		cv_ptr = cv_bridge::toCvCopy(msg_ptr, enc::BGR8);					// For some reason it reads as BGR, not gray
	}
	
	
	act_on_image();
	
}

void streamerNode::act_on_image() {
	
	//cout << "Acting on image..." << endl;
	
	updateCameraInfo();
	
	newImage = cv::Mat(cv_ptr->image);
	
	cv::Mat grayImage;
	
	if (newImage.type() == CV_16UC3) {
		cv::cvtColor(newImage, frame, CV_RGB2GRAY);
	} else {
		frame = cv::Mat(newImage);
	}
	
	// OPTRIS HACK
	/*
	for (unsigned int iii = 0; iii < frame.rows; iii++) {
		for (unsigned int jjj = 0; jjj < frame.cols; jjj++) {
			frame.at<unsigned short>(iii,jjj) = (frame.at<unsigned short>(iii,jjj) > 32768) ? frame.at<unsigned short>(iii,jjj) - 32768 : frame.at<unsigned short>(iii,jjj) + 32768;
		}
	}
	*/
	
	if (readyToPublish) {
		// This means it still hasn't published the last requested frame
		//return;
	} else {
		readyToPublish = true;
	}
	
	
	if (!configData.resampleMode) {
		
		if (processImage()) {
			if (readyToPublish) {
				updateCameraInfo();
				publishTopics();
				writeData();
			}
		}
		
	}
	
	return;
	
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

void getMapping(int mapIndex, bool extremes, int& mapCode, int& mapParam) {
	
	if (mapIndex == 0) {
		mapCode = GRAYSCALE;
	} else if (mapIndex == 1) {
		mapCode = CIECOMP;
	} else if (mapIndex == 2) {
		mapCode = BLACKBODY;
	} else if (mapIndex == 3) {
		mapCode = RAINBOW;
	} else if (mapIndex == 4) {
		mapCode = IRON;
	} else if (mapIndex == 5) {
		mapCode = BLUERED;
	} else if (mapIndex == 6) {
		mapCode = JET;
	} else if (mapIndex == 7) {
		mapCode = CIELUV;
	} else if (mapIndex == 8) {
		mapCode = ICEIRON;
	} else if (mapIndex == 9) {
		mapCode = ICEFIRE;
	} else if (mapIndex == 10) {
		mapCode = REPEATED;
	} else if (mapIndex == 11) {
		mapCode = HIGHLIGHTED;
	} else {
		mapCode = GRAYSCALE;
	}
	
	if (extremes) {
		mapParam = 0;
	} else {
		mapParam = 1;
	}
	
}

bool streamerNode::isVideoValid() {
	
	if (::wantsToShutdown) {
		if (configData.verboseMode){ ROS_INFO("Wants to shut down.."); }
		setValidity(false);
	}	
		
	return videoValid;
	
}

void streamerNode::setValidity(bool val) {
	if ((val == false) && (configData.verboseMode)) { ROS_INFO("Validity being set to false.."); }
	videoValid = val;
}

streamerSource * streamerNode::getMainVideoSource() {
	return mainVideoSource;
}

cv::VideoCapture * streamerNode::getVideoCapture() {
	return &cap;
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
            updateCameraInfoExtrinsics();

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

void streamerNode::serverCallback(thermalvis::streamerConfig &config, uint32_t level) {
	
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
			if (configData.outputType != "CV_8UC3") { configData.outputType = "CV_16UC1"; }
		} else if (configData.usbMode == USB_MODE_8) {
			config.inputDatatype = DATATYPE_8BIT;
			if (configData.outputType != "CV_8UC3") { configData.outputType = "CV_8UC1"; }
		}
	
		
	}
	
	
	
	/*
	if (configData.readThermistor != config.readThermistor) {
		configData.readThermistor = config.readThermistor;
		
		if (configData.readThermistor) {
			if (configData.verboseMode) { ROS_INFO("Wants to read from thermistor"); }
			//ROS_WARN("Writing thermistor values to (%s)...", thermistorLogFile.c_str());
			//ofs_thermistor_log.open(thermistorLogFile.c_str());
		} else {
			if (ofs_thermistor_log.is_open()) ofs_thermistor_log.close();
			
			// Want system to know that the last reading is invalid..
			lastThermistorReading = -99.0;
		}
		
	}
	*/
	
	//configData.wantsToMarkDuplicates = config.markDuplicates;
	//configData.wantsToOutputDuplicates = config.dumpDuplicates;
	
	//configData.temporalSmoothing = config.temporalSmoothing;
	
	//configData.maxThermistorDiff = config.maxThermistorDiff;
	
	//configData.alternatePeriod = config.alternatePeriod;
	
	if (configData.serialPollingRate != config.serialPollingRate) {
		configData.serialPollingRate = config.serialPollingRate;
		if (configData.verboseMode) { ROS_INFO("Updating period with rate (%f)", configData.serialPollingRate); }
		
		if (configData.serialPollingRate > 0.1) {
			serial_timer.setPeriod(ros::Duration(1.0 / configData.serialPollingRate));
		} else {
			serial_timer.setPeriod(ros::Duration(1.0 / 1.0));
		}
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
	//configData.displayThermistor = config.displayThermistor;
	
	//configData.maxIntensityChange = config.maxShift;
	//configData.temporalMemory = config.temporalMemory;
            
	//configData.syncDiff = config.syncDiff;
	
	
	
	//configData.loopMode = config.loopMode; 
	
	configData.normFactor = config.normFactor;

	/*
	bool updateWriteParams = false;
	if ((configData.writeQuality != config.writeQuality) || (configData.outputFormat != config.outputFormat)) {
		updateWriteParams = true;
	}
	*/
	
	/*
	if (configData.forceInputGray != config.forceInputGray) {
		firstFrame = true;
	}
	*/
	
	//configData.forceInputGray = config.forceInputGray;
	
	//configData.fixDudPixels = config.fixDudPixels;
	
	//configData.writeQuality = config.writeQuality;
	
	if (config.framerate != 0.0) {
		configData.pauseMode = false;
	}
	
	/*
	if ((config.autoAlpha != configData.autoAlpha) || ((!config.autoAlpha) && (config.alpha != configData.alpha))) {
  
		configData.autoAlpha = config.autoAlpha;
		configData.alpha = config.alpha;
		
		alphaChanged = true;
		
		
		
	} else {

                //HGH
                if (configData.alpha != config.alpha){
                    if (configData.wantsToAddExtrinsics){
                        getRectification();
                        updateCameraInfoExtrinsics();
                    }
                }

		configData.alpha = config.alpha;
	}
	*/
	
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

		timer.setPeriod(ros::Duration(1.0 / DEFAULT_READ_RATE));
		
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
			timer.setPeriod(ros::Duration(1.0 / config.framerate));
		} else {
			configData.pauseMode = true;
			timer.setPeriod(ros::Duration(1.0 / 1.0));
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
	
	if (wantsToRefreshCameras) {
		refreshCameraAdvertisements();
	}
            
	getMapping(config.map, configData.extremes, configData.mapCode, configData.mapParam);
	colourMap.load_standard(configData.mapCode, configData.mapParam);
    
	if (configData.outputFolder != "outputFolder") {
		
		/*
		if (config.writeImages) {
			
			if (configData.wantsToWrite != config.writeImages) {
			
				// Create folders if necessary
				struct stat st;
				if(stat(configData.outputFolder.c_str(),&st) != 0) {
					char folderCommand[256];
					sprintf(folderCommand, "mkdir -p %s", configData.outputFolder.c_str());
					int res = system(folderCommand);
					
					if (res == 0) {
						ROS_WARN("system() call returned 0...");
					}
				}
				
			}
			
		}
		
		configData.wantsToWrite = config.writeImages;
		*/
		
		/*
		if (configData.wantsToDumpTimestamps != config.dumpTimestamps) {
			
			if (configData.wantsToDumpTimestamps) {
				ofs.open(configData.outputTimeFile.c_str(), fstream::ate);
			} else {
				ofs.close();
			}
			
		}
			
		configData.wantsToDumpTimestamps = config.dumpTimestamps;
		*/
		
	} else {
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
	
	//configData.wantsToRemoveDuplicates = config.removeDuplicates;
	
	//configData.outputFormat = config.outputFormat;
	
	//ROS_ERROR("configData.outputFormat = %d", configData.outputFormat);
	
	/*
	if (configData.outputFormat == 0) {
		configData.outputFormatString = "jpg";
	} else if (configData.outputFormat == 1) {
		configData.outputFormatString = "pgm";
	} else if (configData.outputFormat == 2) {
		configData.outputFormatString = "bmp";
	} else if (configData.outputFormat == 3) {
		configData.outputFormatString = "ppm";
	} else if (configData.outputFormat == 4) {
		configData.outputFormatString = "png";
	}
	*/
	
	//ROS_ERROR("configData.outputFormatString = %s", configData.outputFormatString.c_str());
	
	/*
	if (config.outputType == 0) {
		configData.outputType = "CV_8UC1";
	} else if (config.outputType == 1) {
		configData.outputType = "CV_8UC3";
		if (configData.outputFormatString == "pgm") {
			ROS_WARN("PGM cannot write as CV_8UC3...");
		}
	} else if (config.outputType == 2) {
		configData.outputType = "CV_16UC1";
		if ((configData.outputFormatString == "jpg") || (configData.outputFormatString == "bmp")) {
			ROS_WARN("JPG/BMP cannot write as CV_16UC1...");
		}
	}
	*/
	
	if (!firstServerCallbackProcessed) {
		firstServerCallbackProcessed = true;
	}
	
	if (configData.verboseMode) { ROS_INFO("Reconfigure request complete..."); }
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

