/*! \file	sparse_flow.cpp
 *  \brief	Definitions for sparse optical flow.
*/

#include "sparse_flow.hpp"

flowManager::flowManager() {
	tD.debugMode = true;
	fD = new featureTrackerNode(tD);

}

bool flowManager::applyToFrame(cv::Mat& targetFrame) {
	
	if (tD.cameraData.cameraSize.width == 0) {
		tD.cameraData.cameraSize.width = targetFrame.cols;
		tD.cameraData.cameraSize.height = targetFrame.rows;
		tD.cameraData.imageSize.at<unsigned short>(0, 0) = tD.cameraData.cameraSize.width;
		tD.cameraData.imageSize.at<unsigned short>(0, 1) = tD.cameraData.cameraSize.height;
		tD.cameraData.updateCameraParameters();
		fD->updateTrackerData(tD);
		fD->process_info();
	}
	
	fD->handle_camera(targetFrame);
	fD->features_loop();
	
	return 0;
}

trackerData::trackerData() : 
	maxFeatures(300), 
	minFeatures(30), 
	flowThreshold(0.002), 
	attemptMatching(true),
	detectEveryFrame(false),
	matchingMode(2),
	//detector_1(1),
	//sensitivity_1(0.1),
	//detector_2(0),
	//sensitivity_2(0.1),
	//detector_3(0),
	//sensitivity_3(0.1),
	adaptiveWindow(false), // originally was true - once fixed it can be again
	velocityPrediction(false), // originally was true - once fixed it can be again 
	maxVelocity(200.0),
	maxFrac(0.05),
	minSeparation(3.0),
	delayTimeout(0.0),
	newFeaturesPeriod(0.5),
	attemptHistoricalRecovery(false), // originally was true - once fixed it can be again  
	//multiplier_1(3),
	//multiplier_2(10),
	verboseMode(false),
	debugMode(false),
	autoTrackManagement(true),
	showTrackHistory(false),
	drawingHistory(25),
	numDetectors(1), 
	outputTrackCount(false), 
	outputFeatureMotion(false) 
{
	detector[0] = "FAST";
}

void trackerData::initializeDescriptors(cv::Ptr<cv::DescriptorExtractor> *desc, cv::Ptr<cv::DescriptorExtractor> *hom) {
	
	desc[0] = new cv::BriefDescriptorExtractor( );
	// desc[0] = new cv::OrbDescriptorExtractor( );
	
	hom[0] = new cv::BriefDescriptorExtractor( );
	
}

void featureTrackerNode::updateTrackerData(trackerData newTrackerData) {
	configData = newTrackerData;
}

void featureTrackerNode::act_on_image() {
	
	#ifdef _BUILD_FOR_ROS_
	cv::Mat newImage(cv_ptr->image);
	#else
	cv::Mat newImage(bridgeReplacement);
	#endif

	if ((newImage.type() == CV_16UC3) || (newImage.type() == CV_16UC1)) {
		displayMessage("This node should only receive 8-bit images..", MESSAGE_ERROR);
		return;
	} else if (newImage.type() == CV_8UC3) {
		cvtColor(newImage, grayImage, CV_RGB2GRAY);
	} else if (newImage.type() == CV_8UC1) {
		grayImage = newImage;
	}
	
	
	
	testTime = timeElapsedMS(test_timer, true);
	//ROS_WARN("Handle time: (%f)", testTime);
	
	testTime = timeElapsedMS(test_timer);
	//ROS_WARN("After equality test: (%f)", testTime);
	
	elapsedTime = timeElapsedMS(cycle_timer);
	
	if (configData.verboseMode) { ROS_INFO("Updating buffer entry (%u from %u) with index (%u)", frameCount % 2, frameCount, currentIndex); }
	bufferIndices[frameCount % 2] = currentIndex;
	grayImage.copyTo(grayImageBuffer[frameCount % 2]);
	
	if (frameCount == 0) {
		for (unsigned int ppp = 0; ppp < MAX_HISTORY_FRAMES; ppp++) {
			grayImageBuffer[0].copyTo(olderImages[ppp]);
			olderTimes[ppp] = original_time;
			olderIndices[ppp] = bufferIndices[0];
		}
	}
	
	//mappedImage = grayImage;
	//mappedImage.copyTo(mappedImageBuffer[(frameCount) % MAXIMUM_FRAMES_TO_STORE]);
	
	testTime = timeElapsedMS(test_timer);
	//ROS_WARN("After remapping: (%f)", testTime);
	
	// Now this is where you should prepare the image pair for tracking...
	
	
	
	cv::Mat tmpX, tmpY;
	
	testTime = timeElapsedMS(test_timer, true);
	//ROS_WARN("Historical prep time: (%f)", testTime);
		
	if (configData.debugMode) {
		// downsampleToColor(grayImageBuffer[(frameCount-1) % MAXIMUM_FRAMES_TO_STORE], drawImage);
		//adaptiveDownsample(grayImageBuffer[(frameCount-1) % MAXIMUM_FRAMES_TO_STORE], dispMat, STANDARD_NORM_MODE);
		
		
		//double vals[2], percentiles[2], thresh = 0.005;
		//percentiles[0] = thresh;
		//percentiles[1] = 1 - thresh;
		
		//findPercentiles(grayImageBuffer[(frameCount) % MAXIMUM_FRAMES_TO_STORE], vals, percentiles, 2);
		
		//displayImage = Mat(grayImageBuffer[(frameCount) % MAXIMUM_FRAMES_TO_STORE].size(), CV_8UC1);
		
		//double v;
		
		//for (int iii = 0; iii < displayImage.rows; iii++) {
			//for (int jjj = 0; jjj < displayImage.cols; jjj++) {
				
				//v = (double) grayImageBuffer[(frameCount) % MAXIMUM_FRAMES_TO_STORE].at<unsigned char>(iii,jjj);
				
				//v -= vals[0];
				//v *= (255.0/(vals[1]-vals[0]));
				//v = max(0.0, v);
				//v = min(255.0, v);
				
				//displayImage.at<unsigned char>(iii,jjj) = (unsigned char) v;
				
			//}
		//}
		
		
		displayImage = grayImageBuffer[frameCount % 2];
		
		//normalize(grayImageBuffer[(frameCount) % MAXIMUM_FRAMES_TO_STORE], displayImage, -255*vals[0], 255 + 255*(vals[1]-255), NORM_MINMAX);
		//normalize(grayImageBuffer[(frameCount) % MAXIMUM_FRAMES_TO_STORE], displayImage, 0, 255, NORM_MINMAX);
		//straightCLAHE(grayImageBuffer[(frameCount) % MAXIMUM_FRAMES_TO_STORE], displayImage, 0.2);
		
		cvtColor(displayImage, drawImage, CV_GRAY2RGB);
		
		
		//displayImage.copyTo(drawImage);
	}
	
	frameCount++;

	if (undergoingDelay) {
		handle_very_new();		// Use this to get homography between images to guide continued tracking
	}
	
	testTime = timeElapsedMS(test_timer, true);
	//ROS_WARN("Detection time: (%f)", testTime);
}

void featureTrackerNode::trimFeatureTrackVector() {
	
	activeTrackIndices.clear();
	
	vector<unsigned int> activeFrameIndices;
	
	activeFrameIndices.push_back(bufferIndices[0]);
	activeFrameIndices.push_back(bufferIndices[1]);
	
	for (unsigned int iii = 0; iii < numHistoryFrames; iii++) {
		if ( (configData.multiplier[iii] != 0) && configData.attemptHistoricalRecovery) {
			activeFrameIndices.push_back(olderIndices[iii]);
			if (configData.verboseMode) { ROS_INFO("Trimming but retaining (%d)...", olderIndices[iii]); }
		}
		
	}
	
	for (unsigned int iii = 0; iii < activeFrameIndices.size(); iii++) {
		if (configData.verboseMode) { ROS_INFO("activeFrameIndices(%d) = (%d)", iii, activeFrameIndices.at(iii)); }
	}
	
	
	
	//ROS_ERROR("activeFrameIndices.size() = (%d)", activeFrameIndices.size());
	
	removeObsoleteElements(featureTrackVector, activeFrameIndices);
		
}

void featureTrackerNode::trimDisplayTrackVectors() {
	
	vector<unsigned int> activeFrameIndices;

	for (unsigned int iii = ((unsigned int) (max(0,currentIndex-configData.drawingHistory))); iii <= ((unsigned int) currentIndex); iii++) {
		activeFrameIndices.push_back(iii);
	}

	removeObsoleteElements(displayTracks, activeFrameIndices);
		
}

void featureTrackerNode::attemptTracking() {
	
	if (previousIndex < 0) { 
		// ROS_ERROR("<%s> previousIndex = (%d)", __FUNCTION__, previousIndex); }
		return;
	}
	
	cv::vector<cv::Point2f> startingPoints, originalFinishingPoints, finishingPoints;
	
	startingPoints.insert(startingPoints.end(), globalStartingPoints.begin(), globalStartingPoints.end());

	// unsigned int ptsBefore = globalStartingPoints.size();
	
	//ROS_INFO("Debug (%d)", 4);
	
	// =========================================================
	// ACTUAL TRACKING
	// =========================================================
	lostTrackIndices.clear();
	if (H12.rows != 0) {
		elapsedTime = timeElapsedMS(cycle_timer, false);
		
		if (configData.verboseMode) { cout << "H12 = " << H12 << endl; }
		
		if (configData.verboseMode) { ROS_INFO("About to attempt to track points (%d, %d), using homography and constraint (%d) threshold (%f)", ((int)globalStartingPoints.size()), ((int)globalFinishingPoints.size()), distanceConstraint, configData.flowThreshold); }
		
		finishingPoints.insert(finishingPoints.end(), startingPoints.begin(), startingPoints.end());
		
		transformPoints(finishingPoints, H12);
		originalFinishingPoints.insert(originalFinishingPoints.end(), finishingPoints.begin(), finishingPoints.end());
			
		#ifdef _BUILD_FOR_ROS_
		while (0) {
			
			cv::Mat im1_col, im2_col;
			
			cvtColor(grayImageBuffer[(readyFrame-1) % 2], im1_col, CV_GRAY2RGB);
			cvtColor(grayImageBuffer[readyFrame % 2], im2_col, CV_GRAY2RGB);
			
			
			//warpPerspective(im1, im1b, H12, im1.size());
			
			cv::Scalar color_x = CV_RGB(255, 0, 0);
			displayKeyPoints(im1_col, startingPoints, im1_col, color_x);
			displayKeyPoints(im2_col, finishingPoints, im2_col, color_x);
			
			//imshow("temp_disp", im1);	// Current image
			
			std::copy(&(im1_col.at<unsigned char>(0,0)), &(im1_col.at<unsigned char>(0,0))+(drawImage.cols*drawImage.rows*drawImage.channels()), msg_debug.data.begin());				
			debug_pub.publish(msg_debug, debug_camera_info);
			cv::waitKey(500);
			//imshow("temp_disp", im2);	// Previous image under transform
			std::copy(&(im2_col.at<unsigned char>(0,0)), &(im2_col.at<unsigned char>(0,0))+(drawImage.cols*drawImage.rows*drawImage.channels()), msg_debug.data.begin());				
			debug_pub.publish(msg_debug, debug_camera_info);
			cv::waitKey(500);
	
		}
		#endif

		cv::Mat H_;
		trackPoints(grayImageBuffer[(readyFrame-1) % 2], grayImageBuffer[readyFrame % 2], startingPoints, finishingPoints, distanceConstraint, 0.0, lostTrackIndices, H_, configData.cameraData);
		// trackPoints(grayImageBuffer[(readyFrame-1) % 2], grayImageBuffer[readyFrame % 2], startingPoints, finishingPoints, distanceConstraint, configData.flowThreshold, lostTrackIndices, H12, configData.cameraData);
		
		if (lostTrackIndices.size() > finishingPoints.size()) {
			ROS_WARN("(%d) tracks lost and (%d) retained over interruption handling.", ((int)lostTrackIndices.size()), ((int)finishingPoints.size()));
			
		} else {
			ROS_INFO("(%d) tracks lost and (%d) retained over interruption handling.", ((int)lostTrackIndices.size()), ((int)finishingPoints.size()));
		}
		
		
	
		/*
		if ((globalFinishingPoints.size() > 10) && (lostTrackIndices.size() < 10)) {
			ROS_ERROR("Freezing next output");
			freezeNextOutput = true;
		}
		*/
		
		//if (configData.verboseMode) { ROS_WARN("Tracking complete."); }
		elapsedTime = timeElapsedMS(cycle_timer, false);
		
	} else {
		cv::Mat H_;
		
		if (configData.verboseMode) { ROS_INFO("About to attempt tracking of (%d) points (no homography for guidance)..", ((int)startingPoints.size())); }
		
		if (configData.velocityPrediction) {
			// ROS_WARN("(%d) About to try and use existing feature velocity knowledge to initialize search locations...", globalFinishingPoints.size());
			// create finishing points estimates...
			//ROS_WARN("Estimating point locations : (%f)", original_time.toSec() - previous_time.toSec());
			#ifdef _BUILD_FOR_ROS_
			assignEstimatesBasedOnVelocities(featureTrackVector, startingPoints, finishingPoints, bufferIndices[(readyFrame-1) % 2], previous_time.toSec(), original_time.toSec());
			#else
			boost::posix_time::time_duration diff = original_time - previous_time;
			assignEstimatesBasedOnVelocities(featureTrackVector, startingPoints, finishingPoints, bufferIndices[(readyFrame-1) % 2], 0.0, diff.total_milliseconds()/1000.0);
			#endif
			originalFinishingPoints.insert(originalFinishingPoints.end(), finishingPoints.begin(), finishingPoints.end());
		} else {
			finishingPoints.insert(finishingPoints.end(), startingPoints.begin(), startingPoints.end());
			originalFinishingPoints.insert(originalFinishingPoints.end(), finishingPoints.begin(), finishingPoints.end());
		}
		
		
		
		//if (configData.verboseMode) { ROS_WARN("About to attempt to track points in regular mode (without homography)..."); }
		trackPoints(grayImageBuffer[(readyFrame-1) % 2], grayImageBuffer[readyFrame % 2], startingPoints, finishingPoints, distanceConstraint, configData.flowThreshold, lostTrackIndices, H_, configData.cameraData);
		
	}
	
	/*
	for (unsigned int xxx = 0; xxx < startingPoints.size(); xxx++) {
		cv::Point2f disp1, disp2, disp3;
		
		disp1.x = startingPoints.at(xxx).x - finishingPoints.at(xxx).x;
		disp1.y = startingPoints.at(xxx).y - finishingPoints.at(xxx).y;
		
		double totalDisplacement = pow(pow(disp1.x,2.0)+pow(disp1.y,2.0),0.5);
		
		if (totalDisplacement > 100.0) {
			ROS_WARN("Point displaced: (%f) : ", totalDisplacement);
		}

		
	}
	*/
	
	if (configData.verboseMode) { ROS_INFO("trackPoints returned (%d) points.", ((int)finishingPoints.size())); }
	
	//ROS_INFO("Debug (%d)", 5);
	
	testTime = timeElapsedMS(test_timer, false);
	//ROS_WARN("2: (%f)", testTime);
	
	//ROS_INFO("(%d / %d) points successfully tracked. last time: (%d) tracks.", globalFinishingPoints.size(), ptsBefore, previouslyTrackedPoints);
	
	successfullyTrackedFeatures += int(globalFinishingPoints.size());
	
	lostTrackCount += int(lostTrackIndices.size());
	
	// Filter points that may be in high-noise region...
	//double distProp = 0.75;
	//filterTrackingsByRadialProportion(globalStartingPoints, globalFinishingPoints, configData.cameraData.Kx, configData.cameraData.expandedCamMat, configData.cameraData.distCoeffs, configData.cameraData.cameraSize, distProp);
	
	//printf("%s::%s << Points (%d:%d) after tracking: %d\n", __PROGRAM__, __FUNCTION__, current_idx, kkk, globalFinishingPoints.size());
	
	
	
	if ((previousIndex == 0) || (currentIndex == 0)) { ROS_WARN("About to add matches from (%d) to (%d): (%d, %d)", previousIndex, currentIndex, ((int)startingPoints.size()), ((int)finishingPoints.size())); }
	addMatchesToVector(featureTrackVector, previousIndex, startingPoints, currentIndex, finishingPoints, lastAllocatedTrackIndex, configData.minSeparation, false);
	
	if (configData.debugMode) {
		addMatchesToVector(displayTracks, previousIndex, startingPoints, currentIndex, finishingPoints, lastAllocatedTrackIndex, configData.minSeparation);
	}
	
	globalStartingPoints.clear();
	globalStartingPoints.insert(globalStartingPoints.end(), startingPoints.begin(), startingPoints.end());
	
	globalFinishingPoints.clear();
	globalFinishingPoints.insert(globalFinishingPoints.end(), finishingPoints.begin(), finishingPoints.end());
	
	
}

void featureTrackerNode::updateDistanceConstraint() {
	if (configData.verboseMode) { ROS_INFO("Entered (%s).", __FUNCTION__); }
		
	int minDim = min(configData.cameraData.cameraSize.width, configData.cameraData.cameraSize.height);
	distanceConstraint = int(double(minDim) * configData.maxFrac);
	distanceConstraint += (distanceConstraint + 1) % 2;
	
	#ifdef _BUILD_FOR_ROS_
	if (configData.verboseMode) { ROS_INFO("Time diff = (%f)", original_time.toSec() - previous_time.toSec()); }
	#else
	if (configData.verboseMode) { 
		boost::posix_time::time_duration diff = original_time - previous_time;
		printf("%s << Time diff = (%f)\n", __FUNCTION__, (diff.total_milliseconds()/1000.0)); 
	}
	#endif

	if (configData.verboseMode) { ROS_INFO("Non-adaptive initial distance constraint = (%d)", distanceConstraint); }
		
	if (configData.adaptiveWindow) {
		
		unsigned int activePoints = 0;
		activePoints += int(globalFinishingPoints.size());
		
		double predictedDisplacement;
		
		if (configData.velocityPrediction) {
			#ifdef _BUILD_FOR_ROS_
			predictedDisplacement = featuresVelocity * (original_time.toSec() - previous_time.toSec());
			#else
			boost::posix_time::time_duration diff = original_time - previous_time;
			predictedDisplacement = featuresVelocity * (diff.total_milliseconds()/1000.0);
			#endif
		} else {
			predictedDisplacement = distanceConstraint;
		}
		
		
		//ROS_INFO("predictedDisplacement = (%f), (%f), (%f)", predictedDisplacement, featuresVelocity, abs(original_time.toSec() - previous_time.toSec()));
		int predictedRequirement = int(ceil(ADAPTIVE_WINDOW_CONTIGENCY_FACTOR*predictedDisplacement));
		predictedRequirement += (predictedRequirement + 1) % 2;
		
		if (configData.verboseMode) { ROS_INFO("activePoints = (%d)", activePoints); }
		
		int maximumSearchDist;
		
		if (activePoints == 0) {
			maximumSearchDist = predictedRequirement;
		} else {
			maximumSearchDist = (distanceConstraint * configData.maxFeatures) / activePoints;
		}
		 
		maximumSearchDist += (maximumSearchDist + 1) % 2;
		
		if (configData.verboseMode) { ROS_INFO("predictedRequirement = (%d)", predictedRequirement); }
		if (configData.verboseMode) { ROS_INFO("maximumSearchDist = (%d)", maximumSearchDist); }

		distanceConstraint = min(predictedRequirement, maximumSearchDist);
		
		
		//ROS_ERROR("activePoints = (%d), maximumSearchDist = (%d), predictedDisplacement = (%f), predictedRequirement = (%d)", activePoints, maximumSearchDist, predictedDisplacement, predictedRequirement);
		
	}
	
	distanceConstraint = max(distanceConstraint, TIGHT_DISTANCE_CONSTRAINT);
	distanceConstraint = min(distanceConstraint, minDim);
	
	if (configData.verboseMode) { ROS_INFO("distanceConstraint = (%d) (%d)", distanceConstraint, minDim); }
}

#ifdef _BUILD_FOR_ROS_
void featureTrackerNode::process_info(const sensor_msgs::CameraInfoConstPtr& info_msg) {
#else
void featureTrackerNode::process_info() {
#endif

	try	{

		// Read in camera matrix
		configData.cameraData.K = cv::Mat::eye(3, 3, CV_64FC1);
		unsigned int maxDistortionIndex;

		#ifdef _BUILD_FOR_ROS_
		for (unsigned int mmm = 0; mmm < 3; mmm++) {
				for (unsigned int nnn = 0; nnn < 3; nnn++) {
						configData.cameraData.K.at<double>(mmm, nnn) = info_msg->K[3*mmm + nnn];
				}
		}
		
		configData.cameraData.cameraSize.width = info_msg->width;
		configData.cameraData.cameraSize.height = info_msg->height;
		
		//ROS_INFO("Distance constraint calculated as (%d)", distanceConstraint);
		
		//ROS_INFO("(%d, %d)\n", configData.cameraData.cameraSize.width, configData.cameraData.cameraSize.height);
		
		
		if (info_msg->distortion_model == "plumb_bob") {
			maxDistortionIndex = 5;
		} else if (info_msg->distortion_model == "rational_polynomial") {
			maxDistortionIndex = 8;
		} else {
			maxDistortionIndex = 5;
		}
		#else
			maxDistortionIndex = 5;
		#endif
		configData.cameraData.distCoeffs = cv::Mat::zeros(1, maxDistortionIndex, CV_64FC1);
		
		#ifdef _BUILD_FOR_ROS_
		for (unsigned int iii = 0; iii < maxDistortionIndex; iii++) {
				configData.cameraData.distCoeffs.at<double>(0, iii) = info_msg->D[iii];		
		}
		#endif

		cout << configData.cameraData.distCoeffs << endl;
		
		configData.cameraData.newCamMat = cv::Mat::zeros(3, 3, CV_64FC1);
		
		cv::Rect validPixROI;
		
		bool centerPrincipalPoint = true;
		
		configData.cameraData.newCamMat = getOptimalNewCameraMatrix(configData.cameraData.K, configData.cameraData.distCoeffs, configData.cameraData.cameraSize, DEFAULT_ALPHA, configData.cameraData.cameraSize, &validPixROI, centerPrincipalPoint);
		
		//double optimalRatio = validPixROI.width / validPixROI.height;
		//double desiredRatio = configData.cameraData.cameraSize.width / configData.cameraData.cameraSize.height;
		
		double expansionFactor = std::min(((double) validPixROI.width) / ((double) configData.cameraData.cameraSize.width), ((double) validPixROI.height) / ((double) configData.cameraData.cameraSize.height));
		
		configData.cameraData.expandedSize = cv::Size(int(((double) configData.cameraData.cameraSize.width) / expansionFactor), int(((double) configData.cameraData.cameraSize.height) / expansionFactor));
		
		//ROS_INFO("originalSize = (%d, %d)\n", configData.cameraData.cameraSize.width, configData.cameraData.cameraSize.height);
		//ROS_INFO("correctedSize = (%d, %d)\n", configData.cameraData.expandedSize.width, configData.cameraData.expandedSize.height);
		
		configData.cameraData.K.copyTo(configData.cameraData.Kx);
		
		configData.cameraData.Kx.at<double>(0,0) /= expansionFactor;
		configData.cameraData.Kx.at<double>(0,2) /= expansionFactor;
		configData.cameraData.Kx.at<double>(1,1) /= expansionFactor;
		configData.cameraData.Kx.at<double>(1,2) /= expansionFactor;
		
		configData.cameraData.expandedCamMat = getOptimalNewCameraMatrix(configData.cameraData.Kx, configData.cameraData.distCoeffs, configData.cameraData.expandedSize, DEFAULT_ALPHA, configData.cameraData.expandedSize, &validPixROI, centerPrincipalPoint);
		
		#ifdef _BUILD_FOR_ROS_
		msg_debug.width = configData.cameraData.cameraSize.width; 
		msg_debug.height = configData.cameraData.cameraSize.height;
		msg_debug.encoding = "bgr8";
		msg_debug.is_bigendian = false;
		msg_debug.step = configData.cameraData.cameraSize.width*3;
		msg_debug.data.resize(configData.cameraData.cameraSize.width*configData.cameraData.cameraSize.height*3);
		ROS_INFO("Optical frame = (%s)", info_msg->header.frame_id.c_str());
		optical_frame = info_msg->header.frame_id;
		#endif

		infoProcessed = true;
		
		ROS_INFO("Image info processed.");
		
	} catch (...) {
		#ifdef _BUILD_FOR_ROS_
		ROS_ERROR("Some failure in reading in the camera parameters...");
		#else
		printf("%s << Some failure in reading in the camera parameters...\n", __FUNCTION__);
		#endif
	}
		
}

void featureTrackerNode::publishRoutine() {
	// =================================
	// PUBLICATION
	// =================================
	if (configData.debugMode) {
		

		//vector<Point2f> redistortedPoints;
		//redistortPoints(globalFinishingPoints[jjj], redistortedPoints, configData.cameraData.Kx, configData.cameraData.distCoeffs, configData.cameraData.expandedCamMat);
		
		

		/*
		vector<cv::Point2f> dummyVelocities;
		addMatchesToVector(displayTracks, previousIndex, finalPoints1, currentIndex, finalPoints2, configData.minSeparation, dummyVelocities);
		*/
		
		if (configData.autoTrackManagement) { trimDisplayTrackVectors(); }
		
		//drawFeatureTracks(drawImage2, drawImage2, displayTracks, drawingColors, current_idx, history);
		
		//vector<featureTrack> redistortedTracks;
		//redistortTracks(displayTracks, redistortedTracks, configData.cameraData.Kx, configData.cameraData.distCoeffs, (readyFrame), configData.cameraData.expandedCamMat, history);
		
		//ROS_WARN("displayTracks[%d].size() = (%d)", kkk, displayTracks.size());
		
		if (configData.verboseMode) { ROS_INFO("Drawing (%d) pts from image index (%d)...", ((int)displayTracks.size()), currentIndex); }
		drawFeatureTracks(drawImage, drawImage, displayTracks, COLOR_TRACKED_PATH, COLOR_TRACKED_POINTS, currentIndex, configData.drawingHistory);
		if (configData.verboseMode) { ROS_INFO("Points drawn."); }
		
			
		/*
		if (preservedRecoveredPoints.size() > 0) {
			
			if (configData.verboseMode) { ROS_INFO("Drawing (%d) recovered pts...", preservedRecoveredPoints.size()); }
			
			//ROS_ERROR("allRecoveredPoints.size() = (%d)", allRecoveredPoints.size());

		
			displayKeyPoints(drawImage, preservedRecoveredPoints, drawImage, COLOR_RECOVERED_POINTS, 0, true);
			preservedRecoveredPoints.clear();
			
			if (0) {
				cv::imshow("test", drawImage);
				cv::waitKey(1);
			}
			
			allRecoveredPoints.clear();
		}
		*/
		
		if (newlySensedFeatures.size() > 0) {
			
			if (configData.verboseMode) { ROS_INFO("Drawing (%d) new pts...", ((int)newlySensedFeatures.size())); }
		
			displayKeyPoints(drawImage, newlySensedFeatures, drawImage, COLOR_UNMATCHED_POINTS, 0, true);
			
		}
		
		if (matchedFeatures.size() > 0) {
			
			if (configData.verboseMode) { ROS_INFO("Drawing (%d) matched pts...", ((int)matchedFeatures.size())); }
			displayKeyPoints(drawImage, matchedFeatures, drawImage, COLOR_MATCHED_POINTS, 0, true);
			
		}
		
		
		
		//ROS_INFO("Debug (%d)", 26);
		
		if (drawImage.size() != configData.cameraData.cameraSize) {
			if (configData.verboseMode) { ROS_INFO("Resizing image..."); }
			resize(drawImage, drawImage_resized, configData.cameraData.cameraSize, 0, 0, cv::INTER_CUBIC);
		} else {
			drawImage_resized = drawImage;
		}
		
		//ROS_INFO("Debug (%d)", 27);
		
		
		if (configData.verboseMode) { ROS_INFO("Copying image (%u, %u) to publisher...", drawImage_resized.rows, drawImage_resized.cols); }
		
		#ifdef _BUILD_FOR_ROS_
		std::copy(&(drawImage_resized.at<cv::Vec3b>(0,0)[0]), &(drawImage_resized.at<cv::Vec3b>(0,0)[0])+(drawImage_resized.cols*drawImage_resized.rows*drawImage_resized.channels()), msg_debug.data.begin());
		#endif

		//ROS_INFO("Debug (%d)", 28);
		
		#ifdef _BUILD_FOR_ROS
		if (configData.verboseMode) { ROS_INFO("Publishing to: (%s) with (%d) and (%d)", debug_pub_name, msg_debug.header.seq, debug_camera_info.header.seq); }
		debug_pub.publish(msg_debug, debug_camera_info);
		if (configData.verboseMode) { ROS_INFO("Published."); }
		#endif
		//ROS_INFO("Debug (%d)", 29);
		
		//imshow("test", drawImage);
		//waitKey(1);


		
	}
	
	//ROS_INFO("Debug (%d)", 30);
	
	#ifdef _BUILD_FOR_ROS_
	int publishedTrackCount = publish_tracks(&tracks_pub, currentIndex);
	#else
	int publishedTrackCount = -1;
	#endif

	if (configData.outputTrackCount) {
		// Successfully tracked
		// Newly detected
		// Ancient recovered
		// Newly discarded
		// Total published
		
		// globalFinishingPoints[0].size()
		trackCountStream << readyFrame << " ";
		trackCountStream << successfullyTrackedFeatures << " ";
		trackCountStream << publishedTrackCount << " ";
		trackCountStream << newlyDetectedFeatures << " ";
		trackCountStream << newlyRecoveredFeatures << " ";
		//trackCountStream << discardedNewFeatures << " ";
		trackCountStream << lostTrackCount << " ";
		trackCountStream << averageTrackLength << endl;
	}
	
	if (configData.outputFeatureMotion) {
		
		// Calculate feature motion
		double motion_x, motion_y;
		
		int tracked = calculateFeatureMotion(bufferIndices[readyFrame % 2], motion_x, motion_y);
		
		if (tracked >= MIN_FEATURES_FOR_FEATURE_MOTION) {
			featureMotionStream << original_time << " ";
			featureMotionStream << currentIndex << " ";
			featureMotionStream << motion_x << " ";
			featureMotionStream << motion_y << " ";
			featureMotionStream << tracked << endl;
		}
		
	}
	
	if (freezeNextOutput) {
		configData.debugMode = false;
	}
	//ROS_INFO("Debug (%d)", 31);
}

void featureTrackerNode::matchWithExistingTracks() {
	
	if (configData.verboseMode) { ROS_INFO("(%s) : Entered function", __FUNCTION__); }
	
	// Determine untracked features

	// Convert points into keypoints
	vector<cv::KeyPoint> featuresFromPts[2];
	
	cv::Point2f ptToAdd;
	cv::KeyPoint kpToAdd;
	kpToAdd.size = 3.0;
	kpToAdd.angle = -1.0;
	kpToAdd.response = 1.0;
	kpToAdd.octave = 0;
	kpToAdd.class_id = 0;
	
	
	
	// IF YOU WANTED YOU COULD USE THE FINAL INDICES TO IDENTIFY WHICH FEATURES LAST
	// APPEARED IN HISTORICAL FRAMES, AND SINCE YOU STILL HAVE ACCESS TO THOSE IMAGES, YOU COULD
	// IN THEORY FORM DESCRIPTORS
	
	// Then get all brand new points:
	
	for (unsigned int iii = 0; iii < newlySensedFeatures.size(); iii++) {
		
		kpToAdd.pt = newlySensedFeatures.at(iii);
		kpToAdd.size = 3.0;
		featuresFromPts[1].push_back(kpToAdd);

	}
	
	if (configData.verboseMode) { ROS_INFO("...with (%d) brand new points", ((int)featuresFromPts[1].size())); }

	cv::Mat lastChanceDescriptors[2];
	
	descriptorExtractor -> compute(grayImageBuffer[readyFrame % 2], featuresFromPts[1], lastChanceDescriptors[1]);
	
	
	allRecoveredPoints.clear();
	preservedRecoveredPoints.clear();
	
	for (unsigned int ppp = 0; ppp <= MAX_HISTORY_FRAMES; ppp++) {
		
		if (configData.verboseMode) { ROS_INFO("(%s) : Looping for ppp = (%d)", __FUNCTION__, ppp); }
		
		if (ppp != MAX_HISTORY_FRAMES) {
			
			if ((configData.multiplier[ppp] == 0.0) || !configData.attemptHistoricalRecovery) {
				continue;
			}
			
			
			
			if (featuresVelocity > configData.maxVelocity) {
				//continue;
			}
			
			//ROS_ERROR("Will try and recover with (%d), featuresVelocity = (%f)", ppp, featuresVelocity);
			
		}
		
		if (configData.verboseMode) { ROS_INFO("(%s) : Passed initial tests..", __FUNCTION__); }
		
		featuresFromPts[0].clear();
		
		unsigned int aimedIndex;
		
		if (ppp == MAX_HISTORY_FRAMES) { // THis is the exception of the loop, where you look at the previous frame
			aimedIndex = bufferIndices[(readyFrame-1) % 2];
		} else {
			aimedIndex = olderIndices[ppp];
		}
		
		for (unsigned int iii = 0; iii < featureTrackVector.size(); iii++) {
			
			if (featureTrackVector.at(iii).locations.at(featureTrackVector.at(iii).locations.size()-1).imageIndex == bufferIndices[(readyFrame) % 2]) {
				continue;
			}
			
			if (ppp == MAX_HISTORY_FRAMES) {
				if (featureTrackVector.at(iii).locations.at(featureTrackVector.at(iii).locations.size()-1).imageIndex == aimedIndex) {
					// This track should have been a lost one...
					ptToAdd = featureTrackVector.at(iii).locations.at(featureTrackVector.at(iii).locations.size()-1).featureCoord;
					kpToAdd.pt = ptToAdd;
					featuresFromPts[0].push_back(kpToAdd);
					
				}
			} else {
				
				// assignHistoricalPoints(featureTrackVector, olderIndices[ppp], bufferIndices[readyFrame % 2], historicalPoints);
				

				for (unsigned int jjj = 0; jjj < featureTrackVector.at(iii).locations.size(); jjj++) {
					
					if (featureTrackVector.at(iii).locations.at(jjj).imageIndex == aimedIndex) {
						ptToAdd = featureTrackVector.at(iii).locations.at(jjj).featureCoord;
						kpToAdd.pt = ptToAdd;
						featuresFromPts[0].push_back(kpToAdd);
						continue;
					}	
					
				}
			}
			
			//ROS_ERROR("%d: (%d)", bufferIndices[readyFrame % 2], featureTrackVector.at(iii).locations.at(featureTrackVector.at(iii).locations.size()-1).imageIndex);
		}
		
		// Create descriptors for untracked features
		
		if (ppp == MAX_HISTORY_FRAMES) { // THis is the exception of the loop, where you look at the previous frame
			descriptorExtractor -> compute(grayImageBuffer[(readyFrame-1) % 2], featuresFromPts[0], lastChanceDescriptors[0]);
		} else {
			descriptorExtractor -> compute(olderImages[ppp], featuresFromPts[0], lastChanceDescriptors[0]);
		}
		
		if ((featuresFromPts[0].size() > 0) && (featuresFromPts[1].size() > 0)) {
		
			if (configData.verboseMode) { ROS_INFO("Will now attempt to match (%d) and (%d) points...", ((int)featuresFromPts[0].size()), ((int)featuresFromPts[1].size())); }
			
			cv::Mat matchingMatrix;
			createMatchingMatrix(matchingMatrix, lastChanceDescriptors[0], lastChanceDescriptors[1]);
			if (configData.verboseMode) { ROS_INFO("Matching matrix created..."); }
			
			vector<vector<cv::DMatch> > matches;
			twoWayPriorityMatching(matchingMatrix, matches);
			if (configData.verboseMode) { ROS_INFO("Initially (%d) achieved (%d) matches...", ppp, ((int)matches.size())); }
			
			vector<cv::Point2f> estimatedFinalLocs;
			
			if (H12.rows != 0) {
				vector<cv::Point2f> tempPts;
				cv::KeyPoint::convert(featuresFromPts[0], tempPts);
				estimatedFinalLocs.insert(estimatedFinalLocs.end(), tempPts.begin(), tempPts.end());
				transformPoints(estimatedFinalLocs, H12);
			} else if ((configData.velocityPrediction) && (ppp == MAX_HISTORY_FRAMES)) {
				vector<cv::Point2f> tempPts;
				cv::KeyPoint::convert(featuresFromPts[0], tempPts);
				#ifdef _BUILD_FOR_ROS_
				assignEstimatesBasedOnVelocities(featureTrackVector, tempPts, estimatedFinalLocs, bufferIndices[(readyFrame-1) % 2], previous_time.toSec(), original_time.toSec());
				#else
				boost::posix_time::time_duration diff = original_time - previous_time;
				assignEstimatesBasedOnVelocities(featureTrackVector, tempPts, estimatedFinalLocs, bufferIndices[(readyFrame-1) % 2], 0.0, diff.total_milliseconds()/1000.0);
				#endif
			} else {
				cv::KeyPoint::convert(featuresFromPts[0], estimatedFinalLocs);
			}
			
			double matchingConstraint;
		
			if (ppp == MAX_HISTORY_FRAMES) { // THis is the exception of the loop, where you look at the previous frame
				matchingConstraint = distanceConstraint;
			} else {
				matchingConstraint = TIGHT_DISTANCE_CONSTRAINT;
			}
			
			cv::Point2f p1, p2;
			for( size_t i = 0; i < matches.size(); i++ ) {
				// Get rid of matches that have moved too far...
				
				p1 = estimatedFinalLocs.at(matches[i][0].queryIdx);
				p2 = featuresFromPts[1].at(matches[i][0].trainIdx).pt;
				
				if (pow(pow(p1.x - p2.x, 2.0) + pow(p1.y - p2.y, 2.0), 0.5) > matchingConstraint) {
					matches.erase(matches.begin() + i);
					i--;
				}
				
				
			}
			
			if (configData.verboseMode) { ROS_INFO("After (%d) distance constraint, achieved (%d) matches...", ppp, ((int)matches.size())); }			
			if (matches.size() > 0) {
				
				sortMatches(matches);
			
				// Reduce points to matched points
				vector<cv::Point2f> points1, points2;
				vector<int> queryIdxs( matches.size() );
				vector<int> trainIdxs( matches.size() );
				
				for( size_t i = 0; i < matches.size(); i++ ) {
					queryIdxs[i] = matches[i][0].queryIdx;
					trainIdxs[i] = matches[i][0].trainIdx;
				}
				
				cv::KeyPoint::convert(featuresFromPts[0], points1, queryIdxs);
				cv::KeyPoint::convert(featuresFromPts[1], points2, trainIdxs);

				if (configData.verboseMode) { ROS_INFO("Still have (%d / %d) (%d/%d) (%d / %d) matches...", featuresFromPts[0].size(), featuresFromPts[1].size(), points1.size(), points2.size(), queryIdxs.size(), trainIdxs.size()); }

				// Integrate matches into feature tracks structure
				if (configData.verboseMode) { ROS_WARN("About to add matches (%d, %d)", points1.size(), points2.size()); }
				
				if (ppp != MAX_HISTORY_FRAMES) {
					//continue;
				}
				
				addMatchesToVector(featureTrackVector, aimedIndex, points1, bufferIndices[readyFrame % 2], points2, lastAllocatedTrackIndex, configData.minSeparation, false);
				
				if (configData.debugMode) {
					addMatchesToVector(displayTracks, aimedIndex, points1, bufferIndices[readyFrame % 2], points2, lastAllocatedTrackIndex, configData.minSeparation);
				}
				
				if (configData.verboseMode)  { ROS_INFO("Added (%d) (%d) matches to vector", ppp, points2.size()); }
				
				matchedFeatures.insert(matchedFeatures.end(), points2.begin(), points2.end());
				
				
				// concatenation should rightly destroy the input vector, because all newly detected points should have already been added to globalFinishingPoints
				
				concatenateWithExistingPoints(globalFinishingPoints, points2, configData.maxFeatures, configData.minSeparation);
				if (configData.verboseMode) { ROS_INFO("After (%d) concat, have (%d) points...", ppp, points2.size()); }
				
			}
			
			
			
		}
	
		// concatenateWithExistingPoints(globalFinishingPoints, recoveredPoints, configData.maxFeatures, configData.minSeparation);
		
	}
	
	
	
	
	
	if (configData.verboseMode) { ROS_INFO("(%s) : Exiting function", __FUNCTION__); }
	
}

void featureTrackerNode::detectNewFeatures() {
	vector<cv::KeyPoint> currPoints;
	
	
	
	//for (unsigned int jjj = 0; jjj < configData.numDetectors; jjj++) {
		//allPoints.insert(allPoints.end(), globalFinishingPoints.begin(), globalFinishingPoints.end());
		
	//}
	
	if (configData.verboseMode) { ROS_INFO("Entered (%s) : Currently have (%d) points", __FUNCTION__, globalFinishingPoints.size()); }
	
	//ROS_INFO("Debug (%d)", 11);
	
	
	for (unsigned int jjj = 0; jjj < configData.numDetectors; jjj++) {
		

		
		if (configData.verboseMode) { ROS_INFO("Considering application of detector (%u), with (%d) pts", jjj, globalFinishingPoints.size()); }
		
		//ROS_INFO("Debug (%d)", 12);
		
		testTime = timeElapsedMS(test_timer, false);
		//ROS_WARN("1: (%f)", testTime);

		
		bool wantNewDetection = false;
		
		if (globalFinishingPoints.size() < ((unsigned int) configData.maxFeatures)) {
			wantNewDetection = true;
			if (configData.verboseMode) { ROS_INFO("Detector (%u) wants new detection because you have (%d) points compared with a maximum of (%d).", jjj, globalFinishingPoints.size(), configData.maxFeatures); }
		} 
		
		testTime = timeElapsedMS(test_timer, false);
		//ROS_WARN("2: (%f)", testTime);

		
		//ROS_INFO("Debug (%d)", 13);
		
		if (wantNewDetection) {
			
			//ROS_INFO("Debug (%d)", 14);
			
			currPoints.clear();
			keypointDetector[jjj] -> detect(grayImageBuffer[readyFrame % 2], currPoints);
			
			if (configData.verboseMode) { ROS_INFO("Detector (%u) found (%d) points.", jjj, currPoints.size()); }
			
			//ROS_INFO("Debug (%d)", 15);
			
			//ROS_INFO("(%d) new points detected...", currPoints[jjj].size());
			
			discardedNewFeatures += int(currPoints.size());

			testTime = timeElapsedMS(test_timer, false);
			//ROS_WARN("3: (%f)", testTime);
			
			
			if (currPoints.size() != 0) {
				
				
				
				//ROS_INFO("Debug (%d)", 16);
			
				sortKeyPoints(currPoints);
				
				//ROS_INFO("Debug (%d) : (%d)", 161, currPoints[jjj].size());
				
				testTime = timeElapsedMS(test_timer, false);
				//ROS_WARN("4: (%f)", testTime);
									
				vector<cv::Point2f> candidates;
				
				reduceEdgyFeatures(currPoints, configData.cameraData);
				
				//ROS_INFO("Debug (%d)", 162);
				
				testTime = timeElapsedMS(test_timer, false);
				//ROS_WARN("5: (%f)", testTime);
				
				//ROS_INFO("(%d) points after edge-reduction...", currPoints[jjj].size());

				//reduceWeakFeatures(workingImNew, currPoints[jjj], minResponse);
				
				testTime = timeElapsedMS(test_timer, false);
				//ROS_WARN("6: (%f)", testTime);
				
				//ROS_INFO("(%d) points after weakness-reduction...", currPoints[jjj].size());
				
				// Want to cull new points to fit limit
				
				//ROS_INFO("Debug (%d)", 17);
				
				for (unsigned int zzz = 0; zzz < currPoints.size(); zzz++) {
					// if distance between [ currPoints[jjj].at(zzz) ] and all points in globalFinishingPoints[jjj]
					// is greater than [ configData.minSeparation ] , delete and decrement index
					
					bool violatesProximity = false;
					
					for (unsigned int yyy = 0; yyy < globalFinishingPoints.size(); yyy++) {
						
						if (distBetweenPts2f(currPoints.at(zzz).pt, globalFinishingPoints.at(yyy)) < configData.minSeparation) {
							violatesProximity = true;
							break;
						}
						
					}
					
					if (violatesProximity) {
						
						// delete
						currPoints.erase(currPoints.begin() + zzz);
						zzz--;
					}
				}
				
				if (configData.verboseMode) { ROS_INFO("Reduced to (%d) candidate points based on proximity.", currPoints.size()); }
				
				//ROS_INFO("Debug (%d)", 18);
				
				if (int(currPoints.size() + globalFinishingPoints.size()) > configData.maxFeatures) {
					currPoints.erase(currPoints.begin() + (configData.maxFeatures - globalFinishingPoints.size()), currPoints.end());
					
					//ROS_ERROR("Reduced to (%d) points because of maxFeatures...", currPoints[jjj].size());
					
				}

				if (configData.verboseMode) { ROS_INFO("Further reduced to (%d) candidate points based on maxFeatures limit.", currPoints.size()); }
				
				newlyDetectedFeatures += int(currPoints.size());
				discardedNewFeatures -= int(currPoints.size());
				
				cv::KeyPoint::convert(currPoints, candidates);
				
				//ROS_INFO("Debug (%d)", 19);
				
				if (candidates.size() > 0) {
					
					//ROS_INFO("Debug (%d)", 20);
					
					//cornerSubPix(workingImNew, candidates, Size(1,1), Size(-1,-1), cvTermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 15, 0.1)); // cvTermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 15, 0.1)
				
					testTime = timeElapsedMS(test_timer, false);
					//ROS_WARN("7: (%f)", testTime);
				
					// reduce candidates based on features existing in close proximity
					//reduceProximalCandidates(allPoints, candidates);
					//reduceEdgyCandidates(candidates, configData.cameraData);
					
					testTime = timeElapsedMS(test_timer, false);
					//ROS_WARN("8: (%f)", testTime);
					
					//ROS_INFO("(%d) points after proximity-reduction...", candidates.size());
					
					//printf("%s << candidates after reducing ones close to edge: %d\n", __FUNCTION__, candidates.size());
					//reduceUnrefinedCandidates(candidates);
					//printf("%s << candidates after reducing ones with subpixel refinement: %d\n", __FUNCTION__, candidates.size());
					//allPoints.insert(allPoints.end(), candidates.begin(), candidates.end());
					
					//unsigned int beforeCount = 
					
					
					
					if (configData.verboseMode) { ROS_INFO("Size of featureTrackVector before adding projections on frame (%d) = (%d)", bufferIndices[readyFrame % 2], featureTrackVector.size()); }
					
					if (featureTrackVector.size() > 0) {
						if (featureTrackVector.at(featureTrackVector.size()-1).locations.size() > 0) {
							if (configData.verboseMode) { ROS_INFO("Latest index in vector = (%d)", featureTrackVector.at(featureTrackVector.size()-1).locations.at(featureTrackVector.at(featureTrackVector.size()-1).locations.size()-1).imageIndex); }
						}
					}
					
					if (bufferIndices[readyFrame % 2] == 0) {
						//ROS_ERROR("bufferIndices[%d mod 2] is 0!", readyFrame);
					}
					
					int before = lastAllocatedTrackIndex;
					addProjectionsToVector(featureTrackVector, bufferIndices[readyFrame % 2], candidates, lastAllocatedTrackIndex, configData.minSeparation);
					
					
					clearDangerFeatures(featureTrackVector, lastAllocatedTrackIndex);
					
					//ROS_ERROR("lastAllocatedTrackIndex = (%d) -> (%d)", before, ((int)lastAllocatedTrackIndex));
					
					if (configData.verboseMode) { ROS_INFO("Size of featureTrackVector after adding projections = (%d)", featureTrackVector.size()); }
					
					if (configData.verboseMode) { ROS_ERROR("About to concatenate with (%d) + (%d) / (%d) points and minsep of (%f)", globalFinishingPoints.size(), candidates.size(), configData.maxFeatures, configData.minSeparation); }
					
					concatenateWithExistingPoints(globalFinishingPoints, candidates, configData.maxFeatures, configData.minSeparation);
					
					if (configData.verboseMode) { ROS_INFO("Size of finishing points / candidates after concatenation = (%d / %d)", globalFinishingPoints.size(), candidates.size()); }
					
					
					//printf("%s << left after concatenation: %d\n", __FUNCTION__, globalFinishingPoints[jjj].size());
					
					if (configData.verboseMode) { ROS_WARN("Adding (%d) new features", candidates.size()); }
					
					newlySensedFeatures.insert(newlySensedFeatures.end(), candidates.begin(), candidates.end());
					
					testTime = timeElapsedMS(test_timer, false);
					//ROS_WARN("9: (%f)", testTime);
					
					//ROS_INFO("Debug (%d)", 21);
					
				}
				
				//ROS_INFO("Debug (%d)", 22);
				
				
				
				//ROS_INFO("Debug (%d)", 23);
				
			}
			
			//ROS_INFO("Debug (%d)", 24);
			
		}
		
	}
	
	//ROS_INFO("Debug (%d)", 25);
	
	if (H12.rows != 0) {
		H12.release();
	}
}

#ifdef _BUILD_FOR_ROS_
void featureTrackerNode::features_loop(const ros::TimerEvent& event) {
#else
void featureTrackerNode::features_loop() {
#endif
	successfullyTrackedFeatures = 0;
	newlyDetectedFeatures = 0;
	newlyRecoveredFeatures = 0;
	discardedNewFeatures = 0;
	lostTrackCount = 0;
	
	matchedFeatures.clear();
	newlySensedFeatures.clear();
	
	if (readyFrame >= frameCount) {
		return;
	}
	
	//ROS_INFO("Entered features loop for frame (%d)", frameCount);
	
	vector<cv::Point2f> recPoints1, recPoints2;
	
	testTime = timeElapsedMS(test_timer, true);
	//ROS_WARN("?NUC time: (%f)", testTime);
	
	if (configData.verboseMode) { ROS_ERROR("=========== Starting features loop for frame (%u [%u]) with (%d) finishing points.", readyFrame, bufferIndices[readyFrame % 2], globalFinishingPoints.size()); }
	
	//if (readyFrame > 0) {
		
	if (configData.verboseMode) { ROS_WARN("About to update distance constraint."); }
	updateDistanceConstraint();
	if (configData.verboseMode) { ROS_WARN("Distance constraint updated.\n"); }
	
	globalStartingPoints.clear();
	globalStartingPoints.insert(globalStartingPoints.end(), globalFinishingPoints.begin(), globalFinishingPoints.end());
	
	if (H12.rows != 0) {		
		transformPoints(globalFinishingPoints, H12);
		featuresVelocity = 9e99;
		if (configData.verboseMode) { ROS_INFO("(%d) Points transformed.", globalFinishingPoints.size()); }
	} else {
		globalFinishingPoints.clear();
	}
	
	if (configData.verboseMode) { ROS_WARN("About to attempt tracking..."); }
	attemptTracking();
	if (configData.verboseMode) { ROS_WARN("Tracking completed.\n"); }
	
	#ifdef _BUILD_FOR_ROS_
	double prelimVelocity = obtainFeatureSpeeds(featureTrackVector, bufferIndices[(readyFrame-1) % 2], previous_time.toSec(), bufferIndices[(readyFrame) % 2], original_time.toSec());
	#else
	boost::posix_time::time_duration diff = original_time - previous_time;
	double prelimVelocity = obtainFeatureSpeeds(featureTrackVector, bufferIndices[(readyFrame-1) % 2], 0.0, bufferIndices[(readyFrame) % 2], diff.total_milliseconds()/1000.0);
	#endif
	featuresVelocity = max(prelimVelocity, featuresVelocity);

	bool featuresTooLow;
	
	if (configData.verboseMode) { ROS_ERROR("Using (%d) to update (%d).", ((int)globalFinishingPoints.size()), previousTrackedPointsPeak); }
	
	previousTrackedPointsPeak = max(previousTrackedPointsPeak, ((unsigned int) globalFinishingPoints.size()));
	
	if (((int)globalFinishingPoints.size()) < configData.minFeatures) {
	//if (((int) globalFinishingPoints.size()) < max(configData.minFeatures, ((int) ((1.00 - FEATURE_DROP_TRIGGER)*double(previousTrackedPointsPeak))) ) ) {
		featuresTooLow = true;
		if (configData.verboseMode) { ROS_ERROR("featuresTooLow == true, because feature count is (%d) vs (%d, %u).", globalFinishingPoints.size(), configData.minFeatures, previousTrackedPointsPeak); }
		previousTrackedPointsPeak = (unsigned int)(globalFinishingPoints.size());
	} else {
		featuresTooLow = false;
	}
	
	/*
	if (readyFrame == 0) {
		featuresTooLow = true;
	} else {
		featuresTooLow = false;
	}
	*/
	
	if (cycleFlag) {
		if (configData.verboseMode) { ROS_INFO("Cycle flag is true (%d) vs (%d, %d, %d)", cycleCount, configData.multiplier[0], configData.multiplier[1], configData.multiplier[2]); }
	}
	
	
	
	if (featuresTooLow || configData.detectEveryFrame || (H12.rows != 0)) {
		if (configData.verboseMode) { ROS_INFO("About to detect new features on frame (%d).. because (%d, %d, %d, %d)", currentIndex, featuresTooLow, cycleFlag, configData.detectEveryFrame, (H12.rows != 0)); }
		detectNewFeatures();
		if (configData.verboseMode) { ROS_INFO("(%d) New features detected..", ((int)newlySensedFeatures.size())); }
		
		
	}
	
	if (configData.attemptMatching && (readyFrame > 0) && (newlySensedFeatures.size() > 0)) { // or perhaps, if there are any features to match
		// Attempt to match features
		matchWithExistingTracks();
	}

	
	for (unsigned int ppp = 0; ppp < MAX_HISTORY_FRAMES; ppp++) {
		
		if ((configData.multiplier[ppp] == 0.0) || !configData.attemptHistoricalRecovery) {
			
			continue;
		}
		
		if (configData.verboseMode) { ROS_INFO("cycleCount = (%d) vs configData.multiplier[%d] = (%d)", cycleCount, ppp, configData.multiplier[ppp]); }
		
		if ( ((cycleCount % configData.multiplier[ppp]) == 0) && (cycleFlag) ) {
			
			//  (featuresVelocity > configData.maxVelocity)
			
			if (configData.verboseMode) { ROS_INFO("About to replace historical data (%d)..", ppp);}
			
			grayImageBuffer[readyFrame % 2].copyTo(olderImages[ppp]);
			olderTimes[ppp] = original_time;
			olderIndices[ppp] = bufferIndices[readyFrame % 2];
			
			/*
			if (featuresVelocity > configData.maxVelocity) {
				cycleCount = 0;
			}
			*/
		}
		
	}
	
	cycleFlag = false;
	
	
	/*
	// Ideally would do too this if: featuresTooLow & H12.rows != 0
	if (cycleFlag || (H12.rows != 0)) {
		
		if (configData.attemptHistoricalRecovery && (configData.newFeaturesPeriod > 0.0) ) {
			if (configData.verboseMode) { ROS_WARN("About to attempt to recover tracks."); }
			unsigned int recoveredTrackCount = recoverTracks();
			if (configData.verboseMode) { ROS_WARN("Recovered (%d) tracks\n", recoveredTrackCount); }
		}
		
		cycleFlag = false;

	}
	*/
	
	if (configData.verboseMode) { ROS_WARN("globalFinishingPoints.size() = (%d)", ((int)globalFinishingPoints.size())); }
	
	if (readyFrame > 0) {
		
		
	
		if ((globalFinishingPoints.size() < ((unsigned int) configData.minFeatures/2)) && !lowPointsWarning) {
			lowPointsWarning = true;
			ROS_WARN("Successfully tracked points (%d) is currently low...", globalFinishingPoints.size());

			
		} else if ((globalFinishingPoints.size() > ((unsigned int) configData.minFeatures)) && lowPointsWarning) {
			lowPointsWarning = false;
			ROS_INFO("Minimum feature count now re-achieved.");
		}
		
		//featuresVelocity = calculateFeatureSpeeds(finalPoints1, finalPoints2, finalVelocities, previous_time.toSec(), original_time.toSec());
		
		//ROS_INFO("About to attempt to update feature speeds.");
		#ifdef _BUILD_FOR_ROS_
		featuresVelocity = updateFeatureSpeeds(featureTrackVector, bufferIndices[(readyFrame-1) % 2], previous_time.toSec(), bufferIndices[(readyFrame) % 2], original_time.toSec(), configData.maxVelocity);
		#else
		boost::posix_time::time_duration diff = original_time - previous_time;
		featuresVelocity = updateFeatureSpeeds(featureTrackVector, bufferIndices[(readyFrame-1) % 2], 0.0, bufferIndices[(readyFrame) % 2], diff.total_milliseconds()/1000.0, configData.maxVelocity);
		#endif
		
		//ROS_INFO("Done.");
	}
	
	publishRoutine();
	
	if (configData.verboseMode) { ROS_WARN("featuresVelocity = (%f)", featuresVelocity); }
	
	if (configData.velocityPrediction) {
		
		unsigned int activeTrackCount = getActiveTrackCount(featureTrackVector, bufferIndices[(readyFrame-1) % 2], bufferIndices[(readyFrame) % 2]);
		// ROS_ERROR("activeTrackCount = (%d)", activeTrackCount);
		if ((featuresVelocity == 0.0) && (activeTrackCount > 0)) { 
			ROS_WARN("featuresVelocity = (%f) : Are you forgetting to mark duplicate images using <streamer>?", featuresVelocity); 
		} else if (configData.verboseMode) {
			#ifdef _BUILD_FOR_ROS_
			ROS_INFO("featuresVelocity = (%f) over (%f) seconds", featuresVelocity, (original_time.toSec()-previous_time.toSec()));
			#else
			boost::posix_time::time_duration diff = original_time - previous_time;
			printf("%s << featuresVelocity = (%f) over (%f) seconds\n", __FUNCTION__, featuresVelocity, diff.total_milliseconds()/1000.0);
			#endif
		}
	}
	
	if (configData.verboseMode) { ROS_ERROR("Completed features loop\n"); }
	
	readyFrame++;
	
	
}

int featureTrackerNode::calculateFeatureMotion(unsigned int idx, double& mx, double &my) {
	
	// Go through all active features
	// Find ones that were tracked in previous frame
	// Add to calcs
	
	double xSumm = 0.0, ySumm = 0.0;
	int count = 0;
	
	int *monitorIndices;
	
	monitorIndices = new int[featureTrackVector.size()]; // 0 means can't use, 1 means 2nd frame, 2 means 3rd frame
	
	if (configData.normalizeFeatureVelocities) {
		
		// 0. INITIALLY CLASSIFY ALL FEATURES IN TERMS OF THEIR RECENT HISTORY (0, 1 or 2 recent consecutive appearances)
		// Also, count how many have had 2 recent consecutive appearances
		
		// For each existing feature
		for (unsigned int iii = 0; iii < featureTrackVector.size(); iii++) {
		
			// If this feature was detected in the most recent frame
			if (featureTrackVector.at(iii).locations.at(featureTrackVector.at(iii).locations.size()-1).imageIndex == idx) {
				
				// If it has at least 3 detections
				if (featureTrackVector.at(iii).locations.size() > 2) {
					
					// If the 2nd most recent detection was in the previous frame
					if (featureTrackVector.at(iii).locations.at(featureTrackVector.at(iii).locations.size()-2).imageIndex == (idx-1)) {
						
						// And the 3rd most recent detection was in the frame before that
						if (featureTrackVector.at(iii).locations.at(featureTrackVector.at(iii).locations.size()-3).imageIndex == (idx-2)) {
							
							// Now you have a 3rd frame!
							count++;
							
							monitorIndices[iii] = 2;
							
						} else {
							monitorIndices[iii] = 1;
						}
						
					} else {
						monitorIndices[iii] = 0;
					}
					
				} else if (featureTrackVector.at(iii).locations.size() > 1) {
					
					if (featureTrackVector.at(iii).locations.at(featureTrackVector.at(iii).locations.size()-2).imageIndex == (idx-1)) {
						
						monitorIndices[iii] = 1;
						
					} else {
						monitorIndices[iii] = 0;
					}
					
				} else {
					monitorIndices[iii] = 0;
				}
				
			} else {
				monitorIndices[iii] = 0;
			}
			
		}
		
		// All features that have already been successfully tracked (i.e. this is their 3rd frame) have valid velocity weightings
		
		// Assign all velocities
		
		// Establish an average velocity from the 3rd appearance features [will this basically be your output?]
		
		if (count > 0) {
			for (unsigned int iii = 0; iii < featureTrackVector.size(); iii++) {
				
				if (monitorIndices[iii] >= 2) {
								
					featureTrackVector.at(iii).velocity_x = (featureTrackVector.at(iii).locations.at(featureTrackVector.at(iii).locations.size()-1).featureCoord.x - featureTrackVector.at(iii).locations.at(featureTrackVector.at(iii).locations.size()-2).featureCoord.x);
					featureTrackVector.at(iii).velocity_y = (featureTrackVector.at(iii).locations.at(featureTrackVector.at(iii).locations.size()-1).featureCoord.y - featureTrackVector.at(iii).locations.at(featureTrackVector.at(iii).locations.size()-2).featureCoord.y);
					
				}
				
				if (monitorIndices[iii] == 2) {
					
					xSumm += featureTrackVector.at(iii).velocity_x;
					ySumm += featureTrackVector.at(iii).velocity_y;
					
				} 
				
			}
			
			mx = xSumm / double(count);
			my = ySumm / double(count);
			
			if (configData.verboseMode) { ROS_INFO("Average feature motion since previous frame (%d / %d): (%06.2f, %06.2f)", count, 2, mx, my); }
			
			// Re-calculate weightings so that they all normalize to this average velocity
			
			for (unsigned int iii = 0; iii < featureTrackVector.size(); iii++) {
				
				if (monitorIndices[iii] == 2) {
								
					featureTrackVector.at(iii).velocityWeighting = mx / featureTrackVector.at(iii).velocity_x;
					
				} 
				
			}
			
			// Now go through all 2nd appearance features, assign them a weighting to normalize them to the velocity from the reliable (3rd frame) features
			
			for (unsigned int iii = 0; iii < featureTrackVector.size(); iii++) {
				
				if (monitorIndices[iii] == 1) {
								
					featureTrackVector.at(iii).velocityWeighting = mx / featureTrackVector.at(iii).velocity_x;
					
				} 
				
			}
			
		} else {
			// If no 3rd appearance features, assign a weighting to 2nd appearance features to normalize them to their own average velocity
			// These features will now be valid references in future
			
			for (unsigned int iii = 0; iii < featureTrackVector.size(); iii++) {
				
				
				if (monitorIndices[iii] == 1) {
					
					xSumm += featureTrackVector.at(iii).velocity_x;
					ySumm += featureTrackVector.at(iii).velocity_y;
					
					count++;
					
				} 
				
			}
			
			mx = xSumm / double(count);
			my = ySumm / double(count);
			
			if (configData.verboseMode) { ROS_INFO("Average feature motion since previous frame (%d / %d): (%06.2f, %06.2f)", count, 1, mx, my); }
			
			for (unsigned int iii = 0; iii < featureTrackVector.size(); iii++) {
				
				if (monitorIndices[iii] == 1) {
								
					featureTrackVector.at(iii).velocityWeighting = mx / featureTrackVector.at(iii).velocity_x;
					
				} 
				
			}
			
		}
	
	} else {
		
		for (unsigned int iii = 0; iii < featureTrackVector.size(); iii++) {
		
			if (featureTrackVector.at(iii).locations.at(featureTrackVector.at(iii).locations.size()-1).imageIndex == idx) {
				
				if (featureTrackVector.at(iii).locations.size() > 1) {
					
					if (featureTrackVector.at(iii).locations.at(featureTrackVector.at(iii).locations.size()-2).imageIndex == (idx-1)) {
						
						xSumm += (featureTrackVector.at(iii).locations.at(featureTrackVector.at(iii).locations.size()-1).featureCoord.x - featureTrackVector.at(iii).locations.at(featureTrackVector.at(iii).locations.size()-2).featureCoord.x);
						ySumm += (featureTrackVector.at(iii).locations.at(featureTrackVector.at(iii).locations.size()-1).featureCoord.y - featureTrackVector.at(iii).locations.at(featureTrackVector.at(iii).locations.size()-2).featureCoord.y);
						
						count++;
						
					}
					
				}
				
			}
			
		}
		
		mx = xSumm / double(count);
		my = ySumm / double(count);
		
		if (configData.verboseMode) { ROS_INFO("Average feature motion since previous frame: (%06.2f, %06.2f)", mx, my); }
		
	}
	
	
	
	return count;
	
}

void featureTrackerNode::prepareForTermination() {
	if (trackCountStream.is_open()) trackCountStream.close();	
	if (featureMotionStream.is_open()) featureMotionStream.close();
}

void featureTrackerNode::handle_very_new() {
	
	unsigned int current_idx = frameCount-1;
	
	if (configData.verboseMode) { ROS_WARN("Handling new frame (interruption backend)"); }
	
	
	
	// Feature detection
	
	homogPoints[1].clear();
	homographyDetector -> detect(grayImageBuffer[current_idx % 2], homogPoints[1]);
	
	if (homogPoints[1].size() < 4) {
		ROS_ERROR("Insufficient points detected (%d) in second frame for calculating homography..", homogPoints[1].size());
	} 
	
	if ((homogPoints[0].size() < 4) || (homogPoints[1].size() < 4)) {
		
		H12 = cv::Mat::eye(3, 3, CV_64FC1);
		undergoingDelay = false;
		return;
		
	}

	if (configData.verboseMode) { ROS_INFO("About to extract homography descriptors for second frame.."); }

	// Feature description

	homographyExtractor -> compute(grayImageBuffer[current_idx % 2], homogPoints[1], homogDescriptors[1]);
	
	// Feature matching
	
	cv::Mat matchingMatrix;
	
	if (configData.verboseMode) { ROS_INFO("Creating homography matching matrix.."); }
	
	createMatchingMatrix(matchingMatrix, homogDescriptors[0], homogDescriptors[1]);
			
	//constrainMatchingMatrix(matchingMatrix, homogPoints[0], homogPoints[1], MATCHING_DIST_CONSTRAINT, MATCHING_SIZE_CONSTRAINT);
	
	vector<vector<cv::DMatch> > matches;
	
	twoWayPriorityMatching(matchingMatrix, matches, configData.matchingMode);
	
	if (configData.verboseMode) { ROS_INFO("Sorting homography matches.."); }
	
	sortMatches(matches);
	
	// Reduce points to matched points

	vector<cv::Point2f> points1, points2;
	
	vector<int> queryIdxs( matches.size() );
	vector<int> trainIdxs( matches.size() );
	
	for( size_t i = 0; i < matches.size(); i++ ) {
		queryIdxs[i] = matches[i][0].queryIdx;
		trainIdxs[i] = matches[i][0].trainIdx;
	}
	
	cv::KeyPoint::convert(homogPoints[0], points1, queryIdxs);
	cv::KeyPoint::convert(homogPoints[1], points2, trainIdxs);

	vector<uchar> validityMask;
	vector<char> validityMask2;  
	
	// Homography determination
	
	if ((points1.size() < 4) || (points2.size() < 4)) {
		H12 = cv::Mat::eye(3, 3, CV_64FC1);
		undergoingDelay = false;
		return;
	}
	
	if (configData.verboseMode) { ROS_INFO("Attempting to find homography with (%d, %d) matched points..", points1.size(), points2.size()); }
	
	
	
	H12 = cv::findHomography( cv::Mat(points1), cv::Mat(points2), validityMask, CV_RANSAC, 5.0 );
		
	unsigned int validPoints = 0;
	
	for (unsigned int iii = 0; iii < validityMask.size(); iii++) {
		if (validityMask.at(iii) > 0) {
			validPoints++;
		}
	}
	
	//if (configData.verboseMode) { ROS_INFO("Homography found (%d / %d / %d)", points1.size(), points2.size(), validPoints); }
	cv::Mat im1, im2;
	
	
	warpPerspective(grayImageBuffer[(current_idx-1) % 2], im2, H12, grayImageBuffer[(current_idx-1) % 2].size());
	//grayImageBuffer[(current_idx-1) % MAXIMUM_FRAMES_TO_STORE].copyTo(im2);
	
	grayImageBuffer[current_idx % 2].copyTo(im1);
	
	// homogPoints[1] and therefore points2 are actually from the newer image...
	
	unsigned int inlierPoints = cv::countNonZero(validityMask);
	
	if (configData.verboseMode) { ROS_INFO("Number of matches for interruption homography = (%u/%d)", inlierPoints, homogPoints[0].size()); } // , matches.size(), points1.size() 
	
	if (configData.verboseMode) { cout << "H12 = " << H12 << endl; }
	
	#ifdef _BUILD_FOR_ROS_
	while (0) {
		
		cv::Mat im1_col, im1_col_B, im2_col;
		
		cvtColor(im1, im1_col, CV_GRAY2RGB);
		cvtColor(im2, im2_col, CV_GRAY2RGB);
		
		cv::Scalar color_x = CV_RGB(255, 0, 0);
		displayKeyPoints(im1_col, homogPoints[1], im1_col_B, color_x);

		//imshow("temp_disp", im1);	// Current image
		std::copy(&(im1_col_B.at<unsigned char>(0,0)), &(im1_col_B.at<unsigned char>(0,0))+(drawImage.cols*drawImage.rows*drawImage.channels()), msg_debug.data.begin());				
		debug_pub.publish(msg_debug, debug_camera_info);
		cv::waitKey(500);
		//imshow("temp_disp", im2);	// Previous image under transform
		std::copy(&(im2_col.at<unsigned char>(0,0)), &(im2_col.at<unsigned char>(0,0))+(drawImage.cols*drawImage.rows*drawImage.channels()), msg_debug.data.begin());				
		debug_pub.publish(msg_debug, debug_camera_info);
		cv::waitKey(500);
		
		
	}
	#endif
	
	
	// Then, do you want to move "globalFinishingPoints" so that they're at the locations estimated by H12?
	
	
	
	undergoingDelay = false;
}

void featureTrackerNode::handle_delay() {
		
	unsigned int current_idx = frameCount-1;
	
	//ROS_WARN("Handling a delay now...");
	
	
	// Feature detection
	
	
	
	//ROS_INFO("Current index: (%d)", current_idx);
	
	//ROS_INFO("grayIm: (%d, %d, %d, %d))", grayImageBuffer[current_idx % 2].rows, grayImageBuffer[current_idx % 2].cols, grayImageBuffer[current_idx % 2].depth(), grayImageBuffer[current_idx % 2].channels());
	
	homogPoints[0].clear();
	homographyDetector -> detect(grayImageBuffer[current_idx % 2], homogPoints[0]);
	//keypointDetector[0] -> detect(grayImageBuffer[current_idx % MAXIMUM_FRAMES_TO_STORE], homogPoints[0]);
	
	//ROS_INFO("Homography frame #1 (%d) pts detected", homogPoints[0].size());
	
	if (homogPoints[0].size() < 4) {
		ROS_ERROR("Insufficient points detected (%d) in first frame for calculating homography..", homogPoints[0].size());
		
	} else {

		sortKeyPoints(homogPoints[0], 200);
		
		//ROS_INFO("Sorting points...");
	
		if (configData.debugMode) {
			/*
			displayImage.copyTo(drawImage);
			displayKeyPoints(displayImage, homogPoints[0], drawImage, CV_RGB(255, 0, 0), 0);
			
			std::copy(&(drawImage.at<unsigned char>(0,0)), &(drawImage.at<unsigned char>(0,0))+(drawImage.cols*drawImage.rows*drawImage.channels()), msg_debug.data.begin());				
			debug_pub.publish(msg_debug, debug_camera_info);
			*/
		}
		
		homographyExtractor -> compute(grayImageBuffer[current_idx % 2], homogPoints[0], homogDescriptors[0]);
	}
	
	// Feature description
	undergoingDelay = true;
}

#ifdef _BUILD_FOR_ROS_
featureTrackerNode::featureTrackerNode(ros::NodeHandle& nh, trackerData startupData) {
#else
featureTrackerNode::featureTrackerNode(trackerData startupData) {
#endif
	debugInitialized = false;
	lastAllocatedTrackIndex = -1;
	cycleFlag = false;
	cycleCount = 0;
	freezeNextOutput = false;
	#ifdef _BUILD_FOR_ROS_
	lastCycleFrameTime = ros::Time(0.0);
	#else
	lastCycleFrameTime = boost::posix_time::ptime(boost::posix_time::min_date_time);
	bridgeReplacement = new cv::Mat();
	#endif
	
	peakTracks = 0;
	
	skippedFrameCount = 0;
	capturedFrameCount = 0;
	
	skipTime = timeElapsedMS(skip_timer);
	
	previousIndex = -1;
	currentIndex = -1;
	
	undergoingDelay = false;
	
	if (configData.outputFolder == "outputFolder") {
		
		char timeString[256];
	
		#ifdef _BUILD_FOR_ROS_
		sprintf(timeString, "%010d.%09d", ros::Time::now().sec, ros::Time::now().nsec);
		#else
		boost::posix_time::ptime pt = boost::posix_time::microsec_clock::local_time();
		sprintf(timeString, "%010d.%09d", pt.time_of_day().total_seconds(), pt.time_of_day().total_microseconds());
		#endif
		
		
		string defaultOutput = configData.read_addr + "nodes/flow/log/" + timeString;
		ROS_WARN("No output folder specified, outputting by default to (%s)", defaultOutput.c_str());
		configData.outputFolder = defaultOutput;
		
		char buffer[256];
		sprintf(buffer,"mkdir -p %s",configData.outputFolder.c_str());
		int mkdirResult = system(buffer);
		
		ROS_INFO("Result of mkdir command: (%d)", mkdirResult);
		
		char buffer_check[256];
		sprintf(buffer_check,"%s", &buffer[9]);
	
		if (configData.verboseMode) { ROS_INFO("Checking that directory (%s) has been created..", buffer_check); }
	
		while (!boost::filesystem::exists( buffer_check ) ) {
			
		}
		
	}
		
	referenceFrame = -1;
	
	frameCount = 0;
	readyFrame = 0;
	
	#ifdef _BUILD_FOR_ROS_
	sprintf(nodeName, "%s", ros::this_node::getName().c_str());
	#else
	sprintf(nodeName, "/%s", __PROGRAM__);
	#endif
	sprintf(debug_pub_name, "thermalvis%s/image_col", nodeName);
	
	infoProcessed = false;
	infoSent = false;
	
	configData = startupData;
	
	for (unsigned int ppp = 0; ppp < MAX_HISTORY_FRAMES; ppp++) {
		configData.multiplier[ppp] = 0;
	}
	
	configData.initializeDetectors(keypointDetector, &homographyDetector);
	configData.initializeDescriptors(&descriptorExtractor, &homographyExtractor);
	
	//printf("%s::%s << Startup data: (%s)\n", __PROGRAM__, __FUNCTION__, configData.video_stream.c_str());
	
	//std::string topic = nh.resolveName(configData.video_stream + configData.video_sequence);
	#ifdef _BUILD_FOR_ROS_
	std::string topic = nh.resolveName(configData.topic);
	string topic_info = configData.topic.substr(0, configData.topic.find_last_of("/") + 1) + "camera_info";
	
	//ROS_INFO("Connecting to camera_info. %s", topic_info.c_str());
	
	//image_transport::ImageTransport it(nh);
	it = new image_transport::ImageTransport(nh);
	
	ROS_INFO("Subscribing to camera topic (%s)", topic.c_str());
	camera_sub = it->subscribeCamera(topic, 1, &featureTrackerNode::handle_camera, this);
#endif

	if (configData.tracksOutputTopic == "tracksOutputTopic") {
		
		
		string tmpString = configData.topicParent + "/tracks";
		//string tmpString = configData.topic.substr(0, configData.topic.find_last_of("/") + 1);
		//ROS_WARN("(%s), (%s)", configData.topic.c_str(), tmpString.c_str());
		sprintf(tracks_pub_name, "%s", tmpString.c_str());
		
		
		
	} else {
		// sprintf(tracks_pub_name, "thermalvis%s/tracks", nodeName);
		sprintf(tracks_pub_name, "%s", configData.tracksOutputTopic.c_str());
	}
	
	#ifdef _BUILD_FOR_ROS_
	ROS_INFO("Publishing tracks data at (%s)", tracks_pub_name);
	
	ros::AdvertiseOptions op = ros::AdvertiseOptions::create<thermalvis::feature_tracks>(tracks_pub_name, 1, &connected, &disconnected, ros::VoidPtr(), NULL);
	op.has_header = false;
	
	//tracks_pub = nh.advertise<thermalvis::feature_tracks>(tracks_pub_name, 1);
	tracks_pub = nh.advertise(op);
	
	timer = nh.createTimer(ros::Duration(0.05), &featureTrackerNode::timed_loop, this);
	
	features_timer = nh.createTimer(ros::Duration(0.01), &featureTrackerNode::features_loop, this);
	#endif
	//printf("%s::%s << Image transport subscribed.\n", __PROGRAM__, __FUNCTION__);
	
	
	
	
	if (configData.outputTrackCount) {
		#ifdef _BUILD_FOR_ROS_
		string outputTrackCountFile = configData.outputFolder + "/" + ros::this_node::getName().substr(1,ros::this_node::getName().size()-1) + "_trackcount.txt";
		#else
		string outputTrackCountFile = configData.outputFolder + "/" + __PROGRAM__ + "_trackcount.txt";
		#endif
		ROS_INFO("outputTrackCountFile = (%s)", outputTrackCountFile.c_str());
		
		trackCountStream.open(outputTrackCountFile.c_str(), ios::out);
	}
	
	if (configData.outputFeatureMotion) {
		
		#ifdef _BUILD_FOR_ROS_
		string outputFeatureMotionFile = configData.outputFolder + "/" + ros::this_node::getName().substr(1,ros::this_node::getName().size()-1) + "_motion.txt";
		#else
		string outputFeatureMotionFile = configData.outputFolder + "/" + __PROGRAM__ + "_motion.txt";
		#endif
		
	
		ROS_INFO("outputFeatureMotionFile = (%s)", outputFeatureMotionFile.c_str());
		
		featureMotionStream.open(outputFeatureMotionFile.c_str(), ios::out);
	}
	
	//cin.get();
	
	#ifdef _BUILD_FOR_ROS_
	ROS_INFO("Establishing server callback...");
	f = boost::bind (&featureTrackerNode::serverCallback, this, _1, _2);
    server.setCallback (f);
	#endif
    
}

#ifdef _BUILD_FOR_ROS_
void featureTrackerNode::handle_camera(const sensor_msgs::ImageConstPtr& msg_ptr, const sensor_msgs::CameraInfoConstPtr& info_msg) {
#else
void featureTrackerNode::handle_camera(const cv::Mat& inputImage) {
#endif
	#ifdef _BUILD_FOR_ROS_
	while (!infoProcessed) { 
		process_info(info_msg);
	}
	#endif

	if (readyFrame < frameCount) {
		// Want to skip if you're still processing previous frame
		return;
	}
	
	#ifdef _BUILD_FOR_ROS_
	original_time = info_msg->header.stamp;
	currentIndex = info_msg->header.seq;
	#else
	original_time = boost::posix_time::microsec_clock::local_time();
	currentIndex++;
	#endif

	#ifdef _BUILD_FOR_ROS_
	if (((original_time.toSec() - lastCycleFrameTime.toSec()) > configData.newFeaturesPeriod) && (configData.newFeaturesPeriod != 0.0) ) {
	#else
	boost::posix_time::time_duration diff = original_time - lastCycleFrameTime;
	if (((diff.total_milliseconds()/1000.0) > configData.newFeaturesPeriod) && (configData.newFeaturesPeriod != 0.0) ) {
	#endif
		lastCycleFrameTime = original_time;
		cycleCount++;
		cycleFlag = true;
	}

	#ifdef _BUILD_FOR_ROS_
	if (lastCycleFrameTime.toSec() > original_time.toSec()) {
	#else
	if (lastCycleFrameTime > original_time) {
	#endif
		lastCycleFrameTime = original_time;
	}

	if (currentIndex < previousIndex) {
		#ifdef _BUILD_FOR_ROS_
		ROS_WARN("Current received image index is lower than previous, assuming watching a looped video. (%d) vs (%d) : (%d)", previousIndex, currentIndex, frameCount);
		#else
		printf("%s << WARNING: Current received image index is lower than previous, assuming watching a looped video. (%d) vs (%d) : (%d)\n", __FUNCTION__, previousIndex, currentIndex, frameCount);
		#endif
		featuresVelocity = 9e99;
		capturedFrameCount++;
	} else if (currentIndex > (previousIndex+1)) {
		
		if (!undergoingDelay) { 
			skippedFrameCount++;
		}
		featuresVelocity = 9e99;
	} else {
		capturedFrameCount++;
	}
	
	#ifdef _BUILD_FOR_ROS_
	if ((frameCount > 0) && (!undergoingDelay)) {
		if (info_msg->binning_y == 1) {
			ROS_WARN("Current frame is a duplicate, going into NUC-handling routine...");
			handle_delay();
			featuresVelocity = 9e99;
			return;
		}	
	} 
	
	if (info_msg->binning_y == 1) {
		return;
	}
	#else
	// TODO: Not implemented
	#endif

	#ifdef _BUILD_FOR_ROS_
	debug_camera_info = (*info_msg);
	//ROS_ERROR("debug camera seq = (%d) vs (%d)", debug_camera_info.header.seq, info_msg->header.seq);
	cv_ptr = cv_bridge::toCvCopy(msg_ptr, enc::BGR8);					// For some reason it reads as BGR, not gray
	msg_debug.header.stamp = info_msg->header.stamp;
	msg_debug.header.seq = info_msg->header.seq;
	#else
	bridgeReplacement = &inputImage;
	#endif
	act_on_image();
	
}


void trackerData::initializeDetectors(cv::Ptr<cv::FeatureDetector> *det, cv::Ptr<cv::FeatureDetector> *hom) {
	
	for (unsigned int iii = 0; iii < numDetectors; iii++) {
		if (detector[iii] == "SURF") {
			ROS_ERROR("SURF has been deactivated due to copyright protection!");
			//det[iii] = new SurfFeatureDetector( sensitivity[iii] );
		} else if (detector[iii] == "FAST") {
			det[iii] = new cv::FastFeatureDetector( int(sensitivity[iii] * FAST_DETECTOR_SENSITIVITY_SCALING) );
		} else if (detector[iii] == "GFTT") {
			det[iii] = new cv::GoodFeaturesToTrackDetector( maxFeatures, max(MINIMUM_GFTT_SENSITIVITY, sensitivity[iii]), 1.0, 3, false );
		} else if (detector[iii] == "STAR") {
			det[iii] = new cv::StarFeatureDetector( 16, int(sensitivity[iii]) );
		} else if (detector[iii] == "ORB") {
			det[iii] = new cv::OrbFeatureDetector( maxFeatures );
		} else if (detector[iii] == "HARRIS") {
			det[iii] = new cv::GoodFeaturesToTrackDetector( maxFeatures, max(MINIMUM_HARRIS_SENSITIVITY, sensitivity[iii]), 1.0, 3, true );
		} else {
			printf("%s << ERROR! Shouldn't have got here!\n", __FUNCTION__);
		}
	}
	
	//hom[0] = new cv::OrbFeatureDetector( maxFeatures );
	hom[0] = new cv::FastFeatureDetector( int(FAST_DETECTOR_SENSITIVITY_SCALING*0.02) );
	//hom[0] = new SurfFeatureDetector( 1.00 );
	
}

#ifdef _BUILD_FOR_ROS_
void featureTrackerNode::timed_loop(const ros::TimerEvent& event) {
#else
void featureTrackerNode::timed_loop() {
#endif
	elapsedTime = timeElapsedMS(cycle_timer, false);
	
	if ((frameCount > 0) && (elapsedTime > configData.delayTimeout*MS_PER_SEC) && !undergoingDelay && handleDelays) {
		featuresVelocity = 9e99;
		handle_delay();
	}
	
	
	
	skipTime = timeElapsedMS(skip_timer, false);
	
	//ROS_INFO("skipTime = (%f) vs (%f)", skipTime, SKIP_WARNING_TIME_PERIOD);
	
	if (skipTime > 1000.0*SKIP_WARNING_TIME_PERIOD) {
		skipTime = timeElapsedMS(skip_timer);
		
		if (skippedFrameCount > 0) {
			ROS_WARN("(%d) skipped frames and (%d) captured frames in last (%f) seconds..", skippedFrameCount, capturedFrameCount, SKIP_WARNING_TIME_PERIOD); // , current frame at risk of being mid-NUC if duplicates are not marked. (%d) vs (%d) : (%d)", previousIndex, currentIndex, frameCount);
		}
		
		skipTime = 0.0;
		skippedFrameCount = 0;
		capturedFrameCount = 0;
	} 
	
	
}




