/*! \file	calibrator.cpp
 *  \brief	Definitions for the CALIBRATOR node.
*/

#include "calibrator/calib.hpp"

calibratorData::calibratorData()
{
		// ...
}

#ifndef _BUILD_FOR_ROS_
bool calibratorConfig::assignStartingData(calibratorData& startupData) {

	verboseMode = startupData.verboseMode;
	debugMode = startupData.debugMode;
	
	return true;
}
#endif

#ifdef _USE_BOOST_ 
#ifndef _BUILD_FOR_ROS_
bool calibratorData::assignFromXml(xmlParameters& xP) {

	int countOfNodes = 0;

	BOOST_FOREACH(boost::property_tree::ptree::value_type &v, xP.pt.get_child("launch")) {
		if (v.first.compare("node")) continue; // only progresses if its a "node" tag
		if (!v.second.get_child("<xmlattr>.type").data().compare("calibrator")) countOfNodes++;
	}

	if (countOfNodes == 0) {
		ROS_ERROR("No relevant nodes found in XML config!");
		return false;
	}

	if (countOfNodes > 1) {
		ROS_ERROR("More than 1 relevant node found in XML config! This functionality is not supported in Windows..");
		return false;
	}

	BOOST_FOREACH(boost::property_tree::ptree::value_type &v, xP.pt.get_child("launch")) { // Within tree (pt), finds launch, and loops all tags within it
		if (v.first.compare("node")) continue;
		if (v.second.get_child("<xmlattr>.type").data().compare("calibrator")) {
			if ((!v.second.get_child("<xmlattr>.type").data().compare("reconfigure_gui")) || (!v.second.get_child("<xmlattr>.type").data().compare("rqt_reconfigure"))) {
				if (!v.second.get_child("<xmlattr>.args").data().compare("calibrator")) displayGUI = true;
				if (!v.second.get_child("<xmlattr>.args").data().compare("/calibrator")) displayGUI = true;
			}
			continue;
		}

		BOOST_FOREACH(boost::property_tree::ptree::value_type &v2, v.second) { // Traverses the subtree...
			if (v2.first.compare("param")) continue;

			if (!v2.second.get_child("<xmlattr>.name").data().compare("debugMode")) debugMode = !v2.second.get_child("<xmlattr>.value").data().compare("true");
			if (!v2.second.get_child("<xmlattr>.name").data().compare("verboseMode")) verboseMode = !v2.second.get_child("<xmlattr>.value").data().compare("true");
			if (!v2.second.get_child("<xmlattr>.name").data().compare("outputFolder")) outputFolder = v2.second.get_child("<xmlattr>.value").data();

			if (!v2.second.get_child("<xmlattr>.name").data().compare("autoAlpha")) autoAlpha = !v2.second.get_child("<xmlattr>.value").data().compare("true");
			if (!v2.second.get_child("<xmlattr>.name").data().compare("undistortImages")) undistortImages = !v2.second.get_child("<xmlattr>.value").data().compare("true");

			if (!v2.second.get_child("<xmlattr>.name").data().compare("video_stream")) video_stream = v2.second.get_child("<xmlattr>.value").data();
			
			if (!v2.second.get_child("<xmlattr>.name").data().compare("calibType")) calibType = v2.second.get_child("<xmlattr>.value").data();
			if (!v2.second.get_child("<xmlattr>.name").data().compare("patternType")) patternType = v2.second.get_child("<xmlattr>.value").data();
			if (!v2.second.get_child("<xmlattr>.name").data().compare("optMethod")) optMethod = v2.second.get_child("<xmlattr>.value").data();

			

			if (!v2.second.get_child("<xmlattr>.name").data().compare("maxPatterns")) maxPatterns = atoi(v2.second.get_child("<xmlattr>.value").data().c_str());
			if (!v2.second.get_child("<xmlattr>.name").data().compare("maxFrames")) maxFrames = atoi(v2.second.get_child("<xmlattr>.value").data().c_str());
			if (!v2.second.get_child("<xmlattr>.name").data().compare("setSize")) setSize = atoi(v2.second.get_child("<xmlattr>.value").data().c_str());
			if (!v2.second.get_child("<xmlattr>.name").data().compare("xCount")) xCount = atoi(v2.second.get_child("<xmlattr>.value").data().c_str());
			if (!v2.second.get_child("<xmlattr>.name").data().compare("yCount")) yCount = atoi(v2.second.get_child("<xmlattr>.value").data().c_str());

			if (!v2.second.get_child("<xmlattr>.name").data().compare("gridSize")) gridSize = atof(v2.second.get_child("<xmlattr>.value").data().c_str());
			if (!v2.second.get_child("<xmlattr>.name").data().compare("alpha")) alpha = atof(v2.second.get_child("<xmlattr>.value").data().c_str());
			
    }

		// Substitute tildes if in Windows
#ifdef _WIN32
      CleanAndSubstitutePath( outputFolder );
#endif

	}

	processStartingData();
	return true;

}
#endif
#endif

void calibratorNode::prepareExtrinsicPatternSubsets() {

        ROS_WARN("prepareExtrinsicPatternSubsets...");
        
        srand((unsigned)time(0));
        
        ROS_INFO("extrinsicsPointSets[0].size() = (%d)", int(extrinsicsPointSets[0].size()));

	vector<vector<Point2f> > tmpSet[MAX_ALLOWABLE_CAMS];
	
	for (unsigned int xxx = 0; xxx < configData.numCams; xxx++) {
		
		for (unsigned int iii = 0; iii < extrinsicsPointSets[xxx].size(); iii++) {
			tmpSet[xxx].push_back(extrinsicsPointSets[xxx].at(iii));
		}
		
	}
	
	// While the candidate quantity is insufficient
	while ( (((int)extrinsicCandidateSets[0].size()) < configData.maxCandidates) && (configData.maxCandidates > 0) && (tmpSet[0].size() > 0) ) {
		
		unsigned int randomIndex = rand() % tmpSet[0].size();
		
		for (unsigned int xxx = 0; xxx < configData.numCams; xxx++) {
			extrinsicCandidateSets[xxx].push_back(tmpSet[xxx].at(randomIndex));
			tmpSet[xxx].erase(tmpSet[xxx].begin() + randomIndex);
		}
		
	}
	
	ROS_INFO("extrinsicCandidateSets[0].size() = (%d)", int(extrinsicCandidateSets[0].size()));
	
	for (unsigned int xxx = 0; xxx < configData.numCams; xxx++) tmpSet[xxx].clear();
	
	for (unsigned int xxx = 0; xxx < configData.numCams; xxx++) {
		for (unsigned int iii = 0; iii < extrinsicsPointSets[xxx].size(); iii++) tmpSet[xxx].push_back(extrinsicsPointSets[xxx].at(iii));
	}
	
	// While the testing quantity is insufficient
	while ( (((int)extrinsicTestingSets[0].size()) < configData.maxTests) && (configData.maxTests > 0) && (tmpSet[0].size() > 0) ) {
		
		unsigned int randomIndex = rand() % tmpSet[0].size();
		
		for (unsigned int xxx = 0; xxx < configData.numCams; xxx++) {
			extrinsicTestingSets[xxx].push_back(tmpSet[xxx].at(randomIndex));
			tmpSet[xxx].erase(tmpSet[xxx].begin() + randomIndex);
		}
		
	}
	
	ROS_INFO("extrinsicTestingSets[0].size() = (%d)", int(extrinsicTestingSets[0].size()));
	
}

void calibratorNode::create_virtual_pairs() {
	
	ROS_INFO("Extracted patterns for camera (1) : (%d/%d) & (2) : (%d/%d)", int(pointSets[0].size()), int(frameCount[0]), int(pointSets[1].size()), int(frameCount[1]));

	
	// For each pattern from first camera
	for (int aaa = 0; aaa < int(configData.numCams); aaa++) {
		for (int bbb = 0; bbb < int(configData.numCams); bbb++) {
			
			if (aaa == bbb) continue;
			
			if (configData.verboseMode) { ROS_INFO("Finding virtual frames for camera relationship (%d) -> (%d).", aaa, bbb); }
			
			for (int iii = 0; iii < patternTimestamps[aaa].size(); iii++) {
		
				int matchingIndex = 0;
				
				// Find two closest patterns for second camera
				while (patternTimestamps[aaa].at(iii).toSec() > patternTimestamps[bbb].at(matchingIndex).toSec()) {
					//ROS_INFO("matchingIndex = (%d); time2 = (%f)", matchingIndex, patternTimestamps[bbb].at(matchingIndex).toSec());
					matchingIndex++;
					if (matchingIndex >= patternTimestamps[bbb].size()) break;
				}
				
				if (matchingIndex >= patternTimestamps[bbb].size()) {
					ROS_INFO("No matches found, breaking...");
					break;
				}
				
				//ROS_INFO("Debug B");
				
				if (matchingIndex == 0) {
					ROS_INFO("No matches found, continuing...");
					continue;
				}
				
				if (configData.verboseMode) { ROS_INFO("Best index matches for (%d/%d) = (%d-%d/%d)", iii, int(patternTimestamps[aaa].size()), matchingIndex-1, matchingIndex, int(patternTimestamps[bbb].size())); }
				if (configData.verboseMode) { ROS_INFO("Best index matches for (%d)[%f] = (%d)[%f] and (%d)[%f]", iii, patternTimestamps[aaa].at(iii).toSec(), matchingIndex-1, patternTimestamps[bbb].at(matchingIndex-1).toSec(), matchingIndex, patternTimestamps[bbb].at(matchingIndex).toSec()); }
				
				double interp_distance_1 = abs(patternTimestamps[aaa].at(iii).toSec() - patternTimestamps[bbb].at(matchingIndex-1).toSec());
				double interp_distance_2 = abs(patternTimestamps[aaa].at(iii).toSec() - patternTimestamps[bbb].at(matchingIndex).toSec());
				double interp_distance = max(interp_distance_1, interp_distance_2);
				
				if (configData.verboseMode) { ROS_INFO("Interpolation distance = (%f) {max(%f, %f)}", interp_distance, interp_distance_1, interp_distance_2); }
				
				//ROS_INFO("Debug C");
				
				if (interp_distance < configData.maxInterpolationTimegap) {
					
					// Interpolate feature locations, and add to pairs
					
					vector<Point2f> virtualPointset;
					
					double biasFraction = (patternTimestamps[aaa].at(iii).toSec() - patternTimestamps[bbb].at(matchingIndex-1).toSec()) / (patternTimestamps[bbb].at(matchingIndex).toSec() - patternTimestamps[bbb].at(matchingIndex-1).toSec());
					
					double motion = generateVirtualPointset(pointSets[bbb].at(matchingIndex-1), pointSets[bbb].at(matchingIndex), virtualPointset, biasFraction);
					
					if (motion < configData.maxInterpolationMotion) {
						extrinsicsPointSets[aaa].push_back(pointSets[aaa].at(iii));
						extrinsicsPointSets[bbb].push_back(virtualPointset);
					}	
				}
				
			}
		
		}	
		
	}
	
	if (configData.verboseMode) { ROS_INFO("Found (%d // %d) virtual pairs", int(extrinsicsPointSets[0].size()), int(extrinsicsPointSets[1].size())); }
	 
	if (configData.verboseMode) { ROS_INFO("Completed generation of virtual frame pairs."); }

}

void calibratorNode::preparePatternSubsets() {

	ROS_WARN("Preparing pattern subsets...");
	
	srand((unsigned)time(0));
	
	

	for (unsigned int xxx = 0; xxx < configData.numCams; xxx++) {
		vector<vector<Point2f> > tmpSet;
	
		if (configData.verboseMode) { ROS_INFO("Looping for cam (%d)", xxx); }
		
		for (unsigned int iii = 0; iii < pointSets[xxx].size(); iii++) {
			tmpSet.push_back(pointSets[xxx].at(iii));
			if (pointSets[xxx].at(iii).size() == 0) {
				ROS_ERROR("FOR SOME REASON A POINT SET HAS NO POINTS IN IT!!");
				imshow("tmp", prevMat[xxx]);
				waitKey(0);
			}
		}
		
		if (configData.verboseMode) { ROS_INFO("Loop (%d) completed", xxx); }
		
		int actualMaxCandidates;
		actualMaxCandidates = min(configData.maxCandidates, int(pointSets[xxx].size()));
		
		// While the candidate quantity is insufficient
		while ( (((double) candidateSets[xxx].size()) < actualMaxCandidates) && (configData.maxCandidates > 0) && (tmpSet.size() > 0)  ) {
			
			//if (configData.verboseMode) { ROS_INFO("Adding new set (%d existing)...", xxx); }
			
			unsigned int randomIndex = rand() % tmpSet.size();
			
			candidateSets[xxx].push_back(tmpSet.at(randomIndex));
			
			tmpSet.erase(tmpSet.begin() + randomIndex);
			
		}
		
		tmpSet.clear();
		
		for (unsigned int iii = 0; iii < pointSets[xxx].size(); iii++) {
			tmpSet.push_back(pointSets[xxx].at(iii));
		}
		
		int actualMaxTests;
		actualMaxTests = min(configData.maxTests, int(pointSets[xxx].size()));
		
		// While the testing quantity is insufficient
		while ( (((double) testingSets[xxx].size()) < actualMaxTests) && (configData.maxTests > 0) && (tmpSet.size() > 0) ) {
			
			unsigned int randomIndex = rand() % tmpSet.size();
			
			testingSets[xxx].push_back(tmpSet.at(randomIndex));
			
			tmpSet.erase(tmpSet.begin() + randomIndex);
			
		}
		
	}

}

void calibratorNode::prepareForTermination() {
	return;	
}

void calibratorNode::assignIntrinsics() {
	
	configData.K[0].copyTo(cameraMatrices[0]);
	configData.K[1].copyTo(cameraMatrices[1]);
	configData.distCoeffs[0].copyTo(distCoeffVecs[0]);
	configData.distCoeffs[1].copyTo(distCoeffVecs[1]);

}

bool calibratorNode::wantsIntrinsics() {
	if (configData.calibType == "single") {
		return true;
	} else {
		if (configData.wantsIntrinsics) {
			return true;
		} else {
			return false;
		}
	}
}

bool calibratorNode::wantsExtrinsics() {
	if (configData.calibType == "stereo") {
		return true;
	} else {
		return false;
	}
}

void calibratorNode::writeResults() {

	
	if (wantsIntrinsics()) {
		for (unsigned int xxx = 0; xxx < configData.numCams; xxx++) {
			
			updateIntrinsicMap(xxx);
			
			stringstream ss;
			ss << (xxx+1);
			string outputName = configData.outputFolder + "/" + "intrinsics-" + ss.str() + ".yml";
			
			if (configData.verboseMode) { ROS_INFO("outputName = (%s)", outputName.c_str()); }
			
			FileStorage fs(outputName, FileStorage::WRITE);
			
			Mat imageSize(1, 2, CV_16UC1);
			imageSize.at<unsigned short>(0,0) = imSize[xxx].width;
			imageSize.at<unsigned short>(0,1) = imSize[xxx].height;
			fs << "imageSize" << imageSize;
			fs << "cameraMatrix" << cameraMatrices[xxx];
			fs << "distCoeffs" << distCoeffVecs[xxx];
			fs << "newCamMat" << newCamMat[xxx];
			fs << "alpha" << configData.alpha;

			fs << "reprojectionError" << reprojectionError_intrinsics[xxx];
			fs << "generalisedError" << extendedReprojectionError_intrinsics[xxx];

			fs << "patternsUsed" << ((int) subselectedTags_intrinsics[xxx].size());

			fs.release();

                         ROS_WARN("Wrote intrinsics calibration data to:\n %s", outputName.c_str());
			
                }

	}
	
	if (wantsExtrinsics()) {
		
		string outputName = configData.outputFolder + "/" + "extrinsics" + ".yml";
		
		if (configData.verboseMode) { ROS_INFO("outputName = (%s)", outputName.c_str()); }
		
		FileStorage fs(outputName, FileStorage::WRITE);
		
		//ROS_INFO("FileStorage configured.");
		
		string tmpString;
		char tmp[64];
		
		sprintf(tmp, "reprojectionErr");
        tmpString = string(tmp);
        fs << tmpString << stereoError;
        
        //ROS_INFO("DEBUG A.");
        
		sprintf(tmp, "generalisedMRE");
        tmpString = string(tmp);
        fs << tmpString << extendedExtrinsicReprojectionError;

		//ROS_INFO("DEBUG B.");
		
		for (unsigned int i = 0; i < configData.numCams; i++) {

            sprintf(tmp, "R%d", i);
            //printf("%s << tmp = %s.\n", __FUNCTION__, tmp);
            tmpString = string(tmp);
            fs << tmpString << R[i];

            sprintf(tmp, "Rvec%d", i);
            tmpString = string(tmp);
            fs << tmpString << Rv[i];

            sprintf(tmp, "T%d", i);
            //printf("%s << tmp = %s.\n", __FUNCTION__, tmp);
            tmpString = string(tmp);
            fs << tmpString << T[i];

            sprintf(tmp, "R_%d", i);
            //printf("%s << tmp = %s.\n", __FUNCTION__, tmp);
            tmpString = string(tmp);
            fs << tmpString << R_[i];

            sprintf(tmp, "P_%d", i);
            //printf("%s << tmp = %s.\n", __FUNCTION__, tmp);
            tmpString = string(tmp);
            fs << tmpString << P_[i];

            sprintf(tmp, "cameraMatrix%d", i);
            //printf("%s << tmp = %s.\n", __FUNCTION__, tmp);
            tmpString = string(tmp);
            fs << tmpString << cameraMatrices[i];

            sprintf(tmp, "distCoeffs%d", i);
            //printf("%s << tmp = %s.\n", __FUNCTION__, tmp);
            tmpString = string(tmp);
            fs << tmpString << distCoeffVecs[i];
        }
        
        fs << "E" << E[1];
        fs << "F" << F[1];

        //HGH
         fs << "R" << R[1];
         fs << "T" << T[1];
         fs << "alpha" << configData.alpha;

         fs << "Q0" << Q[0];
         fs << "Q1" << Q[1];


		
        fs.release();

          ROS_WARN("Wrote extrinsic calibration data to:\n %s", outputName.c_str());
          
          if (!configData.fixIntrinsics) {
			  
			  for (unsigned int xxx = 0; xxx < configData.numCams; xxx++) {
			  
					stringstream ss;
					ss << (xxx+1);
					string outputName = configData.outputFolder + "/" + "intrinsics-modified-" + ss.str() + ".yml";

					if (configData.verboseMode) { ROS_INFO("outputName = (%s)", outputName.c_str()); }

					FileStorage fs(outputName, FileStorage::WRITE);

					Mat imageSize(1, 2, CV_16UC1);
					imageSize.at<unsigned short>(0,0) = imSize[xxx].width;
					imageSize.at<unsigned short>(0,1) = imSize[xxx].height;
					fs << "imageSize" << imageSize;
					fs << "cameraMatrix" << cameraMatrices[xxx];
					fs << "distCoeffs" << distCoeffVecs[xxx];
					
					Rect* validPixROI = 0;
					bool centerPrincipalPoint = true;
	
					newCamMat[xxx] = getOptimalNewCameraMatrix(cameraMatrices[xxx], distCoeffVecs[xxx], imSize[xxx], configData.alpha, imSize[xxx], validPixROI, centerPrincipalPoint);
					
					fs << "newCamMat" << newCamMat[xxx];
					//fs << "alpha" << configData.alpha;

					//fs << "reprojectionError" << reprojectionError_intrinsics[xxx];
					//fs << "generalisedError" << extendedReprojectionError_intrinsics[xxx];

					//fs << "patternsUsed" << ((int) subselectedTags_intrinsics[xxx].size());

					fs.release();

					 ROS_WARN("Wrote modified intrinsics calibration data to:\n %s", outputName.c_str());
				
					}
			}
	  }
			
	
	
	
}

bool calibratorNode::wantsToUndistort() {
	return configData.undistortImages;
}

bool calibratorNode::wantsToRectify() {
	return configData.rectifyImages;
}

void calibratorNode::getAverageTime() {
	
	unsigned int divisorFrames = 0;
	
	for (unsigned int iii = 0; iii < configData.numCams; iii++) {
		divisorFrames = max(divisorFrames, frameCount[iii]);
	}

	avgTime /= ((double) divisorFrames - 1);
	ROS_INFO("?Average time per frame: (%f)", avgTime);
	
}

void calibratorNode::getFrameTrackingTime() {
	
	unsigned int divisorFrames = 0;
	
	for (unsigned int iii = 0; iii < configData.numCams; iii++) {
		divisorFrames = max(divisorFrames, frameCount[iii]);
	}
	
	frameTrackingTime /= ((double) divisorFrames - 1);
	ROS_INFO("Average tracking time per frame: (%f)", frameTrackingTime);
	
}



void calibratorNode::startUndistortionPublishing() {
	
	
	//Rect* validPixROI = 0;
	
	//bool centerPrincipalPoint = true;
	
	for (unsigned int xxx = 0; xxx < configData.numCams; xxx++) {
		updateIntrinsicMap(xxx);
		//newCamMat[xxx] = getOptimalNewCameraMatrix(cameraMatrices[xxx], distCoeffVecs[xxx], imSize[xxx], configData.alpha, imSize[xxx], validPixROI, centerPrincipalPoint);
		//initUndistortRectifyMap(cameraMatrices[xxx], distCoeffVecs[xxx], default_R, newCamMat[xxx], imSize[xxx], CV_32FC1, map1[xxx], map2[xxx]);
	}
	
	
#ifdef _BUILD_FOR_ROS_
	// Set up timer
	// timer = ref->createTimer(ros::Duration(avgTime / 1000.0), &calibratorNode::publishUndistorted, this);
#endif

}

void calibratorNode::startRectificationPublishing() {

	
        
	//bool centerPrincipalPoint = true;
	
	Point pt1, pt2;
	vector<Point2f> rectangleBounds, newRecBounds;

	

	Mat blankCoeffs(1, 8, CV_64F);
	
	/*
	for (unsigned int xxx = 0; xxx < configData.numCams; xxx++) {
		newCamMat[xxx] = getOptimalNewCameraMatrix(cameraMatrices[xxx], distCoeffVecs[xxx], imSize[xxx], alpha, imSize[xxx], validPixROI, centerPrincipalPoint);
		initUndistortRectifyMap(cameraMatrices[xxx], distCoeffVecs[xxx], R[xxx], newCamMat[xxx], imSize[xxx], CV_32FC1, map1[xxx], map2[xxx]);
	}
	*/
	
	for (int i = 0; i < 2; i++) {

		ROS_INFO("Initializing rectification map (%d)...", i);

                initUndistortRectifyMap(cameraMatrices[i],
                                                                distCoeffVecs[i],
                                                                R_[i],
                                                                P_[i],  // newCamMat[i]
                                                                imSize[i],
                                                                CV_32F,     // CV_16SC2
                                                                map1[i],    // map1[i]
                                                                map2[i]);   // map2[i]


		ROS_INFO("Map (%d) initialized.", i);

		pt1 = Point(roi[i].x, roi[i].y);
		pt2 = Point(roi[i].x + roi[i].width, roi[i].y + roi[i].height);

		if ((pt1.x == 0) && (pt1.y == 0) && (pt2.x == 0) && (pt2.y == 0)) {
			ROS_WARN("ROI is defined as having zero area...");
		} else {
			printf("%s << (und) pt1 = (%d, %d); pt2 = (%d, %d)\n", __FUNCTION__, pt1.x, pt1.y, pt2.x, pt2.y);

			rectangleBounds.push_back(Point2f(float(pt1.x), float(pt1.y)));
			rectangleBounds.push_back(Point2f(float(pt2.x), float(pt2.y)));
			
			Rect validROI_x;
                        //HGH
                        //rectCamMat[i] = getOptimalNewCameraMatrix(cameraMatrices[i], distCoeffVecs[i], imSize[i], 0.5, imSize[i], &validROI_x);
                        rectCamMat[i] = getOptimalNewCameraMatrix(cameraMatrices[i], distCoeffVecs[i], imSize[i], configData.alpha, imSize[i], &validROI_x);




                        /*
			undistortPoints(Mat(rectangleBounds), newRecBounds, rectCamMat[i], blankCoeffs, R_[i], P_[i]);

			//printf("%s << Original rectangle points: = (%d, %d) & (%d, %d)\n", __FUNCTION__, pt1.x, pt1.y, pt2.x, pt2.y);

			pt1 = Point(int(newRecBounds.at(0).x), int(newRecBounds.at(0).y));
			pt2 = Point(int(newRecBounds.at(1).x), int(newRecBounds.at(1).y));

			printf("%s << pt1 = (%d, %d); pt2 = (%d, %d)\n", __FUNCTION__, pt1.x, pt1.y, pt2.x, pt2.y);

			if (pt1.y > topValidHeight)
			{
				topValidHeight = pt1.y;
			}

			if (pt2.y < botValidHeight)
			{
				botValidHeight = pt2.y;
			}

			leftValid[i] = pt1.x;
			rightValid[i] = pt2.x;
			*/
		}

		
	}

	printf("%s << topValidHeight = %d; botValidHeight = %d\n", __FUNCTION__, topValidHeight, botValidHeight);

	

	// Prepare epipolar lines etc:
	for (int k = 1; k < 32; k++)
	{
		// should try to center it on the final (thermal) image
		leftLinePoints.push_back(Point2f(0.f, float(k*(botValidHeight - topValidHeight)/32 - 1)));
		rightLinePoints.push_back(Point2f(float(imSize[0].width), float(k*(botValidHeight - topValidHeight)/32 - 1)));
	}
	
	
	
#ifdef _BUILD_FOR_ROS_
	// Set up timer
	// timer = ref->createTimer(ros::Duration(avgTime / 1000.0), &calibratorNode::publishRectified, this);
#endif

}

//#ifdef _BUILD_FOR_ROS_
//void calibratorNode::publishUndistorted(const ros::TimerEvent& event) {
//#else
void calibratorNode::publishUndistorted() {
//#endif
	// if (configData.verboseMode) { ROS_INFO("Publishing undistorted images..."); }
	
	Mat dispMat;
	
	if (configData.calibType == "single") {
		if (undistortionCount >= frameCount[0]) {
			configData.undistortImages = false;
			return;
		}
		
		// Draw grid
		
		Mat colorMat, preMat, postMat;
		
		cv::cvtColor(displayImages[0].at(undistortionCount), colorMat, CV_GRAY2RGB);
		
		
		if (configData.drawGrids) {
			drawGrid(colorMat, preMat, 0);
		} else {
			colorMat.copyTo(preMat);
		}
		
		
		remap(preMat, postMat, map1[0], map2[0], INTER_LINEAR, BORDER_TRANSPARENT);
		
		
		if (configData.drawGrids) {	
			drawGrid(postMat, dispMat, 1); 
		} else {
			postMat.copyTo(dispMat);
		}
		
#ifdef _BUILD_FOR_ROS_
		std::copy(&(dispMat.at<Vec3b>(0,0)[0]), &(dispMat.at<Vec3b>(0,0)[0])+(dispMat.cols*dispMat.rows*dispMat.channels()), msg_debug[0].data.begin());
		debug_pub[0].publish(msg_debug[0], debug_camera_info[0]);
#else
		char windowName[256];
		sprintf(windowName, "undistorted");
		imshow(windowName, dispMat);
		waitKey((frameCount[0] > 50) ? 40 : 200);
#endif

		// if (configData.verboseMode) { ROS_INFO("Image published..."); }
		
	} else {
		if (undistortionCount >= validPairs[0].size()) {
			configData.undistortImages = false;
			return;
		}
		
		Mat preMat, postMat;
		
		//ROS_WARN("About to publish undistorted frames...");
		drawGrid(displayImages[0].at(validPairs[0].at(undistortionCount)), preMat, 0);
		remap(preMat, postMat, map1[0], map2[0], INTER_LINEAR, BORDER_TRANSPARENT);
		drawGrid(postMat, dispMat, 1);

#ifdef _BUILD_FOR_ROS_
		std::copy(&(dispMat.at<Vec3b>(0,0)[0]), &(dispMat.at<Vec3b>(0,0)[0])+(dispMat.cols*dispMat.rows*dispMat.channels()), msg_debug[0].data.begin());
		debug_pub[0].publish(msg_debug[0], debug_camera_info[0]);
		//ROS_WARN("Message 1 published...");
#else
		char windowName[256];
		sprintf(windowName, "undistorted[0]");
		imshow(windowName, dispMat);
		waitKey((validPairs[0].size() > 50) ? 40 : 200);
#endif

		if (configData.numCams > 1) {
			drawGrid(displayImages[1].at(validPairs[1].at(undistortionCount)), preMat, 0);
			remap(preMat, postMat, map1[1], map2[1], INTER_LINEAR, BORDER_TRANSPARENT);
			drawGrid(postMat, dispMat, 1);
#ifdef _BUILD_FOR_ROS_
			std::copy(&(dispMat.at<Vec3b>(0,0)[0]), &(dispMat.at<Vec3b>(0,0)[0])+(dispMat.cols*dispMat.rows*dispMat.channels()), msg_debug[1].data.begin());
			debug_pub[1].publish(msg_debug[1], debug_camera_info[1]);		
#else
			char windowName2[256];
			sprintf(windowName2, "undistorted[1]");
			imshow(windowName2, dispMat);
			waitKey((validPairs[0].size() > 50) ? 40 : 200);
#endif
		}
	}
	
	undistortionCount++;
}

//#ifdef _BUILD_FOR_ROS_
//void calibratorNode::publishRectified(const ros::TimerEvent& event) {
//#else
void calibratorNode::publishRectified() {
//#endif
	//ROS_INFO("Entered timed (rectification) loop...");
	
	Mat dispMat;
	
	
	if (rectificationCount >= validPairs[0].size()) {
		configData.rectifyImages = false;
		return;
	}
	
	Mat und1, und2, disp1, disp2;
	
	//ROS_WARN("Publishing rectified images...");
        remap(displayImages[0].at(validPairs[0].at(rectificationCount)), und1, map1[0], map2[0], INTER_LINEAR);
        //HGH
        //disp1 = displayImages[0].at(validPairs[0].at(rectificationCount));
        //und1 = displayImages[0].at(validPairs[0].at(rectificationCount));
	drawGrid(und1, disp1, 1);

        //HGH - draw roi
        rectangle(disp1, Point2f(float(roi[0].x), float(roi[0].y)), Point2f(float(roi[0].x + roi[0].width), float(roi[0].y + roi[0].height)), CV_RGB(127, 255, 0));

#ifdef _BUILD_FOR_ROS_
	std::copy(&(disp1.at<Vec3b>(0,0)[0]), &(disp1.at<Vec3b>(0,0)[0])+(disp1.cols*disp1.rows*disp1.channels()), msg_debug[0].data.begin());
	debug_pub[0].publish(msg_debug[0], debug_camera_info[0]);
#else
	char windowName[256];
	sprintf(windowName, "rectified[0]");
	imshow(windowName, disp1);
	waitKey((validPairs[0].size() > 50) ? 40 : 200);
#endif

	remap(displayImages[1].at(validPairs[1].at(rectificationCount)), und2, map1[1], map2[1], INTER_LINEAR);
	drawGrid(und2, disp2, 1);
        //HGH - draw roi
        rectangle(disp2, Point2f(float(roi[1].x), float(roi[1].y)), Point2f(float(roi[1].x + roi[1].width), float(roi[1].y + roi[1].height)), CV_RGB(127, 255, 0));

#ifdef _BUILD_FOR_ROS_
	std::copy(&(disp2.at<Vec3b>(0,0)[0]), &(disp2.at<Vec3b>(0,0)[0])+(disp2.cols*disp2.rows*disp2.channels()), msg_debug[1].data.begin());
	debug_pub[1].publish(msg_debug[1], debug_camera_info[1]);
#else
	char windowName2[256];
	sprintf(windowName2, "rectified[1]");
	imshow(windowName2, disp2);
	waitKey((validPairs[0].size() > 50) ? 40 : 200);
#endif

	rectificationCount++;
}

void calibratorNode::performExtrinsicCalibration() {

	
	ROS_INFO("Initial intrinsics: ");
	
	std::cout << "K1 = " << configData.K[0] << endl;
	std::cout << "K2 = " << configData.K[1] << endl;
	
	std::cout << "imSize[0] = (" << imSize[0].height << ", " << imSize[0].width << ")" << endl;
	std::cout << "imSize[1] = (" << imSize[1].height << ", " << imSize[1].width << ")" << endl;
	
	std::vector<std::vector<Point2f> > emptyPointSetVector;
	vector<int> emptyIntVector;

	vector<vector<int> > extrinsicTagNames, extrinsicSelectedTags;
	//cv::vector<cv::vector<cv::vector<Point2f> > > extrinsicsList, extrinsicsCandidates;

	std::vector<Mat> extrinsicsDistributionMap;
	extrinsicsDistributionMap.resize(2);

	for (unsigned int nnn = 0; nnn < configData.numCams; nnn++) {
		//extrinsicsList.push_back(emptyPointSetVector);
		//extrinsicsCandidates.push_back(emptyPointSetVector);
		extrinsicTagNames.push_back(emptyIntVector);

		extrinsicsDistributionMap.at(nnn) = Mat(imSize[nnn], CV_8UC1);
	}

	
	for (unsigned int iii = 0; iii < extrinsicsPointSets[0].size(); iii++) {

		for (unsigned int nnn = 0; nnn < 2; nnn++) {
			//extrinsicsList.at(nnn).push_back(extrinsicsPointSets[nnn].at(iii));
			//extrinsicsCandidates.at(nnn).push_back(extrinsicsPointSets[nnn].at(iii));
			extrinsicTagNames.at(nnn).push_back(iii);
		}
	}
	
	vector<Size> extrinsicsSizes;
	extrinsicsSizes.push_back(imSize[0]);
	extrinsicsSizes.push_back(imSize[1]);

        //HGH - use all available patterns
        //-> configData.optimizationMethod = allPatterns
    
    ROS_INFO("Optimizing calibration sets, (%d) candidates and (%d) testing patterns", int(extrinsicCandidateSets[0].size()), int(extrinsicTestingSets[0].size()));
    
    int extrinsicsFlags = 0;
    
    if (!configData.noConstraints) {
		extrinsicsFlags += CALIB_ZERO_TANGENT_DIST;
	}
    
    if (configData.fixPrincipalPoint) {
		extrinsicsFlags += CALIB_FIX_PRINCIPAL_POINT;
	}
    
    if (configData.fixIntrinsics) {
		ROS_WARN("Fixing intrinsic parameters for extrinsic calibration..");
		extrinsicsFlags += CALIB_FIX_INTRINSIC + CALIB_FIX_FOCAL_LENGTH + CALIB_FIX_K6 + CALIB_FIX_K5 + CALIB_FIX_K4 + CALIB_FIX_K3 + CALIB_FIX_K2 + CALIB_FIX_K1;
	} else {
		extrinsicsFlags += CALIB_USE_INTRINSIC_GUESS;
	}
	
	if (configData.useRationalModel) {
		extrinsicsFlags += CALIB_RATIONAL_MODEL;
	}
	
	std::cout << "Super-Original camera matrices: " << endl;
    std::cout << "cameraMatrices[0] = " << cameraMatrices[0] << endl;
	std::cout << "cameraMatrices[1] = " << cameraMatrices[1] << endl;
    std::cout << "distCoeffVecs[0] = " << distCoeffVecs[0] << endl;
	std::cout << "distCoeffVecs[1] = " << distCoeffVecs[1] << endl;
	
	ROS_WARN("Optimizing with (%d) candidates and (%d) test patterns...", int(extrinsicCandidateSets[0].size()), int(extrinsicTestingSets[0].size()));
	
	optimizeCalibrationSets(extrinsicsSizes, 2, cameraMatrices, distCoeffVecs, extrinsicsDistributionMap, extrinsicCandidateSets, extrinsicTestingSets, row, configData.optimizationMethod, configData.setSize, extrinsicTagNames, extrinsicSelectedTags, extrinsicsFlags);

	//ROS_WARN("extrinsicsList.size() = %d", extrinsicsList.at(0).size());
	//ROS_WARN("extrinsicsCandidates.size() = %d", extrinsicsCandidates.at(0).size());
	
	// SO CANDIDATES IS WHERE THE SUBSELECTION ARE KEPT..
	
	TermCriteria term_crit;
	term_crit = TermCriteria(TermCriteria::COUNT+ TermCriteria::EPS, 30, 1e-6);


	// Should make R[0] and T[0] the "identity" matrices
	R[0] = Mat::eye(3, 3, CV_64FC1);
	T[0] = Mat::zeros(3, 1, CV_64FC1);
        // HGH - R[0] and T[0] not used but instead R[1] and T[1]
        R[1] = Mat::eye(3, 3, CV_64FC1);
        T[1] = Mat::zeros(3, 1, CV_64FC1);

	std::vector< std::vector<Point3f> > objectPoints;


        //HGH
//	for (int iii = 0; iii < extrinsicsCandidates[0].size(); iii++)
//	{
//		objectPoints.push_back(row);
//	}
        //HGH -or version from above can be used...
        objectPoints.resize(extrinsicCandidateSets[0].size());
        for (unsigned int iii = 0; iii < extrinsicCandidateSets[0].size(); iii++)
        {
            for (int j = 0; j < configData.yCount; j++ ){
                for (int i = 0; i < configData.xCount; i++ ){
                    objectPoints[iii].push_back(Point3f(float(j)*float(configData.gridSize/1000.0), float(i)*float(configData.gridSize/1000.0),0));
                }
            }
        }
        
        
	std::cout << "Original camera matrices: " << endl;
    std::cout << "cameraMatrices[0] = " << cameraMatrices[0] << endl;
	std::cout << "cameraMatrices[1] = " << cameraMatrices[1] << endl;
	std::cout << "distCoeffVecs[0] = " << distCoeffVecs[0] << endl;
	std::cout << "distCoeffVecs[1] = " << distCoeffVecs[1] << endl;

#ifdef _OPENCV_VERSION_3_PLUS_
	stereoError = stereoCalibrate(objectPoints,	extrinsicCandidateSets[0], extrinsicCandidateSets[1],
					cameraMatrices[0], distCoeffVecs[0], cameraMatrices[1], distCoeffVecs[1],
					extrinsicsSizes[0], R[1], T[1], E[1], F[1],
					extrinsicsFlags, term_crit);
#else
	stereoError = stereoCalibrate(objectPoints,	extrinsicCandidateSets[0], extrinsicCandidateSets[1],
					cameraMatrices[0], distCoeffVecs[0], cameraMatrices[1], distCoeffVecs[1],
					extrinsicsSizes[0], R[1], T[1], E[1], F[1],
					term_crit, extrinsicsFlags);
#endif
	
	std::cout << "Refined camera matrices: " << endl;
	std::cout << "cameraMatrices[0] = " << cameraMatrices[0] << endl;
	std::cout << "cameraMatrices[1] = " << cameraMatrices[1] << endl;
	std::cout << "distCoeffVecs[0] = " << distCoeffVecs[0] << endl;
	std::cout << "distCoeffVecs[1] = " << distCoeffVecs[1] << endl;
        std::cout << "R[1] = " << R[1] << endl;
        std::cout << "T[1] = " << T[1] << endl;
	printf("extrinsicsSizes[0] = (%d, %d)\n", extrinsicsSizes[0].width, extrinsicsSizes[0].height);

	//alpha = 1.00;

        //HGH - use optimal alpha based on left camera if using auto alpha
        if (configData.autoAlpha) {
                configData.alpha = findBestAlpha(cameraMatrices[0], distCoeffVecs[0], imSize[0]);
                ROS_INFO("Optimal alpha = (%f)", configData.alpha);
        }

        std::cout << "alpha = " << configData.alpha << endl;

        //HGH
        stereoRectify(cameraMatrices[0], distCoeffVecs[0], cameraMatrices[1], distCoeffVecs[1],
                                  imSize[0],
                                  R[1], T[1],
                                  R_[0], R_[1], P_[0], P_[1],
                                  Q[0],
                                  CALIB_ZERO_DISPARITY, /* or 0 */
                                  configData.alpha, imSize[0], &roi[0], &roi[1]);
//        stereoRectify(cameraMatrices[0], distCoeffVecs[0], cameraMatrices[1], distCoeffVecs[1],
//                                  imSize[0],
//                                  R[1], T[1],
//                                  R_[0], R_[1], P_[0], P_[1],
//                                  Q[0],
//                                  CALIB_ZERO_DISPARITY,
//                                  configData.alpha, imSize[0], &roi[0], &roi[1]);

/* HGH - this part was obviously to debug something?!!!
        Rect dummy_roi[2];
        stereoRectify(cameraMatrices[1], distCoeffVecs[1], cameraMatrices[0], distCoeffVecs[0],
                                  imSize[1],
                                  R[1].inv(), -T[1],
                                  R_[1], R_[0], P_[1], P_[0],
                                  Q[1],
                                  CALIB_ZERO_DISPARITY,
                                  configData.alpha, imSize[1], &dummy_roi[0], &dummy_roi[1]);
//*/


        printf("imsize[0] = (%d, %d)\n", imSize[0].width, imSize[0].height);
        printf("imsize[1] = (%d, %d)\n", imSize[1].width, imSize[1].height);

				  
	std::cout << "R_[0] = " << R_[0] << endl;
	std::cout << "R_[1] = " << R_[1] << endl;
	std::cout << "P_[0] = " << P_[0] << endl;
        std::cout << "P_[1] = " << P_[1] << endl;
	std::cout << "Q[0] = " << Q[0] << endl;
	std::cout << "Q[1] = " << Q[1] << endl;
				  
	printf("roi[0] = (%d, %d, %d, %d)\n", roi[0].x, roi[0].y, roi[0].width, roi[0].height);
	printf("roi[1] = (%d, %d, %d, %d)\n", roi[1].x, roi[1].y, roi[1].width, roi[1].height);
	
	extendedExtrinsicReprojectionError = calculateExtrinsicERE(2, objectPoints.at(0), extrinsicsPointSets, cameraMatrices, distCoeffVecs, R, T);
					
	std::cout << "R = " << R[1] << endl;
	std::cout << "T = " << T[1] << endl;
	
	return;
	
}

void calibratorNode::performIntrinsicCalibration() {
	
	// Restrictive: CV_CALIB_FIX_K5 + CV_CALIB_FIX_K4 + CV_CALIB_FIX_K3 + CV_CALIB_FIX_K2 + CV_CALIB_FIX_PRINCIPAL_POINT + CV_CALIB_FIX_ASPECT_RATIO + CV_CALIB_ZERO_TANGENT_DIST
	// Rational: CV_CALIB_RATIONAL_MODEL
	// Conventional: 0
	
	int intrinsicsFlags = 0;
	
	if (!configData.noConstraints) intrinsicsFlags += CALIB_FIX_ASPECT_RATIO; // + CV_CALIB_ZERO_TANGENT_DIST;
	
	if (configData.useRationalModel) intrinsicsFlags += CALIB_RATIONAL_MODEL;
	
	if (configData.ignoreDistortion) intrinsicsFlags += CALIB_FIX_K6 + CALIB_FIX_K5 + CALIB_FIX_K4 + CALIB_FIX_K3 + CALIB_FIX_K2 + CALIB_FIX_K1 + CALIB_ZERO_TANGENT_DIST;
	
	if (configData.fixPrincipalPoint) intrinsicsFlags += CALIB_FIX_PRINCIPAL_POINT;
	
	// #pragma parallel for
	for (int xxx = 0; xxx < int(configData.numCams); xxx++) {
		
		ROS_INFO("Intrinsically calibrating camera (%d) with (%d / %d / %d) patterns...", xxx, int(candidateSets[xxx].size()), int(testingSets[xxx].size()), int(pointSets[xxx].size()));
		
		bool debugOptimizationFunction = false;
		optimizeCalibrationSet(imSize[xxx], candidateSets[xxx], testingSets[xxx], row, subselectedTags_intrinsics[xxx], configData.optimizationMethod, configData.setSize, debugOptimizationFunction, configData.removeSpatialBias, configData.generateFigures, configData.useUndistortedLocations, intrinsicsFlags);
		
		ROS_INFO("Set optimized (%d)", int(subselectedTags_intrinsics[xxx].size()));
		
		vector< vector<Point3f> > objectPoints;
		vector< vector<Point2f> > subselectedPatterns;
		vector<Mat> rvecs, tvecs;

		for (unsigned int iii = 0; iii < subselectedTags_intrinsics[xxx].size(); iii++) {
			objectPoints.push_back(row);
			subselectedPatterns.push_back(candidateSets[xxx].at(iii));
		}

		rvecs.resize(subselectedTags_intrinsics[xxx].size());
		tvecs.resize(subselectedTags_intrinsics[xxx].size());
		
		Mat camMat, distCoeffs;
		
		
		
		reprojectionError_intrinsics[xxx] = calibrateCamera(objectPoints, subselectedPatterns, imSize[xxx], cameraMatrices[xxx], distCoeffVecs[xxx], rvecs, tvecs, intrinsicsFlags);
		
		ROS_INFO("Reprojection error = %f, distCoeffVecs[%d].size() = (%d)", reprojectionError_intrinsics[xxx], xxx, distCoeffVecs[xxx].cols);

		//extendedReprojectionError_intrinsics[xxx] = calculateERE(imSize[xxx], objectPoints.at(0), subselectedPatterns, cameraMatrices[xxx], distCoeffVecs[xxx], configData.generateFigures, errValues);
		extendedReprojectionError_intrinsics[xxx] = calculateERE(imSize[xxx], objectPoints.at(0), testingSets[xxx], cameraMatrices[xxx], distCoeffVecs[xxx], configData.removeSpatialBias, configData.generateFigures, configData.useUndistortedLocations);
		
		ROS_INFO("Extended Reprojection error = %f (%d patterns)", extendedReprojectionError_intrinsics[xxx], int(testingSets[xxx].size()));	
	}
	

	
}

#ifdef _BUILD_FOR_ROS_
bool calibratorData::obtainStartingData(ros::NodeHandle& nh) {

	nh.param<bool>("verboseMode", verboseMode, false);
	
	nh.param<bool>("fixIntrinsics", fixIntrinsics, true);
	
	nh.param<bool>("fixPrincipalPoint", fixPrincipalPoint, false);
	
	nh.param<bool>("noConstraints", noConstraints, false);
	
	
	nh.param<std::string>("video_stream", video_stream, "video_stream");
	
	
	
	nh.param<bool>("ignoreDistortion", ignoreDistortion, false);

	
	nh.param<bool>("useRationalModel", useRationalModel, false);
	nh.param<std::string>("calibType", calibType, "calibType");
	
	
	
	nh.param<bool>("useUndistortedLocations", useUndistortedLocations, false);
	nh.param<bool>("removeSpatialBias", removeSpatialBias, true);

	
	nh.param<std::string>("video_stream_secondary", video_stream_secondary, "video_stream_secondary");

	

	nh.param<double>("flowThreshold", flowThreshold, 1e-4);
	nh.param<double>("maxFrac", maxFrac, 0.05);
	nh.param<double>("errorThreshold", errorThreshold, 0.0);
		
	nh.param<std::string>("outputFolder", outputFolder, "outputFolder");
	
	
	
	
	
	nh.param<bool>("debugMode", debugMode, false);
	
	
	
	nh.param<std::string>("patternType", patternType, "mask");
	
	
	
	if (verboseMode) { ROS_INFO("<patternType> = (%s) [%d]", patternType.c_str(), pattType); }
	
	nh.param<int>("setSize", setSize, DEFAULT_SET_SIZE);
	
	if (verboseMode) { ROS_INFO("maxFrames = %d; maxPatterns = %d; setSize = (%d)", maxFrames, maxPatterns, setSize); }
	
	nh.param<double>("gridSize", gridSize, DEFAULT_GRID_SIZE);
	
	nh.param<double>("maxTimeDiff", maxTimeDiff, DEFAULT_MAX_TIME_DIFF);
	
	nh.param<int>("xCount", xCount, DEFAULT_X_COUNT);
	nh.param<int>("yCount", yCount, DEFAULT_Y_COUNT);
	
	nh.param<std::string>("optMethod", optMethod, "enhancedMCM");
	
	nh.param<bool>("undistortImages", undistortImages, false);
	
	nh.param<bool>("rectifyImages", rectifyImages, false);

	nh.param<bool>("wantsIntrinsics", wantsIntrinsics, false);

	nh.param<std::string>("intrinsicsFile_primary", intrinsicsFiles[0], "intrinsicsFile");
	nh.param<std::string>("intrinsicsFile_secondary", intrinsicsFiles[1], "intrinsicsFile");
	
	nh.param<double>("alpha", alpha, 0.00);
	
	nh.param<bool>("autoAlpha", autoAlpha, true);
	
	nh.param<bool>("stopCapturingAtLimit", stopCapturingAtLimit, false);
	nh.param<std::string>("patternDetectionMode", patternDetectionMode, "find");

	return processStartingData();
}
#endif

bool calibratorData::processStartingData() {

	if (video_stream != "video_stream") {
		ROS_INFO("<video_stream> (%s) selected.", video_stream.c_str());
	} else {
		ROS_ERROR("<video_stream> not specified.");
		return false;
	}

	invert[0] = invertPrimary;
	invert[1] = invertSecondary;

	if (numCams == 2) {
		if (video_stream_secondary != "video_stream_secondary") {
			ROS_INFO("<video_stream_secondary> = (%s).", video_stream_secondary.c_str());
		} else {
			ROS_ERROR("<video_stream_secondary> not specified!");
			return false;
		}
	}

	if (outputFolder != "outputFolder") {
		if (verboseMode) { ROS_INFO("Output folder (%s) selected.", outputFolder.c_str()); }
	} else {
		outputFolder = read_addr + "/../nodes/calibrator/data";
		if (verboseMode) { ROS_INFO("No output folder supplied. Defaulting to (%s)", outputFolder.c_str()); }
	}

	if (calibType == "single") {
		ROS_INFO("Single-camera calibration selected.");
		numCams = 1;
	} else if (calibType == "stereo") {
		ROS_INFO("Stereo calibration selected.");
		numCams = 2;
	} else if (calibType == "calibType") {
		ROS_WARN("No calibration type specified. Single-camera assumed.");
		calibType = "single";
	} else {
		ROS_ERROR("Invalid calibration type specified. Please choose either 'single' or 'stereo'.");
		return false;
	}

	if (calibType == "single") {
		if (wantsIntrinsics == false) {
			ROS_WARN("<single> camera mode has been selected, but <wantsIntrinsics> has not. Intrinsics WILL be calculated.");
			wantsIntrinsics = true;
		}
	} else if (calibType == "stereo") {
		if (wantsIntrinsics) { ROS_INFO("<stereo> mode: option to also calculate intrinsics has been selected.");	} 
		else {
			if (intrinsicsFiles[0] == "intrinsicsFile") ROS_WARN("No intrinsics specified for primary camera, so will attempt to use those encoded in topic header.");
			if (intrinsicsFiles[1] == "intrinsicsFile") ROS_WARN("No intrinsics specified for secondary camera, so will attempt to use those encoded in topic header.");
		}
	}

	if (verboseMode) { if (debugMode) { ROS_INFO("Running in DEBUG mode."); } else { ROS_INFO("Running in OPTIMIZED mode."); } }

	if (patternType == "mask") { pattType = MASK_FINDER_CODE; } 
	else if (patternType == "chessboard") {	pattType = CHESSBOARD_FINDER_CODE; } 
	else if (patternType == "heated_chessboard") { pattType = HEATED_CHESSBOARD_FINDER_CODE; } 
	else {
		ROS_ERROR("<patternType> incorrectly specified (mask/chessboard/heated-chessboard)");
		return false;
	}

	if (optMethod == "enhancedMCM") { optimizationMethod = ENHANCED_MCM_OPTIMIZATION_CODE; } 
	else if (optMethod == "allPatterns") { optimizationMethod = ALL_PATTERNS_OPTIMIZATION_CODE; } 
	else if (optMethod == "randomSet") { optimizationMethod = RANDOM_SET_OPTIMIZATION_CODE;	} 
	else if (optMethod == "firstN") { optimizationMethod = FIRST_N_PATTERNS_OPTIMIZATION_CODE; } 
	else if (optMethod == "randomBest") { optimizationMethod = BEST_OF_RANDOM_PATTERNS_OPTIMIZATION_CODE; } 
	else if (optMethod == "exhaustiveSearch") { optimizationMethod = EXHAUSTIVE_SEARCH_OPTIMIZATION_CODE; } 
	else if (optMethod == "randomSeed") { optimizationMethod = RANDOM_SEED_OPTIMIZATION_CODE; } 
	else if (optMethod == "scoreBased") { optimizationMethod = SCORE_BASED_OPTIMIZATION_CODE; } 
	else {
		ROS_ERROR("<optMethod> incorrectly specified...");
		return false;
	}

	return true;
}

void calibratorNode::updateIntrinsicMap(unsigned int idx) {
	
	if (configData.verboseMode) { ROS_INFO("Updating map..."); }
	
	if (configData.autoAlpha) {
		configData.alpha = findBestAlpha(cameraMatrices[idx], distCoeffVecs[idx], imSize[idx]);
		if (configData.verboseMode) { ROS_INFO("Optimal alpha = (%f)", configData.alpha); }
	}
	
	Rect* validPixROI = 0;
	bool centerPrincipalPoint = true;
	
	newCamMat[idx] = getOptimalNewCameraMatrix(cameraMatrices[idx], distCoeffVecs[idx], imSize[idx], configData.alpha, imSize[idx], validPixROI, centerPrincipalPoint);
		
	if (configData.verboseMode) { ROS_WARN("Adjusting undistortion mapping..."); }
	initUndistortRectifyMap(cameraMatrices[idx], distCoeffVecs[idx], default_R, newCamMat[idx], imSize[idx], CV_32FC1, map1[idx], map2[idx]);

	if (configData.verboseMode) { cout << "newCamMat: " << newCamMat[idx] << endl; }
	
	alphaChanged = false;
	
}
    
void calibratorNode::assignDebugCameraInfo() {
	
	// ..?
	

}

bool calibratorNode::isStillCollecting() {
		return stillCollecting;
}

////HGH

bool calibratorNode::isVerifying() {
                return doVerify;
}


bool calibratorNode::findPattern(const Mat& im, vector<Point2f>& dst, Mat& prev, const int cameraNumber = -1) {
	
	ElapsedTimeMilliseconds(tracking_timer);
	
    //HGH
    bool retVal = false;
    Mat tmpMat;

	if ((configData.trackingMode) && (dst.size() > 0) && (prev.rows > 0) && (prev.cols > 0)) {
		
		int distanceConstraint = int((min(im.cols, im.rows) * configData.maxFrac) + ((((int) (max(im.cols, im.rows) * configData.maxFrac)) + 1) % 2));

			vector<Point2f> src;
			vector<unsigned int> lost;
			lost.clear();
			src.insert(src.end(), dst.begin(), dst.end());
			dst.clear();

			//ROS_WARN("ATTEMPTING TO TRACK...");

			//ROS_INFO("im data = (%d, %d) : (%d, %d)", prev.rows, prev.cols, im.rows, im.cols);

			//ROS_INFO("Tracking parameters: (%f, %f, %f)", distanceConstraint, configData.flowThreshold, configData.errorThreshold);

			trackPoints2(prev, im, src, dst, distanceConstraint, configData.flowThreshold, lost, cvSize(configData.xCount, configData.yCount), configData.errorThreshold);

			int windowSize = distanceConstraint;
			
			if ((windowSize % 2) == 0) windowSize++;


			cornerSubPix(im, dst, Size(windowSize, windowSize), Size(-1,-1), cvTermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 15, 0.1));

			double maxMotion = 0.0;
				
			for (unsigned int iii = 0; iii < dst.size(); iii++) {
				
				double dist = max((dst.at(iii).x -  src.at(iii).x), (dst.at(iii).y -  src.at(iii).y));
				
				if (dist > maxMotion) maxMotion = dist;
			}
			
			if ((maxMotion > distanceConstraint) || (lost.size() > 0)) {
				if (configData.verboseMode) { ROS_WARN("Tracking failed."); } // configData.verboseMode
				dst.clear();
				retVal = false;
			} else if (!verifyCorners(im.size(), cvSize(configData.xCount, configData.yCount), dst)) {
				if (configData.verboseMode) { ROS_WARN("Tracking verification failed."); }
				dst.clear();
				retVal = false;
			} else {
				if (configData.verboseMode) { ROS_INFO("Tracking successful."); }
				//HGH
				retVal = true;
			}
			
	}
	
	if (!retVal) {
		
		switch (configData.pattType) {
			
			case CHESSBOARD_FINDER_CODE:
				retVal = findChessboardCorners(im, cvSize(configData.xCount, configData.yCount), dst);
				break;
			case MASK_FINDER_CODE:
				if (configData.adjustMSER[0] && cameraNumber == 0){
					retVal = findMaskCorners(im, cvSize(configData.xCount, configData.yCount), dst, PATTERN_FINDER_CV_CORNER_SUBPIX_FLAG, configData.delta[0], configData.maxVar[0], configData.minDiv[0]);
				} else if (configData.adjustMSER[1] && cameraNumber == 1){
					retVal = findMaskCorners(im, cvSize(configData.xCount, configData.yCount), dst, PATTERN_FINDER_CV_CORNER_SUBPIX_FLAG, configData.delta[1], configData.maxVar[1], configData.minDiv[1]);
				} else {
					retVal = findMaskCorners(im, cvSize(configData.xCount, configData.yCount), dst, PATTERN_FINDER_CV_CORNER_SUBPIX_FLAG);
				}
				break;
			case HEATED_CHESSBOARD_FINDER_CODE:
				invertMatIntensities(im, tmpMat);
				retVal = findChessboardCorners(tmpMat, cvSize(configData.xCount, configData.yCount), dst);
				break;
			default:
				retVal = findChessboardCorners(im, cvSize(configData.xCount, configData.yCount), dst);
				break;
				
		}
	}
	
	elapsedTrackingTime = ElapsedTimeMilliseconds(tracking_timer);
	
	// ROS_INFO("elapsedTrackingTime = (%f)", elapsedTrackingTime);
		

	frameTrackingTime += elapsedTrackingTime;
	
	ROS_INFO("dst.size() = (%d); status = (%d)", int(dst.size()), retVal);

    return retVal;
		
}

void calibratorNode::determineValidPairs() {

        ROS_INFO("Determine valid frame pairs...");

	unsigned int maxCam1 = std::min((unsigned int)(times[0].size()), frameCount[0]);
	unsigned int maxCam2 = std::min((unsigned int)(times[1].size()), frameCount[1]);


        //Mat dispMat1, dispMat2;
        //ROS_WARN("HERE BEGIN---");
	while (checkIndex[0] < maxCam1) {
		
		if (checkIndex[0] >= duplicateFlags[0].size()) {
			break;
		}
		
                //ROS_WARN("checkIndex[0] %d-",checkIndex[0]);

		if  (duplicateFlags[0].at(checkIndex[0]) == 1) {
			checkIndex[0]++;
			continue;
		}
		
                //ROS_WARN("- checkIndex[0] %d-",checkIndex[0]);
                //ROS_WARN("times[0].size() %d-",times[0].size());
                //cout << times[0].at(checkIndex[0]) << endl;


		double leastDiff = 9e99;
		int bestMatch = -1;
		
		bool candidatesExhausted = false;
	
                //ROS_WARN("checkIndex[1] %d-",checkIndex[1]);
                //ROS_WARN("times[1].size() %d-",times[1].size());
                //cout << times[1].at(checkIndex[1]) << endl;

                //cout << "m(TimeDifference(times[1].at(checkIndex[1]), times[0].at(checkIndex[0])) < configData.maxTimeDiff) " << (TimeDifference(times[1].at(checkIndex[1]), times[0].at(checkIndex[0])) < configData.maxTimeDiff) << endl;
                //ROS_WARN("diffdone");
		
		while ((TimeDifference(times[1].at(checkIndex[1]), times[0].at(checkIndex[0])) < configData.maxTimeDiff)) {
			
                        //ROS_WARN("maxCam2-1= %d-",maxCam2-1);
                        //ROS_WARN("checkIndex[1] %d-",checkIndex[1]);
			if (checkIndex[1] >= (maxCam2-1)) {
                                ROS_WARN("...");
				break;
				
			}

                        if (checkIndex[1] >= duplicateFlags[1].size()) {
                            break;
                        }

                        //ROS_WARN("DEUBG %s", 3);
			if  (duplicateFlags[1].at(checkIndex[1]) == 1) {
				checkIndex[1]++;
				continue;
			}
                        //ROS_WARN("DEUBG %s", 4);
			double diff = abs(TimeDifference(times[1].at(checkIndex[1]), times[0].at(checkIndex[0])));
			
			
                       // ROS_WARN("DEUBG %s", 5);
			if (diff < leastDiff) {
				leastDiff = diff;
				bestMatch = checkIndex[1];
			}
			
			checkIndex[1]++;
		}

                //ROS_WARN("DEUBG %s", 10);
		
		if ((TimeDifference(times[1].at(checkIndex[1]), times[0].at(checkIndex[0])) >= configData.maxTimeDiff)) {
			candidatesExhausted = true;
		}
		
		//checkIndex[1] = bestMatch;
		
		if ((leastDiff < configData.maxTimeDiff) && (bestMatch >= 0)) {
			
			//ROS_WARN("Pushing back match (%f)  (%d) [%02d.%04d] and (%d) [%02d.%04d]...", leastDiff, checkIndex[0], times[0].at(checkIndex[0]).sec, times[0].at(checkIndex[0]).nsec, bestMatch, times[1].at(bestMatch).sec, times[1].at(bestMatch).nsec);
			
			validPairs[0].push_back(checkIndex[0]);
			checkIndex[0]++;
			validPairs[1].push_back(bestMatch);


		} else if (candidatesExhausted) { // (checkIndex[1] < (maxCam2-1)) {
			checkIndex[0]++;
		} else {
			break;
		}
		
	}
			
}


void calibratorNode::evaluateFrames() {

     ROS_WARN("evaluateFrames...");

        vector<validPattern> validPatternData[2];


        // check if pattern was found in valid frame:

        for (unsigned int camNumber = 0; camNumber <= 2; camNumber++ ){

            for (unsigned int i = 0; i < validPairs[camNumber].size(); i++){

                for (unsigned int j = 0; j < frameCounts[camNumber].size(); j++){

                    validPattern tmpData;
                    tmpData.validFrameID = validPairs[camNumber].at(i);
                    tmpData.patternID = j;


                    if (validPairs[camNumber].at(i)==frameCounts[camNumber].at(j)){

                        tmpData.patternFound = true;
                        tmpData.isValidFrame = true;

                        validPatternData[camNumber].push_back(tmpData);
                        break;


                        ROS_INFO("Valid pattern found for cam (%d) in frame %d", camNumber, frameCounts[camNumber].at(j));

                    } else if(j==frameCounts[camNumber].size()-1){

                        //no valid pattern found for valid pair ///stimmt loop?

                        tmpData.patternFound = false;
                        tmpData.isValidFrame = true;

                         validPatternData[camNumber].push_back(tmpData);

                    }
                }
            }
        }


        Mat dispMat1, dispMat2;



        int totalValidPatterns = 0;

        for(unsigned int i=0; i < validPatternData[0].size(); i++){

            if ( validPatternData[0].at(i).patternFound && validPatternData[1].at(i).patternFound){

                 if ( !configData.debugMode) {
                    extrinsicsPointSets[0].push_back(pointSets[0].at(validPatternData[0].at(i).patternID));
                    extrinsicsPointSets[1].push_back(pointSets[1].at(validPatternData[1].at(i).patternID));
                 }
                totalValidPatterns++;

            }

        }

        cout << "synced frames: " << validPairs[0].size() << endl;
        cout << "cam1 patterns found: " <<  frameCounts[0].size() << endl;
        cout << "cam2 patterns found: " <<  frameCounts[1].size() << endl;

        cout << "valid pairs: " <<  validPatternData[0].size() << endl;
        cout << "total valid patterns: " << totalValidPatterns << endl;



        if ( configData.debugMode) {

            doVerify = true;

            int currentValidPattern = 0;

            for(unsigned int i=0; i < validPatternData[0].size(); i++){

                if ( validPatternData[0].at(i).patternFound && validPatternData[1].at(i).patternFound){

                    ROS_INFO("\nValid pattern pair %d out of  %d :  frame cam1: %d, frame cam2 %d ", currentValidPattern + 1, totalValidPatterns, validPatternData[0].at(i).patternID ,  validPatternData[1].at(i).patternID );



                    int dispFrame1 = validPatternData[0].at(i).validFrameID;
                    int dispFrame2 = validPatternData[1].at(i).validFrameID;

                    int dispPattern1 = validPatternData[0].at(i).patternID;
                    int dispPattern2 = validPatternData[1].at(i).patternID;

                    if (displayImages[0].at(dispFrame1).type() == CV_8UC3) {
                        displayImages[0].at(dispFrame1).copyTo(dispMat1);
                    } else {
                        cvtColor(displayImages[0].at(dispFrame1), dispMat1, CV_GRAY2RGB);
                    }


                    drawChessboardCorners(dispMat1, cvSize(configData.xCount, configData.yCount), Mat(pointSets[0].at(dispPattern1)), true);


                    if (displayImages[1].at(dispFrame2).type() == CV_8UC3) {
                        displayImages[1].at(dispFrame2).copyTo(dispMat2);
                    } else {
                        cvtColor(displayImages[1].at(dispFrame2), dispMat2, CV_GRAY2RGB);
                    }

                    drawChessboardCorners(dispMat2, cvSize(configData.xCount, configData.yCount), Mat(pointSets[1].at(dispPattern2)), true);


                    if(currentValidPattern < totalValidPatterns - 1){

                        imshow("pattern1", dispMat1);
                        imshow("pattern2", dispMat2);

                        ROS_INFO("%d << Press 'c' to use this keyframes or any other key to discard...", i);


                        char key = waitKey();
                        if (key == 'c'){
                            extrinsicsPointSets[0].push_back(pointSets[0].at(validPatternData[0].at(i).patternID));
                            extrinsicsPointSets[1].push_back(pointSets[1].at(validPatternData[1].at(i).patternID));
                            ROS_INFO("Added patterns for valid pair %d...", currentValidPattern);
                        }

                        currentValidPattern++;

                    } else {
                        destroyWindow("pattern1");
                        destroyWindow("pattern2");
                        doVerify = false;
                    }

                }

            }

        }

}

//preprocess images
void calibratorNode::preprocessImage(Mat src, Mat &dst, double a = 1.0, double b = 0.0, bool normalize = false, bool negative = false) {

    Mat newImage = Mat::zeros(src.size(), src.type());

	if (src.type() == CV_8UC1) { for (int j = 0; j < src.rows; j++) { for (int i = 0; i < src.cols; i++) { newImage.at<uchar>(j,i) = saturate_cast<uchar> (a * (src.at<uchar>(j,i))+ b); } } }
	else { for (int j = 0; j < src.rows; j++) { for (int i = 0; i < src.cols; i++) { for (int c = 0; c< 3; c++) { newImage.at<Vec3b>(j,i)[c] = saturate_cast<uchar> (a * (src.at<Vec3b>(j,i)[c])+ b); } } } }

    if (normalize) {
        cvtColor(newImage, src, CV_RGB2GRAY);
        equalizeHist(src,newImage);
    }

    //invert
    if (negative) {

    if (src.type() == CV_8UC1){

        for (int j = 0; j < newImage.rows; j++){
            for (int i = 0; i < newImage.cols; i++){
                    newImage.at<uchar>(j,i) = 255 - saturate_cast<uchar> (newImage.at<uchar>(j,i));
            }
        }

    }else{

        for (int j = 0; j < newImage.rows; j++){
            for (int i = 0; i < newImage.cols; i++){
                for (int c = 0; c< 3; c++){
                    newImage.at<Vec3b>(j,i)[c] = 255 - saturate_cast<uchar> ( newImage.at<Vec3b>(j,i)[c]);
                }
            }
        }

    }

    }

    dst = Mat(newImage);
}


void calibratorNode::updatePairs() {
	
	if (configData.numCams == 1) validSets = (unsigned int) pointSets[0].size();

    //HGH
    determineValidPairs();
    Mat dispMat1, dispMat2;
    /* HGH

        int maxCam1 = std::min(((int) times[0].size()), frameCount[0]);
        int maxCam2 = std::min(((int) times[1].size()), frameCount[1]);

        Mat dispMat1, dispMat2;

        while (checkIndex[0] < maxCam1) {

                if (checkIndex[0] >= duplicateFlags[0].size()) {
                        break;
                }

                if  (duplicateFlags[0].at(checkIndex[0]) == 1) {
                        checkIndex[0]++;
                        continue;
                }

                double leastDiff = 9e99;
                int bestMatch = -1;

                bool candidatesExhausted = false;


                while ((TimeDifference(times[1].at(checkIndex[1]), times[0].at(checkIndex[0])) < configData.maxTimeDiff)) {

                        if (checkIndex[1] >= (maxCam2-1)) {
                                break;

                        }

                        if (checkIndex[1] >= duplicateFlags[1].size()) {
                                break;
                        }

                        if  (duplicateFlags[1].at(checkIndex[1]) == 1) {
                                checkIndex[1]++;
                                continue;
                        }

                        double diff = abs(TimeDifference(times[1].at(checkIndex[1]), times[0].at(checkIndex[0])));



                        if (diff < leastDiff) {
                                leastDiff = diff;
                                bestMatch = checkIndex[1];
                        }

                        checkIndex[1]++;
                }

                if ((TimeDifference(times[1].at(checkIndex[1]), times[0].at(checkIndex[0])) >= configData.maxTimeDiff)) {
                        candidatesExhausted = true;
                }

                //checkIndex[1] = bestMatch;

                if ((leastDiff < configData.maxTimeDiff) && (bestMatch >= 0)) {

                        //ROS_WARN("Pushing back match (%f)  (%d) [%02d.%04d] and (%d) [%02d.%04d]...", leastDiff, checkIndex[0], times[0].at(checkIndex[0]).sec, times[0].at(checkIndex[0]).nsec, bestMatch, times[1].at(bestMatch).sec, times[1].at(bestMatch).nsec);

                        validPairs[0].push_back(checkIndex[0]);
                        checkIndex[0]++;
                        validPairs[1].push_back(bestMatch);
                } else if (candidatesExhausted) { // (checkIndex[1] < (maxCam2-1)) {
                        checkIndex[0]++;
                } else {
                        break;
                }

        }

        //*/

        if (validPairs[0].size() > publishCount) {

                int currentPair = int(validPairs[0].size())-1;

                if (currentPair == -1) return;



                // This is probably where it's worth attempting to find the patterns

                bool patternFound_1 = false, patternFound_2 = false;

                Mat tmpMat1, tmpMat2;

                vector<Point2f> cornerSet_1, cornerSet_2;


                if ((validPairs[0].at(currentPair) >= displayImages[0].size()) || (validPairs[1].at(currentPair) >= displayImages[1].size())) {
                        ROS_WARN("Skipping detection because there appear to be insufficient buffered frames [(%d)(%d) : (%d)(%d)", validPairs[0].at(currentPair), int(displayImages[0].size()), validPairs[1].at(currentPair), int(displayImages[1].size()));
                        return;
                }

                //ROS_WARN("Processing pair (%d) (%d, %d)[sync'ed: %f]", currentPair, validPairs[0].at(currentPair), validPairs[1].at(currentPair), TimeDifference(times[1].at(validPairs[1].at(currentPair)), times[0].at(validPairs[0].at(currentPair))));

                Mat grayMat1, grayMat2;

                if (displayImages[0].at(validPairs[0].at(currentPair)).type() == CV_8UC3) {
                        cvtColor(displayImages[0].at(validPairs[0].at(currentPair)), grayMat1, CV_RGB2GRAY);
                } else {
                        grayMat1 = Mat(displayImages[0].at(validPairs[0].at(currentPair)));
                }

                //ROS_WARN("First image retrieved..");

                if (displayImages[1].at(validPairs[1].at(currentPair)).type() == CV_8UC3) {
                        cvtColor(displayImages[1].at(validPairs[1].at(currentPair)), grayMat2, CV_RGB2GRAY);
                } else {
                        grayMat2 = Mat(displayImages[1].at(validPairs[1].at(currentPair)));
                }


                switch (configData.pattType) {
                        case CHESSBOARD_FINDER_CODE:
                                patternFound_1 = findChessboardCorners(grayMat1, cvSize(configData.xCount, configData.yCount), cornerSet_1);
                                patternFound_2 = findChessboardCorners(grayMat2, cvSize(configData.xCount, configData.yCount), cornerSet_2);
                                break;
                        case MASK_FINDER_CODE:
                                patternFound_1 = findMaskCorners(grayMat1, cvSize(configData.xCount, configData.yCount), cornerSet_1, PATTERN_FINDER_CV_CORNER_SUBPIX_FLAG);
                                patternFound_2 = findMaskCorners(grayMat2, cvSize(configData.xCount, configData.yCount), cornerSet_2, PATTERN_FINDER_CV_CORNER_SUBPIX_FLAG);
                                break;
                        case HEATED_CHESSBOARD_FINDER_CODE:
                                invertMatIntensities(grayMat1, tmpMat1);
                                patternFound_1 = findChessboardCorners(tmpMat1, cvSize(configData.xCount, configData.yCount), cornerSet_1);

                                invertMatIntensities(grayMat2, tmpMat2);
                                patternFound_2 = findChessboardCorners(tmpMat2, cvSize(configData.xCount, configData.yCount), cornerSet_2);

                                break;
                        default:
                                patternFound_1 = findChessboardCorners(grayMat1, cvSize(configData.xCount, configData.yCount), cornerSet_1);
                                patternFound_2 = findChessboardCorners(grayMat2, cvSize(configData.xCount, configData.yCount), cornerSet_2);
                                break;
                }

                if (patternFound_1 && patternFound_2) {

                        extrinsicsPointSets[0].push_back(cornerSet_1);
                        extrinsicsPointSets[1].push_back(cornerSet_2);
                        ROS_INFO("(%d) patterns found from (%d) valid frame pairs...", int(extrinsicsPointSets[0].size()), int(validPairs[0].size()));
                        pointSets[0].push_back(cornerSet_1);
                        pointSets[1].push_back(cornerSet_2);
                        //ROS_INFO("For cam (0), (%d) patterns found.", pointSets[0].size());
                        //ROS_INFO("For cam (1), (%d) patterns found.", pointSets[1].size());
                } else if (patternFound_1) {
                        pointSets[0].push_back(cornerSet_1);
                        //ROS_INFO("For cam (0), (%d) patterns found.", pointSets[0].size());
                } else if (patternFound_2) {
                        pointSets[1].push_back(cornerSet_2);
                        //ROS_INFO("For cam (1), (%d) patterns found.", pointSets[1].size());
                }

                if (configData.debugMode) {

                        if (displayImages[0].at(validPairs[0].at(currentPair)).type() == CV_8UC3) {
                                displayImages[0].at(validPairs[0].at(currentPair)).copyTo(dispMat1);
                        } else {
                                cvtColor(displayImages[0].at(validPairs[0].at(currentPair)), dispMat1, CV_GRAY2RGB);
                        }


                        drawChessboardCorners(dispMat1, cvSize(configData.xCount, configData.yCount), Mat(cornerSet_1), patternFound_1);

#ifdef _BUILD_FOR_ROS_
                        if (msg_debug[0].width > 0) std::copy(&(dispMat1.at<Vec3b>(0,0)[0]), &(dispMat1.at<Vec3b>(0,0)[0])+(dispMat1.cols*dispMat1.rows*dispMat1.channels()), msg_debug[0].data.begin());
#endif
                        if (displayImages[1].at(validPairs[1].at(currentPair)).type() == CV_8UC3) {
                                displayImages[1].at(validPairs[1].at(currentPair)).copyTo(dispMat2);
                        } else {
                                cvtColor(displayImages[1].at(validPairs[1].at(currentPair)), dispMat2, CV_GRAY2RGB);
                        }

                        drawChessboardCorners(dispMat2, cvSize(configData.xCount, configData.yCount), Mat(cornerSet_2), patternFound_2);

#ifdef _BUILD_FOR_ROS_
                        if (msg_debug[1].width > 0) std::copy(&(dispMat2.at<Vec3b>(0,0)[0]), &(dispMat2.at<Vec3b>(0,0)[0])+(dispMat2.cols*dispMat2.rows*dispMat2.channels()), msg_debug[1].data.begin());

                        //ROS_ERROR("Publishing frame pair...");
                        debug_pub[0].publish(msg_debug[0], debug_camera_info[0]);
                        debug_pub[1].publish(msg_debug[1], debug_camera_info[1]);
#endif
                }

                publishCount = (unsigned int) validPairs[0].size();
                //publishCount++;
        }


}

#ifdef _BUILD_FOR_ROS_
void calibratorNode::handle_camera(const sensor_msgs::ImageConstPtr& msg_ptr, const sensor_msgs::CameraInfoConstPtr& info_msg, const unsigned int camera_index) {
#else
void calibratorNode::handle_camera(const cv::Mat& inputImage, const sensor_msgs::CameraInfo *info_msg, const unsigned int camera_index) {
#endif

    vacantInputTime = ElapsedTimeMilliseconds(vacant_timer);
    
    if (frameCount[camera_index] == 0) {
		elapsedInputTime = ElapsedTimeMilliseconds(elapsed_timer);
	}
    
    unsigned int totalMinPatterns = (unsigned int) patternIndices[0].size();
    
    for (unsigned int iii = 1; iii < configData.numCams; iii++) {
		totalMinPatterns = min(totalMinPatterns, ((unsigned int)(patternIndices[iii].size())));
	}
    
    if ((configData.stopCapturingAtLimit) || ((((int)totalMinPatterns) > configData.maxPatterns) && (configData.maxPatterns > 0)) || ((((int)frameCount[camera_index]) > configData.maxFrames) && (configData.maxFrames > 0)) ) { 
		stillCollecting = false; 
		isValid = false;
	}
    
	if (!stillCollecting) {	return; }

	 if (configData.verboseMode) { ROS_INFO("Handling image (camera: %d)", camera_index); }

	while (!infoProcessed[camera_index]) {
		
		if (configData.wantsIntrinsics) {
			infoProcessed[camera_index] = true;
		} else {

			if (configData.intrinsicsFiles[camera_index] != "intrinsicsFile") {
			
				ROS_WARN("For camera (%d), using intrinsics file (%s)", camera_index, configData.intrinsicsFiles[camera_index].c_str());

				if (configData.intrinsicsFiles[camera_index].at(0) != '/') {
					configData.intrinsicsFiles[camera_index] = configData.read_addr + configData.intrinsicsFiles[camera_index];
				}

				
				try {

					FileStorage fs(configData.intrinsicsFiles[camera_index], FileStorage::READ);
					fs["cameraMatrix"] >> configData.K[camera_index];
					fs["distCoeffs"] >> configData.distCoeffs[camera_index];
					fs.release();

					ROS_INFO("Calibration data read.");

					if (configData.K[camera_index].empty()){
						ROS_ERROR("Intrinsics file (%s) invalid! Please check path and filecontent...\n", configData.intrinsicsFiles[camera_index].c_str());
					}

					infoProcessed[camera_index] = true;

				} catch (...) {
					ROS_ERROR("Some failure in reading in the camera parameters for camera (%d) from a file.", camera_index);
				}

			} else {
				
				ROS_WARN("For camera (%d), trying to extract intrinsics from msg header", camera_index);
				
				// Extract intrinsics from msg header
				try	{
					
					configData.K[camera_index] = Mat::eye(3, 3, CV_64FC1);
					
					for (unsigned int mmm = 0; mmm < 3; mmm++) {
						for (unsigned int nnn = 0; nnn < 3; nnn++) {
							configData.K[camera_index].at<double>(mmm, nnn) = info_msg->K[3*mmm + nnn];
						}
					}
					
					unsigned int maxDistortionIndex;
					if (info_msg->distortion_model == "plumb_bob") {
						maxDistortionIndex = 5;
					} else if (info_msg->distortion_model == "rational_polynomial") {
						maxDistortionIndex = 8;
					}
					
					configData.distCoeffs[camera_index] = Mat::zeros(1, maxDistortionIndex, CV_64FC1);
					
					for (unsigned int iii = 0; iii < maxDistortionIndex; iii++) {
						configData.distCoeffs[camera_index].at<double>(0, iii) = info_msg->D[iii];
					}
					
					ROS_INFO("Camera (%d) intrinsics:", camera_index);
					cout << configData.K[camera_index] << endl;
					cout << configData.distCoeffs[camera_index] << endl;
					infoProcessed[camera_index] = true;
					ROS_INFO("Debug marker (%d) [%d]:", camera_index, 0);
				} catch (...) /*(sensor_msgs::CvBridgeException& e)*/ {
					ROS_ERROR("Some failure in reading in the camera parameters for camera (%d) from the topic header.", camera_index);
				}

			}
		} 
		
		if (configData.verboseMode) { ROS_INFO("Camera info for cam (%d) now processed.", camera_index); };
		
	}
	
	if (infoProcessed[camera_index]) {

		elapsedTime = ElapsedTimeMilliseconds(cycle_timer);
		
		if (frameCount[camera_index] > 0) {
			avgTime += elapsedTime;
		}
		
		times[camera_index].push_back(info_msg->header.stamp);
		
		if (configData.verboseMode) { ROS_INFO("Bridging (%d)...", camera_index); };
		
#ifdef _BUILD_FOR_ROS_
	cv_ptr[camera_index] = cv_bridge::toCvCopy(msg_ptr, enc::BGR8);
#else
	cv_ptr[camera_index] = inputImage;
#endif
		
		
		if (configData.verboseMode) { ROS_INFO("Bridged (%d).", camera_index); };
		
#ifdef _BUILD_FOR_ROS_
		Mat newImage(cv_ptr[camera_index]->image);
#else
		Mat newImage(cv_ptr[camera_index]);
#endif
		
		Mat preDisplay, grayMat;
		
		switch (newImage.type()) {
		case CV_8UC3:
			cvtColor(newImage, grayMat, CV_RGB2GRAY);
			newImage.copyTo(preDisplay);
			break;
		case CV_16UC1:
			adaptiveDownsample(newImage, grayMat, NORM_MODE_FIXED_TEMP_RANGE, 0.2);
			cvtColor(newImage, preDisplay, CV_GRAY2RGB);
			break;
		case CV_8UC1:
			newImage.copyTo(grayMat);
			cvtColor(grayMat, preDisplay, CV_GRAY2RGB);
			break;
		default:
			ROS_ERROR("Image type not recognized!");
			return;
		}

		if (configData.invert[camera_index]) invertMatIntensities(grayMat, grayMat); 
        
        if (configData.verboseMode) { ROS_INFO("Color and inversion processing complete (%d).", camera_index); };

		if (displayImages[camera_index].size() > 0) {
			if (matricesAreEqual(displayImages[camera_index].at(displayImages[camera_index].size()-1), grayMat)) {
				duplicateFlags[camera_index].push_back(1);
				ROS_WARN("Received duplicate frame at cam 1 (%d)", frameCount[0]);
			} else duplicateFlags[camera_index].push_back(0);
		} else {
			duplicateFlags[camera_index].push_back(0);
			imSize[camera_index] = newImage.size();
		}
		
		// Check for sync'ed frames here (only with camera 1)
		
		//ROS_WARN("DEBUG checking frames img1..");
		//HGH
		if (duplicateFlags[camera_index].at(duplicateFlags[camera_index].size()-1) == 0) {
			
			displayImages[camera_index].push_back(grayMat);
		
			if (configData.debugMode) {
			
				if (configData.verboseMode) { ROS_INFO("Configuring debug image (%d).", camera_index); };
				
#ifdef _BUILD_FOR_ROS_
				if (msg_debug[camera_index].width == 0) {
					msg_debug[camera_index].width = preDisplay.cols; 
					msg_debug[camera_index].height = preDisplay.rows;
					msg_debug[camera_index].encoding = "bgr8";
					msg_debug[camera_index].is_bigendian = false;
					msg_debug[camera_index].step = preDisplay.cols*3;
					msg_debug[camera_index].data.resize(preDisplay.cols*preDisplay.rows*preDisplay.channels());
				}
#endif
				if (configData.verboseMode) { ROS_INFO("Debug image configured (%d).", camera_index); };
				
			}
			
			if (configData.verboseMode) { ROS_INFO("Searching for pattern (%d).", camera_index); };
			bool patternFound = findPattern(grayMat, cornerSet[camera_index], prevMat[camera_index]);
			if (configData.verboseMode) { ROS_INFO("Pattern found? (%d) [%d].", camera_index, patternFound); };
			
			if (patternFound) {
				
				if (cornerSet[camera_index].size() == 0) {
					ROS_ERROR("Says pattern found, but no points!");
					imshow("tmp", prevMat[camera_index]);
					waitKey();
				}
							
				patternIndices[camera_index].push_back(frameCount[camera_index]);
				grayMat.copyTo(prevMat[camera_index]);
				pointSets[camera_index].push_back(cornerSet[camera_index]);
				
				patternTimestamps[camera_index].push_back(info_msg->header.stamp);
				
			}
				
			
			
			if (configData.debugMode) {
				
				Mat dispMat;
				
				preDisplay.copyTo(dispMat);

				if (configData.verboseMode) { ROS_INFO("Drawing pattern (%d).", camera_index); };
				drawChessboardCorners(dispMat, cvSize(configData.xCount, configData.yCount), Mat(cornerSet[camera_index]), patternFound);
				if (configData.verboseMode) { ROS_INFO("Pattern drawn (%d).", camera_index); };

				
				
#ifdef _BUILD_FOR_ROS_
				if (configData.verboseMode) { ROS_INFO("About to copy (%d).", camera_index); };
				
				std::copy(&(dispMat.at<Vec3b>(0,0)[0]), &(dispMat.at<Vec3b>(0,0)[0])+(dispMat.cols*dispMat.rows*dispMat.channels()), msg_debug[camera_index].data.begin());
							
				if (configData.verboseMode) { ROS_INFO("Preparing to publish (%d).", camera_index); };
				debug_pub[camera_index].publish(msg_debug[camera_index], debug_camera_info[camera_index]);
				if (configData.verboseMode) { ROS_INFO("Published (%d).", camera_index); };
#else
				char windowName[256];
				sprintf(windowName, "calibration[%d]", camera_index);
				imshow(windowName, dispMat);
				waitKey(1);
#endif
			}
			
			frameCount[camera_index]++;
			
		} else {
			ROS_ERROR("Skipping frame because it is a duplicate...");
		}

			
	}
		
	
	
}

#ifdef _BUILD_FOR_ROS_
calibratorNode::calibratorNode(ros::NodeHandle& nh, calibratorData startupData) :
#else
calibratorNode::calibratorNode(calibratorData startupData) :
#endif
	alpha(0.0),
	vacantInputTime(0.0),
	elapsedInputTime(0.0),
	alphaChanged(true),
	publishCount(0),
	readyForOutput(false),
	topValidHeight(0),
	botValidHeight(65535),
	stillCollecting(true),
	doVerify(false),
	avgTime(0.0),
	frameTrackingTime(0.0),
	undistortionCount(0),
	rectificationCount(0)
{
	ElapsedTimeMilliseconds(vacant_timer);
	ElapsedTimeMilliseconds(elapsed_timer);

#ifdef _BUILD_FOR_ROS_
	timer = nh.createTimer(ros::Duration(DEFAULT_TIMER_PERIOD), &calibratorNode::timerCallback, this);
	sprintf(nodeName, "%s", ros::this_node::getName().c_str());
#endif

	default_R = cv::Mat::eye(3,3,CV_64FC1);

	for (unsigned int iii = 0; iii < MAX_ALLOWABLE_CAMS; iii++) {
		frameCount[iii] = 0;
		totalFrameCount[iii] = 0;
		infoProcessed[iii] = false;
		checkIndex[iii] = 0;
	}
	
	configData = startupData;
	
	if (configData.calibType == "single") {
		
		sprintf(debug_pub_name[0], "/thermalvis%s/image_col", nodeName);
		
#ifdef _BUILD_FOR_ROS_
		topic[0] = nh.resolveName(configData.video_stream);
		
		ROS_INFO("Subscribing to topic (%s)", topic[0].c_str());
		
		image_transport::ImageTransport it(nh);
		
		camera_sub[0] = it.subscribeCamera(topic[0], 1, boost::bind(&calibratorNode::handle_camera, this, _1, _2, 0));
		
		if (configData.debugMode) {
			debug_pub[0] = it.advertiseCamera(debug_pub_name[0], 1);
			ROS_INFO("Advertising tracker debugging video (%s)", debug_pub_name[0]);
		}
#endif		
	} else if (configData.calibType == "stereo") {
		
		sprintf(debug_pub_name[0], "thermalvis/calib_left/image_col");
		sprintf(debug_pub_name[1], "thermalvis/calib_right/image_col");

#ifdef _BUILD_FOR_ROS_
		topic[0] = nh.resolveName(configData.video_stream);
		topic[1] = nh.resolveName(configData.video_stream_secondary);

		if (configData.verboseMode) { ROS_INFO("(cam 1): Subscribing to topic (%s)...", topic[0].c_str()); }
		if (configData.verboseMode) { ROS_INFO("(cam 2): Subscribing to topic (%s)...", topic[1].c_str()); }
		
		image_transport::ImageTransport it(nh);

		camera_sub[0] = it.subscribeCamera(topic[0], 1, boost::bind(&calibratorNode::handle_camera, this, _1, _2, 0));
		camera_sub[1] = it.subscribeCamera(topic[1], 1, boost::bind(&calibratorNode::handle_camera, this, _1, _2, 1));
				
		if (configData.verboseMode) { ROS_INFO("Subscribed to topics..."); }
		
		if (configData.debugMode) {
			debug_pub[0] = it.advertiseCamera(debug_pub_name[0], 1);
			debug_pub[1] = it.advertiseCamera(debug_pub_name[1], 1);
			ROS_INFO("Publishing debug topics at (%s) & (%s)", debug_pub_name[0], debug_pub_name[1]);
		}
#endif
	}
	
	// Pattern Corner Co-ordinates
    for (int iii = 0; iii < configData.yCount; iii++) {
        for (int jjj = 0; jjj < configData.xCount; jjj++) {
            row.push_back(Point3f(((float)iii)*float(configData.gridSize/1000.0), ((float)jjj)*float(configData.gridSize/1000.0), 0.0));
        }
    }

#ifdef _BUILD_FOR_ROS_
	ROS_INFO("Establishing server callback...");
	f = boost::bind (&calibratorNode::serverCallback, this, _1, _2);
    server.setCallback (f);
#endif
    
}

#ifdef _BUILD_FOR_ROS_
void calibratorNode::timerCallback(const ros::TimerEvent&) {
#else
void calibratorNode::loopCallback() {
#endif

	int totalFrameCount = 0;
	for (unsigned int iii = 0; iii < configData.numCams; iii++) {
		totalFrameCount += frameCount[iii];
	}
	if (totalFrameCount == 0) {
		vacantInputTime = ElapsedTimeMilliseconds(vacant_timer);
		elapsedInputTime = ElapsedTimeMilliseconds(elapsed_timer);
		return;
	}
	
	vacantInputTime = ElapsedTimeMilliseconds(vacant_timer, 0);
	elapsedInputTime = ElapsedTimeMilliseconds(elapsed_timer, 0);
	
	if ((configData.autoTimeout != 0.0) && (vacantInputTime > 1000.0*configData.autoTimeout)) {
		ROS_WARN("No new frames in (%f) seconds, terminating collection procedure..", vacantInputTime/1000.0);
		stillCollecting = false;
	}
	
	if ((elapsedInputTime/1000.0 > configData.maxTime) && (configData.maxTime > 0) ) {
		ROS_WARN("Maximum data collection time of (%f) seconds reached, terminating collection procedure..", configData.maxTime);
		stillCollecting = false;
	}
	
}

#ifdef _BUILD_FOR_ROS_
void calibratorNode::serverCallback(thermalvis::calibratorConfig &config, uint32_t level) {
#else
void calibratorNode::serverCallback(calibratorConfig &config) {
#endif

	configData.verboseMode = config.verboseMode;
	
	configData.drawGrids = config.drawGrids;
	
	configData.maxFrames = config.maxFrames;
	configData.maxPatterns = config.maxPatterns;
	configData.maxTime = config.maxTime;
	configData.maxCandidates = config.maxCandidates;
	configData.maxTests = config.maxTests;
	
	configData.maxInterpolationTimegap = config.maxInterpolationTimegap;
	configData.maxInterpolationMotion = config.maxInterpolationMotion;
	
	//configData.generateFigures = config.generateFigures;
	
	configData.autoTimeout = config.autoTimeout;
            
	if ((config.autoAlpha != configData.autoAlpha) || ((!config.autoAlpha) && (config.alpha != configData.alpha))) {
  
		configData.autoAlpha = config.autoAlpha;
		configData.alpha = config.alpha;
		
		alphaChanged = true;
		
		if (configData.verboseMode) { ROS_WARN("About to try and update maps..."); }
		if (readyForOutput) {
			if (configData.verboseMode) { ROS_WARN("Updating maps..."); }
			
			for (unsigned int iii = 0; iii < configData.numCams; iii++) {
				updateIntrinsicMap(iii);
			}
			
		}
		
	} else {
		configData.alpha = config.alpha;
	}
	
	configData.trackingMode = config.trackingMode;
	
	// ROS_WARN("Tracking mode has been set as (%d)", configData.trackingMode ? 1 : 0);
	
	configData.flowThreshold = config.flowThreshold;
	
	configData.maxFrac = config.maxFrac;

        //HGH
        configData.errorThreshold = config.errorThreshold;

        configData.stopCapturingAtLimit = config.stopCapturingAtLimit;

		/*
        configData.invert[0] = config.invertPrimary;
        configData.adjustMSER[0] = config.adjustMSER_primary;
        configData.delta[0] = config.delta_primary;
        configData.maxVar[0] = config.maxVar_primary;
        configData.minDiv[0] = config.minDiv_primary;
        configData.areaThreshold[0] = config.areaThreshold_primary;
        
        configData.invert[1] = config.invertSecondary;
        configData.adjustMSER[1] = config.adjustMSER_secondary;
        configData.maxVar[1] = config.maxVar_secondary;
        configData.delta[1] = config.delta_secondary;
		configData.areaThreshold[1] = config.areaThreshold_secondary;
		configData.minDiv[1] = config.minDiv_secondary;
		*/
}

void calibratorNode::set_ready_for_output() {
	readyForOutput = true;
}
