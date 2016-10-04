/*! \file	features.cpp
 *  \brief	Definitions for local feature detection, description and matching.
*/

#include "core/features.hpp"


double calculateFeatureSpeeds(const vector<cv::Point2f>& pts1, const vector<cv::Point2f>& pts2, vector<cv::Point2f>& velocities, double time1, double time2) {
	
	if ((pts1.size() != pts2.size()) || (pts1.size() == 0)) {
		printf("%s << ERROR! pts vectors are of sizes (%zu, %zu)\n", __FUNCTION__, pts1.size(), pts2.size());
		return 0.0;
	}
	
	double avSpeed = 0.0;
	
	for (unsigned int iii = 0; iii < pts1.size(); iii++) {
		
		float vel_x = float((pts2.at(iii).x - pts1.at(iii).x) / (time2 - time1));
		float vel_y = float((pts2.at(iii).y - pts1.at(iii).y) / (time2 - time1));
		
		velocities.push_back(cv::Point2f(vel_x, vel_y));
		avSpeed += pow((pow(vel_x, 2.0) + pow(vel_y, 2.0)), 0.5);
	}
	
	avSpeed /= float(pts1.size());
	
	return avSpeed;
	
}

double generateVirtualPointset(const vector<cv::Point2f>& pts1, const vector<cv::Point2f>& pts2, vector<cv::Point2f>& virtual_pts, double bias_fraction) {
	
	if (pts1.size() != pts2.size()) {
		return -1.0;
	}
	
	double totalMotion = 0.0;
	
	for (unsigned int iii = 0; iii < pts1.size(); iii++) {
		cv::Point2f vpt;
		
		totalMotion += pow(pow(pts1.at(iii).x - pts2.at(iii).x, 2.0)+pow(pts1.at(iii).y - pts2.at(iii).y, 2.0),0.5);
		
		vpt.x = float((1.0 - bias_fraction)*pts1.at(iii).x + (bias_fraction)*pts2.at(iii).x);
		vpt.y = float((1.0 - bias_fraction)*pts1.at(iii).y + (bias_fraction)*pts2.at(iii).y);
		
		virtual_pts.push_back(vpt);
		
	}
	
	totalMotion /= double(pts1.size());
	
	return totalMotion;
	
}

void crossCheckMatching( cv::Ptr<cv::DescriptorMatcher>& descriptorMatcher,
                         const cv::Mat& descriptors1, const cv::Mat& descriptors2,
                         vector<cv::DMatch>& filteredMatches12, int knn )
{

    //printf("%s << ENTERED.\n", __FUNCTION__);

    filteredMatches12.clear();

    //printf("%s << DEBUG %d.\n", __FUNCTION__, 0);
    //vector<vector<cv::DMatch> > matches12, matches21;
    vector<vector<cv::DMatch> > matches12, matches21;


    //printf("%s << DEBUG %d.\n", __FUNCTION__, 1);

    //printf("%s << descriptors1.size() = (%d, %d).\n", __FUNCTION__, descriptors1.rows, descriptors1.cols);
    //printf("%s << descriptors2.size() = (%d, %d).\n", __FUNCTION__, descriptors2.rows, descriptors2.cols);



    descriptorMatcher->knnMatch( descriptors1, descriptors2, matches12, knn );


    //printf("%s << DEBUG %d.\n", __FUNCTION__, 2);
    descriptorMatcher->knnMatch( descriptors2, descriptors1, matches21, knn );

    //printf("%s << matches12.size() = %d.\n", __FUNCTION__, matches12.size());

    for( size_t m = 0; m < matches12.size(); m++ )
    {
        bool findCrossCheck = false;

        //printf("%s << matches12[%d].size() = %d.\n", __FUNCTION__, m, matches12[m].size());

        for( size_t fk = 0; fk < matches12[m].size(); fk++ )
        {
            cv::DMatch forward = matches12[m][fk];

            //printf("%s << matches21[%d].size() = %d.\n", __FUNCTION__, forward.trainIdx, matches21[forward.trainIdx].size());

            for( size_t bk = 0; bk < matches21[forward.trainIdx].size(); bk++ )
            {
                cv::DMatch backward = matches21[forward.trainIdx][bk];
                if( backward.trainIdx == forward.queryIdx )
                {
                    filteredMatches12.push_back(forward);
                    findCrossCheck = true;
                    break;
                }
            }
            if( findCrossCheck ) break;
        }
    }

}

void proximityViolationFilter(vector<cv::KeyPoint>& kps, vector<cv::Point2f>& pts, double minSep) {
	
	int zzz = 0;
	while (zzz < int(kps.size())) {
		bool violatesProximity = false;
					
		for (unsigned int yyy = 0; yyy < pts.size(); yyy++) {
			if (distBetweenPts2f(kps.at(zzz).pt, pts.at(yyy)) < minSep) {
				violatesProximity = true;
				break;
			}
		}		
		if (violatesProximity) {
			kps.erase(kps.begin() + zzz);
		} else zzz++;
	}
}

void filterTrackingsByRadialProportion(vector<cv::Point2f>& pts1, vector<cv::Point2f>& pts2, cv::Mat& K, cv::Mat& newCamMat, cv::Mat& distCoeffs, cv::Size& imSize, double prop) {

	vector<cv::Point2f> newPts;
	redistortPoints(pts2, newPts, K, distCoeffs, newCamMat);
	
	cv::Point2f centrePt = cv::Point2f(float(imSize.width)/float(2.0),float(imSize.height)/float(2.0));
	double maxDist = prop * double(imSize.width)/2.0;
	
	double dist;
	unsigned int iii = 0;
	
	while (iii < pts1.size()) {
		
		dist = distBetweenPts2f(pts2.at(iii), centrePt);
		if (dist > maxDist) {
			pts1.erase(pts1.begin() + iii);
			pts2.erase(pts2.begin() + iii);
		} else iii++;
	}
}

void reduceEdgyFeatures(vector<cv::KeyPoint>& KeyPoints, cameraParameters& camData) {

	float minBorderDist = 2.0;
	vector<cv::Point2f> candidates, redistortedPoints;
	cv::KeyPoint::convert(KeyPoints, candidates);

	redistortPoints(candidates, redistortedPoints, camData.Kx, camData.distCoeffs, camData.expandedCamMat);

	float xDist, yDist, dist;
	for (int iii = int(candidates.size())-1; iii >= 0; iii--) {
		xDist = min(abs(((float) camData.expandedSize.width) - redistortedPoints.at(iii).x), abs(redistortedPoints.at(iii).x));
		yDist = min(abs(((float) camData.expandedSize.height) - redistortedPoints.at(iii).y), abs(redistortedPoints.at(iii).y));
		dist = min(xDist, yDist) - KeyPoints.at(iii).size;
		if (dist < minBorderDist) KeyPoints.erase(KeyPoints.begin() + iii);
	}
}

bool checkRoomForFeature(vector<cv::Point2f>& pts, cv::Point2f& candidate, double dist) {
	for (unsigned int iii = 0; iii < pts.size(); iii++) {
		if (distanceBetweenPoints(pts.at(iii), candidate) < dist) return false;
	}
	return true;
}

void reduceUnrefinedCandidates(vector<cv::Point2f>& candidates) {

	if (candidates.size() == 0) return;

	for (int iii = int(candidates.size())-1; iii >= 0; iii--) {

		if (((candidates.at(iii).x - floor(candidates.at(iii).x)) == 0.0) && ((candidates.at(iii).y - floor(candidates.at(iii).y)) == 0.0)) {
			printf("%s << erasing: (%f, %f)\n", __FUNCTION__, candidates.at(iii).x, candidates.at(iii).y);
			candidates.erase(candidates.begin() + iii);
		}
	}

}

void reduceProximalCandidates(vector<cv::Point2f>& existing, vector<cv::Point2f>& candidates) {

	for (unsigned int iii = 0; iii < candidates.size(); iii++) {

		bool isValid = checkRoomForFeature(existing, candidates.at(iii), 3.0);

		if (!isValid) {
			candidates.erase(candidates.begin() + iii);
			iii--;
		}
	}
}

void concatenateWithExistingPoints(vector<cv::Point2f>& pts, vector<cv::Point2f>& kps, int size, double min_sep, bool debug) {

	for (int iii = int(kps.size())-1; iii >= 0; iii--) {
		if (int(pts.size()) >= size) break;
		bool isValid = checkRoomForFeature(pts, kps.at(iii), min_sep);
		if (!isValid) { kps.erase(kps.begin()+iii); } else { pts.push_back(kps.at(iii)); }
	}
}

void initializeDrawingColors(cv::Scalar* kColors, cv::Scalar* tColors, int num) {

	for (int iii = 0; iii < num; iii++) {
		switch (iii) {
			case 0:
				tColors[iii] = cv::Scalar(255, 128, 128);
				kColors[iii] = cv::Scalar(255, 0, 0);
				break;
			case 1:
				tColors[iii] = cv::Scalar(128, 128, 255);
				kColors[iii] = cv::Scalar(0, 0, 255);
				break;
			case 2:
				tColors[iii] = cv::Scalar(128, 255, 128);
				kColors[iii] = cv::Scalar(0, 255, 0);
				break;
			case 3:
				tColors[iii] = cv::Scalar(128, 255, 255);
				kColors[iii] = cv::Scalar(0, 255, 255);
				break;
			case 4:
				tColors[iii] = cv::Scalar(128, 128, 255);
				kColors[iii] = cv::Scalar(255, 0, 255);
				break;
			case 5:
				tColors[iii] = cv::Scalar(255, 255, 128);
				kColors[iii] = cv::Scalar(255, 255, 0);
				break;
			default:
				tColors[iii] = cv::Scalar(128, 128, 255);
				kColors[iii] = cv::Scalar(255, 255, 255);
				break;

		}
	}

}

void transformPoints(vector<cv::Point2f>& pts1, cv::Mat& H) {
	cv::Mat ptCol(3, 1, CV_64FC1), newPos(3, 1, CV_64FC1);

	//vector<Point2f> tempPoints;
	//tempPoints.insert(tempPoints.end(), finishingPoints[kkk].begin(), finishingPoints[kkk].end());

	for (unsigned int mmm = 0; mmm < pts1.size(); mmm++) {

		ptCol.at<double>(0,0) = pts1.at(mmm).x;
		ptCol.at<double>(1,0) = pts1.at(mmm).y;
		ptCol.at<double>(2,0) = 1.0;

		newPos = H * ptCol;

		//printf("%s::%s << Changing point [%d][%d] from (%f, %f) to (%f, %f)\n", __PROGRAM__, __FUNCTION__, kkk, mmm, finishingPoints[kkk].at(mmm).x, finishingPoints[kkk].at(mmm).y, newPos.at<double>(0,0), newPos.at<double>(1,0));

		pts1.at(mmm).x = float(newPos.at<double>(0,0)/newPos.at<double>(2,0));
		pts1.at(mmm).y = float(newPos.at<double>(1,0)/newPos.at<double>(2,0));

	}
}

void reduceWeakFeatures(cv::Mat& im, vector<cv::KeyPoint>& feats, double minResponse) {

	vector<float> responseLevels;
	for (unsigned int iii = 0; iii < feats.size(); iii++) {

		if (feats.at(iii).response == 0.0) {
			feats.at(iii).response = float(estimateSalience(im, feats.at(iii).pt, ((double) feats.at(iii).size / 2.0)));
			responseLevels.push_back(feats.at(iii).response);
		}
	}

	sort(responseLevels.begin(), responseLevels.end());

	for (int iii = int(feats.size())-1; iii >= 0; iii--) {
		if (feats.at(iii).response < minResponse) feats.erase(feats.begin() + iii);
	}
}

int getOpticalFlowFlags() {
#ifdef _OPENCV_VERSION_3_PLUS_
	return cv::OPTFLOW_LK_GET_MIN_EIGENVALS;
#else
	return CV_LKFLOW_GET_MIN_EIGENVALS;
#endif
}

void trackPoints(const cv::Mat& im1, const cv::Mat& im2, vector<cv::Point2f>& pts1, vector<cv::Point2f>& pts2, int distanceConstraint, double thresh, vector<unsigned int>& lostIndices, cv::Mat H12, cameraParameters camData) {

	vector<cv::Point2f> originalGuides;
	bool debugFlag = false, guiding = false, warping = false;
	
	if (debugFlag) { printf("%s << ENTERED. im1 = (%u, %u); im2 = (%u, %u), pts1 = (%lu), pts2 = (%lu)\n", __FUNCTION__, im1.cols, im1.rows, im2.cols, im2.rows, (long unsigned int)(pts1.size()), (long unsigned int)(pts2.size())); }
	
	if (pts1.size() == 0) return;
	if ((distanceConstraint % 2) == 0) distanceConstraint++;
	
	cv::Size winSize = cv::Size(distanceConstraint, distanceConstraint);
	int maxLevel = 3;
	cv::TermCriteria criteria = cv::TermCriteria( cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01);
	
	int opticalFlowFlags = getOpticalFlowFlags();

	cv::Mat im1b, mask, blank;
	vector<cv::Point2f> pts1b;
	vector<uchar> statusVec;
	vector<float> err;

	blank = cv::Mat::ones(im1.size(), CV_8UC1);
	blank *= 255;
	
	vector<cv::Point2f> originalPts, originalEstimates;
	originalPts.insert(originalPts.end(), pts1.begin(), pts1.end());
	originalEstimates.insert(originalEstimates.end(), pts2.begin(), pts2.end());
	
	if (pts2.size() == pts1.size()) {
		if (debugFlag) { printf("%s << guiding is set to TRUE.\n", __FUNCTION__); }
		guiding = true;
		originalGuides.insert(originalGuides.end(), pts2.begin(), pts2.end());
		opticalFlowFlags += cv::OPTFLOW_USE_INITIAL_FLOW;
	} else if (debugFlag) printf("%s << guiding is maintained as FALSE.\n", __FUNCTION__);

	if ((H12.rows != 0) && (pts2.size() == pts1.size())) {
		warping = true;
		pts1b.insert(pts1b.end(), pts1.begin(), pts1.end());
		transformPoints(pts1b, H12);
		warpPerspective(im1, im1b, H12, im1.size());
		warpPerspective(blank, mask, H12, blank.size());
	} else {
		im1b = cv::Mat(im1); // .copyTo(im1b);
		if (!guiding) pts2.clear();
	}
	
	if (debugFlag) { printf("%s << Debug (%d)\n", __FUNCTION__, 5); }

	if (warping) {
		if (debugFlag) { printf("%s << (warped) Before. (%u, %u) (%u, %u) (%lu) (%u, %u)\n", __FUNCTION__, im1b.rows, im1b.cols, im2.rows, im2.cols, (long unsigned int)(pts1b.size()), winSize.height, winSize.width); }
		cv::calcOpticalFlowPyrLK(im1b, im2, pts1b, pts2, statusVec, err, winSize, maxLevel, criteria, opticalFlowFlags, thresh);
		checkDistances(originalEstimates, pts2, statusVec, (double) distanceConstraint);
		if (debugFlag) { printf("%s << After.\n", __FUNCTION__); }
		markAbsentPoints(pts1b, pts2, statusVec, im1.size());
	} else {
		if (debugFlag) { printf("%s << (not-warped) Before. (%u, %u) (%u, %u) (%lu) (%u, %u)\n", __FUNCTION__, im1b.rows, im1b.cols, im2.rows, im2.cols, (long unsigned int)(pts1.size()), winSize.height, winSize.width); }
		
		cv::calcOpticalFlowPyrLK(im1b, im2, pts1, pts2, statusVec, err, winSize, maxLevel, criteria, opticalFlowFlags, thresh);
		checkDistances(originalEstimates, pts2, statusVec, (double) distanceConstraint);
		if (debugFlag) { printf("%s << After.\n", __FUNCTION__); }

		if (debugFlag) {
			
			printf("%s << winSize = (%d, %d)\n", __FUNCTION__, winSize.width, winSize.height);
			cv::Mat a, b;
			
			displayKeyPoints(im2, pts1, a, cv::Scalar(255,0,0), 0);
			displayKeyPoints(im2, pts2, b, cv::Scalar(0,0,255), 0);
			
			if /*while*/ (1) {
				
				imshow("test1", a);
				cv::waitKey(1);
				imshow("test2", b);
				cv::waitKey(1);
				
				printf("%s << After flow: %d good out of / %lu\n", __FUNCTION__, cv::countNonZero(statusVec), (long unsigned int)(statusVec.size()));
			}
		}
		markAbsentPoints(pts1, pts2, statusVec, im1.size());
		if (debugFlag) printf("%s << After absent: %d\n", __FUNCTION__, cv::countNonZero(statusVec));
	}
	
	if (debugFlag) { printf("%s << Debug (%d)\n", __FUNCTION__, 6); }
	
	
	if (camData.Kx.rows != 0) markEdgyTracks(pts2, statusVec, camData);
	
	if (warping) {
		if (debugFlag) { printf("%s << W: pts1.size() = %zu [before filtering]\n", __FUNCTION__, pts1.size()); }
		if (debugFlag) { printf("%s << Debug (%d.%d) : (%zu, %zu)\n", __FUNCTION__, 7, 1, pts1.size(), pts2.size()); }
		
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
		filterVectors(pts1, pts2, statusVec, ((double) max(im1.rows, im1.cols)), DO_EPIPOLAR_CHECK);
#else
		filterVectors(pts1, pts2, statusVec, ((double) std::max(im1.rows, im1.cols)), DO_EPIPOLAR_CHECK);
#endif
		
		if (0) {

			cv::Mat im1x, im2x;
			//warpPerspective(grayImageBuffer[(current_idx-1) % MAXIMUM_FRAMES_TO_STORE], im2, H12, grayImageBuffer[(current_idx-1) % MAXIMUM_FRAMES_TO_STORE].size());

			im1.copyTo(im1x);
			im2.copyTo(im2x);

			displayKeyPoints(im1, pts1, im1x, cv::Scalar(255,0,0), 0);
			displayKeyPoints(im2x, pts2, im2x, cv::Scalar(255,0,0), 0);

			//warpPerspective(im1, im1b, H12, im1.size());

			while (1) {

				imshow("temp_disp", im1x);	// Previous image with features
				cv::waitKey(500);
				imshow("temp_disp", im2x);	// Current image
				cv::waitKey(500);

			}
		}
	} else {
		if (debugFlag) { printf("%s << pts1.size() = %zu [before filtering]\n", __FUNCTION__, pts1.size()); }
		//printf("%s << passed distanceConstraint = %d\n", __FUNCTION__, distanceConstraint);
		if (debugFlag) { printf("%s << Debug (%d.%d)\n", __FUNCTION__, 7, 3); }
		
		if (guiding) {
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
			filterVectors(pts1, pts2, statusVec, ((double) max(im1.rows, im1.cols)), DO_EPIPOLAR_CHECK);
#else
			filterVectors(pts1, pts2, statusVec, ((double) std::max(im1.rows, im1.cols)), DO_EPIPOLAR_CHECK);
#endif
			
		} else filterVectors(pts1, pts2, statusVec, ((double) distanceConstraint));
		
		if (0) { // (pts2.size() < 5) {

			cv::Mat im1x, im2x;
			//warpPerspective(grayImageBuffer[(current_idx-1) % MAXIMUM_FRAMES_TO_STORE], im2, H12, grayImageBuffer[(current_idx-1) % MAXIMUM_FRAMES_TO_STORE].size());

#ifdef _OPENCV_VERSION_3_PLUS_
			cvtColor(im1b, im1x, cv::COLOR_GRAY2RGB);
			cvtColor(im2, im2x, cv::COLOR_GRAY2RGB);
#else
			cvtColor(im1b, im1x, CV_GRAY2RGB);
			cvtColor(im2, im2x, CV_GRAY2RGB);
#endif
			
			displayKeyPoints(im1x, originalPts, im1x, cv::Scalar(255,0,0), 0);
			displayKeyPoints(im2x, pts2, im2x, cv::Scalar(255,0,0), 0);

			while (1) {
				imshow("temp_disp", im1b);	// Previous image with features
				cv::waitKey(1500);
				imshow("temp_disp", im2x);	// Current image
				cv::waitKey(500);
			}
		}

	}

	for (unsigned int iii = 0; iii < statusVec.size(); iii++) if (statusVec.at(iii) == 0) lostIndices.push_back(iii);
}

//HGH
void trackPoints2(const cv::Mat& im1, const cv::Mat& im2, vector<cv::Point2f>& pts1, vector<cv::Point2f>& pts2, int distanceConstraint, double thresh, vector<unsigned int>& lostIndices, const cv::Size patternSize, double& errorThreshold) {

        unsigned int corners = patternSize.width * patternSize.height;

        bool debugFlag = false;
        if (debugFlag) printf("%s << ENTERED!\n", __FUNCTION__);
        if (pts1.size() == 0) return;

        if ((distanceConstraint % 2) == 0) distanceConstraint++;

        cv::Size winSize = cv::Size(distanceConstraint, distanceConstraint);
        int maxLevel = 3;
        cv::TermCriteria criteria = cv::TermCriteria( cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01);

        cv::Mat im1b;
        cv::Mat blank;

        vector<uchar> statusVec;
        vector<float> err;

        blank = cv::Mat::ones(im1.size(), CV_8UC1);
        blank *= 255;


        vector<cv::Point2f> originalPts;
        originalPts.insert(originalPts.end(), pts1.begin(), pts1.end());

        im1b = cv::Mat(im1); // .copyTo(im1b);

        pts2.clear();
        pts2.insert(pts2.end(), pts1.begin(), pts1.end());

		#ifdef _OPENCV_VERSION_3_PLUS_
		int opticalFlowFlags = cv::OPTFLOW_USE_INITIAL_FLOW + cv::OPTFLOW_LK_GET_MIN_EIGENVALS; // + OPTFLOW_LK_GET_MIN_EIGENVALS;
		#else
		int opticalFlowFlags = cv::OPTFLOW_USE_INITIAL_FLOW + CV_LKFLOW_GET_MIN_EIGENVALS; // + OPTFLOW_LK_GET_MIN_EIGENVALS;
		#endif
        

		//printf("%s << Before. (%d, %d) (%d, %d) (%d) (%d, %d)\n", __FUNCTION__, im1b.rows, im1b.cols, im2.rows, im2.cols, pts1.size(), winSize.height, winSize.width);
        cv::calcOpticalFlowPyrLK(im1b, im2, pts1, pts2, statusVec, err, winSize, maxLevel, criteria, opticalFlowFlags, thresh);

        if (corners != statusVec.size() || corners != err.size()){
            lostIndices.push_back(-1);
        }

        //cout << *max_element(err.begin(),err.end()) << endl;
        for (unsigned int iii = 0; iii < err.size(); iii++) {

            if (err.at(iii) > errorThreshold && errorThreshold != 0.0){
                //cout << err.at(iii)  <<   " for  "  << iii << endl;
                lostIndices.push_back(-2);
            }
        }

        for (unsigned int iii = 0; iii < statusVec.size(); iii++) {
            if (statusVec.at(iii) == 0) {
                lostIndices.push_back(iii);
            }
        }


}

void markAbsentPoints(vector<cv::Point2f>&pts1, vector<cv::Point2f>&pts2, vector<uchar>&statusVec, cv::Size imSize) {

	float buffer = 2.0;

	#pragma omp parallel for
	for (unsigned int iii = 0; iii < statusVec.size(); iii++) {

		if (statusVec.at(iii) != 0) {

			if ((pts1.at(iii).x < buffer) || (pts1.at(iii).x > imSize.width - buffer)) {
				statusVec.at(iii) = 0;
			} else if ((pts2.at(iii).x < buffer) || (pts2.at(iii).x > imSize.width - buffer)) {
				statusVec.at(iii) = 0;
			} else if ((pts1.at(iii).y < buffer) || (pts1.at(iii).y > imSize.height - buffer)) {
				statusVec.at(iii) = 0;
			} else if ((pts2.at(iii).y < buffer) || (pts2.at(iii).y > imSize.height - buffer)) {
				statusVec.at(iii) = 0;
			}

		}

	}

}

bool constructPatch(cv::Mat& img, cv::Mat& patch, cv::Point2f& center, double radius, int cells) {

    //printf("%s << ENTERED.\n", __FUNCTION__);

	patch.release();

    if ((cells % 2) == 0) cells++;

    patch = cv::Mat::zeros(cells, cells, CV_64FC1);

    int range = (cells-1)/2;

	for (int iii = -range; iii <= +range; iii++) {
		for (int jjj = -range; jjj <= +range; jjj++) {

			cv::Point2f currPt = cv::Point2f(center.x + ((float) iii)*float(radius), center.y + ((float) jjj)*float(radius));

			//printf("%s << currPt = (%f, %f)\n", __FUNCTION__, currPt.x, currPt.y);

			if ((currPt.x <= 0.0) || (currPt.x >= ((double) img.cols)) || (currPt.y >= ((double) img.rows)) || (currPt.y <= 0.0)) return false;

            //printf("%s << Extracting (%f, %f) of (%d, %d)\n", __FUNCTION__, currPt.x, currPt.y, img.cols, img.rows);
			double val = getInterpolatedVal(img, currPt);
			//printf("%s << Assigning (%d, %d) of (%d, %d)\n", __FUNCTION__, iii+range, jjj+range, cells, cells);
			patch.at<double>(iii+range,jjj+range) = val;

		}
	}

	//printf("%s << EXITING.\n", __FUNCTION__);

	return true;

}


void extendKeyPoints(cv::Mat& img, vector<cv::KeyPoint>& pts, bool updateStrength, bool updateLocation) {

	if (updateLocation) {
		// Use cornersubpix to refine locations

		vector<cv::Point2f> candidates;
		cv::KeyPoint::convert(pts, candidates);

		#ifdef _OPENCV_VERSION_3_PLUS_
		cv::cornerSubPix(img, candidates, cv::Size(1,1), cv::Size(-1,-1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 15, 0.1));
		#else
		cv::cornerSubPix(img, candidates, cv::Size(1,1), cv::Size(-1,-1), cvTermCriteria(TERMCRIT_EPS+TERMCRIT_ITER, 15, 0.1));
		#endif

		#pragma omp parallel for
		for (unsigned int iii = 0; iii < candidates.size(); iii++) {
			pts.at(iii).pt = candidates.at(iii);
		}

	}

	if (updateStrength) {


		#pragma omp parallel for
		for (unsigned int iii = 0; iii < pts.size(); iii++) {
			double radius = ((double) pts.at(iii).size) / 2.0;
			double salience = estimateSalience(img, pts.at(iii).pt, radius);
			pts.at(iii).response = float(salience);
		}

	}

}

double estimateSalience(cv::Mat& img, cv::Point2f& center, double radius) {

	// Assumes 8-bit image
	double salience = 0.0;



	// Find center
	//cv::Point center = Point(((int) kp.pt.x), ((int) kp.pt.y));

	// Find radius


	if (((center.x - radius) < 0) || ((center.y - radius) < 0) || ((center.x + radius) >= img.cols) || ((center.y + radius) >= img.rows)) {
		return salience;
	}

	cv::Mat patch;
	int patchSize = int(radius * 2);
	if ((patchSize % 2) == 0) {
        //patchSize++;
	}

	constructPatch(img, patch, center, radius, patchSize);
	//cout << patch << endl;

	salience = getValueFromPatch(patch);


	return salience;

}

double getValueFromPatch(cv::Mat& patch) {

    cv::Mat convertedPatch = cv::Mat::zeros(patch.size(), CV_32FC1);

	#pragma omp parallel for
	for (int iii = 0; iii < patch.rows; iii++) {
		for (int jjj = 0; jjj < patch.cols; jjj++) {
			convertedPatch.at<float>(iii,jjj) = ((float) patch.at<double>(iii,jjj));
		}
	}

    cv::Mat eigenMat;
	
	#ifdef _OPENCV_VERSION_3_PLUS_
	cornerMinEigenVal(convertedPatch, eigenMat, patch.rows, cv::BORDER_REFLECT);
	#else
	cornerMinEigenVal(convertedPatch, eigenMat, patch.rows, CV_SCHARR);
	#endif

    double eigenValue = 0.0, blankVal;

    minMaxLoc(eigenMat, &blankVal, &eigenValue);

	return eigenValue;
}

void markUnrefinedPoints(vector<cv::Point2f>& pts, vector<uchar>&statusVec) {

	if (pts.size() == 0) return;

	#pragma omp parallel for
	for (unsigned int iii = 0; iii < statusVec.size(); iii++) {
		if (statusVec.at(iii) != 0) {
			if (((pts.at(iii).x - floor(pts.at(iii).x)) == 0.0) && ((pts.at(iii).y - floor(pts.at(iii).y)) == 0.0)) statusVec.at(iii) = 0;
		}
	}
}


void markBlandTracks(cv::Mat& img, vector<cv::Point2f>& pts, vector<uchar>& statusVec, double thresh) {

	if (pts.size() == 0) return;
	cv::Mat patch;
	for (unsigned int iii = 0; iii < statusVec.size(); iii++) {
		if (statusVec.at(iii) != 0) {
			constructPatch(img, patch, pts.at(iii), 1.5);
			double salience = getValueFromPatch(patch);
			if (salience < thresh) statusVec.at(iii) = 0;
		}
	}
}

void filterBlandKeyPoints(cv::Mat& img, vector<cv::KeyPoint>& pts, double thresh) {

	if (pts.size() == 0) return;
	for (int iii = int(pts.size())-1; iii >= 0; iii--) {
		if (pts.at(iii).response < thresh) pts.erase(pts.begin() + iii);
	}
}

void markStationaryPoints(vector<cv::Point2f>&pts1, vector<cv::Point2f>&pts2, vector<uchar>&statusVec) {

	#pragma omp parallel for
	for (unsigned int iii = 0; iii < statusVec.size(); iii++) {

		if (statusVec.at(iii) != 0) {

			double absDifference = pow(pow(pts1.at(iii).x - pts2.at(iii).x, 2.0) + pow(pts1.at(iii).y - pts2.at(iii).y, 2.0), 0.5);

			//printf("%s << absDifference = %f\n", __FUNCTION__, absDifference);

			if (absDifference == 0.0) {
				statusVec.at(iii) = 0;
			}

		}
	}

}

void displayKeyPoints(const cv::Mat& image, const vector<cv::KeyPoint>& KeyPoints, cv::Mat& outImg, const cv::Scalar& color, int thickness, bool pointsOnly) {

    bool randomColours = (color == cv::Scalar::all(-1));

    cv::Scalar newColour;
    if (!randomColours) newColour = color;
    image.copyTo(outImg);

    cv::Point centerPt;

    int radius, crossLength; //, aimedSize;

    if (thickness == 0) thickness = 1;

    // New method:
	for (unsigned int i = 0; i < KeyPoints.size(); i++) {
		//int colorCoeff = (int)( (255.0 * ((double) KeyPoints.size() - (double)i)) / (double) KeyPoints.size());

		//printf("%s << %d: c = %d\n", __FUNCTION__, i, colorCoeff);
		//cin.get();
        newColour = color; // cv::Scalar(255, 255-colorCoeff, 0);

        //printf("%s << DEBUG %d_%d\n", __FUNCTION__, 2, i);

        centerPt = KeyPoints.at(i).pt;

        centerPt.x = int(KeyPoints.at(i).pt.x * 16.0);
        centerPt.y = int(KeyPoints.at(i).pt.y * 16.0);

        //aimedSize = 6 - ((int) 4.0*((double) i / (double) pts.size()));


        //radius = KeyPoints.at(i).size/2;

        //aimedSize = 6 - ((int) 4.0*((double) i / (double) KeyPoints.size()));
        radius = int(16.0 * (KeyPoints.at(i).size/2.0)); // 2
        //thickness = 3 - ((int) 2.0*((double) i / (double) KeyPoints.size()));
        //crossLength = int (1.5 * (double) aimedSize);
        crossLength = int(2 * 16.0);
        
        if (pointsOnly) {
			#ifdef _OPENCV_VERSION_3_PLUS_
			circle(outImg, centerPt, 1, newColour, 2, cv::LINE_AA, 4);
			#else
			circle(outImg, centerPt, 1, newColour, 2, CV_AA, 4);
			#endif
		} else {
			 if (radius > 0) {
				#ifdef _OPENCV_VERSION_3_PLUS_
				circle(outImg, centerPt, radius, newColour, thickness, cv::LINE_AA, 4);
				#else
				circle(outImg, centerPt, radius, newColour, thickness, CV_AA, 4);
				#endif
			}

			#ifdef _OPENCV_VERSION_3_PLUS_
			line(outImg, cv::Point(centerPt.x-crossLength, centerPt.y), cv::Point(centerPt.x+crossLength, centerPt.y), newColour, thickness, cv::LINE_AA, 4);
			line(outImg, cv::Point(centerPt.x, centerPt.y-crossLength), cv::Point(centerPt.x, centerPt.y+crossLength), newColour, thickness, cv::LINE_AA, 4);
			#else
			line(outImg, cv::Point(centerPt.x-crossLength, centerPt.y), cv::Point(centerPt.x+crossLength, centerPt.y), newColour, thickness, CV_AA, 4);
			line(outImg, cv::Point(centerPt.x, centerPt.y-crossLength), cv::Point(centerPt.x, centerPt.y+crossLength), newColour, thickness, CV_AA, 4);
			#endif
			

		}
    }

    return;
}

void displayKeyPoints(const cv::Mat& image, const vector<cv::Point2f>& pts, cv::Mat& outImg, cv::Scalar color, int thickness, bool pointsOnly) {

    bool randomColours = (color == cv::Scalar::all(-1));

    cv::Scalar newColour;

    if (!randomColours)  newColour = color;

    image.copyTo(outImg);

    cv::Point centerPt;

    if (thickness == 0) thickness = 1;

    int crossLength; //, aimedSize;

    // New method:
	for (unsigned int i = 0; i < pts.size(); i++) {

        centerPt.x = int(pts.at(i).x * 16.0);
        centerPt.y = int(pts.at(i).y * 16.0);

        //aimedSize = 6 - ((int) 4.0*((double) i / (double) pts.size()));
        crossLength = int(2 * 16.0);

		if (pointsOnly) {
			#ifdef _OPENCV_VERSION_3_PLUS_
			circle(outImg, centerPt, 16, color, -1, cv::LINE_AA, 4);
			#else
			circle(outImg, centerPt, 16, color, -1, CV_AA, 4);
			#endif
		} else {
			#ifdef _OPENCV_VERSION_3_PLUS_
			line(outImg, cv::Point(centerPt.x-crossLength, centerPt.y), cv::Point(centerPt.x+crossLength, centerPt.y), color, thickness, cv::LINE_AA, 4);
			line(outImg, cv::Point(centerPt.x, centerPt.y-crossLength), cv::Point(centerPt.x, centerPt.y+crossLength), color, thickness, cv::LINE_AA, 4);
			#else
			line(outImg, cv::Point(centerPt.x-crossLength, centerPt.y), cv::Point(centerPt.x+crossLength, centerPt.y), color, thickness, CV_AA, 4);
			line(outImg, cv::Point(centerPt.x, centerPt.y-crossLength), cv::Point(centerPt.x, centerPt.y+crossLength), color, thickness, CV_AA, 4);
			#endif
			
		}

    }

    return;

}

bool KeyPoint_comparison(cv::KeyPoint i, cv::KeyPoint j) { 
	return (i.response < j.response); 
}

void sortMatches(vector<vector<cv::DMatch> >& matches1to2) {
	vector<vector<cv::DMatch> > matchesCpy, newMatches;

	if (matches1to2.size() <= 1) return;

	matchesCpy.assign(matches1to2.begin(), matches1to2.end());

	while (matchesCpy.size() > 0) {
		double bestDistance = matchesCpy.at(0).at(0).distance;
		int bestIndex = 0;

		for (unsigned int iii = 0; iii < matchesCpy.size(); iii++) {

			if (matchesCpy.at(iii).at(0).distance <= bestDistance) {
				bestDistance = matchesCpy.at(iii).at(0).distance;
				bestIndex = iii;
			}
		}

		newMatches.push_back(matchesCpy.at(bestIndex));
		matchesCpy.erase(matchesCpy.begin() + bestIndex);
	}

	newMatches.swap(matches1to2);
}

void filterMatches(vector<vector<cv::DMatch> >& matches1to2, double threshold) {

	if (matches1to2.size() == 0) return;

	int currIndex = int(matches1to2.size()) - 1;

	while (matches1to2.at(currIndex).at(0).distance > threshold) {
		currIndex--;

		if (currIndex == -1) {
			matches1to2.clear();
			return;
		}
	}

	if (currIndex < ((int) matches1to2.size() - 1)) matches1to2.erase(matches1to2.begin() + currIndex + 1, matches1to2.end());
}

double distanceBetweenPoints(const cv::Point2f& pt1, const cv::Point2f& pt2) {
	return pow(pow(pt1.x - pt2.x, 2.0) + pow(pt1.y - pt2.y, 2.0), 0.5);
}

double distanceBetweenPoints(const cv::KeyPoint& pt1, const cv::KeyPoint& pt2) {
	return pow(pow(pt1.pt.x - pt2.pt.x, 2.0) + pow(pt1.pt.y - pt2.pt.y, 2.0), 0.5);
}

void constrainMatchingMatrix(cv::Mat& matchingMatrix, vector<cv::KeyPoint>& kp1, vector<cv::KeyPoint>& kp2, int distanceConstraint, double sizeConstraint) {

	#pragma omp parallel for
	for (unsigned int iii = 0; iii < kp1.size(); iii++) {
		for (unsigned int jjj = 0; jjj < kp2.size(); jjj++) {
			// Check distance constraint
			
			int distanceBetween = (int) distanceBetweenPoints(kp1.at(iii), kp2.at(jjj));

			if (distanceBetween > distanceConstraint) matchingMatrix.at<double>(iii,jjj) = -1;

			if (sizeConstraint != 0.0) {
				if (max(kp1.at(iii).size/kp2.at(jjj).size, kp2.at(jjj).size/kp1.at(iii).size) > (1.00 + sizeConstraint)) matchingMatrix.at<double>(iii,jjj) = -1;
			}
		}
	}
}

void twoWayPriorityMatching(cv::Mat& matchingMatrix, vector<vector<cv::DMatch> >& bestMatches, int mode) {

	bestMatches.clear();

    // void getPriorityScores(Mat& matchingMatrix, vector<vector<double> >& priorityScores, vector<vector<int> >& priorityMatches);

    // Potentially more efficient way of doing things:
        // Just search matrix for lowest match score, and record the next best in that 'cross' and blank out that cross,
        // then search for the lowest match score remaining in the matrix and so on and so forth...
        //
        // Actually many-many, 1-many, 1-1 can all use this basic approach, but just with different "blanking" strategies
        // e.g. 1-many would blank out the entire column, but not row
        // only many-many would not be able to record a valid "second best" score
            // and many-many would also have N^2 matches rather than just N for the others...

	bool rowsStillRemain = false;
	int remainingRows = 0;

	for (int iii = 0; iii < matchingMatrix.rows; iii++) {
		bool anyInThisRow = false;
		for (int jjj = 0; jjj < matchingMatrix.cols; jjj++) {
			if (matchingMatrix.at<double>(iii,jjj) >= 0) {
				rowsStillRemain = true;
				anyInThisRow = true;
			}
		}

		if (anyInThisRow) remainingRows++;
	}

	while (rowsStillRemain) {

		double bestScore = std::numeric_limits<double>::max();
		int bestRow = -1, bestCol = -1;

		for (int iii = 0; iii < matchingMatrix.rows; iii++) {
			for (int jjj = 0; jjj < matchingMatrix.cols; jjj++) {
				if ((matchingMatrix.at<double>(iii,jjj) <= bestScore) && (matchingMatrix.at<double>(iii,jjj) >= 0)) {
					bestScore = matchingMatrix.at<double>(iii,jjj);
					bestRow = iii;
					bestCol = jjj;
				}
			}
		}

		if ((bestScore < 0) || (bestScore == std::numeric_limits<double>::max())) {
			rowsStillRemain = false;
			break;
		}

		matchingMatrix.at<double>(bestRow,bestCol) = -1.0;

		cv::DMatch currentMatch;
		vector<cv::DMatch> currentMatchVector;

		currentMatch.queryIdx = bestRow;
		currentMatch.trainIdx = bestCol;

		double secondScore = std::numeric_limits<double>::max();

		for (int iii = 0; iii < matchingMatrix.rows; iii++) {
			if ((matchingMatrix.at<double>(iii,bestCol) <= secondScore) && (matchingMatrix.at<double>(iii,bestCol) >= 0)) {
				secondScore = matchingMatrix.at<double>(iii,bestCol);
			}
			matchingMatrix.at<double>(iii,bestCol) = -1;
		}

		for (int iii = 0; iii < matchingMatrix.cols; iii++) {
			if ((matchingMatrix.at<double>(bestRow,iii) <= secondScore) && (matchingMatrix.at<double>(bestRow,iii) >= 0)) {
				secondScore = matchingMatrix.at<double>(bestRow,iii);
			}
			matchingMatrix.at<double>(bestRow,iii) = -1;
		}

		// If it is literally the last match available, it won't have a second best score...
		if (secondScore == std::numeric_limits<double>::max()) secondScore = bestScore;

		// Then normalize the "distance" based on this ratio, or can just use the ratio...
		switch (mode) {
		case MATCHING_MODE_NN:
			currentMatch.distance = float(bestScore);
			break;
		case MATCHING_MODE_NNDR:
			currentMatch.distance = float(bestScore / secondScore);
			break;
		case MATCHING_MODE_SVM:
			currentMatch.distance = float(reweightDistanceWithLinearSVM(bestScore, (bestScore/secondScore), DEFAULT_SVM_GRADIENT));
			break;
		default:
			currentMatch.distance = float(bestScore);
		}
		
		currentMatchVector.push_back(currentMatch);
		bestMatches.push_back(currentMatchVector);
	}
}

double calcDistance(double dist, double ratio, double *coeffs) {
	return abs(coeffs[0] * dist - ratio + coeffs[2]) / sqrt(pow(coeffs[0], 2) + 1);
}

double reweightDistanceWithLinearSVM(double dist, double ratio, double gradient) {
    return dist - (1.0 / gradient) * ratio;
}

void reduceFeaturesToMaximum(vector<cv::KeyPoint>& keypoints, int maximumToKeep) {

	if (int(keypoints.size()) <= maximumToKeep) return;

	// Determine number of points at lowest error
	while (int(keypoints.size()) > maximumToKeep) {
		float weakestScore = keypoints.at(0).response;
		int weakestCount = 0;
		for (int iii = 0; iii < int(keypoints.size()); iii++) {
			if (keypoints.at(iii).response == weakestScore) { 
				weakestCount++; 
			} else break;
		}

		if ((int(keypoints.size()) - weakestCount) > maximumToKeep) {
			keypoints.erase(keypoints.begin(), keypoints.begin() + weakestCount);
		} else {
			InitializeRandomNumberGeneration();
			while (int(keypoints.size()) > maximumToKeep) {
				int randIndex = rand() % (keypoints.size() - maximumToKeep);
				keypoints.erase(keypoints.begin() + randIndex);
			}
		}

	}
	
}

void createMatchingMatrix(cv::Mat& matchingMatrix, const cv::Mat& desc1, const cv::Mat& desc2) {

    vector<vector<cv::DMatch> > matches1to2, matches2to1;
	cv::Ptr<cv::DescriptorMatcher> dMatcher;

	if (desc1.type() == CV_8UC1) {
		dMatcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
	} else if ((desc1.type() == CV_64FC1) || (desc1.type() == CV_32FC1)) {
		dMatcher = cv::DescriptorMatcher::create("BruteForce");
	} else {
		printf("%s << ERROR! desc1.type() (%d) unrecognized.\n", __FUNCTION__, desc1.type());
	}

	dMatcher->knnMatch( desc1, desc2, matches1to2, desc1.rows );
	dMatcher->knnMatch( desc2, desc1, matches2to1, desc2.rows );

    matchingMatrix = cv::Mat::zeros(int(matches1to2.size()), int(matches2to1.size()), CV_64FC1);

    cv::Mat countMat = cv::Mat::zeros(int(matches1to2.size()), int(matches2to1.size()), CV_64FC1);

    // IM 1 to IM 2
    for (unsigned int iii = 0; iii < matches1to2.size(); iii++) {
        for (unsigned int jjj = 0; jjj < matches1to2[iii].size(); jjj++) {
            matchingMatrix.at<double>(iii, matches1to2.at(iii).at(jjj).trainIdx) += matches1to2.at(iii).at(jjj).distance;
            countMat.at<double>(iii, matches1to2.at(iii).at(jjj).trainIdx) += 1.0;
        }
    }

    // IM 2 to IM 1
    for (unsigned int iii = 0; iii < matches2to1.size(); iii++) {
        for (unsigned int jjj = 0; jjj < matches2to1[iii].size(); jjj++) {
            matchingMatrix.at<double>(matches2to1.at(iii).at(jjj).trainIdx, iii) += matches2to1.at(iii).at(jjj).distance;
            countMat.at<double>(matches2to1.at(iii).at(jjj).trainIdx, iii) += 1.0;
        }
    }

	/*
	cv::Mat matMatDisp = normForDisplay(matchingMatrix);
    cv::imshow("matchingMatrix", matMatDisp);
    cv::waitKey();

    Mat matMatDisp = normForDisplay(countMat);
    imshow("countMat", matMatDisp);
    cv::waitKey();
    */

    matchingMatrix /= 2.0;

}

void filterTrackedPoints(vector<uchar>& statusVec, vector<float>& err, double maxError) {
	for (unsigned int iii = 0; iii < statusVec.size(); iii++) {
		if (err.at(iii) > maxError) statusVec.at(iii) = 0;
	}
}

void checkDistances(vector<cv::Point2f>& pts1, vector<cv::Point2f>& pts2, vector<uchar>& statusVec, double distanceConstraint) {
	
	double totalDist, xDist, yDist;
	
	for (unsigned int iii = 0; iii < pts1.size(); iii++) {
		xDist = pts1.at(iii).x - pts2.at(iii).x;
		yDist = pts1.at(iii).y - pts2.at(iii).y;
		totalDist = pow((pow((xDist), 2.0) + pow((yDist), 2.0)), 0.5);

		if (totalDist > distanceConstraint) statusVec.at(iii) = 0;
	}
}

void filterVectors(vector<cv::Point2f>& pts1, vector<cv::Point2f>& pts2, vector<uchar>& statusVec, double distanceConstraint, bool epipolarCheck) {

	bool debugFlag = false;
	//printf("%s << sizes = (%d, %d)\n", __FUNCTION__, pts1.size(), pts2.size());

	double totalDist, xDist, yDist;

	if (debugFlag) { printf("%s << Debug (%d)\n", __FUNCTION__, 1); }

	for (int iii = int(statusVec.size())-1; iii >= 0; iii--) {
		if (statusVec.at(iii) == 0) {
			pts1.erase(pts1.begin() + iii);
			pts2.erase(pts2.begin() + iii);
		} else {
			// distance check
			xDist = pts1.at(iii).x - pts2.at(iii).x;
			yDist = pts1.at(iii).y - pts2.at(iii).y;
			totalDist = pow((pow((xDist), 2.0) + pow((yDist), 2.0)), 0.5);

			//printf("%s << totalDist (%f) ([%f, %f]) vs distanceConstraint (%f)\n", __FUNCTION__, totalDist, xDist, yDist, distanceConstraint);

			if (totalDist > distanceConstraint) {
				pts1.erase(pts1.begin() + iii);
				pts2.erase(pts2.begin() + iii);
			}
		}
	}
	
	if (debugFlag) { printf("%s << Debug (%d)\n", __FUNCTION__, 2); }

	//printf("%s << sizes2 = (%d, %d)\n", __FUNCTION__, pts1.size(), pts2.size());

	// Then some kind of epipolar geometry check:

	if ((epipolarCheck) && (pts1.size() > 0) && (pts2.size() > 0)) {
		cv::Mat F, matchesMask_F_matrix;
		
		if ((pts1.size() > 4) && (pts2.size() > 4)) {
			F = findFundamentalMat(cv::Mat(pts1), cv::Mat(pts2), cv::FM_RANSAC, 1.00, 0.99, matchesMask_F_matrix);
		if (debugFlag) { printf("%s << Found.\n", __FUNCTION__); }
		
			for (int iii = matchesMask_F_matrix.rows-1; iii >= 0; iii--) {
				//printf("%s << matchesMask_F_matrix(%d,0) = %d\n", __FUNCTION__, iii, matchesMask_F_matrix.at<unsigned char>(iii,0));
				if (matchesMask_F_matrix.at<unsigned char>(iii,0) == 0) {
					pts1.erase(pts1.begin() + iii);
					pts2.erase(pts2.begin() + iii);
				}
			}
		} else {
			pts1.clear();
			pts2.clear();
		}
	}
}

bool checkSufficientFeatureSpread(vector<cv::Point2f>& pts, cv::Size matSize, int minFeaturesInImage) {

	cv::Mat countMat;
	int horizontalDivisions = 4, verticalDivisions = 3;
	countMat = cv::Mat::zeros(3, 4, CV_16UC1);
	int minPerSegment = minFeaturesInImage / (8 * horizontalDivisions * verticalDivisions);

	//printf("%s << minPerSegment = %d\n", __FUNCTION__, minPerSegment);

	for (unsigned int iii = 0; iii < pts.size(); iii++) {
		int r, c;
		r = int( (float(horizontalDivisions) * pts.at(iii).x) / float(matSize.width) );
		c = int( (float(verticalDivisions) * pts.at(iii).y) / float(matSize.height) );
		countMat.at<unsigned short>(r, c) += 1;
	}

	int badSegments = 0;

	for (int iii = 0; iii < countMat.rows; iii++) {
		for (int jjj = 0; jjj < countMat.cols; jjj++) {
			if (countMat.at<unsigned short>(iii, jjj) < minPerSegment) {
				badSegments++;
			}
		}
	}

	//printf("%s << badSegments = %d\n", __FUNCTION__, badSegments);

	if (badSegments > (horizontalDivisions * verticalDivisions / 2)) return false;
	return true;
}

void drawMatchPaths(cv::Mat& src, cv::Mat& dst, vector<cv::Point2f>& kp1, vector<cv::Point2f>& kp2, const cv::Scalar& color) {

	src.copyTo(dst);
	cv::Point a, b;

	for (unsigned int iii = 0; iii < kp1.size(); iii++) {

		a = cv::Point(int(kp1.at(iii).x * 16.0), int(kp1.at(iii).y * 16.0));
		b = cv::Point(int(kp2.at(iii).x * 16.0), int(kp2.at(iii).y * 16.0));

		#ifdef _OPENCV_VERSION_3_PLUS_
		cv::line(dst, a, b, color, 1, cv::LINE_AA, 4);
		#else
		cv::line(dst, a, b, color, 1, CV_AA, 4);
		#endif
	}
}

void drawMatchPaths(cv::Mat& src, cv::Mat& dst, vector<cv::KeyPoint>& kp1, vector<cv::KeyPoint>& kp2, vector<vector<cv::DMatch> >& matches1to2) {

	cv::Point a, b;

	for (unsigned int iii = 0; iii < matches1to2.size(); iii++) {
		for (unsigned int jjj = 0; jjj < matches1to2.at(iii).size(); jjj++) {

			//printf("%s << (%f,%f) vs (%f,%f)\n", __FUNCTION__, kp1.at(iii).pt.x, kp1.at(iii).pt.y, kp2.at(iii).pt.x, kp2.at(iii).pt.y);

			//printf("%s << (%d, %d) - (%d, %d) / %f\n", __FUNCTION__, iii, jjj, matches1to2.at(iii).at(jjj).trainIdx, matches1to2.at(iii).at(jjj).queryIdx, matches1to2.at(iii).at(jjj).distance);
			//cin.get();
			// matches1to2.at(iii).at(jjj).trainIdx

			a = cv::Point(int(kp1.at(matches1to2.at(iii).at(jjj).queryIdx).pt.x * 16.0), int(kp1.at(matches1to2.at(iii).at(jjj).queryIdx).pt.y * 16.0));
			b = cv::Point(int(kp2.at(matches1to2.at(iii).at(jjj).trainIdx).pt.x * 16.0), int(kp2.at(matches1to2.at(iii).at(jjj).trainIdx).pt.y * 16.0));

			#ifdef _OPENCV_VERSION_3_PLUS_
			line(dst, a, b, cv::Scalar(0,0,255), 1, cv::LINE_AA, 4);
			circle(dst, b, 1, cv::Scalar(255, 0, 0), 1, cv::LINE_AA, 4);
			#else
			line(dst, a, b, cv::Scalar(0,0,255), 1, CV_AA, 4);
			circle(dst, b, 1, cv::Scalar(255, 0, 0), 1, CV_AA, 4);
			#endif
			
		}
	}
}

void markEdgyTracks(vector<cv::Point2f>& pts, vector<uchar>& statusVec, cameraParameters& camData) {

	vector<cv::Point2f> rpts;

	float minBorderDist = 15.0;
	redistortPoints(pts, rpts, camData.Kx, camData.distCoeffs, camData.expandedCamMat);

	cv::Mat tmpDisp;

	tmpDisp = cv::Mat::zeros(camData.expandedSize, CV_8UC3);

	cv::Mat testMat;
	displayKeyPoints(tmpDisp, rpts, tmpDisp, cv::Scalar(0,255,0), 0);

	unsigned int edgyMarks = 0;

	float xDist, yDist, dist;

	for (int iii = int(pts.size())-1; iii >= 0; iii--) {

		xDist = min(((float) camData.expandedSize.width) - rpts.at(iii).x, rpts.at(iii).x);
		yDist = min(((float) camData.expandedSize.height) - rpts.at(iii).y, rpts.at(iii).y);
		dist = min(xDist, yDist);

		if (dist < minBorderDist) {

			if (statusVec.at(iii) != 0) edgyMarks++;
			statusVec.at(iii) = 0;
			rpts.erase(rpts.begin() + iii);
		}

	}

	displayKeyPoints(tmpDisp, rpts, tmpDisp, cv::Scalar(0,0,255), 0);

}

double estimateStability(cv::Mat& img, cv::Point2f& center, double radius) {
    double stability = 0.0;

    if (((center.x - radius) < 0) || ((center.y - radius) < 0) || ((center.x + radius) >= img.cols) || ((center.y + radius) >= img.rows)) return stability;

    cv::Mat patch;
	constructPatch(img, patch, center, radius, 15);

    double minVal, maxVal;
    minMaxLoc(patch, &minVal, &maxVal);

	return (1.0 / getPatchVariance(patch));
}

double getPatchVariance(const cv::Mat& patch) {
    double mean = 0.0;
	double sigma = 0.0;

	for (int iii = 0; iii < patch.rows; iii++) {
		for (int jjj = 0; jjj < patch.cols; jjj++) {
			mean += patch.at<double>(iii,jjj) / ((double) (patch.rows * patch.cols));
		}
	}

	for (int iii = 0; iii < patch.rows; iii++) {
		for (int jjj = 0; jjj < patch.cols; jjj++) {
			//printf("%s << (%f vs %f)\n", __FUNCTION__, patch.at<double>(iii,jjj), mean);
			sigma += pow((patch.at<double>(iii,jjj) - mean), 2.0) / ((double) (patch.rows * patch.cols));
		}
	}

	sigma = pow(sigma, 0.5);

	return sigma;
}
