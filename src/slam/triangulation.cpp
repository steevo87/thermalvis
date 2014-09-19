/*! \file	triangulation.cpp
 *  \brief	Definitions for triangulation.
*/

#include "slam/triangulation.hpp"

void triangulateTracks(vector<featureTrack>& tracks, vector<unsigned int>& indices, cameraParameters& cameraData, cv::Mat *cameras, unsigned int earliest_index, unsigned int latest_index) {
	
	unsigned int minPairs = MIN_PAIRS_OF_PROJECTIONS_FOR_TRIANGULATION;
	
	// Should first check through camera pairs to see which ones can achive valid triangulations:
	
	unsigned int pair_width = latest_index - earliest_index + 1;
	cv::Mat validFramePairs = cv::Mat::zeros(pair_width, pair_width, CV_8UC1);
	
	double translations[3];
	
	for (unsigned int iii = earliest_index; iii < latest_index; iii++) {
		for (unsigned int jjj = iii+1; jjj <= latest_index; jjj++) {
			if (cameras[iii].rows != 4) continue;
			if (cameras[jjj].rows != 4) continue;
			
			getTranslationBetweenCameras(cameras[iii], cameras[jjj], translations);
			
			// Conditions for validity as a camera pair
			if ((abs(translations[0] > 0.2)) || (abs(translations[1] > 0.2))) {	//  || (abs(translations[2] > 1.0))
				validFramePairs.at<unsigned char>(iii-earliest_index,jjj-earliest_index) = 1;
			}
		}
	}
		
	unsigned int validCount = countNonZero(validFramePairs);
	
	if (validCount < minPairs) return;
	
	vector<cv::Point3d> estimatedLocations;
	
	cv::Point3d pt3d(0.0, 0.0, 0.0), mean3d(0.0, 0.0, 0.0), stddev3d(0.0, 0.0, 0.0);
	
	for (unsigned int iii = 0; iii < indices.size(); iii++) {
		if (tracks.size() <= indices.at(iii)) return;
		if (tracks.at(indices.at(iii)).locations.size() < 2) continue;
		if (tracks.at(indices.at(iii)).isTriangulated) continue;
		
		estimatedLocations.clear();
			
		pt3d = cv::Point3d(0.0, 0.0, 0.0);
		mean3d = cv::Point3d(0.0, 0.0, 0.0);
		stddev3d = cv::Point3d(0.0, 0.0, 0.0);
		
		unsigned int rrr, sss;
		
		for (unsigned int jjj = 0; jjj < tracks.at(indices.at(iii)).locations.size()-1; jjj++) {
			
			rrr = tracks.at(indices.at(iii)).locations.at(jjj).imageIndex;
			
			if ((rrr >= earliest_index) && (rrr <= latest_index)) {
				
				if (cameras[rrr].rows != 4) continue;
				
				cv::Mat temp_C0;
				cameras[rrr].copyTo(temp_C0);
				
				for (unsigned int kkk = jjj+1; kkk < tracks.at(indices.at(iii)).locations.size(); kkk++) {
					
					sss = tracks.at(indices.at(iii)).locations.at(kkk).imageIndex;
										
					if ((sss >= earliest_index) && (sss <= latest_index)) {
						
						if (validFramePairs.at<unsigned char>(rrr-earliest_index,sss-earliest_index) == 0) continue;
						if (cameras[sss].rows != 4) continue;
						
						cv::Mat temp_C1;
						cameras[sss].copyTo(temp_C1);
						
						cv::Point2f pt1_, pt2_;
						cv::Point3d pt3d_;
						
						pt1_ = tracks.at(indices.at(iii)).getCoord(rrr);
						pt2_ = tracks.at(indices.at(iii)).getCoord(sss);
						
						Triangulate(pt1_, pt2_, cameraData.K, cameraData.K_inv,	temp_C0, temp_C1, pt3d_, false);
						
						estimatedLocations.push_back(pt3d_);
					}
				}
			}
			
		}
		
		if (estimatedLocations.size() >= minPairs) {

			for (unsigned int qqq = 0; qqq < estimatedLocations.size(); qqq++) {
				mean3d.x += (estimatedLocations.at(qqq).x / ((double) estimatedLocations.size()));
				mean3d.y += (estimatedLocations.at(qqq).y / ((double) estimatedLocations.size()));
				mean3d.z += (estimatedLocations.at(qqq).z / ((double) estimatedLocations.size()));
			}
			
			for (unsigned int qqq = 0; qqq < estimatedLocations.size(); qqq++) {
				stddev3d.x += (pow((estimatedLocations.at(qqq).x - mean3d.x), 2.0) / ((double) estimatedLocations.size()));
				stddev3d.y += (pow((estimatedLocations.at(qqq).y - mean3d.y), 2.0) / ((double) estimatedLocations.size()));
				stddev3d.z += (pow((estimatedLocations.at(qqq).z - mean3d.z), 2.0) / ((double) estimatedLocations.size()));
			}
			
			stddev3d.x = pow(stddev3d.x, 0.5);
			stddev3d.y = pow(stddev3d.y, 0.5);
			stddev3d.z = pow(stddev3d.z, 0.5);
			
			for (int qqq = estimatedLocations.size()-1; qqq >= 0; qqq--) {
				
				double abs_diff_x = abs(estimatedLocations.at(qqq).x - mean3d.x);
				double abs_diff_y = abs(estimatedLocations.at(qqq).y - mean3d.y); 
				double abs_diff_z = abs(estimatedLocations.at(qqq).z - mean3d.z); 
				
				if ((abs_diff_x > 2*stddev3d.x) || (abs_diff_y > 2*stddev3d.y) || (abs_diff_z > 2*stddev3d.z)) {
					estimatedLocations.erase(estimatedLocations.begin() + qqq);
				}
			}

			if (estimatedLocations.size() >= minPairs) {
				for (unsigned int qqq = 0; qqq < estimatedLocations.size(); qqq++) {

					pt3d.x += (estimatedLocations.at(qqq).x / ((double) estimatedLocations.size()));
					pt3d.y += (estimatedLocations.at(qqq).y / ((double) estimatedLocations.size()));
					pt3d.z += (estimatedLocations.at(qqq).z / ((double) estimatedLocations.size()));
					
				}

				tracks.at(indices.at(iii)).set3dLoc(pt3d);
			}
		}
	}
}

unsigned int putativelyTriangulateNewTracks(vector<featureTrack>& tracks, vector<unsigned int>& indices, cameraParameters& cameraData, cv::Mat *cameras, unsigned int earliest_index, unsigned int latest_index) {
	
	//printf("%s << Entered: (%d, %d, %d, %d)\n", __FUNCTION__, tracks.size(), indices.size(), earliest_index, latest_index);
	
	unsigned int totalTriangulatedPoints = 0;
	
	for (unsigned int iii = 0; iii < indices.size(); iii++) {
		
		//printf("%s << iii = %d\n", __FUNCTION__, iii);
		
		if (tracks.size() < indices.at(iii)) {
			continue;
		}
		
		if (tracks.at(indices.at(iii)).locations.size() < 2) {
			continue;
		}
		
		for (unsigned int jjj = 0; jjj < tracks.at(indices.at(iii)).locations.size()-1; jjj++) {
			
			//printf("%s << jjj = %d\n", __FUNCTION__, jjj);
			
			if (tracks.at(indices.at(iii)).isTriangulated) {
				continue;
			}
			
			//printf("%s << jjj (continuing)\n", __FUNCTION__, jjj);
			
			unsigned int contribCount = 0;
			cv::Point3d pt3d(0.0, 0.0, 0.0);
			
			unsigned int rrr = tracks.at(indices.at(iii)).locations.at(jjj).imageIndex;
			
			if (cameras[rrr].rows != 4) {
				continue;
			}
			
			if ((rrr >= earliest_index) && (rrr <= latest_index)) {
				
				//printf("%s << rrr = %d\n", __FUNCTION__, rrr);
				
				cv::Mat temp_C0;
				cameras[rrr].copyTo(temp_C0);
				
				for (unsigned int kkk = jjj+1; kkk < tracks.at(indices.at(iii)).locations.size(); kkk++) {
					
					unsigned int sss = tracks.at(indices.at(iii)).locations.at(kkk).imageIndex;
					
					if (cameras[sss].rows != 4) {
						continue;
					}
					
					//printf("%s << sss = %d\n", __FUNCTION__, sss);
					
					if ((sss >= earliest_index) && (sss <= latest_index)) {
						
						//printf("%s << sss = %d\n", __FUNCTION__, sss);
						
						cv::Mat temp_C1;
						cameras[sss].copyTo(temp_C1);
						
						// This should be a valid pair of projections to triangulate from for this point..
						
						cv::Point2f pt1_, pt2_;
						cv::Point3d pt3d_;
						
						pt1_ = tracks.at(indices.at(iii)).getCoord(rrr);
						pt2_ = tracks.at(indices.at(iii)).getCoord(sss);
						
						//printf("%s << Triangulating...\n", __FUNCTION__);
						//printf("%s << pts = (%f, %f) & (%f, %f)\n", __FUNCTION__, pt1_.x, pt1_.y, pt2_.x, pt2_.y);
						//cout << cameraData.K << endl;
						//cout << cameraData.K_inv << endl;
						//cout << temp_C1 << endl;
						//cout << temp_C0 << endl;
						
						Triangulate(pt1_, pt2_, cameraData.K, cameraData.K_inv,	temp_C0, temp_C1, pt3d_, false);
						
						//if ((pointIsInFront(temp_C0, pt3d_)) && (pointIsInFront(temp_C1, pt3d_))) {
							pt3d.x += pt3d_.x;
							pt3d.y += pt3d_.y;
							pt3d.z += pt3d_.z;
							contribCount++;
						//}
					}
				}
			}
			
			if (contribCount > 0) {
				
				//printf("%s << Assigning (%d)\n", __FUNCTION__, indices.at(iii));
				
				pt3d.x /= ((double) contribCount); 
				pt3d.y /= ((double) contribCount);
				pt3d.z /= ((double) contribCount);
				
				tracks.at(indices.at(iii)).set3dLoc(pt3d);
			}

			
			
		}
		
	}
	
	for (unsigned int iii = 0; iii < tracks.size(); iii++) {
		if (tracks.at(iii).isTriangulated) {
			totalTriangulatedPoints++;
		}
	}
	
	return totalTriangulatedPoints;
	
}

unsigned int addNewPoints(vector<featureTrack>& tracks, vector<unsigned int>& indices, cameraParameters& cameraData, cv::Mat *cameras, unsigned int earliest_index, unsigned int latest_index) {

	unsigned int totalTriangulatedPoints = 0;
	
	for (unsigned int iii = 0; iii < indices.size(); iii++) {
		if (tracks.size() < indices.at(iii)) continue;
		if (tracks.at(indices.at(iii)).locations.size() < 2) continue;
		
		for (unsigned int jjj = 0; jjj < tracks.at(indices.at(iii)).locations.size()-1; jjj++) {
			
			if (tracks.at(indices.at(iii)).isTriangulated) continue;
			
			unsigned int contribCount = 0;
			cv::Point3d pt3d(0.0, 0.0, 0.0);
			
			unsigned int rrr = tracks.at(indices.at(iii)).locations.at(jjj).imageIndex;
			
			if (rrr == earliest_index) {
			
				if (cameras[rrr].rows != 4) continue;
				
				cv::Mat temp_C0;
				cameras[rrr].copyTo(temp_C0);
				
				for (unsigned int kkk = jjj+1; kkk < tracks.at(indices.at(iii)).locations.size(); kkk++) {
					
					unsigned int sss = tracks.at(indices.at(iii)).locations.at(kkk).imageIndex;
					
					if (sss == latest_index) {
					
						if (cameras[sss].rows != 4) continue;
						
						cv::Mat temp_C1;
						cameras[sss].copyTo(temp_C1);
						
						// This should be a valid pair of projections to triangulate from for this point..
						
						cv::Point2f pt1_, pt2_;
						cv::Point3d pt3d_;
						
						pt1_ = tracks.at(indices.at(iii)).getCoord(rrr);
						pt2_ = tracks.at(indices.at(iii)).getCoord(sss);
						
						//printf("%s << Triangulating...\n", __FUNCTION__);
						//printf("%s << pts = (%f, %f) & (%f, %f)\n", __FUNCTION__, pt1_.x, pt1_.y, pt2_.x, pt2_.y);
						//cout << cameraData.K << endl;
						//cout << cameraData.K_inv << endl;
						//cout << temp_C1 << endl;
						//cout << temp_C0 << endl;
						
						Triangulate(pt1_, pt2_, cameraData.K, cameraData.K_inv,	temp_C0, temp_C1, pt3d_, false);
						
						//if ((pointIsInFront(temp_C0, pt3d_)) && (pointIsInFront(temp_C1, pt3d_))) {
							pt3d.x += pt3d_.x;
							pt3d.y += pt3d_.y;
							pt3d.z += pt3d_.z;
							contribCount++;
						//}
					}
				}
			}
			
			if (contribCount > 0) {
				
				pt3d.x /= ((double) contribCount); 
				pt3d.y /= ((double) contribCount);
				pt3d.z /= ((double) contribCount);
				
				tracks.at(indices.at(iii)).set3dLoc(pt3d);
			}
		}
		
	}
	
	for (unsigned int iii = 0; iii < tracks.size(); iii++) {
		if (tracks.at(iii).isTriangulated) totalTriangulatedPoints++;
	}
	
	return totalTriangulatedPoints;
}

void findTriangulatableTracks(vector<featureTrack>& tracks, vector<unsigned int>& indices, vector<unsigned int>& cameras, unsigned int min_length) {
	
	unsigned int insuffAppeared = 0;
	for (unsigned int iii = 0; iii < tracks.size(); iii++) {
		
		if (tracks.at(iii).locations.size() < min_length) {
			continue;
		}
		
		unsigned int appearanceCount = 0;
		unsigned int progressIndex = 0;
		
		for (unsigned int kkk = 0; kkk < cameras.size(); kkk++) {
			
			while (progressIndex < tracks.at(iii).locations.size()) {
				if (tracks.at(iii).locations.at(progressIndex).imageIndex == cameras.at(kkk) ) {
					appearanceCount++;
					progressIndex++;
					break;
				}
				progressIndex++;
			}
			
		}
		
		if (appearanceCount >= min_length) {
			indices.push_back(iii);
		} else {
			insuffAppeared++;
			
		}
		
	}
	
	//printf("%s << insuffAppeared = (%d)\n", __FUNCTION__, insuffAppeared);
	
}

void findTriangulatableTracks3(vector<featureTrack>& tracks, vector<unsigned int>& indices, int latest_index, unsigned int min_length) {
	
	for (unsigned int iii = 0; iii < tracks.size(); iii++) {
		
		if (latest_index == -1) {
			if (tracks.at(iii).locations.size() >= min_length) {
				indices.push_back(iii);
			}
		} else {
			
			unsigned int appearanceCount = 0;
			
			for (unsigned int jjj = 0; jjj < tracks.at(iii).locations.size(); jjj++) {
				if (tracks.at(iii).locations.at(jjj).imageIndex <= ((unsigned int) latest_index)) {
					appearanceCount++;
				}
				
				if (appearanceCount >= min_length) {
					indices.push_back(iii);
					break;
				}
			}
		}
		
	}
	
}

#ifdef _BUILD_FOR_ROS_
int initialTrackTriangulationDummy(vector<featureTrack>& tracks, vector<unsigned int>& indices, cameraParameters& cameraData, geometry_msgs::PoseStamped *keyframePoses, unsigned int keyframeCount, double minSeparation, double maxSeparation, int minEstimates, double maxStandardDev, bool handedness, int xCode) {
	
	int lim = 10; // 10
	
	cv::Point2f dummyPoints1[10], dummyPoints2[10];
	
	// far
	if (0) {
		dummyPoints1[0].x = 200.5;
		dummyPoints1[0].y = 143.5;
		dummyPoints2[0].x = 180.5;
		dummyPoints2[0].y = 143.5;
		
	} else {
	
		dummyPoints1[0].x = 225.5;
		dummyPoints1[0].y = 163.5;
		dummyPoints2[0].x = 115.5;
		dummyPoints2[0].y = 163.5;
		
		dummyPoints1[1].x = 235.5;
		dummyPoints1[1].y = 143.5;
		dummyPoints2[1].x = 125.5;
		dummyPoints2[1].y = 143.5;
		
		dummyPoints1[2].x = 235.5;
		dummyPoints1[2].y = 123.5;
		dummyPoints2[2].x = 125.5;
		dummyPoints2[2].y = 123.5;
		
		dummyPoints1[3].x = 265.5;
		dummyPoints1[3].y = 163.5;
		dummyPoints2[3].x = 155.5;
		dummyPoints2[3].y = 163.5;
		
		dummyPoints1[4].x = 255.5;
		dummyPoints1[4].y = 143.5;
		dummyPoints2[4].x = 145.5;
		dummyPoints2[4].y = 143.5;
		
		dummyPoints1[5].x = 255.5;
		dummyPoints1[5].y = 123.5;
		dummyPoints2[5].x = 145.5;
		dummyPoints2[5].y = 123.5;
		
		// near
		dummyPoints1[6].x = 245.5;
		dummyPoints1[6].y = 163.5;
		dummyPoints2[6].x = 95.5;
		dummyPoints2[6].y = 163.5;
		
		dummyPoints1[7].x = 255.5;
		dummyPoints1[7].y = 123.5;
		dummyPoints2[7].x = 105.5;
		dummyPoints2[7].y = 123.5;
		
		dummyPoints1[8].x = 285.5;
		dummyPoints1[8].y = 163.5;
		dummyPoints2[8].x = 135.5;
		dummyPoints2[8].y = 163.5;
		
		dummyPoints1[9].x = 275.5;
		dummyPoints1[9].y = 123.5;
		dummyPoints2[9].x = 125.5;
		dummyPoints2[9].y = 123.5;
	}
	
	vector<cv::Point3f> testPt_3d;
	cv::Point3f tmp;
	tmp.x = 1.5;
	tmp.y = 1.5;
	tmp.z = 2.0;
	testPt_3d.push_back(tmp);
	
	/*
	cv::Mat testPt_3d(3, 1, CV_64FC1);
	testPt_3d.at<double>(0,0) = 1.5;
	testPt_3d.at<double>(1,0) = 1.5;
	testPt_3d.at<double>(2,0) = 1.0;
	*/
	
	vector<cv::Point2f> testPts_2d;
	
	cv::Mat rvec, tvec, cameraMatrix, distCoeffs;
	tvec = cv::Mat::zeros(3, 1, CV_64FC1);
	rvec = cv::Mat::zeros(3, 1, CV_64FC1);
	cameraMatrix = cv::Mat::eye(3, 3, CV_64FC1);
	distCoeffs = cv::Mat::zeros(5, 1, CV_64FC1);;
	
	// projectPoints(InputArray objectPoints, InputArray rvec, InputArray tvec, InputArray cameraMatrix, InputArray distCoeffs, OutputArray imagePoints, OutputArray jacobian=noArray(), double aspectRatio=0 );
		
	tvec.at<double>(0,0) = 0.0;
	tvec.at<double>(1,0) = 0.0;
	tvec.at<double>(2,0) = 0.0;
	projectPoints(testPt_3d, rvec, tvec, cameraMatrix, distCoeffs, testPts_2d);
	//printf("%s << Point (%f, %f, %f) projected to (%f, %f) and (%f, %f)\n", __FUNCTION__, testPt_3d.at<double>(0,0), testPt_3d.at<double>(1,0), testPt_3d.at<double>(2,0), testPts_2d.at(0).x, testPts_2d.at(0).y);
	printf("%s << Point (%f, %f, %f) projected to (%f, %f) for cam (%f, %f, %f)\n", __FUNCTION__, testPt_3d.at(0).x, testPt_3d.at(0).y, testPt_3d.at(0).z, testPts_2d.at(0).x, testPts_2d.at(0).y, tvec.at<double>(0,0), tvec.at<double>(1,0), tvec.at<double>(2,0));
	
	tvec.at<double>(0,0) = -1.5;
	tvec.at<double>(1,0) = -1.5;
	tvec.at<double>(2,0) = 0.0;
	
	projectPoints(testPt_3d, rvec, tvec, cameraMatrix, distCoeffs, testPts_2d);
	//printf("%s << Point (%f, %f, %f) projected to (%f, %f) and (%f, %f)\n", __FUNCTION__, testPt_3d.at<double>(0,0), testPt_3d.at<double>(1,0), testPt_3d.at<double>(2,0), testPts_2d.at(0).x, testPts_2d.at(0).y);
	printf("%s << Point (%f, %f, %f) projected to (%f, %f) for cam (%f, %f, %f)\n", __FUNCTION__, testPt_3d.at(0).x, testPt_3d.at(0).y, testPt_3d.at(0).z, testPts_2d.at(0).x, testPts_2d.at(0).y, tvec.at<double>(0,0), tvec.at<double>(1,0), tvec.at<double>(2,0));
	
	tvec.at<double>(0,0) = -3.0;
	tvec.at<double>(1,0) = -3.0;
	tvec.at<double>(2,0) = 0.0;
	
	projectPoints(testPt_3d, rvec, tvec, cameraMatrix, distCoeffs, testPts_2d);
	//printf("%s << Point (%f, %f, %f) projected to (%f, %f) and (%f, %f)\n", __FUNCTION__, testPt_3d.at<double>(0,0), testPt_3d.at<double>(1,0), testPt_3d.at<double>(2,0), testPts_2d.at(0).x, testPts_2d.at(0).y);
	printf("%s << Point (%f, %f, %f) projected to (%f, %f) for cam (%f, %f, %f)\n", __FUNCTION__, testPt_3d.at(0).x, testPt_3d.at(0).y, testPt_3d.at(0).z, testPts_2d.at(0).x, testPts_2d.at(0).y, tvec.at<double>(0,0), tvec.at<double>(1,0), tvec.at<double>(2,0));
	
	tvec.at<double>(0,0) = 0.0;
	tvec.at<double>(1,0) = -1.5;
	tvec.at<double>(2,0) = 0.0;
	
	projectPoints(testPt_3d, rvec, tvec, cameraMatrix, distCoeffs, testPts_2d);
	//printf("%s << Point (%f, %f, %f) projected to (%f, %f) and (%f, %f)\n", __FUNCTION__, testPt_3d.at<double>(0,0), testPt_3d.at<double>(1,0), testPt_3d.at<double>(2,0), testPts_2d.at(0).x, testPts_2d.at(0).y);
	printf("%s << Point (%f, %f, %f) projected to (%f, %f) for cam (%f, %f, %f)\n", __FUNCTION__, testPt_3d.at(0).x, testPt_3d.at(0).y, testPt_3d.at(0).z, testPts_2d.at(0).x, testPts_2d.at(0).y, tvec.at<double>(0,0), tvec.at<double>(1,0), tvec.at<double>(2,0));
	
	tvec.at<double>(0,0) = -1.5;
	tvec.at<double>(1,0) = 0.0;
	tvec.at<double>(2,0) = 0.0;
	
	projectPoints(testPt_3d, rvec, tvec, cameraMatrix, distCoeffs, testPts_2d);
	//printf("%s << Point (%f, %f, %f) projected to (%f, %f) and (%f, %f)\n", __FUNCTION__, testPt_3d.at<double>(0,0), testPt_3d.at<double>(1,0), testPt_3d.at<double>(2,0), testPts_2d.at(0).x, testPts_2d.at(0).y);
	printf("%s << Point (%f, %f, %f) projected to (%f, %f) for cam (%f, %f, %f)\n", __FUNCTION__, testPt_3d.at(0).x, testPt_3d.at(0).y, testPt_3d.at(0).z, testPts_2d.at(0).x, testPts_2d.at(0).y, tvec.at<double>(0,0), tvec.at<double>(1,0), tvec.at<double>(2,0));
	
	vector<cv::Point3d> estimatedLocations;
	for (int iii = 0; iii < lim; iii++) {
		
		cv::Mat R1, t1, temp_C1, temp_P1;
		Quaterniond q1;
		Eigen::Vector4d v1;
		
		
		q1 = Quaterniond(1.0, 0.0, 0.0, 0.0);
		//q1 = Quaterniond(0.70711, 0.0, -0.70711, 0.0); // trying to get projection matrix of identity..
		v1 = Eigen::Vector4d(1.3, 1.5, 0.0, 1.0);
		
		quaternionToMatrix(q1, R1, handedness);
		convertVec4dToMat(v1, t1);
		composeTransform(R1, t1, temp_C1);
		
		
		cv::Mat R2, t2, temp_C2, temp_P2;
		Quaterniond q2;
		Eigen::Vector4d v2;
		q2 = Quaterniond(1.0, 0.0, 0.0, 0.0);
		//q2 = Quaterniond(0.70711, 0.0, -0.70711, 0.0); // trying to get projection matrix of identity..
		v2 = Eigen::Vector4d(1.7, 1.5, 0.0, 1.0);
		
		
		quaternionToMatrix(q2, R2, handedness);
		convertVec4dToMat(v2, t2);
		composeTransform(R2, t2, temp_C2);
		
		cv::Point2f pt1_, pt2_;
		cv::Point3d pt3d_;
		
		pt1_ = dummyPoints1[iii]; // left camera, point further to right
		pt2_ = dummyPoints2[iii];
		
		temp_C1 = temp_C1.inv();
		temp_C2 = temp_C2.inv();
		
		cout << "temp_C1 = " << temp_C1 << endl;
		cout << "temp_C2 = " << temp_C2 << endl;
		
		Triangulate_1(pt1_, pt2_, cameraData.K, cameraData.K_inv, temp_C1, temp_C2, pt3d_, true);
		
		//pt3d_ = pt3d_;	// HACK!
		
		printf("%s << Triangulated (%f, %f) & (%f, %f) to (%f, %f, %f)\n", __FUNCTION__, pt1_.x, pt1_.y, pt2_.x, pt2_.y, pt3d_.x, pt3d_.y, pt3d_.z);
		
		estimatedLocations.push_back(pt3d_);
		
		tracks.at(indices.at(iii)).set3dLoc(pt3d_);
	}
	
	return lim;
	
}
#endif

void filterNearPoints(vector<featureTrack>& featureTrackVector, double x, double y, double z, double limit) {
	
	for (unsigned int iii = 0; iii < featureTrackVector.size(); iii++) {
		
		if (featureTrackVector.at(iii).isTriangulated) {
			cv::Point3d pt3d = cv::Point3d(x, y, z);
			cv::Point3d pt3d_ = featureTrackVector.at(iii).get3dLoc();
			if (distBetweenPts(pt3d, pt3d_) < limit) {
				featureTrackVector.at(iii).isTriangulated = false;
			}
		}
		
	}
	
}

#ifdef _BUILD_FOR_ROS_
int initialTrackTriangulation(vector<featureTrack>& tracks, vector<unsigned int>& indices, cameraParameters& cameraData, geometry_msgs::PoseStamped *keyframePoses, unsigned int keyframeCount, double minSeparation, double maxSeparation, int minEstimates, double maxStandardDev, double maxReprojectionDisparity) {
	
	cv::Point3d pt3d, mean3d(0.0, 0.0, 0.0), stddev3d(0.0, 0.0, 0.0);
	Quaterniond q0(1.0, 0.0, 0.0, 0.0); // corresponds to default projection matrix
	
	int minProjCount = 0, clusterFail = 0;
	vector<int> validPairs, tooClosePairs, tooFarPairs, tooNearPairs, inFrontPairs, withinDisparityPairs;
	
	
	int triangulatedCounter = 0;
	
	int minProjections_ = minProjections(minEstimates);
	
	for (unsigned int iii = 0; iii < indices.size(); iii++) {
		
		unsigned int ptsBehind = 0, ptsInfront = 0;
		
		int validPairs_ = 0, tooClosePairs_ = 0, tooFarPairs_ = 0, tooNearPairs_ = 0, inFrontPairs_ = 0, withinDisparityPairs_ = 0;
		
		if (tracks.size() <= indices.at(iii)) {
			printf("%s << ERROR 1!\n", __FUNCTION__);
			continue;
		}
		//if (tracks.at(iii).isTriangulated) continue;
		
		
		if (((int)tracks.at(indices.at(iii)).locations.size()) < minProjections_) {
			printf("%s << ERROR 2!\n", __FUNCTION__);
			continue;
		}
		
		//printf("%s << Continuing with (%d)...\n", __FUNCTION__, iii);
		
		vector<cv::Point3d> estimatedLocations;
		vector<double> separationsVector;
			
		pt3d = cv::Point3d(0.0, 0.0, 0.0);
		
		int rrr, sss;
		
		// For each projection of interest (to make the FIRST one)
		for (unsigned int jjj = 0; jjj < tracks.at(indices.at(iii)).locations.size()-1; jjj++) {
			
			//printf("%s << Here (%d) (jjj = %d)...\n", __FUNCTION__, iii, jjj);
			
			// Get the index of that image
			rrr = tracks.at(indices.at(iii)).locations.at(jjj).imageIndex;
			
			// Check if you have that camera:
			int cameraIndex1 = -1;
			for (unsigned int kkk = 0; kkk < keyframeCount; kkk++) {
				if (((int)keyframePoses[kkk].header.seq) == rrr) {
					cameraIndex1 = kkk;
					break;
				}
			}
			
			// If you don't, move to next projection
			if (cameraIndex1 == -1) {
				//printf("%s << Error! missing camera 1.. (%d), (%d)\n", __FUNCTION__, rrr, cameraIndex1);
				continue;
			}
			
			// Assign first camera to this matrix
			cv::Mat R1, t1, temp_C1, temp_P1;
			Quaterniond q1;
			Eigen::Vector4d v1;
			
			q1 = Quaterniond(keyframePoses[cameraIndex1].pose.orientation.w, keyframePoses[cameraIndex1].pose.orientation.x, keyframePoses[cameraIndex1].pose.orientation.y, keyframePoses[cameraIndex1].pose.orientation.z);
			v1 = Eigen::Vector4d(keyframePoses[cameraIndex1].pose.position.x, keyframePoses[cameraIndex1].pose.position.y, keyframePoses[cameraIndex1].pose.position.z, 1.0);
			
			quaternionToMatrix(q1, R1);
			convertVec4dToMat(v1, t1);
			composeTransform(R1, t1, temp_C1);
			temp_C1 = temp_C1.inv();
				
			transformationToProjection(temp_C1, temp_P1);
			
			for (unsigned int kkk = jjj+1; kkk < tracks.at(indices.at(iii)).locations.size(); kkk++) {
				
				//printf("%s << Here (%d) (kkk = %d)...\n", __FUNCTION__, iii, kkk);
				
				sss = tracks.at(indices.at(iii)).locations.at(kkk).imageIndex;
				
				// Check if you have that camera:
				int cameraIndex2 = -1;
				for (unsigned int mmm = 0; mmm < keyframeCount; mmm++) {
					if (((int)keyframePoses[mmm].header.seq) == sss) {
						cameraIndex2 = mmm;
						break;
					}
				}
				
				// If you don't, move to next projection
				if (cameraIndex2 == -1) {
					//printf("%s << Error! missing camera 2.. (%d), (%d)\n", __FUNCTION__, sss, cameraIndex2);
					continue;
				}
				
				//printf("%s << index (%d) is continuing...\n", __FUNCTION__, iii);
				
				validPairs_++;
						
				// Assign second camera to this matrix
				cv::Mat R2, t2, temp_C2, temp_P2;
				Quaterniond q2;
				Eigen::Vector4d v2;
				
				q2 = Quaterniond(keyframePoses[cameraIndex2].pose.orientation.w, keyframePoses[cameraIndex2].pose.orientation.x, keyframePoses[cameraIndex2].pose.orientation.y, keyframePoses[cameraIndex2].pose.orientation.z);
				v2 = Eigen::Vector4d(keyframePoses[cameraIndex2].pose.position.x, keyframePoses[cameraIndex2].pose.position.y, keyframePoses[cameraIndex2].pose.position.z, 1.0);				
				
				quaternionToMatrix(q2, R2);
				convertVec4dToMat(v2, t2);
				composeTransform(R2, t2, temp_C2);
				temp_C2 = temp_C2.inv();
				transformationToProjection(temp_C2, temp_P2);
				
				// Check whether separation is sufficient...
				double separation = pow(pow(v1.x() - v2.x(), 2.0) + pow(v1.y() - v2.y(), 2.0) + pow(v1.z() - v2.z(), 2.0), 0.5);
				
				if ( (separation < minSeparation) && (minSeparation != 0.0) ) {
					//printf("%s << Error! (%f) < (%f)\n", __FUNCTION__, separation, minSeparation);
					tooClosePairs_++;
					continue;
				} else if ( (separation > maxSeparation) && (maxSeparation != 0.0) ) {
					//printf("%s << Error! (%f) > (%f)\n", __FUNCTION__, separation , maxSeparation);
					tooFarPairs_++;
					continue;
				}

				cv::Point2f pt1_, pt2_;
				cv::Point3d pt3d_, pt3d_temp;
				
				pt1_ = tracks.at(indices.at(iii)).getCoord(rrr);
				pt2_ = tracks.at(indices.at(iii)).getCoord(sss);
				
				Triangulate_1(pt1_, pt2_, cameraData.K, cameraData.K_inv, temp_C1, temp_C2, pt3d_, true);
			
				//printf("%s << Triangulated (%d) to (%f, %f, %f)...\n", __FUNCTION__, iii, pt3d_.x, pt3d_.y, pt3d_.z);
			
				if (pointIsInFront(temp_C1, pt3d_) && pointIsInFront(temp_C2, pt3d_)) {
					
					inFrontPairs_++;
					
					vector<cv::Point3f> testPt_3d;
					testPt_3d.push_back(pt3d_);
					cv::Mat distCoeffs = cv::Mat::zeros(5, 1, CV_64FC1);
					
					cv::Mat t1x, r1x, R1_X;
					cv::Mat t2x, r2x, R2_X;
					decomposeTransform(temp_C1, R1_X, t1x);
					decomposeTransform(temp_C2, R2_X, t2x);
					Rodrigues(R1_X, r1x);
					Rodrigues(R2_X, r2x);
					
					
					/*
					
					cv::Mat t1x, r1x;
					cv::Mat t2x, r2x;
					t1x = -t1;
					t2x = -t2;
					
					Rodrigues(R1, r1x);
					Rodrigues(R2, r2x);
					r1x = -r1x;
					r2x = -r2x;
					*/
					
					vector<cv::Point2f> testPts_2d_1, testPts_2d_2;
					
					projectPoints(testPt_3d, r1x, t1x, cameraData.K, distCoeffs, testPts_2d_1);
					double dist1 = pow(pow(testPts_2d_1.at(0).x - pt1_.x, 2.0) + pow(testPts_2d_1.at(0).y - pt1_.y, 2.0), 0.5);
					projectPoints(testPt_3d, r2x, t2x, cameraData.K, distCoeffs, testPts_2d_2);
					double dist2 = pow(pow(testPts_2d_2.at(0).x - pt2_.x, 2.0) + pow(testPts_2d_2.at(0).y - pt2_.y, 2.0), 0.5);
					
					//printf("%s << Track (%d):(%d,%d) from (%d,%d)/(%d,%d) to (%d,%d)/(%d,%d) for 3D (%3.1f, %3.1f, %3.1f)\n", __FUNCTION__, iii, jjj, kkk, int(pt1_.x), int(pt1_.y), int(pt2_.x), int(pt2_.y), int(testPts_2d_1.at(0).x), int(testPts_2d_1.at(0).y), int(testPts_2d_2.at(0).x), int(testPts_2d_2.at(0).y), testPt_3d.at(0).x, testPt_3d.at(0).y, testPt_3d.at(0).z);
					
					if ( ( (dist1 < maxReprojectionDisparity) && (dist2 < maxReprojectionDisparity) ) || (maxReprojectionDisparity == 0.0) ) {
						withinDisparityPairs_++;
						//printf("%s << And here (%d)...\n", __FUNCTION__, iii);
						if ( (pt3d_.x != 0.0) && (pt3d_.y != 0.0) && (pt3d_.z != 0.0)) {
							//printf("%s << And finally here (%d)...\n", __FUNCTION__, iii);
							estimatedLocations.push_back(pt3d_);
							separationsVector.push_back(separation);
						}
					}
				} else {
					ptsBehind++;
				}
			}
		}
		
		
		
		validPairs.push_back(validPairs_);
		tooClosePairs.push_back(tooClosePairs_);
		tooFarPairs.push_back(tooFarPairs_);
		tooNearPairs.push_back(tooNearPairs_);
		inFrontPairs.push_back(inFrontPairs_);
		withinDisparityPairs.push_back(withinDisparityPairs_);
		
		//printf("%s << (%d, %d, %d, %d, %d, %d)\n", __FUNCTION__, validPairs_, tooClosePairs_, tooFarPairs_, tooNearPairs_, inFrontPairs_, withinDisparityPairs_);
		
		if (((int)estimatedLocations.size()) < minEstimates) {
			// printf("%s << Error! Insufficient estimates (%d) < (%d)\n", __FUNCTION__, ((int)estimatedLocations.size()) , minEstimates);
			minProjCount++;
			continue;
		} else {
			//printf("%s << Can continue..\n", __FUNCTION__);
		}
		
		int mode = CLUSTER_MEAN_MODE; // DEFAULT_MEAN_MODE
		bool validResult = findClusterMean(estimatedLocations, pt3d, mode, minEstimates, maxStandardDev);
		
		if (validResult) {
			
			//printf("%s << Clustering result is valid. (%f)\n", __FUNCTION__, maxStandardDev);
			
			/*
			if (tracks.at(indices.at(iii)).isTriangulated) {
				cv::Point3d oldPt = tracks.at(indices.at(iii)).get3dLoc();
				double dist = distBetweenPts(oldPt, pt3d);
				if (dist < maxStandardDev) {
					printf("%s << Error! Dist is too low: (%f) < (%f)\n", __FUNCTION__, ((int)estimatedLocations.size()) , minEstimates);
					continue;
				}
			}			
			*/
			
			tracks.at(indices.at(iii)).set3dLoc(pt3d);
			triangulatedCounter++;
			
		} else {
			clusterFail++;
			//printf("%s << error: clusterFail!\n", __FUNCTION__);
		}

	}
	
	double validPairs_ave = 0.0, tooClosePairs_ave = 0.0, tooFarPairs_ave = 0.0, tooNearPairs_ave = 0.0, inFrontPairs_ave = 0.0, withinDisparityPairs_ave = 0.0;
	
	for (unsigned int iii = 0; iii < validPairs.size(); iii++) {
		validPairs_ave += validPairs.at(iii);
		tooClosePairs_ave += tooClosePairs.at(iii);
		tooFarPairs_ave += tooFarPairs.at(iii);
		tooNearPairs_ave += tooNearPairs.at(iii);
		inFrontPairs_ave += inFrontPairs.at(iii);
		withinDisparityPairs_ave += withinDisparityPairs.at(iii);
	}
	
	validPairs_ave /= double(validPairs.size());
	tooClosePairs_ave /= double(validPairs.size());
	tooFarPairs_ave /= double(validPairs.size());
	tooNearPairs_ave /= double(validPairs.size());
	inFrontPairs_ave /= double(validPairs.size());
	withinDisparityPairs_ave /= double(validPairs.size());
	
	// printf("%s << summ: (%f, %f, %f, %f, %f, %f)\n", __FUNCTION__, validPairs_ave, tooClosePairs_ave, tooFarPairs_ave, tooNearPairs_ave, inFrontPairs_ave, withinDisparityPairs_ave);
	
	//printf("%s << failed triangulations: minCOunt = (%d), cluster = (%d)\n", __FUNCTION__, minProjCount, clusterFail);
	
	return triangulatedCounter;
	
}
#endif

int TriangulateNewTracks(vector<featureTrack>& trackVector, const int index1, const int index2, const cv::Mat& K, const cv::Mat& K_inv, const cv::Mat& P0, const cv::Mat& P1, bool initializeOnFocalPlane) {
	
	cv::Mat coordinateTransform;
	
	int tracksTriangulated = 0;

	
	for (unsigned int iii = 0; iii < trackVector.size(); iii++) {
		
		for (unsigned int jjj = 0; jjj < trackVector.at(iii).locations.size()-1; jjj++) {
					
			int trInd1 = trackVector.at(iii).locations.at(jjj).imageIndex;
				
			if (trInd1 == index1) {
				
				for (unsigned int kkk = jjj+1; kkk < trackVector.at(iii).locations.size(); kkk++) {
					
					int trInd2 = trackVector.at(iii).locations.at(kkk).imageIndex;
					
					if (trInd2 == index2) {
						
						cv::Point2f loc1 = trackVector.at(iii).locations.at(jjj).featureCoord;
						cv::Point2f loc2 = trackVector.at(iii).locations.at(kkk).featureCoord;
						
						cv::Point3d xyzPoint;
			
						// This puts the 3D point out but in camera A's co-ordinates
						if (initializeOnFocalPlane) {
							xyzPoint.x = loc1.x;
							xyzPoint.y = loc1.y;
							xyzPoint.z = 1.0;
						} else {
							Triangulate(loc1, loc2, K, K_inv, P0, P1, xyzPoint);
						}
						
						//printf("%s << PT relative to cam A: (%f, %f, %f)\n", __FUNCTION__, xyzPoint.x, xyzPoint.y, xyzPoint.z);
						
						// Convert XYZ co-ordinate to WORLD co-ordinates
						
						trackVector.at(iii).set3dLoc(xyzPoint);
						//trackVector.at(iii).xyzEstimate.x = xyzPoint.x;
						//trackVector.at(iii).xyzEstimate.y = xyzPoint.y;
						//trackVector.at(iii).xyzEstimate.z = xyzPoint.z;
						
						//printf("%s << PT relative to WORLD: (%f, %f, %f)\n", __FUNCTION__, trackVector.at(iii).xyzEstimate.x, trackVector.at(iii).xyzEstimate.y, trackVector.at(iii).xyzEstimate.z);
						
						tracksTriangulated++;
						
					}
					
				}
				
			}
				
				
		}
		
		
		
	
	}
	
	return tracksTriangulated;

}

void Triangulate_1(const cv::Point2d& pt1, const cv::Point2d& pt2, const cv::Mat& K, const cv::Mat& Kinv, const cv::Mat& P1, const cv::Mat& P2, cv::Point3d& xyzPoint, bool iterate) {

	cv::Mat C1, C2;
	projectionToTransformation(P1, C1);
	projectionToTransformation(P2, C2);

	cv::Mat U1(3, 1, CV_64FC1);
	U1.at<double>(0,0) = pt1.x;
	U1.at<double>(1,0) = pt1.y;
	U1.at<double>(2,0) = 1.0;
	cv::Mat UM1 = Kinv * U1;
	// Get the image point into normalized camera co-ordinates..
	cv::Point3d u1(UM1.at<double>(0,0), UM1.at<double>(1,0), UM1.at<double>(2,0));
	
	cv::Mat U2(3, 1, CV_64FC1);
	U2.at<double>(0,0) = pt2.x;
	U2.at<double>(1,0) = pt2.y;
	U2.at<double>(2,0) = 1.0;
	cv::Mat UM2 = Kinv * U2;
	// Get the image point into normalized camera co-ordinates..
	cv::Point3d u2(UM2.at<double>(0,0), UM2.at<double>(1,0), UM2.at<double>(2,0));
	
	// printf("%s << normalized co-ordinates = (%f, %f) & (%f, %f)\n", __FUNCTION__, u1.x, u1.y, u2.x, u2.y);
	
	cv::Mat X;
	
	if (iterate) {
		IterativeLinearLSTriangulation(X, u1, C1, u2, C2);
		
		/*
		cv::Mat C1X, C2X;
		transformationToProjection(C1, C1X);
		transformationToProjection(C2, C2X);
		
		cv::Mat U1X(2, 1, CV_64FC1);
		U1X.at<double>(0,0) = UM1.at<double>(0,0);
		U1X.at<double>(1,0) = UM1.at<double>(1,0);
		cv::Mat U2X(2, 1, CV_64FC1);
		U2X.at<double>(0,0) = UM2.at<double>(0,0);
		U2X.at<double>(1,0) = UM2.at<double>(1,0);
		
		triangulatePoints(C1X, C2X, U1X, U2X, X);
		*/
		
	} else {
		LinearLSTriangulation(X, u1, C1, u2, C2);
	}


	xyzPoint = cv::Point3d(X.at<double>(0,0), X.at<double>(1,0), X.at<double>(2,0));
	
	//printf("%s << Points triangulated to (%f, %f, %f)\n", __FUNCTION__, xyzPoint.x, xyzPoint.y, xyzPoint.z);

}

void Triangulate(const cv::Point2f& pt1, const cv::Point2f& pt2, const cv::Mat& K, const cv::Mat& Kinv,	const cv::Mat& P1, const cv::Mat& P2, cv::Point3d& xyzPoint, bool debug) {

	cv::Mat C1, C2;
	projectionToTransformation(P1, C1);
	projectionToTransformation(P2, C2);
	
	// Relative
	cv::Mat P0, PR, C0, CR;
	initializeP0(P0);
	projectionToTransformation(P0, C0);
	CR = C2 * C1.inv();
	transformationToProjection(CR, PR);

	cv::Point3d xyzPoint_R;

	
	
	//cv::Point2f kp = pt1;
	cv::Point3d u_1(pt1.x, pt1.y, 1.0);
	cv::Mat umx1 = Kinv * cv::Mat(u_1);
	
	u_1.x = umx1.at<double>(0, 0);
	u_1.y = umx1.at<double>(1, 0);
	u_1.z = umx1.at<double>(2, 0);
	
	//cv::Point2f kp1 = pt2;
	cv::Point3d u_2(pt2.x, pt2.y,1.0);
	cv::Mat umx2 = Kinv * cv::Mat(u_2);
	
	u_2.x = umx2.at<double>(0, 0);
	u_2.y = umx2.at<double>(1, 0);
	u_2.z = umx2.at<double>(2, 0);
	

	
	cv::Mat X;
	//LinearLSTriangulation(X, u_1, P0, u_2, PR);
	IterativeLinearLSTriangulation(X, u_1, P0, u_2, PR);	// switched from u then u1
	
	xyzPoint_R = cv::Point3d( X.at<double>(0, 0), X.at<double>(1, 0), X.at<double>(2, 0) );
	
	cv::Mat A(4, 1, CV_64FC1), B(4, 1, CV_64FC1);
	
	A.at<double>(0,0) = xyzPoint_R.x;
	A.at<double>(1,0) = xyzPoint_R.y;
	A.at<double>(2,0) = xyzPoint_R.z;
	A.at<double>(3,0) = 1.0;

	
	B = C1 * A;
	
	xyzPoint.x = B.at<double>(0,0) / B.at<double>(3,0);
	xyzPoint.y = B.at<double>(1,0) / B.at<double>(3,0);
	xyzPoint.z = B.at<double>(2,0) / B.at<double>(3,0);
	
	if (debug) {
		
		printf("%s << DEBUG SUMMARY:\n", __FUNCTION__);
		
		
		cout << endl << "P1 = " << P1 << endl;
		cout << "P2 = " << P2 << endl;
		cout << "P0 = " << P0 << endl;
		cout << "PR = " << PR << endl;
		cout << "C1 = " << C1 << endl << endl;
		
		printf("pt1 -> u_1 = (%f, %f), (%f, %f, %f)\n", pt1.x, pt1.y, u_1.x, u_1.y, u_1.z);
		printf("pt2 -> u_2 = (%f, %f), (%f, %f, %f)\n\n", pt2.x, pt2.y, u_2.x, u_2.y, u_2.z);

		printf("xyzPoint_R = (%f, %f, %f)\n", xyzPoint_R.x, xyzPoint_R.y, xyzPoint_R.z);
		printf("xyzPoint = (%f, %f, %f)\n\n", xyzPoint.x, xyzPoint.y, xyzPoint.z);
		
		cin.get();
	}
	

}

void TriangulatePoints(const vector<cv::Point2f>& pt_set1,
                       const vector<cv::Point2f>& pt_set2,
                       const cv::Mat& K,
                       const cv::Mat& Kinv,
                       const cv::Mat& P,
                       const cv::Mat& P1,
                       vector<cv::Point3d>& pointcloud,
                       vector<cv::Point2f>& correspImg1Pt)
{
 
    pointcloud.clear();
    correspImg1Pt.clear();
    
    cv::Point3d point_3;

    for (unsigned int iii = 0; iii < pt_set1.size(); iii++) {    
		
		Triangulate(pt_set1.at(iii), pt_set2.at(iii), K, Kinv,	P, P1, point_3, false);
		 
		pointcloud.push_back(point_3);
		correspImg1Pt.push_back(pt_set1[iii]);
	}
}

void LinearLSTriangulation(cv::Mat& dst, const cv::Point3d& u1, const cv::Mat& P1, const cv::Point3d& u2, const cv::Mat& P2) {


	
	// dst 	: 3D point (homogeneous?)
	// u1 	: image 1 homogenous 2D point
	// P1	: image 1 camera projection (3,4)
	// u2 	: image 2 homogenous 2D point
	// P2	: image 2 camera projection	
	
    // First, build matrix A for homogenous equation system Ax = 0
    
    cv::Mat A(4, 3, CV_64FC1);
    
    A.at<double>(0,0) = u1.x * P1.at<double>(2,0) - P1.at<double>(0,0);
    A.at<double>(0,1) = u1.x * P1.at<double>(2,1) - P1.at<double>(0,1);
    A.at<double>(0,2) = u1.x * P1.at<double>(2,2) - P1.at<double>(0,2);
    
    A.at<double>(1,0) = u1.y * P1.at<double>(2,0) - P1.at<double>(1,0);
    A.at<double>(1,1) = u1.y * P1.at<double>(2,1) - P1.at<double>(1,1);
    A.at<double>(1,2) = u1.y * P1.at<double>(2,2) - P1.at<double>(1,2);
    
    A.at<double>(2,0) = u2.x * P2.at<double>(2,0) - P2.at<double>(0,0);
    A.at<double>(2,1) = u2.x * P2.at<double>(2,1) - P2.at<double>(0,1);
    A.at<double>(2,2) = u2.x * P2.at<double>(2,2) - P2.at<double>(0,2);
    
    A.at<double>(3,0) = u2.y * P2.at<double>(2,0) - P2.at<double>(1,0);
    A.at<double>(3,1) = u2.y * P2.at<double>(2,1) - P2.at<double>(1,1);
    A.at<double>(3,2) = u2.y * P2.at<double>(2,2) - P2.at<double>(1,2);

	// Assume X = (x,y,z,1), for Linear-LS method
    
    // Which turns it into a AX = B system, where A is 4x3, X is 3x1 and B is 4x1

	cv::Mat B(4, 1, CV_64FC1);
	
	B.at<double>(0,0) = -((double) u1.x * P1.at<double>(2,3) - P1.at<double>(0,3));
	B.at<double>(0,1) = -((double) u1.y * P1.at<double>(2,3) - P1.at<double>(1,3));
	B.at<double>(0,2) = -((double) u2.x * P2.at<double>(2,3) - P2.at<double>(0,3));
	B.at<double>(0,3) = -((double) u2.y * P2.at<double>(2,3) - P2.at<double>(1,3));
 
    cv::solve(A, B, dst, cv::DECOMP_SVD);

}

void IterativeLinearLSTriangulation(cv::Mat& dst, const cv::Point3d& u1, const cv::Mat& P1, const cv::Point3d& u2, const cv::Mat& P2) {
	
    int maxIterations = 10; //Hartley suggests 10 iterations at most
    
    cv::Mat X(4, 1, CV_64FC1), XA;
    
    
    
    LinearLSTriangulation(XA, u1, P1, u2, P2);

    
    X.at<double>(0,0) = XA.at<double>(0,0);
	X.at<double>(1,0) = XA.at<double>(1,0); 
	X.at<double>(2,0) = XA.at<double>(2,0); 
	X.at<double>(3,0) = 1.0;
    
    double wi1 = 1.0, wi2 = 1.0;
    
    for (int i = 0; i < maxIterations; i++) {
 
        // recalculate weights
        cv::Mat P1X = P1.row(2) * X;
        double p1a = P1X.at<double>(0, 0);
        
        cv::Mat P2X = P2.row(2) * X;
        double p2a = P2X.at<double>(0, 0);
        
        // breaking point
        if ((fabsf(float(wi1 - p1a)) <= EPSILON) && (fabsf(float(wi2 - p2a)) <= EPSILON)) {
			break;
		} 
 
        wi1 = p1a;
        wi2 = p2a;
 
        // reweight equations and solve
        cv::Mat A(4, 3, CV_64FC1);
        
        A.at<double>(0,0) = (u1.x * P1.at<double>(2,0) - P1.at<double>(0,0)) / wi1;
        A.at<double>(0,1) = (u1.x * P1.at<double>(2,1) - P1.at<double>(0,1)) / wi1;
        A.at<double>(0,2) = (u1.x * P1.at<double>(2,2) - P1.at<double>(0,2)) / wi1;
        
        A.at<double>(1,0) = (u1.y * P1.at<double>(2,0) - P1.at<double>(1,0)) / wi1;
        A.at<double>(1,1) = (u1.y * P1.at<double>(2,1) - P1.at<double>(1,1)) / wi1;
        A.at<double>(1,2) = (u1.y * P1.at<double>(2,2) - P1.at<double>(1,2)) / wi1;
        
        A.at<double>(2,0) = (u2.x * P2.at<double>(2,0) - P2.at<double>(0,0)) / wi2;
        A.at<double>(2,1) = (u2.x * P2.at<double>(2,1) - P2.at<double>(0,1)) / wi2;
        A.at<double>(2,2) = (u2.x * P2.at<double>(2,2) - P2.at<double>(0,2)) / wi2;
        
        A.at<double>(3,0) = (u2.y * P2.at<double>(2,0) - P2.at<double>(1,0)) / wi2;
        A.at<double>(3,1) = (u2.y * P2.at<double>(2,1) - P2.at<double>(1,1)) / wi2;
        A.at<double>(3,2) = (u2.y * P2.at<double>(2,2) - P2.at<double>(1,2)) / wi2;
        
        cv::Mat B(4, 1, CV_64FC1);
        
        B.at<double>(0,0) = -(u1.x * P1.at<double>(2,3) - P1.at<double>(0,3)) / wi1;
        B.at<double>(1,0) = -(u1.y * P1.at<double>(2,3) - P1.at<double>(1,3)) / wi1;
        B.at<double>(2,0) = -(u2.x * P2.at<double>(2,3) - P2.at<double>(0,3)) / wi2;
        B.at<double>(3,0) = -(u2.y * P2.at<double>(2,3) - P2.at<double>(1,3)) / wi2;

        cv::solve(A, B, XA, cv::DECOMP_SVD);

        X.at<double>(0,0) = XA.at<double>(0,0);
        X.at<double>(1,0) = XA.at<double>(1,0); 
        X.at<double>(2,0) = XA.at<double>(2,0); 
        X.at<double>(3,0) = 1.0; 
        
    }
    
    X.copyTo(dst);

}

void getPoints3dFromTracks(vector<featureTrack>& tracks, vector<cv::Point3d>& cloud, int idx1, int idx2) {
	
	cloud.clear();
	
	for (unsigned int iii = 0; iii < tracks.size(); iii++) {
		
		if (!tracks.at(iii).isTriangulated) continue;
		if (tracks.at(iii).locations.size() == 0) continue;
		
		for (unsigned int kkk = 0; kkk < tracks.at(iii).locations.size(); kkk++) {
			
			bool addedPoint = false;
			// If the track extends between the two images of interest
			if ((((int)tracks.at(iii).locations.at(kkk).imageIndex) == idx1) || (idx1 == -1)) {
				
				for (unsigned int jjj = 0; jjj < tracks.at(iii).locations.size(); jjj++) {
					
					if ((((int)tracks.at(iii).locations.at(jjj).imageIndex) == idx2) || (idx2 == -1)) {
						cloud.push_back(tracks.at(iii).get3dLoc());
						addedPoint = true;
						break;
					}
				}
				
			}
			
			if (addedPoint) break;
		}
	}
}
