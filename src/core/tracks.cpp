/*! \file	tracks.cpp
 *  \brief	Definitions for managing local feature tracks across video sequences.
*/

#include "core/tracks.hpp"

void featureTrack::set3dLoc(const cv::Point3d& loc) {
	if (!isTriangulated) {
		xyzEstimate = loc;
		isTriangulated = true;
	}
}

void featureTrack::reset3dLoc(const cv::Point3d& loc) {
	
	xyzEstimate.x = loc.x;
	xyzEstimate.y = loc.y;
	xyzEstimate.z = loc.z;

	isTriangulated = true;
	
}

void featureTrack::addFeature(indexedFeature& feat) {
		
	if (locations.size() == 0) { 
		locations.push_back(feat);
		return;
	}
		
	unsigned int pos = 0;
				
	while (locations.at(pos).imageIndex < feat.imageIndex) {
		pos++;
		if (pos >= locations.size()) break;
	}
	locations.insert(locations.begin()+pos, feat);
}

unsigned int getActiveTrackCount(const vector<featureTrack>& featureTrackVector, unsigned int previousIndex, unsigned int latestIndex) {
	
	unsigned int trackCount = 0;
	for (unsigned int iii = 0; iii < featureTrackVector.size(); iii++) {
		if (featureTrackVector.at(iii).locations.size() < 2) continue;
		if (featureTrackVector.at(iii).locations.at(featureTrackVector.at(iii).locations.size()-1).imageIndex == latestIndex) {
			if (featureTrackVector.at(iii).locations.at(featureTrackVector.at(iii).locations.size()-2).imageIndex == previousIndex) {
				trackCount++;
			}
		}
	}
	return trackCount;
}

double obtainFeatureSpeeds(const vector<featureTrack>& featureTrackVector, unsigned int idx1, double time1, unsigned int idx2, double time2) {
	
	double cumulativeVelocity = 0.0;
	unsigned int cumulativeCount = 0;
	
	if ((time1 == time2) || (time1 == 0.0) || (time2 == 0.0)) return -1.0;
	
	for (unsigned int iii = 0; iii < featureTrackVector.size(); iii++) {
		
		if (featureTrackVector.at(iii).locations.size() < 2) continue;
		
		// If the last two features match the frame indices you're interested in...
		if (featureTrackVector.at(iii).locations.at(featureTrackVector.at(iii).locations.size()-1).imageIndex == idx2) {
			
			if (featureTrackVector.at(iii).locations.at(featureTrackVector.at(iii).locations.size()-2).imageIndex == idx1) {
				
				cumulativeCount++;
				cv::Point2f p1, p2, v;
				
				p1 = featureTrackVector.at(iii).locations.at(featureTrackVector.at(iii).locations.size()-2).featureCoord;
				p2 = featureTrackVector.at(iii).locations.at(featureTrackVector.at(iii).locations.size()-1).featureCoord;
				
				v.x = float((p2.x - p1.x) / (time2 - time1));
				v.y = float((p2.y - p1.y) / (time2 - time1));
				
				cumulativeVelocity += pow(pow(v.x, 2.0)+pow(v.y, 2.0), 0.5);
			}
		}
	}
	
	
	(cumulativeCount > 0) ? cumulativeVelocity /= double(cumulativeCount) : cumulativeVelocity = 0.0;
	return cumulativeVelocity;
}

double updateFeatureSpeeds(vector<featureTrack>& featureTrackVector, unsigned int idx1, double time1, unsigned int idx2, double time2, double maxVelocity) {
	
	for (unsigned int iii = 0; iii < featureTrackVector.size(); iii++) {
		featureTrackVector.at(iii).velocity_x = 0.0;
		featureTrackVector.at(iii).velocity_y = 0.0;
	}
	
	if ((time1 == time2) || (time2 == 0.0) || (abs(time1 - time2) > MAX_TIME_DIFF_FOR_PREDICTION)) return -1.0;
	
	double cumulativeVelocity = 0.0;
	unsigned int cumulativeCount = 0;
	
	for (unsigned int iii = 0; iii < featureTrackVector.size(); iii++) {
		if (featureTrackVector.at(iii).locations.size() < 2) continue;
		
		// If the last two features match the frame indices you're interested in...
		if (featureTrackVector.at(iii).locations.at(featureTrackVector.at(iii).locations.size()-1).imageIndex == idx2) {
			if (featureTrackVector.at(iii).locations.at(featureTrackVector.at(iii).locations.size()-2).imageIndex == idx1) {
				
				cumulativeCount++;
				cv::Point2f p1, p2, v;
				
				p1 = featureTrackVector.at(iii).locations.at(featureTrackVector.at(iii).locations.size()-2).featureCoord;
				p2 = featureTrackVector.at(iii).locations.at(featureTrackVector.at(iii).locations.size()-1).featureCoord;
				
				v.x = float((p2.x - p1.x) / (time2 - time1));
				v.y = float((p2.y - p1.y) / (time2 - time1));
				
				double s = maxVelocity / pow(pow(v.x, 2.0)+pow(v.y,2.0),0.5);
				s = min(s,1.0);
				
				featureTrackVector.at(iii).velocity_x = v.x * s;
				featureTrackVector.at(iii).velocity_y = v.y * s;
				
				cumulativeVelocity += pow(pow(featureTrackVector.at(iii).velocity_x, 2.0)+pow(featureTrackVector.at(iii).velocity_y, 2.0), 0.5);
			}
		}
	}
	
	if (cumulativeCount > 0) {
		cumulativeVelocity /= double(cumulativeCount);
	} else cumulativeVelocity = 0.0;
	
	return cumulativeVelocity;
}

void removeObsoleteElements(vector<featureTrack>& featureTrackVector, const vector<unsigned int>& activeFrameIndices) {
	
	for (unsigned int iii = 0; iii < featureTrackVector.size(); iii++) {
		
		if (!featureTrackVector.at(iii).occursInSequence(activeFrameIndices)) {
			featureTrackVector.erase(featureTrackVector.begin() + iii);
			iii--;
		} else {
			// Need to go through individual indices and delete them
			for (unsigned int jjj = 0; jjj < featureTrackVector.at(iii).locations.size(); jjj++) {
				
				bool isPresent = false;
				
				for (unsigned int kkk = 0; kkk < activeFrameIndices.size(); kkk++) {
					if (featureTrackVector.at(iii).locations.at(jjj).imageIndex == activeFrameIndices.at(kkk)) {
						isPresent = true;
						break;
					}
				}
				
				if (!isPresent) {
					featureTrackVector.at(iii).locations.erase(featureTrackVector.at(iii).locations.begin() + jjj);
					jjj--;
				}
				
			}
			
		}
	}
}

int findTrackPosition(const vector<featureTrack>& featureTrackVector, long int index) {
	
	if (featureTrackVector.size() == 0) return -1;
	int retVal = -1;
	
	for (unsigned int iii = 0; iii < featureTrackVector.size(); iii++) {
		if (featureTrackVector.at(iii).trackIndex == index) {
			retVal = int(iii);
			//printf("%s << index = (%d); featureTrackVector.at(%d).trackIndex = (%d), retVal = (%d)\n", __FUNCTION__, index, iii, featureTrackVector.at(iii).trackIndex, retVal);
			break;
		}
	}
	return retVal;
}

bool featureTrack::occursInSequence(const vector<unsigned int>& indices) {
	
	for (unsigned int iii = 0; iii < locations.size(); iii++) {
		for (unsigned int jjj = 0; jjj < indices.size(); jjj++) {	
			if (locations.at(iii).imageIndex == indices.at(jjj)) return true;
		}
	}
	return false;
}

cv::Point2f featureTrack::getCoord(unsigned int cam_idx) {
	
	cv::Point2f blankPt(-1.0, -1.0);
	for (unsigned int iii = 0; iii < locations.size(); iii++) {
		
		if (((unsigned int) locations.at(iii).imageIndex) == cam_idx) {
			blankPt.x = locations.at(iii).featureCoord.x;
			blankPt.y = locations.at(iii).featureCoord.y;
			return blankPt;
		}
	}
	return blankPt;
}

void assignEstimatesBasedOnVelocities(const vector<featureTrack>& featureTrackVector, const vector<cv::Point2f>& pts1, vector<cv::Point2f>& pts2, unsigned int idx, double time1, double time2) {
	
	for (unsigned int iii = 0; iii < pts1.size(); iii++) {
		cv::Point2f velocity(0.0,0.0);
		
		for (unsigned int jjj = 0; jjj < featureTrackVector.size(); jjj++) {
			if (featureTrackVector.at(jjj).locations.at(featureTrackVector.at(jjj).locations.size()-1).imageIndex == idx) {
				if (featureTrackVector.at(jjj).locations.at(featureTrackVector.at(jjj).locations.size()-1).featureCoord.x == pts1.at(iii).x) {
					if (featureTrackVector.at(jjj).locations.at(featureTrackVector.at(jjj).locations.size()-1).featureCoord.y == pts1.at(iii).y) {
					
						velocity.x = float(featureTrackVector.at(jjj).velocity_x);
						velocity.y = float(featureTrackVector.at(jjj).velocity_y);
						
						//printf("%s << Found feature (%f, %f) (idx = %d) with velocity (%f, %f).\n", __FUNCTION__, pts1.at(iii).x, pts1.at(iii).y, idx, velocity.x, velocity.y);
						// printf("%s << Found feature (%f, %f) (idx = %d) with velocity (%f, %f).\n", __FUNCTION__, pts1.at(iii).x, pts1.at(iii).y, idx, velocity.x, velocity.y);
						break;
					}
				}
			}
		}
		
		if (abs(time1 - time2) < MAX_TIME_DIFF_FOR_PREDICTION) {
			cv::Point2f newLoc;
		
			newLoc.x = float(pts1.at(iii).x + (time2-time1)*velocity.x);
			newLoc.y = float(pts1.at(iii).y + (time2-time1)*velocity.y);
			
			pts2.push_back(newLoc);
		} else {
			pts2.push_back(pts1.at(iii));
		}

		//printf("%s << Predicted location of (%f, %f) is (%f, %f)\n", __FUNCTION__, pts1.at(iii).x, pts1.at(iii).y, pts2.at(iii).x, pts2.at(iii).y);
	}
}

void addProjectionsToVector(vector<featureTrack>& featureTrackVector, unsigned int index, vector<cv::Point2f>& points, long int &starting_track_index, double minSeparation) {

	for (int iii = 0; iii < int(points.size()-1); iii++) {
		for (int jjj = iii+1; jjj < int(points.size()); jjj++) {
			double dist = pow(pow(points.at(iii).x - points.at(jjj).x, 2.0)+pow(points.at(iii).y - points.at(jjj).y, 2.0),0.5);
			if (dist < minSeparation) {
				points.erase(points.begin()+iii);
				iii--;
				break;
			}

		}
	}

	for (unsigned int mmm = 0; mmm < points.size(); mmm++) addProjectionToVector(featureTrackVector, index, points.at(mmm), starting_track_index);
}

void addMatchesToVector(vector<featureTrack>& featureTrackVector, unsigned int index1, vector<cv::Point2f>& points1, unsigned int index2, vector<cv::Point2f>& points2, long int &starting_track_index, double minSeparation, bool debug) {	
	
	if (points1.size() != points2.size()) {
		printf("%s << ERROR! Vector lengths are not equal.\n", __FUNCTION__);
		return;
	}
	
	// Can you even just assume that both are in order of age?? I think that might be how it works...
	
	if (debug) { printf("%s << Points to add before proximity trimming = (%zu)\n", __FUNCTION__, points2.size()); }
	
	for (unsigned int jjj = 1; jjj < points2.size(); jjj++) {
		
		// So for point (jjj) you have found the 
						
		// Check violation:
		for (unsigned int kkk = 0; kkk < jjj; kkk++) {
			
			// Distance between this point and higher-ranked ones
			double dist = pow(pow(points2.at(kkk).x - points2.at(jjj).x, 2.0)+pow(points2.at(kkk).x - points2.at(jjj).x, 2.0),0.5);
			
			if (dist < minSeparation) {
				points1.erase(points1.begin()+jjj);
				points2.erase(points2.begin()+jjj);
				jjj--;
				break;
			}
		}
	}
	
	if (debug) { printf("%s << Points to add after proximity trimming = (%zu)\n", __FUNCTION__, points2.size()); }
	
	for (unsigned int mmm = 0; mmm < points1.size(); mmm++) {
		if (debug) { printf("%s << Adding match from index (%d) to (%d)..\n", __FUNCTION__, index1, index2); }
		addMatchToVector(featureTrackVector, index1, points1.at(mmm), index2, points2.at(mmm), starting_track_index);
	}
	
	if (debug) { printf("%s << Loop completed.\n", __FUNCTION__); }
}

void clearDangerFeatures(vector<featureTrack>& featureTrackVector, long int index) {
	
	//long int rev_index = max(((long int)100000000) - max(((long int)2147483648) - index, ((long int) 0)), ((long int) 0));
	
	long int buffer = 10000000;
	
	long int bottom_index, top_index;
	(index < buffer) ? bottom_index = 0 : bottom_index = index - buffer;
	((2147483647 - index) < buffer) ? top_index = 2147483647 : top_index = index + buffer;

	//printf("%s << (%d) < (%d) < (%d)\n", __FUNCTION__, bottom_index, index, top_index);
	
	for (unsigned int iii = 0; iii < featureTrackVector.size(); iii++) {
		if (featureTrackVector.at(iii).trackIndex > top_index) {
			featureTrackVector.erase(featureTrackVector.begin() + iii);
			iii--;
		} else if (featureTrackVector.at(iii).trackIndex < bottom_index) {
			featureTrackVector.erase(featureTrackVector.begin() + iii);
			iii--;
		}
	}
	
}

void addProjectionToVector(vector<featureTrack>& featureTrackVector, unsigned int index, const cv::Point2f& point, long int &starting_track_index) {
	
	// Go through all of the feature tracks to check if either of these features exist.
	
	bool foundFeature = false;

	// For each track
	for (unsigned int iii = 0; iii < featureTrackVector.size(); iii++) {
		
		// For each existing feature in the track
		for (unsigned int jjj = 0; jjj < featureTrackVector.at(iii).locations.size(); jjj++) {
			
			// If you've found a feature from the correct image
			if (featureTrackVector.at(iii).locations.at(jjj).imageIndex == index) {
				if (featureTrackVector.at(iii).locations.at(jjj).featureCoord == point) {
					foundFeature = true;
					break;
				}
			}
			
		}
		
		if (foundFeature) break;
	}
	
	if (!foundFeature) {
		
		starting_track_index++; 
		featureTrack newFeatureTrack;
		newFeatureTrack.trackIndex = starting_track_index;
		indexedFeature featToAdd(index, point);
		newFeatureTrack.addFeature(featToAdd);
		newFeatureTrack.velocity_x = 0.0;
		newFeatureTrack.velocity_y = 0.0;
		newFeatureTrack.firstOccurence = index;
		
		featureTrackVector.push_back(newFeatureTrack);
	}
	
}

int addMatchToVector(vector<featureTrack>& featureTrackVector, unsigned int index1, const cv::Point2f& point1, unsigned int index2, const cv::Point2f& point2, long int &starting_track_index, const cv::Point2f& velocity, bool debug) {
	
	int trackIndex = -1;
	
	// Go through all of the feature tracks to check if either of these features exist.
	
	bool everFoundFeature1or2 = false;

	// For each track
	for (unsigned int iii = 0; iii < featureTrackVector.size(); iii++) {
		
		bool foundFeature1 = false, foundFeature2 = false;
		bool foundImage1 = false, foundImage2 = false;
		
		// int foundIndex1 = -1, foundIndex2 = -1;

		// For each existing feature in the track
		for (unsigned int jjj = 0; jjj < featureTrackVector.at(iii).locations.size(); jjj++) {
			
			// If you've found a feature from the correct image
			if (featureTrackVector.at(iii).locations.at(jjj).imageIndex == index1) {
				foundImage1 = true;
				if (featureTrackVector.at(iii).locations.at(jjj).featureCoord == point1) foundFeature1 = true;
			}
			
			if (featureTrackVector.at(iii).locations.at(jjj).imageIndex == index2) {
				foundImage2 = true;
				if (featureTrackVector.at(iii).locations.at(jjj).featureCoord == point2) foundFeature2 = true;
			}
			
		}
		
		if (foundFeature1 && foundFeature2) {
			everFoundFeature1or2 = true;
		} else if (foundFeature1) {
			everFoundFeature1or2 = true;
			// add feature 2 if no feature from this image in track
			if (!foundImage2) {
				indexedFeature featToAdd(index2, point2);
				featureTrackVector.at(iii).addFeature(featToAdd);
			}
		} else if (foundFeature2) {
			everFoundFeature1or2 = true;
			// add feature 1 if no feature from this image in track
			if (!foundImage1) {
				indexedFeature featToAdd(index1, point1);
				featureTrackVector.at(iii).addFeature(featToAdd);
			}
			
		}
		
		if (everFoundFeature1or2) {
			if (debug) { printf("%s << Setting track (%d) velocity to (%f, %f)\n", __FUNCTION__, iii, velocity.x, velocity.y); }
			featureTrackVector.at(iii).velocity_x = velocity.x;
			featureTrackVector.at(iii).velocity_y = velocity.y;
			trackIndex = iii;
			break;
		}
		
	}
	
	if (!everFoundFeature1or2) {
		
		
		starting_track_index++;
		// If neither does, create a new track
		featureTrack newFeatureTrack;
		newFeatureTrack.trackIndex = starting_track_index;
		indexedFeature featToAdd1(index1, point1);
		newFeatureTrack.addFeature(featToAdd1);
		indexedFeature featToAdd2(index2, point2);
		newFeatureTrack.addFeature(featToAdd2);
		newFeatureTrack.velocity_x = velocity.x;
		newFeatureTrack.velocity_y = velocity.y;
		newFeatureTrack.firstOccurence = index1;
		featureTrackVector.push_back(newFeatureTrack);
		trackIndex = int(featureTrackVector.size())-1;
		
	}
	
	return trackIndex;
}

void drawFeatureTracks(cv::Mat& src, cv::Mat& dst, vector<featureTrack>& tracks, const cv::Scalar& tColor, const cv::Scalar& kColor, unsigned int index, unsigned int history) {
	
	cv::Point p1, p2;
	
	src.copyTo(dst);
	
	for (unsigned int iii = 0; iii < tracks.size(); iii++) {
		
		//printf("%s << DEBUG [%d] : %d\n", __FUNCTION__, iii, 0);
		
		if (tracks.at(iii).locations.size() < 1) continue;
		
		if (tracks.at(iii).locations.at(tracks.at(iii).locations.size()-1).imageIndex == index) {
			p1 = cv::Point(int(tracks.at(iii).locations.at(tracks.at(iii).locations.size()-1).featureCoord.x * 16.0), int(tracks.at(iii).locations.at(tracks.at(iii).locations.size()-1).featureCoord.y * 16.0));
			
			#ifdef _OPENCV_VERSION_3_PLUS_
			circle(dst, p1, 16, kColor, -1, cv::LINE_AA, 4);
			#else
			circle(dst, p1, 16, kColor, -1, CV_AA, 4);
			#endif

		} else continue;
		
		if (tracks.at(iii).locations.size() < 2) continue;
		
		for (int jjj = ((int)tracks.at(iii).locations.size())-2; jjj >= 0; jjj--) {
			
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
			if (((int)tracks.at(iii).locations.at(((unsigned int) jjj)).imageIndex) < max(((int)index)-((int) history), 0)) break;
#else
			if (((int)tracks.at(iii).locations.at(((unsigned int) jjj)).imageIndex) < std::max(((int)index)-((int) history), 0)) break;
#endif
			
			//printf("%s << DEBUG [%d][%d] : [%d][%d]\n", __FUNCTION__, iii, jjj, tracks.at(iii).locations.at(((unsigned int) jjj)).imageIndex, index);
			
			p1 = cv::Point(int(tracks.at(iii).locations.at(((unsigned int) jjj)+1).featureCoord.x * 16.0), int(tracks.at(iii).locations.at(((unsigned int) jjj)+1).featureCoord.y * 16.0));
			p2 = cv::Point(int(tracks.at(iii).locations.at(((unsigned int) jjj)).featureCoord.x * 16.0), int(tracks.at(iii).locations.at(((unsigned int) jjj)).featureCoord.y * 16.0));
	
#ifdef _OPENCV_VERSION_3_PLUS_
			cv::line(dst, p1, p2, tColor, 1, cv::LINE_AA, 4);
#else
			cv::line(dst, p1, p2, tColor, 1, CV_AA, 4);
#endif
		}
		
	}
	
	
}

void redistortTracks(const vector<featureTrack>& src, vector<featureTrack>& dst, const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs, unsigned int latest, const cv::Mat& newCamMat, unsigned int history) {
	
	dst.clear();
	dst.insert(dst.end(), src.begin(), src.end());
	
	vector<cv::Point2f> tempPoints, newPoints;
	
	for (unsigned int iii = 0; iii < src.size(); iii++) {
		
		if (src.at(iii).locations.at(src.at(iii).locations.size()-1).imageIndex != ((int) latest)) continue;
		
		//printf("%s << continuing...\n", __FUNCTION__);
		
		newPoints.clear();
		tempPoints.clear();
		
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
		for (int jjj = int(src.at(iii).locations.size())-1; jjj >= ((int) max((int(src.at(iii).locations.size())) - ((int) history), ((int) 0))); jjj--) {
			tempPoints.push_back(src.at(iii).locations.at(jjj).featureCoord);
		}
#else
		for (int jjj = int(src.at(iii).locations.size())-1; jjj >= ((int) std::max((int(src.at(iii).locations.size())) - ((int) history), ((int) 0))); jjj--) {
			tempPoints.push_back(src.at(iii).locations.at(jjj).featureCoord);
		}
#endif

		redistortPoints(tempPoints, newPoints, cameraMatrix, distCoeffs, newCamMat);
		
		unsigned int currIndex = 0;

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
		for (int jjj = int(src.at(iii).locations.size())-1; jjj >= ((int) max((int(src.at(iii).locations.size())) - ((int) history), ((int) 0))); jjj--) {
			dst.at(iii).locations.at(jjj).featureCoord = newPoints.at(currIndex++);
		}
#else
		for (int jjj = int(src.at(iii).locations.size())-1; jjj >= ((int) std::max((int(src.at(iii).locations.size())) - ((int) history), ((int) 0))); jjj--) {
			dst.at(iii).locations.at(jjj).featureCoord = newPoints.at(currIndex++);
		}
#endif

	}
}

bool createTrackMatrix(const vector<featureTrack>& src, cv::Mat& dst, int latest) {
	
	unsigned int rows = 60, cols = 160;
	cv::Mat tmp(rows, cols, CV_8UC3);
	
	for (int iii = 0; iii < tmp.rows; iii++) {
		for (int jjj = 0; jjj < tmp.cols; jjj++) {
			tmp.at<cv::Vec3b>(iii,jjj) = cv::Vec3b(255,255,255);
		}
	}
	
	int earliest = -1;

	// Determine the latest frame index
	if (latest == -1) {

		for (unsigned int iii = 0; iii < src.size(); iii++) {
			
			// printf("%s << iii = (%d / %d) : (%d)\n", __FUNCTION__, iii, src.size(), src.at(iii).locations.size());
			
			if (src.at(iii).locations.size() > 0) {
				if (((int) src.at(iii).locations.at(src.at(iii).locations.size()-1).imageIndex) > latest) {
					// printf("%s << imageIndex = (%d)\n", __FUNCTION__, src.at(iii).locations.at(src.at(iii).locations.size()-1).imageIndex);
					latest = src.at(iii).locations.at(src.at(iii).locations.size()-1).imageIndex;
				}
				
				if ( (earliest == -1) || ((int) src.at(iii).locations.at(src.at(iii).locations.size()-1).imageIndex) < earliest) {
					earliest = src.at(iii).locations.at(src.at(iii).locations.size()-1).imageIndex;
				}
			}
			
			
		}
	}
	
	vector<int> activeTrackIndices;
	
	// Determine which tracks in the feature track structure were identified in latest frame
	for (unsigned int iii = 0; iii < src.size(); iii++) {
		if (src.at(iii).locations.size() > 0) {
			if (src.at(iii).locations.at(src.at(iii).locations.size()-1).imageIndex == ((int) latest)) {
				activeTrackIndices.push_back(iii);
			}
		}
	}
	
	// Colorize the pixels in the debug image
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
	for (unsigned int iii = 0; iii < min((unsigned int)rows, ((unsigned int) activeTrackIndices.size())); iii++) {
		for (unsigned int jjj = 0; jjj < min((unsigned int)cols, ((unsigned int)src.at(activeTrackIndices.at(iii)).locations.size())); jjj++) {
#else
	for (unsigned int iii = 0; iii < std::min((unsigned int)rows, ((unsigned int) activeTrackIndices.size())); iii++) {
		for (unsigned int jjj = 0; jjj < std::min((unsigned int)cols, ((unsigned int)src.at(activeTrackIndices.at(iii)).locations.size())); jjj++) {
#endif
			unsigned int column = latest - src.at(activeTrackIndices.at(iii)).locations.at(jjj).imageIndex;
			
			if (column < cols) {
				tmp.at<cv::Vec3b>(iii,column)[1] = 0;
				tmp.at<cv::Vec3b>(iii,column)[2] = 0;				
			}
		}
	}

	resize(tmp, dst, cv::Size(4*tmp.cols, 4*tmp.rows), 0.0, 0.0, cv::INTER_NEAREST);
	return true;
}

void assignHistoricalPoints(const vector<featureTrack>& src, unsigned int idx_1, unsigned int idx_2, vector<cv::Point2f>& dst) {
	
	//if ((src.size() > 0) && (src.at(0).locations.size() > 0)) { printf("%s << First occurence of oldest track = (%d)\n", __FUNCTION__, src.at(0).locations.at(0).imageIndex); }
	
	for (unsigned int iii = 0; iii < src.size(); iii++) {
		
		if (src.at(iii).locations.at(src.at(iii).locations.size()-1).imageIndex == idx_2) continue;
		
		for (unsigned int jjj = 0; jjj < src.at(iii).locations.size(); jjj++) {
			
			//printf("%s << (%d : %d) vs (%d : %d)\n", __FUNCTION__, src.at(iii).locations.at(jjj).imageIndex, idx_1, src.at(iii).locations.at(src.at(iii).locations.size()-1).imageIndex, idx_2);
			
			if (src.at(iii).locations.at(jjj).imageIndex == idx_1) dst.push_back(src.at(iii).locations.at(jjj).featureCoord);
		}
	}
}

void reduceVectorsToTrackedPoints(const vector<cv::Point2f>& points1, vector<cv::Point2f>& trackedPoints1, const vector<cv::Point2f>& points2, vector<cv::Point2f>& trackedPoints2, vector<uchar>& statusVec) {
	
	trackedPoints1.clear();
	trackedPoints2.clear();
	
	for (unsigned int iii = 0; iii < statusVec.size(); iii++) {
		if (statusVec.at(iii) > 0) {
			trackedPoints1.push_back(points1.at(iii));
			trackedPoints2.push_back(points2.at(iii));
		}
	}
}

void getPointsFromTracks(vector<featureTrack>& tracks, vector<cv::Point2f>& pts1, vector<cv::Point2f>& pts2, int idx1, int idx2) {
	
	// Could make more efficient by only testing tracks that are long enough, but
	// If there are missing frames (acceptable?) then the simple way of achieving this won't work)
	
	//printf("%s << searching for indices (%d) & (%d) [tracks.size() = (%d)]\n", __FUNCTION__, idx1, idx2, tracks.size());
	
	for (unsigned int iii = 0; iii < tracks.size(); iii++) {
		
		//printf("%s << Searching track (%d)...\n", __FUNCTION__, iii);
		
		for (unsigned int kkk = 0; kkk < tracks.at(iii).locations.size(); kkk++) {
			
			//printf("%s << Searching element (%d) ; (%d)\n", __FUNCTION__, kkk, tracks.at(iii).locations.at(kkk).imageIndex);
			
			//if (tracks.at(iii).locations.at(kkk).imageIndex == 0) {
				//printf("%s << FOUND A ZERO TRACK!!!\n", __FUNCTION__);
			//}
			
			// If the track extends between the two images of interest
			if (((int)tracks.at(iii).locations.at(kkk).imageIndex) == idx1) {
				
				//printf("%s << Found index A (%d)...\n", __FUNCTION__, idx1);
				
				for (unsigned int jjj = 0; jjj < tracks.at(iii).locations.size(); jjj++) {
					
					//printf("%s << Searching element (%d)...\n", __FUNCTION__, jjj);
					
					if (((int)tracks.at(iii).locations.at(jjj).imageIndex) == idx2) {
						
						//printf("%s << Found match!\n", __FUNCTION__);
						
						pts1.push_back(tracks.at(iii).locations.at(kkk).featureCoord);
						pts2.push_back(tracks.at(iii).locations.at(jjj).featureCoord);
						break;
					}
				}
				
			}
		}
	}
}

double getFeatureMotion(vector<featureTrack>& tracks, vector<unsigned int>& indices, unsigned int idx_1, unsigned int idx_2) {

	vector<double> distances, distances_x, distances_y;

	for (unsigned int iii = 0; iii < indices.size(); iii++) {
		
		for (unsigned int jjj = 0; jjj < tracks.at(indices.at(iii)).locations.size()-1; jjj++) {
			
			if (tracks.at(indices.at(iii)).locations.at(jjj).imageIndex == int(idx_1)) {
				
				for (unsigned int kkk = jjj+1; kkk < tracks.at(indices.at(iii)).locations.size(); kkk++) {
					
					if (tracks.at(indices.at(iii)).locations.at(kkk).imageIndex == int(idx_2)) {
						
						double dist = distanceBetweenPoints(tracks.at(indices.at(iii)).locations.at(jjj).featureCoord, tracks.at(indices.at(iii)).locations.at(kkk).featureCoord);
						
						double dist_x = tracks.at(indices.at(iii)).locations.at(jjj).featureCoord.x - tracks.at(indices.at(iii)).locations.at(kkk).featureCoord.x;
						double dist_y = tracks.at(indices.at(iii)).locations.at(jjj).featureCoord.y - tracks.at(indices.at(iii)).locations.at(kkk).featureCoord.y;
												
						distances_x.push_back(dist_x);
						distances_y.push_back(dist_y);
						distances.push_back(dist);
						
					}
					
				}
				
			}
			
			
		}
		
	}
	
	double mean_dist = 0.00, std_dev = 0.00, mean_x = 0.00, mean_y = 0.00, std_x = 0.00, std_y = 0.00;
	
	for (unsigned int iii = 0; iii < distances.size(); iii++) {
		mean_dist += (distances.at(iii) / ((double) distances.size()));
		mean_x += (distances_x.at(iii) / ((double) distances.size()));
		mean_y += (distances_y.at(iii) / ((double) distances.size()));


	}
	
	for (unsigned int iii = 0; iii < distances.size(); iii++) {
		std_dev += pow(distances.at(iii) - mean_dist, 2.0);
		std_x += pow(distances_x.at(iii) - mean_x, 2.0);
		std_y += pow(distances_y.at(iii) - mean_y, 2.0);		
	}
		
	std_dev /= distances.size();
	std_x /= distances.size();
	std_y /= distances.size();

	pow(std_dev, 0.5);
	pow(std_x, 0.5);
	pow(std_y, 0.5);

	
	//printf("%s << (%f, %f, %f) : (%f, %f, %f)\n", __FUNCTION__, mean_dist, mean_y, mean_x, std_dev, std_x, std_y);
	
	// Need to have some variation in x OR y translation, otherwise it could just be a homography
	return ((std_x + std_y) / 2.0);
	
	return mean_dist;
	
}

void findRelevantIndices(vector<featureTrack>& tracks, vector<unsigned int>& triangulated, vector<unsigned int>& untriangulated, unsigned int last_index, unsigned int new_index) {
	
	for (unsigned int iii = 0; iii < tracks.size(); iii++) {
		(tracks.at(iii).isTriangulated) ? triangulated.push_back(iii) : untriangulated.push_back(iii);
	}
}
