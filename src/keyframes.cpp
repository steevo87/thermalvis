/*! \file	keyframes.cpp
 *  \brief	Definitions for calculations and management relating to keyframes (for loop closure).
*/

#include "keyframes.hpp"

bool getValidLocalMaxima(cv::Mat& scores, unsigned int last_iii, unsigned int last_jjj, unsigned int& opt_iii, unsigned int& opt_jjj) {

	// First, find the cell/s that could be optimized now
	
	if (last_iii == 0) {
		return false;
	}
	
	bool foundMaxima = false;
	
	
	// make sure you never select it if it's value is zero - or do that check in the main prog?
	
	
	// cell to the above left is a candidate
	double candidateScore = scores.at<double>(last_iii-1, last_jjj-1);
	
	foundMaxima = true;

	#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
		for (unsigned int aaa = ((unsigned int) max(((int) last_iii)-2,0)); ((int)aaa) < min(((int) last_iii),(int)scores.cols); aaa++) {
			for (unsigned int bbb = ((unsigned int) max(((int) last_jjj)-2,0)); ((int)bbb) < min(((int) last_jjj),(int)scores.rows); bbb++) {
				if (scores.at<double>(aaa,bbb) > candidateScore) {
					foundMaxima = false;
				}
			}
		}
	#else
		for (unsigned int aaa = ((unsigned int) std::max(((int) last_iii)-2,0)); ((int)aaa) < std::min(((int) last_iii),(int)scores.cols); aaa++) {
			for (unsigned int bbb = ((unsigned int) std::max(((int) last_jjj)-2,0)); ((int)bbb) < std::min(((int) last_jjj),(int)scores.rows); bbb++) {
				if (scores.at<double>(aaa,bbb) > candidateScore) {
					foundMaxima = false;
				}
			}
		}
	#endif

	
	
	double bestCandidateScore = 0.00;
	
	if (foundMaxima == true) {
		bestCandidateScore = candidateScore;
		opt_iii = last_iii-1;
		opt_jjj = last_jjj-1;
	}
	
	if (last_jjj == (scores.cols-1)) {
		foundMaxima = true;
		
		// cell to the direct above is a candidate
		candidateScore = scores.at<double>(last_iii-1, last_jjj);

		#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
			for (unsigned int aaa = ((unsigned int) max(((int) last_iii)-2,0)); ((int)aaa) < min(((int) last_iii),(int)scores.cols); aaa++) {
				for (unsigned int bbb = ((unsigned int) max(((int) last_jjj)-1,0)); ((int)bbb) < min(((int) last_jjj)+1,(int)scores.rows); bbb++) {
				
					if (scores.at<double>(aaa,bbb) > candidateScore) {
						foundMaxima = false;
					}
				
				}
			}
		#else
			for (unsigned int aaa = ((unsigned int) std::max(((int) last_iii)-2,0)); ((int)aaa) < std::min(((int) last_iii),(int)scores.cols); aaa++) {
				for (unsigned int bbb = ((unsigned int) std::max(((int) last_jjj)-1,0)); ((int)bbb) < std::min(((int) last_jjj)+1,(int)scores.rows); bbb++) {
				
					if (scores.at<double>(aaa,bbb) > candidateScore) {
						foundMaxima = false;
					}
				
				}
			}
		#endif
		
		
		
		if (foundMaxima == true) {
			if (candidateScore > bestCandidateScore) {
				bestCandidateScore = candidateScore;
				opt_iii = last_iii-1;
				opt_jjj = last_jjj;
			}
			
		}
		
	}
	
	if ((last_jjj == (scores.cols-1)) && (last_iii == (scores.rows-2))) {
		foundMaxima = true;
		
		// current cell is a candidate
		candidateScore = scores.at<double>(last_iii, last_jjj);

		#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
			for (unsigned int aaa = ((unsigned int) max(((int) last_iii)-1,0)); ((int)aaa) < min(((int) last_iii)+1,(int)scores.cols); aaa++) {
				for (unsigned int bbb = ((unsigned int) max(((int) last_jjj)-1,0)); ((int)bbb) < min(((int) last_jjj)+1,(int)scores.rows); bbb++) {
				
					if (scores.at<double>(aaa,bbb) > candidateScore) {
						foundMaxima = false;
					}
				
				}
			}
		#else
			for (unsigned int aaa = ((unsigned int) std::max(((int) last_iii)-1,0)); ((int)aaa) < std::min(((int) last_iii)+1,(int)scores.cols); aaa++) {
				for (unsigned int bbb = ((unsigned int) std::max(((int) last_jjj)-1,0)); ((int)bbb) < std::min(((int) last_jjj)+1,(int)scores.rows); bbb++) {
				
					if (scores.at<double>(aaa,bbb) > candidateScore) {
						foundMaxima = false;
					}
				
				}
			}
		#endif
		
		
		
		if (foundMaxima == true) {
			if (candidateScore > bestCandidateScore) {
				bestCandidateScore = candidateScore;
				opt_iii = last_iii;
				opt_jjj = last_jjj;
			}
			
		}
	}
	
	//printf("%s << opt_iii = %d; opt_jjj = %d\n", __FUNCTION__, opt_iii, opt_jjj);
	
	if (bestCandidateScore > 0.00) {
		return true;
	} else {
		return false;
	}
	
}

void keyframeStore::findStrongConnections(int idx, vector<unsigned int>& cIndices) {
	
	bool connectionsAllFound = false;
	
	vector<unsigned int> kIndices;
	kIndices.push_back(idx);
	
	//printf("%s << Entered. connections.size() = %d\n", __FUNCTION__, connections.size());
	
	while (!connectionsAllFound) {
		
		connectionsAllFound = true;
		
		for (unsigned int iii = 0; iii < connections.size(); iii++) {
			
			//printf("%s << Connection between (%d) and (%d) [searching for %d]\n", __FUNCTION__, connections.at(iii).idx1, connections.at(iii).idx2, idx);
			
			bool connectionAlreadyAdded = false;
			
			for (unsigned int jjj = 0; jjj < cIndices.size(); jjj++) {
			
				if (cIndices.at(jjj) == iii) {
					connectionAlreadyAdded = true;
				}
				
			}
			
			if (connectionAlreadyAdded == true) {
				continue;
			}
			
			for (unsigned int jjj = 0; jjj < kIndices.size(); jjj++) {
				if ((connections.at(iii).idx1 == kIndices.at(jjj)) || (connections.at(iii).idx2 == kIndices.at(jjj))) {
					
					bool connection_index_added = false;
					for (unsigned int kkk = 0; kkk < cIndices.size(); kkk++) {
						if (iii == cIndices.at(kkk)) {
							connection_index_added = true;
						}
					}
					
					if (connection_index_added) {
						break;
					}
					
					cIndices.push_back(iii);
					connectionsAllFound = false;
					
					// Then only add these keyframe indices if they haven't been added already..
					
					bool alreadyAdded1 = false, alreadyAdded2 = false;
					for (unsigned int kkk = 0; kkk < kIndices.size(); kkk++) {
						
						if (kIndices.at(kkk) == connections.at(iii).idx1) {
							alreadyAdded1 = true;
						}
						
						if (kIndices.at(kkk) == connections.at(iii).idx2) {
							alreadyAdded2 = true;
						}
						
					}
					
					if (!alreadyAdded1) {
						kIndices.push_back(connections.at(iii).idx1);
					}
					
					if (!alreadyAdded2) {
						kIndices.push_back(connections.at(iii).idx2);
					}

				}
			}
			
			
		}
	}
	
	sort(kIndices.begin(), kIndices.end());
	
	cIndices.clear();
	
	for (unsigned int iii = 0; iii < connections.size(); iii++) {
		
		for (unsigned int jjj = 0; jjj < kIndices.size(); jjj++) {
			
			if (connections.at(iii).idx1 == kIndices.at(jjj)) {
				cIndices.push_back(iii);
				break;
			}
			
			if (connections.at(iii).idx2 == kIndices.at(jjj)) {
				cIndices.push_back(iii);
				break;
			}
			
		}	
	}

}

void keyframe::detectKeypoints(cv::Ptr<cv::FeatureDetector>& detector) {
	detector -> detect(im, keypoints);
	sortKeyPoints(keypoints, MAXIMUM_FEATURES_PER_FRAME);
}

void keyframe::extractDescriptors(cv::Ptr<cv::DescriptorExtractor>& extractor) {
	extractor->compute(im, keypoints, descriptors);
}

keyframeStore::keyframeStore() {
	count = 0;
}

bool keyframeStore::getBestPair(int& idx1, int& idx2) {

	return true;
	
}

void keyframeStore::addKeyframe(int idx, cv::Mat& image) {
	
	keyframe newFrame;
	
	newFrame.idx = idx;
	image.copyTo(newFrame.im);
	
	//newFrame.pose = Mat::eye(4, 4, CV_64FC1); 
	newFrame.poseDetermined = false;
	
	keyframes.push_back(newFrame);
	
	
	
	count++;
	
}

void keyframeStore::addConnection(int idx1, int idx2, int type, cv::Mat rel) {
	
	connection cnct;
	cnct.idx1 = idx1;
	cnct.idx2 = idx2;
	cnct.type = type;
	rel.copyTo(cnct.relation);
	cnct.processed = false;
	
	connections.push_back(cnct);
	
}

void keyframeStore::findMatches() {
	
	
	for (unsigned int iii = 0; iii < keyframes.size()-1; iii++) {
		
		for (unsigned int jjj = iii+1; jjj < keyframes.size(); jjj++) {
			
			bool connectionExists = false;
			int idx;
			
			for (unsigned int kkk = 0; kkk < connections.size(); kkk++) {
				
				if ((connections.at(kkk).idx1 == iii) && (connections.at(kkk).idx2 == jjj)) {
					
					idx = kkk;
					connectionExists = true;
					continue;
					
				}
				
			}
			
			if (!connectionExists) {
				
				connection cnct;
				cnct.idx1 = iii;
				cnct.idx2 = jjj;
				cnct.type = KF_CONNECTION_WEAK;
				
				createMatchingMatrix(cnct.matchingMatrix, keyframes.at(iii).descriptors, keyframes.at(jjj).descriptors);
				
				constrainMatchingMatrix(cnct.matchingMatrix, keyframes.at(iii).keypoints, keyframes.at(jjj).keypoints, MATCHING_DIST_CONSTRAINT, MATCHING_SIZE_CONSTRAINT);
				
				twoWayPriorityMatching(cnct.matchingMatrix, cnct.matches1to2);
				
				sortMatches(cnct.matches1to2);
				
				// 0.5 for ratio prioritization
				filterMatches(cnct.matches1to2, 0.5);
				
				connections.push_back(cnct);
				
			} else if (connections.at(idx).type == KF_CONNECTION_WEAK) {
				
				createMatchingMatrix(connections.at(idx).matchingMatrix, keyframes.at(iii).descriptors, keyframes.at(jjj).descriptors);
				
				constrainMatchingMatrix(connections.at(idx).matchingMatrix, keyframes.at(iii).keypoints, keyframes.at(jjj).keypoints, MATCHING_DIST_CONSTRAINT, MATCHING_SIZE_CONSTRAINT);
				
				twoWayPriorityMatching(connections.at(idx).matchingMatrix, connections.at(idx).matches1to2);
				
				sortMatches(connections.at(idx).matches1to2);
				
				// 0.5 for ratio prioritization
				filterMatches(connections.at(idx).matches1to2, 0.5);
				
			}
			
		}
		
	}
	
}

