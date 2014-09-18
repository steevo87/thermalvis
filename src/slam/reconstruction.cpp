/*! \file	reconstruction.cpp
 *  \brief	Definitions for triangulation and other recovery-of-3D structure functions.
*/

#ifdef _USE_PCL_

#include "slam/reconstruction.hpp"

void summarizeTransformation(const cv::Mat& C, char *summary) {

	cv::Mat P, R, Rv, t;
	transformationToProjection(C, P);
	decomposeTransform(C, R, t);
	Rodrigues(R, Rv);
	
	double unitsDist, degreesRot;
	unitsDist = getDistanceInUnits(t);
	degreesRot = getRotationInDegrees(R);
	
	sprintf(summary, "%f units (%f, %f, %f); %f degrees", unitsDist, t.at<double>(0,0), t.at<double>(1,0), t.at<double>(2,0), degreesRot);
	
}

int findBestCandidate(const cv::Mat *CX, const cv::Mat& K, const vector<cv::Point2f>& pts1, const vector<cv::Point2f>& pts2, cv::Mat& C) {
	
	int bestScore = 0, bestCandidate = 0;

	for (int kkk = 0; kkk < 4; kkk++) {

		int validPts = pointsInFront(CX[kkk], K, pts1, pts2);
		
		if (validPts > bestScore) {
			bestScore = validPts;
			bestCandidate = kkk;
		}
		
		//printf("%s << validPts(%d) = %d\n", __FUNCTION__, kkk, validPts);
	}
	
	CX[bestCandidate].copyTo(C);
	
	return bestScore;
}

void reverseTranslation(cv::Mat& C) {
	
	for (unsigned int iii = 0; iii < 3; iii++) {
		C.at<double>(iii,3) = -C.at<double>(iii,3);
	}
	
}



void findFeaturesForPoints(vector<featureTrack>& tracks, vector<cv::Point2f>& pts1, vector<cv::Point2f>& pts2, vector<cv::Point3d>& pts3d, int idx1, int idx2) {

	// For each 3d point, find the track that contains it, then find the projections...
	for (unsigned int jjj = 0; jjj < pts3d.size(); jjj++) {
		
		//printf("%s << jjj = %d (%f, %f, %f)\n", __FUNCTION__, jjj, pts3d.at(jjj).x, pts3d.at(jjj).y, pts3d.at(jjj).z);
		
		for (unsigned int iii = 0; iii < tracks.size(); iii++) {

			//printf("%s << iii = %d\n", __FUNCTION__, iii);
			
			// MONKEY!!
			if (tracks.at(iii).get3dLoc() == pts3d.at(jjj)) {
				
				//printf("%s << MATCH! (%d, %d)\n", __FUNCTION__, jjj, iii);
				
				for (unsigned int kkk = 0; kkk < tracks.at(iii).locations.size(); kkk++) {
					
					//printf("%s << kkk = %d (%d, %d)\n", __FUNCTION__, kkk, tracks.at(iii).locations.at(kkk).imageIndex, tracks.at(iii).locations.at(kkk).imageIndex);
			
					if (((int)tracks.at(iii).locations.at(kkk).imageIndex) == idx1) {
						
						pts1.push_back(tracks.at(iii).locations.at(kkk).featureCoord);
						
					} else if (((int)tracks.at(iii).locations.at(kkk).imageIndex) == idx2) {
						
						pts2.push_back(tracks.at(iii).locations.at(kkk).featureCoord);
						
					}
					
				}
				
			}
			
		}
		
		
	}
	
}

int countTriangulatedTracks(const vector<featureTrack>& tracks) {
	
	int triangulatedCount = 0;
	
	for (unsigned int iii = 0; iii < tracks.size(); iii++) {
		
		if (tracks.at(iii).isTriangulated) {
			triangulatedCount++;
		}
	}
	
	return triangulatedCount;
	
}

int countActiveTriangulatedTracks(vector<unsigned int>& indices, vector<featureTrack>& tracks) {
	
	int triangulatedCount = 0;
	
	for (unsigned int iii = 0; iii < indices.size(); iii++) {
		
		if (tracks.at(indices.at(iii)).isTriangulated) {
			triangulatedCount++;
		}
		
	}
	
	return triangulatedCount;
	
}

void getIndicesForTriangulation(vector<unsigned int>& dst, vector<unsigned int>& src, vector<unsigned int>& already_triangulated) {
	
	dst.clear();
	
	for (unsigned int iii = 0; iii < src.size(); iii++) {
		
		bool triangulated = false;
		
		for (unsigned int jjj = 0; jjj < already_triangulated.size(); jjj++) {
			
			if (already_triangulated.at(jjj) == src.at(iii)) {
				triangulated = true;
				continue;
			}

			
		}
		
		if (!triangulated) {
			
			dst.push_back(src.at(iii));
			
		}

		
	}
	
}

void reconstructSubsequence(vector<featureTrack>& tracks, vector<cv::Point3d>& ptCloud, int idx1, int idx2) {
	
}

void removeShortTracks(vector<featureTrack>& tracks, int idx1, int idx2) {
	
	int minLength = (idx2 - idx1) / 2;
	
	for (unsigned int iii = 0; iii < tracks.size(); iii++) {
		int trackLength = tracks.at(iii).locations.size();
		if (trackLength < minLength) {
			int final_image = tracks.at(iii).locations.at(trackLength-1).imageIndex;
			
			if ((final_image >= idx1) && (final_image < idx2)) {
				tracks.erase(tracks.begin() + iii);
				iii--;
			}
			
		}
	}
	
}

#ifdef _USE_SBA_
void updateSystemTracks(SysSBA& sys, vector<featureTrack>& tracks, unsigned int start_index) {
	
	//sys = SysSBA();
	
	sys.tracks.clear();

	
	for (unsigned int iii = 0; iii < tracks.size(); iii++) {
		
		//printf("%s << Checking track (%d) (size = %d)\n", __FUNCTION__, iii, tracks.at(iii).locations.size());
		
		if (tracks.at(iii).isTriangulated) {

			
			//printf("%s << Track is triangulated\n", __FUNCTION__);
			
			Vector4d temppoint(tracks.at(iii).get3dLoc().x, tracks.at(iii).get3dLoc().y, tracks.at(iii).get3dLoc().z, 1.0);
			sys.addPoint(temppoint);
			
			//printf("%s << Point added; nodes = %d\n", __FUNCTION__, sys.nodes.size());
			//printf("%s << sys.tracks.size() = %d\n", __FUNCTION__, sys.tracks.size());
			
			// sys.tracks.size()-1
			
	
			for (unsigned int jjj = 0; jjj < tracks.at(iii).locations.size(); jjj++) {
				
				// Only want to add projections for active nodes
				
				if ((((unsigned int) tracks.at(iii).locations.at(jjj).imageIndex) >= start_index) && (((unsigned int) tracks.at(iii).locations.at(jjj).imageIndex) < (start_index+sys.nodes.size()))) {
					//printf("%s << Assembling projection (%d)(%d)\n", __FUNCTION__, iii, jjj);
				
					Vector2d proj;
					
					proj.x() = tracks.at(iii).locations.at(jjj).featureCoord.x;
					proj.y() = tracks.at(iii).locations.at(jjj).featureCoord.y;
					
					//printf("%s << Adding projection (track: %d)(loc: %d) node #: [%d]\n", __FUNCTION__, iii, jjj, tracks.at(iii).locations.at(jjj).imageIndex);
					
					//printf("%s << vec size (tracks: %d, nodes for this track: %d)\n", __FUNCTION__, tracks.size(), tracks.at(iii).locations.size());
					//printf("%s << sys size (tracks: %d, nodes: %d)\n", __FUNCTION__, sys.tracks.size(), sys.nodes.size());
					
					// If you only have 30 nodes, then how are you adding a track that has an image index of 53?
					
					//printf("%s << Adding mono projection: (cam: %d / %d, track: %d / %d) = (%f, %f)\n", __FUNCTION__, tracks.at(iii).locations.at(jjj).imageIndex, sys.nodes.size(), sys.tracks.size()-1, sys.tracks.size(), proj.x(), proj.y());
					sys.addMonoProj(tracks.at(iii).locations.at(jjj).imageIndex-start_index, sys.tracks.size()-1, proj);
					
					//printf("%s << Added.\n", __FUNCTION__);
					
					// 0, 53, proj
				}
				
				
				
			}
			
			//printf("%s << Projections added\n", __FUNCTION__);
		
		}
		
	}
	
}
#endif

void updateTriangulatedPoints(vector<featureTrack>& tracks, vector<unsigned int>& indices, vector<cv::Point3d>& cloud) {
	
	if (cloud.size() != indices.size()) {
		printf("%s << ERROR! Cloud size and index list size don't match (%d vd %d)\n", __FUNCTION__, ((int)cloud.size()), ((int)indices.size()));
	}
	
	for (unsigned int iii = 0; iii < indices.size(); iii++) {
		
		if (tracks.at(indices.at(iii)).isTriangulated) {
			//printf("%s << (%d) is triangulated; moving from (%f, %f, %f) to (%f, %f, %f)\n", __FUNCTION__, iii, tracks.at(indices.at(iii)).get3dLoc().x, tracks.at(indices.at(iii)).get3dLoc().y, tracks.at(indices.at(iii)).get3dLoc().z, cloud.at(iii).x, cloud.at(iii).y, cloud.at(iii).z);
		}
		
		tracks.at(indices.at(iii)).set3dLoc(cloud.at(iii));
		
		
		
	}
	
}

void reduceActiveToTriangulated(vector<featureTrack>& tracks, vector<unsigned int>& indices, vector<unsigned int>& untriangulated) {
	
	for (unsigned int iii = 0; iii < indices.size(); iii++) {
		
		if (!tracks.at(indices.at(iii)).isTriangulated) {
			untriangulated.push_back(indices.at(iii));
			indices.erase(indices.begin() + iii);
			iii--;
		}
		
	}
	
}

void filterToActivePoints(vector<featureTrack>& tracks, vector<cv::Point2f>& pts1, vector<cv::Point2f>& pts2, vector<unsigned int>& indices, int idx1, int idx2) {

	pts1.clear();
	pts2.clear();
	
	for (unsigned int iii = 0; iii < indices.size(); iii++) {
		
		for (unsigned int jjj = 0; jjj < tracks.at(indices.at(iii)).locations.size()-1; jjj++) {
			
			if (((int)tracks.at(indices.at(iii)).locations.at(jjj).imageIndex) == idx1) {
				
				pts1.push_back(tracks.at(indices.at(iii)).locations.at(jjj).featureCoord);
				
				for (unsigned int kkk = jjj+1; kkk < tracks.at(indices.at(iii)).locations.size(); kkk++) {
					
					if (((int)tracks.at(indices.at(iii)).locations.at(kkk).imageIndex) == idx2) {
						
						pts2.push_back(tracks.at(indices.at(iii)).locations.at(kkk).featureCoord);
						
						continue;
					}
					
				}
				
				continue;
				
			}
			
		}

	}
	
	
}

void getActive3dPoints(vector<featureTrack>& tracks, vector<unsigned int>& indices, vector<cv::Point3d>& cloud) {
	
	cloud.clear();
	
	for (unsigned int iii = 0; iii < indices.size(); iii++) {
		cloud.push_back(tracks.at(indices.at(iii)).get3dLoc());
	}
	
}

void filterToCompleteTracks(vector<unsigned int>& dst, vector<unsigned int>& src, vector<featureTrack>& tracks, int idx1, int idx2) {
	
	dst.clear();
	
	for (unsigned int iii = 0; iii < src.size(); iii++) {
		
		if (tracks.at(src.at(iii)).locations.size() >= (idx2 - idx1)) {
			
			for (unsigned int jjj = 0; jjj < tracks.at(src.at(iii)).locations.size()-(idx2-idx1); jjj++) {
				
				//printf("%s << DEBUG [%d][%d]\n", __FUNCTION__, iii, jjj);
			
				if (((int)tracks.at(src.at(iii)).locations.at(jjj).imageIndex) == idx1) {
					
					//printf("%s << DEBUG [%d][%d] X\n", __FUNCTION__, iii, jjj);
					
					for (unsigned int kkk = jjj+1; kkk < tracks.at(src.at(iii)).locations.size(); kkk++) {
						
						//printf("%s << DEBUG [%d][%d][%d]\n", __FUNCTION__, iii, jjj, kkk);
						
						if (((int)tracks.at(src.at(iii)).locations.at(kkk).imageIndex) == idx2) {
							
							//printf("%s << DEBUG [%d][%d][%d] X\n", __FUNCTION__, iii, jjj, kkk);
							
							dst.push_back(src.at(iii));
							
						}
						
					}
					
				}
				
				
			}
			
		}
		
	}
	
}

void getActiveTracks(vector<unsigned int>& indices, vector<featureTrack>& tracks, int idx1, int idx2) {
	
	//printf("%s << Entered.\n", __FUNCTION__);
	
	// Should only consider a track active if it contains at least two projections in the subsequence
	
	indices.clear();
	
	for (unsigned int iii = 0; iii < tracks.size(); iii++) {
		
		//printf("%s << DEBUG [%d]\n", __FUNCTION__, iii);
		
		if (tracks.at(iii).locations.size() < 2) {
			continue;
		}
		
		for (unsigned int jjj = 0; jjj < tracks.at(iii).locations.size()-1; jjj++) {
			
			//printf("%s << DEBUG [%d][%d]\n", __FUNCTION__, iii, jjj);
		
			if ((((int)tracks.at(iii).locations.at(jjj).imageIndex) >= idx1) && (((int)tracks.at(iii).locations.at(jjj).imageIndex) <= idx2)) {
				
				for (unsigned int kkk = jjj+1; kkk < tracks.at(iii).locations.size(); kkk++) {
					
					if ((((int)tracks.at(iii).locations.at(kkk).imageIndex) >= idx1) && (((int)tracks.at(iii).locations.at(kkk).imageIndex) <= idx2)) {
						indices.push_back(iii);
						break;
					}
					
				}
				
				break;
			}
			
			
		}
		
		
		
		
		
	}
	
	//printf("%s << Exiting.\n", __FUNCTION__);
	
}

bool estimatePoseFromKnownPoints(cv::Mat& dst, cameraParameters camData, vector<featureTrack>& tracks, unsigned int index, const cv::Mat& guide, unsigned int minAppearances, unsigned int iterCount, double maxReprojErr, double inliersPercentage, double *reprojError, double *pnpInlierProp, bool debug) {
	
	vector<cv::Point3f> points_3d;
	vector<cv::Point2f> points_2d;
	
	unsigned int triangulatedCount = 0;
	*reprojError = 0.0;
	*pnpInlierProp = 0.0;
	
	if (debug) { printf("%s << minAppearances = (%d)\n", __FUNCTION__, minAppearances); }
	
	for (unsigned int iii = 0; iii < tracks.size(); iii++) {
		
		//printf("%s << DEBUG [%d]\n", __FUNCTION__, iii);
		
		if (!tracks.at(iii).isTriangulated) {
			continue;
		}
		
		triangulatedCount++;
		
		if (tracks.at(iii).locations.size() < minAppearances) {
			printf("%s << Error! size() = (%lu) < (%d)\n", __FUNCTION__, tracks.at(iii).locations.size(), minAppearances);
			continue;
		}
		
		bool pointAdded = false;
		
		for (unsigned int jjj = 0; jjj < tracks.at(iii).locations.size(); jjj++) {
			
			//printf("%s << DEBUG [%d][%d]\n", __FUNCTION__, iii, jjj);
		
			if (tracks.at(iii).locations.at(jjj).imageIndex == index) {
				
				points_2d.push_back(tracks.at(iii).locations.at(jjj).featureCoord);
				cv::Point3f tmp_pt = cv::Point3f(((float) tracks.at(iii).get3dLoc().x), ((float) tracks.at(iii).get3dLoc().y), ((float) tracks.at(iii).get3dLoc().z));
				points_3d.push_back(tmp_pt);
				pointAdded = true;
				break;
			}
		}
		
		/*
		if (!pointAdded) {
			printf("%s << Error! Pt (%d) was not found in current view (%d)\n", __FUNCTION__, iii, index);
			for (unsigned int jjj = 0; jjj < tracks.at(iii).locations.size(); jjj++) {
				printf("%s << Index (%d)...\n", __FUNCTION__, tracks.at(iii).locations.at(jjj).imageIndex);
			}
			
		}
		*/
		
	}
	
	unsigned int utilizedPointsCount = points_3d.size();
	
	cv::Mat Rvec, R, t;
	bool guided = false;
	
	cv::Mat guide_copy;
	
	guide_copy = guide.inv();
	
	if (guide_copy.rows > 0) {
		decomposeTransform(guide_copy, R, t);
		Rodrigues(R, Rvec);
		guided = true;
	}
	
	//cv::Mat inlierPts;
	vector<int> inlierPts;
	
	if (points_2d.size() < 8) {
		if (debug) { printf("%s << PnP Failed: Insufficient pts: (%d) < (%d) : [%d, %d, %d]\n", __FUNCTION__, ((int)points_2d.size()), 8, utilizedPointsCount, triangulatedCount, ((int)tracks.size())); }
		guide.copyTo(dst);
		return false;
	}
	
	#ifdef _OPENCV_VERSION_3_PLUS_
	solvePnPRansac(points_3d, points_2d, camData.K, camData.blankCoeffs, Rvec, t, guided, iterCount, maxReprojErr, ((unsigned int) ((double) points_2d.size()) * inliersPercentage), inlierPts, cv::EPNP); // ITERATIVE, P3P
	#else
	solvePnPRansac(points_3d, points_2d, camData.K, camData.blankCoeffs, Rvec, t, guided, iterCount, maxReprojErr, ((unsigned int) ((double) points_2d.size()) * inliersPercentage), inlierPts, CV_EPNP); // ITERATIVE, CV_P3P
	#endif
	
	
	
	//if ( ((unsigned int) inlierPts.size()) < ((unsigned int) (((double) points_2d.size()) * inliersPercentage)) ) {
	if ( inlierPts.size() < 8 ) {
		if (debug) { printf("%s << PnP Failed:  Insufficient inliers: (%lu) / (%lu) : [%d, %d, %d]\n", __FUNCTION__, inlierPts.size(), points_2d.size(), utilizedPointsCount, triangulatedCount, ((int)tracks.size())); }
		guide.copyTo(dst);
		return false;
	}
	
	if (debug) { printf("%s << PnP Succeeded:  Inliers: (%d) / (%lu) : [%d, %d, %d]\n", __FUNCTION__, (int)inlierPts.size(), points_2d.size(), utilizedPointsCount, triangulatedCount, ((int)tracks.size())); }
	
	*pnpInlierProp = double(inlierPts.size()) / double(points_2d.size());
	
	Rodrigues(Rvec, R);
	cv::Mat P;
	findP1Matrix(P, R, t);			
	projectionToTransformation(P, dst);
	dst = dst.inv();
	
	// Obtain some kind of error
	
	//printf("%s << points3d.size() = (%d); guided.size() = (%d)\n", __FUNCTION__, points_3d.size(), guided.size());
	//double reprojError = 0.0;
	
	//printf("%s << debug (%d)\n", __FUNCTION__, 0);
	
	std::vector<cv::Point3f> validPts;
	for (unsigned int iii = 0; iii < inlierPts.size(); iii++) {
		validPts.push_back(points_3d.at(inlierPts.at(iii)));
	}
	
	//printf("%s << debug (%d)\n", __FUNCTION__, 1);
	std::vector<cv::Point2f> point_2f;
	projectPoints(validPts, Rvec, t, camData.K, camData.blankCoeffs, point_2f);
	
	//printf("%s << debug (%d)\n", __FUNCTION__, 2);
	
	for (unsigned int iii = 0; iii < inlierPts.size(); iii++) {
		//printf("%s << points_2d.at(%d) = (%f, %f), point_2f.at(%d) = (%f, %f)\n", __FUNCTION__, iii, points_2d.at(inlierPts.at(iii)).x, points_2d.at(inlierPts.at(iii)).y, iii, point_2f.at(iii).x, point_2f.at(iii).y);
		*reprojError += distBetweenPts2f(points_2d.at(inlierPts.at(iii)), point_2f.at(iii));
	
	}
	
	if (inlierPts.size() > 0) {
		*reprojError /= double(inlierPts.size());
	} else {
		*reprojError = -1.0;
	}
	
	//cout << "guide = " << guide << endl;
	//cout << "dst = " << dst << endl;
	
	return true;
	
}



void getCorrespondingPoints(vector<featureTrack>& tracks, const vector<cv::Point2f>& pts1, vector<cv::Point2f>& pts2, int idx0, int idx1) {
	
	printf("%s << Entered X.\n", __FUNCTION__);
	
	// For each track
	for (unsigned int iii = 0; iii < tracks.size(); iii++) {
		
		if (tracks.at(iii).locations.size() < 2) {
			continue;
		}
		//printf("%s << Debug (%d)\n", __FUNCTION__, iii);
		// For each projection in the track
		for (unsigned int kkk = 0; kkk < tracks.at(iii).locations.size()-1; kkk++) {
		
			//printf("%s << Debug (%d)(%d)\n", __FUNCTION__, iii, kkk);
			
			// If the camera of the projection matches the aim (idx0)
			if (((int)tracks.at(iii).locations.at(kkk).imageIndex) == idx0) {
				// Found a feature that exists in the first image
				
				//printf("%s << Debug (%d)(%d) 2\n", __FUNCTION__, iii, kkk);
				
				// For each of the first set of points
				for (unsigned int jjj = 0; jjj < pts1.size(); jjj++) {
				
					//printf("%s << Debug (%d)(%d)(%d)\n", __FUNCTION__, iii, kkk, jjj);
				
					// If the projection location matches the first point at this point 
					if ((tracks.at(iii).locations.at(kkk).featureCoord.x == pts1.at(jjj).x) && (tracks.at(iii).locations.at(kkk).featureCoord.y == pts1.at(jjj).y)) {
						
						//printf("%s << Debug (%d)(%d)(%d) 2\n", __FUNCTION__, iii, kkk, jjj);
						
						// For each of the later projections on that track
						for (unsigned int ppp = kkk; ppp < tracks.at(iii).locations.size(); ppp++) {
							
							//printf("%s << Debug (%d)(%d)(%d)(%d)\n", __FUNCTION__, iii, kkk, jjj, ppp);
							
							// If this later projection matches the second aimed index
							if (((int)tracks.at(iii).locations.at(ppp).imageIndex) == idx1) {
								
								//printf("%s << Debug (%d)(%d)(%d)(%d) 2\n", __FUNCTION__, iii, kkk, jjj, ppp);
								
								// If the first projec....?
								if (jjj != pts2.size()) {
									printf("%s << ERROR! Vectors are not syncing (jjj = %d), pts2.size() = %d\n", __FUNCTION__, jjj, ((int)pts2.size()));
								} else {
									pts2.push_back(tracks.at(iii).locations.at(ppp).featureCoord);
								}

								
							}
	
							
						}
						
						break;
					}
					
				}
				
				
				break;
			}
			
		}
	}
		
	printf("%s << Exiting.\n", __FUNCTION__);
	
}

void getTriangulatedFullSpanPoints(vector<featureTrack>& tracks, vector<cv::Point2f>& pts1, vector<cv::Point2f>& pts2, int idx1, int idx2, vector<cv::Point3f>& points3) {
	
	pts1.clear();
	pts2.clear();
	points3.clear();
	
	for (unsigned int iii = 0; iii < tracks.size(); iii++) {
		
		for (unsigned int kkk = 0; kkk < tracks.at(iii).locations.size()-1; kkk++) {
			
			// If the track extends between the two images of interest
			if (((int)tracks.at(iii).locations.at(kkk).imageIndex) == idx1) {
				
				for (unsigned int jjj = kkk+1; jjj < tracks.at(iii).locations.size(); jjj++) {
					
					if (((int)tracks.at(iii).locations.at(jjj).imageIndex) == idx2) {
						
						if (tracks.at(iii).isTriangulated) {
							
							pts1.push_back(tracks.at(iii).locations.at(kkk).featureCoord);
							pts2.push_back(tracks.at(iii).locations.at(jjj).featureCoord);
							
							points3.push_back(cv::Point3f(((float) tracks.at(iii).get3dLoc().x), ((float) tracks.at(iii).get3dLoc().y), ((float) tracks.at(iii).get3dLoc().z)));
							
							break;
						}
						
						
						
					}
				}
				
			}
		}
	}
	
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

void getTranslationBetweenCameras(cv::Mat& C1, cv::Mat& C2, double *translations) {
	
	cv::Mat CD = C2 - C1;
	
	translations[0] = CD.at<double>(0,3);
	translations[1] = CD.at<double>(1,3);
	translations[2] = CD.at<double>(2,3);

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

bool findClusterMean(const vector<cv::Point3d>& pts, cv::Point3d& pt3d, int mode, int minEstimates, double maxStandardDev) {
	
	cv::Point3d mean3d = cv::Point3d(0.0, 0.0, 0.0);
	cv::Point3d stddev3d = cv::Point3d(0.0, 0.0, 0.0);
	
	vector<cv::Point3d> estimatedLocations;
	estimatedLocations.insert(estimatedLocations.end(), pts.begin(), pts.end());
	vector<int> clusterCount;
	
	double outlierLimit = 3.0;
	
	if (mode == CLUSTER_MEAN_MODE) {
		
		
		for (unsigned int iii = 0; iii < estimatedLocations.size(); iii++) {
			clusterCount.push_back(1);
			//printf("%s << pt(%d) = (%f, %f, %f)\n", __FUNCTION__, iii, estimatedLocations.at(iii).x, estimatedLocations.at(iii).y, estimatedLocations.at(iii).z);
		}
		
		for (unsigned int iii = 0; iii < estimatedLocations.size()-1; iii++) {
			
			cv::Point3d basePt = estimatedLocations.at(iii);
			
			
			
			for (unsigned int jjj = iii+1; jjj < estimatedLocations.size(); jjj++) {
				
				double separation = pow(pow(basePt.x - estimatedLocations.at(jjj).x, 2.0) + pow(basePt.y - estimatedLocations.at(jjj).y, 2.0) + pow(basePt.z - estimatedLocations.at(jjj).z, 2.0), 0.5);
				
				if ( (separation < maxStandardDev) || (maxStandardDev == 0.0) ) {
					
					estimatedLocations.at(iii) *= double(clusterCount.at(iii));
					clusterCount.at(iii)++;
					estimatedLocations.at(iii) += estimatedLocations.at(jjj);
					estimatedLocations.at(iii).x /= double(clusterCount.at(iii));
					estimatedLocations.at(iii).y /= double(clusterCount.at(iii));
					estimatedLocations.at(iii).z /= double(clusterCount.at(iii));
					
					estimatedLocations.erase(estimatedLocations.begin() + jjj);
					clusterCount.erase(clusterCount.begin() + jjj);
					jjj--;
					
				}
				
			}
		}
		
		int maxClusterSize = 0, maxClusterIndex = -1;
		
		for (unsigned int iii = 0; iii < estimatedLocations.size(); iii++) {

			if (clusterCount.at(iii) >= maxClusterSize) {
				maxClusterSize = clusterCount.at(iii);
				maxClusterIndex = iii;
			}

			//printf("%s << cluster(%d) = (%f, %f, %f) [%d]\n", __FUNCTION__, iii, estimatedLocations.at(iii).x, estimatedLocations.at(iii).y, estimatedLocations.at(iii).z, clusterCount.at(iii));
		}
		
		if (maxClusterSize >= minEstimates) {
			pt3d = estimatedLocations.at(maxClusterIndex);
		} else {
			return false;
		}
		
	} else {
		//printf("%s << A; estimatedLocations.size() = (%d)\n", __FUNCTION__, estimatedLocations.size());
		
		for (unsigned int qqq = 0; qqq < estimatedLocations.size(); qqq++) {
			
			//printf("%s << pt(%d) = (%f, %f, %f)\n", __FUNCTION__, qqq, estimatedLocations.at(qqq).x, estimatedLocations.at(qqq).y, estimatedLocations.at(qqq).z); /* , separationsVector.at(qqq) */
			
			mean3d.x += (estimatedLocations.at(qqq).x / ((double) estimatedLocations.size()));
			mean3d.y += (estimatedLocations.at(qqq).y / ((double) estimatedLocations.size()));
			mean3d.z += (estimatedLocations.at(qqq).z / ((double) estimatedLocations.size()));
			
		}
		
		//printf("%s << mean = (%f, %f, %f)\n", __FUNCTION__, mean3d.x, mean3d.y, mean3d.z);
		
		//printf("%s << mean point = (%f, %f, %f)\n", __FUNCTION__, mean3d.x, mean3d.y, mean3d.z);
		
		// Calculate initial standard deviation
		for (unsigned int qqq = 0; qqq < estimatedLocations.size(); qqq++) {
			
			stddev3d.x += (pow((estimatedLocations.at(qqq).x - mean3d.x), 2.0) / ((double) estimatedLocations.size()));
			stddev3d.y += (pow((estimatedLocations.at(qqq).y - mean3d.y), 2.0) / ((double) estimatedLocations.size()));
			stddev3d.z += (pow((estimatedLocations.at(qqq).z - mean3d.z), 2.0) / ((double) estimatedLocations.size()));
			
		}
		
		stddev3d.x = pow(stddev3d.x, 0.5);
		stddev3d.y = pow(stddev3d.y, 0.5);
		stddev3d.z = pow(stddev3d.z, 0.5);
		
		//printf("%s << stddev3d = (%f, %f, %f)\n", __FUNCTION__, stddev3d.x, stddev3d.y, stddev3d.z);
		
		//printf("%s << Point triangulated from (%d) view pairs: (%f, %f, %f) / (%f, %f, %f)\n", __FUNCTION__, estimatedLocations.size(), mean3d.x, mean3d.y, mean3d.z, stddev3d.x, stddev3d.y, stddev3d.z);
		
		// Reject projections that are more than X standard deviations away
		for (int qqq = estimatedLocations.size()-1; qqq >= 0; qqq--) {
			
			double abs_diff_x = abs(estimatedLocations.at(qqq).x - mean3d.x);
			double abs_diff_y = abs(estimatedLocations.at(qqq).y - mean3d.y); 
			double abs_diff_z = abs(estimatedLocations.at(qqq).z - mean3d.z); 
			
			if ((abs_diff_x > outlierLimit*stddev3d.x) || (abs_diff_y > outlierLimit*stddev3d.y) || (abs_diff_z > outlierLimit*stddev3d.z)) {
				estimatedLocations.erase(estimatedLocations.begin() + qqq);
			}

		}
		
		//printf("%s << B: estimatedLocations.size() = (%d)\n", __FUNCTION__, estimatedLocations.size());

		// Recalculate the standard deviation
		stddev3d.x = 0.0;
		stddev3d.y = 0.0;
		stddev3d.z = 0.0;
		
		for (unsigned int qqq = 0; qqq < estimatedLocations.size(); qqq++) {
			
			stddev3d.x += (pow((estimatedLocations.at(qqq).x - mean3d.x), 2.0) / ((double) estimatedLocations.size()));
			stddev3d.y += (pow((estimatedLocations.at(qqq).y - mean3d.y), 2.0) / ((double) estimatedLocations.size()));
			stddev3d.z += (pow((estimatedLocations.at(qqq).z - mean3d.z), 2.0) / ((double) estimatedLocations.size()));
			
		}
		
		stddev3d.x = pow(stddev3d.x, 0.5);
		stddev3d.y = pow(stddev3d.y, 0.5);
		stddev3d.z = pow(stddev3d.z, 0.5);
		
		//printf("%s << stddev3d = (%f, %f, %f) [%d]\n", __FUNCTION__, stddev3d.x, stddev3d.y, stddev3d.z, minEstimates);
		
		// Reject track if the standard deviation is still too high, or not enough rays remain
		if ( ((stddev3d.x > maxStandardDev) || (stddev3d.y > maxStandardDev) || (stddev3d.z > maxStandardDev) || (((int)estimatedLocations.size()) < minEstimates)) && (maxStandardDev != 0.0) ) { 
			return false;
		}
		
		//printf("%s << C: estimatedLocations.size() = (%d)\n", __FUNCTION__, estimatedLocations.size());
		
		// Final calculation of average point location
		pt3d.x = 0.0;
		pt3d.y = 0.0;
		pt3d.z = 0.0;
		
		for (unsigned int qqq = 0; qqq < estimatedLocations.size(); qqq++) {
				
			pt3d.x += (estimatedLocations.at(qqq).x / ((double) estimatedLocations.size()));
			pt3d.y += (estimatedLocations.at(qqq).y / ((double) estimatedLocations.size()));
			pt3d.z += (estimatedLocations.at(qqq).z / ((double) estimatedLocations.size()));
			
		}
	}
	
	return true;
}

void triangulateTracks(vector<featureTrack>& tracks, vector<unsigned int>& indices, cameraParameters& cameraData, cv::Mat *cameras, unsigned int earliest_index, unsigned int latest_index) {
	
	//return;
	
	//printf("%s << Entered.\n", __FUNCTION__);
	
	unsigned int minPairs = 10;
	
	// Should first check through camera pairs to see which ones can achive valid triangulations:
	
	//printf("%s << latest_index/earliest_index = (%d/%d)\n", __FUNCTION__, latest_index, earliest_index);
	unsigned int pair_width = latest_index - earliest_index + 1;
	cv::Mat validFramePairs = cv::Mat::zeros(pair_width, pair_width, CV_8UC1);
	
	double translations[3];
	
	for (unsigned int iii = earliest_index; iii < latest_index; iii++) {
		for (unsigned int jjj = iii+1; jjj <= latest_index; jjj++) {
			
			if (cameras[iii].rows != 4) {
				continue;
			}
			
			if (cameras[jjj].rows != 4) {
				continue;
			}
			
			getTranslationBetweenCameras(cameras[iii], cameras[jjj], translations);
			
			//printf("%s << translations = (%f, %f, %f)\n", __FUNCTION__, translations[0], translations[1], translations[2]);
			
			//if ((abs(translations[0]) + abs(translations[1])) < 2.0*abs(translations[2])) {
				
			
				
			// Conditions for validity as a camera pair
			if ((abs(translations[0] > 0.2)) || (abs(translations[1] > 0.2))) {	//  || (abs(translations[2] > 1.0))
				validFramePairs.at<unsigned char>(iii-earliest_index,jjj-earliest_index) = 1;
			}
		}
	}
		
	unsigned int validCount = countNonZero(validFramePairs);
	
	//printf("%s << validCount = %d\n", __FUNCTION__, validCount);
	
	if (validCount < minPairs) {
		return;
	}
	
	//printf("%s << Continuing.\n", __FUNCTION__);
	
	vector<cv::Point3d> estimatedLocations;
	
	cv::Point3d pt3d(0.0, 0.0, 0.0);
	cv::Point3d mean3d(0.0, 0.0, 0.0);
	cv::Point3d stddev3d(0.0, 0.0, 0.0);
	
	
	
	for (unsigned int iii = 0; iii < indices.size(); iii++) {
		
		//printf("%s << Track (%d) ...\n", __FUNCTION__, indices.at(iii));
		
		if (tracks.size() <= indices.at(iii)) {
			return;
		}
		
		if (tracks.at(indices.at(iii)).locations.size() < 2) {
			continue;
		}
		
		if (tracks.at(indices.at(iii)).isTriangulated) {
			continue;
		}
		
		//printf("%s << Track (%d) DEBUG [%d]\n", __FUNCTION__, indices.at(iii), 0);
		
		estimatedLocations.clear();
			
		pt3d = cv::Point3d(0.0, 0.0, 0.0);
		mean3d = cv::Point3d(0.0, 0.0, 0.0);
		stddev3d = cv::Point3d(0.0, 0.0, 0.0);
		
		unsigned int rrr, sss;
		
		for (unsigned int jjj = 0; jjj < tracks.at(indices.at(iii)).locations.size()-1; jjj++) {
			
			//printf("%s << Track (%d) jjj [%d]\n", __FUNCTION__, indices.at(iii), jjj);
			
			rrr = tracks.at(indices.at(iii)).locations.at(jjj).imageIndex;
			
			if ((rrr >= earliest_index) && (rrr <= latest_index)) {
				
				//printf("%s << Track (%d) jjj [%d] debug [%d]\n", __FUNCTION__, indices.at(iii), jjj, 0);
			
				if (cameras[rrr].rows != 4) {
					//printf("%s << Breaking rrr (%d)...\n", __FUNCTION__, rrr);
					continue;
				}
				
				//printf("%s << Track (%d) jjj [%d] debug [%d]\n", __FUNCTION__, indices.at(iii), jjj, 1);
			
				//printf("%s << rrr = %d\n", __FUNCTION__, rrr);
				
				cv::Mat temp_C0;
				cameras[rrr].copyTo(temp_C0);
				
				for (unsigned int kkk = jjj+1; kkk < tracks.at(indices.at(iii)).locations.size(); kkk++) {
					
					//printf("%s << Track (%d) kkk [%d]\n", __FUNCTION__, indices.at(iii), kkk);
					
					sss = tracks.at(indices.at(iii)).locations.at(kkk).imageIndex;
										
					if ((sss >= earliest_index) && (sss <= latest_index)) {
						
						//printf("%s << Track (%d) kkk [%d] debug [%d]\n", __FUNCTION__, indices.at(iii), kkk, 0);
						
						//printf("%s << testing
						
						if (validFramePairs.at<unsigned char>(rrr-earliest_index,sss-earliest_index) == 0) {
							continue;
						}
					
						if (cameras[sss].rows != 4) {
							//printf("%s << Breaking sss (%d)...\n", __FUNCTION__, sss);
							continue;
						}
						
						//printf("%s << Track (%d) kkk [%d] debug [%d]\n", __FUNCTION__, indices.at(iii), kkk, 1);

						
						cv::Mat temp_C1;
						cameras[sss].copyTo(temp_C1);
						
						// This should be a valid pair of projections to triangulate from for this point..
						
						// Check that the projections have enough translation:
						
						
							
						//}
						
						cv::Point2f pt1_, pt2_;
						cv::Point3d pt3d_;
						
						pt1_ = tracks.at(indices.at(iii)).getCoord(rrr);
						pt2_ = tracks.at(indices.at(iii)).getCoord(sss);
						
						Triangulate(pt1_, pt2_, cameraData.K, cameraData.K_inv,	temp_C0, temp_C1, pt3d_, false);
						
						//if ((pointIsInFront(temp_C0, pt3d_)) && (pointIsInFront(temp_C1, pt3d_))) {
							
							estimatedLocations.push_back(pt3d_);
							
							/*
							pt3d.x += pt3d_.x;
							pt3d.y += pt3d_.y;
							pt3d.z += pt3d_.z;
							contribCount++;
							*/
						//}
						
						if (jjj == 0) {
							//printf("%s << Adding triangulation: (%f, %f, %f)\n", __FUNCTION__, pt3d_.x, pt3d_.y, pt3d_.z);
						}
					}
				}
			}
			
		}
		
		//printf("%s << Track (%d) : estimatedLocations.size() = %d\n", __FUNCTION__, indices.at(iii), estimatedLocations.size());
		
		if (estimatedLocations.size() >= minPairs) {
				
			//printf("%s << Assigning (%d)\n", __FUNCTION__, indices.at(iii));
			
			//printf("%s << Calculating means..\n", __FUNCTION__);
			
			for (unsigned int qqq = 0; qqq < estimatedLocations.size(); qqq++) {
				
				mean3d.x += (estimatedLocations.at(qqq).x / ((double) estimatedLocations.size()));
				mean3d.y += (estimatedLocations.at(qqq).y / ((double) estimatedLocations.size()));
				mean3d.z += (estimatedLocations.at(qqq).z / ((double) estimatedLocations.size()));
				
			}
			
			//printf("%s << Calculating deviations..\n", __FUNCTION__);
			
			for (unsigned int qqq = 0; qqq < estimatedLocations.size(); qqq++) {
				
				stddev3d.x += (pow((estimatedLocations.at(qqq).x - mean3d.x), 2.0) / ((double) estimatedLocations.size()));
				stddev3d.y += (pow((estimatedLocations.at(qqq).y - mean3d.y), 2.0) / ((double) estimatedLocations.size()));
				stddev3d.z += (pow((estimatedLocations.at(qqq).z - mean3d.z), 2.0) / ((double) estimatedLocations.size()));
				
			}
			
			stddev3d.x = pow(stddev3d.x, 0.5);
			stddev3d.y = pow(stddev3d.y, 0.5);
			stddev3d.z = pow(stddev3d.z, 0.5);
			
			//printf("%s << Erasing outliers..\n", __FUNCTION__);
			
			for (int qqq = estimatedLocations.size()-1; qqq >= 0; qqq--) {
				
				//printf("%s << qqq = %d\n", __FUNCTION__, qqq);
				
				double abs_diff_x = abs(estimatedLocations.at(qqq).x - mean3d.x);
				double abs_diff_y = abs(estimatedLocations.at(qqq).y - mean3d.y); 
				double abs_diff_z = abs(estimatedLocations.at(qqq).z - mean3d.z); 
				
				if ((abs_diff_x > 2*stddev3d.x) || (abs_diff_y > 2*stddev3d.y) || (abs_diff_z > 2*stddev3d.z)) {
					estimatedLocations.erase(estimatedLocations.begin() + qqq);
				}

			}

				
			//printf("%s << Mean: (%f, %f, %f)\n", __FUNCTION__, mean3d.x, mean3d.y, mean3d.z);
			//printf("%s << Std deviation: (%f, %f, %f)\n", __FUNCTION__, stddev3d.x, stddev3d.y, stddev3d.z);
			
			
			if (estimatedLocations.size() >= minPairs) {
				//printf("%s << Re-calculating mean..\n", __FUNCTION__);
			
				for (unsigned int qqq = 0; qqq < estimatedLocations.size(); qqq++) {
					
					//printf("%s << qqq = %d\n", __FUNCTION__, qqq);
					
					pt3d.x += (estimatedLocations.at(qqq).x / ((double) estimatedLocations.size()));
					pt3d.y += (estimatedLocations.at(qqq).y / ((double) estimatedLocations.size()));
					pt3d.z += (estimatedLocations.at(qqq).z / ((double) estimatedLocations.size()));
					
				}

				tracks.at(indices.at(iii)).set3dLoc(pt3d);
				
				//printf("%s << Adding contribution: (%f, %f, %f)\n", __FUNCTION__, pt3d.x, pt3d.y, pt3d.z);
				
			}
			
			
			
			
		}
		
	//	printf("%s << Track (%d) : DONE!\n", __FUNCTION__, indices.at(iii));
	}
	
	//printf("%s << Exiting.\n", __FUNCTION__);
}

unsigned int addNewPoints(vector<featureTrack>& tracks, vector<unsigned int>& indices, cameraParameters& cameraData, cv::Mat *cameras, unsigned int earliest_index, unsigned int latest_index) {

	unsigned int totalTriangulatedPoints = 0;
	
	for (unsigned int iii = 0; iii < indices.size(); iii++) {
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
			
			
			
			
			if (rrr == earliest_index) {
			
				if (cameras[rrr].rows != 4) {
					printf("%s << Breaking rrr (%d)...\n", __FUNCTION__, rrr);
					continue;
				}
			
				//printf("%s << rrr = %d\n", __FUNCTION__, rrr);
				
				cv::Mat temp_C0;
				cameras[rrr].copyTo(temp_C0);
				
				for (unsigned int kkk = jjj+1; kkk < tracks.at(indices.at(iii)).locations.size(); kkk++) {
					
					unsigned int sss = tracks.at(indices.at(iii)).locations.at(kkk).imageIndex;
					
					
					
					//printf("%s << sss = %d\n", __FUNCTION__, sss);
					
					if (sss == latest_index) {
					
						if (cameras[sss].rows != 4) {
							printf("%s << Breaking sss (%d)...\n", __FUNCTION__, sss);
							continue;
						}
						
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

void obtainAppropriateBaseTransformation(cv::Mat& C0, vector<featureTrack>& tracks) {
	
	cv::Point3d centroid, deviations;
	
	int numTriangulatedPts = 0;
	
	for (unsigned int iii = 0; iii < tracks.size(); iii++) {
		if (tracks.at(iii).isTriangulated) {
			
			numTriangulatedPts += 1;
			
		}
	}
	
	if (numTriangulatedPts == 0) {
		C0 = cv::Mat::eye(4, 4, CV_64FC1);
	} else {
		// Find centroid and standard deviations of points
		findCentroidAndSpread(tracks, centroid, deviations);
		
		// Create transformation matrix with a centroid that is distance
		cv::Mat t(4, 1, CV_64FC1), R;
		
		R = cv::Mat::eye(3, 3, CV_64FC1);
		
		t.at<double>(0,0) = centroid.x + 3.0 * deviations.x;
		t.at<double>(0,0) = centroid.y + 3.0 * deviations.y;
		t.at<double>(0,0) = centroid.z + 3.0 * deviations.z;
		t.at<double>(3,0) = 1.0;

		composeTransform(R, t, C0);
	}
}

void findCentroidAndSpread(vector<featureTrack>& tracks, cv::Point3d& centroid, cv::Point3d& deviations) {
	
	centroid = cv::Point3d(0.0, 0.0, 0.0);
	deviations = cv::Point3d(0.0, 0.0, 0.0);
	
	double numPoints = 0.00;
	
	for (unsigned int iii = 0; iii < tracks.size(); iii++) {
		if (tracks.at(iii).isTriangulated) {
			
			numPoints += 1.00;
			
		}
	}
	
	if (numPoints == 0.00) {
		return;
	}
	
	for (unsigned int iii = 0; iii < tracks.size(); iii++) {
		
		if (tracks.at(iii).isTriangulated) {
			
			cv::Point3d tmpPt;
			tmpPt = tracks.at(iii).get3dLoc();
			centroid.x += (tmpPt.x / numPoints);
			centroid.y += (tmpPt.y / numPoints);
			centroid.z += (tmpPt.z / numPoints);
			
		}
		
	}
	
	for (unsigned int iii = 0; iii < tracks.size(); iii++) {
		
		if (tracks.at(iii).isTriangulated) {
			cv::Point3d tmpPt;
			tmpPt = tracks.at(iii).get3dLoc();
			deviations.x += (pow((tmpPt.x - centroid.x), 2.0) / numPoints);
			deviations.y += (pow((tmpPt.y - centroid.y), 2.0) / numPoints);
			deviations.z += (pow((tmpPt.z - centroid.z), 2.0) / numPoints);
		}
		
	}
	
	deviations.x = pow(deviations.x, 0.5);
	deviations.y = pow(deviations.y, 0.5);
	deviations.z = pow(deviations.z, 0.5);
}

void getPoints3dFromTracks(vector<featureTrack>& tracks, vector<cv::Point3d>& cloud, int idx1, int idx2) {
	
	cloud.clear();
	
	for (unsigned int iii = 0; iii < tracks.size(); iii++) {
		
		if (!tracks.at(iii).isTriangulated) {
			continue;
		}
		
		if (tracks.at(iii).locations.size() == 0) {
			continue;
		}
		
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
			
			if (addedPoint) {
				break;
			}
			
		}
		
		

	}
	
}


void findFourTransformations(cv::Mat *C, const cv::Mat& E, const cv::Mat& K, const vector<cv::Point2f>& pts1, const vector<cv::Point2f>& pts2) {
	
	
	cv::Mat I, negI, t33, W, Winv, Z, Kinv, zeroTrans, R, t, Rvec;
	
	Kinv = K.inv();
	zeroTrans = cv::Mat::zeros(3, 1, CV_64FC1);
	I = cv::Mat::eye(3, 3, CV_64FC1);
	negI = -I;
	
	getWandZ(W, Winv, Z);
	
	//cout << __FUNCTION__ << " << W = " << W << endl;
	//cout << __FUNCTION__ << " << Winv = " << Winv << endl;
	
	//R = svd.u * W * svd.vt;
	//t = svd.u.col(2);
	
	//cout << __FUNCTION__ << " << R = " << R << endl;
	
	//Rodrigues(R, Rvec);
	
	//cout << __FUNCTION__ << " << Rvec = " << Rvec << endl;
	
	//cout << __FUNCTION__ << " << t = " << t << endl;
	
	cv::Mat c;
	//compileTransform(c, R, t);
	
	//cout << __FUNCTION__ << " << c = " << c << endl;
	
	cv::Mat u_transposePos, u_transposeNeg;
	cv::SVD svdPos, svdNeg;
	
	svdPos = cv::SVD(E);
	svdNeg = cv::SVD(-E);
		
	transpose(svdPos.u, u_transposePos);
	transpose(svdNeg.u, u_transposeNeg);
	
	double detVal;
	
	for (unsigned int zzz = 0; zzz < 4; zzz++) {
		
		//printf("%s << Loop [%d]\n", __FUNCTION__, zzz);
		
		switch (zzz) {
			case 0:
				//t33 = v * W * SIGMA * vt;
				t33 = svdPos.u * Z * u_transposePos;
				R = svdPos.u * Winv * svdPos.vt;
				detVal = determinant(R);
				if (detVal < 0) {
					t33 = svdNeg.u * Z * u_transposeNeg;
					R = svdNeg.u * Winv * svdNeg.vt;
				}
				break;
			case 1:
				//t33 = v * W_inv * SIGMA * vt;
				t33 = negI * svdPos.u * Z * u_transposePos;
				R = svdPos.u * Winv * svdPos.vt;
				detVal = determinant(R);
				if (detVal < 0) {
					t33 = negI * svdNeg.u * Z * u_transposeNeg;
					R = svdNeg.u * Winv * svdNeg.vt;
				}
				break;
			case 2:
				//t33 = v * W * SIGMA * vt;
				t33 = svdPos.u * Z * u_transposePos;
				R = svdPos.u * W * svdPos.vt;
				detVal = determinant(R);
				if (detVal < 0) {
					t33 = svdNeg.u * Z * u_transposeNeg;
					R = svdNeg.u * W * svdNeg.vt;
				}
				break;
			case 3:
				//t33 = v * W_inv * SIGMA * vt;
				t33 = negI * svdPos.u * Z * u_transposePos;
				R = svdPos.u * W * svdPos.vt;
				detVal = determinant(R);
				if (detVal < 0) {
					t33 = negI * svdNeg.u * Z * u_transposeNeg;
					R = svdNeg.u * W * svdNeg.vt;
				}
				break;
			default:
				break;
		}
		
		// Converting 3x3 t mat to vector
		t = cv::Mat::zeros(3, 1, CV_64F);
		t.at<double>(0,0) = -t33.at<double>(1,2); // originally had -ve
		t.at<double>(1,0) = t33.at<double>(0,2); // originall had a +ve
		t.at<double>(2,0) = -t33.at<double>(0,1); // originally had -ve
		
		//cout << __FUNCTION__ << " << t[" << zzz << "] = " << t << endl;
		
		if (abs(1.0 - detVal) > 0.01) {
			//printf("%s << Solution [%d] is invalid - det(R) = %f\n", __FUNCTION__, zzz, detVal);
		} else {
			//printf("%s << Solution [%d] det(R) = %f\n", __FUNCTION__, zzz, detVal);
		}
		
		//compileTransform(c, R, -R*t);
		//compileTransform(c, R, t);
		composeTransform(R, t, c);
		
		//char printSummary[256];
		//summarizeTransformation(c, printSummary);
		//printf("%s << Original c[%d] = %s\n", __FUNCTION__, zzz, printSummary);
		
		//c = c.inv();
		
		c.copyTo(C[zzz]);
		
		//cout << __FUNCTION__ << " << Czzz = " << C[zzz] << endl;
		
	}

}



bool pointIsInFront(const cv::Mat& C, const cv::Point3d& pt) {
	
	cv::Point3d p1;
	
	transfer3dPoint(pt, p1, C);

	if (p1.z > 0) {
		return true;
	} else {
		
		return false;
		
	}
	
	
}

// This function is when you have the 3D points and both transformation matrices
int pointsInFront(const cv::Mat& C1, const cv::Mat& C2, const vector<cv::Point3d>& pts) {
	int retVal = 0;
	
	vector<cv::Point3d> pts1, pts2;
	
	cv::Mat R1, t1, R2, t2;
	decomposeTransform(C1, R1, t1);
	decomposeTransform(C2, R2, t2);
	
	double rot1, rot2;
	rot1 = getRotationInDegrees(R1);
	rot2 = getRotationInDegrees(R2);
	
	printf("%s << Two rotations = (%f, %f)\n", __FUNCTION__, rot1, rot2);
	
	// maybe something to do with intrinsics matrix....
	
	// want to transfer the 3D point locations into the co-ordinate system of each camera
	transfer3DPoints(pts, pts1, C1);
	transfer3DPoints(pts, pts2, C2);
	
	for (unsigned int iii = 0; iii < pts.size(); iii++) {
		
		//printf("%s << [%d](%f, %f, %f)-(%f, %f, %f)\n", __FUNCTION__, iii, pts1.at(iii).x, pts1.at(iii).y, pts1.at(iii).z, pts2.at(iii).x, pts2.at(iii).y, pts2.at(iii).z);
		
		if ((pts1.at(iii).z > 0) && (pts2.at(iii).z > 0)) {
			
			if (retVal < 5) {
				printf("%s << Point location is (%f, %f, %f)\n", __FUNCTION__, pts.at(iii).x, pts.at(iii).y, pts.at(iii).z);
				printf("%s << Rel #1 is (%f, %f, %f)\n", __FUNCTION__, pts1.at(iii).x, pts1.at(iii).y, pts1.at(iii).z);
				printf("%s << Rel #2 is (%f, %f, %f)\n", __FUNCTION__, pts2.at(iii).x, pts2.at(iii).y, pts2.at(iii).z);
			}
			
			retVal++;
		}
	}
	
	return retVal;
}

// This function is when you have the relative transformation matrix, cam matrix, and 2D locations of points..
int pointsInFront(const cv::Mat& C1, const cv::Mat& K, const vector<cv::Point2f>& pts1, const vector<cv::Point2f>& pts2) {
	
	int retVal = 0;
	
	cv::Mat Kinv;
	Kinv = K.inv();
	
	// Representing initial camera relative to itself
	cv::Mat P0, C0;
	initializeP0(P0);
	projectionToTransformation(P0, C0);
	
	// Representing second camera relative to first camera
	cv::Mat P1;
	transformationToProjection(C1, P1);
	

	vector<cv::Point3d> pc1, pc2;
	vector<cv::Point2f> correspImg1Pt, correspImg2Pt;	

	// Get 3D locations of points relative to first camera
	TriangulatePoints(pts1, pts2, K, Kinv, P0, P1, pc1, correspImg1Pt);
	
	// Transfer 3D locations so that they are relative to second camera
	cv::Mat C1inv;
	C1inv = C1.inv();
	
	// src, dst, transform : if C1 represents transform from world to cam 1...
	transfer3DPoints(pc1, pc2, C1);	// trying C1inv...
	
	//Mat R1, t1;
	//decomposeTransform(C, R1, t1);
	//double deg1 = getRotationInDegrees(R1);
	//printf("%s << Forwards rotation = %f\n", __FUNCTION__, deg1);

	
	//printf("%s << DEBUG %d\n", __FUNCTION__, 2);
	//Cinv = C.inv();
	//transformationToProjection(Cinv, P1b);
	
	//Mat R2, t2;
	//decomposeTransform(Cinv, R2, t2);
	//double deg2 = getRotationInDegrees(R2);
	//printf("%s << Backwards rotation = %f\n", __FUNCTION__, deg2);
	
	//printf("%s << DEBUG %d\n", __FUNCTION__, 3);
	//TriangulatePoints(pts2, pts1, K, Kinv, P0, P1b, pc2, correspImg2Pt);

	//printf("%s << DEBUG %d\n", __FUNCTION__, 4);
	
	int inFront1 = 0, inFront2 = 0;
	
	for (unsigned int iii = 0; iii < pc1.size(); iii++) {
		
		// Check z-coordinate in both clouds to make sure it's > 0
		
		//printf("%s << pc1 = (%f, %f, %f); pc2 = (%f, %f, %f)\n", __FUNCTION__, pc1.at(iii).x, pc1.at(iii).y, pc1.at(iii).z, pc2.at(iii).x, pc2.at(iii).y, pc2.at(iii).z);
		
		if (pc1.at(iii).z > 0) {
			inFront1++;
		}
		
		if (pc2.at(iii).z > 0) {
			inFront2++;
		}
		
		if ((pc1.at(iii).z > 0) && (pc2.at(iii).z > 0)) {
			retVal++;
		}
		
	}
	
	//printf("%s << Pts in front of cams: %d, %d\n", __FUNCTION__, inFront1, inFront2);
	
	return retVal;
	
}

void convertPoint3dToMat(const cv::Point3d& src, cv::Mat& dst) {
	dst = cv::Mat::zeros(4, 1, CV_64FC1);
	
	dst.at<double>(3, 0) = 1.0;

	dst.at<double>(0,0) = src.x;
	dst.at<double>(1,0) = src.y;
	dst.at<double>(2,0) = src.z;
}

void transfer3dPoint(const cv::Point3d& src, cv::Point3d& dst, const cv::Mat& C) {
	
	cv::Mat pt1, pt2;
	
	pt1 = cv::Mat::zeros(4, 1, CV_64FC1);
	pt2 = cv::Mat::zeros(4, 1, CV_64FC1);
	
	pt1.at<double>(3, 0) = 1.0;
	pt2.at<double>(3, 0) = 1.0;

	
	pt1.at<double>(0,0) = src.x;
	pt1.at<double>(1,0) = src.y;
	pt1.at<double>(2,0) = src.z;
	
	pt2 = C * pt1;
	
	dst = cv::Point3d(pt2.at<double>(0,0), pt2.at<double>(1,0), pt2.at<double>(2,0));
	
}

void transfer3DPoints(const vector<cv::Point3d>& src, vector<cv::Point3d>& dst, const cv::Mat& C) {
	
	cv::Mat pt1, pt2;
	
	pt1 = cv::Mat::zeros(4, 1, CV_64FC1);
	pt2 = cv::Mat::zeros(4, 1, CV_64FC1);
	
	pt1.at<double>(3, 0) = 1.0;
	pt2.at<double>(3, 0) = 1.0;
	
	cv::Point3d shiftedPoint;
		
	for (unsigned int iii = 0; iii < src.size(); iii++) {
		pt1.at<double>(0,0) = src.at(iii).x;
		pt1.at<double>(1,0) = src.at(iii).y;
		pt1.at<double>(2,0) = src.at(iii).z;
		
		pt2 = C * pt1;		// pt2 = C.inv() * pt1;
		
		//printf("%s << pt2.at<double>(3,0) = %f\n", __FUNCTION__, pt2.at<double>(3,0));
		
		shiftedPoint.x = pt2.at<double>(0,0);
		shiftedPoint.y = pt2.at<double>(1,0);
		shiftedPoint.z = pt2.at<double>(2,0);
		
		dst.push_back(shiftedPoint);
	}
	
	
	
}



double calcInlierGeometryDistance(vector<cv::Point2f>& points1, vector<cv::Point2f>& points2, cv::Mat& mat, cv::Mat& mask, int distMethod) {
	
	double error = 0.0;
	
	int inlierCounter = 0;

	for (unsigned int iii = 0; iii < points1.size(); iii++) {
		if (mask.at<char>(iii, 0) > 0) {
			
			error += calcGeometryDistance(points1.at(iii), points2.at(iii), mat, distMethod);
			
			inlierCounter++;
		}
	}
	
	error /= (double) inlierCounter;

	return error;
}

double calcGeometryScore(vector<cv::Point2f>& points1, vector<cv::Point2f>& points2, cv::Mat& F, cv::Mat& Fmask, cv::Mat& H, cv::Mat& Hmask) {
	double gScore = 0.0;
	
	int inliers_H = 0, inliers_F = 0;
	
	/*
	for (int iii = 0; iii < points1.size(); iii++) {
		if (Hmask.at<char>(iii, 0) > 0) {
			inliers_H++;
		}
		
		if (Fmask.at<char>(iii, 0) > 0) {
			inliers_F++;
		}		
	}
	* */
	
	inliers_H = countNonZero(Hmask);
	inliers_F = countNonZero(Fmask);
	
	printf("%s << Inliers: %d (H), %d (F)\n", __FUNCTION__, inliers_H, inliers_F);
	
	double sampson_H, sampson_F;
	
	sampson_H = calcSampsonError(points1, points2, H, Hmask);
	sampson_F = calcSampsonError(points1, points2, F, Fmask);
	
	double sampson_H2 = 0.0, sampson_F2 = 0.0;
	
	printf("%s << Sampson errors: %f (H), %f (F)\n", __FUNCTION__, sampson_H, sampson_F);
	
	for (unsigned int iii = 0; iii < points1.size(); iii++) {
		sampson_H2 += calcSampsonDistance(points1.at(iii), points2.at(iii), H) / (double)points1.size();
		sampson_F2 += calcSampsonDistance(points1.at(iii), points2.at(iii), F) / (double)points1.size();
	}
	
	printf("%s << Sampson errors2: %f (H), %f (F)\n", __FUNCTION__, sampson_H2, sampson_F2);
	
	
	gScore = (((double) inliers_F) / ((double) inliers_H)) * ( sampson_H / pow(sampson_F, 0.5));
	
	printf("%s << gScore = %f\n", __FUNCTION__, gScore);
	
	return gScore;
}

double calcFrameScore(double geomScore, int numFeatures, int numTracks) {
	
	double frameScore;
	
	frameScore = geomScore * ((double) numTracks) / ((double) numFeatures);
	
	return frameScore;
}
 
double calcSampsonError(vector<cv::Point2f>& points1, vector<cv::Point2f>& points2, cv::Mat& H, cv::Mat& Hmask) {
	double sampsonError = 0.0;
	
	int inlierCounter = 0;

	
	for (unsigned int iii = 0; iii < points1.size(); iii++) {
		if (Hmask.at<char>(iii, 0) > 0) {
			
			sampsonError += lourakisSampsonError(points1.at(iii), points2.at(iii), H);
			
			inlierCounter++;
		}
	}
	
	sampsonError /= (double) inlierCounter;

	return sampsonError;

}

// http://www.mathworks.com.au/help/toolbox/vision/ref/estimatefundamentalmatrix.html
double calcSampsonDistance(cv::Point2f& pt1, cv::Point2f& pt2, cv::Mat& F) {
	
	double dist;
	
	cv::Mat point1(1, 3, CV_64FC1), point2(1, 3, CV_64FC1);
	
	point1.at<double>(0, 0) = (double) pt1.x;
	point1.at<double>(0, 1) = (double) pt1.y;
	point1.at<double>(0, 2) = 1.0;
	
	cv::Mat t1;
	transpose(point1, t1);
	
	point2.at<double>(0, 0) = (double) pt2.x;
	point2.at<double>(0, 1) = (double) pt2.y;
	point2.at<double>(0, 2) = 1.0;
	
	cv::Mat a, b, c;
	
	a = point2 * F * t1;
	b = F * t1;
	c = point2 * F;
	
	double val1 = pow(a.at<double>(0, 0), 2.0);
	double val2 = 1.0 / ((pow(b.at<double>(0, 0), 2.0)) + (pow(b.at<double>(1, 0), 2.0)));
	double val3 = 1.0 / ((pow(c.at<double>(0, 0), 2.0)) + (pow(c.at<double>(1, 0), 2.0)));
	
	dist = val1 * (val2 + val3);
	
	return dist;
	
}

double calcGeometryScore(int numInliers_H, int numInliers_F, double sampsonError_H, double sampsonError_F) {
	double geometryScore = 0.0;
	
	geometryScore = ((double) numInliers_F / (double) numInliers_H) * sampsonError_H / pow(sampsonError_F, 0.5);
	
	return geometryScore;
}

void assignIntrinsicsToP0(cv::Mat& P0, const cv::Mat& K) {
	P0 = cv::Mat::zeros(3, 4, CV_64FC1);
	
	for (int iii = 0; iii < 3; iii++) {
		for (int jjj = 0; jjj < 3; jjj++) {
			P0.at<double>(iii,jjj) = K.at<double>(iii,jjj);
		}
	}
	
}

double getQuaternionAngle(const Quaterniond& q1, const Quaterniond& q2) {
	double angle, dot_prod;
	
	dot_prod = dotProduct(q1, q2);
	
	angle = 2.0 * acos(abs(dot_prod));
	
	return angle;
}

double dotProduct(const Quaterniond& q1, const Quaterniond& q2) {
	double prod;
	
	prod = q1.x()*q2.x() + q1.y()*q2.y() + q1.z()*q2.z() + q1.w()*q2.w();
	
	return prod;
}

void combineTransforms(cv::Mat& CN, const cv::Mat& C0, const cv::Mat& C1) {
	CN = cv::Mat::eye(4, 4, CV_64FC1);
	
	CN = C0 * C1;
}

double dotProduct(const cv::Mat& vec1, const cv::Mat& vec2) {
	double prod;
	
	prod = vec1.at<double>(0,0)*vec2.at<double>(0,0) + vec1.at<double>(0,0)*vec2.at<double>(0,0) + vec1.at<double>(0,0)*vec2.at<double>(0,0);

	return prod;

}

double getRotationInDegrees(const cv::Mat& R) {
	Quaterniond rotation;
	matrixToQuaternion(R, rotation);
	Quaterniond dq = defaultQuaternion();
	double qa = getQuaternionAngle(rotation, dq) * 180.0 / M_PI;
	
	return qa;

}

double getDistanceInUnits(const cv::Mat& t) {
	double retVal = 0.0;
	
	retVal += pow(t.at<double>(0,0), 2.0);
	retVal += pow(t.at<double>(1,0), 2.0);
	retVal += pow(t.at<double>(2,0), 2.0);
	retVal = pow(retVal, 0.5);
	
	return retVal;
}

void convertVec4dToMat(const Vector4d& vec4, cv::Mat& mat) {
	mat = cv::Mat::zeros(3, 1, CV_64FC1);
	
	mat.at<double>(0,0) = vec4.x();
	mat.at<double>(1,0) = vec4.y();
	mat.at<double>(2,0) = vec4.z();
	
}

#ifdef _USE_SBA_
void addFixedCamera(SysSBA& sys, cameraParameters& cameraData, const cv::Mat& C) {
	addBlankCamera(sys, cameraData, true);
	updateCameraNode_2(sys, C, sys.nodes.size()-1);
}

void addNewCamera(SysSBA& sys, cameraParameters& cameraData, const cv::Mat& C) {
	addBlankCamera(sys, cameraData, false);
	updateCameraNode_2(sys, C, sys.nodes.size()-1);
}

void updateTracks(vector<featureTrack>& trackVector, const SysSBA& sys) {
	cv::Point3d tmpPt;
	for (unsigned int iii = 0; iii < sys.tracks.size(); iii++) {
		if (iii < trackVector.size()) {
			tmpPt.x = sys.tracks[iii].point.x();
			tmpPt.y = sys.tracks[iii].point.y();
			tmpPt.z = sys.tracks[iii].point.z();
			trackVector.at(iii).set3dLoc(tmpPt);
		} // otherwise these points mightn't belong in the track vector..
		
	}
}

void retrieveAllPoints(vector<cv::Point3d>& pts, const SysSBA& sys) {
	
	cv::Point3d point_3;
	
	for (unsigned int iii = 0; iii < sys.tracks.size(); iii++) {
		point_3.x = sys.tracks[iii].point.x();
		point_3.y = sys.tracks[iii].point.y();
		point_3.z = sys.tracks[iii].point.z();
			
		pts.push_back(point_3);
	}
}

void retrieveCameraPose(const SysSBA& sys, unsigned int idx, cv::Mat& camera) {
	
	cv::Mat R, t;
	quaternionToMatrix(sys.nodes[idx].qrot, R);
	convertVec4dToMat(sys.nodes[idx].trans, t);
	composeTransform(R, t, camera);
	
}

double retrieveCameraPose(const SysSBA& sys, unsigned int idx, geometry_msgs::Pose& pose) {
	
	cv::Point3d sysPt(sys.nodes[idx].trans(0), sys.nodes[idx].trans(1), sys.nodes[idx].trans(2));
	cv::Point3d posePt(pose.position.x, pose.position.y, pose.position.z);
	
	double dist = distBetweenPts(sysPt, posePt);
	
	
	pose.orientation.w = sys.nodes[idx].qrot.w();
	pose.orientation.x = sys.nodes[idx].qrot.x();
	pose.orientation.y = sys.nodes[idx].qrot.y();
	pose.orientation.z = sys.nodes[idx].qrot.z();
	
	pose.position.x = sys.nodes[idx].trans(0);
	pose.position.y = sys.nodes[idx].trans(1);
	pose.position.z = sys.nodes[idx].trans(2);
	
	return dist;
	
}
		

void retrieveAllCameras(cv::Mat *allCameraPoses, const SysSBA& sys) {
	
	for (unsigned int iii = 0; iii < sys.nodes.size(); iii++) {
		//if (allCameraPoses[iii].rows < 4) {
			
		retrieveCameraPose(sys, iii, allCameraPoses[iii]);
			
		/*
		Mat R, t, C;
		quaternionToMatrix(sys.nodes[iii].qrot, R);
		convertVec4dToMat(sys.nodes[iii].trans, t);
		
		compileTransform(C, R, t);
		C.copyTo(allCameraPoses[iii]);
		*/
		//} 
	}
}

void assignTracksToSBA(SysSBA& sys, vector<featureTrack>& trackVector, int maxIndex) {
	
	sys.tracks.clear();
	
	// ConnMat ??
	sys.connMat.clear();
	
	
	//printf("%s << Assigning (%d) tracks...\n", __FUNCTION__, trackVector.size());
	
	
	// For each track
	for (unsigned int iii = 0; iii < trackVector.size(); iii++) {
		
		// The first occurence of this feature has to be before the "maxIndex"
		if (((int)trackVector.at(iii).locations.at(0).imageIndex) < maxIndex) {
			cv::Point3d tmp3dPt = trackVector.at(iii).get3dLoc();
			Vector4d temppoint(tmp3dPt.x, tmp3dPt.y, tmp3dPt.z, 1.0);
			sys.addPoint(temppoint);
		}
		
	}
	
	// For each track
	for (unsigned int iii = 0; iii < trackVector.size(); iii++) {
		Vector2d proj;
		
		// The first occurence of this feature has to be before the "maxIndex"
		if (((int)trackVector.at(iii).locations.at(0).imageIndex) < maxIndex) {
			
			// For each projection
			for (unsigned int jjj = 0; jjj < trackVector.at(iii).locations.size(); jjj++) {

				// Projection cannot be in an image beyond the currently "active" camera
				if (trackVector.at(iii).locations.at(jjj).imageIndex < (maxIndex+1)) {
					proj.x() = trackVector.at(iii).locations.at(jjj).featureCoord.x;
					proj.y() = trackVector.at(iii).locations.at(jjj).featureCoord.y;
					
					sys.addMonoProj(trackVector.at(iii).locations.at(jjj).imageIndex, iii, proj);
				}

				
			}
		}
		
							
	}
	
}
#endif

void estimateNewPose(vector<featureTrack>& tracks, cv::Mat& K, int idx, cv::Mat& pose) {
	// Find any tracks that have at least 2 "sightings" (so that 3d location is known)
	// Use 3D points and 2d locations to estimate R and t and then get pose
	
	vector<cv::Point3d> worldPoints;
	vector<cv::Point2f> imagePoints1, imagePoints2;
	
	cv::Mat blankCoeffs = cv::Mat::zeros(1, 8, CV_64FC1);
	
	for (unsigned int iii = 0; iii < tracks.size(); iii++) {
		if (tracks.at(iii).locations.size() >= 2) {
			
			int finalIndex = tracks.at(iii).locations.size()-1;
			
			if (((int)tracks.at(iii).locations.at(finalIndex).imageIndex) == idx) {
				//Point3d tmpPt(tracks.at(iii).xyzEstimate.x, tracks.at(iii).xyzEstimate.y, tracks.at(iii).xyzEstimate.z);
				worldPoints.push_back(tracks.at(iii).get3dLoc());
				cv::Point2f tmpPt2(tracks.at(iii).locations.at(finalIndex-1).featureCoord.x, tracks.at(iii).locations.at(finalIndex-1).featureCoord.y);
				imagePoints1.push_back(tmpPt2);
				tmpPt2 = cv::Point2f(tracks.at(iii).locations.at(finalIndex).featureCoord.x, tracks.at(iii).locations.at(finalIndex).featureCoord.y);
				imagePoints2.push_back(tmpPt2);
			}
			
		}
	}
	
	
	cv::Mat Rvec, R, t;

	vector<cv::Point3f> objectPoints;
	cv::Point3f tmpPt;
	
	for (unsigned int jjj = 0; jjj < worldPoints.size(); jjj++) {
		tmpPt = cv::Point3f((float) worldPoints.at(jjj).x, (float) worldPoints.at(jjj).y, (float) worldPoints.at(jjj).z);
		objectPoints.push_back(tmpPt);
	}
	
	//solvePnPRansac(InputArray objectPoints, InputArray imagePoints, InputArray cameraMatrix, InputArray distCoeffs, OutputArray rvec, OutputArray tvec, bool useExtrinsicGuess=false, int iterationsCount=100, float reprojectionError=8.0, int minInliersCount=100, OutputArray inliers=noArray() );
	solvePnPRansac(objectPoints, imagePoints2, K, blankCoeffs, Rvec, t);
	Rodrigues(Rvec, R);

	//compileTransform(pose, R, t);
	composeTransform(R, t, pose);
}

#ifdef _USE_SBA_
void addNewPoints(SysSBA& sys, const vector<cv::Point3d>& pc) {
	
	Vector2d proj;
	
	proj.x() = 0.0;
	proj.y() = 0.0;
	
	for (unsigned int iii = 0; iii < pc.size(); iii++) {
		Vector4d temppoint(pc.at(iii).x, pc.at(iii).y, pc.at(iii).z, 1.0);
			
		sys.addPoint(temppoint);
		
		sys.addMonoProj(0, iii, proj);
	}
}

int addToTracks(SysSBA& sys, int im1, vector<cv::Point2f>& pts1, int im2, vector<cv::Point2f>& pts2) {
	
	//printSystemSummary(sys);
	
	//printf("%s << Entered.\n", __FUNCTION__);
	
	Vector2d proj;
	
	//printf("%s << oldTrackCount = %d\n", __FUNCTION__, oldTrackCount);
	//printf("%s << nodes = %d\n", __FUNCTION__, sys.nodes.size());
	
	for (unsigned int iii = 0; iii < pts1.size(); iii++) {
		
		bool pointInTracks = false;
		
		//printf("%s << [%d][%d][%d] : tracks = %d\n", __FUNCTION__, im1, im2, iii, sys.tracks.size());
		
		for (unsigned int ttt = 0; ttt < sys.tracks.size(); ttt++) { // NOT oldTrackCount, in case of automatic re-ordering...
			
			//printf("%s << old_track(%d) size = %d\n", __FUNCTION__, ttt, sys.tracks.at(ttt).projections.size());
			
			int finalIndex = sys.tracks.at(ttt).projections.size() - 1;
			
			//printf("%s << [%d] finalIndex = %d\n", __FUNCTION__, ttt, finalIndex);
			
			if (finalIndex > -1) {
				
				//printf("%s << DEBUG[%d] (%d vs %d)\n", __FUNCTION__, 0, finalIndex, sys.tracks.at(ttt).projections.size());
				
				printf("%s << Attempting [%d][%d][%d][%d]\n", __FUNCTION__, im1, im2, iii, ttt);
				printf("%s << ndi(%d / %d) = \n", __FUNCTION__, finalIndex, ((int)sys.tracks.at(ttt).projections.size()));
				
				printf("%s << sizeof() %d\n", __FUNCTION__, int(sizeof(sys.tracks.at(ttt).projections)));
				
				if (finalIndex > 0) {
					printf("%s << (-1) %d\n", __FUNCTION__, sys.tracks.at(ttt).projections.at(finalIndex-1).ndi);
				}
				
				printf("%s << %d\n", __FUNCTION__, sys.tracks.at(ttt).projections.at(finalIndex).ndi);
				
				if ((im1 == 2) && (im2 == 3) && (iii == 16) && (ttt == 68)) {
					//cin.get();
				}
				// 3, 4, 0, #49/50 track
				
				// why would it say that projections is large enough, but then you can't read ndi...?
				// or is size returning the wrong info?
				
				if (sys.tracks.at(ttt).projections.at(finalIndex).ndi == im1) {

					//printf("%s << DEBUG[%d]\n", __FUNCTION__, 1);
					
					if ((sys.tracks.at(ttt).projections.at(finalIndex).kp.x() == pts1.at(iii).x) && (sys.tracks.at(ttt).projections.at(finalIndex).kp.y() == pts1.at(iii).y)) {
						
						//printf("%s << DEBUG[%d]\n", __FUNCTION__, 2);
						
						//printf("%s << Point exists in this track (%d)!\n", __FUNCTION__, ttt);
						
						pointInTracks = true;
						
						proj.x() = pts2.at(iii).x;
						proj.y() = pts2.at(iii).y;
						
						if (!sys.addMonoProj(im2, ttt, proj)) {
							printf("%s << Adding point failed!\n", __FUNCTION__);
						}
						
						//printf("%s << Projection added.\n", __FUNCTION__);
						
						break;
						
					}
					
					//printf("%s << DEBUG[%d]\n", __FUNCTION__, 3);
				}
				
				//printf("%s << DEBUG[%d]\n", __FUNCTION__, 4);
			}
			
			

		}
		
		if (!pointInTracks) {
			
			//printf("%s << Point not in any of the tracks...\n", __FUNCTION__);
			
			int newTrackNum = sys.tracks.size();
			
			Vector4d temppoint(0.0, 0.0, 0.0, 1.0);
			
			//temppoint.x() = 0.0;
			//temppoint.y() = 0.0;
			//temppoint.z() = 0.0;
			sys.addPoint(temppoint);
			
			// Then add a track?
			//sys.tracks.push_back(Track(temppoint));
			
			proj.x() = pts1.at(iii).x;
			proj.y() = pts1.at(iii).y;
			
			if (!sys.addMonoProj(im1, newTrackNum, proj)) {
				printf("%s << Adding point failed!\n", __FUNCTION__);
			}
			
			proj.x() = pts2.at(iii).x;
			proj.y() = pts2.at(iii).y;
			
			if (!sys.addMonoProj(im2, newTrackNum, proj)) {
				printf("%s << Adding point failed!\n", __FUNCTION__);
			}
			
			//printf("%s << Two projections added.\n", __FUNCTION__);
		} else {
			//printf("%s << Point already in tracks...\n", __FUNCTION__);
		}
		
		
	}
	
	return sys.tracks.size();
}

void addBlankCamera(SysSBA& sys, cameraParameters& cameraData, bool isFixed) {
	
	frame_common::CamParams cam_params;
	
	
    cam_params.fx = cameraData.K.at<double>(0, 0); 	// Focal length in x
    cam_params.fy = cameraData.K.at<double>(1, 1); 	// Focal length in y
    
    cam_params.cx = cameraData.K.at<double>(0, 2); 	// X position of principal point
    cam_params.cy = cameraData.K.at<double>(1, 2); 	// Y position of principal point
    cam_params.tx = 0;   							// Baseline (no baseline since this is monocular)
    
    // printf("%s << Added camera: (%f, %f, %f, %f)\n", __FUNCTION__, cam_params.fx, cam_params.fy, cam_params.cx, cam_params.cy);
    
	Vector4d trans(0, 0, 0, 1);
            
	// Don't rotate.
	Quaterniond rot(1, 0, 0, 0);
	rot.normalize();
	
	//printf("%s << rot = (%f, %f, %f, %f)\n", __FUNCTION__, rot.x(), rot.y(), rot.z(), rot.w());

	// Add a new node to the system.
	sys.addNode(trans, rot, cam_params, isFixed);	// isFixed
	
	sys.nodes.at(sys.nodes.size()-1).setDr(true);
	
	if (isFixed) {
		// sys.nFixed++;
	}
	
	/*
	sys.nodes[currIndex].normRot();
	sys.nodes[currIndex].setTransform();
	sys.nodes[currIndex].setProjection();
	sys.nodes[currIndex].setDr(true);
	*/

}

void printSystemSummary(SysSBA& sys) {
	printf("%s << sys.nodes.size() = %d\n", __FUNCTION__, ((int)sys.nodes.size()));
	printf("%s << sys.tracks.size() = %d\n", __FUNCTION__, ((int)sys.tracks.size()));
}

void updateCameraNode_2(SysSBA& sys, const cv::Mat& C, int image_index) {
	cv::Mat R, t;
	
	cv::Mat Cnew;
	
	if (C.rows == 4) {
		C.copyTo(Cnew);
	} else {
		Cnew = cv::Mat(4, 4, CV_64FC1);
		for (unsigned int iii = 0; iii < 3; iii++) {
			for (unsigned int jjj = 0; jjj < 4; jjj++) {
				Cnew.at<double>(iii,jjj) = C.at<double>(iii,jjj);
				
			}
		}
		
		Cnew.at<double>(3,0) = 0.0;
		Cnew.at<double>(3,1) = 0.0;
		Cnew.at<double>(3,2) = 0.0;
		Cnew.at<double>(3,3) = 1.0;
	}
	
	cv::Mat C_inv = Cnew.inv();
	
	decomposeTransform(C, R, t);
	
	for (unsigned int iii = 0; iii < 3; iii++) {
		for (unsigned int jjj = 0; jjj < 4; jjj++) {
			sys.nodes.at(image_index).w2n(iii,jjj) = C.at<double>(iii,jjj);
		}
	}
	
	updateCameraNode_2(sys, R, t, image_index);
}

void updateCameraNode_2(SysSBA& sys, const cv::Mat& R, const cv::Mat& t, int image_index) {
	Vector4d translation;
	Quaterniond rotation;
	
	// printf("%s << t = (%f, %f, %f, %f)\n", __FUNCTION__, t.at<double>(0,0), t.at<double>(1,0), t.at<double>(2,0), t.at<double>(3,0));
	
	translation.x() = t.at<double>(0,0);
	translation.y() = t.at<double>(1,0);
	translation.z() = t.at<double>(2,0);
	
	//printf("%s << Assigning (%d) translation: (%f, %f, %f)\n", __FUNCTION__, image_index, translation.x(), translation.y(), translation.z());
	
	matrixToQuaternion(R, rotation);
	
	//Quaterniond dq = defaultQuaternion();
	//double qa = getQuaternionAngle(rotation, dq);
	
	//printf("%s << Angle = %f / %f\n", __FUNCTION__, qa, qa * 180.0 / M_PI);
	
	//cout << __FUNCTION__ << " << R = " << R << endl;
	
	//printf("%s << rotation = (%f, %f, %f, %f)\n", __FUNCTION__, rotation.x(), rotation.y(), rotation.z(), rotation.w());
	
	//cv::Mat R2;
	//quaternionToMatrix(rotation, R2);
	
	//cout << __FUNCTION__ << " << R2 = " << R2 << endl;
	
	sys.nodes[image_index].trans = translation;
	sys.nodes[image_index].qrot = rotation;
	
}
#endif

Quaterniond defaultQuaternion() {
	Quaterniond defQuaternion(1, 0, 0, 0);
	
	return defQuaternion;
}

void convertProjectionMatCVToEigen(const cv::Mat& mat, Eigen::Matrix< double, 3, 4 >& m) {
	
	m(0,0) = mat.at<double>(0,0);
	m(0,1) = mat.at<double>(0,1);
	m(0,2) = mat.at<double>(0,2);
	m(0,3) = mat.at<double>(0,3);
	
	m(1,0) = mat.at<double>(1,0);
	m(1,1) = mat.at<double>(1,1);
	m(1,2) = mat.at<double>(1,2);
	m(1,3) = mat.at<double>(1,3);
	
	m(2,0) = mat.at<double>(2,0);
	m(2,1) = mat.at<double>(2,1);
	m(2,2) = mat.at<double>(2,2);
	m(2,3) = mat.at<double>(2,3);
	
}

void convertProjectionMatEigenToCV(const Eigen::Matrix< double, 3, 4 >& m, cv::Mat& mat) {
	
	mat = cv::Mat::zeros(3, 4, CV_64FC1);
	
	mat.at<double>(0,0) = m(0,0);
	mat.at<double>(0,1) = m(0,1);
	mat.at<double>(0,2) = m(0,2);
	mat.at<double>(0,3) = m(0,3);
	
	mat.at<double>(1,0) = m(1,0);
	mat.at<double>(1,1) = m(1,1);
	mat.at<double>(1,2) = m(1,2);
	mat.at<double>(1,3) = m(1,3);
	
	mat.at<double>(2,0) = m(2,0);
	mat.at<double>(2,1) = m(2,1);
	mat.at<double>(2,2) = m(2,2);
	mat.at<double>(2,3) = m(2,3);
	
}



// J.M.P. van Waveren
float ReciprocalSqrt( float x ) {
	long i;
	float y, r;
	
	y = x * 0.5f;
	i = *(long *)( &x );
	i = 0x5f3759df - ( i >> 1 );
	r = *(float *)( &i );
	r = r * ( 1.5f - r * r * y );
	return r;
}

#ifdef _USE_SBA_
void updateCameraNode(SysSBA& sys, cv::Mat R, cv::Mat t, int img1, int img2) {
	
	cv::Mat full_R;
	
	Rodrigues(R, full_R);
	
	printf("%s << Trying to determine position of camera node (%d, %d)...\n", __FUNCTION__, img1, img2);
	
	cout << "R = " << R << endl;
	cout << "t = " << t << endl;
	
	// First camera parameters
	Vector4d temptrans = sys.nodes[img1].trans;
	Quaterniond tempqrot = sys.nodes[img1].qrot;
	
	//printf("%s << Debug [%d]\n", __FUNCTION__, 0);
	
	// NEED TO COMBINE TRANSLATION AND QUATERNION FOR PREVIOUS CAMERA WITH THE ONE FOR THE PRESENT ONE...
	// NEED TO KNOW: 	CONVERSION FROM R/t to quaternion
	//					mapping of one co-ordinate system onto another
	
	// Establish ORIGIN relative to NODE A
			Vector4d temptrans_N;
			Quaterniond tempqrot_N;
			temptrans_N.x() = -temptrans.x();
			temptrans_N.y() = -temptrans.y();
			temptrans_N.z() = -temptrans.z();
			tempqrot_N.x() = tempqrot.x();
			tempqrot_N.y() = tempqrot.y();
			tempqrot_N.z() = tempqrot.z();
			tempqrot_N.w() = -tempqrot.w();
			tempqrot_N.normalize();
			
			sba::Node negativeOrigin;
			negativeOrigin.trans = temptrans_N;
			negativeOrigin.qrot = tempqrot_N;
	
	//Mat testMat;
	//Quaterniond testQuaterniond;
	//printf("%s << q(1) = (%f, %f, %f, %f)\n", __FUNCTION__, tempqrot_N.x(), tempqrot_N.y(), tempqrot_N.z(), tempqrot_N.w());
	//convertFromQuaternionToMat(tempqrot_N, testMat);
	//cout << "testMat = " << testMat << endl;
	//convertFromMatToQuaternion(testMat, testQuaterniond);
	//printf("%s << q(2) = (%f, %f, %f, %f)\n", __FUNCTION__, testQuaterniond.x(), testQuaterniond.y(), testQuaterniond.z(), testQuaterniond.w());	
	//convertFromQuaternionToMat(testQuaterniond, testMat);
	//cout << "testMat2 = " << testMat << endl;
	
	//cout << "temptrans_N = " << temptrans_N << endl;
	//cout << "tempqrot_N = " << tempqrot_N << endl;
			
	//printf("%s << Debug [%d]\n", __FUNCTION__, 1);
			
	// Establish NODE B relative to NODE A
				
			Vector4d temptrans_B;
			Quaterniond tempqrot_B;
			temptrans_B.x() = t.at<double>(0,0);
			temptrans_B.y() = t.at<double>(1,0);
			temptrans_B.z() = t.at<double>(2,0);
			
			matrixToQuaternion(full_R, tempqrot_B);
			
			//cout << "full_R = " << full_R << endl;
			//printf("%s << tempqrot_B = (%f, %f, %f, %f)\n", __FUNCTION__, tempqrot_B.x(), tempqrot_B.y(), tempqrot_B.z(), tempqrot_B.w());
			
			//tempqrot_B.x() = R.at<double>(0,0);		// not correct conversion to quaternion
			//tempqrot_B.y() = R.at<double>(1,0);
			//tempqrot_B.z() = R.at<double>(2,0);
			tempqrot_B.normalize();
			
			sba::Node secondNode;
			secondNode.trans = temptrans_B;
			secondNode.qrot = tempqrot_B;
			
	//printf("%s << Debug [%d]\n", __FUNCTION__, 2);
	
	// Find transform from Origin to camera B

			Eigen::Matrix< double, 4, 1 > transformationMat;
			Eigen::Quaternion< double > quaternionVec;

			sba::transformN2N(transformationMat, quaternionVec, negativeOrigin, secondNode);
			
	//printf("%s << Debug [%d]\n", __FUNCTION__, 3);
			
	// Assign this transformation to the new camera mode
	
			temptrans.x() = transformationMat(0,0);
			temptrans.y() = transformationMat(1,0);
			temptrans.z() = transformationMat(2,0);
		
			tempqrot.x() = quaternionVec.x();
			tempqrot.y() = quaternionVec.y();
			tempqrot.z() = quaternionVec.z();
			tempqrot.normalize();

			sys.nodes[img2].trans = temptrans;
			sys.nodes[img2].qrot = tempqrot;

			sys.nodes[img2].normRot();
			sys.nodes[img2].setTransform();
			sys.nodes[img2].setProjection();
			sys.nodes[img2].setDr(true);
			
	//printf("%s << Debug [%d]\n", __FUNCTION__, 4);
	
	return;
	
	// OLD


	// Add the new translations
	temptrans.x() += t.at<double>(0,0);
	temptrans.y() += t.at<double>(1,0);
	temptrans.z() += t.at<double>(2,0);
	
	// This is probably not a valid conversion 
	
	tempqrot.x() += R.at<double>(0,0);
	tempqrot.y() += R.at<double>(1,0);
	tempqrot.z() += R.at<double>(2,0);
	tempqrot.normalize();

	sys.nodes[img2].trans = temptrans;
	sys.nodes[img2].qrot = tempqrot;

	// These methods should be called to update the node.
	
	sys.nodes[img2].normRot();
	sys.nodes[img2].setTransform();
	sys.nodes[img2].setProjection();
	sys.nodes[img2].setDr(true);
	
}


void constrainDodgyPoints(SysSBA& sys) {
	
	for (unsigned int iii = 0; iii < sys.nodes.size(); iii++) {
		if ((abs(sys.nodes[iii].trans.x()) > POSITION_LIMIT) || (abs(sys.nodes[iii].trans.y()) > POSITION_LIMIT) || (abs(sys.nodes[iii].trans.z()) > POSITION_LIMIT)) {
			sys.nodes[iii].trans.x() = 0.0;
			sys.nodes[iii].trans.y() = 0.0;
			sys.nodes[iii].trans.z() = 0.0;
		}
	}
	
	for (unsigned int iii = 0; iii < sys.tracks.size(); iii++) {
		if ((abs(sys.tracks[iii].point.x()) > POSITION_LIMIT) || (abs(sys.tracks[iii].point.y()) > POSITION_LIMIT) || (abs(sys.tracks[iii].point.z()) > POSITION_LIMIT)) {
			sys.tracks[iii].point.x() = 0.0;
			sys.tracks[iii].point.y() = 0.0;
			sys.tracks[iii].point.z() = 0.0;
		}
	}
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

void ExtractPointCloud(vector<cv::Point3d>& cloud, vector<featureTrack>& trackVector) {
	
	
	for (unsigned int iii = 0; iii < trackVector.size(); iii++) {
		// Copy the track vector co-ordinate
		cloud.push_back(trackVector.at(iii).get3dLoc());
	}
	
	
}



void reduceVectorsToTrackedPoints(const vector<cv::Point2f>& points1, vector<cv::Point2f>& trackedPoints1, const vector<cv::Point2f>& points2, vector<cv::Point2f>& trackedPoints2, vector<uchar>& statusVec) {
	
	trackedPoints1.clear();
	trackedPoints2.clear();
	
	for (unsigned int iii = 0; iii < statusVec.size(); iii++) {
		
		if (statusVec.at(iii) > 0) {
			//printf("%s << 1: statusVec.at(%d) = %c\n", __FUNCTION__, iii, statusVec.at(iii));
			trackedPoints1.push_back(points1.at(iii));
			trackedPoints2.push_back(points2.at(iii));
		} else {
			//printf("%s << 0: statusVec.at(%d) = %c\n", __FUNCTION__, iii, statusVec.at(iii));
		}
	}
	
}

bool findBestReconstruction(const cv::Mat& P0, cv::Mat& P1, cv::Mat& R, cv::Mat& t, const cv::SVD& svd, const cv::Mat& K, const vector<cv::Point2f>& pts1, const vector<cv::Point2f>& pts2, bool useDefault) {
	
	bool validity = false;
	
	//printf("%s << Entered.\n", __FUNCTION__);
	
	if (pts1.size() != pts2.size()) {
		printf("%s << ERROR! Point vector size mismatch.\n", __FUNCTION__);
	}
	
	int bestIndex = 0, bestScore = -1;
	cv::Mat t33, t_tmp[4], R_tmp[4], W, Z, Winv, I, negI, P1_tmp[4];
	cv::Mat Kinv;
	//printf("%s << DEBUG [%d].\n", __FUNCTION__, -2);
	//printf("%s << DEBUG [%d].\n", __FUNCTION__, -1);
	Kinv = K.inv();
	
	//printf("%s << DEBUG [%d].\n", __FUNCTION__, 0);
	
	cv::Mat zeroTrans;
	zeroTrans = cv::Mat::zeros(3, 1, CV_64F);	// correct dimension?
	
	I = cv::Mat::eye(3, 3, CV_64FC1);
	negI = -I;
	getWandZ(W, Winv, Z);
	
	//printf("%s << DEBUG [%d].\n", __FUNCTION__, 1);
	
	cv::Mat u_transpose;
	transpose(svd.u, u_transpose);
	
	cv::Mat rotation1;
	Rodrigues(I, rotation1);
	
	//printf("%s << DEBUG [%d].\n", __FUNCTION__, 2);

	//cout << "rotation1 = " << rotation1 << endl;
	
	float sig = rotation1.at<double>(0,1);
	float phi = rotation1.at<double>(0,2);

	//printf("%s << sig = %f; phi = %f\n", __FUNCTION__, sig, phi);

	cv::Point3d unitVec1(1.0*sin(sig)*cos(phi), 1.0*sin(sig)*cos(phi), 1.0*cos(sig));
	
	//printf("%s << DEBUG [%d].\n", __FUNCTION__, 3);

	cv::Mat uv1(unitVec1);
	
	//printf("%s << DEBUG [%d].\n", __FUNCTION__, 4);
	
	// Go through all four cases...
	for (int zzz = 0; zzz < 4; zzz++) {
		
		//printf("%s << Loop [%d]\n", __FUNCTION__, zzz);
		
		switch (zzz) {
			case 0:
				//t33 = v * W * SIGMA * vt;
				t33 = svd.u * Z * u_transpose;
				R_tmp[zzz] = svd.u * Winv * svd.vt;
				break;
			case 1:
				//t33 = v * W_inv * SIGMA * vt;
				t33 = negI * svd.u * Z * u_transpose;
				R_tmp[zzz] = svd.u * Winv * svd.vt;
				break;
			case 2:
				//t33 = v * W * SIGMA * vt;
				t33 = svd.u * Z * u_transpose;
				R_tmp[zzz] = svd.u * W * svd.vt;
				break;
			case 3:
				//t33 = v * W_inv * SIGMA * vt;
				t33 = negI * svd.u * Z * u_transpose;
				R_tmp[zzz] = svd.u * W * svd.vt;
				break;
			default:
				break;
		}
		
		// Converting 3x3 t mat to vector
		t_tmp[zzz] = cv::Mat::zeros(3, 1, CV_64F);
		t_tmp[zzz].at<double>(0,0) = -t33.at<double>(1,2);
		t_tmp[zzz].at<double>(1,0) = t33.at<double>(0,2);
		t_tmp[zzz].at<double>(2,0) = -t33.at<double>(0,1);
		
		// Get new P1 matrix
		findP1Matrix(P1_tmp[zzz], R_tmp[zzz], t_tmp[zzz]);
		
		// Find camera vectors
		cv::Mat rotation2;
		Rodrigues(R_tmp[zzz], rotation2);

		//cout << "rotation2 = " << rotation2 << endl;

		sig = rotation2.at<double>(0,1);
		phi = rotation2.at<double>(0,2);

		//printf("%s << sig = %f; phi = %f\n", __FUNCTION__, sig, phi);

		cv::Point3d unitVec2(1.0*sin(sig)*cos(phi), 1.0*sin(sig)*cos(phi), 1.0*cos(sig));

		cv::Mat uv2(unitVec2);

		//printf("%s << unitVec2 = (%f, %f, %f)\n", __FUNCTION__, unitVec2.x, unitVec2.y, unitVec2.z);
		
		int badCount = 0;
		
		vector<cv::Point3d> pointcloud;
		vector<cv::Point2f> correspImg1Pt;
		
		//printf("%s << DEBUG [%d].\n", __FUNCTION__, 7);
		
		TriangulatePoints(pts1, pts2, K, Kinv, P0, P1_tmp[zzz], pointcloud, correspImg1Pt);
		
		//printf("%s << DEBUG [%d]. (pointcloud.size() = %d)\n", __FUNCTION__, 8, pointcloud.size());
		
		cv::Mat Rs_k[2], ts_k[2];

		// Then check how many points are NOT in front of BOTH of the cameras
		for (unsigned int kkk = 0; kkk < pointcloud.size(); kkk++) {
			
			
			
			//printf("%s << DEBUG [%d][%d].\n", __FUNCTION__, kkk, 0);

			cv::Point3d rayVec1, rayVec2;

			Rs_k[0] = I;
			Rs_k[1] = R_tmp[zzz];

			ts_k[0] = zeroTrans;
			ts_k[1] = t_tmp[zzz];
			
			//printf("%s << DEBUG [%d][%d].\n", __FUNCTION__, kkk, 1);
			
			//cout << endl << "t_tmp[zzz] = " << t_tmp[zzz] << endl;

			//printf("%s << objpt = (%f, %f, %f)\n", __FUNCTION__, objpt.x, objpt.y, objpt.z);

			// so this is getting the displacements between the points and both cameras...
			rayVec1.x = pointcloud.at(kkk).x - ts_k[0].at<double>(0,0);
			rayVec1.y = pointcloud.at(kkk).y - ts_k[0].at<double>(1,0);
			rayVec1.z = pointcloud.at(kkk).z - ts_k[0].at<double>(2,0);

			rayVec2.x = pointcloud.at(kkk).x - ts_k[1].at<double>(0,0);
			rayVec2.y = pointcloud.at(kkk).y - ts_k[1].at<double>(1,0);
			rayVec2.z = pointcloud.at(kkk).z - ts_k[1].at<double>(2,0);
			
			/*
			if (kkk < 0) {
				printf("%s << pt(%d) = (%f, %f, %f)\n", __FUNCTION__, kkk, pointcloud.at(kkk).x, pointcloud.at(kkk).y, pointcloud.at(kkk).z);
				printf("%s << rayVec1(%d) = (%f, %f, %f)\n", __FUNCTION__, kkk, rayVec1.x, rayVec1.y, rayVec1.z);
				printf("%s << rayVec2(%d) = (%f, %f, %f)\n", __FUNCTION__, kkk, rayVec2.x, rayVec2.y, rayVec2.z);
			}
			* */
			
			//printf("%s << DEBUG [%d][%d].\n", __FUNCTION__, kkk, 2);

			//printf("%s << rayVec1 = (%f, %f, %f)\n", __FUNCTION__, rayVec1.x, rayVec1.y, rayVec1.z);
			//printf("%s << rayVec2 = (%f, %f, %f)\n", __FUNCTION__, rayVec2.x, rayVec2.y, rayVec2.z);

			//cin.get();

			double dot1, dot2;
			
			cv::Mat rv1(rayVec1), rv2(rayVec2);
			
			/*
			cout << endl << "uv1 = " << uv1 << endl;
			cout << endl << "rv1 = " << rv1 << endl;
			
			cout << endl << "uv2 = " << uv2 << endl;
			cout << endl << "rv2 = " << rv2 << endl;
			*/

			dot1 = uv1.dot(rv1);
			dot2 = uv2.dot(rv2);

			//printf("%s << DEBUG [%d][%d].\n", __FUNCTION__, kkk, 3);

			//printf("%s << dot1 = %f; dot2 = %f\n", __FUNCTION__, dot1, dot2);

			double mag1 = pow(pow(rayVec1.x, 2) + pow(rayVec1.y, 2) + pow(rayVec1.z, 2), 0.5);
			double mag2 = pow(pow(rayVec2.x, 2) + pow(rayVec2.y, 2) + pow(rayVec2.z, 2), 0.5);

			//printf("%s << mag1 = %f; mag2 = %f\n", __FUNCTION__, mag1, mag2);

			double ang1 = fmod(acos(dot1 / mag1), 2*M_PI);
			double ang2 = fmod(acos(dot2 / mag2), 2*M_PI);

			//printf("%s << ang1 = %f; ang2 = %f\n", __FUNCTION__, ang1, ang2);
			// just try to check if angle is greater than 90deg from where each camera is facing to the pt

			// get vector from each camera to the point

			// use dot product to determine angle
			if ((abs(ang1) < M_PI/2 ) || (abs(ang2) < M_PI/2)) {
				//printf("%s << ang1 = %f; ang2 = %f\n", __FUNCTION__, ang1, ang2);
			//if ((dot1 < 0.0) || (dot2 < 0.0)) {
				validity = false;
				badCount++;
			}

			if (rayVec1.z < 0) {
				//printf("%s << rayVec1.z < 0; abs(ang1) = %f\n", __FUNCTION__, abs(ang1));

			} else {
				//printf("%s << rayVec1.z >= 0; abs(ang1) = %f\n", __FUNCTION__, abs(ang1));

			}

			//cin.get();

		}

		printf("%s << badCount[%d] = %d\n", __FUNCTION__, zzz, badCount);
		
		if (zzz == 0) {
			bestScore = badCount;
		} else {
			if (badCount < bestScore) {
				bestScore = badCount;
				bestIndex = zzz;
			}
		}

		
	}
	
	if (bestScore == 0) {
		validity = true;
	}
	
	// Should be 2 - but try different ones...
	if (useDefault) {
		bestIndex = 2;
		printf("%s << Defaulting to model #%d\n", __FUNCTION__, bestIndex);
	}
	
	findP1Matrix(P1, R_tmp[bestIndex], t_tmp[bestIndex]);

	R_tmp[bestIndex].copyTo(R);
	t_tmp[bestIndex].copyTo(t);
	
	return validity;
}

void initializeP0(cv::Mat& P) {
	P = cv::Mat::zeros(3, 4, CV_64FC1);
	P.at<double>(0,0) = 1.0;
	P.at<double>(0,1) = 0.0;
	P.at<double>(0,2) = 0.0;
	P.at<double>(0,3) = 0.0;
	P.at<double>(1,0) = 0.0;
	P.at<double>(1,1) = 1.0;
	P.at<double>(1,2) = 0.0;
	P.at<double>(1,3) = 0.0;
	P.at<double>(2,0) = 0.0;
	P.at<double>(2,1) = 0.0;
	P.at<double>(2,2) = 1.0;
	P.at<double>(2,3) = 0.0;
}

void LinearLSTriangulation(cv::Mat& dst, const cv::Point3d& u1, const cv::Mat& P1, const cv::Point3d& u2, const cv::Mat& P2) {

	// https://github.com/MasteringOpenCV/code/blob/master/Chapter4_StructureFromMotion/Triangulation.cpp
	// http://www.morethantechnical.com/2012/01/04/simple-triangulation-with-opencv-from-harley-zisserman-w-code/
	
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
        if ((fabsf(wi1 - p1a) <= EPSILON) && (fabsf(wi2 - p2a) <= EPSILON)) {
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

void getWandZ(cv::Mat& W, cv::Mat& Winv, cv::Mat& Z) {
	W = cv::Mat::zeros(3, 3, CV_64FC1);
	
	W.at<double>(0,0) = 0.0;
	W.at<double>(0,1) = -1.0;
	W.at<double>(0,2) = 0.0;
	
	W.at<double>(1,0) = 1.0;
	W.at<double>(1,1) = 0.0;
	W.at<double>(1,2) = 0.0;
	
	W.at<double>(2,0) = 0.0;
	W.at<double>(2,1) = 0.0;
	W.at<double>(2,2) = 1.0;
	
	Winv = cv::Mat::zeros(3, 3, CV_64FC1);
	
	Winv.at<double>(0,0) = 0.0;
	Winv.at<double>(0,1) = 1.0;
	Winv.at<double>(0,2) = 0.0;
	
	Winv.at<double>(1,0) = -1.0;
	Winv.at<double>(1,1) = 0.0;
	Winv.at<double>(1,2) = 0.0;
	
	Winv.at<double>(2,0) = 0.0;
	Winv.at<double>(2,1) = 0.0;
	Winv.at<double>(2,2) = 1.0;	
	
	Z = cv::Mat::zeros(3, 3, CV_64FC1);
	
	Z.at<double>(0,1) = 1.0;
	Z.at<double>(1,0) = -1.0;
}

void findP1Matrix(cv::Mat& P1, const cv::Mat& R, const cv::Mat& t) {
	//cv::Mat W, Winv, Z;
	
	//getWandZ(W, Z);
	//Winv = W.inv();

	//R = svd.u * W * svd.vt; //HZ 9.19
	//t = svd.u.col(2); //u3
	
	P1 = cv::Mat(3, 4, CV_64FC1);
			
	P1.at<double>(0,0) = R.at<double>(0,0);
	P1.at<double>(0,1) = R.at<double>(0,1);
	P1.at<double>(0,2) = R.at<double>(0,2);
	P1.at<double>(0,3) = t.at<double>(0,0);
	
	P1.at<double>(1,0) = R.at<double>(1,0);
	P1.at<double>(1,1) = R.at<double>(1,1);
	P1.at<double>(1,2) = R.at<double>(1,2);
	P1.at<double>(1,3) = t.at<double>(1,0);
	
	P1.at<double>(2,0) = R.at<double>(2,0);
	P1.at<double>(2,1) = R.at<double>(2,1);
	P1.at<double>(2,2) = R.at<double>(2,2);
	P1.at<double>(2,3) = t.at<double>(2,0);
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

void findCentroid(vector<featureTrack>& tracks, cv::Point3d& centroid, cv::Point3d& stdDeviation) {
	
	unsigned int triangulatedPoints = 0;
	
	for (unsigned int iii = 0; iii < tracks.size(); iii++) {
		if (tracks.at(iii).isTriangulated) {
			triangulatedPoints++;
		}
	}
	
	printf("%s << triangulatedPoints = %d\n", __FUNCTION__, triangulatedPoints);
	
	centroid = cv::Point3d(0.0, 0.0, 0.0);
	stdDeviation = cv::Point3d(0.0, 0.0, 0.0);
	
	for (unsigned int iii = 0; iii < tracks.size(); iii++) {
		if (tracks.at(iii).isTriangulated) {
			centroid.x += (tracks.at(iii).get3dLoc().x / ((double) triangulatedPoints));
			centroid.y += (tracks.at(iii).get3dLoc().y / ((double) triangulatedPoints));
			centroid.z += (tracks.at(iii).get3dLoc().z / ((double) triangulatedPoints));
		}
	}
	
	for (unsigned int iii = 0; iii < tracks.size(); iii++) {
		if (tracks.at(iii).isTriangulated) {
			stdDeviation.x += (pow((tracks.at(iii).get3dLoc().x - centroid.x), 2.0) / ((double) triangulatedPoints));
			stdDeviation.y += (pow((tracks.at(iii).get3dLoc().y - centroid.y), 2.0) / ((double) triangulatedPoints));
			stdDeviation.z += (pow((tracks.at(iii).get3dLoc().z - centroid.z), 2.0) / ((double) triangulatedPoints));
		}
		
	}
	
	stdDeviation.x = pow(stdDeviation.x, 0.5);
	stdDeviation.y = pow(stdDeviation.y, 0.5);
	stdDeviation.z = pow(stdDeviation.z, 0.5);
	
}

#endif
