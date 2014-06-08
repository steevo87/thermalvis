/*! \file	sba.cpp
 *  \brief	Definitions for sparse bundle adjustment related functions.
*/

#ifdef _USE_PCL_
#ifdef _USE_SBA_

#include "sba.hpp"

void findRelevantIndices(vector<featureTrack>& tracks, vector<unsigned int>& triangulated, vector<unsigned int>& untriangulated, unsigned int last_index, unsigned int new_index) {
	
	//unsigned int min_sightings = max(0, (((int) new_index) - ((int) last_index)) / 2);
	
	for (unsigned int iii = 0; iii < tracks.size(); iii++) {
		
		if (tracks.at(iii).isTriangulated) {
			triangulated.push_back(iii);
			/*
			unsigned int sightings = 0;
			
			for (unsigned int jjj = 0; jjj < tracks.at(iii).locations.size(); jjj++) {
				
				if (tracks.at(iii).locations.at(jjj).imageIndex <= new_index) {
					sightings++;
				}
				
			}
			* */
			
			/*
			if (sightings >= min_sightings) {
				triangulated.push_back(iii);
			}
			* */
		} else {
			untriangulated.push_back(iii);
		}
		

		
	}
	
	
}

void findIntermediatePoses(vector<featureTrack>& tracks, cameraParameters& camData, cv::Mat *cameras, unsigned int image_idx_1, unsigned int image_idx_2, bool fixBothEnds) {
	
	printf("%s << ENTERED.\n", __FUNCTION__);
	
	vector<unsigned int> activeTrackIndices, fullSpanIndices, untriangulatedIndices;
	
	int subsequence_iters = 10;
			
	getActiveTracks(activeTrackIndices, tracks, image_idx_1, image_idx_2);
	filterToCompleteTracks(fullSpanIndices, activeTrackIndices, tracks, image_idx_1, image_idx_2);
	
	reduceActiveToTriangulated(tracks, fullSpanIndices, untriangulatedIndices);
	
	vector<cv::Point2f> pts1, pts2;
	getPointsFromTracks(tracks, pts1, pts2, image_idx_1, image_idx_2);
	
	vector<cv::Point3d> ptsInCloud;
	//printf("%s << ptsInCloud.size() (pre-update) = %d\n", __FUNCTION__, ptsInCloud.size());
	getActive3dPoints(tracks, fullSpanIndices, ptsInCloud);
	//printf("%s << ptsInCloud.size() (post-update) = %d\n", __FUNCTION__, ptsInCloud.size());
		
	int relativeIndex = image_idx_2 - image_idx_1;
		
	SysSBA subsys;
	addPointsToSBA(subsys, ptsInCloud);
	
	addFixedCamera(subsys, camData, cameras[image_idx_1]);
	addFixedCamera(subsys, camData, cameras[image_idx_2]);
	
	addProjectionsToSBA(subsys, pts1, 0);	// keyframe_idx_1
	addProjectionsToSBA(subsys, pts2, 1);
	
	if (fixBothEnds) {
		subsys.nFixed = 2;
	} else {
		subsys.nFixed = 1;
	}
	
	
	for (int jjj = 1; jjj < relativeIndex; jjj++) {
		
		vector<cv::Point2f> latestPoints;
		getCorrespondingPoints(tracks, pts1, latestPoints, image_idx_1, image_idx_1+jjj);
		
		
		
		vector<cv::Point3f> objectPoints;
		cv::Point3f tmpPt;
		
		for (unsigned int kkk = 0; kkk < ptsInCloud.size(); kkk++) {
			tmpPt = cv::Point3f((float) ptsInCloud.at(kkk).x, (float) ptsInCloud.at(kkk).y, (float) ptsInCloud.at(kkk).z);
			objectPoints.push_back(tmpPt);
		}
		
		cv::Mat t, R, Rvec;
		
		//printf("%s << Solving PnP... (%d) (oP.size() = %d; lP.size() = %d)\n", __FUNCTION__, jjj, objectPoints.size(), latestPoints.size());
		
		//solvePnPRansac(InputArray objectPoints, InputArray imagePoints, InputArray cameraMatrix, InputArray distCoeffs, OutputArray rvec, OutputArray tvec, bool useExtrinsicGuess=false, int iterationsCount=100, float reprojectionError=8.0, int minInliersCount=100, OutputArray inliers=noArray() );
		solvePnPRansac(objectPoints, latestPoints, camData.K, camData.blankCoeffs, Rvec, t);
		
		Rodrigues(Rvec, R);
		
		//cout << __FUNCTION__ << " << [" << jjj << "] R = " << R << endl;
		//cout << __FUNCTION__ << " << [" << jjj << "] t = " << t << endl;
		
		cv::Mat newCam;
		composeTransform(R, t, newCam);
		//compileTransform(newCam, R, t);
		
		// Inverting camera "learned" by PnP since it always seems to need to be inverted when passed to 
		// and from SBA...
		
		cv::Mat newCamC;
		projectionToTransformation(newCam, newCamC);
		newCamC = newCamC.inv();
		transformationToProjection(newCamC, newCam);
		
		addNewCamera(subsys, camData, newCam);
		
		addProjectionsToSBA(subsys, latestPoints, jjj+1);
		
		//avgError = optimizeSystem(subsys, 1e-4, 10);
		//printf("%s << Progressive subsequence BA error = %f.\n", __FUNCTION__, avgError);
		
		//drawGraph2(subsys, camera_pub, points_pub, decimation, bicolor);
		
	}
	
	printf("%s << About to optimize subsystem... (nFixed = %d)\n", __FUNCTION__, subsys.nFixed);
	//if (iii == 0) {
		
	double avgError = optimizeSystem(subsys, 1e-4, subsequence_iters);
	
	if (0) {
		//drawGraph2(subsys, camera_pub, points_pub, decimation, bicolor);
	}
	
	if (avgError < 0.0) {
		printf("%s << ERROR! Subsystem optimization failed to converge..\n", __FUNCTION__);
	}
	
	printf("%s << Subsystem optimized\n", __FUNCTION__);
	
	
		
		
	
	retrieveCameraPose(subsys, 0, cameras[image_idx_1]);
	retrieveCameraPose(subsys, 1, cameras[image_idx_2]);
	
	for (unsigned int ttt = 1; ttt < image_idx_2-image_idx_1; ttt++) {
		retrieveCameraPose(subsys, ttt+1, cameras[image_idx_1+ttt]);
	}
	
	ptsInCloud.clear();
	retrieveAllPoints(ptsInCloud, subsys);
	
	updateTriangulatedPoints(tracks, fullSpanIndices, ptsInCloud);	
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

void removePoorTracks(vector<featureTrack>& tracks, cameraParameters& camData, cv::Mat *cameras, unsigned int start_cam, unsigned int finish_cam) {

	//return;
	
	printf("%s << Entered.\n", __FUNCTION__);

	// go through each 3D point and find projection to each camera, if the projection is too far off
	// then remove it
	
	
	
	SysSBA sys;
	
	for (unsigned int iii = 0; iii <= finish_cam; iii++) {
		if (cameras[iii].rows < 3) {
			addBlankCamera(sys, camData);
		} else {
			addNewCamera(sys, camData, cameras[iii]);
			//cout << cameras[iii] << endl;
		}
		
		
	}
	
	printf("%s << sys.nodes.size() = %d\n", __FUNCTION__, int(sys.nodes.size()));
	
	for (unsigned int iii = 0; iii < tracks.size(); iii++) {
		if (tracks.at(iii).isTriangulated) {
			
			Vector4d point_3(tracks.at(iii).get3dLoc().x, tracks.at(iii).get3dLoc().y, tracks.at(iii).get3dLoc().z, 1.0);
			
			for (unsigned int jjj = 0; jjj < tracks.at(iii).locations.size(); jjj++) {
				
				unsigned int camIndex = tracks.at(iii).locations.at(jjj).imageIndex;
				
				//printf("%s << Checking (%d)(%d)(%d)...\n", __FUNCTION__, iii, jjj, camIndex);
				
				if ((camIndex >= start_cam) && (camIndex <= finish_cam)) {
					
					// Find 2d projection based on 3d point; compare with 2d loc stored in tracks structure
					
					Vector2d proj;
					
					//printf("%s << nodes(%d) trans = (%f, %f, %f)\n", __FUNCTION__, camIndex, sys.nodes.at(camIndex).trans.x(), sys.nodes.at(camIndex).trans.y(), sys.nodes.at(camIndex).trans.z());
					
					sys.nodes.at(camIndex).setProjection();
					
					
					//printf("%s << w2n = (%f, %f, %f; %f %f %f; ...)\n", __FUNCTION__, sys.nodes.at(camIndex).w2n(0,0), sys.nodes.at(camIndex).w2n(0,1), sys.nodes.at(camIndex).w2n(0,2), sys.nodes.at(camIndex).w2n(1,0), sys.nodes.at(camIndex).w2n(1,1), sys.nodes.at(camIndex).w2n(1,2));
					
					
					sys.nodes.at(camIndex).project2im(proj, point_3);
					
					//printf("%s << t(%d)c(%d) = (%f, %f) vs (%f, %f)\n", __FUNCTION__, iii, camIndex, proj.x(), proj.y(), tracks.at(iii).locations.at(jjj).featureCoord.x, tracks.at(iii).locations.at(jjj).featureCoord.y);
					
				}
				
			}
			
		}
	}
	
}

void copySys(const SysSBA& src, SysSBA& dst) {
	
	dst.tracks.clear();
	dst.nodes.clear();
	
	for (unsigned int iii = 0; iii < src.tracks.size(); iii++) {
		dst.tracks.push_back(src.tracks.at(iii));
	}
	
	for (unsigned int iii = 0; iii < src.nodes.size(); iii++) {
		dst.nodes.push_back(src.nodes.at(iii));
	}
	
}

void rescaleSBA(SysSBA& sba, unsigned int idx1, unsigned int idx2) {
	
	// Find the distance from first to last camera
	double totalDist = pow(pow(sba.nodes.at(idx2).trans.x() - sba.nodes.at(idx1).trans.x(), 2.0) + pow(sba.nodes.at(idx2).trans.y() - sba.nodes.at(idx1).trans.y(), 2.0) + pow(sba.nodes.at(idx2).trans.z() - sba.nodes.at(idx1).trans.z(), 2.0), 0.5);

	printf("%s << totalDist = %f\n", __FUNCTION__, totalDist);

	// Scale ALL locations by the ratio of this distance to 1.0
	for (unsigned int iii = 0; iii < sba.tracks.size(); iii++) {
		sba.tracks.at(iii).point.x() /= totalDist;
		sba.tracks.at(iii).point.y() /= totalDist;
		sba.tracks.at(iii).point.z() /= totalDist;
	}
	
	for (unsigned int iii = 0; iii < sba.nodes.size(); iii++) {
		sba.nodes.at(iii).trans.x() /= totalDist;
		sba.nodes.at(iii).trans.y() /= totalDist;
		sba.nodes.at(iii).trans.z() /= totalDist;
	}
	
	printf("%s << Exiting...\n", __FUNCTION__);

}

void renormalizeSBA(SysSBA& sba, cv::Point3d& desiredCenter) {
	
	cv::Point3d centroid, stdDeviation;
	
	// Find current centroid
	findCentroid(sba, centroid, stdDeviation);
	
	double largestAxis = 4.0 * max(stdDeviation.x, stdDeviation.z);
	double lowestHeight = -2.0 * stdDeviation.y;
	
	// Move everything to center around point (0,0,0)
	for (unsigned int iii = 0; iii < sba.tracks.size(); iii++) {
		sba.tracks.at(iii).point.x() -= centroid.x;
		sba.tracks.at(iii).point.y() -= centroid.y;
		sba.tracks.at(iii).point.z() -= centroid.z;
	}
	
	for (unsigned int iii = 0; iii < sba.nodes.size(); iii++) {
		sba.nodes.at(iii).trans.x() -= centroid.x;
		sba.nodes.at(iii).trans.y() -= centroid.y;
		sba.nodes.at(iii).trans.z() -= centroid.z;
	}
	
	
	
	// Re-size to a good scale
	for (unsigned int iii = 0; iii < sba.tracks.size(); iii++) {
		sba.tracks.at(iii).point.x() /= (largestAxis / 10.0);
		sba.tracks.at(iii).point.y() /= (largestAxis / 10.0);
		sba.tracks.at(iii).point.z() /= (largestAxis / 10.0);
	}
	
	for (unsigned int iii = 0; iii < sba.nodes.size(); iii++) {
		sba.nodes.at(iii).trans.x() /= (largestAxis / 10.0);
		sba.nodes.at(iii).trans.y() /= (largestAxis / 10.0);
		sba.nodes.at(iii).trans.z() /= (largestAxis / 10.0);
	}
	
	// Shift up to mostly sit on top of plane
	for (unsigned int iii = 0; iii < sba.tracks.size(); iii++) {
		sba.tracks.at(iii).point.y() += lowestHeight;
	}
	
	for (unsigned int iii = 0; iii < sba.nodes.size(); iii++) {
		sba.nodes.at(iii).trans.y() += lowestHeight;
	}
	
	// Move everything to desired centroid
	for (unsigned int iii = 0; iii < sba.tracks.size(); iii++) {
		sba.tracks.at(iii).point.x() += desiredCenter.x;
		sba.tracks.at(iii).point.y() += desiredCenter.y;
		sba.tracks.at(iii).point.z() += desiredCenter.z;
	}
	
	for (unsigned int iii = 0; iii < sba.nodes.size(); iii++) {
		sba.nodes.at(iii).trans.x() += desiredCenter.x;
		sba.nodes.at(iii).trans.y() += desiredCenter.y;
		sba.nodes.at(iii).trans.z() += desiredCenter.z;
	}
	
	
}

void findCentroid(SysSBA& sba, cv::Point3d& centroid, cv::Point3d& stdDeviation) {
	
	printf("%s << sba.tracks.size() = %d\n", __FUNCTION__, int(sba.tracks.size()));
	
	centroid = cv::Point3d(0.0, 0.0, 0.0);
	stdDeviation = cv::Point3d(0.0, 0.0, 0.0);
	
	for (unsigned int iii = 0; iii < sba.tracks.size(); iii++) {
		centroid.x += (sba.tracks.at(iii).point.x() / ((double) sba.tracks.size()));
		centroid.y += (sba.tracks.at(iii).point.y() / ((double) sba.tracks.size()));
		centroid.z += (sba.tracks.at(iii).point.z() / ((double) sba.tracks.size()));
	}
	
	for (unsigned int iii = 0; iii < sba.tracks.size(); iii++) {
		stdDeviation.x += (pow((sba.tracks.at(iii).point.x() - centroid.x), 2.0) / ((double) sba.tracks.size()));
		stdDeviation.y += (pow((sba.tracks.at(iii).point.y() - centroid.y), 2.0) / ((double) sba.tracks.size()));
		stdDeviation.z += (pow((sba.tracks.at(iii).point.z() - centroid.z), 2.0) / ((double) sba.tracks.size()));
	}
	
	stdDeviation.x = pow(stdDeviation.x, 0.5);
	stdDeviation.y = pow(stdDeviation.y, 0.5);
	stdDeviation.z = pow(stdDeviation.z, 0.5);
	
}

void optimizeFullSystem(vector<featureTrack>& tracks, cameraParameters& camData, cv::Mat *cameras, unsigned int last_index) {
	/*
	SysSBA fullsys;
	
	vector<unsigned int> camera_indices;
	for (unsigned int iii = 0; iii <= last_index; iii++) {
		if (cameras[iii].rows == 4) {
			camera_indices.push_back(iii);
			
			if (fullsys.nodes.size() == 0) {
				addFixedCamera(fullsys, camData, cameras[iii]);
			} else {
				addNewCamera(fullsys, camData, cameras[iii]);
			}
		}
	}
	
	for (unsigned int iii = 0; iii < tracks.size(); iii++) {
		
		if (tracks.at(iii).isTriangulated) {
			
			for (unsigned int jjj = 0; jjj < tracks.at(iii).locations.size(); jjj++) {
				
				unsigned int cam_idx = tracks.at(iii).locations.at(jjj).imageIndex;
				
				for (unsigned int kkk = 0; kkk < 
				
			}
			
			
		}
		
	}
	
	*/
}

double keyframeBundleAdjustment(cameraParameters& camData, vector<featureTrack>& tracks, cv::Mat *cameras, vector<unsigned int>& indices, unsigned int iterations, bool allFree, bool allFixedExceptLast, unsigned int fixed_cams) {
	
	//printf("%s << Entered kBA (1)...\n", __FUNCTION__);
	
	if (indices.size() == 0) {
		return -1.00;
	}
	
	//printf("%s << indices.size() = %d\n", __FUNCTION__, indices.size());
	
	vector<unsigned int> activeTrackIndices;
	
	SysSBA sys;
	sys.verbose = 0;
	
	unsigned int maxFixedIndex = std::max(1, ((int)indices.size()) - 1);
	
	//printf("%s << DEBUG (%02d)\n", __FUNCTION__, 0);
		
	for (unsigned int iii = 0; iii < indices.size(); iii++) {
		
		unsigned int img_idx = indices.at(iii);
		
		//printf("%s << indices.at(iii) = %d\n", __FUNCTION__, img_idx);
		//cout << cameras[img_idx] << endl;
		
		if (iii < maxFixedIndex) {
			addNewCamera(sys, camData, cameras[img_idx]);
		} else {
			addNewCamera(sys, camData, cameras[img_idx]);
		}

	}
	
	//printf("%s << DEBUG (%02d)\n", __FUNCTION__, 1);

	if (allFree && allFixedExceptLast) {
		// Bug to fix ALL points
		sys.nFixed = sys.nodes.size();
	} else if (allFree) {
		sys.nFixed = 1;
	} else if (allFixedExceptLast) {
		sys.nFixed = sys.nodes.size()-1;
	} else if (fixed_cams != 0) {
		sys.nFixed = fixed_cams;
	} else {
		sys.nFixed = maxFixedIndex;
	}
	
	//printf("%s << DEBUG (%02d)\n", __FUNCTION__, 2);
	
	//printf("%s << sys.nodes.size() = %d\n", __FUNCTION__, sys.nodes.size());
	
	for (unsigned int iii = 0; iii < indices.size()-1; iii++) {
		
		//printf("%s << Defining kf indices...\n", __FUNCTION__);
		unsigned int kf_ind_1 = indices.at(iii);
		unsigned int kf_ind_2 = indices.at(iii+1);
		
		vector<unsigned int> tmp_indices, untriangulated;
		
		//printf("%s << About to get active tracks...\n", __FUNCTION__);
		getActiveTracks(tmp_indices, tracks, kf_ind_1, kf_ind_2);
		
		//printf("%s << Reducing active to triangulated...\n", __FUNCTION__);
		reduceActiveToTriangulated(tracks, tmp_indices, untriangulated	);
		
		//printf("%s << Adding unique to vector...\n", __FUNCTION__);
		addUniqueToVector(activeTrackIndices, tmp_indices);
	}
	
	//printf("%s << DEBUG (%02d)\n", __FUNCTION__, 3);
	
	//printf("%s << activeTrackIndices.size() = %d\n", __FUNCTION__, activeTrackIndices.size());
	
	vector<cv::Point3d> cloud;
	getActive3dPoints(tracks, activeTrackIndices, cloud);
	
	//printf("%s << cloud.size() = %d\n", __FUNCTION__, cloud.size());

	addPointsToSBA(sys, cloud);
	
	//printf("%s << DEBUG (%02d)\n", __FUNCTION__, 4);
	
	for (unsigned int iii = 0; iii < activeTrackIndices.size(); iii++) {
		// For each active track, see if it has a 2D location in each image
		
		for (unsigned int jjj = 0; jjj < indices.size(); jjj++) {
			
			// what is the image index that you're after?
			unsigned int cam_idx_1 = indices.at(jjj);
			
			//printf("%s << Searching for projection in (%d)\n", __FUNCTION__, cam_idx_1);
			
			for (unsigned int kkk = 0; kkk < tracks.at(activeTrackIndices.at(iii)).locations.size(); kkk++) {
				
				if (tracks.at(activeTrackIndices.at(iii)).locations.at(kkk).imageIndex == ((int) cam_idx_1)) {
					// Found a projection of track onto this camera
					
					if (iii == 0) {
							//printf("%s << Adding projection (%f, %f) of track (%d) to camera (%d)...\n", __FUNCTION__, tracks.at(activeTrackIndices.at(iii)).locations.at(kkk).featureCoord.x, tracks.at(activeTrackIndices.at(iii)).locations.at(kkk).featureCoord.y, iii, kf_store.keyframes.at(validKeyframes.at(jjj)).idx); 
					}
					
					
					addProjectionToSBA(sys, tracks.at(activeTrackIndices.at(iii)).locations.at(kkk).featureCoord, iii, jjj);
					 
					 
				} 
			}
			
			
			
		}

		
	}
	
	//printf("%s << DEBUG (%02d)\n", __FUNCTION__, 5);
	
	double avgError = optimizeSystem(sys, 1e-4, iterations);
	
	//rescaleSBA(sys, 0, sys.nodes.size()-1);
	
	//printf("%s << avgError = %f\n", __FUNCTION__, avgError);

	//printf("%s << DEBUG (%02d)\n", __FUNCTION__, 51);
	
	for (unsigned int iii = 0; iii < indices.size(); iii++) {
		
		unsigned int img_idx = indices.at(iii);
		
		retrieveCameraPose(sys, iii, cameras[img_idx]);
	}
	
	//printf("%s << DEBUG (%02d)\n", __FUNCTION__, 6);
	
	for (unsigned int iii = 0; iii < sys.tracks.size(); iii++) {
		
		cv::Point3d new3dLoc(sys.tracks.at(iii).point.x(), sys.tracks.at(iii).point.y(), sys.tracks.at(iii).point.z());
		
		//if (validKeyframes.size() == 2) {
			// Hack to get it to reset locations on first go
			tracks.at(activeTrackIndices.at(iii)).reset3dLoc(new3dLoc);
		//} else {
		//	tracks.at(activeTrackIndices.at(iii)).set3dLoc(new3dLoc);
		//}
		
	}
	
	//printf("%s << DEBUG (%02d)\n", __FUNCTION__, 7);
	
	return avgError;
}


double keyframeBundleAdjustment(cameraParameters& camData, vector<featureTrack>& tracks, keyframeStore& kf_store, cv::Mat *cameras, unsigned int kfIndex, unsigned int iterations) {

	//printf("%s << Entered kBA (2)...\n", __FUNCTION__);

	//unsigned int optIndex = kf_store.keyframes.at(kf_idx).idx;	// index of frame to be adjusted
	
	unsigned int kf_idx = kfIndex;
	
	vector<unsigned int> validConnections, validKeyframes;
	// Find all connected keyframes, and establish initial camera poses
	//kf_store.findStrongConnections(kf_idx, validConnections, kf_idx);
	
	for (unsigned int iii = 0; iii < kf_idx; iii++) {
		validConnections.push_back(iii);
	}
	
	//printf("%s << validConnections.size() = %d\n", __FUNCTION__, validConnections.size());
	
	for (unsigned int iii = 0; iii < validConnections.size(); iii++) {
		
		//printf("%s << validConnections.at(%d) = (%d) (%d)\n", __FUNCTION__, iii, kf_store.connections.at(validConnections.at(iii)).idx1, kf_store.connections.at(validConnections.at(iii)).idx2);
		
		bool kf1added = false, kf2added = false;
		
		for (unsigned int jjj = 0; jjj < validKeyframes.size(); jjj++) {
			
			if (kf_store.connections.at(validConnections.at(iii)).idx1 == validKeyframes.at(jjj)) {
				kf1added = true;
			} 
			
			if (kf_store.connections.at(validConnections.at(iii)).idx2 == validKeyframes.at(jjj)) {
				kf2added = true;
			}
			
		}
		
		if (!kf1added) {
			validKeyframes.push_back(kf_store.connections.at(validConnections.at(iii)).idx1);
		}
		
		if (!kf2added) {
			validKeyframes.push_back(kf_store.connections.at(validConnections.at(iii)).idx2);
		}
		
		
	}
	
	//printf("%s << Valid keyframes size: %d\n", __FUNCTION__, validKeyframes.size());
	
	
	// For each connection, find any tracks that have been triangulated
	
	vector<unsigned int> activeTrackIndices;
	
	SysSBA sys;
	sys.verbose = 0;
	
	//printf("%s << Before 2-frame BA: \n", __FUNCTION__);
	
	for (unsigned int iii = 0; iii < validKeyframes.size(); iii++) {
		
		unsigned int img_idx = kf_store.keyframes.at(validKeyframes.at(iii)).idx;
		
		//Mat C_mat;
		//printf("%s << validKeyframes.at(%d) = %d\n", __FUNCTION__, iii, validKeyframes.at(iii));
		//(kf_store.keyframes.at(validKeyframes.at(iii)).pose).copyTo(C_mat);

		
		if (iii == validKeyframes.size()-1) {
			addNewCamera(sys, camData, cameras[img_idx]);
		} else {
			addFixedCamera(sys, camData, cameras[img_idx]);	// addFixedCamera(sys, camData, C_mat);
			sys.nodes.at(iii).isFixed = true;
			//addNewCamera(sys, camData, cameras[img_idx]);
			
			//printf("%s << sys.nFixed = %d\n", __FUNCTION__, sys.nFixed);
		}
		
		
		
		if (int(iii) >= max(((int) 0), ((int) validKeyframes.size())-4)) {
			//cout << __FUNCTION__ << " << C(" << img_idx << ") = " << endl << cameras[img_idx] << endl;
		}
		
		
	}	
	
	int minUnfixed = 5;
	
	sys.nFixed = max(1, ((int) validKeyframes.size())-minUnfixed);
	
	
	for (unsigned int iii = 0; iii < validConnections.size(); iii++) {
		
		unsigned int kf_ind_1 = kf_store.connections.at(validConnections.at(iii)).idx1;
		unsigned int kf_ind_2 = kf_store.connections.at(validConnections.at(iii)).idx2;
		
		vector<unsigned int> tmp_indices;
		
		//printf("%s << indexes = (%d, %d)\n", __FUNCTION__, kf_store.keyframes.at(kf_ind_1).idx, kf_store.keyframes.at(kf_ind_2).idx);
		
		getActiveTracks(tmp_indices, tracks, kf_store.keyframes.at(kf_ind_1).idx, kf_store.keyframes.at(kf_ind_2).idx);
		
		//printf("%s << active tracks for (%d, %d) = %d\n", __FUNCTION__, kf_store.keyframes.at(kf_ind_1).idx, kf_store.keyframes.at(kf_ind_2).idx, tmp_indices.size());
		
		vector<unsigned int> untriangulated;
		reduceActiveToTriangulated(tracks, tmp_indices, untriangulated);
		
		//printf("%s << after triangulation (%d, %d) = %d\n", __FUNCTION__, kf_store.keyframes.at(kf_ind_1).idx, kf_store.keyframes.at(kf_ind_2).idx, tmp_indices.size());
		// 193 have been triangulated...
		
		//vector<Point3d> cloud;
		//getActive3dPoints(tracks, tmp_indices, cloud);
		

		
		//addPointsToSBA(subsys, ptsInCloud);
		//addProjectionsToSBA(subsys, pts1, 0);
		
		addUniqueToVector(activeTrackIndices, tmp_indices);
	}
	
	//printf("%s << activeTrackIndices.size() = %d\n", __FUNCTION__, activeTrackIndices.size());
			
	//reduceActiveToTriangulated(tracks, activeTrackIndices);
	
	//printf("%s << triangulatedIndices.size() = %d\n", __FUNCTION__, activeTrackIndices.size());
	
	// Get projections and 3D locations for these tracks relative to the relevant cameras
	
	vector<cv::Point3d> cloud;
	getActive3dPoints(tracks, activeTrackIndices, cloud);
	
	//printf("%s << cloud.size() = %d (tracks.size() = %d; activeTrackIndices.size() = %d)\n", __FUNCTION__, cloud.size(), tracks.size(), activeTrackIndices.size());
	
	addPointsToSBA(sys, cloud);
	//drawGraph(sys, camera_pub, points_pub);
	
	for (unsigned int iii = 0; iii < activeTrackIndices.size(); iii++) {
		// For each active track, see if it has a 2D location in each image
		
		for (unsigned int jjj = 0; jjj < validKeyframes.size(); jjj++) {
			
			// what is the image index that you're after?
			unsigned int cam_idx_1 = kf_store.keyframes.at(validKeyframes.at(jjj)).idx;
			
			//printf("%s << Searching for projection in (%d)\n", __FUNCTION__, cam_idx_1);
			
			for (unsigned int kkk = 0; kkk < tracks.at(activeTrackIndices.at(iii)).locations.size(); kkk++) {
				
				if (tracks.at(activeTrackIndices.at(iii)).locations.at(kkk).imageIndex == ((int) cam_idx_1)) {
					// Found a projection of track onto this camera
					
					if (iii == 0) {
							//printf("%s << Adding projection (%f, %f) of track (%d) to camera (%d)...\n", __FUNCTION__, tracks.at(activeTrackIndices.at(iii)).locations.at(kkk).featureCoord.x, tracks.at(activeTrackIndices.at(iii)).locations.at(kkk).featureCoord.y, iii, kf_store.keyframes.at(validKeyframes.at(jjj)).idx); 
					}
					
					
					addProjectionToSBA(sys, tracks.at(activeTrackIndices.at(iii)).locations.at(kkk).featureCoord, iii, jjj);
					 
					 
				} 
			}
			
			
			
		}

		
	}
	
	//sys.setupSys(1.0e-4);
	
	// Bundle adjust this system, with only the "kf_index" camera unfixed
			// what about the 3D points? might it change some of their positions?
					// perhaps this will be handled by the overall BA....
					
	//printf("%s << Cameras (nodes): %d, Points: %d [iters = %d]\n", __FUNCTION__, (int)sys.nodes.size(), (int)sys.tracks.size(), iterations);
	
	double avgError = optimizeSystem(sys, 1e-4, iterations);
	//printf("%s << avgError = %f.\n", __FUNCTION__, avgError);
	
	//printf("%s << () sys.nFixed = %d\n", __FUNCTION__, sys.nFixed);
	
	// Then need to update locations of 3D points and location of camera
	
	//unsigned int img_idx = kf_store.keyframes.at(validKeyframes.size()-1).idx;
	
	//retrieveCamera(cameras[img_idx], sys, validKeyframes.size()-1);
	
	
	
	
	//retrieveCamera(kf_store.keyframes.at(validKeyframes.size()-1).pose, sys, validKeyframes.size()-1);
	
	//printf("%s << After 2-frame BA: \n", __FUNCTION__);
	//cout << P0 << endl;
	//cout << kf_store.keyframes.at(validKeyframes.size()-1).pose << endl;
	
	for (unsigned int iii = 0; iii < validKeyframes.size(); iii++) {
		
		unsigned int img_idx = kf_store.keyframes.at(iii).idx;
		
		retrieveCameraPose(sys, iii, cameras[img_idx]);
		
		if (int(iii) >= max(((int) 0), ((int) validKeyframes.size())-4)) {
			//cout << __FUNCTION__ << " << C(" << img_idx << ") = " << endl << cameras[img_idx] << endl;
		}
		
		
		//cout << "C = " << endl << cameras[kf_store.keyframes.at(validKeyframes.at(iii)).idx] << endl;
	}
	
	
	for (unsigned int iii = 0; iii < sys.tracks.size(); iii++) {
		
		cv::Point3d new3dLoc(sys.tracks.at(iii).point.x(), sys.tracks.at(iii).point.y(), sys.tracks.at(iii).point.z());
		
		if (validKeyframes.size() == 2) {
			// Hack to get it to reset locations on first go
			tracks.at(activeTrackIndices.at(iii)).reset3dLoc(new3dLoc);
		} else {
			tracks.at(activeTrackIndices.at(iii)).set3dLoc(new3dLoc);
		}
		
	}
	
	return avgError;
}

double distanceBetweenPoints(Eigen::Matrix<double, 4, 1>& pt1, Eigen::Matrix<double, 4, 1>& pt2) {
	
	double dist = 0.0;
	
	dist += pow(pt1.x() - pt2.x(), 2.0);
	dist += pow(pt1.y() - pt2.y(), 2.0);
	dist += pow(pt1.z() - pt2.z(), 2.0);
	
	dist = pow(dist, 0.5);
	
	return dist;
	
}

double determineSystemSize(SysSBA& sys) {
	
	double maxDist = 0.0;
	
	for (unsigned int iii = 0; iii < sys.nodes.size()-1; iii++) {
		for (unsigned int jjj = iii+1; jjj < sys.nodes.size(); jjj++) {
			double dist = distanceBetweenPoints(sys.nodes.at(iii).trans, sys.nodes.at(jjj).trans);
			maxDist = max(maxDist, dist);
		}
	}
	
	return maxDist;
}

void normalizeSystem(SysSBA& sys, double factor) {
	
	for (unsigned int iii = 0; iii < sys.nodes.size(); iii++) {
		sys.nodes.at(iii).trans.x() *= factor;
		sys.nodes.at(iii).trans.y() *= factor;
		sys.nodes.at(iii).trans.z() *= factor;
	}
	
	for (unsigned int iii = 0; iii < sys.tracks.size(); iii++) {
		sys.tracks.at(iii).point.x() *= factor;
		sys.tracks.at(iii).point.y() *= factor;
		sys.tracks.at(iii).point.z() *= factor;
	}
}

double predictiveBundleAdjustment(cameraParameters& camData, vector<featureTrack>& tracks, geometry_msgs::PoseStamped *keyframePoses, bool *keyframeTypes, unsigned int keyframeCount, geometry_msgs::PoseStamped& newPose, unsigned int iterations, bool debug, int mode, double err, int* triangulations, double *averagePointShift) {
	
	double avgError;
	vector<unsigned int> activeTrackIndices;
	
	SysSBA sys;
	debug ? sys.verbose = 1 : sys.verbose = 0;
	sys.nFixed = 0;
	
	for (unsigned int iii = 0; iii < keyframeCount; iii++) {
		
		cv::Mat C, R, t;
		Eigen::Quaternion<double> Q;
		t = cv::Mat::zeros(3, 1, CV_64FC1);
		
		convertPoseFormat(keyframePoses[iii].pose, t, Q);
		quaternionToMatrix(Q, R);
		composeTransform(R, t, C);
				
		addFixedCamera(sys, camData, C);
		
		// was if (1) {
		if (keyframeTypes[iii]) { //(iii == (keyframeCount-1)) { // (iii == 0) { // 
			sys.nodes.at(iii).isFixed = true;
			sys.nFixed++;
		} else { sys.nodes.at(iii).isFixed = false; }
	}
	
	if (sys.nFixed == 0) {
		// To deal with the special case of ALL keyframes being video-based...
		//printf("%s << DEALING WITH SPECIAL CASE!\n", __FUNCTION__);
		sys.nodes.at(0).isFixed = true;
		sys.nFixed++;
	}
	
	cv::Mat C, R, t;
	
	Eigen::Quaternion<double> Q;
	t = cv::Mat::zeros(3, 1, CV_64FC1);
	
	convertPoseFormat(newPose.pose, t, Q);
	quaternionToMatrix(Q, R);
	composeTransform(R, t, C);
	addFixedCamera(sys, camData, C);
	sys.nodes.at(sys.nodes.size()-1).isFixed = false;
	
	
	if (0) { printf("%s << Cameras added = (%d) + 1\n", __FUNCTION__, (keyframeCount-1)); }
	

	unsigned int tracksAdded = 0, projectionsAdded = 0;
	
	vector<int> track_indices;
	
	if (0) { printf("%s << tracks provided = (%d).\n", __FUNCTION__, ((int)tracks.size())); }
	
	int triCount = 0;
	
	for (unsigned int iii = 0; iii < tracks.size(); iii++) {
		
		
		
		if (tracks.at(iii).isTriangulated) {
			
			unsigned int projectionCount = 0;
			
			vector<int> indices, camera_indices; 
			
			if (0) {printf("%s << tracks.at(%d).locations.size() = (%d)\n", __FUNCTION__, iii, ((int)tracks.at(iii).locations.size())); }
			
			for (unsigned int jjj = 0; jjj < tracks.at(iii).locations.size(); jjj++) {
				
				if (0) {printf("%s << [%d] [%d]\n", __FUNCTION__, iii, jjj); }
				if (0) {printf("%s << (%d)\n", __FUNCTION__, tracks.at(iii).locations.at(jjj).imageIndex); }
				
				bool indexAdded = false;
				
				for (unsigned int kkk = 0; kkk < keyframeCount; kkk++) {
					
					if (int(tracks.at(iii).locations.at(jjj).imageIndex) == int(keyframePoses[kkk].header.seq)) {
						projectionCount++;
						if (0) {printf("%s << A Adding index (%d)\n", __FUNCTION__, jjj); }
						indices.push_back(jjj);
						indexAdded = true;
						camera_indices.push_back(kkk);
						break;
					}
					
				}
				
				if (0) {printf("%s << Searching for presence in flexible camera... (%d, %d)\n", __FUNCTION__, iii, jjj); }
				if (0) {printf("%s << (%d)\n", __FUNCTION__, newPose.header.seq); }
				
				if (!indexAdded && (int(tracks.at(iii).locations.at(jjj).imageIndex) == int(newPose.header.seq))) {
					projectionCount++;
					if (0) {printf("%s << B Adding index (%d)\n", __FUNCTION__, jjj); }
					indices.push_back(jjj);
					camera_indices.push_back(sys.nodes.size()-1);
				}
				
				
			}
			
			if (0) {printf("%s << [%d] Out of loop\n", __FUNCTION__, iii); }
			
			//printf("%s << projectionCount = (%d)\n", __FUNCTION__, projectionCount);
			
			if (projectionCount >= 2) {
				// add point
				//Vector4d temppoint(tracks.at(iii).get3dLoc().x, tracks.at(iii).get3dLoc().y, tracks.at(iii).get3dLoc().z, 1.0);
				Vector4d temppoint(tracks.at(iii).get3dLoc().x, tracks.at(iii).get3dLoc().y, tracks.at(iii).get3dLoc().z, 1.0);
				
				//printf("%s << Adding point (%d) (%f, %f, %f)\n", __FUNCTION__, tracksAdded, temppoint.x(), temppoint.y(), temppoint.z());
				sys.addPoint(temppoint);
				track_indices.push_back(iii);
				
				if (0) {printf("%s << [%d] Adding projections\n", __FUNCTION__, iii); }
				
				// add projections
				for (unsigned int jjj = 0; jjj < indices.size(); jjj++) {
					
					if (0) {printf("%s << Adding projection for index (%d)(%d)... camera_indices.size() = [%d] {%d}\n", __FUNCTION__, jjj, indices.at(jjj), ((int)camera_indices.size()), camera_indices.at(jjj)); }
					
					//addProjectionToSBA(sys, tracks.at(iii).locations.at(jjj).featureCoord, tracks.at(iii).locations.at(jjj)., unsigned int camNo) 
					
					if (0) {printf("%s << iii = (%d)\n", __FUNCTION__, iii); }
					if (0) {printf("%s << tracks.size() = (%d)\n", __FUNCTION__, ((int)tracks.size())); }
					if (0) {printf("%s << jjj = (%d)\n", __FUNCTION__, jjj); }
					if (0) {printf("%s << X tracks.at(%d).locations.size() = (%d); (%d)\n", __FUNCTION__, iii, ((int)tracks.at(iii).locations.size()), indices.at(jjj)); }
					
					// jjj value was too large to access that "location"
					Vector2d temp2d(tracks.at(iii).locations.at(indices.at(jjj)).featureCoord.x, tracks.at(iii).locations.at(indices.at(jjj)).featureCoord.y);
					if (0) {printf("%s << sys.nodes.size() = (%d)\n", __FUNCTION__, ((int)sys.nodes.size())); }
					sys.addMonoProj(camera_indices.at(jjj), tracksAdded, temp2d);
					//printf("%s << Adding projection (%f, %f) for track (%d) and camera (%d)\n", __FUNCTION__, temp2d.x(), temp2d.y(), tracksAdded, camera_indices.at(jjj));
					projectionsAdded++;
					
				}
				
				if (0) {printf("%s << [%d] Projections added.\n", __FUNCTION__, iii); }
				
				tracksAdded++;
				
			}
			
			triCount++;
		}
		
		//
		
	}
	
	if (0) { printf("%s << triangulated count = (%d).\n", __FUNCTION__, triCount); }
	
	if ((tracksAdded < 8) || (projectionsAdded < 8)) {
		if (debug) { printf("%s << ERROR! Insufficient tracks (%d) or projections (%d) added for bundle adjustment.\n", __FUNCTION__, ((int)sys.tracks.size()), sys.countProjs()); }
		return -1.0;
	}
	
	
	if (debug) {
		printf("%s << Total tracks/cams/projections = (%d, %d, %d)\n", __FUNCTION__, ((int)sys.tracks.size()), ((int)sys.nodes.size()), sys.countProjs());
		sys.printStats();
	}
	
	// BUNDLE ADJUST!!
	//sys.setupSys(1.0e-4);
	
	//printf("%s << About to bundle adjust...\n", __FUNCTION__);
	
	double system_size = determineSystemSize(sys);
	
	// printf("%s << system_size = (%f)\n", __FUNCTION__, system_size);
	
	normalizeSystem(sys, 3.0/system_size);
	
	if (debug) { printf("%s << Number of points in system = (%d)\n", __FUNCTION__, sys.tracks.size()); }
	
	avgError = optimizeSystem(sys, err, iterations, debug, mode);
	
	normalizeSystem(sys, system_size/3.0);
	
	/*
	for (unsigned int iii = 0; iii < keyframeCount; iii++) {
		retrieveCameraPose(sys, iii, keyframePoses[iii].pose);
	}
	*/
	geometry_msgs::Pose tmpPose = newPose.pose;
	double cameraShift = retrieveCameraPose(sys, keyframeCount, tmpPose);
	
	if (debug) { printf("%s << cameraShift = (%f)\n", __FUNCTION__, cameraShift); }
	
	if (debug) { printf("%s << avgError = (%f)\n", __FUNCTION__, avgError); }
	
	//if (!overwritePoints(sys, track_indices, tracks, DEFAULT_MAX_DISPLACEMENT, debug)) return -1.0;
	
	bool ptsOverwritten = overwritePoints(sys, track_indices, tracks, DEFAULT_MAX_DISPLACEMENT, averagePointShift, debug); /*DEFAULT_MAX_DISPLACEMENT*/
	
	if (debug) {
		if (ptsOverwritten) {
			printf("%s << Some points were overwritten...\n", __FUNCTION__);
		} else {
			printf("%s << No points were overwritten...\n", __FUNCTION__);
		}
	}
	
	*triangulations = projectionsAdded;
	
	if (cameraShift == 0.0) {
		return -1.0;
	} else {
		newPose.pose = tmpPose;
		return avgError;
	}
	
	//printf("%s << Adjustment error = (%f), (%d) out of (%d) pts changed\n", __FUNCTION__, avgError, changedCount, sys.tracks.size());
	
}

double odometryBundleAdjustment(cameraParameters& camData, vector<featureTrack>& tracks, geometry_msgs::PoseStamped *keyframePoses, unsigned int keyframeCount, unsigned int iterations, bool debug) {
	
	double avgError;
	vector<unsigned int> activeTrackIndices;
	
	SysSBA sys;
	
	debug ? sys.verbose = 1 : sys.verbose = 0;
	
	sys.nFixed = 0;
	
	for (unsigned int iii = 0; iii < keyframeCount; iii++) {
		
		if (0) { printf("%s << keyframePoses[%d].header.seq = (%d)\n", __FUNCTION__, iii, keyframePoses[iii].header.seq); }
		
		cv::Mat C, R, t;
		Eigen::Quaternion<double> Q;
		t = cv::Mat::zeros(3, 1, CV_64FC1);
		convertPoseFormat(keyframePoses[iii].pose, t, Q);
		quaternionToMatrix(Q, R);
		composeTransform(R, t, C);
		addFixedCamera(sys, camData, C);
		
		if ( (iii == 0) || (iii == (keyframeCount-1)) ) {
			sys.nodes.at(iii).isFixed = true;
			sys.nFixed++;
		} else { sys.nodes.at(iii).isFixed = false; }
	}	
	
	unsigned int tracksAdded = 0, projectionsAdded = 0;
	vector<int> track_indices;
	
	for (unsigned int iii = 0; iii < tracks.size(); iii++) {
		if (tracks.at(iii).isTriangulated) {
			
			unsigned int projectionCount = 0;
			vector<int> indices, camera_indices; 
			
			// Determine all cameras that have this feature in them..
			for (unsigned int jjj = 0; jjj < tracks.at(iii).locations.size(); jjj++) {
				
				if (0) { printf("%s << tracks.at(%d).locations.at(%d).imageIndex = (%d)\n", __FUNCTION__, iii, jjj, tracks.at(iii).locations.at(jjj).imageIndex); }
				
				for (unsigned int kkk = 0; kkk < keyframeCount; kkk++) {
					
					if (int(tracks.at(iii).locations.at(jjj).imageIndex) == int(keyframePoses[kkk].header.seq)) {
						projectionCount++;
						indices.push_back(jjj);
						camera_indices.push_back(kkk);
						if (0) { printf("%s << found in keyframePoses[%d].header.seq = (%d)\n", __FUNCTION__, kkk, keyframePoses[kkk].header.seq); }
						break;
					}
					
				}
				
			}
			
			if (projectionCount > 2) {

				Vector4d temppoint = Vector4d(tracks.at(iii).get3dLoc().x, tracks.at(iii).get3dLoc().y, tracks.at(iii).get3dLoc().z, 1.0);

				sys.addPoint(temppoint);
				track_indices.push_back(iii);
				
				for (unsigned int jjj = 0; jjj < indices.size(); jjj++) {
					Vector2d temp2d(tracks.at(iii).locations.at(indices.at(jjj)).featureCoord.x, tracks.at(iii).locations.at(indices.at(jjj)).featureCoord.y);
					sys.addMonoProj(camera_indices.at(jjj), tracksAdded, temp2d);
					if (0) { printf("%s << Adding projection (%f, %f) for track (%d) and camera (%d)\n", __FUNCTION__, temp2d.x(), temp2d.y(), tracksAdded, camera_indices.at(jjj)); }
					projectionsAdded++;
				}
				
				tracksAdded++;
				
			}
			
			
		}
	}
	
	if ((tracksAdded < 8) || (projectionsAdded == 8)) {
		if (debug) { printf("%s << ERROR! Insufficient tracks (%d) or projections (%d) added for bundle adjustment.\n", __FUNCTION__, ((int)sys.tracks.size()), sys.countProjs()); }
		return -1.0;
	}
	
	if (debug) {
		sys.printStats();
	}
	
	avgError = optimizeSystem(sys, 1e-5, iterations, debug);
	
	if (0) { printf("%s << avgError = (%f)\n", __FUNCTION__, avgError); }
	
	double avePointShift;
	if (!overwritePoints(sys, track_indices, tracks, DEFAULT_MAX_DISPLACEMENT, &avePointShift, debug)) return -1.0;
	
	if (avgError < 1.0) {
		/*
		for (unsigned int iii = 0; iii < keyframeCount; iii++) {
			double cameraShift = retrieveCameraPose(sys, iii, keyframePoses[iii].pose);
			printf("%s << shifting camera (%d) by (%f)\n", __FUNCTION__, iii, cameraShift);
		}
		*/
	}
	
	return avgError;
	
}

bool overwritePoints(const SysSBA& sys, const vector<int>& track_indices, vector<featureTrack>& tracks, double maxDisplacement, double *averagePointShift, bool debug) {
	
	unsigned int changedCount = 0, couldveCount = 0;
	
	*averagePointShift = 0.0;
	
	if (debug) { printf("%s << Overwriting points with constraint of (%f)\n", __FUNCTION__, maxDisplacement); }
	
	for (unsigned int iii = 0; iii < sys.tracks.size(); iii++) {
		
		cv::Point3d old3dLoc = tracks.at(track_indices.at(iii)).get3dLoc();
		cv::Point3d new3dLoc(sys.tracks.at(iii).point.x(), sys.tracks.at(iii).point.y(), sys.tracks.at(iii).point.z());
		
		double dist = distBetweenPts(old3dLoc, new3dLoc);
		
		if ( (dist < maxDisplacement) && (dist > 0.0) ) {
			*averagePointShift += dist;
			changedCount++;			
			tracks.at(track_indices.at(iii)).reset3dLoc(new3dLoc);
		} else {
			// printf("%s << Point not overwritten because (%f) !=! (%f, %f)\n", __FUNCTION__, dist, 0.0, maxDisplacement);
		}
		
		if (dist > 0.0) {
			couldveCount++;
		}
		
	}
	
	if (changedCount == 0) {
		*averagePointShift = -1.0;
	} else {
		*averagePointShift /= double(changedCount);
	}
	

	
	if (debug) { printf("%s << Changed a total of (%u) out of (%u / %u) 3D point locations (%f)\n", __FUNCTION__, changedCount, couldveCount, sys.tracks.size(), *averagePointShift); }
	
	if (changedCount > 0) { return true; } else { return false; }
	
}

void retrievePartialSystem(SysSBA& sys, cv::Mat *C, vector<featureTrack>& tracks, vector<unsigned int>& indices) {

	for (unsigned int iii = 0; iii < indices.size(); iii++) {
		
		if (C[indices.at(iii)].rows == 4) {
			retrieveCameraPose(sys, iii, C[indices.at(iii)]);
		}
		
	}
	
	unsigned int trackIndex = 0;
	
	for (unsigned int iii = 0; iii < sys.tracks.size(); iii++) {
		
		while (!tracks.at(trackIndex).isTriangulated) {
			trackIndex++;
			
			if (trackIndex >= tracks.size()) {
				break;
			}
		}
		
		cv::Point3d new3dLoc(sys.tracks.at(iii).point.x(), sys.tracks.at(iii).point.y(), sys.tracks.at(iii).point.z());
		tracks.at(trackIndex).reset3dLoc(new3dLoc);
		
		trackIndex++;
		
		if (trackIndex >= tracks.size()) {
			break;
		}
		
	}
	
}

void getActiveCameras(cv::Mat *C, vector<unsigned int>& indices, unsigned int min_index, unsigned int max_index) {
	for (unsigned int iii = min_index; iii <= max_index; iii++) {
		if (C[iii].rows == 4) {
			indices.push_back(iii);
		}
	}
}

void getActiveTracks(vector<featureTrack>& tracks, vector<unsigned int>& cameras, vector<unsigned int>& indices) {
	
	printf("%s << Entered.\n", __FUNCTION__);
	
	for (unsigned int iii = 0; iii < tracks.size(); iii++) {
		
		bool isAdded = false;
		
		if (tracks.at(iii).isTriangulated) {
		
			for (unsigned int jjj = 0; jjj < cameras.size(); jjj++) {
				
				for (unsigned int kkk = 0; kkk < tracks.at(iii).locations.size(); kkk++) {
					
					printf("%s << about to access tracks(%d / %d)(%d / %d) & cameras(%d / %d)\n", __FUNCTION__, int(iii), int(tracks.size()), int(kkk), int(tracks.at(iii).locations.size()), int(jjj), int(cameras.size()));
					if (tracks.at(iii).locations.at(kkk).imageIndex == int(cameras.at(jjj))) {
						indices.push_back(iii);
						isAdded = true;
						break;
					}
					
				}
				
				if (isAdded) {
					break;
				}
				
			}
			
		}
		
	}
	
	printf("%s << Exiting.\n", __FUNCTION__);
	
}
 
void retrieveFullSystem(SysSBA& sys, cv::Mat *C, vector<featureTrack>& tracks, unsigned int start_cam, unsigned int final_cam) {
	
	//vector<unsigned int> camera_indices;
	
	
	for (unsigned int iii = 0; iii < sys.nodes.size(); iii++) {
		
		retrieveCameraPose(sys, iii, C[start_cam+iii]);
		
	}
	
	
	/*
	for (unsigned int jjj = start_cam; jjj <= final_cam; jjj++) {
		
		if (C[jjj].rows == 4) {
			retrieveCamera(C[camera_indices.size()], sys, jjj);
			
			camera_indices.push_back(jjj);
		}
	
	}
	*/
	
	// The way the tracks are being retrived is incorrect!!!
	for (unsigned int iii = 0; iii < sys.tracks.size(); iii++) {
		
		// Only want to retrieve relevant tracks!
		
		cv::Point3d new3dLoc(sys.tracks.at(iii).point.x(), sys.tracks.at(iii).point.y(), sys.tracks.at(iii).point.z());
		tracks.at(iii).reset3dLoc(new3dLoc);
	}
}

double optimizeSubsystem(cameraParameters& camData, cv::Mat *C, vector<unsigned int>& c_i, vector<featureTrack>& tracks, vector<unsigned int>& t_i, unsigned int iterations) {

	SysSBA sys;
	sys.verbose = 0;
	
	vector<cv::Point3d> cloud;
	
	getActive3dPoints(tracks, t_i, cloud);

	printf("%s << active points (%d) out of (%d) valid tracks..\n", __FUNCTION__, int(cloud.size()), int(t_i.size()));

	addPointsToSBA(sys, cloud);
	
	// Add first and last cameras as fixed
	printf("%s << Adding (%d) and (%d) as fixed cameras...\n", __FUNCTION__, c_i.at(0), c_i.at(c_i.size()-1));
	addFixedCamera(sys, camData, C[c_i.at(0)]);
	addFixedCamera(sys, camData, C[c_i.at(c_i.size()-1)]);
	
	for (unsigned int iii = 1; iii < c_i.size()-1; iii++) {
		addNewCamera(sys, camData, C[c_i.at(iii)]);
	}
	
	for (unsigned int iii = 0; iii < c_i.size(); iii++) {
		
		//printf("%s << (%d) : %d\n", __FUNCTION__, iii, c_i.at(iii));
		
		// the node that corresponds to the camera
		
		unsigned int sysIndex;
		
		if (iii == 0) {
			sysIndex = 0;
		} else if (iii == c_i.size()-1) {
			sysIndex = 1;
		} else {
			sysIndex = iii + 1;
		}


		
		//cout << C[c_i.at(iii)] << endl;	
		
		for (unsigned int jjj = 0; jjj < t_i.size(); jjj++) {
			
			for (unsigned int kkk = 0; kkk < tracks.at(t_i.at(jjj)).locations.size(); kkk++) {
				
				if (tracks.at(t_i.at(jjj)).locations.at(kkk).imageIndex == ((int) c_i.at(iii))) {
					
					Vector2d proj;
					
					proj.x() = tracks.at(t_i.at(jjj)).locations.at(kkk).featureCoord.x;
					proj.y() = tracks.at(t_i.at(jjj)).locations.at(kkk).featureCoord.y;
	
					sys.addMonoProj(sysIndex, jjj, proj);
				}
			}
			
		}
		
		
		
	}	
	
	sys.nFixed = 2;
	
	unsigned int numFixed = sys.nFixed;
	
	printf("%s << nodes = %d; tracks = %d (nFixed = %d)\n", __FUNCTION__, int(sys.nodes.size()), int(sys.tracks.size()), int(numFixed));
	
	double avgError = optimizeSystem(sys, 1e-4, iterations);
	
	printf("%s << avgError = %f\n", __FUNCTION__, avgError);
	
	printf("%s << Retrieving 0 -> %d; and 1 -> %d\n", __FUNCTION__, c_i.at(0), c_i.at(c_i.size()-1));
	
	retrieveCameraPose(sys, 0, C[c_i.at(0)]);
	retrieveCameraPose(sys, 1, C[c_i.at(c_i.size()-1)]);
	
	for (unsigned int iii = 1; iii < c_i.size()-1; iii++) {
		retrieveCameraPose(sys, iii+1, C[c_i.at(iii)]);
	}
	
	for (unsigned int iii = 0; iii < sys.tracks.size(); iii++) {
		
		cv::Point3d new3dLoc(sys.tracks.at(iii).point.x(), sys.tracks.at(iii).point.y(), sys.tracks.at(iii).point.z());
		
		tracks.at(t_i.at(iii)).set3dLoc(new3dLoc);
	}
	
	
	return avgError;
	
}

double optimizeKeyframePair(vector<featureTrack>& tracks, cameraParameters& camData, int idx1, int idx2, cv::Mat *cameras) {
	
	vector<cv::Point2f> pts1, pts2;
	getPointsFromTracks(tracks, pts1, pts2, idx1, idx2);
	
	vector<cv::Point3d> cloud;
	vector<cv::Point2f> corresp;
	
	for (unsigned int iii = 0; iii < 10; iii++) {
		//printf("%s << pt(%d) = (%f, %f) & (%f, %f)\n", __FUNCTION__, iii, pts1.at(iii).x, pts1.at(iii).y, pts2.at(iii).x, pts2.at(iii).y);
	}
	//if (debug) {
		printf("%s << Before 2-frame BA: \n", __FUNCTION__);
		cout << cameras[idx1] << endl;
		cout << cameras[idx2] << endl;
	//}
	
	TriangulatePoints(pts1, pts2, camData.K, camData.K.inv(), cameras[idx1], cameras[idx2], cloud, corresp);
	
	double twoErr = twoViewBundleAdjustment(camData, cameras[idx1], cameras[idx2], cloud, pts1, pts2, 1000);
	
	//if (debug) {
		printf("%s << After 2-frame BA: \n", __FUNCTION__);
		cout << cameras[idx1] << endl;
		cout << cameras[idx2] << endl;
	//}
	
	
	return twoErr;
	
}

void assignPartialSystem(SysSBA &sba, vector<featureTrack>& tracks, cameraParameters& camData, cv::Mat *cameras, vector<unsigned int>& indices, bool assignProjections) {
	
	sba.tracks.clear();
	sba.nodes.clear();
	
	vector<unsigned int> added_indices;
	
	for (unsigned int jjj = 0; jjj < indices.size(); jjj++) {
			
		if (cameras[indices.at(jjj)].rows == 4) {
			if (jjj == 0) {
				addFixedCamera(sba, camData, cameras[indices.at(jjj)]);
			} else {
				addNewCamera(sba, camData, cameras[indices.at(jjj)]);
			}
			added_indices.push_back(indices.at(jjj));
		}
	
	}
	
	unsigned int addedTracks = -1;
	
	for (unsigned int iii = 0; iii < tracks.size(); iii++) {
		
		if (tracks.at(iii).isTriangulated) {

			Vector4d temppoint(tracks.at(iii).get3dLoc().x, tracks.at(iii).get3dLoc().y, tracks.at(iii).get3dLoc().z, 1.0);
			sba.addPoint(temppoint);
			addedTracks++;	
			
			if (assignProjections) {
				
				for (unsigned int jjj = 0; jjj < tracks.at(iii).locations.size(); jjj++) {
					
					unsigned int cam_index = tracks.at(iii).locations.at(jjj).imageIndex;
					
					for (unsigned int kkk = 0; kkk < added_indices.size(); kkk++) {
						
						if (added_indices.at(kkk) == cam_index) {
							//printf("%s << adding projection (%f, %f) to (cam: %d, track: %d)\n", __FUNCTION__, tracks.at(iii).locations.at(jjj).featureCoord.x, tracks.at(iii).locations.at(jjj).featureCoord.y, kkk, iii);
							addProjectionToSBA(sba, tracks.at(iii).locations.at(jjj).featureCoord, addedTracks, kkk);
							
							break;
						}
						
					}
					
					
					
				}
							
			}		
		}
		
		
		
	}
	
	
	
}

double adjustFullSystem(vector<featureTrack>& tracks, cameraParameters& camData, cv::Mat *cameras, unsigned int min_index, unsigned int max_index, unsigned int its) {
	SysSBA fullsys;
	
	cv::Mat eye4 = cv::Mat::eye(4, 4, CV_64FC1);
	
	for (unsigned int iii = 0; iii < max_index; iii++) {
		if (cameras[iii].rows == 4) {
			//estimatePoseFromKnownPoints(cameras[iii], camData, tracks, iii, eye4);
		}
	}
	
	// Get active cameras
	vector<unsigned int> active_camera_indices;
	getActiveCameras(cameras, active_camera_indices, min_index, max_index);
	
	// Get active tracks
	vector<unsigned int> active_track_indices;
	getActiveTracks(tracks, active_camera_indices, active_track_indices);
	
	assignSystem(fullsys, tracks, camData, cameras, active_camera_indices, active_track_indices);
	
	double avgError;
	
	avgError = optimizeSystem(fullsys, 1e-4, its );
	//retrievePartialSystem(sys, ACM, featureTrackVector, keyframeIndices);
	
	retrieveSystem(fullsys, tracks, camData, cameras, active_camera_indices, active_track_indices);
	
	return avgError;
	
	
}

void retrieveSystem(SysSBA &sba, vector<featureTrack>& tracks, cameraParameters& camData, cv::Mat *cameras, vector<unsigned int>& camera_indices, vector<unsigned int>& track_indices) {
	
	for (unsigned int iii = 0; iii < camera_indices.size(); iii++) {
		retrieveCameraPose(sba, iii, cameras[camera_indices.at(iii)]);
	}
	
	for (unsigned int iii = 0; iii < track_indices.size(); iii++) {
		cv::Point3d new3dLoc(sba.tracks.at(iii).point.x(), sba.tracks.at(iii).point.y(), sba.tracks.at(iii).point.z());
		tracks.at(track_indices.at(iii)).reset3dLoc(new3dLoc);
	}
	
}

void assignSystem(SysSBA &sba, vector<featureTrack>& tracks, cameraParameters& camData, cv::Mat *cameras, vector<unsigned int>& camera_indices, vector<unsigned int>& track_indices) {
	
	sba.tracks.clear();
	sba.nodes.clear();
	
	printf("%s << DEBUG [%d]\n", __FUNCTION__, 0);
	
	for (unsigned int iii = 0; iii < camera_indices.size(); iii++) {
		if (sba.nodes.size() == 0) {
			addFixedCamera(sba, camData, cameras[camera_indices.at(iii)]);
		} else {
			addNewCamera(sba, camData, cameras[camera_indices.at(iii)]);
		}
	}
	
	printf("%s << DEBUG [%d]\n", __FUNCTION__, 1);
	
	for (unsigned int iii = 0; iii < track_indices.size(); iii++) {
		
		Vector4d temppoint(tracks.at(track_indices.at(iii)).get3dLoc().x, tracks.at(track_indices.at(iii)).get3dLoc().y, tracks.at(track_indices.at(iii)).get3dLoc().z, 1.0);
		sba.addPoint(temppoint);
		
		for (unsigned int jjj = 0; jjj < tracks.at(track_indices.at(iii)).locations.size(); jjj++) {
			
			for (unsigned int kkk = 0; kkk < camera_indices.size(); kkk++) {
				if (int(camera_indices.at(kkk)) == tracks.at(track_indices.at(iii)).locations.at(jjj).imageIndex) {
					
					Vector2d proj;
			
					proj.x() = tracks.at(track_indices.at(iii)).locations.at(jjj).featureCoord.x;
					proj.y() = tracks.at(track_indices.at(iii)).locations.at(jjj).featureCoord.y;
										
					//printf("%s << Adding mono projection: (cam: %d / %d, track: %d / %d) = (%f, %f)\n", __FUNCTION__, tracks.at(iii).locations.at(jjj).imageIndex, sys.nodes.size(), sys.tracks.size()-1, sys.tracks.size(), proj.x(), proj.y());
					sba.addMonoProj(kkk, iii, proj);
				}
			}
			
		}

		
	}
	
	printf("%s << DEBUG [%d]\n", __FUNCTION__, 2);
	
}

void assignFullSystem(SysSBA &sba, vector<featureTrack>& tracks, cameraParameters& camData, cv::Mat *cameras, unsigned int start_index, unsigned int finish_index, bool dummy) {
	
	sba.tracks.clear();
	sba.nodes.clear();
	
	vector<unsigned int> camera_indices;
	
	for (unsigned int jjj = start_index; jjj <= finish_index; jjj++) {
		
		if (cameras[jjj].rows == 4) {
			if (sba.nodes.size() == 0) {
				addFixedCamera(sba, camData, cameras[jjj]);
			} else {
				addNewCamera(sba, camData, cameras[jjj]);
			}
			
			camera_indices.push_back(jjj);
		}
	
	}
	
	for (unsigned int iii = 0; iii < tracks.size(); iii++) {
		
		if (tracks.at(iii).isTriangulated) {

			Vector4d temppoint(tracks.at(iii).get3dLoc().x, tracks.at(iii).get3dLoc().y, tracks.at(iii).get3dLoc().z, 1.0);
			sba.addPoint(temppoint);			
	
			for (unsigned int jjj = 0; jjj < tracks.at(iii).locations.size(); jjj++) {
				
				// Only want to add projections for active nodes
				
				if (!dummy) {
					if ((((unsigned int) tracks.at(iii).locations.at(jjj).imageIndex) >= start_index) && (((unsigned int) tracks.at(iii).locations.at(jjj).imageIndex) <= (finish_index))) {
						
						for (unsigned int kkk = 0; kkk < camera_indices.size(); kkk++) {
							
							if (int(camera_indices.at(kkk)) == tracks.at(iii).locations.at(jjj).imageIndex) {
								Vector2d proj;
						
								proj.x() = tracks.at(iii).locations.at(jjj).featureCoord.x;
								proj.y() = tracks.at(iii).locations.at(jjj).featureCoord.y;
													
								//printf("%s << Adding mono projection: (cam: %d / %d, track: %d / %d) = (%f, %f)\n", __FUNCTION__, tracks.at(iii).locations.at(jjj).imageIndex, sys.nodes.size(), sys.tracks.size()-1, sys.tracks.size(), proj.x(), proj.y());
								sba.addMonoProj(kkk, sba.tracks.size()-1, proj);
							}
							
						}
						
						//printf("%s << Assembling projection (%d)(%d)\n", __FUNCTION__, iii, jjj);
					
						
						
					}
				}
				
				
				
				
				
			}
		
		}
		
	}
	
	
	
}

bool reconstructFreshSubsequencePair(vector<featureTrack>& tracks, vector<cv::Point3d>& ptCloud, vector<unsigned int>& triangulatedIndices, cv::Mat& real_C0, cv::Mat& real_C1, cameraParameters camData, int idx1, int idx2) {
	
	if ((camData.K.rows != 3) || (camData.K.cols != 3) || (camData.K_inv.rows != 3) || (camData.K_inv.cols != 3)) {
		printf("%s << ERROR! Camera intrinsics dimensions are invalid.\n", __FUNCTION__);
		return false;
	}
	
	
	//printf("%s << DEBUG [%d]\n", __FUNCTION__, 0);
	
	cv::Mat real_P0, real_P1; 
	
	vector<cv::Point2f> pts1_, pts2_, pts1, pts2;
	
	printf("%s << Getting pts from tracks: (%d), (%d, %d), (%d, %d)\n", __FUNCTION__, ((int)tracks.size()), ((int)pts1_.size()), ((int)pts2_.size()), idx1, idx2);
	getPointsFromTracks(tracks, pts1_, pts2_, idx1, idx2);
	
	//subselectPoints(pts1_, pts1, pts2_, pts2);
	pts1.insert(pts1.end(), pts1_.begin(), pts1_.end());
	pts2.insert(pts2.end(), pts2_.begin(), pts2_.end());
	
	printf("%s << Checking pts size... (%d)\n", __FUNCTION__, ((int)pts1.size()));
	
	if (pts1.size() < 8) {
		printf("%s << ERROR! Too few corresponding points (%d)\n", __FUNCTION__, ((int)pts1.size()));
		return false;
	}
	
	vector<unsigned int> activeTrackIndices, fullSpanIndices;
	getActiveTracks(activeTrackIndices, tracks, idx1, idx2);
	filterToCompleteTracks(fullSpanIndices, activeTrackIndices, tracks, idx1, idx2);
	
	//printf("%s << DEBUG [%d]\n", __FUNCTION__, 2);
	
	cv::Mat F, matchesMask_F_matrix;
	F = cv::findFundamentalMat(cv::Mat(pts1), cv::Mat(pts2), cv::FM_RANSAC, 1.00, 0.99, matchesMask_F_matrix);

	cv::Mat E = camData.K.t() * F * camData.K;	
	cv::Mat CX[4];
	findFourTransformations(CX, E, camData.K, pts1, pts2);
	
	//printf("%s << DEBUG [%d]\n", __FUNCTION__, 3);
	
	cv::Mat C;
	int validPts = findBestCandidate(CX, camData.K, pts1, pts2, C);
	
	if (validPts < ((int) (0.5 * ((double) pts1.size())))) {
		printf("%s << ERROR! too few tracks (%d / %d) in best transform are in front of camera.\n", __FUNCTION__, ((int)validPts), ((int)pts1.size()));
		return false;
	}
		
	//printf("%s << DEBUG [%d]\n", __FUNCTION__, 4);
	
	cv::Mat P1, R1, t1, Rvec;
	transformationToProjection(C, P1);
	decomposeTransform(C, R1, t1);
	Rodrigues(R1, Rvec);

	//printf("%s << DEBUG [%d]\n", __FUNCTION__, 5);
	
	real_C1 = C * real_C0;	// real_C1 = C * real_C0;
	
	//real_C1 = C.inv() * real_C0;
	
	transformationToProjection(real_C0, real_P0);
	transformationToProjection(real_C1, real_P1);
	
	/*
	printf("%s << Putative camera poses (%d & %d)\n", __FUNCTION__, idx1, idx2);
	cout << "real_C0 = " << endl;
	cout << real_C0 << endl;
	cout << "real_C1 = " << endl;
	cout << real_C1 << endl;
	*/
	
	//printf("%s << DEBUG [%d]\n", __FUNCTION__, 6);
	
	//cout << camData.K << endl;
	//cout << camData.K_inv << endl;
	
	vector<cv::Point2f> correspPoints;
	TriangulatePoints(pts1, pts2, camData.K, camData.K_inv, real_C0, real_C1, ptCloud, correspPoints);
	//TriangulatePoints(pts1_, pts2_, camData.K, camData.K_inv, real_C1.inv(), real_C0.inv(), ptCloud, correspPoints);
	//TriangulatePoints(pts1, pts2, camData.K, camData.K_inv, real_P0, real_P1, ptCloud, correspPoints);
	//printf("%s << %d points triangulated.\n", __FUNCTION__, ptsInCloud.size());
	
	//printf("%s << DEBUG [%d]\n", __FUNCTION__, 7);
	
	triangulatedIndices.clear();
	triangulatedIndices.insert(triangulatedIndices.end(), fullSpanIndices.begin(), fullSpanIndices.end());
	
	//printf("%s << triangulatedIndices.size() = %d\n", __FUNCTION__, triangulatedIndices.size());
	
	updateTriangulatedPoints(tracks, triangulatedIndices, ptCloud);
	
	//printf("%s << DEBUG [%d]\n", __FUNCTION__, 8);
	
	//real_C1 = real_C1.inv();
	
	return true;			
}

void subselectPoints(const vector<cv::Point2f>& src1, vector<cv::Point2f>& dst1, const vector<cv::Point2f>& src2, vector<cv::Point2f>& dst2) {
	
	vector<cv::Point2f> new_src_1, new_src_2;
	new_src_1.insert(new_src_1.end(), src1.begin(), src1.end());
	new_src_2.insert(new_src_2.end(), src2.begin(), src2.end());
	
	int aimedPoints = 48;
	
	if (int(src1.size()) <= aimedPoints) {
		dst1.insert(dst1.end(), src1.begin(), src1.end());
		dst2.insert(dst2.end(), src2.begin(), src2.end());
		
		return;
	}
	
	// Find centroid
	cv::Point2f centroid_1, centroid_2;
	
	for (unsigned int iii = 0; iii < src1.size(); iii++) {
		centroid_1.x += src1.at(iii).x / ((double) src1.size());
		centroid_1.y += src1.at(iii).y / ((double) src1.size());
		
		centroid_2.x += src2.at(iii).x / ((double) src2.size());
		centroid_2.y += src2.at(iii).y / ((double) src2.size());
	} 
	
	printf("%s << Centroids: (%f, %f) & (%f, %f)\n", __FUNCTION__, centroid_1.x, centroid_1.y, centroid_2.x, centroid_2.y);
	
	// First add most central point
	int mostCentralIndex = -1;
	float bestDistance = 9e99;
	
	for (unsigned int iii = 0; iii < src1.size(); iii++) {
		
		float dist = distanceBetweenPoints(src1.at(iii), centroid_1) + distanceBetweenPoints(src2.at(iii), centroid_2);
		
		if ((dist < bestDistance) || (iii == 0)) {
			bestDistance = dist;
			mostCentralIndex = iii;
		}
		
	}
	
	if (mostCentralIndex < 0) {
		
		dst1.insert(dst1.end(), src1.begin(), src1.end());
		dst2.insert(dst2.end(), src2.begin(), src2.end());
		
		return;
		
	}
	
	dst1.push_back(new_src_1.at(mostCentralIndex));
	dst2.push_back(new_src_2.at(mostCentralIndex));
	
	// Correct centroid by removing contribution of removed point, multiplying by old size divided by new size
	centroid_1.x -= new_src_1.at(mostCentralIndex).x / ((double) src1.size());
	centroid_1.x *= ((double) src1.size()) / ((double) new_src_1.size());
	
	centroid_2.x -= new_src_2.at(mostCentralIndex).x / ((double) src2.size());
	centroid_2.x *= ((double) src2.size()) / ((double) new_src_2.size());
	
	new_src_1.erase(new_src_1.begin() + mostCentralIndex);
	new_src_2.erase(new_src_2.begin() + mostCentralIndex);
	
	printf("%s << Most central index: %d\n", __FUNCTION__, mostCentralIndex);
	
	for (int iii = 0; iii < aimedPoints-1; iii++) {
		
		cv::Point2f centroid_1, centroid_2;
		
		float largestDist;
		unsigned int largestIndex;
	
		for (unsigned int jjj = 0; jjj < new_src_1.size(); jjj++) {

			float dist = distanceBetweenPoints(new_src_1.at(jjj), centroid_1) + distanceBetweenPoints(new_src_2.at(jjj), centroid_2);
			
			if ((jjj == 0) || (dist > largestDist)) {
				largestDist = dist;
				largestIndex = jjj;
			}
			
		}
		
		centroid_1.x -= new_src_1.at(largestIndex).x / ((double) new_src_1.size());
		centroid_1.x *= ((double) new_src_1.size()) / ((double) new_src_1.size() - 1);
	
		centroid_2.x -= new_src_2.at(mostCentralIndex).x / ((double) new_src_2.size());
		centroid_2.x *= ((double) new_src_2.size()) / ((double) new_src_2.size() - 1);
		
		dst1.push_back(new_src_1.at(largestIndex));
		dst2.push_back(new_src_2.at(largestIndex));
		
		new_src_1.erase(new_src_1.begin() + largestIndex);
		new_src_2.erase(new_src_2.begin() + largestIndex);
		
	}
	
	//dst1.insert(dst1.end(), src1.begin(), src1.end());
	//dst2.insert(dst2.end(), src2.begin(), src2.end());
	
}

int estimatePoseBetweenCameras(cameraParameters& camData, vector<cv::Point2f>& pts1, vector<cv::Point2f>& pts2, cv::Mat& C) {
	cv::Mat F, matchesMask_F_matrix;
	
	F = cv::findFundamentalMat(cv::Mat(pts1), cv::Mat(pts2), cv::FM_RANSAC, 1.00, 0.99, matchesMask_F_matrix);
	
	cv::Mat E = camData.K.t() * F * camData.K;	
	
	cv::Mat CX[4];
	
	findFourTransformations(CX, E, camData.K, pts1, pts2);
	
	int validPts = findBestCandidate(CX, camData.K, pts1, pts2, C);
	
	return validPts;
}

double testKeyframePair(vector<featureTrack>& tracks, cameraParameters& camData, double *scorecard[], int idx1, int idx2, double *score, cv::Mat& pose, bool evaluate, bool debug) {
	
	//printf("%s << Entered.\n", __FUNCTION__);
	
	//if (debug) {
		//printf("%s << Testing frames (%d) & (%d)\n", __FUNCTION__, idx1, idx2);
	//}
	
	double keyframeScore;
	
	for (unsigned int iii = 0; iii < 5; iii++) {
		score[iii] = -1.0;
	}
	
	vector<cv::Point2f> pts1_, pts2_, pts1, pts2;
	getPointsFromTracks(tracks, pts1, pts2, idx1, idx2);
	
	//printf("%s << A: pts1.size() = %d; pts2.size() = %d\n", __FUNCTION__, pts1.size(), pts2.size());
	
	subselectPoints(pts1_, pts1, pts2_, pts2);
	
	pts1.insert(pts1.end(), pts1_.begin(), pts1_.end());
	pts2.insert(pts2.end(), pts2_.begin(), pts2_.end());
	
	//printf("%s << B: pts1.size() = %d; pts2.size() = %d\n", __FUNCTION__, pts1.size(), pts2.size());
	
	unsigned int min_pts = std::min(pts1.size(), pts2.size());
	
	if (min_pts < 16) {
		printf("%s << Returning with (-1); insufficient points.\n", __FUNCTION__);
		return -1.00;
	}
	
	cv::Mat matchesMask_F_matrix, matchesMask_H_matrix;
	
	cv::Mat F = cv::findFundamentalMat(cv::Mat(pts1), cv::Mat(pts2), cv::FM_RANSAC, 1.00, 0.99, matchesMask_F_matrix);
	cv::Mat H = cv::findHomography(cv::Mat(pts1), cv::Mat(pts2), cv::FM_RANSAC, 1.00, matchesMask_H_matrix);
	
	//int inliers_H = countNonZero(matchesMask_H_matrix);
	//int inliers_F = countNonZero(matchesMask_F_matrix);
	
	//double new_F_score = calcInlierGeometryDistance(pts1, pts2, F, matchesMask_F_matrix, SAMPSON_DISTANCE);
	//double new_H_score = calcInlierGeometryDistance(pts1, pts2, H, matchesMask_H_matrix, LOURAKIS_DISTANCE);
	
	//double geometryScore = calcGeometryScore(inliers_H, inliers_F, new_H_score, new_F_score);
	
	// gric score
	double fGric, hGric;
	double gricScore = normalizedGRICdifference(pts1, pts2, F, H, matchesMask_F_matrix, matchesMask_H_matrix, fGric, hGric);
	gricScore = fGric / hGric;
	//double gricIdeal = 2.0, gricMax = 10.0, gricMin = 1.0;
	double nGRIC = asymmetricGaussianValue(gricScore, scorecard[1][0], scorecard[1][2], scorecard[1][2]);
	//printf("%s << SCORES: geom (%f), conv (%f), gric (%f / [%d / %d]), pts (%f)\n", __FUNCTION__, geometryScore, twoErr, gricScore, (int) fGric, (int) hGric, infrontScore);
	
	score[1] = gricScore;
	
	//else if (geometryScore < 1.00) {
		// geometryScore = -1.00;
	//}
	
	if (nGRIC == 0.00) {
		if (!evaluate) {
			printf("%s << Returning with (0); nGric = 0.00.\n", __FUNCTION__);
			return 0.00;
		}
	}

	cv::Mat E = camData.K.t() * F * camData.K;	
	cv::Mat CX[4], C;
	findFourTransformations(CX, E, camData.K, pts1, pts2);
	int validPts = findBestCandidate(CX, camData.K, pts1, pts2, C);
	
	// infrontScore
	//double infrontIdeal = 1.00, infrontMax = 1.00, infrontMin = 0.90;
	double infrontScore = ((double) validPts) / ((double) pts1.size());
	double nIFS = asymmetricGaussianValue(infrontScore, scorecard[2][0], scorecard[2][2], scorecard[2][1]);
	
	score[2] = infrontScore;
	
	// Now correct if amt in front is greater than mean:
	if (infrontScore >= scorecard[2][0]) {
		nIFS = 1.00;
	}
	
	if (nIFS == 0) {
		if (!evaluate) {
			printf("%s << Returning with (0); nIFS = 0.00.\n", __FUNCTION__);
			return 0.00;
		}
	}
	
	cv::Mat absolute_C0, P0, P1;
	absolute_C0 = cv::Mat::eye(4, 4, CV_64FC1);
	
	transformationToProjection(absolute_C0, P0);
	transformationToProjection(C, P1);
	//pose.copyTo(P1);
	
	vector<cv::Point3d> cloud;
	vector<cv::Point2f> corresp;
	
	if (debug) {
		/*
		printf("%s << Before triangulation: \n", __FUNCTION__);
		cout << P0 << endl;
		cout << P1 << endl;
		*/
	}
	
	TriangulatePoints(pts1, pts2, camData.K, camData.K.inv(), P0, P1, cloud, corresp);
	
	//printf("%s << Before 2-frame BA: \n", __FUNCTION__);
	//cout << P0 << endl;
	//cout << P1 << endl;
	
	if (debug) {
		/*
		printf("%s << Before 2-frame BA: \n", __FUNCTION__);
		cout << P0 << endl;
		cout << P1 << endl;
		*/
	}
	
	// convergence
	//double convIdeal = 0.30, convMax = 0.80, convMin = 0.10;
	double twoErr = twoViewBundleAdjustment(camData, P0, P1, cloud, pts1, pts2, 10);
	
	score[0] = twoErr;
	
	if (debug) {
		/*
		printf("%s << After 2-frame BA: \n", __FUNCTION__);
		cout << P0 << endl;
		cout << P1 << endl;
		*/
		for (unsigned int iii = 0; iii < 10; iii++) {
			//printf("%s << pt(%d) = (%f, %f) & (%f, %f)\n", __FUNCTION__, iii, pts1.at(iii).x, pts1.at(iii).y, pts2.at(iii).x, pts2.at(iii).y);
		}
	}
	
	double nCONV = asymmetricGaussianValue(twoErr, scorecard[0][0], scorecard[0][2], scorecard[0][1]);
	
	if (nCONV == 0.00) {
		if (!evaluate) {
			printf("%s << Returning with (0); nCONV = 0.00.\n", __FUNCTION__);
			return 0.00;
		}
	}
	
	projectionToTransformation(P1, C);
	
	// break it down to get Z component and angle...
	cv::Mat R, t;
	decomposeTransform(C, R, t);
	
	// translation score
	//double transIdeal = 3.00, transMax = 5.00, transMin = 1.00;
	double tScore = (abs(t.at<double>(0,0)) + abs(t.at<double>(1,0))) / abs(t.at<double>(2,0));
	
	score[3] = tScore;
	
	double nTRN = asymmetricGaussianValue(tScore, scorecard[3][0], scorecard[3][2], scorecard[3][1]);
	
	if (nTRN == 0.00) {
		if (!evaluate) {
			return 0.00;
		}
	}
	
	// angle score
	//double angleIdeal = 10.0, angleMax = 15.0, angleMin = 5.0;
	double dScore = getRotationInDegrees(R);
	
	score[4] = dScore;
	
	double nANG = asymmetricGaussianValue(dScore, scorecard[4][0], scorecard[4][2], scorecard[4][1]);
	
	if (nANG == 0.00) {
		if (!evaluate) {
			printf("%s << Returning with (0); nANG = 0.00.\n", __FUNCTION__);
			return 0.00;
		}
	}
	
	//double homographyScore = 0.00;	// later this will be replaced by a method for determining FOV change / rot angle
	
	// Followed by checks to set homography score to be less than zero (not enough view change, H-score too low etc)
	
	
	
	
	
	
	
	//cout << t << endl;
	
	//printf("%s << Convergence score = %f\n", __FUNCTION__, twoErr);
	
	unsigned int numTerms = 5;
	keyframeScore = pow(nCONV * nGRIC * nIFS * nTRN * nANG, 1.0 / ((double) numTerms));
	
	//if (debug) {
		printf("%s << [%f]: conv (%1.2f, %1.2f), gric (%1.2f, %1.2f), if (%1.2f, %1.2f), trans (%1.2f, %1.2f), ang (%02.1f, %1.2f)\n", __FUNCTION__, keyframeScore, twoErr, nCONV, gricScore, nGRIC, infrontScore, nIFS, tScore, nTRN, dScore, nANG);
	//}
	
	
	P1.copyTo(pose);
	
	//printf("%s << Exiting.\n", __FUNCTION__);
	
	return keyframeScore;
	
}


//double testKeyframePair(vector<featureTrack>& tracks, cameraParameters& camData, int idx1, int idx2, double *score, Mat& pose, bool evaluate, bool debug) {
	
	//printf("%s << Entered.\n", __FUNCTION__);
	
	////if (debug) {
		//printf("%s << Testing frames (%d) & (%d)\n", __FUNCTION__, idx1, idx2);
	////}
	
	//double keyframeScore;
	
	//for (unsigned int iii = 0; iii < 5; iii++) {
		//score[iii] = -1.0;
	//}
	
	//vector<Point2f> pts1_, pts2_, pts1, pts2;
	//getPointsFromTracks(tracks, pts1, pts2, idx1, idx2);
	
	//printf("%s << A: pts1.size() = %d; pts2.size() = %d\n", __FUNCTION__, pts1.size(), pts2.size());
	
	//subselectPoints(pts1_, pts1, pts2_, pts2);
	
	//pts1.insert(pts1.end(), pts1_.begin(), pts1_.end());
	//pts2.insert(pts2.end(), pts2_.begin(), pts2_.end());
	
	//printf("%s << B: pts1.size() = %d; pts2.size() = %d\n", __FUNCTION__, pts1.size(), pts2.size());
	
	//unsigned int min_pts = std::min(pts1.size(), pts2.size());
	
	//if (min_pts < 16) {
		//return -1.00;
	//}
	
	//Mat matchesMask_F_matrix, matchesMask_H_matrix;
	
	//Mat F = findFundamentalMat(Mat(pts1), Mat(pts2), FM_RANSAC, 1.00, 0.99, matchesMask_F_matrix);
	//Mat H = findHomography(Mat(pts1), Mat(pts2), FM_RANSAC, 1.00, matchesMask_H_matrix);
	
	////int inliers_H = countNonZero(matchesMask_H_matrix);
	////int inliers_F = countNonZero(matchesMask_F_matrix);
	
	////double new_F_score = calcInlierGeometryDistance(pts1, pts2, F, matchesMask_F_matrix, SAMPSON_DISTANCE);
	////double new_H_score = calcInlierGeometryDistance(pts1, pts2, H, matchesMask_H_matrix, LOURAKIS_DISTANCE);
	
	////double geometryScore = calcGeometryScore(inliers_H, inliers_F, new_H_score, new_F_score);
	
	//// gric score
	//double fGric, hGric;
	//double gricScore = normalizedGRICdifference(pts1, pts2, F, H, matchesMask_F_matrix, matchesMask_H_matrix, fGric, hGric);
	//gricScore = fGric / hGric;
	//double gricIdeal = 2.0, gricMax = 10.0, gricMin = 1.0;
	//double nGRIC = asymmetricGaussianValue(gricScore, gricIdeal, gricMin, gricMax);
	////printf("%s << SCORES: geom (%f), conv (%f), gric (%f / [%d / %d]), pts (%f)\n", __FUNCTION__, geometryScore, twoErr, gricScore, (int) fGric, (int) hGric, infrontScore);
	
	////else if (geometryScore < 1.00) {
		//// geometryScore = -1.00;
	////}
	
	//if (nGRIC == 0.00) {
		//if (!evaluate) {
			//return 0.00;
		//}
	//}

	//Mat E = camData.K.t() * F * camData.K;	
	//Mat CX[4], C;
	//findFourTransformations(CX, E, camData.K, pts1, pts2);
	//int validPts = findBestCandidate(CX, camData.K, pts1, pts2, C);
	
	//// infrontScore
	//double infrontIdeal = 1.00, infrontMax = 1.00, infrontMin = 0.90;
	//double infrontScore = ((double) validPts) / ((double) pts1.size());
	//double nIFS = asymmetricGaussianValue(infrontScore, infrontIdeal, infrontMin, infrontMax);
	
	//if (nIFS == 0) {
		//if (!evaluate) {
			//return 0.00;
		//}
	//}
	
	//Mat absolute_C0, P0, P1;
	//absolute_C0 = Mat::eye(4, 4, CV_64FC1);
	
	//transformationToProjection(absolute_C0, P0);
	//transformationToProjection(C, P1);
	////pose.copyTo(P1);
	
	//vector<Point3d> cloud;
	//vector<Point2f> corresp;
	
	//if (debug) {
		///*
		//printf("%s << Before triangulation: \n", __FUNCTION__);
		//cout << P0 << endl;
		//cout << P1 << endl;
		//*/
	//}
	
	//TriangulatePoints(pts1, pts2, camData.K, camData.K.inv(), P0, P1, cloud, corresp);
	
	////printf("%s << Before 2-frame BA: \n", __FUNCTION__);
	////cout << P0 << endl;
	////cout << P1 << endl;
	
	//if (debug) {
		///*
		//printf("%s << Before 2-frame BA: \n", __FUNCTION__);
		//cout << P0 << endl;
		//cout << P1 << endl;
		//*/
	//}
	
	//// convergence
	//double convIdeal = 0.30, convMax = 0.80, convMin = 0.10;
	//double twoErr = twoViewBundleAdjustment(camData, P0, P1, cloud, pts1, pts2, 10);
	
	
	
	//if (debug) {
		///*
		//printf("%s << After 2-frame BA: \n", __FUNCTION__);
		//cout << P0 << endl;
		//cout << P1 << endl;
		//*/
		//for (unsigned int iii = 0; iii < 10; iii++) {
			////printf("%s << pt(%d) = (%f, %f) & (%f, %f)\n", __FUNCTION__, iii, pts1.at(iii).x, pts1.at(iii).y, pts2.at(iii).x, pts2.at(iii).y);
		//}
	//}
	
	//double nCONV = asymmetricGaussianValue(twoErr, convIdeal, convMin, convMax);
	
	//if (nCONV == 0.00) {
		//if (!evaluate) {
			//return 0.00;
		//}
	//}
	
	//projectionToTransformation(P1, C);
	
	//// break it down to get Z component and angle...
	//Mat R, t;
	//decomposeTransform(C, R, t);
	
	//// translation score
	//double transIdeal = 3.00, transMax = 5.00, transMin = 1.00;
	//double tScore = (abs(t.at<double>(0,0)) + abs(t.at<double>(1,0))) / abs(t.at<double>(2,0));
	//double nTRN = asymmetricGaussianValue(tScore, transIdeal, transMin, transMax);
	
	//if (nTRN == 0.00) {
		//if (!evaluate) {
			//return 0.00;
		//}
	//}
	
	//// angle score
	//double angleIdeal = 10.0, angleMax = 15.0, angleMin = 5.0;
	//double dScore = getRotationInDegrees(R);
	//double nANG = asymmetricGaussianValue(dScore, angleIdeal, angleMin, angleMax);
	
	//if (nANG == 0.00) {
		//if (!evaluate) {
			//return 0.00;
		//}
	//}
	
	////double homographyScore = 0.00;	// later this will be replaced by a method for determining FOV change / rot angle
	
	//// Followed by checks to set homography score to be less than zero (not enough view change, H-score too low etc)
	
	//score[0] = twoErr;
	//score[1] = gricScore;
	//score[2] = infrontScore;
	//score[3] = tScore;
	//score[4] = dScore;
	
	////cout << t << endl;
	
	////printf("%s << Convergence score = %f\n", __FUNCTION__, twoErr);
	
	//unsigned int numTerms = 5;
	//keyframeScore = pow(nCONV * nGRIC * nIFS * nTRN * nANG, 1.0 / ((double) numTerms));
	
	////if (debug) {
		//printf("%s << [%f]: conv (%1.2f, %1.2f), gric (%1.2f, %1.2f), if (%1.2f, %1.2f), trans (%1.2f, %1.2f), ang (%02.1f, %1.2f)\n", __FUNCTION__, keyframeScore, twoErr, nCONV, gricScore, nGRIC, infrontScore, nIFS, tScore, nTRN, dScore, nANG);
	////}
	
	
	//P1.copyTo(pose);
	
	//printf("%s << Exiting.\n", __FUNCTION__);
	
	//return keyframeScore;
	
//}


double optimizeSystem(SysSBA &sba, double err, int iterations, bool debug, int mode) {
	
	if (0) { printf("%s << ENTERED.\n", __FUNCTION__); }
	
	double error = -1.0, prevError = 9e99;
	
	unsigned int groupsize = min(50, iterations/10);
	
	if (debug) {
		sba.verbose = 1;
	} else {
		sba.verbose = 0;
	}
	
	// Filtering...?
	//sba.reduceTracks();
	//sba.reduceLongTracks();
	//sba.remExcessTracks();
	//sba.removeBad();
	
	
	//printf("%s << Bad points [pre] = (%d)\n", __FUNCTION__, sba.numBadPoints());
	
	
	
	
	//printf("%s << DEBUG (%d)\n", __FUNCTION__, 0);
	
	for (unsigned int iii = 0; iii < iterations/groupsize; iii++) {
		
		//printf("%s << DEBUG (%d)(%d) : (%d, %d)\n", __FUNCTION__, iii, 0, sba.nodes.size(), sba.tracks.size());
		
		if (debug) { printf("%s << About to attempt SBA (%d) / (%d)\n", __FUNCTION__, ((int)iii), iterations/groupsize); }
		
		int actualIts = sba.doSBA(groupsize, err, mode);
		
		if (actualIts == 0) {
			if (debug) { printf("%s << SBA did not iterate... (%f)\n", __FUNCTION__, sba.calcAvgError()); }
			break;
			//return 9e99;
		}
		if (debug) { printf("%s << SBA completed with (%d) iterations.\n", __FUNCTION__, actualIts); }
		
		//printf("%s << DEBUG (%d)(%d)\n", __FUNCTION__, iii, 1);
		
		error = sba.calcAvgError();
		if (debug) { printf("%s << current error = (%f)\n", __FUNCTION__, error); }
		
		//printf("%s << DEBUG (%d)(%d)\n", __FUNCTION__, iii, 2);
		
		if (error >= prevError) {
			break; // return error;
		} else {
			prevError = error;
		}
		
		//printf("%s << DEBUG (%d)(%d)\n", __FUNCTION__, iii, 3);
		//printf("%s << error(%d) = %f\n", __FUNCTION__, iii, error);
	}
	
	//printf("%s << DEBUG (%d)\n", __FUNCTION__, 1);
	
	//int iterationsPerformed = sba.doSBA(iterations, err, SBA_SPARSE_CHOLESKY);
	//printf("%s << its performed = %d\n", __FUNCTION__, iterationsPerformed);

	if (debug) { printf("%s << Bad points [post] = (%d)\n", __FUNCTION__, sba.numBadPoints()); }

	// sba.doSBA(iterations, err, SBA_SPARSE_CHOLESKY);
	//constrainDodgyPoints(sba);
	
	//printf("%s << Bad points [final] = (%d)\n", __FUNCTION__, sba.numBadPoints());
	
	if (debug) { printf("%s << EXITING.\n", __FUNCTION__); }
	
	return error;
}

double twoViewBundleAdjustment(cameraParameters cam_data, cv::Mat& cam1, cv::Mat& cam2, vector<cv::Point3d>& cloud, vector<cv::Point2f>& pts1, vector<cv::Point2f>& pts2, int iterations) {	
	
	SysSBA sys;
	
	sys.verbose = 0;
	
	addFixedCamera(sys, cam_data, cam1);
	addNewCamera(sys, cam_data, cam2);
	
	addPointsToSBA(sys, cloud);
	
	addProjectionsToSBA(sys, pts1, 0);
	addProjectionsToSBA(sys, pts2, 1);
	
	// printf("%s << nodes = %d; tracks = %d\n", __FUNCTION__, sys.nodes.size(), sys.tracks.size());
	
	double avgError = optimizeSystem(sys, 1e-4, iterations);
	
	retrieveCameraPose(sys, 1, cam2);
	cloud.clear();
	retrieveAllPoints(cloud, sys);
	
	return avgError;
	
}

void addPointsToSBA(SysSBA& sba, vector<cv::Point3d>& cloud) {
	
	//Vector2d proj;
	
	for (unsigned int iii = 0; iii < cloud.size(); iii++) {
		
		Vector4d temppoint(cloud.at(iii).x, cloud.at(iii).y, cloud.at(iii).z, 1.0);
		
		sba.addPoint(temppoint);
		
		/*
		proj.x() = loc1.at(iii).x;
		proj.y() = loc1.at(iii).y;
		
		sba.addMonoProj(idx0, iii, proj);
		
		proj.x() = loc2.at(iii).x;
		proj.y() = loc2.at(iii).y;
		
		sba.addMonoProj(idx1, iii, proj);
		* */
	}	
}

void addProjectionToSBA(SysSBA& sba, cv::Point2f& loc, unsigned int trackNo, unsigned int camNo) {
	
	Vector2d proj;
	proj.x() = loc.x;
	proj.y() = loc.y;
	sba.addMonoProj(camNo, trackNo, proj);
}

void addProjectionsToSBA(SysSBA& sba, vector<cv::Point2f>& loc, int idx) {
	
	Vector2d proj;
	
	// printf("%s << ENTERED.\n", __FUNCTION__);
	
	for (unsigned int iii = 0; iii < loc.size(); iii++) {
		
		if (iii >= sba.tracks.size()) {
			printf("%s << WARNING! provided locations exceed number of tracks in system...\n", __FUNCTION__);
			break;
		}
		
		//printf("%s << it(%d)\n", __FUNCTION__, iii);
		proj.x() = loc.at(iii).x;
		proj.y() = loc.at(iii).y;
		
		//printf("%s << Adding projection... (%d / %d) : (%d / %d)\n", __FUNCTION__, iii, sba.tracks.size(), idx, sba.nodes.size());
		sba.addMonoProj(idx, iii, proj);
		
		//printf("%s << Projection added. (%d)\n", __FUNCTION__, iii);
	}
	
	printf("%s << EXITING.\n", __FUNCTION__);
}

void extractPointCloud(const SysSBA &sba, pcl::PointCloud<pcl::PointXYZ>& point_cloud) {
	
	for (unsigned int kkk = 0; kkk < sba.tracks.size(); kkk++) {
			
			Vector4d pt = sba.tracks[kkk].point;
			
			pcl::PointXYZ pclPt(pt(2), -pt(0), -pt(1));				
			point_cloud.points.push_back(pclPt);
			//printf("%s << cloud_pt(%d) = (%f, %f, %f)\n", __FUNCTION__, kkk, pointCloud.at(kkk).x, pointCloud.at(kkk).y, pointCloud.at(kkk).z);
		}
}

void extractCameras(const SysSBA &sba, visualization_msgs::MarkerArray& cameraArray, visualization_msgs::Marker& marker_path) {
	
	uint32_t shape = visualization_msgs::Marker::ARROW;
	
	cameraArray.markers.clear();
	
	visualization_msgs::Marker marker;
	marker.header.frame_id = "/pgraph";
	marker.ns = "cameras";
	marker.type = shape;
	marker.action = visualization_msgs::Marker::ADD;
	marker.scale.x = 0.5;
	marker.scale.y = 0.5;
	marker.scale.z = 0.5;
	
	// And adjust path...
	
	marker_path.points.clear();
	
	marker_path.header.frame_id = "/pgraph";
	marker_path.header.stamp = ros::Time();
	marker_path.ns = "camera_path";
	marker_path.id = 0;
	marker_path.type = visualization_msgs::Marker::LINE_STRIP;
	marker_path.action = visualization_msgs::Marker::ADD;
	
	marker_path.pose.position.x = 0.0;
	marker_path.pose.position.y = 0.0;
	marker_path.pose.position.z = 0.0;
	
	marker_path.pose.orientation.x = 0.0;
	marker_path.pose.orientation.y = 0.0;
	marker_path.pose.orientation.z = 0.0;
	marker_path.pose.orientation.w = 1.0;
	
	marker_path.scale.x = 0.02;
	
	marker_path.color.a = 1.0;
	marker_path.color.r = 0.0;
	marker_path.color.g = 0.0;
	marker_path.color.b = 1.0;
	
	marker_path.lifetime = ros::Duration();
	
	geometry_msgs::Point p;
	
	for (unsigned int iii = 0; iii < sba.nodes.size(); iii++) {
		
		printf("%s << Adding node #%d\n", __FUNCTION__, iii);
		marker.header.stamp = ros::Time();
		marker.id = iii;
		
		
		
		marker.pose.position.x = sba.nodes.at(iii).trans.x();
		marker.pose.position.y = sba.nodes.at(iii).trans.y();
		marker.pose.position.z = sba.nodes.at(iii).trans.z();
		
		p.x = sba.nodes.at(iii).trans.x();
		p.y = sba.nodes.at(iii).trans.y();
		p.z = sba.nodes.at(iii).trans.z();
		
		printf("%s << New co-ordinates = [%f, %f, %f]\n", __FUNCTION__, p.x, p.y, p.z);
		
		marker.pose.orientation.x = sba.nodes.at(iii).qrot.x();
		marker.pose.orientation.y = sba.nodes.at(iii).qrot.y();
		marker.pose.orientation.z = sba.nodes.at(iii).qrot.z();
		marker.pose.orientation.w = 1.0;
		
		printf("%s << New rotations are = [%f, %f, %f]\n", __FUNCTION__, marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z);
		
		marker_path.points.push_back(p);
		
		cameraArray.markers.push_back( marker );
	}

}

void initializeFrameCamera(frame_common::CamParams& cam_params, const cv::Mat& newCamMat, int& maxx, int& maxy, const cv::Size& cameraSize) {
    
    cam_params.fx = newCamMat.at<double>(0,0); // Focal length in x
    cam_params.fy = newCamMat.at<double>(1,1); // Focal length in y
    cam_params.cx = newCamMat.at<double>(0,2); // X position of principal point
    cam_params.cy = newCamMat.at<double>(1,2); // Y position of principal point
    cam_params.tx = 0;   // Baseline (no baseline since this is monocular)

    // Define dimensions of the image.
    maxx = cameraSize.width;
    maxy = cameraSize.height;
}

void drawKeyframes(const ros::Publisher &camera_pub, const geometry_msgs::PoseStamped *keyframePoses, unsigned int keyframeCount) {

	visualization_msgs::Marker camera_marker;
	camera_marker.header.frame_id = "/world";
	camera_marker.header.stamp = ros::Time::now();
	camera_marker.ns = "world";
	camera_marker.id = 0;
	camera_marker.action = visualization_msgs::Marker::ADD;
	camera_marker.pose.position.x = 0;
	camera_marker.pose.position.y = 0;
	camera_marker.pose.position.z = 0;
	camera_marker.pose.orientation.x = 0.0;
	camera_marker.pose.orientation.y = 0.0;
	camera_marker.pose.orientation.z = 0.0;
	camera_marker.pose.orientation.w = 1.0;
	camera_marker.scale.x = 0.02;
	camera_marker.scale.y = 0.02;
	camera_marker.scale.z = 0.02;
	camera_marker.color.r = 1.0f;
	camera_marker.color.g = 0.0f;
	camera_marker.color.b = 0.0f;
	camera_marker.color.a = 1.0f;
	camera_marker.lifetime = ros::Duration();
	camera_marker.type = visualization_msgs::Marker::LINE_LIST;
	
	int num_cameras = keyframeCount;
	
	camera_marker.points.resize(num_cameras*6);
	camera_marker.colors.resize(num_cameras*6);

	double y_length = 0.02, x_length = 0.05, z_length = 0.10;


	//double maxCam = 0.00, maxCam2 = 0.00;
	//double maxOpt = 0.00, maxOpt2 = 0.00;



	for (int i=0, ii=0; i < num_cameras; i++)
	 {
		 
		 float rv = ((float) (((float) i) / ((float) num_cameras)) * 1.0f);
		 float bv = ((float) (((float) (num_cameras - i)) / ((float) num_cameras)) * 1.0f);
		 
	   //const Node &nd = sba.nodes[i];
	   //Node nd(sba.nodes[i]);
	   
	   //q1 = Quaterniond(keyframePoses[cameraIndex1].pose.orientation.w, keyframePoses[cameraIndex1].pose.orientation.x, keyframePoses[cameraIndex1].pose.orientation.y, keyframePoses[cameraIndex1].pose.orientation.z);
	  //v1 = Eigen::Vector4d(keyframePoses[cameraIndex1].pose.position.x, keyframePoses[cameraIndex1].pose.position.y, keyframePoses[cameraIndex1].pose.position.z, 1.0);
			
	   Matrix<double,3,4> tr;
	   Vector3d  opt;
	   
	   //printf("%s << keyframePoses[(%d)].pose.orientation = (%f, %f, %f, %f)\n", __FUNCTION__, i, keyframePoses[i].pose.orientation.w, keyframePoses[i].pose.orientation.x, keyframePoses[i].pose.orientation.y, keyframePoses[i].pose.orientation.z);
	   
	   Eigen::Quaterniond q(keyframePoses[i].pose.orientation.w, keyframePoses[i].pose.orientation.x, keyframePoses[i].pose.orientation.y, keyframePoses[i].pose.orientation.z);
	   Matrix<double,4,1> t;
	   
	   t(0,0) = keyframePoses[i].pose.position.x;
	   t(1,0) = keyframePoses[i].pose.position.y;
	   t(2,0) = keyframePoses[i].pose.position.z;
	   t(3,0) = 1.0;
	   
	   transformF2W(tr,t,q);
	   
	   camera_marker.colors[ii].r = rv;
	   camera_marker.colors[ii].b = bv;

	   camera_marker.points[ii].x = keyframePoses[i].pose.position.x;
	   camera_marker.points[ii].y = keyframePoses[i].pose.position.y;
	   camera_marker.points[ii++].z = keyframePoses[i].pose.position.z;

		camera_marker.colors[ii].r = rv;
	   camera_marker.colors[ii].b = bv;
	   
	   opt = tr*Vector4d(0,0,z_length,1);
	   
	   camera_marker.points[ii].x = opt.x();
	   camera_marker.points[ii].y = opt.y();
	   camera_marker.points[ii++].z = opt.z();

		camera_marker.colors[ii].r = rv;
	   camera_marker.colors[ii].b = bv;
	   
	   camera_marker.points[ii].x = keyframePoses[i].pose.position.x;
	   camera_marker.points[ii].y = keyframePoses[i].pose.position.y;
	   camera_marker.points[ii++].z = keyframePoses[i].pose.position.z;
	   
		camera_marker.colors[ii].r = rv;
	   camera_marker.colors[ii].b = bv;
	   
	   opt = tr*Vector4d(x_length,0,0,1);
	   
	   camera_marker.points[ii].x = opt.x();
	   camera_marker.points[ii].y = opt.y();
	   camera_marker.points[ii++].z = opt.z();
	   
	   
		 camera_marker.colors[ii].r = rv;
	   camera_marker.colors[ii].b = bv;


	   camera_marker.points[ii].x = keyframePoses[i].pose.position.x;
	   camera_marker.points[ii].y = keyframePoses[i].pose.position.y;
	   camera_marker.points[ii++].z = keyframePoses[i].pose.position.z;
	   
	   camera_marker.colors[ii].r = rv;
	   camera_marker.colors[ii].b = bv;
	   
	   opt = tr*Vector4d(0,y_length,0,1);
	   
	   
	   
	   camera_marker.points[ii].x = opt.x();
	   camera_marker.points[ii].y = opt.y();
	   camera_marker.points[ii++].z = opt.z();

	 }
	 
	 //printf("%s << Publishing keyframe markers (%d, %d)\n", __FUNCTION__, camera_marker.points.size(), camera_marker.colors.size());
	 camera_pub.publish(camera_marker);
}

void drawGraph2(const SysSBA &sba, const ros::Publisher &camera_pub,
                const ros::Publisher &point_pub, const ros::Publisher &path_pub, int decimation, int bicolor, double scale)
 {
   int num_points = sba.tracks.size();
   int num_cameras = sba.nodes.size();
   if (num_points == 0 && num_cameras == 0) return;
   
   visualization_msgs::Marker camera_marker, point_marker, path_marker;
   camera_marker.header.frame_id = "/pgraph";
   camera_marker.header.stamp = ros::Time::now();
   camera_marker.ns = "pgraph";
   camera_marker.id = 0;
   camera_marker.action = visualization_msgs::Marker::ADD;
   camera_marker.pose.position.x = 0;
   camera_marker.pose.position.y = 0;
   camera_marker.pose.position.z = 0;
   camera_marker.pose.orientation.x = 0.0;
   camera_marker.pose.orientation.y = 0.0;
   camera_marker.pose.orientation.z = 0.0;
   camera_marker.pose.orientation.w = 1.0;
   camera_marker.scale.x = 0.02;
   camera_marker.scale.y = 0.02;
   camera_marker.scale.z = 0.02;
   camera_marker.color.r = 1.0f;
   camera_marker.color.g = 0.0f;
   camera_marker.color.b = 0.0f;
   camera_marker.color.a = 1.0f;
   camera_marker.lifetime = ros::Duration();
   camera_marker.type = visualization_msgs::Marker::LINE_LIST;
 
   point_marker = camera_marker;
   point_marker.color.r = 0.5f;
   point_marker.color.g = 0.5f;
   point_marker.color.b = 1.0f;
   point_marker.color.a = 1.0f;	// 0.5f
   point_marker.scale.x = scale;
   point_marker.scale.y = scale;
   point_marker.scale.z = scale;
   point_marker.type = visualization_msgs::Marker::POINTS;
   
   path_marker = point_marker;
   path_marker.color.r = 0.0f;
   path_marker.color.g = 0.5f;
   path_marker.color.b = 0.0f;
   path_marker.color.a = 1.0f;
   path_marker.scale.x = 0.03;
   path_marker.type = visualization_msgs::Marker::LINE_STRIP;
   
   
   double maxPt = 0.00, maxPt2 = 0.00;
	
   // draw points, decimated
   point_marker.points.resize((int)(num_points/(double)decimation + 0.5));
   point_marker.colors.resize((int)(num_points/(double)decimation + 0.5));
   for (int i=0, ii=0; i < num_points; i += decimation, ii++)
     {
       //const Vector4d &pt = sba.tracks[i].point;
       Vector4d pt(sba.tracks[i].point);
       //point_marker.colors[ii].r = 1.0f;
       //if (bicolor > 0 && i >= bicolor)
         //point_marker.colors[ii].g = 1.0f;
       //else
         //point_marker.colors[ii].g = 0.0f;
       //point_marker.colors[ii].b = 0.0f;

		maxPt = max(maxPt, abs(pt(2)));
		maxPt = max(maxPt, abs(pt(1)));
		maxPt = max(maxPt, abs(pt(0)));
       
       if (abs(pt(2)) > MAX_RVIZ_DISPLACEMENT) {
		   pt(2) = 0.0;
	   }
	   
	   if (abs(pt(0)) > MAX_RVIZ_DISPLACEMENT) {
		   pt(0) = 0.0;
	   }
	   
	   if (abs(pt(1)) > MAX_RVIZ_DISPLACEMENT) {
		   pt(1) = 0.0;
	   }
	   
	   maxPt2 = max(maxPt2, abs(pt(2)));
		maxPt2 = max(maxPt2, abs(pt(1)));
		maxPt2 = max(maxPt2, abs(pt(0)));
	   
       point_marker.points[ii].x = pt(2);
       point_marker.points[ii].y = -pt(0);
       point_marker.points[ii].z = -pt(1);
       
       point_marker.colors[ii].r = 0.0;
       point_marker.colors[ii].g = 0.0;
       point_marker.colors[ii].b = 0.0;
       point_marker.colors[ii].a = 1.0;
       
     }
     
   //hajmig
   //printf("%s << maxPt = (%f, %f)\n", __FUNCTION__, maxPt, maxPt2);
 
   // draw cameras
   camera_marker.points.resize(num_cameras*6);
   camera_marker.colors.resize(num_cameras*6);
   
   double y_length = 0.00, x_length = 0.00, z_length = 0.30;
   
   
  double maxCam = 0.00, maxCam2 = 0.00;
  double maxOpt = 0.00, maxOpt2 = 0.00;

   
   
   for (int i=0, ii=0; i < num_cameras; i++)
     {
		 
		 float rv = ((float) (((float) i) / ((float) num_cameras)) * 1.0f);
		 float bv = ((float) (((float) (num_cameras - i)) / ((float) num_cameras)) * 1.0f);
		 
       //const Node &nd = sba.nodes[i];
       Node nd(sba.nodes[i]);
       Vector3d  opt;
       Matrix<double,3,4> tr;
       transformF2W(tr,nd.trans,nd.qrot);
       
       maxCam = max(maxCam, abs(nd.trans.z()));
       maxCam = max(maxCam, abs(nd.trans.x()));
       maxCam = max(maxCam, abs(nd.trans.y()));
       
       if (abs(nd.trans.z()) > MAX_RVIZ_DISPLACEMENT) {
		   nd.trans.z() = 0.0;
	   }
	   
	   if (abs(nd.trans.x()) > MAX_RVIZ_DISPLACEMENT) {
		   nd.trans.x() = 0.0;
	   }
	   
	   if (abs(nd.trans.y()) > MAX_RVIZ_DISPLACEMENT) {
		   nd.trans.y() = 0.0;
	   }
	   
	   maxCam2 = max(maxCam2, abs(nd.trans.z()));
       maxCam2 = max(maxCam2, abs(nd.trans.x()));
       maxCam2 = max(maxCam2, abs(nd.trans.y()));
       
       camera_marker.colors[ii].r = rv;
       camera_marker.colors[ii].b = bv;
 
       camera_marker.points[ii].x = nd.trans.z();
       camera_marker.points[ii].y = -nd.trans.x();
       camera_marker.points[ii++].z = -nd.trans.y();
       
        camera_marker.colors[ii].r = rv;
       camera_marker.colors[ii].b = bv;
       
       opt = tr*Vector4d(0,0,z_length,1);
       
       maxOpt = max(maxOpt, abs(opt.z()));
       maxOpt = max(maxOpt, abs(opt.x()));
       maxOpt = max(maxOpt, abs(opt.y()));
       
       if (abs(opt.z()) > MAX_RVIZ_DISPLACEMENT) {
		   opt.z() = 0.0;
	   }
	   
	   if (abs(opt.x()) > MAX_RVIZ_DISPLACEMENT) {
		   opt.x() = 0.0;
	   }
	   
	   if (abs(opt.y()) > MAX_RVIZ_DISPLACEMENT) {
		   opt.y() = 0.0;
	   }
       
		maxOpt2 = max(maxOpt2, abs(opt.z()));
       maxOpt2 = max(maxOpt2, abs(opt.x()));
       maxOpt2 = max(maxOpt2, abs(opt.y()));
       
       
       camera_marker.points[ii].x = opt.z();
       camera_marker.points[ii].y = -opt.x();
       camera_marker.points[ii++].z = -opt.y();
 
        camera_marker.colors[ii].r = rv;
       camera_marker.colors[ii].b = bv;
       
       camera_marker.points[ii].x = nd.trans.z();
       camera_marker.points[ii].y = -nd.trans.x();
       camera_marker.points[ii++].z = -nd.trans.y();
       
        camera_marker.colors[ii].r = rv;
       camera_marker.colors[ii].b = bv;
       
       opt = tr*Vector4d(x_length,0,0,1);
       
       if (abs(opt.z()) > MAX_RVIZ_DISPLACEMENT) {
		   opt.z() = 0.0;
	   }
	   
	   if (abs(opt.x()) > MAX_RVIZ_DISPLACEMENT) {
		   opt.x() = 0.0;
	   }
	   
	   if (abs(opt.y()) > MAX_RVIZ_DISPLACEMENT) {
		   opt.y() = 0.0;
	   }
	   
       camera_marker.points[ii].x = opt.z();
       camera_marker.points[ii].y = -opt.x();
       camera_marker.points[ii++].z = -opt.y();
       
       
         camera_marker.colors[ii].r = rv;
       camera_marker.colors[ii].b = bv;
 
	
       camera_marker.points[ii].x = nd.trans.z();
       camera_marker.points[ii].y = -nd.trans.x();
       camera_marker.points[ii++].z = -nd.trans.y();
       
       camera_marker.colors[ii].r = rv;
       camera_marker.colors[ii].b = bv;
       
       opt = tr*Vector4d(0,y_length,0,1);
       
       if (abs(opt.z()) > MAX_RVIZ_DISPLACEMENT) {
		   opt.z() = 0.0;
	   }
	   
	   if (abs(opt.x()) > MAX_RVIZ_DISPLACEMENT) {
		   opt.x() = 0.0;
	   }
	   
	   if (abs(opt.y()) > MAX_RVIZ_DISPLACEMENT) {
		   opt.y() = 0.0;
	   }
	   
       camera_marker.points[ii].x = opt.z();
       camera_marker.points[ii].y = -opt.x();
       camera_marker.points[ii++].z = -opt.y();

       
     }
     
    //hajmig
    // printf("%s << maxCam = (%f, %f)\n", __FUNCTION__, maxCam, maxCam2);
    // printf("%s << maxOpt = (%f, %f)\n", __FUNCTION__, maxOpt, maxOpt2);
     
     // Draw path
     path_marker.points.resize(num_cameras);
     path_marker.colors.resize(num_cameras);
     
     //Matrix<double,3,4> pr;
     
     for (int ii=0; ii < num_cameras; ii++)
     {
		 
		 float rv = ((float) (((float) ii) / ((float) num_cameras)) * 1.0f);
		 float bv = ((float) (((float) (num_cameras - ii)) / ((float) num_cameras)) * 1.0f);

       const Node &nd = sba.nodes[ii];
       Vector3d opt;
       Matrix<double,3,4> tr;
       transformF2W(tr,nd.trans,nd.qrot);
 
       path_marker.points[ii].x = nd.trans.z();
       path_marker.points[ii].y = -nd.trans.x();
       path_marker.points[ii].z = -nd.trans.y();
       
       path_marker.colors[ii].r = rv;
       path_marker.colors[ii].b = bv;
       
     }
 
   // draw point-plane projections
   //int num_tracks = sba.tracks.size();
   //int ii = camera_marker.points.size();
 
 /*
   for (int i=0; i < num_tracks; i++)
     {
       const ProjMap &prjs = sba.tracks[i].projections;
       for (ProjMap::const_iterator itr = prjs.begin(); itr != prjs.end(); itr++)
         {
           const Proj &prj = (*itr).second;
           if (prj.pointPlane)   // have a ptp projection
             {
               camera_marker.points.resize(ii+2);
               sba::Point pt0 = sba.tracks[i].point;
               Vector3d plane_point = prj.plane_point;
               Vector3d plane_normal = prj.plane_normal;
               Eigen::Vector3d w = pt0.head<3>()-plane_point;
               //              Eigen::Vector3d projpt = plane_point+(w.dot(plane_normal))*plane_normal;
               Eigen::Vector3d projpt = pt0.head<3>() - (w.dot(plane_normal))*plane_normal;
               //              Vector3d pt1 = pt0.head<3>()+0.1*plane_normal;
               Vector3d pt1 = projpt;
                   
               camera_marker.points[ii].x = pt0.z();
               camera_marker.points[ii].y = -pt0.x();
               camera_marker.points[ii++].z = -pt0.y();
               camera_marker.points[ii].x = pt1.z();
               camera_marker.points[ii].y = -pt1.x();
               camera_marker.points[ii++].z = -pt1.y();
             }
         } 
     }
 */
 
   path_pub.publish(path_marker);
   camera_pub.publish(camera_marker);
   point_pub.publish(point_marker);
 }

void finishTracks(vector<featureTrack>& tracks, vector<cv::Point2f>& pts, double retainProp, unsigned int maxOccurrences) {
	
	if (maxOccurrences == 0) {
		return;
	}
	
	if (tracks.size() == 0) {
		return;
	}
	
	if (pts.size() == 0) {
		return;
	}
	
	vector<unsigned char> invalidFlags;
	
	for (unsigned int jjj = 0; jjj < tracks.size(); jjj++) {
		
		if (tracks.at(jjj).locations.size() >= maxOccurrences) {
			
			for (unsigned int iii = 0; iii < pts.size(); iii++) {
		
				cout << pts.at(iii) << endl;
				cout << tracks.at(jjj).locations.size() << endl;
				cout << tracks.at(jjj).locations.at(tracks.at(jjj).locations.size()-1).featureCoord.x << endl;
				
				if (pts.at(iii) == tracks.at(jjj).locations.at(tracks.at(jjj).locations.size()-1).featureCoord) {
					printf("%s << Found feature! (%d, %d)\n", __FUNCTION__, iii, jjj);
					
					invalidFlags.push_back(iii);
					
					
				}
				
			}
		}
		
	}
	
	if (invalidFlags.size() == 0) {
		return;
	}
	
	double origSize = ((double) pts.size());
	
	while (((double) pts.size()) > (retainProp * origSize)) {
		
		unsigned int index = rand() % invalidFlags.size();
		
		pts.erase(pts.begin() + invalidFlags.at(index));
		invalidFlags.erase(invalidFlags.begin() + index);
		
	}

	
}

#endif
#endif