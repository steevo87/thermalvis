/*! \file	cloudproc.cpp
 *  \brief	Definitions for cloud processing.
*/

#ifdef _USE_PCL_

#include "slam/cloudproc.hpp"

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

void findThermalCloudPercentiles(pcl::PointCloud<pcl::RGB>::Ptr& thermalCloud, double *vals, double *percentiles, unsigned int num) {

	// img -> pcl::PointCloud<pcl::RGB>::Ptr& thermalCloud

	cv::Mat mx;
	
	unsigned int validPoints = 0;
	
	for (unsigned int i = 0; i < thermalCloud->size(); i++) {
		if (thermalCloud->points[i].b != 0) {
			validPoints++;
		}
	}
	
	//printf("%s << validPoints = (%d)\n", __FUNCTION__, validPoints);
	
	
	cv::Mat img = cv::Mat::zeros(1, validPoints, CV_16UC1);
	
	unsigned int validPointsIndex = 0;
	
	for (unsigned int i = 0; i < thermalCloud->size(); i++) {
		if (thermalCloud->points[i].b != 0) {
			img.at<unsigned short>(0,validPointsIndex) = 256*thermalCloud->points[i].r + thermalCloud->points[i].g;
			//printf("%s << Assigning value to (%d) of = (%d)\n", __FUNCTION__, validPointsIndex, img.at<unsigned short>(0,validPointsIndex));
			validPointsIndex++;
			
		}
	}
	
	//double currMin, currMax;
	//minMaxLoc(img, &currMin, &currMax);
	//printf("%s << min/max = (%f, %f)\n", __FUNCTION__, currMin, currMax);
	
	//printf("%s << Looking for (%f, %f) : (%d, %d)\n", __FUNCTION__, percentiles[0], percentiles[1], img.rows, img.cols);
	findPercentiles(img, vals, percentiles, num);
	
	for (unsigned int iii = 0; iii < num; iii++) {
		//printf("%s << Correcting from [%d](%f)\n", __FUNCTION__, iii, vals[iii]);
		vals[iii] = (vals[iii] - 1000.0) / 10.0;
		
	}
	

	
}

void cCloudScheme::confidence_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ioCloud, pcl::PointCloud<pcl::RGB>::Ptr& thermalCloud) {
	
	for (size_t i = 0; i < ioCloud->size (); ++i) {
		ioCloud->points[i].b = thermalCloud->points[i].b;
		ioCloud->points[i].g = thermalCloud->points[i].b;
		ioCloud->points[i].r = thermalCloud->points[i].b;
	}
}

void cCloudScheme::falsify_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ioCloud, pcl::PointCloud<pcl::RGB>::Ptr& thermalCloud, double minTemp, double maxTemp, int deadPointMode, double volumeSize, double minConfidence, bool clampOutliers) {

	//printf("%s << %f -> %f; (%d)\n", __FUNCTION__, minConfidence, (minConfidence * 255.0), clampOutliers);

	double currentTemp;
	int lookupIndex = 0;
	
	//printf("%s << %f -> %f\n", __FUNCTION__, minConfidence, (minConfidence * 255.0));

	for (size_t i = 0; i < ioCloud->size (); ++i) {
		
		//if (thermalCloud->points[i].b != 0) { printf("%s << (%d) (%f)\n", __FUNCTION__, thermalCloud->points[i].b, double(thermalCloud->points[i].b)); }
		
		
		
		currentTemp = ((double(thermalCloud->points[i].r)*256.0 + double(thermalCloud->points[i].g)) - 1000.0) / 10.0;
		
		bool thermalValid = (double(thermalCloud->points[i].b) >= (minConfidence * 255.0)) && !(!clampOutliers && ( (currentTemp < minTemp) || (currentTemp > maxTemp) ) );
		
		
		if (!thermalValid) {
			
			// printf("%s << Dead point.\n", __FUNCTION__);
					
			if (deadPointMode == DPM_GRAY) {
				ioCloud->points[i].b = 128;
				ioCloud->points[i].g = 128;
				ioCloud->points[i].r = 128;
			} else if (deadPointMode == DPM_AXES) {
				ioCloud->points[i].b = (unsigned char) (255.0 * (ioCloud->points[i].x / volumeSize));
				ioCloud->points[i].g = (unsigned char) (255.0 * (ioCloud->points[i].y / volumeSize));
				ioCloud->points[i].r = (unsigned char) (255.0 * (ioCloud->points[i].z / volumeSize));
			}
		} else {
			
			// use ( clampOutliers ) to decide what to do with temps outside the range
			// currentTemp = max(minTemp, min(maxTemp, ((double(thermalCloud->points[i].r)*256.0 + double(thermalCloud->points[i].g)) - 1000.0) / 10.0));
			
			currentTemp = max(minTemp, min(maxTemp, currentTemp));
			
			// must convert temp to look up index using min/max
			
			lookupIndex = 255.0 * ((currentTemp - minTemp) / (maxTemp - minTemp));
			
			if ((lookupIndex < 0) || (lookupIndex >= 65536)) {
				
				if (deadPointMode == DPM_GRAY) {
				ioCloud->points[i].b = 128;
				ioCloud->points[i].g = 128;
				ioCloud->points[i].r = 128;
			} else if (deadPointMode == DPM_AXES) {
				ioCloud->points[i].b = (unsigned char) (255.0 * (ioCloud->points[i].x / volumeSize));
				ioCloud->points[i].g = (unsigned char) (255.0 * (ioCloud->points[i].y / volumeSize));
				ioCloud->points[i].r = (unsigned char) (255.0 * (ioCloud->points[i].z / volumeSize));
			}
			
			continue;
				//printf("%s << currentTemp = (%f); lookupIndex = (%d)\n", __FUNCTION__, currentTemp, lookupIndex);
			} 
			
			//if (lookupIndex == 0) {
			//	printf("%s << currentTemp = (%f); lookupIndex = (%d)\n", __FUNCTION__, currentTemp, lookupIndex);
			//}
			
			ioCloud->points[i].b = lookupTable_1[lookupIndex][0];
			ioCloud->points[i].g = lookupTable_1[lookupIndex][1];
			ioCloud->points[i].r = lookupTable_1[lookupIndex][2];
			
			
			/*
			ioCloud->points[i].b = lookupIndex;
			ioCloud->points[i].g = 0;
			ioCloud->points[i].r = 0;
			*/
		}
		
	}

}

void cCloudScheme::falsify_cloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& ioCloud, pcl::PointCloud<pcl::RGB>::Ptr& thermalCloud, double minTemp, double maxTemp, int deadPointMode, double volumeSize, double minConfidence, bool clampOutliers) {

	double currentTemp;
	int lookupIndex = 0;
	
	

	for (size_t i = 0; i < ioCloud->size (); ++i) {
		
		//if (thermalCloud->points[i].b != 0) { printf("%s << (%d) (%f)\n", __FUNCTION__, thermalCloud->points[i].b, double(thermalCloud->points[i].b)); }
		
		
		
		currentTemp = ((double(thermalCloud->points[i].r)*256.0 + double(thermalCloud->points[i].g)) - 1000.0) / 10.0;
		
		bool thermalValid = (double(thermalCloud->points[i].b) >= (minConfidence * 255.0)) && !(!clampOutliers && ( (currentTemp < minTemp) || (currentTemp > maxTemp) ) );
		
		
		if (!thermalValid) {
			
			// printf("%s << Dead point.\n", __FUNCTION__);
					
			if (deadPointMode == DPM_GRAY) {
				ioCloud->points[i].b = 128;
				ioCloud->points[i].g = 128;
				ioCloud->points[i].r = 128;
			} else if (deadPointMode == DPM_AXES) {
				ioCloud->points[i].b = (unsigned char) (255.0 * (ioCloud->points[i].x / volumeSize));
				ioCloud->points[i].g = (unsigned char) (255.0 * (ioCloud->points[i].y / volumeSize));
				ioCloud->points[i].r = (unsigned char) (255.0 * (ioCloud->points[i].z / volumeSize));
			}
		} else {
			
			// use ( clampOutliers ) to decide what to do with temps outside the range
			// currentTemp = max(minTemp, min(maxTemp, ((double(thermalCloud->points[i].r)*256.0 + double(thermalCloud->points[i].g)) - 1000.0) / 10.0));
			
			currentTemp = max(minTemp, min(maxTemp, currentTemp));
			
			// must convert temp to look up index using min/max
			
			lookupIndex = 255.0 * ((currentTemp - minTemp) / (maxTemp - minTemp));
			
			
			//if (lookupIndex == 0) {
			//	printf("%s << currentTemp = (%f); lookupIndex = (%d)\n", __FUNCTION__, currentTemp, lookupIndex);
			//}
			
			ioCloud->points[i].b = lookupTable_1[lookupIndex][0];
			ioCloud->points[i].g = lookupTable_1[lookupIndex][1];
			ioCloud->points[i].r = lookupTable_1[lookupIndex][2];
			
			
			/*
			ioCloud->points[i].b = lookupIndex;
			ioCloud->points[i].g = 0;
			ioCloud->points[i].r = 0;
			*/
		}
		
	}

}


void cCloudScheme::fuse_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ioCloud, pcl::PointCloud<pcl::RGB>::Ptr& colorCloud, pcl::PointCloud<pcl::RGB>::Ptr& thermalCloud, double minTemp, double maxTemp, int deadPointMode, double volumeSize, double minConfidence, bool clampOutliers, double *params) {
	
	double currentTemp, visibleVal, lumChange;
	int lookupIndex = 0;
	
	double working_params[2];
	
	if (params == NULL) {
		working_params[0] = DEFAULT_LOWER_VISIBLE_FUSION_LIMIT;
		working_params[1] = DEFAULT_UPPER_VISIBLE_FUSION_LIMIT;
	} else {
		working_params[0] = params[0];
		working_params[1] = params[1];
	}

	for (size_t i = 0; i < ioCloud->size (); ++i) {
		
		visibleVal = 0.0;
				
		/*
		visibleVal += 0.299 * (((double) colorCloud->points[i].r) * working_params[0] + 255.0 * ((1.0 - working_params[1])/2.0));
		visibleVal += 0.587 * (((double) colorCloud->points[i].g) * working_params[0] + 255.0 * ((1.0 - working_params[1])/2.0));
		visibleVal += 0.114 * (((double) colorCloud->points[i].b) * working_params[0] + 255.0 * ((1.0 - working_params[1])/2.0));
		lumChange = 2.0 * (visibleVal/255.0 - 0.5);
		*/
		
		visibleVal += 0.299 * ((double) colorCloud->points[i].r);
		visibleVal += 0.587 * ((double) colorCloud->points[i].g);
		visibleVal += 0.114 * ((double) colorCloud->points[i].b);
		lumChange = 2.0 * ((visibleVal / 255.0) - 0.5) * (working_params[1] - working_params[0]);
		
		
		
		bool visibleValid = (colorCloud->points[i].r != 0) || (colorCloud->points[i].g != 0) || (colorCloud->points[i].b != 0);
		
		currentTemp = ((double(thermalCloud->points[i].r)*256.0 + double(thermalCloud->points[i].g)) - 1000.0) / 10.0;
		
		bool thermalValid = (double(thermalCloud->points[i].b) >= (minConfidence * 255.0)) && !(!clampOutliers && ( (currentTemp < minTemp) || (currentTemp > maxTemp) ) );
		
		currentTemp = max(minTemp, min(maxTemp, currentTemp));
		lookupIndex = 255.0 * ((currentTemp - minTemp) / (maxTemp - minTemp));
		
		// If thermal (whether or not RGB is valid)
		if (thermalValid) {
			ioCloud->points[i].b = lookupTable_1[lookupIndex][0];
			ioCloud->points[i].g = lookupTable_1[lookupIndex][1];
			ioCloud->points[i].r = lookupTable_1[lookupIndex][2];
		}
		
		// Both thermal and RGB
		if (visibleValid && thermalValid) {			
			
			if (lumChange > 0) {
				ioCloud->points[i].b = ioCloud->points[i].b + (255.0 - ((double)ioCloud->points[i].b)) * lumChange;
				ioCloud->points[i].g = ioCloud->points[i].g + (255.0 - ((double)ioCloud->points[i].g)) * lumChange;
				ioCloud->points[i].r = ioCloud->points[i].r + (255.0 - ((double)ioCloud->points[i].r)) * lumChange;
			}
			else {
				ioCloud->points[i].b = ioCloud->points[i].b + ((double)ioCloud->points[i].b) * lumChange;
				ioCloud->points[i].g = ioCloud->points[i].g + ((double)ioCloud->points[i].g) * lumChange;
				ioCloud->points[i].r = ioCloud->points[i].r + ((double)ioCloud->points[i].r) * lumChange;
			}
		}
		// RGB only (grayscale RGB)
		else if (visibleValid) {
			ioCloud->points[i].b = colorCloud->points[i].r;
			ioCloud->points[i].g = colorCloud->points[i].g;
			ioCloud->points[i].r = colorCloud->points[i].b;
			
		}
		// Neither (Saturation 0%, Lightness 50%)
		else if (!visibleValid & !thermalValid) {
			
			if (deadPointMode == DPM_GRAY) {
				ioCloud->points[i].b = 128;
				ioCloud->points[i].g = 128;
				ioCloud->points[i].r = 128;
			} else if (deadPointMode == DPM_AXES) {
				ioCloud->points[i].b = (unsigned char) (255.0 * (ioCloud->points[i].x / volumeSize));
				ioCloud->points[i].g = (unsigned char) (255.0 * (ioCloud->points[i].y / volumeSize));
				ioCloud->points[i].r = (unsigned char) (255.0 * (ioCloud->points[i].z / volumeSize));
			}
			
			
		}

	}
	
}

void cCloudScheme::fuse_cloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& ioCloud, pcl::PointCloud<pcl::RGB>::Ptr& colorCloud, pcl::PointCloud<pcl::RGB>::Ptr& thermalCloud, double minTemp, double maxTemp, int deadPointMode, double volumeSize, double minConfidence, bool clampOutliers, double *params) {
	
	double currentTemp, visibleVal, lumChange;
	int lookupIndex = 0;
	
	double working_params[2];
	
	if (params == NULL) {
		working_params[0] = DEFAULT_LOWER_VISIBLE_FUSION_LIMIT;
		working_params[1] = DEFAULT_UPPER_VISIBLE_FUSION_LIMIT;
	} else {
		working_params[0] = params[0];
		working_params[1] = params[1];
	}

	for (size_t i = 0; i < ioCloud->size (); ++i) {
		
		visibleVal = 0.0;
				
		/*
		visibleVal += 0.299 * (((double) colorCloud->points[i].r) * working_params[0] + 255.0 * ((1.0 - working_params[1])/2.0));
		visibleVal += 0.587 * (((double) colorCloud->points[i].g) * working_params[0] + 255.0 * ((1.0 - working_params[1])/2.0));
		visibleVal += 0.114 * (((double) colorCloud->points[i].b) * working_params[0] + 255.0 * ((1.0 - working_params[1])/2.0));
		lumChange = 2.0 * (visibleVal/255.0 - 0.5);
		*/
		
		visibleVal += 0.299 * ((double) colorCloud->points[i].r);
		visibleVal += 0.587 * ((double) colorCloud->points[i].g);
		visibleVal += 0.114 * ((double) colorCloud->points[i].b);
		lumChange = 2.0 * ((visibleVal / 255.0) - 0.5) * (working_params[1] - working_params[0]);
		
		
		
		bool visibleValid = (colorCloud->points[i].r != 0) || (colorCloud->points[i].g != 0) || (colorCloud->points[i].b != 0);
		
		currentTemp = ((double(thermalCloud->points[i].r)*256.0 + double(thermalCloud->points[i].g)) - 1000.0) / 10.0;
		
		bool thermalValid = (double(thermalCloud->points[i].b) >= (minConfidence * 255.0)) && !(!clampOutliers && ( (currentTemp < minTemp) || (currentTemp > maxTemp) ) );
		
		currentTemp = max(minTemp, min(maxTemp, currentTemp));
		lookupIndex = 255.0 * ((currentTemp - minTemp) / (maxTemp - minTemp));
		
		// If thermal (whether or not RGB is valid)
		if (thermalValid) {
			ioCloud->points[i].b = lookupTable_1[lookupIndex][0];
			ioCloud->points[i].g = lookupTable_1[lookupIndex][1];
			ioCloud->points[i].r = lookupTable_1[lookupIndex][2];
		}
		
		// Both thermal and RGB
		if (visibleValid && thermalValid) {			
			
			if (lumChange > 0) {
				ioCloud->points[i].b = ioCloud->points[i].b + (255.0 - ((double)ioCloud->points[i].b)) * lumChange;
				ioCloud->points[i].g = ioCloud->points[i].g + (255.0 - ((double)ioCloud->points[i].g)) * lumChange;
				ioCloud->points[i].r = ioCloud->points[i].r + (255.0 - ((double)ioCloud->points[i].r)) * lumChange;
			}
			else {
				ioCloud->points[i].b = ioCloud->points[i].b + ((double)ioCloud->points[i].b) * lumChange;
				ioCloud->points[i].g = ioCloud->points[i].g + ((double)ioCloud->points[i].g) * lumChange;
				ioCloud->points[i].r = ioCloud->points[i].r + ((double)ioCloud->points[i].r) * lumChange;
			}
		}
		// RGB only (grayscale RGB)
		else if (visibleValid) {
			ioCloud->points[i].b = colorCloud->points[i].r;
			ioCloud->points[i].g = colorCloud->points[i].g;
			ioCloud->points[i].r = colorCloud->points[i].b;
			
		}
		// Neither (Saturation 0%, Lightness 50%)
		else if (!visibleValid & !thermalValid) {
			
			if (deadPointMode == DPM_GRAY) {
				ioCloud->points[i].b = 128;
				ioCloud->points[i].g = 128;
				ioCloud->points[i].r = 128;
			} else if (deadPointMode == DPM_AXES) {
				ioCloud->points[i].b = (unsigned char) (255.0 * (ioCloud->points[i].x / volumeSize));
				ioCloud->points[i].g = (unsigned char) (255.0 * (ioCloud->points[i].y / volumeSize));
				ioCloud->points[i].r = (unsigned char) (255.0 * (ioCloud->points[i].z / volumeSize));
			}
			
			
		}

	}
	
}

#endif