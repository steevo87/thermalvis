/*! \file	radiometric.cpp
 *  \brief	Definitions for radiometric image processing.
*/
#include "radiometric.hpp"

rScheme::rScheme() {
	// ...
}

rScheme::rScheme(const cv::Mat &mM, const double &minT, const double &maxT, const double &minG, const double &maxG) {
	
	update(mM, minT, maxT, minG, maxG);
	
}

rScheme::~rScheme() { 
		
	mappingMatrix.release();
	
}

void rScheme::update(const cv::Mat &mM, const double &minT, const double &maxT, const double &minG, const double &maxG) {

	minTemp = float(minT);
	maxTemp = float(maxT);
	
	minGraylevel = float(minG);
	maxGraylevel = float(maxG);
	
	mM.copyTo(mappingMatrix);

}

void rScheme::apply(const cv::Mat& src, cv::Mat& dst, float thermistorTemp, float interpolateVal) {

	if (dst.rows == 0) {
		dst = cv::Mat::zeros(src.size(), CV_32FC1);
	}
	
	//printf("%s << mappingMatrix.size() = (%d,%d)\n", __FUNCTION__, mappingMatrix.rows, mappingMatrix.cols);
	
	// Find index for thermistorTemp:
	float thermistorIndex = ((std::max(std::min(thermistorTemp, maxTemp), minTemp) - minTemp) / (maxTemp - minTemp)) * float(mappingMatrix.cols - 1);
	
	//printf("%s << thermistor of (%f) out of (%f -> %f) gets index (%f)\n", __FUNCTION__, thermistorTemp, minTemp, maxTemp, thermistorIndex);
	
	float graylevelIndex;
	
	
	
	
	
	if (interpolateVal) {
		
		unsigned int tI[2], gI[2];
		tI[0] = (unsigned int) (floor(thermistorIndex));
		tI[1] = (unsigned int) (ceil(thermistorIndex));
		
		float v[4], d[4], final_val;
		
		/*
		unsigned int tX, gX;
		tX = (unsigned int) (floor(thermistorIndex + 0.5));
		*/
		
		for (int iii = 0; iii < src.rows; iii++) {
			for (int jjj = 0; jjj < src.cols; jjj++) {
				
				final_val = 0.0;
				
				graylevelIndex = ((std::max(std::min(float(src.at<unsigned short>(iii,jjj)), maxGraylevel), minGraylevel) - minGraylevel) / (maxGraylevel - minGraylevel)) * float(mappingMatrix.rows - 1);
				gI[0] = (unsigned int) (floor(graylevelIndex));
				gI[1] = (unsigned int) (ceil(graylevelIndex));
				
				v[0] = float(mappingMatrix.at<double>(gI[0], tI[0]));
				v[1] = float(mappingMatrix.at<double>(gI[0], tI[1]));
				v[2] = float(mappingMatrix.at<double>(gI[1], tI[0]));
				v[3] = float(mappingMatrix.at<double>(gI[1], tI[1]));
				
				d[0] = float(pow(abs(gI[0] - float(graylevelIndex)), 2.0) + pow(abs(tI[0] - float(thermistorIndex)), 2.0));
				d[1] = float(pow(abs(gI[0] - float(graylevelIndex)), 2.0) + pow(abs(tI[1] - float(thermistorIndex)), 2.0));
				d[2] = float(pow(abs(gI[1] - float(graylevelIndex)), 2.0) + pow(abs(tI[0] - float(thermistorIndex)), 2.0));
				d[3] = float(pow(abs(gI[1] - float(graylevelIndex)), 2.0) + pow(abs(tI[1] - float(thermistorIndex)), 2.0));
				
				if (d[0] == 0.0) {
					dst.at<float>(iii,jjj) = v[0];
				} else if (d[1] == 0.0) {
					dst.at<float>(iii,jjj) = v[1];
				} else if (d[2] == 0.0) {
					dst.at<float>(iii,jjj) = v[2];
				} else if (d[3] == 0.0) {
					dst.at<float>(iii,jjj) = v[3];
				} else {
					final_val += float(v[0] * 1.0 / d[0]);
					final_val += float(v[1] * 1.0 / d[1]);
					final_val += float(v[2] * 1.0 / d[2]);
					final_val += float(v[3] * 1.0 / d[3]);
					
					dst.at<float>(iii,jjj) = float(final_val / ((1.0/d[0]) + (1.0/d[1]) + (1.0/d[2]) + (1.0/d[3])));
				}

				/*
				
				graylevelIndex = ((std::max(std::min(float(src.at<unsigned short>(iii,jjj)), maxGraylevel), minGraylevel) - minGraylevel) / (maxGraylevel - minGraylevel)) * float(mappingMatrix.rows - 1);
				gX = (unsigned int) (floor(graylevelIndex + 0.5));
				
				if (((iii % 100) == 0) && ((jjj % 100) == 0)) {
					printf("%s << [%d,%d] << [%f] (%d, %d) [%f] (%d, %d)<< (%f,%f) & (%f,%f) & (%f,%f) & (%f,%f) : [%f]\n", __FUNCTION__, iii, jjj, graylevelIndex, gI[0], gI[1], thermistorIndex, tI[0], tI[1], v[0], d[0], v[1], d[1], v[2], d[2], v[3], d[3], dst.at<float>(iii,jjj));
					printf("interp vs NN = (%f) vs (%f)\n", __FUNCTION__, dst.at<float>(iii,jjj), mappingMatrix.at<double>(gX, tX));
					
				}
				*/

			}
		}
		
	} else {
		
		unsigned int tI, gI;
		tI = (unsigned int) (floor(thermistorIndex + 0.5));

	
		for (int iii = 0; iii < src.rows; iii++) {
			for (int jjj = 0; jjj < src.cols; jjj++) {
				
				graylevelIndex = ((std::max(std::min(float(src.at<unsigned short>(iii,jjj)), maxGraylevel), minGraylevel) - minGraylevel) / (maxGraylevel - minGraylevel)) * float(mappingMatrix.rows - 1);
				gI = (unsigned int) (floor(graylevelIndex + 0.5));
				dst.at<float>(iii,jjj) = float(mappingMatrix.at<double>(gI, tI));
				
			}
		}
	}
	
	
	
	// TEMP for testing
	/*
	double currMin = 7500, currMax = 9000;
	//minMaxLoc(src, &currMin, &currMax);
	
	for (unsigned int iii = 0; iii < src.rows; iii++) {
		for (unsigned int jjj = 0; jjj < src.cols; jjj++) {
			
			dst.at<float>(iii,jjj) = ((float(src.at<unsigned short>(iii,jjj)) - currMin) / (currMax - currMin)) * 30.0 + 20.0;
			
		}
	}
	
	double newMin, newMax;
	
	minMaxLoc(dst, &newMin, &newMax);
	*/
	
	// printf("%s << (%f, %f)\n", __FUNCTION__, newMin, newMax);

}

