/*! \file	annotation.cpp
 *  \brief	Definitions for drawing on images.
*/

#include "core/annotation.hpp"

void drawRichKeyPoints(const cv::Mat& src, std::vector<cv::KeyPoint>& kpts, cv::Mat& dst) {
	
	cv::Mat grayFrame;
	
	#ifdef _OPENCV_VERSION_3_PLUS_
	cvtColor(src, grayFrame, cv::COLOR_RGB2GRAY);
	#else
	cvtColor(src, grayFrame, CV_RGB2GRAY);
	#endif

	dst = cv::Mat::zeros(src.size(), src.type());
	
	if (kpts.size() == 0) return;
	
	std::vector<cv::KeyPoint> kpts_cpy, kpts_sorted;
	
	kpts_cpy.insert(kpts_cpy.end(), kpts.begin(), kpts.end());
	
	double maxResponse = kpts_cpy.at(0).response;
	double minResponse = kpts_cpy.at(0).response;
	
	while (kpts_cpy.size() > 0) {
		
		double maxR = 0.0;
		unsigned int idx = 0;
		
		for (unsigned int iii = 0; iii < kpts_cpy.size(); iii++) {
			
			if (kpts_cpy.at(iii).response > maxR) {
				maxR = kpts_cpy.at(iii).response;
				idx = iii;
			}
			if (kpts_cpy.at(iii).response > maxResponse) maxResponse = kpts_cpy.at(iii).response;
			if (kpts_cpy.at(iii).response < minResponse) minResponse = kpts_cpy.at(iii).response;
		}
		
		kpts_sorted.push_back(kpts_cpy.at(idx));
		kpts_cpy.erase(kpts_cpy.begin() + idx);
		
	}
	
	int thickness = 1;
	cv::Point center;
	cv::Scalar colour;
	int red = 0, blue = 0, green = 0;
	int radius;
	double normalizedScore;
	
	if (minResponse == maxResponse) colour = cv::Scalar(255, 0, 0);
	
	for (int iii = int(kpts_sorted.size())-1; iii >= 0; iii--) {

		if (minResponse != maxResponse) {
			normalizedScore = pow((kpts_sorted.at(iii).response - minResponse) / (maxResponse - minResponse), 0.25);
			red = int(255.0 * normalizedScore);
			green = int(255.0 - 255.0 * normalizedScore);
			colour = cv::Scalar(red, green, blue);
		}
		
		//center = kpts_sorted.at(iii).pt;
        center.x = int(kpts_sorted.at(iii).pt.x * 16.0);
        center.y = int(kpts_sorted.at(iii).pt.y * 16.0);
        
        radius = int(16.0 * (double(kpts_sorted.at(iii).size)/2.0));
        
        if (radius > 0) draw_circle(dst, center, radius, colour, -1, 4);
	}
	
	fadeImage(src, dst);
	//cvtColor(grayFrame, dst, CV_GRAY2RGB);
	
	for (int iii = int(kpts_sorted.size())-1; iii >= 0; iii--) {

		if (minResponse != maxResponse) {
			normalizedScore = pow((kpts_sorted.at(iii).response - minResponse) / (maxResponse - minResponse), 0.25);
			red = int(255.0 * normalizedScore);
			green = int(255.0 - 255.0 * normalizedScore);
			colour = cv::Scalar(red, green, blue);
		}
		
		center = kpts_sorted.at(iii).pt;
		center.x = int(kpts_sorted.at(iii).pt.x * 16.0);
        center.y = int(kpts_sorted.at(iii).pt.y * 16.0);
        
        radius = int(16.0 * (double(kpts_sorted.at(iii).size)/2.0));
        
        if (radius > 0) draw_circle(dst, center, radius, colour, -1, 4);
	}
}

void draw_circle(cv::Mat& img, cv::Point center, int radius, const cv::Scalar& color, int thickness, int shift) {
#ifdef _OPENCV_VERSION_3_PLUS_
	circle(img, center, radius, color, thickness, cv::LINE_AA, shift);
#else
	circle(img, center, radius, color, thickness, CV_AA, shift);
#endif
}

void showMatches(const cv::Mat& pim1, vector<cv::Point2f>& pts1, const cv::Mat& pim2, vector<cv::Point2f>& pts2, cv::Mat& drawImg) {

	drawImg = cv::Mat::zeros(pim1.rows, pim1.cols + pim2.cols, CV_8UC3);
	
	#pragma omp parallel for
	for (int iii = 0; iii < pim1.rows; iii++) {
		for (int jjj = 0; jjj < pim1.cols; jjj++) {
			
			drawImg.at<cv::Vec3b>(iii,jjj)[0] = pim1.at<unsigned char>(iii,jjj);
			drawImg.at<cv::Vec3b>(iii,jjj)[1] = pim1.at<unsigned char>(iii,jjj);
			drawImg.at<cv::Vec3b>(iii,jjj)[2] = pim1.at<unsigned char>(iii,jjj);
			
			drawImg.at<cv::Vec3b>(iii,640+jjj)[0] = pim2.at<unsigned char>(iii,jjj);
			drawImg.at<cv::Vec3b>(iii,640+jjj)[1] = pim2.at<unsigned char>(iii,jjj);
			drawImg.at<cv::Vec3b>(iii,640+jjj)[2] = pim2.at<unsigned char>(iii,jjj);
		}
	}
	
	cv::Point pt1, pt2;
	int radius = 4;
	int thickness = 1;
	
	radius *= 16;
	for (unsigned int iii = 0; iii < pts1.size(); iii++) {
		
		pt1.x = int(16.0 * pts1.at(iii).x);
		pt1.y = int(16.0 * pts1.at(iii).y);
		
		pt2.x = int(16.0 * (pts2.at(iii).x + 640.0));
		pt2.y = int(16.0 * pts2.at(iii).y);
		
		#ifdef _OPENCV_VERSION_3_PLUS_
		circle(drawImg, pt1, radius, cv::Scalar(255, 0, 0), thickness, cv::LINE_AA, 4);
		circle(drawImg, pt2, radius, cv::Scalar(255, 0, 0), thickness, cv::LINE_AA, 4);
		line(drawImg, pt1, pt2, cv::Scalar(0, 0, 255), thickness, cv::LINE_AA, 4);
		#else
		circle(drawImg, pt1, radius, cv::Scalar(255, 0, 0), thickness, CV_AA, 4);
		circle(drawImg, pt2, radius, cv::Scalar(255, 0, 0), thickness, CV_AA, 4);
		line(drawImg, pt1, pt2, cv::Scalar(0, 0, 255), thickness, CV_AA, 4);
		#endif
		
		
	}
	
}
