/*! \file	stereo.cpp
 *  \brief	Definitions for depth from stereo related functions.
*/

#include "streamer/stereo.hpp"

void depthFromContours(vector<vector<vector<cv::Point> > >& c1, vector<vector<vector<cv::Point> > >& c2, cv::Mat& disp) {
	/*
	for (unsigned int iii = 0; iii < c1.size(); iii++) {
		
	}
	
	
	for (unsigned int iii = 0; iii < disp.rows; iii++) {
		
	}
	*/
}

void getContours(const cv::Mat& src, vector<vector<vector<cv::Point> > >& contours) {
	
	contours.clear();
	
	cv::Mat blurredIm, workingIm;
	
	GaussianBlur(src, blurredIm, cv::Size(25,25), 7.0, 7.0);
	
	for (unsigned int iii = 0; iii < 244; iii++) {
		
		#ifdef _OPENCV_VERSION_3_PLUS_
		cv::threshold(blurredIm, workingIm, iii, 255, cv::THRESH_BINARY);
		#else
		cv::threshold(blurredIm, workingIm, iii, 255, CV_THRESH_BINARY);
		#endif

		vector<vector<cv::Point> > c;
		
		// CV_RETR_TREE - can use this for heirarchical tree...
		
		#ifdef _OPENCV_VERSION_3_PLUS_
		cv::findContours( workingIm, c, cv::RETR_LIST, cv::CHAIN_APPROX_NONE );
		#else
		cv::findContours( workingIm, c, CV_RETR_LIST, CV_CHAIN_APPROX_NONE );
		#endif

		contours.push_back(c);
		
	}
}

void drawContours(const cv::Mat& src, cv::Mat& dst) {
	
	double minVal, maxVal;
	
	
	
	// http://stackoverflow.com/questions/8449378/finding-contours-in-opencv
	
	cv::Mat blurredIm, workingIm, contourImage(src.size(), CV_8UC3, cv::Scalar(0,0,0));
	
	cv::Scalar colors[3];
	colors[0] = cv::Scalar(255, 0, 0);
	colors[1] = cv::Scalar(0, 255, 0);
	colors[2] = cv::Scalar(0, 0, 255);
	
	GaussianBlur(src, blurredIm, cv::Size(25,25), 7.0, 7.0);
	
	minMaxLoc(blurredIm, &minVal, &maxVal);
	
	//dst = cv::Mat::zeros(src.size(), CV_8UC3);
	
	printf("%s << min/max = (%f / %f)\n", __FUNCTION__, minVal, maxVal);
	
	for (unsigned int iii = (unsigned int)(minVal); iii < (unsigned int)(minVal); iii++) {
		
		#ifdef _OPENCV_VERSION_3_PLUS_
		cv::threshold(blurredIm, workingIm, iii, 255, cv::THRESH_BINARY);
		#else
		cv::threshold(blurredIm, workingIm, iii, 255, CV_THRESH_BINARY);
		#endif
		
		std::vector<std::vector<cv::Point> > contours;
		cv::Mat contourOutput = workingIm.clone();
		#ifdef _OPENCV_VERSION_3_PLUS_
		cv::findContours( contourOutput, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE );
		#else
		cv::findContours( contourOutput, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE );
		#endif
		
		
		//Draw the contours
		
		for (size_t idx = 0; idx < contours.size(); idx++) {
			cv::drawContours(contourImage, contours, int(idx), colors[idx % 3]);
		}
		
		
	}
	
	imshow("workingIm", contourImage);
	cv::waitKey(1);
	
}

double findBestAlpha(const cv::Mat& K1, const cv::Mat& K2, const cv::Mat& coeff1, const cv::Mat& coeff2, const cv::Size& camSize, const cv::Mat& R0, const cv::Mat& R1) {

	cv::Mat newCamMat_1, newCamMat_2;
	
	cv::Rect roi1, roi2;
	
	double alpha = 0.00;
	bool centerPrincipalPoint = true;
	
	cv::Mat map11, map12, map21, map22;
	
	unsigned int protectionCounter = 0;
	
	while (1) {
		
		// printf("%s << alpha = (%f)\n", __FUNCTION__, alpha);
		
		cv::Mat white = cv::Mat::ones(camSize, CV_8UC1);
		cv::Mat mapped_1, mapped_2;
		
		newCamMat_1 = cv::getOptimalNewCameraMatrix(K1, coeff1, camSize, alpha, camSize, &roi1, centerPrincipalPoint);
		newCamMat_2 = cv::getOptimalNewCameraMatrix(K2, coeff2, camSize, alpha, camSize, &roi2, centerPrincipalPoint);
		
		cv::initUndistortRectifyMap(K1, coeff1, R0, newCamMat_1, camSize, CV_32FC1, map11, map12);
		cv::initUndistortRectifyMap(K2, coeff2, R1, newCamMat_2, camSize, CV_32FC1, map21, map22);
		
		double minVal, maxVal;
		
		remap(white, mapped_1, map11, map12, cv::INTER_LINEAR, cv::BORDER_CONSTANT, 0);
		
		minMaxLoc(mapped_1, &minVal, &maxVal);
		
		if (minVal == 1.0) {
			remap(white, mapped_2, map11, map12, cv::INTER_LINEAR, cv::BORDER_CONSTANT, 0);
			minMaxLoc(mapped_1, &minVal, &maxVal);
			
			if (minVal == 1.0) {
				break;
			}
		}
		
		if (protectionCounter == 100) {
			alpha = 0.0;
			break;
		}
		
		alpha -= 0.01;
		
	}

	return alpha;
	
}

void drawEpipolarLines(cv::Mat& im1, cv::Mat& im2, cv::Mat& F) {
	
	vector<cv::Point2f> inputPoints;
	vector<cv::Point3f> outputLines[2];

	for (unsigned int iii = 0; iii < 8; iii++) {
		inputPoints.push_back(cv::Point2f(320.0, float(60.0*double(iii)+30.0)));
	}
	
	computeCorrespondEpilines(inputPoints, 1, F, outputLines[0]);
	computeCorrespondEpilines(inputPoints, 2, F, outputLines[1]);
	
	for (unsigned int iii = 0; iii < inputPoints.size(); iii++) {
		
		for (unsigned int jjj = 0; jjj < 2; jjj++) {
			if (outputLines[jjj].at(iii).y != 0.0) {
				
				cv::Point2f start, end;
			
				start.x = 0.0;
				start.y = -outputLines[jjj].at(iii).z / outputLines[jjj].at(iii).y;
				
				end.x = 640.0;
				end.y = (-outputLines[jjj].at(iii).z - outputLines[jjj].at(iii).x*end.x) / outputLines[jjj].at(iii).y;
				
				start *= 16.0;
				end *= 16.0;
				
				if (jjj == 0) {
					#ifdef _OPENCV_VERSION_3_PLUS_
					line(im2, start, end, cv::Scalar(0,0,255), 1, cv::LINE_AA, 4);
					#else
					line(im2, start, end, cv::Scalar(0,0,255), 1, CV_AA, 4);
					#endif
				} else {
					#ifdef _OPENCV_VERSION_3_PLUS_
					line(im1, start, end, cv::Scalar(0,0,255), 1, cv::LINE_AA, 4);
					#else
					line(im1, start, end, cv::Scalar(0,0,255), 1, CV_AA, 4);
					#endif
				}
				
				
			}
		}
		
		
		
	}
	
}

bool relevelImages(const cv::Mat& im1, const cv::Mat& im2, const cv::Mat& old_depth, const cv::Mat& R1, const cv::Mat& R2, const cv::Mat& t, const cv::Mat& Q1, const cv::Mat& Q2, cv::Mat& pim1, cv::Mat& pim2, const cv::Mat& map11, const cv::Mat& map12, const cv::Mat& map21, const cv::Mat& map22) {
	
	cv::Mat _8bit1, _8bit2, mapped1, mapped2;
	
	double minVal_1, maxVal_1, minVal_2, maxVal_2, minVal, maxVal;
		
	minMaxLoc(im1, &minVal_1, &maxVal_1);
	minMaxLoc(im2, &minVal_2, &maxVal_2);
	
	// ROS_WARN("minVal_(1/2) = (%f / %f); maxVal_(1/2) = (%f / %f)", minVal_1, minVal_2, maxVal_1, maxVal_2);
	
	minVal = min(minVal_1, minVal_2);
	maxVal = max(maxVal_1, maxVal_2);
	
	// ROS_WARN("minVal = (%f); maxVal = (%f)", minVal, maxVal);
	
	if (old_depth.rows == 0) {
		
		adaptiveDownsample(im1, _8bit1);
		adaptiveDownsample(im2, _8bit2);
		
		remap(_8bit1, mapped1, map11, map12, cv::INTER_LINEAR);
		GaussianBlur(mapped1, pim1, cv::Size(7,7), 0.5, 0.5);
		
		remap(_8bit2, mapped2, map21, map22, cv::INTER_LINEAR);
		GaussianBlur(mapped2, pim2, cv::Size(7,7), 0.5, 0.5);
		
	} else {
		
		printf("%s << Using existing depth map to radiometrically calibrate...\n", __FUNCTION__);
		
		cv::Mat mapped16_1, mapped16_2;
		remap(im1, mapped16_1, map11, map12, cv::INTER_LINEAR);
		remap(im2, mapped16_2, map21, map22, cv::INTER_LINEAR);
		
		double grad, shift;
		
		findRadiometricMapping2(mapped16_1, mapped16_2, old_depth, R1, R2, t, Q1, Q2, grad, shift);
		
		applyIntensityShift(im1, _8bit1, im2, _8bit2, grad, shift);
		
		/*
		adaptiveDownsample(im1, _8bit1);
		adaptiveDownsample(im2, _8bit2);
		*/
		
		remap(_8bit1, mapped1, map11, map12, cv::INTER_LINEAR);
		GaussianBlur(mapped1, pim1, cv::Size(7,7), 0.5, 0.5);
		
		remap(_8bit2, mapped2, map21, map22, cv::INTER_LINEAR);
		GaussianBlur(mapped2, pim2, cv::Size(7,7), 0.5, 0.5);
		
	}
	
	return true;

}

double fitFixedLine(vector<unsigned short>& x, vector<unsigned short>& y, double grad) {

	double shift;
		
	// Find min and max values...
	
	if (x.size() == 0) {
		return 0.0;
	}
	
	unsigned short minVal = x.at(0);
	unsigned short maxVal = x.at(0);
	
	for (unsigned int iii = 0; iii < x.size(); iii++) {
		
		if (x.at(iii) < minVal) {
			minVal = x.at(iii);
		}
		
		if (x.at(iii) > maxVal) {
			maxVal = x.at(iii);
		}
		
		if (y.at(iii) < minVal) {
			minVal = y.at(iii);
		}
		
		if (y.at(iii) > maxVal) {
			maxVal = y.at(iii);
		}
		
		
	}
	
	printf("%s << minVal = (%d); maxVal = (%d)\n", __FUNCTION__, minVal, maxVal);
	
	
	double bestError = std::numeric_limits<double>::max();
	unsigned int bestCount = 0;
	
	//for (double t_shift = -abs(maxVal-minVal); t_shift < abs(maxVal-minVal); t_shift += 0.5) {
		
	for (double t_shift = -500; t_shift < 500; t_shift += 0.5) {
		
		double totalError = 0.0;
		unsigned int totalCount = 0;
		
		for (unsigned int iii = 0; iii < x.size(); iii++) {
			
			double err = abs(x.at(iii) - (grad * double(y.at(iii)) + t_shift));
			totalError += err / double(x.size());
			
			if (err < 5.0) {
				totalCount++;
			}
			
		}
		
		if (totalError < bestError) {
			//printf("%s << totalError with (%f) is (%f)\n", __FUNCTION__, t_shift, totalError);
			bestError = totalError;
			//shift = t_shift;
		}
		
		if (totalCount > bestCount) {
			bestCount = totalCount;
			shift = t_shift;
			//printf("%s << best count so far of (%d) with shift of (%f)\n", __FUNCTION__, bestCount, shift);
		}
		
	}
	
	return shift;
	
}

void findRadiometricMapping2(const cv::Mat& im1, const cv::Mat& im2, const cv::Mat& depth, const cv::Mat& R1, const cv::Mat& R2, const cv::Mat& t, const cv::Mat& Q1, const cv::Mat& Q2, double& grad, double& shift) {
	
	vector<cv::Point> pts[2];
	vector<unsigned short> intensities[2];
	
	unsigned short val;
	float f_val;
	
	cv::Mat xyz;
	reprojectImageTo3D(depth, xyz, Q1, true);
	
	//printf("%s << xyz.size() = (%d, %d)\n", __FUNCTION__, xyz.rows, xyz.cols);
	// saveXYZ(point_cloud_filename, xyz);
	
	/*
	cv::Mat debug1, debug2, debug1_, debug2_;
	
	adaptiveDownsample(im1, debug1);
	adaptiveDownsample(im2, debug2);

	cvtColor(debug1, debug1_, CV_GRAY2RGB);
	cvtColor(debug2, debug2_, CV_GRAY2RGB);
	*/
	
	for (int iii = 0; iii < depth.rows; iii += 1) {
		for (int jjj = 0; jjj < depth.cols; jjj += 1) {
			
			val = depth.at<unsigned short>(iii,jjj);
			
			f_val = xyz.at<cv::Vec3f>(iii,jjj)[2];
			
			#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
				if ( (f_val != 10000) && (f_val < FLT_MAX) && (f_val > FLT_MIN) ) {
			#else
				if ((f_val != 10000) && (!isinf(f_val))) { // ((f_val < FLT_MAX) && (f_val > FLT_MIN) && (f_val != 10000)) {
			#endif
			
				//printf("%s << xyz(%d, %d) = (%f, %f, %f)\n", __FUNCTION__, iii, jjj, xyz.at<cv::Vec3f>(iii,jjj)[0], xyz.at<cv::Vec3f>(iii,jjj)[1], xyz.at<cv::Vec3f>(iii,jjj)[2]);
				
				// Get x, y for camera (1) back...
				
				// [x y d 1]' = Q.inv() * [X Y Z W]'
				// [x/W y/W d/W 1/W]' = Q.inv() * [3d.x 3d.y 3d.z 1]';
				
				
				cv::Mat _3dLoc = cv::Mat::zeros(4, 1, CV_64FC1);
				_3dLoc.at<double>(0,0) = xyz.at<cv::Vec3f>(iii,jjj)[0];
				_3dLoc.at<double>(1,0) = xyz.at<cv::Vec3f>(iii,jjj)[1];
				_3dLoc.at<double>(2,0) = xyz.at<cv::Vec3f>(iii,jjj)[2];
				_3dLoc.at<double>(3,0) = 1.0;
				
				cv::Mat coords_1 = Q1.inv() * _3dLoc;
				cv::Mat coords_2 = Q2.inv() * _3dLoc;
				
				cv::Point reproj_1, reproj_2;
				unsigned short redist_1, redist_2;
				
				if ((coords_1.at<double>(3,0) != 0.0) && (coords_2.at<double>(3,0) != 0.0)) {
					
					reproj_1.y = int(coords_1.at<double>(1,0) / coords_1.at<double>(3,0));
					reproj_1.x = int(coords_1.at<double>(0,0) / coords_1.at<double>(3,0));
					redist_1 = int(coords_1.at<double>(2,0) / coords_1.at<double>(3,0));
					
					reproj_2.y = int(coords_2.at<double>(1,0) / coords_2.at<double>(3,0));
					reproj_2.x = int(coords_2.at<double>(0,0) / coords_2.at<double>(3,0));
					redist_2 = int(coords_2.at<double>(2,0) / coords_2.at<double>(3,0));
					
					//printf("%s << reproj_1 = (%d, %d, %d) vs pt = (%d, %d, %d)\n", __FUNCTION__, reproj_1.x, reproj_1.y, redist_1, iii, jjj, val);
					//printf("%s << reproj_2 = (%d, %d, %d)\n", __FUNCTION__, reproj_2.x, reproj_2.y, redist_2);
					
					
					
					//circle(debug1_, reproj_1, 1, cv::Scalar(255, 0, 0), 1, CV_AA, 0);
					//circle(debug2_, reproj_2, 1, cv::Scalar(255, 0, 0), 1, CV_AA, 0);
					
					
					
					// perspectiveTransform(InputArray src, OutputArray dst, InputArray m);
					vector<cv::Point3d> inputLoc;
					inputLoc.push_back(cv::Point3d(_3dLoc.at<double>(0,0), _3dLoc.at<double>(1,0), _3dLoc.at<double>(2,0)));
					vector<cv::Point3d> outputLoc;
					//perspectiveTransform(inputLoc, outputLoc, Q1.inv());
					perspectiveTransform(inputLoc, outputLoc, Q2.inv());
					//printf("%s << reproj_2b = (%f, %f, %f)\n", __FUNCTION__, outputLoc.at(0).x, outputLoc.at(0).y, outputLoc.at(0).z);
					
				}
				
				
				
				//printf("%s << coord = (%d, %d, %d) vs pt = (%d, %d, %d)\n", __FUNCTION__, coords.at<double>(0,0), coords.at<double>(1,0), coords.at<double>(2,0), coords.at<double>(3,0), iii, jjj);
				
				int buffer = 40;
				
				if ((reproj_1.y > buffer) && (reproj_1.y < 480-buffer) && (reproj_1.x > buffer) && (reproj_1.x < 640-buffer)) {
					if ((reproj_2.y > buffer) && (reproj_2.y < 480-buffer) && (reproj_2.x > buffer) && (reproj_2.x < 640-buffer)) {
						
						if ((im1.at<unsigned short>(reproj_1.y, reproj_1.x) != 0) && (im2.at<unsigned short>(reproj_2.y, reproj_2.x) != 0)) {
							intensities[0].push_back(im1.at<unsigned short>(reproj_1.y, reproj_1.x));
							//intensities[1].push_back(im2.at<unsigned short>(reproj_2.y, reproj_2.x));
							intensities[1].push_back(im2.at<unsigned short>(reproj_2.y, reproj_2.x));
						}
						
					}
				}
				
				
			}
			
		}
	}
	
	shift = fitFixedLine(intensities[0], intensities[1], 1.0);
	
	cv::Mat dispMat;
	
	double percentileVals[1] = { 0.500 };
	double intensityVals[2][1];
	findPercentiles(im1, intensityVals[0], percentileVals, 1);
	findPercentiles(im2, intensityVals[1], percentileVals, 1);
	
	
	
	
	
	/*
	imshow("debug_1", debug1_);
	cv::waitKey(1);
	imshow("debug_2", debug2_);
	cv::waitKey(1);
	*/
	
	printf("%s << intensities[0].size() = %lu\n", __FUNCTION__, intensities[0].size());
	
	// Simple median-based equation estimation:
	grad = 1.00;
	//shift = (intensityVals[0][0] - intensityVals[1][0]);
	
	plotPoints(dispMat, intensities[0], intensities[1], (unsigned short)(intensityVals[0][0]), (unsigned short)(intensityVals[1][0]), grad, shift);
	
	imshow("intensities", dispMat);
	cv::waitKey(1);
	
	printf("%s << Estimated equation = (%f) * m + (%f)\n", __FUNCTION__, grad, shift);
	
}

void plotPoints(cv::Mat& dispMat, vector<unsigned short>& i1, vector<unsigned short>& i2, unsigned short median_1, unsigned short median_2, double grad, double shift) {

	dispMat = cv::Mat::zeros(480, 640, CV_8UC3);
	
	for (int iii = 0; iii < dispMat.rows; iii++) {
		for (int jjj = 0; jjj < dispMat.cols; jjj++) {
			
			dispMat.at<cv::Vec3b>(iii,jjj)[0] = 255;
			dispMat.at<cv::Vec3b>(iii,jjj)[1] = 255;
			dispMat.at<cv::Vec3b>(iii,jjj)[2] = 255;
			
		}
	}
	
	if (i1.size() == 0) {
		return;
	}
	
	unsigned short minVal_1 = i1.at(0);
	unsigned short maxVal_1 = i1.at(0);
	
	unsigned short minVal_2 = i2.at(0);
	unsigned short maxVal_2 = i2.at(0);
	
	for (unsigned int iii = 0; iii < i1.size(); iii++) {
		
		if (i1.at(iii) < minVal_1) {
			minVal_1 = i1.at(iii);
		}
		
		if (i2.at(iii) < minVal_2) {
			minVal_2 = i2.at(iii);
		}
		
		if (i1.at(iii) > maxVal_1) {
			maxVal_1 = i1.at(iii);
		}
		
		if (i2.at(iii) > maxVal_2) {
			maxVal_2 = i2.at(iii);
		}
		
	}
	
	//minVal = 7000;
	//maxVal = 9000;
	
	printf("%s << minVal/maxVal = (%d, %d) : (%d, %d)\n", __FUNCTION__, minVal_1, maxVal_1, minVal_2, maxVal_2);
	
	
	
	int row, col;
	
	for (unsigned int iii = 0; iii < i1.size(); iii++) {
		

		row = int(480.0 * (double(i1.at(iii) - minVal_1) / double(maxVal_1 - minVal_1)));
		col = int(640.0 * (double(i2.at(iii) - minVal_2) / double(maxVal_2 - minVal_2)));

		if (iii == 50) {
			//printf("%s << row = (%d); col = (%d)\n", __FUNCTION__, row, col);
			//printf("%s << i1 = (%d); i2 = (%d)\n", __FUNCTION__, i1.at(iii), i2.at(iii));
		}
		
		#ifdef _OPENCV_VERSION_3_PLUS_
		circle(dispMat, cv::Point(col,row), 1, cv::Scalar(255, 0, 0), 1, cv::LINE_AA, 0);
		#else
		circle(dispMat, cv::Point(col,row), 1, cv::Scalar(255, 0, 0), 1, CV_AA, 0);
		#endif

		//dispMat.at<cv::Vec3b>(row,col)[0] = 0;
		//dispMat.at<cv::Vec3b>(row,col)[1] = 0;
		
	}
	
	// draw a line assuming equal gradient and a shift of the median difference
	if ((median_1 != 0) && (median_2 != 0)) {
		printf("%s << Median comparison = (%d vs %d)\n", __FUNCTION__, median_1, median_2);
	}
	
	int row_disp = int((median_2 - median_1) / 480.0);
	
	cv::Point p1, p2;
	
	unsigned int basic_pt_x = minVal_2;
	unsigned int basic_pt_y = (unsigned int)(grad * minVal_2 + shift);
	
	row = int(480.0 * (double(basic_pt_y - minVal_1) / double(maxVal_1 - minVal_1)));
	col = int(640.0 * (double(basic_pt_x - minVal_2) / double(maxVal_2 - minVal_2)));
	
	p1 = cv::Point(col,row);
	
	unsigned int new_pt_x = maxVal_2;
	unsigned int new_pt_y = (unsigned int)(grad * maxVal_2 + shift);
	
	row = int(480.0 * (double(new_pt_y - minVal_1) / double(maxVal_1 - minVal_1)));
	col = int(640.0 * (double(new_pt_x - minVal_2) / double(maxVal_2 - minVal_2)));
	
	p2 = cv::Point(col,row);
	
	printf("%s << Drawing line from (%d, %d) to (%d, %d)\n", __FUNCTION__, p1.x, p1.y, p2.x, p2.y);
	
	#ifdef _OPENCV_VERSION_3_PLUS_
	line(dispMat, p1, p2, cv::Scalar(0,0,255), 2, cv::LINE_AA, 0);
	#else
	line(dispMat, p1, p2, cv::Scalar(0,0,255), 2, CV_AA, 0);
	#endif

}
		
/*
bool findRadiometricMapping(const Mat& im1, const Mat& im2, double& grad, double& shift, const Mat& pim1, const Mat& pim2) {
	
	Ptr<FeatureDetector> detector;
    detector = new SurfFeatureDetector( 5.00 );

    vector<KeyPoint> kp1, kp2;
    detector -> detect(pim1, kp1);
    detector -> detect(pim2, kp2);
    
    Ptr<DescriptorExtractor> extractor;
    extractor = DescriptorExtractor::create( "SURF" );
    
    Mat d1, d2;
	extractor->compute(pim1, kp1, d1);
	extractor->compute(pim2, kp2, d2);
	
	Ptr<DescriptorMatcher> descriptorMatcher;
    descriptorMatcher = DescriptorMatcher::create("BruteForce");
    double ransacReprojThreshold = 1.0;
    
    vector<DMatch> filteredMatches;

	crossCheckMatching( descriptorMatcher, d1, d2, filteredMatches, 1 );
	
	Mat drawImg;

	int matchesFlag = DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS;
	
	

	
	
	vector<int> queryIdxs( filteredMatches.size() ), trainIdxs( filteredMatches.size() );
	for( size_t i = 0; i < filteredMatches.size(); i++ )
	{
		queryIdxs[i] = filteredMatches[i].queryIdx;
		trainIdxs[i] = filteredMatches[i].trainIdx;
	}
										
	vector<Point2f> pts1, pts2;
	
	KeyPoint::convert(kp1, pts1, queryIdxs);
	KeyPoint::convert(kp2, pts2, trainIdxs);
	
	printf("%s << filteredMatches.size() = (%d)\n", __FUNCTION__, filteredMatches.size());
	// printf("%s << pts1.size() = (%d)\n", __FUNCTION__, pts1.size());
    
    
    
	
	//int validMatches = countNonZero(validityMask);
	//cout << "validMatches = " << validMatches << endl;
	
	// Can make more efficient by implementing the threshold at the matching stage
	double thresh = 15.0;
	
	for (unsigned int iii = 0; iii < pts1.size(); iii++) {
		
		if (abs(pts1.at(iii).y - pts2.at(iii).y) > thresh) {
			pts1.erase(pts1.begin() + iii);
			pts2.erase(pts2.begin() + iii);
			iii--;
		} 
		
	}
	
	//displayKeypoints(pim1, pts1, drawImg, cv::Scalar(255, 0, 0));
	//displayKeypoints(drawImg, pts2, drawImg, cv::Scalar(0, 0, 255));
	
	showMatches(pim1, pts1, pim2, pts2, drawImg);
	
	// drawMatches( pim1, kp1, pim2, kp2, filteredMatches, drawImg, cv::Scalar(0, 0, 255), cv::Scalar(255, 0, 0), vector<char>(), matchesFlag );
	
	imshow("display", drawImg);
	cv::waitKey(1);
	
	vector<Point2f> sampleValues;
	
	for (unsigned int iii = 0; iii < pts1.size(); iii++) {
		
		Point coord_1(pts1.at(iii).x, pts1.at(iii).y);
		Point coord_2(pts2.at(iii).x, pts2.at(iii).y);
		
		Point2f sample;
		
		if ((coord_1.x >= 0) && (coord_1.y >= 0) && (coord_1.x < 640) && (coord_1.y < 480)) {
			
			if ((coord_2.x >= 0) && (coord_2.y >= 0) && (coord_2.x < 640) && (coord_2.y < 480)) {
				
				//printf("%s << coord_1 = (%d, %d); coord_2 = (%d, %d)\n", __FUNCTION__, coord_1.x, coord_1.y, coord_2.x, coord_2.y);
				
				sample.x = float(im1.at<unsigned short>(coord_1.y, coord_1.x));
				sample.y = float(im2.at<unsigned short>(coord_2.y, coord_2.x));
				
				//printf("%s << sample = (%f, %f)\n", __FUNCTION__, sample.x, sample.y);
				
				sampleValues.push_back(sample);
				
			}
			
			
		}
		
		
		
	}
	
	Mat samplePlot = Mat::ones(480, 640, CV_8UC1);
	samplePlot *= 255;
}
*/
