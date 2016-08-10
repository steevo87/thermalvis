/*! \file	improc.cpp
 *  \brief	Definitions for image processing.
*/

#include "core/improc.hpp"

#ifdef _USE_QT_
QImage Mat2QImage(const cv::Mat &src) {

    // http://stackoverflow.com/questions/5026965/how-to-convert-an-opencv-cvmat-to-qimage
    QImage dest= QImage((uchar*) src.data, int(src.cols), int(src.rows), int(src.step), QImage::Format_RGB888); // Format_ARGB32

    return dest;
}
#endif

void denoiseImage(const cv::Mat& src, cv::Mat& dst, int denoisingMode) {

	bool debugMode = false;

	src.copyTo(dst);

	/*
		PUT CODE HERE
	*/

	if (debugMode) {

		cv::Mat pre_Image, post_Image;

		adaptiveDownsample(src, pre_Image);
		adaptiveDownsample(dst, post_Image);

		cv::imshow("pre_im", pre_Image);
		cv::imshow("post_im", post_Image);
		cv::waitKey(1);
	}

}

void fadeImage(const cv::Mat& src, cv::Mat& dst, double frac) {
	
	#pragma omp parallel for
	for (int iii = 0; iii < src.rows; iii++) {
		for (int jjj = 0; jjj < src.cols; jjj++) {
			
			if ((src.at<cv::Vec3b>(iii,jjj)[0] != 0) && (src.at<cv::Vec3b>(iii,jjj)[1] != 0) && (src.at<cv::Vec3b>(iii,jjj)[2] != 0)) {
				for (unsigned int kkk = 0; kkk < 3; kkk++) {
					
					// check val first
					dst.at<cv::Vec3b>(iii,jjj)[0] = (unsigned char) (((1-frac) * double(src.at<cv::Vec3b>(iii,jjj)[0]) + frac * double(dst.at<cv::Vec3b>(iii,jjj)[0])) / 1.0);
					dst.at<cv::Vec3b>(iii,jjj)[1] = (unsigned char) (((1-frac) * double(src.at<cv::Vec3b>(iii,jjj)[1]) + frac * double(dst.at<cv::Vec3b>(iii,jjj)[1])) / 1.0);
					dst.at<cv::Vec3b>(iii,jjj)[2] = (unsigned char) (((1-frac) * double(src.at<cv::Vec3b>(iii,jjj)[2]) + frac * double(dst.at<cv::Vec3b>(iii,jjj)[2])) / 1.0);
					
				}
			} else {
				dst.at<cv::Vec3b>(iii,jjj)[0] = src.at<cv::Vec3b>(iii,jjj)[0];
				dst.at<cv::Vec3b>(iii,jjj)[1] = src.at<cv::Vec3b>(iii,jjj)[1];
				dst.at<cv::Vec3b>(iii,jjj)[2] = src.at<cv::Vec3b>(iii,jjj)[2];
			}
		}
	}
}

double scoreColorImage(const cv::Mat& src) {
	double score = 0.00;
	double percentileVals[3] = { 0.001, 0.500, 0.999 };
	double intensityVals[3];
	
	findPercentiles(src, intensityVals, percentileVals, 3);
	double range = max(abs(intensityVals[2] - intensityVals[1]), abs(intensityVals[0] - intensityVals[1]));
	score = min(range, 128.0) / 128.0;
	return score;	
}

double getInterpolatedVal(const cv::Mat& img, cv::Point2f& coord) {

	double retVal = 0.00;

	// Find four neighbours
	cv::Point dcoord[4];
	float val[4];
	float dist[4], total_dist = 0.0;
	
	val[0] = 0.0;
	val[1] = 0.0;
	val[2] = 0.0;
	val[3] = 0.0;

	// #1:
	dcoord[0] = cv::Point(int(floor(coord.x)), int(floor(coord.y)));

	if (img.type() == CV_8UC1) {
        val[0] = img.at<unsigned char>(dcoord[0].y, dcoord[0].x);
	} else if (img.type() == CV_64FC1) {
        val[0] = float(img.at<double>(dcoord[0].y, dcoord[0].x));
	}


	dist[0] = float(pow(pow(coord.x - ((float) dcoord[0].x), 2.0) + pow(coord.y - ((float) dcoord[0].y), 2.0), 0.5));
	total_dist += dist[0];
	//printf("%s << coord[0] = (%d, %d); (%f); (%f)\n", __FUNCTION__, dcoord[0].x, dcoord[0].y, val[0], dist[0]);

	// #2:
	dcoord[1] = cv::Point(int(ceil(coord.x)), int(floor(coord.y)));

    if (img.type() == CV_8UC1) {
        val[1] = img.at<unsigned char>(dcoord[1].y, dcoord[1].x);
	} else if (img.type() == CV_64FC1) {
        val[1] = float(img.at<double>(dcoord[1].y, dcoord[1].x));
	}

	dist[1] = float(pow(pow(coord.x - ((float) dcoord[1].x), 2.0) + pow(coord.y - ((float) dcoord[1].y), 2.0), 0.5));
	total_dist += dist[1];
	//printf("%s << coord[1] = (%d, %d); (%f); (%f)\n", __FUNCTION__, dcoord[1].x, dcoord[1].y, val[1], dist[1]);

	// #3:
	dcoord[2] = cv::Point(int(ceil(coord.x)), int(ceil(coord.y)));

    if (img.type() == CV_8UC1) {
        val[2] = img.at<unsigned char>(dcoord[2].y, dcoord[2].x);
	} else if (img.type() == CV_64FC1) {
        val[2] = float(img.at<double>(dcoord[2].y, dcoord[2].x));
	}


	dist[2] = float(pow(pow(coord.x - ((float) dcoord[2].x), 2.0) + pow(coord.y - ((float) dcoord[2].y), 2.0), 0.5));
	total_dist += dist[2];
	//printf("%s << coord[2] = (%d, %d); (%f); (%f)\n", __FUNCTION__, dcoord[2].x, dcoord[2].y, val[2], dist[2]);

	// #4:
	dcoord[3] = cv::Point(int(floor(coord.x)), int(ceil(coord.y)));

    if (img.type() == CV_8UC1) {
        val[3] = img.at<unsigned char>(dcoord[3].y, dcoord[3].x);
	} else if (img.type() == CV_64FC1) {
        val[3] = float(img.at<double>(dcoord[3].y, dcoord[3].x));
	}


	dist[3] = float(pow(pow(coord.x - ((float) dcoord[3].x), 2.0) + pow(coord.y - ((float) dcoord[3].y), 2.0), 0.5));
	total_dist += dist[3];

	cv::Point ref_coord = cv::Point(int(floor(coord.x)), int(floor(coord.y)));

	if (total_dist == 0.0) {
		retVal = val[0];
		return retVal;
	}

	cv::Mat x_mat(1, 2, CV_64FC1);
	x_mat.at<double>(0,0) = 1 - abs(coord.x - ((double) ref_coord.x));
	x_mat.at<double>(0,1) = abs(coord.x - ((double) ref_coord.x));

	cv::Mat y_mat(2, 1, CV_64FC1);
	y_mat.at<double>(0,0) = 1 - abs(coord.y - ((double) ref_coord.y));
	y_mat.at<double>(1,0) = abs(coord.y - ((double) ref_coord.y));

	cv::Mat f_vals(2, 2, CV_64FC1);
	f_vals.at<double>(0,0) = val[0];
	f_vals.at<double>(0,1) = val[3];
	f_vals.at<double>(1,0) = val[1];
	f_vals.at<double>(1,1) = val[2];

	cv::Mat A = x_mat * f_vals * y_mat;

	retVal = A.at<double>(0,0);

	if (0) { // (img.type() == CV_64FC1) {
        cout << "x_mat = " << x_mat << endl;
        cout << "y_mat = " << y_mat << endl;
        cout << "f_vals = " << f_vals << endl;
        cout << "A = " << A << endl;

        printf("%s << vals: (%f, %f, %f, %f) : dists: (%f, %f, %f, %f) : (%f, %f) vs (%d, %d) : (%f)\n", __FUNCTION__, val[0], val[1], val[2], val[3], dist[0], dist[1], dist[2], dist[3], coord.x, coord.y, ref_coord.x, ref_coord.y, retVal);

        cin.get();
	}

	return retVal;
}

bool matricesAreEqual(cv::Mat& mat1, cv::Mat& mat2) {

	if (mat1.rows != mat2.rows) return false;
	if (mat1.cols != mat2.cols) return false;
	if (mat1.type() != mat2.type()) return false;

	if ((mat1.type() != CV_16UC1) && (mat1.type() != CV_16SC1) && (mat1.type() != CV_8UC1) && (mat1.type() != CV_8UC3) && (mat1.type() != CV_8SC1) && (mat1.type() != CV_16UC3) && (mat1.type() != CV_16SC3) && (mat1.type() != CV_64FC1) && (mat1.type() != CV_32FC1))  {
		printf("%s << ERROR! Equality check for this type (%d) has not been implemented!\n", __FUNCTION__, mat1.type());
		return false;
	}
	
	bool isStillValid = true;

	#pragma omp parallel for
	for (int iii = 0; iii < mat1.rows; iii++) {
		for (int jjj = 0; jjj < mat1.cols; jjj++) {
			
			if (!isStillValid) break;
			switch (mat1.type()) {
				case CV_64FC1:
					if (mat1.at<double>(iii,jjj) != mat2.at<double>(iii,jjj)) isStillValid = false;
					break;
				case CV_32FC1:
					if (mat1.at<float>(iii,jjj) != mat2.at<float>(iii,jjj)) isStillValid = false;
					break;
				case CV_16UC1:
					if (mat1.at<unsigned short>(iii,jjj) != mat2.at<unsigned short>(iii,jjj)) isStillValid = false;
					break;
				case CV_16SC1:
					if (mat1.at<short>(iii,jjj) != mat2.at<short>(iii,jjj)) isStillValid = false;
					break;
				case CV_8UC1:
					if (mat1.at<unsigned char>(iii,jjj) != mat2.at<unsigned char>(iii,jjj)) isStillValid = false;
					break;
				case CV_8UC3:
					if ((mat1.at<cv::Vec3b>(iii,jjj)[0] != mat2.at<cv::Vec3b>(iii,jjj)[0]) || (mat1.at<cv::Vec3b>(iii,jjj)[1] != mat2.at<cv::Vec3b>(iii,jjj)[1]) || (mat1.at<cv::Vec3b>(iii,jjj)[2] != mat2.at<cv::Vec3b>(iii,jjj)[2])) isStillValid = false;
					break;
				case CV_8SC1:
					if (mat1.at<char>(iii,jjj) != mat2.at<char>(iii,jjj)) isStillValid = false;
					break;
				case CV_16UC3:
					if ((mat1.at<cv::Vec3s>(iii,jjj)[0] != mat2.at<cv::Vec3s>(iii,jjj)[0]) || (mat1.at<cv::Vec3s>(iii,jjj)[1] != mat2.at<cv::Vec3s>(iii,jjj)[1]) || (mat1.at<cv::Vec3s>(iii,jjj)[2] != mat2.at<cv::Vec3s>(iii,jjj)[2])) {
						isStillValid = false;
					}
					break;
				case CV_16SC3:
					if ((mat1.at<cv::Vec3s>(iii,jjj)[0] != mat2.at<cv::Vec3s>(iii,jjj)[0]) || (mat1.at<cv::Vec3s>(iii,jjj)[1] != mat2.at<cv::Vec3s>(iii,jjj)[1]) || (mat1.at<cv::Vec3s>(iii,jjj)[2] != mat2.at<cv::Vec3s>(iii,jjj)[2])) {
						isStillValid = false;
					}
					break;
				default:
					break;
			}
		}
	}

	if (!isStillValid) return false;
	return true;
}

double perpDist(cv::Point2f& P1, cv::Point2f& P2, cv::Point2f& P3)
{
    // TODO:
    // There may be some kind of issue occuring here... check the bug list at the top of this file

    // P3 is the test point
    double u, x, y, d;

    u = ((P3.x - P1.x)*(P2.x - P1.x) + (P3.y - P1.y)*(P2.y - P1.y)) / (pow(double(P2.x - P1.x), 2.0) + pow(double(P2.y - P1.y), 2.0));

    /*
        printf("denominator = %f\n", pow(double(P2.x - P1.x), 2.0) - pow(double(P2.y - P1.y), 2.0));
        printf("P1 = %f,%f\n", P1.x, P1.y);
        printf("P2 = %f,%f\n", P2.x, P2.y);
        printf("P3 = %f,%f\n", P3.x, P3.y);
        printf("u = %f\n", u);
    */

    x = P1.x + u*(P2.x - P1.x);
    y = P1.y + u*(P2.y - P1.y);

    d = pow(pow(P3.x - x, 2.0) + pow(P3.y - y, 2.0),0.5);

    return d;
}

double distBetweenPts2f(cv::Point2f& P1, cv::Point2f& P2)
{
    /*
    double retVal;
    retVal = pow((pow(double(P1.x - P2.x), 2.0) + pow(double(P1.y - P2.y),2)), 0.5);
    return retVal;
    */

    return pow((pow(double(P1.x - P2.x), 2.0) + pow(double(P1.y - P2.y),2)), 0.5);
}

double distBetweenPts(cv::Point3d& P1, cv::Point3d& P2) {
    return pow(double(pow(double(P1.x - P2.x), 2.0) + pow(double(P1.y - P2.y), 2.0) + pow(double(P1.z - P2.z), 2.0)), 0.5);
}

void writePoints(const char *filename, const vector<cv::Point2f>& pts) {

    ofstream myfile;
	myfile.open(filename);

	for (unsigned int jjj = 0; jjj < pts.size(); jjj++) {
		myfile << pts.at(jjj).x << "," << pts.at(jjj).y << endl;
	}

	myfile.close();

}

void readPoints(const char *filename, vector<cv::Point2f>& pts) {

	pts.clear();
	ifstream myfile;
	myfile.open(filename);
	
	char buffer[512];
	
	cv::Point2f latestPoint;
	char comma;
	
	while (true) {
		
		comma = ' ';
		myfile.getline(buffer, 512);
		
		stringstream ss;
		
		ss << buffer;
		
		ss >> latestPoint.x >> comma >> latestPoint.y;
		
		if (comma == ',') {
			pts.push_back(latestPoint);
		} else {
			break;
		}
		
		
		if (myfile.eof()) {
			break;
		}
	}


	myfile.close();

}

void redistortPoints(const vector<cv::Point2f>& src, vector<cv::Point2f>& dst, const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs, const cv::Mat& newCamMat) {

    double fx, fy, ifx, ify, cx, cy;
	double fx0, fy0, ifx0, ify0, cx0, cy0;
    double k[8]={0,0,0,0,0,0,0,0}; //, RR[3][3]={{1,0,0},{0,1,0},{0,0,1}};
    double r2, icdist, deltaX, deltaY;
    double x, y, x0, y0, x1, y1; //, xx, yy, ww;

	cv::Mat optimalMat(3, 3, CV_64FC1);

	// optimalMat = getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, Size(640,48), 1.0);

    //printf("%s << entered function.\n", __FUNCTION__);

    // will probably crash if it receives the identity matrix etc...

    fx0 = cameraMatrix.at<double>(0, 0);
    fy0 = cameraMatrix.at<double>(1, 1);
    ifx0 = 1./fx0;
    ify0 = 1./fy0;
    cx0 = cameraMatrix.at<double>(0, 2);
    cy0 = cameraMatrix.at<double>(1, 2);

    fx = newCamMat.at<double>(0, 0);
    fy = newCamMat.at<double>(1, 1);
    ifx = 1./fx;
    ify = 1./fy;
    cx = newCamMat.at<double>(0, 2);
    cy = newCamMat.at<double>(1, 2);

    for (int i = 0; i < distCoeffs.cols; i++){
        k[i] = distCoeffs.at<double>(0, i);
    }

    //printf("%s << cx = %f; cy = %f\n", __FUNCTION__, cx, cy);

    //cin.get();

    dst.clear();

    for (unsigned int i = 0; i < src.size(); i++) {
        // Points in undistorted image
        x = src.at(i).x;
        y = src.at(i).y;

        // printf("%s << undistorted points at [%d] = (%f, %f)\n", __FUNCTION__, i, x, y);

        // Apply cameraMatrix parameters (normalization)
        x0 = (x - cx)*ifx;
        y0 = (y - cy)*ify;

		x = x0;
		y = y0;

        // Determine radial and tangential distances/factors
        r2 = x*x + y*y;
        icdist = (1 + ((k[7]*r2 + k[6])*r2 + k[5])*r2)/(1 + ((k[4]*r2 + k[1])*r2 + k[0])*r2);
        deltaX = 2*k[2]*x*y + k[3]*(r2 + 2*x*x);
        deltaY = k[2]*(r2 + 2*y*y) + 2*k[3]*x*y;

		//icdist *= 0.75;

        // Redistort
        ///*
        //x = (x0 - deltaX)/icdist;
        //y = (y0 - deltaY)/icdist;
        //*/
        ///*
        x = (x0/icdist) + deltaX;
        y = (y0/icdist) + deltaY;
        //*/
		//x = x0/icdist;
		//y = y0/icdist;

        // Do something...
        /*
        xx = RR[0][0]*x + RR[0][1]*y + RR[0][2];
        yy = RR[1][0]*x + RR[1][1]*y + RR[1][2];
        ww = 1./(RR[2][0]*x + RR[2][1]*y + RR[2][2]);
        x = xx*ww;
        y = yy*ww;
        */

		x1 = x;
		y1 = y;

        // Reverse cameraMatrix parameters (denormalization)
        x = (x1/ifx0) + cx0;
        y = (y1/ify0) + cy0;

        // printf("%s << redistorted points at [%d] = (%f, %f)\n", __FUNCTION__, i, x, y);

        //cin.get();

        dst.push_back(cv::Point2f(float(x), float(y)));
    }

    // Temporary...
    // dst.assign(src.begin(), src.end());
}

double distBetweenPts(cv::Point& P1, cv::Point& P2)
{
    // TODO:
    // Possible issue.. see above ^

    double retVal;
    retVal = pow(double(pow(double(P1.x - P2.x), 2.0) + pow(double(P1.y - P2.y), 2.0)), 0.5);
    return retVal;
}

cv::Scalar getRandomColour() {
    cv::Scalar color( rand()&255, rand()&255, rand()&255 );

    return color;
}

double lookupValue(double xi, double yi, double maxVal, const cv::Mat& lookupMat) {


    cv::Point2f coord(float(xi*((double) lookupMat.cols)/maxVal), float(yi*((double) lookupMat.rows)/maxVal));

    double d = getInterpolatedVal(lookupMat, coord);

    //printf("%s << interpolating (%f, %f) -> (%f)\n", __FUNCTION__, coord.x, coord.y, d);

    /*
    double vx = (xi / maxVal) * lookupMat.cols;
    double vy = (yi / maxVal) * lookupMat.rows;

    int index_1 = std::max(std::min(((int) vy), lookupMat.rows), 0);
    int index_2 = std::max(std::min(((int) vx), lookupMat.cols), 0);

    double d = lookupMat.at<double>(index_1, index_2);
    */

    return d;

}

double findMinimumSeparation(vector<cv::Point2f>& pts)
{
    double minSep = 9e50;
    double val = 0.0;

    for (unsigned int i = 0; i < pts.size(); i++)
    {
        for (unsigned int j = i+1; j < pts.size(); j++)
        {
            val = norm(pts.at(i)-pts.at(j));
            if (val < minSep)
            {
                minSep = val;
            }
        }
    }

    return minSep;
}

cv::Point2f meanPoint(cv::Point2f& P1, cv::Point2f& P2)
{
    return cv::Point2f((P1.x+P2.x)/2, (P1.y+P2.y)/2);
}

double scoreThermalImage(const cv::Mat& src) {
	
	double score = 0.00;
	
	/*
	for (unsigned int iii = 0; iii < src.rows; iii++) {
		for (unsigned int jjj = 0; jjj < src.cols; jjj++) {
			
		}
	}
	*/
	
	double percentileVals[3] = { 0.001, 0.500, 0.999 };
	double intensityVals[3];
	findPercentiles(src, intensityVals, percentileVals, 3);

	
	double range = max(abs(intensityVals[2] - intensityVals[1]), abs(intensityVals[0] - intensityVals[1]));
	
	range /= 10.0;
	
	// printf("%s << range = (%f)\n", __FUNCTION__, range);
	
	score = min(range, 10.0) / 10.0;
	
	return score;
}

void combineImages(const cv::Mat& im1, const cv::Mat& im2, cv::Mat& dst) {
	
	//printf("%s << im1.size() = (%d, %d); im2.size() = (%d, %d); types = (%d, %d, %d, %d)\n", __FUNCTION__, im1.rows, im1.cols, im2.rows, im2.cols, im1.channels(), im1.depth(), im2.channels(), im2.depth());
	
	dst = cv::Mat::zeros(im1.size(), CV_8UC3);
	
	for (int iii = 0; iii < im1.rows; iii++) {
		for (int jjj = 0; jjj < im1.cols; jjj++) {
			
			dst.at<cv::Vec3b>(iii,jjj)[1] = im1.at<unsigned char>(iii,jjj);
			dst.at<cv::Vec3b>(iii,jjj)[2] = im2.at<unsigned char>(iii,jjj);
			
		}
	}
	
}

void unpackTo8bit3Channel(const cv::Mat& src, cv::Mat& dst) {
	
	dst = cv::Mat(src.size(), CV_8UC3);
	
	for (int iii = 0; iii < src.rows; iii++) {
		for (int jjj = 0; jjj < src.cols; jjj++) {
			
			// First byte should be the major component
			dst.at<cv::Vec3b>(iii,jjj)[0] = src.at<unsigned short>(iii,jjj) >> 8;
			
			// Second byte should be the minor component
			dst.at<cv::Vec3b>(iii,jjj)[1] = src.at<unsigned short>(iii,jjj) & 0x00FF;
			dst.at<cv::Vec3b>(iii,jjj)[2] = 0;
			
			if (src.at<unsigned short>(iii,jjj) != (256 * dst.at<cv::Vec3b>(iii,jjj)[0]) + dst.at<cv::Vec3b>(iii,jjj)[1]) {
				printf("%s << ERROR! (%d) != (%d, %d)\n", __FUNCTION__, src.at<unsigned short>(iii,jjj), dst.at<cv::Vec3b>(iii,jjj)[0], dst.at<cv::Vec3b>(iii,jjj)[1]);
				cin.get();
			}
			
		}
	}
	
}

double findBestAlpha(const cv::Mat& K, const cv::Mat& coeff, const cv::Size& camSize) {
	
	cv::Mat newCamMat;
	
	cv::Rect roi;
	
	double alpha = 0.00;
	bool centerPrincipalPoint = true;
	
	cv::Mat map1, map2;
	
	unsigned int protectionCounter = 0;
	
	cv::Mat R_default = cv::Mat::eye(3, 3, CV_64FC1);
	
	while (1) {
		
		cv::Mat white = cv::Mat::ones(camSize, CV_16UC1);
		cv::Mat mapped;
		
		newCamMat = getOptimalNewCameraMatrix(K, coeff, camSize, alpha, camSize, &roi, centerPrincipalPoint);
		
		initUndistortRectifyMap(K, coeff, R_default, newCamMat, camSize, CV_32FC1, map1, map2);
		
		double minVal, maxVal;
		
		cv::Mat scaled_white;
		white.convertTo(scaled_white, CV_16UC1, 65535, 0);
		
		remap(scaled_white, mapped, map1, map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT, 0);
		
		//imshow("scaled_white", mapped);
		//waitKey();
		
		minMaxLoc(mapped, &minVal, &maxVal);
		
		if (minVal == 65535.0) {
			break;
		}
		
		if (protectionCounter == 100) {
			alpha = 0.0;
			break;
		}
		
		alpha -= 0.01;
		protectionCounter++;
	}

	return alpha;
	
}

void splitMultimodalImage(const cv::Mat& src, cv::Mat& therm, cv::Mat& vis) {

	therm = cv::Mat::zeros(src.size(), CV_8UC1);
	vis = cv::Mat::zeros(src.size(), CV_8UC1);
	
	#pragma omp parallel for
	for (int iii = 0; iii < src.rows; iii++) {
		for (int jjj = 0; jjj < src.cols; jjj++) {
			therm.at<unsigned char>(iii,jjj) = src.at<cv::Vec3b>(iii,jjj)[2];
			vis.at<unsigned char>(iii,jjj) = src.at<cv::Vec3b>(iii,jjj)[1];
		}
	}
	
}

void applyFilter(const cv::Mat& src, cv::Mat& dst, int filter, double param) {
	
	int ksize;
	
	if (filter == GAUSSIAN_FILTERING) {
		ksize = int(param * 2);
	} else {
		if (filter != BILATERAL_FILTERING) { printf("%s << ERROR!\n", __FUNCTION__); }
		ksize = int(param);
	}
	
	if ((int(ksize) % 2) == 0) ksize++;
	
	if (filter == GAUSSIAN_FILTERING) {
		GaussianBlur(src, dst, cv::Size(ksize, ksize), sqrt(param));
	} else if (filter == BILATERAL_FILTERING) {
		bilateralFilter(src, dst, ksize, param * 2.0, param * 2.0);
	} else {
		src.copyTo(dst);
	}
	
}

void temperatureDownsample(const cv::Mat& src, cv::Mat& dst, double minVal, double maxVal) {
	
	if (dst.rows == 0) dst = cv::Mat::zeros(src.size(), CV_8UC1);
	
	#pragma omp parallel for
	for (int iii = 0; iii < src.rows; iii++) {
		for (int jjj = 0; jjj < src.cols; jjj++) {
			dst.at<unsigned char>(iii,jjj) = (unsigned char)(std::min(float(255.0), float(max(0.0, (std::max(src.at<float>(iii,jjj) - minVal, 0.0) / (maxVal - minVal)) * 255.0))));
		}
	}
}
void convertToTemperatureMat(const cv::Mat& src, cv::Mat& dst, double grad, int intercept) {
	
	if (dst.rows == 0) dst = cv::Mat::zeros(src.size(), CV_32FC1);
	
	#pragma omp parallel for
	for (int iii = 0; iii < src.rows; iii++) {
		for (int jjj = 0; jjj < src.cols; jjj++) {
			dst.at<float>(iii,jjj) = (float(src.at<unsigned short>(iii,jjj)) - float(intercept)) / float(grad);
		}
	}
}

void temperatureRangeBasedResample(const cv::Mat& src, cv::Mat& dst, double degreesPerGraylevel, double desiredDegreesPerGraylevel) {
	if (dst.rows == 0) dst = cv::Mat::zeros(src.size(), CV_8UC1);
	
	double percentile_levels[1], percentile_values[1];
	percentile_levels[0] = 0.5;
	findPercentiles(src, percentile_values, percentile_levels, 1);
	int median = int(percentile_values[0]);

	for (int iii = 0; iii < src.rows; iii++) {
		for (int jjj = 0; jjj < src.cols; jjj++) {
			dst.at<unsigned char>(iii,jjj) = min(255, max(0, int(round((float(src.at<unsigned char>(iii,jjj)) - float(median))*(float(degreesPerGraylevel)/float(desiredDegreesPerGraylevel))  + float(median)))));
		}
	}

}

cv::Mat read_image_from_file(std::string path) {
	cv::Mat frame;
#ifdef _OPENCV_VERSION_3_PLUS_
	frame = cv::imread(path, cv::IMREAD_ANYDEPTH);
#else
	frame = cv::imread(path, CV_LOAD_IMAGE_ANYDEPTH);
#endif
	return frame;
}

E_ImageDatatype determineFrameType( cv::Mat& frame ) 
{
	if ( frame.channels() == 1 ) 
  {
		if ( frame.depth() == CV_16U ) 
    {
			return DATATYPE_RAW;
		} 
    else if ( frame.depth() == CV_8U ) 
    {
			return DATATYPE_8BIT;
		} 
    else 
    {
			return DATATYPE_INVALID;
		}
	} 
  else if ( frame.channels() == 3 ) 
  {
		return DATATYPE_8BIT;
	}
	return DATATYPE_INVALID;
}

void temperatureRangeBasedDownsample(const cv::Mat& src, cv::Mat& dst, int newMedian, double degreesPerGraylevel, double desiredDegreesPerGraylevel) {

	if (dst.rows == 0) dst = cv::Mat::zeros(src.size(), CV_8UC1);

	if (newMedian == -1) {
		double percentile_levels[1], percentile_values[1];
		percentile_levels[0] = 0.5;
		findPercentiles(src, percentile_values, percentile_levels, 1);
		newMedian = int(percentile_values[0]);
	}

	for (int iii = 0; iii < src.rows; iii++) {
		for (int jjj = 0; jjj < src.cols; jjj++) {
			dst.at<unsigned char>(iii,jjj) = min(255, max(0, int(round((float(src.at<unsigned short>(iii,jjj)) - float(newMedian))*(float(degreesPerGraylevel)/float(desiredDegreesPerGraylevel))  + 127.5))));
		}
	}
}

void temperatureDownsample16(const cv::Mat& src, cv::Mat& dst) {
	
	if (dst.rows == 0) {
		dst = cv::Mat::zeros(src.size(), CV_16UC1);
	}
	
	#pragma omp parallel for
	for (int iii = 0; iii < src.rows; iii++) {
		for (int jjj = 0; jjj < src.cols; jjj++) {

			dst.at<unsigned short>(iii,jjj) = (unsigned short)(floor(((src.at<float>(iii,jjj) * 10.0) + 1000.0) + 0.5));
			

		}
	}
	
}

void convert16bitTo8bitConfidence(const cv::Mat& src, const cv::Mat& conf, cv::Mat& dst) {
	
	if (dst.rows == 0) {
		dst = cv::Mat::zeros(src.size(), CV_8UC3);
	}
	
	#pragma omp parallel for
	for (int iii = 0; iii < src.rows; iii++) {
		for (int jjj = 0; jjj < src.cols; jjj++) {

			// ORIGINAL PLAN!
			
			dst.at<cv::Vec3b>(iii,jjj)[1] = src.at<unsigned short>(iii,jjj) % 256;
			dst.at<cv::Vec3b>(iii,jjj)[0] = (src.at<unsigned short>(iii,jjj) - (src.at<unsigned short>(iii,jjj) % 256) ) / 256;
			
			
			/*
			dst.at<cv::Vec3b>(iii,jjj)[0] = (unsigned char) ((max(20.0, min(45.0, (float(src.at<unsigned short>(iii,jjj)) - 1000.0) / 10.0))-20.0)*10.0);
			dst.at<cv::Vec3b>(iii,jjj)[1] = 0;
			*/
			
			// printf("%s << splitting (%d) (%f) into (%d) and (%d)\n", __FUNCTION__, src.at<unsigned short>(iii,jjj), float((src.at<unsigned short>(iii,jjj) - 1000) / 10), dst.at<cv::Vec3b>(iii,jjj)[0], dst.at<cv::Vec3b>(iii,jjj)[1]);
			
			dst.at<cv::Vec3b>(iii,jjj)[2] = (unsigned char) (255.0 * conf.at<double>(iii,jjj));

		}
	}
	
}

void fixedDownsample(const cv::Mat& src, cv::Mat& dst, double center, double range) {

	dst = cv::Mat::zeros(src.rows, src.cols, CV_8UC1);

	#pragma omp parallel for
	for (int iii = 0; iii < src.rows; iii++) {
		for (int jjj = 0; jjj < src.cols; jjj++) {

			dst.at<unsigned char>(iii,jjj) = (unsigned char)(std::min(255.0, std::max(0.0, 127.5 + 127.5*(double(src.at<unsigned short>(iii,jjj)) - center)/(0.5 * range))));

		}
	}

}

void applyIntensityShift(const cv::Mat& src1, cv::Mat& dst1, const cv::Mat& src2, cv::Mat& dst2, double grad, double shift) {

	double center = 8000.0, range = 1.0;

	double percentileVals[3] = { 0.001, 0.500, 0.999 };
	double intensityVals[3];
	findPercentiles(src1, intensityVals, percentileVals, 3);

	center = intensityVals[1];
	range = std::max(abs(intensityVals[2] - intensityVals[1]), abs(intensityVals[0] - intensityVals[1]));

	//printf("%s << normalizing with center = (%f) and range = (%f)\n", __FUNCTION__, center, range);

	//adaptiveDownsample(src1, dst1);
	fixedDownsample(src1, dst1, center, 2.0*range);

	cv::Mat src2_shifted(src2.rows, src2.cols, src2.type());
	// src2.copyTo(src2_shifted);

	#pragma omp parallel for
	for (int iii = 0; iii < src2_shifted.rows; iii++) {
		for (int jjj = 0; jjj < src2_shifted.cols; jjj++) {

			src2_shifted.at<unsigned short>(iii,jjj) = (unsigned short)(grad * double(src2.at<unsigned short>(iii,jjj)) + shift);

		}
	}

	//adaptiveDownsample(src2_shifted, dst2);
	fixedDownsample(src2_shifted, dst2, center, range);

}

void drawGrid(const cv::Mat& src, cv::Mat& dst, int mode) {

	src.copyTo(dst);

	cv::Scalar col;

	int shift;

	if (mode == 0) {
		shift = 0;
		col = cv::Scalar(255, 0, 0);
	} else {
		shift = 1;
		col = cv::Scalar(0, 0, 255);
	}

	cv::Point2f startPt, endPt;

	double amt = double(src.cols) / 8.0;

	// Vertical lines
	for (int iii = shift; iii <= 8-shift; iii++) {
		startPt = cv::Point2f(float(16.0 * double(iii)*amt), float(16.0 * double(shift)*amt));
		endPt = cv::Point2f(float(16.0 * double(iii)*amt), float(16.0 * (double(src.rows) - double(shift)*amt)));

		#ifdef _OPENCV_VERSION_3_PLUS_
		line(dst, startPt, endPt, col, 1, cv::LINE_AA, 4);
		#else
		line(dst, startPt, endPt, col, 1, CV_AA, 4);
		#endif
	}

	// Horizontal lines
	for (int iii = shift; iii <= 6-shift; iii++) {

		startPt = cv::Point2f(float(16.0 * double(shift)*amt), float(16.0 * double(iii)*amt));
		endPt = cv::Point2f(float(16.0 * (double(src.cols) - double(shift)*amt)), float(16.0 * double(iii)*amt));

		#ifdef _OPENCV_VERSION_3_PLUS_
		line(dst, startPt, endPt, col, 1, cv::LINE_AA, 4);
		#else
		line(dst, startPt, endPt, col, 1, CV_AA, 4);
		#endif

	}


}

void histExpand8(const cv::Mat& src, cv::Mat& dst) {

	double minVal, maxVal;
	minMaxLoc(src, &minVal, &maxVal);

	dst = cv::Mat::zeros(src.size(), src.type());

	#pragma omp parallel for
	for (int iii = 0; iii < src.rows; iii++) {
		for (int jjj = 0; jjj < src.cols; jjj++) {

			unsigned char val = (unsigned char)( (((double)src.at<unsigned char>(iii,jjj)) - minVal) * 255.0 / (maxVal - minVal));

			dst.at<unsigned char>(iii,jjj) = val;
		}
	}

}

void clusterRectangles(vector<cv::Rect>& rectangles, double minOverlap) {

    if (rectangles.size() == 0) {
        return;
    }

    unsigned int j,k;

    bool hasBeenClustered;

    vector<cv::Rect> falseVector;

    cv::Rect tempRect;

    vector<vector<cv::Rect> > clusteredRectangles;

    clusteredRectangles.push_back(falseVector);

    clusteredRectangles.at(0).push_back(rectangles.at(0));

    // For each remaining rectangle
    for (unsigned int i = 1; i < rectangles.size(); i++) {

        hasBeenClustered = false;

        j = 0;
        k = 0;

        while (!hasBeenClustered) {
            if (rectangleOverlap(rectangles.at(i), clusteredRectangles.at(j).at(k)) > minOverlap) {
                clusteredRectangles.at(j).push_back(rectangles.at(i));
                hasBeenClustered = true;
            } else if (k < clusteredRectangles.at(j).size()-1) {
                k++;
            } else if (j < clusteredRectangles.size()-1) {
                j++;
                k = 0;
            } else {
                clusteredRectangles.push_back(falseVector);
                clusteredRectangles.at(j+1).push_back(rectangles.at(i));
                hasBeenClustered = true;
            }
        }

        //printf("%s << overlapProp = %f\n", __FUNCTION__, overlapProp);

    }

    rectangles.clear();

    for (unsigned int i = 0; i < clusteredRectangles.size(); i++) {
        tempRect = meanRectangle(clusteredRectangles.at(i));
        rectangles.push_back(tempRect);
    }

}

cv::Rect meanRectangle(vector<cv::Rect>& rectangles) {

    cv::Rect retVal;
    double xSum = 0.0, ySum = 0.0, wSum = 0.0, hSum = 0.0;

    for (unsigned int i = 0; i < rectangles.size(); i++) {
        xSum += rectangles.at(i).x;
        ySum += rectangles.at(i).y;
        wSum += rectangles.at(i).width;
        hSum += rectangles.at(i).height;
    }

    xSum /= rectangles.size();
    ySum /= rectangles.size();
    wSum /= rectangles.size();
    hSum /= rectangles.size();

    retVal = cv::Rect(int(xSum), int(ySum), int(wSum), int(hSum));

    return retVal;
}

void weightedMixture(cv::Mat& dst, const std::vector<cv::Mat>& srcs, const std::vector<double>& weightings) {

    double totalWeighting = 0.0;
    vector<double> newWeightings;
    newWeightings.insert(newWeightings.end(), weightings.begin(), weightings.end());

    for (unsigned int iii = 0; iii < weightings.size(); iii++) {
        totalWeighting += weightings.at(iii);
    }

    for (unsigned int iii = 0; iii < weightings.size(); iii++) {
        newWeightings.at(iii) /= totalWeighting;
    }

    cv::Mat tmpDst = cv::Mat::zeros(srcs.at(0).size(), CV_64FC3);

    for (unsigned int i = 0; i < srcs.size(); i++) {

        for (int m = 0; m < srcs.at(i).rows; m++) {

            for (int n = 0; n < srcs.at(i).cols; n++) {

                for (int k = 0; k < 3; k++) {
                    tmpDst.at<cv::Vec3d>(m,n)[k] += double((srcs.at(i)).at<cv::Vec3b>(m,n)[k]) * newWeightings.at(i);
                }

            }

        }

    }

    cv::Mat normMat;
    normalize_64_vec(normMat, tmpDst);

    dst = cv::Mat(normMat.size(), CV_8UC3);
    convertScaleAbs(normMat, dst, 255);

}

void mixImages(cv::Mat& dst, std::vector<cv::Mat>& images) {
    // No checks at present...

    dst = cv::Mat::zeros(images.at(0).size(), CV_64FC3);
    cv::Mat tmp;
    dst.copyTo(tmp);

	
    for (unsigned int i = 0; i < images.size(); i++) {

        for (int m = 0; m < images.at(i).rows; m++) {

            for (int n = 0; n < images.at(i).cols; n++) {

                for (int k = 0; k < 3; k++) {
                    dst.at<cv::Vec3d>(m,n)[k] += double((images.at(i)).at<cv::Vec3b>(m,n)[k]) / images.size();
                }



            }


        }

        normalize_64_vec(tmp, dst);

        //imshow("tmp", tmp);
        //waitKey(0);

    }

}

bool rectangle_contains_centroid(cv::Rect mainRectangle, cv::Rect innerRectangle) {
    bool retVal = false;

    //printf("%s << Main Rectangle: [%d, %d] : [%d, %d]\n", __FUNCTION__, mainRectangle.x, mainRectangle.y, mainRectangle.x+mainRectangle.width, mainRectangle.y+mainRectangle.height);
    //printf("%s << Centroid: [%d, %d]\n", __FUNCTION__, innerRectangle.x + innerRectangle.width/2, innerRectangle.y + innerRectangle.height/2);

    retVal = mainRectangle.contains(cv::Point(innerRectangle.x + innerRectangle.width/2, innerRectangle.y + innerRectangle.height/2));

    return retVal;
}

cv::Point rectangle_center(cv::Rect input) {
    cv::Point center;

    center.x = input.x + input.width/2;
    center.y = input.y + input.height/2;


    return center;
}

double rectangleOverlap(cv::Rect rectangle1, cv::Rect rectangle2) {

    double area1, area2, exLeft, exRight, exTop, exBot, overlapArea, overlapProp;

    area1 = rectangle1.width*rectangle1.height;
    area2 = rectangle2.width*rectangle2.height;

    exLeft = max(rectangle1.x, rectangle2.x);
    exRight = min(rectangle1.x+rectangle1.width, rectangle2.x+rectangle2.width);
    exTop = max(rectangle1.y, rectangle2.y);
    exBot = min(rectangle1.y+rectangle1.height, rectangle2.y+rectangle2.height);

    if ((exLeft > exRight) || (exTop > exBot)) return 0.0;

    overlapArea = (exRight-exLeft)*(exBot-exTop);
    overlapProp = overlapArea / (max(area1, area2));
    return overlapProp;
}

void trimToDimensions(cv::Mat& image, int width, int height) {

    cv::Mat dst;

    int imWidth = image.cols;
    int imHeight = image.rows;

    double actualRatio = double(imWidth)/double(imHeight);
    double wantedRatio = double(width)/double(height);

    int initialWidth, initialHeight;

    printf("%s << image dimensions = %d x %d.\n", __FUNCTION__, imWidth, imHeight);
    printf("%s << desired dimensions = %d x %d.\n", __FUNCTION__, width, height);

    printf("%s << actualRatio = %f; wantedRatio = %f\n", __FUNCTION__, actualRatio, wantedRatio);

    if (actualRatio < wantedRatio) {
        printf("%s << taller than we want.\n", __FUNCTION__);

        initialWidth = width;
        initialHeight = int(double(width)/actualRatio);

        // resize to get width to exactly desired width
        resize(image, dst, cv::Size(initialWidth, initialHeight));

        //imshow("cropping", dst);
        //waitKey(0);

        // cut down height to desired height
        int topY = (dst.rows - height)/2;
        int botY = topY + height;

        printf("%s << topY = %d; botY = %d\n", __FUNCTION__, topY, botY);

        cropImage(dst, cv::Point(0, topY), cv::Point(width, botY));

        //imshow("cropping", dst);
        //waitKey(0);

    } else if (actualRatio >= wantedRatio) {
        printf("%s << wider than we want.\n", __FUNCTION__);

        initialWidth = int(double(height)*actualRatio);
        initialHeight = height;

        // resize to get width to exactly desired width
        resize(image, dst, cv::Size(initialWidth, initialHeight));

        // cut down height to desired height
        int leftX = (dst.cols - width)/2;
        int rightX = leftX + width;

        printf("%s << leftX  = %d; rightX = %d\n", __FUNCTION__, leftX , rightX);

        cropImage(dst, cv::Point(leftX, 0), cv::Point(rightX, height));
    }

    dst.copyTo(image);
}

void process8bitImage(const cv::Mat& src, cv::Mat& dst, int code, double factor) { (code == NORM_MODE_EQUALIZATION) ? equalizeHist(src, dst) : src.copyTo(dst); }

void adaptiveDownsample(const cv::Mat& src, cv::Mat& dst, int code, double factor) {

	cv::Mat dwn, _16;
	double minVal, maxVal;
	minMaxLoc(src, &minVal, &maxVal);
	double percentileVals[5] = { 0.001, (factor / 2.0), 0.500, 1.0 - (factor / 2.0), 0.999 };
	double intensityVals[5];

	findPercentiles(src, intensityVals, percentileVals, 5);

	//printf("%s << percentileVals(%f, %f, %f, %f, %f) = (%d, %d, %d, %d, %d)\n", __FUNCTION__, percentileVals[0], percentileVals[1], percentileVals[2], percentileVals[3], percentileVals[4], intensityVals[0], intensityVals[1], intensityVals[2], intensityVals[3], intensityVals[4]);

	intensityVals[1] = max(intensityVals[1], minVal);
	intensityVals[3] = min(intensityVals[3], maxVal);

	double midVal = (minVal + maxVal) / 2.0;
	double centralVal = intensityVals[1]; 
	double fullRange = abs(intensityVals[3] - intensityVals[1]);
	double compressionFactor = 1.0;

	if (code == NORM_MODE_FULL_STRETCHING) {
		if (fullRange > 255.0) compressionFactor = fullRange / 255.0;

		minVal = max(midVal - 127.5*compressionFactor, 0.0);
		maxVal = min(midVal + 127.5*compressionFactor, 65535.0);
		
		normalize_16(_16, src, minVal, maxVal);
		down_level(dst, _16);
	} else if (code == NORM_MODE_CENTRALIZED) {
		compressionFactor = 255.0 / (4.0 * max(factor, 0.01));

		minVal = max(midVal - compressionFactor, 0.0);
		maxVal = min(midVal + compressionFactor, 65535.0);
		
		normalize_16(_16, src, minVal, maxVal);
		down_level(dst, _16);
	} else if (code == NORM_MODE_EQUALIZATION) {
		normalize_16(_16, src, intensityVals[1], intensityVals[3]);
		down_level(dwn, _16);
		equalizeHist(dwn, dst);
	} else src.convertTo(dst, CV_8UC1);
}



void drawLinesBetweenPoints(cv::Mat& image, const vector<cv::Point2f>& src, const vector<cv::Point2f>& dst)
{

    cv::Point p1, p2;

    for (unsigned int i = 0; i < src.size(); i++)
    {
        p1 = cv::Point(int(src.at(i).x*16.0), int(src.at(i).y*16.0));
        p2 = cv::Point(int(dst.at(i).x*16.0), int(dst.at(i).y*16.0));

        //line(image, p1, p2, cv::Scalar(0,0,255), 1, CV_AA, 4);
        
		#ifdef _OPENCV_VERSION_3_PLUS_
		circle(image, p2, 4*16, cv::Scalar(0,0,255), 1, cv::LINE_AA, 4);
		#else
		circle(image, p2, 4*16, cv::Scalar(0,0,255), 1, CV_AA, 4);
		#endif
    }

}

cv::Point findCentroid(vector<cv::Point>& contour)
{
    cv::Moments momentSet;

    double x,y;

    momentSet = moments(cv::Mat(contour));
    x = momentSet.m10/momentSet.m00;
    y = momentSet.m01/momentSet.m00;

    return cv::Point((int)x,(int)y);
}

cv::Point2f findCentroid2f(vector<cv::Point>& contour)
{
    cv::Moments momentSet;

    float x,y;

    momentSet = moments(cv::Mat(contour));
    x = float(momentSet.m10/momentSet.m00);
    y = float(momentSet.m01/momentSet.m00);

    return cv::Point2f(x,y);
}

void createGaussianMatrix(cv::Mat& gaussianMat, double sigmaFactor)
{

    cv::Mat distributionDisplay(cv::Size(640, 480), CV_8UC1);
    cv::Mat binTemp(gaussianMat.size(), CV_8UC1);

    // sigmaFactor says how many standard deviations should span from the center to the nearest edge
    // (i.e. along the shortest axis)


    // What about an elliptical gaussian function?

    cv::Point2f center((float)((double(gaussianMat.size().height-1))/2), (float)((double(gaussianMat.size().width-1))/2));
    cv::Point2f tmpPt;
    double dist = 0.0;//, average = 0.0, maxVal = 0.0;
    double sigma = min(gaussianMat.size().width, gaussianMat.size().height)/(2*sigmaFactor);

    //double A = (gaussianMat.size().width*gaussianMat.size().height) / (sigma * pow(2*3.142, 0.5));
    double A = 1.0;


    for (int i = 0; i < gaussianMat.size().width; i++)
    {
        for (int j = 0; j < gaussianMat.size().height; j++)
        {
            tmpPt = cv::Point2f(float(j), float(i));
            dist = distBetweenPts2f(center, tmpPt);
            
            gaussianMat.at<double>(j,i) = A*exp(-(pow(dist,2)/(2*pow(sigma,2))));

			/*
            //gaussianMat.at<double>(j,i) = A*exp(-(pow(dist,2)/(2*pow(sigma,2))));
            //gaussianMat.at<double>(j,i) = dist;
            if (dist < max(double(gaussianMat.size().width)/2, double(gaussianMat.size().height)/2))
            {
                gaussianMat.at<double>(j,i) = 1.0;
            }
            else
            {
                gaussianMat.at<double>(j,i) = 0.0;
            }

            average += gaussianMat.at<double>(j,i);

            if (gaussianMat.at<double>(j,i) > maxVal)
            {
                maxVal = gaussianMat.at<double>(j,i);
            }
			*/
            //printf("val = %f\n", gaussianMat.at<double>(j,i));

            //gaussianMat.at<double>(j,i) = double(rand()) / RAND_MAX;
        }
    }

    //average /= gaussianMat.size().width*gaussianMat.size().height;

    //gaussianMat /= average;
    
    /*
     * 
    average = 0.0;
    maxVal = 0.0;

    for (int i = 0; i < gaussianMat.size().width; i++)
    {
        for (int j = 0; j < gaussianMat.size().height; j++)
        {
            average += gaussianMat.at<double>(j,i);
            if (gaussianMat.at<double>(j,i) > maxVal)
            {
                maxVal = gaussianMat.at<double>(j,i);
            }
        }
    }

    average /= gaussianMat.size().width*gaussianMat.size().height;

	*/
	
    //printf("%s << average val = %f\n", __FUNCTION__, average);
    //cin.get();

    /*
    convertScaleAbs(gaussianMat, binTemp, (255.0/maxVal), 0);
    svLib::simpleResize(binTemp, distributionDisplay, Size(480, 640));
    //equalizeHist(distributionDisplay, distributionDisplay);
    imshow("gaussianMat", distributionDisplay);
    waitKey(40);
    */
}

void cropImage(cv::Mat& image, cv::Point tl, cv::Point br)
{
    // not compatible with all image types yet...

    int width, height, xOff, yOff;

    //printf("%s << Starting function.\n", __FUNCTION__);

    //printf("%s << TL = (%d, %d); BR = (%d, %d)\n", __FUNCTION__, tl.x, tl.y, br.x, br.y);

    width = abs(br.x-tl.x);
    height = abs(br.y-tl.y);

    xOff = min(tl.x, br.x);
    yOff = min(tl.y, br.y);

    //printf("%s << width = %d, height = %d, xOff = %d, yOff = %d\n", __FUNCTION__, width, height, xOff, yOff);

    //cin.get();

    cv::Mat tmpMat;

    if (image.channels() == 3)
    {
        tmpMat = cv::Mat(height, width, CV_8UC3);
    }
    else if (image.channels() == 1)
    {
        tmpMat = cv::Mat(height, width, CV_8UC1);
    }



    for (int i = 0; i < width; i++)
    {
        for (int j = 0; j < height; j++)
        {
            //printf("%s << %d / %d\n", __FUNCTION__, i, j);

            if (image.channels() == 3)
            {
                if ((j+yOff < 0) || (j+yOff > image.rows-1) || (i+xOff < 0) || (i+xOff > image.cols-1))
                {
                    tmpMat.at<cv::Vec3b>(j,i)[0] = 0;
                    tmpMat.at<cv::Vec3b>(j,i)[1] = 0;
                    tmpMat.at<cv::Vec3b>(j,i)[2] = 0;
                }
                else
                {
                    tmpMat.at<cv::Vec3b>(j,i) = image.at<cv::Vec3b>(j+yOff,i+xOff);
                }
            }
            else if (image.channels() == 1)
            {
                if ((j+yOff < 0) || (j+yOff > image.rows-1) || (i+xOff < 0) || (i+xOff > image.cols-1))
                {
                    tmpMat.at<unsigned char>(j,i) = 0;
                }
                else
                {
                    tmpMat.at<unsigned char>(j,i) = image.at<unsigned char>(j+yOff,i+xOff);
                }


            }




        }
    }

    //tmpMat.copyTo(image);
    resize(tmpMat, image, cv::Size(width, height)); // working

    //printf("%s << Completing function.\n", __FUNCTION__);

}

void convertVectorToPoint(vector<cv::Point2f>& input, vector<cv::Point>& output)
{
    output.clear();

    for (unsigned int i = 0; i < input.size(); i++)
    {
        output.push_back(cv::Point((int)input.at(i).x, (int)input.at(i).y));
    }
}

void convertVectorToPoint2f(vector<cv::Point>& input, vector<cv::Point2f>& output)
{
    // TODO:
    // Nothing much.

    output.clear();

    for (unsigned int i = 0; i < input.size(); i++)
    {
        output.push_back(cv::Point2f((float)input.at(i).x, (float)input.at(i).y));
    }
}

void simpleResize(cv::Mat& src, cv::Mat& dst, cv::Size size)
{

    dst = cv::Mat::zeros(size, src.type());

    for (int i = 0; i < dst.size().width; i++)
    {
        for (int j = 0; j < dst.size().height; j++)
        {
            if (src.depth() == 1)
            {
                dst.at<unsigned char>(j,i) = src.at<unsigned char>(j*src.size().height/dst.size().height,i*src.size().width/dst.size().width);
            }
            else if (src.depth() == 8)
            {
                dst.at<double>(j,i) = src.at<double>(j*src.size().height/dst.size().height,i*src.size().width/dst.size().width);
            }
            else if (src.depth() == CV_16U)
            {
                dst.at<unsigned short>(j,i) = src.at<unsigned short>(j*src.size().height/dst.size().height,i*src.size().width/dst.size().width);
            }

        }
    }
}

void copyContour(vector<cv::Point>& src, vector<cv::Point>& dst)
{
    // TODO:
    // Make safer.

    dst.clear();

    for (unsigned int i = 0; i < src.size(); i++)
    {
        dst.push_back(src.at(i));
    }
}

void swapElements(vector<cv::Point2f>& corners, int index1, int index2)
{
    // TODO:
    // Nothing much.

    cv::Point2f tempPt;

    tempPt = corners.at(index1);    // copy first element to temp
    corners.at(index1) = corners.at(index2);  // put best element in first
    corners.at(index2) = tempPt;  // copy temp to where best element was
}

void invertMatIntensities(const cv::Mat& src, cv::Mat& dst)
{
    
    if ((dst.size() != src.size()) && (dst.type() != src.type())) {
		dst.release();
		dst = cv::Mat(src.size(), src.type());
	}

    if (src.type() == CV_8UC1)
    {


		#pragma omp parallel for
        for (int iii = 0; iii < src.rows; iii++)
        {
            for (int jjj = 0; jjj < src.cols; jjj++)
            {
                dst.at<unsigned char>(iii,jjj) = 255 - src.at<unsigned char>(iii, jjj);
            }
        }

    }
    else if (src.type() == CV_8UC3)
    {

        // printf("%s << here.\n", __FUNCTION__);
		#pragma omp parallel for
        for (int iii = 0; iii < src.rows; iii++)
        {
            for (int jjj = 0; jjj < src.cols; jjj++)
            {
                //a = src.at<cv::Vec3b>(iii, jjj)[0];
                //z = std::max(std::min(255, int(255 - (1.5*(a - 128)+128))),0);
                //dst.at<cv::Vec3b>(iii, jjj)[0] = z;
                //dst.at<cv::Vec3b>(iii, jjj)[1] = z;
                //dst.at<cv::Vec3b>(iii, jjj)[2] = z;

                dst.at<cv::Vec3b>(iii, jjj)[0] = 255 - src.at<cv::Vec3b>(iii, jjj)[0];
                dst.at<cv::Vec3b>(iii, jjj)[1] = 255 - src.at<cv::Vec3b>(iii, jjj)[1];
                dst.at<cv::Vec3b>(iii, jjj)[2] = 255 - src.at<cv::Vec3b>(iii, jjj)[2];
            }
        }
    }

    //imshow("dst", dst);
    //waitKey();

}

void contourDimensions(vector<cv::Point> contour, double& width, double& height)
{
    // TODO:
    // May want to replace this with something that finds the longest and shortest distances across
    // Because the idea of this is to help determine if it's a square or not.

    // new methodology
    cv::RotatedRect wrapperRectangle;
    cv::Size size;
    vector<cv::Point> contourCpy;
    cv::Point meanPoint;

    // Interpolate contour to get it to an adequate size for "fitEllipse" function
    if (contour.size() < 6)
    {
        for (unsigned int i = 0; i < contour.size()-1; i++)
        {
            contourCpy.push_back(contour.at(i));
            meanPoint.x = (2*contour.at(i).x + 1*contour.at(i+1).x) / 3;
            meanPoint.y = (2*contour.at(i).y + 1*contour.at(i+1).y) / 3;
            contourCpy.push_back(meanPoint);
            meanPoint.x = (1*contour.at(i).x + 2*contour.at(i+1).x) / 3;
            meanPoint.y = (1*contour.at(i).y + 2*contour.at(i+1).y) / 3;
            contourCpy.push_back(meanPoint);
        }

        contourCpy.push_back(contour.at(contour.size()-1));
        meanPoint.x = (2*contour.at(contour.size()-1).x + 1*contour.at(0).x) / 3;
        meanPoint.y = (2*contour.at(contour.size()-1).y + 1*contour.at(0).y) / 3;
        contourCpy.push_back(meanPoint);
        meanPoint.x = (1*contour.at(contour.size()-1).x + 2*contour.at(0).x) / 3;
        meanPoint.y = (1*contour.at(contour.size()-1).y + 2*contour.at(0).y) / 3;
        contourCpy.push_back(meanPoint);

    }
    else
    {
        contourCpy.assign(contour.begin(), contour.end());
    }

    wrapperRectangle = fitEllipse(cv::Mat(contourCpy));
    size = wrapperRectangle.size;
    width = size.width;
    height = size.height;

    // old methodology... (simply using highest and lowest X and Y values..)
    /*
    double xMax = 0.0, yMax = 0.0, xMin = 99999.0, yMin = 99999.0;

    for (unsigned int i = 0; i < contour.size(); i++) {

        if (contour.at(i).x > xMax) {
            xMax = contour.at(i).x;
        }

        if (contour.at(i).y > yMax) {
            yMax = contour.at(i).y;
        }

        if (contour.at(i).x < xMin) {
            xMin = contour.at(i).x;
        }

        if (contour.at(i).y < yMin) {
            yMin = contour.at(i).y;
        }

    }

    width = xMax - xMin;
    height = yMax - yMin;
    */
}







void transferElement(vector<cv::Point2f>& dst, vector<cv::Point2f>& src, int index)
{
    cv::Point2f pointCpy;

    pointCpy = src.at(index);

    // Move from old one to new one
    dst.push_back(pointCpy);

    // Replace and shift points in old one
    for (unsigned int i = index; i < src.size()-1; i++)
    {
        src.at(i) = src.at(i+1);
    }

    // Truncate the original vector (effectively discarding old point)
    src.pop_back();
}


bool checkIfActuallyGray(const cv::Mat& im) {

	bool retVal = true;

	for (int iii = 0; iii < im.rows; iii++) {
		for (int jjj = 0; jjj < im.cols; jjj++) {

			if (im.at<cv::Vec3b>(iii,jjj)[0] != im.at<cv::Vec3b>(iii,jjj)[1]) {
				//printf("%s << (%d, %d) = (%d, %d, %d)\n", __FUNCTION__, iii, jjj, im.at<cv::Vec3b>(iii,jjj)[0], im.at<cv::Vec3b>(iii,jjj)[1], im.at<cv::Vec3b>(iii,jjj)[2]);
				return false;
			}

			if (im.at<cv::Vec3b>(iii,jjj)[2] != im.at<cv::Vec3b>(iii,jjj)[1]) {
				//printf("%s << (%d, %d) = (%d, %d, %d)\n", __FUNCTION__, iii, jjj, im.at<cv::Vec3b>(iii,jjj)[0], im.at<cv::Vec3b>(iii,jjj)[1], im.at<cv::Vec3b>(iii,jjj)[2]);
				return false;
			}

		}
	}

	return retVal;

}

void thresholdRawImage(cv::Mat& img, double *vals) {
	
	for (int iii = 0; iii < img.rows; iii++) {
		for (int jjj = 0; jjj < img.cols; jjj++) {
			
			if (double(img.at<unsigned short>(iii,jjj)) < vals[0]) {
				img.at<unsigned short>(iii,jjj) = (unsigned short) vals[0];
			}
			
			if (double(img.at<unsigned short>(iii,jjj)) > vals[1]) {
				img.at<unsigned short>(iii,jjj) = (unsigned short) vals[1];
			}
			
		}
	}
}

void findPercentiles(const cv::Mat& img, double *vals, double *percentiles, unsigned int num) {

	//double median = 0.0;

	cv::Mat mx;

	unsigned int *aimedPixelCounts;
	aimedPixelCounts = new unsigned int[num];
	bool *found;
	found = new bool[num];
	
	//int maxPixels = img.rows * img.cols;

	for (unsigned int iii = 0; iii < num; iii++) {
		found[iii] = false;
		aimedPixelCounts[iii] = (unsigned int) (((double) img.rows) * ((double) img.cols) * ((double) percentiles[iii]));
		
		if (((int)aimedPixelCounts[iii]) == img.rows*img.cols) {
			aimedPixelCounts[iii]--;
		}
		
		//printf("%s << aimedPixelCounts[%d] = %d (%f)\n", __FUNCTION__, iii, aimedPixelCounts[iii], percentiles[iii]);
	}

	//cin.get();

	//if (img.type() == CV_16UC1) {
		img.convertTo(mx, CV_32FC1);
	//}

	cv::MatND hist;
	int channels[] = {0};
	int ibins = 65536;
	int histSize[] = {ibins};
	float iranges[] = { 0, 65535 };
	const float* ranges[] = { iranges };

	calcHist(&mx, 1, channels, cv::Mat(), hist, 1, histSize, ranges);

	unsigned int pointsTally = 0;

	//bool allCountsReached = false;

	for (int i = 0; i < ibins; i++) {

		pointsTally += (unsigned int)(hist.at<float>(i));

		for (unsigned int iii = 0; iii < num; iii++) {
			
			if (pointsTally <= aimedPixelCounts[iii]) {
				vals[iii] = double(i);
				
			} else if (!found[iii]) {
				found[iii] = true;
				vals[iii] = double(i);
			}

		}

		bool allCountsReached = true;
		
		for (unsigned int iii = 0; iii < num; iii++) {
			if (found[iii] == false) {
				allCountsReached = false;
				break;
			}
		}
		
		if (allCountsReached) return;

	}

}

void shiftIntensities(cv::Mat& im, double scaler, double shifter, double downer) {

	double val;
	for (int iii = 0; iii < im.rows; iii++) {
		for (int jjj = 0; jjj < im.cols; jjj++) {

			val = ((double) im.at<unsigned char>(iii,jjj));

			val -= downer;

			val *= scaler;

			val += (downer + shifter);

			im.at<unsigned char>(iii,jjj) = ((unsigned char) val);

		}
	}

}

void findIntensityValues(double *vals, cv::Mat& im, cv::Mat& mask) {

	vals[0] = std::numeric_limits<double>::max();
	vals[1] = -std::numeric_limits<double>::max();
	vals[2] = 0.0;

	unsigned int activeCount = countNonZero(mask);

	if (activeCount == 0) {
		return;
	}

	unsigned int hist[256];

	for (unsigned int iii = 0; iii < 256; iii++) {
		hist[iii] = 0;
	}

	for (int iii = 0; iii < im.rows; iii++) {
		for (int jjj = 0; jjj < im.cols; jjj++) {

			if (mask.at<unsigned char>(iii,jjj) != 0) {
				vals[2] += ((double) im.at<unsigned char>(iii,jjj)) / ((double) activeCount);

				if (((double) im.at<unsigned char>(iii,jjj)) < vals[0]) {
					vals[0] = ((double) im.at<unsigned char>(iii,jjj));
				}

				if (((double) im.at<unsigned char>(iii,jjj)) > vals[1]) {
					vals[1] = ((double) im.at<unsigned char>(iii,jjj));
				}

				hist[im.at<unsigned char>(iii,jjj)]++;

			}

		}
	}

	unsigned int intensityCount = 0;
	int intensityPtr = -1;
	unsigned int medianCount = countNonZero(mask);

	while (intensityCount < (medianCount/2)) {
		intensityPtr++;
		intensityCount += hist[intensityPtr];
	}

	vals[3] = intensityPtr;


}

void obtainEightBitRepresentation(cv::Mat& src, cv::Mat& dst) {
	if (src.type() == CV_8UC1) {
		dst = src;
	} else if (src.type() == CV_8UC3) {
		#ifdef _OPENCV_VERSION_3_PLUS_
		cvtColor(src, dst, cv::COLOR_RGB2GRAY);
		#else
		cvtColor(src, dst, CV_RGB2GRAY);
		#endif
	} else if (src.type() == CV_16UC1) {
		cv::Mat nMat;
		double currMin, currMax;
		minMaxLoc(src, &currMin, &currMax);
		//normalize_16(nMat, src, currMin, currMax);
		down_level(dst, nMat);
	} else if (src.type() == CV_16UC3) {
		cv::Mat nMat, tMat;
		#ifdef _OPENCV_VERSION_3_PLUS_
		cvtColor(src, tMat, cv::COLOR_RGB2GRAY);
		#else
		cvtColor(src, tMat, CV_RGB2GRAY);
		#endif
		double currMin, currMax;
		minMaxLoc(tMat, &currMin, &currMax);
		//normalize_16(nMat, tMat, currMin, currMax);
		down_level(dst, nMat);
	}
}

void obtainColorRepresentation(cv::Mat& src, cv::Mat& dst) {
	if (src.type() == CV_8UC1) {
		#ifdef _OPENCV_VERSION_3_PLUS_
		cvtColor(src, dst, cv::COLOR_GRAY2RGB);
		#else
		cvtColor(src, dst, CV_GRAY2RGB);
		#endif
	} else if (src.type() == CV_8UC3) {
		dst = src;
	} else if (src.type() == CV_16UC1) {
		cv::Mat nMat, tMat;
		double currMin, currMax;
		minMaxLoc(src, &currMin, &currMax);
		//normalize_16(nMat, src, currMin, currMax);
		down_level(tMat, nMat);
		#ifdef _OPENCV_VERSION_3_PLUS_
		cvtColor(tMat, dst, cv::COLOR_GRAY2RGB);
		#else
		cvtColor(tMat, dst, CV_GRAY2RGB);
		#endif
	} else if (src.type() == CV_16UC3) {
		cv::Mat nMat, tMat, tMat2;
		double currMin, currMax;
		#ifdef _OPENCV_VERSION_3_PLUS_
		cvtColor(src, tMat, cv::COLOR_RGB2GRAY);
		#else
		cvtColor(src, tMat, CV_RGB2GRAY);
		#endif
		minMaxLoc(tMat, &currMin, &currMax);
		//normalize_16(nMat, tMat, currMin, currMax);
		down_level(tMat2, nMat);
		#ifdef _OPENCV_VERSION_3_PLUS_
		cvtColor(tMat2, dst, cv::COLOR_GRAY2RGB);
		#else
		cvtColor(tMat2, dst, CV_GRAY2RGB);
		#endif
	}
}

void fix_bottom_right(cv::Mat& mat) {

	if (mat.type() == CV_16UC3) {
		for (int iii = (mat.cols-6); iii < mat.cols; iii++) {
			mat.at<cv::Vec3s>(mat.rows-1, iii)[0] = std::max( std::min(( (unsigned int) ((mat.at<cv::Vec3s>(mat.rows-2, iii)[0] + mat.at<cv::Vec3s>(mat.rows-1, iii-1)[0]) / 2) ), (unsigned int) 65535) , (unsigned int) 0);
			mat.at<cv::Vec3s>(mat.rows-1, iii)[1] = std::max( std::min(( (unsigned int) ((mat.at<cv::Vec3s>(mat.rows-2, iii)[1] + mat.at<cv::Vec3s>(mat.rows-1, iii-1)[1]) / 2) ), (unsigned int) 65535) , (unsigned int) 0);
			mat.at<cv::Vec3s>(mat.rows-1, iii)[2] = std::max( std::min(( (unsigned int) ((mat.at<cv::Vec3s>(mat.rows-2, iii)[2] + mat.at<cv::Vec3s>(mat.rows-1, iii-1)[2]) / 2) ), (unsigned int) 65535) , (unsigned int) 0);
		}
	} else if (mat.type() == CV_8UC3) {
		for (int iii = (mat.cols-6); iii < mat.cols; iii++) {
			mat.at<cv::Vec3b>(mat.rows-1, iii)[0] = std::max( std::min(( (unsigned int) ((mat.at<cv::Vec3b>(mat.rows-2, iii)[0] + mat.at<cv::Vec3b>(mat.rows-1, iii-1)[0]) / 2) ), (unsigned int) 255) , (unsigned int) 0);
			mat.at<cv::Vec3b>(mat.rows-1, iii)[1] = std::max( std::min(( (unsigned int) ((mat.at<cv::Vec3b>(mat.rows-2, iii)[1] + mat.at<cv::Vec3b>(mat.rows-1, iii-1)[1]) / 2) ), (unsigned int) 255) , (unsigned int) 0);
			mat.at<cv::Vec3b>(mat.rows-1, iii)[2] = std::max( std::min(( (unsigned int) ((mat.at<cv::Vec3b>(mat.rows-2, iii)[2] + mat.at<cv::Vec3b>(mat.rows-1, iii-1)[2]) / 2) ), (unsigned int) 255) , (unsigned int) 0);
		}
	} else if (mat.type() == CV_16UC1) { 
		for (int iii = (mat.cols-6); iii < mat.cols; iii++) {
			mat.at<unsigned short>(mat.rows-1, iii) = std::max( std::min(( (unsigned int) ((mat.at<unsigned short>(mat.rows-2, iii) + mat.at<unsigned short>(mat.rows-1, iii-1)) / 2) ), (unsigned int) 65535) , (unsigned int) 0);
		}
	} else if (mat.type() == CV_8UC1) {
		for (int iii = (mat.cols-6); iii < mat.cols; iii++) {
			mat.at<unsigned char>(mat.rows-1, iii) = std::max( std::min(( (unsigned int) ((mat.at<unsigned char>(mat.rows-2, iii) + mat.at<unsigned char>(mat.rows-1, iii-1)) / 2) ), (unsigned int) 255) , (unsigned int) 0);
		}
	}
	
	
}

void down_level(cv::Mat& dst, cv::Mat& src) {

    // Currently only works for a single channel image i.e. CV_16UC1
	dst = cv::Mat(src.rows, src.cols, CV_8UC1);

    //unsigned int val;
    for (int i = 0; i < dst.size().height; i++) {
        for (int j = 0; j < dst.size().width; j++) {
           dst.at<unsigned char>(i,j) = (src.at<unsigned short>(i,j)) / 256;
        }

    }
}

void reduceToPureImage(cv::Mat& dst, cv::Mat& src) {


	if (dst.cols == 0) {
		if (src.type() == CV_16UC3) {
			dst = cv::Mat(src.size(), CV_16UC1);
		} else if (src.type() == CV_8UC3) {
			dst = cv::Mat(src.size(), CV_8UC1);
		}

	}

		for (int iii = 0; iii < src.cols; iii++) {
			for (int jjj = 0; jjj < src.rows; jjj++) {
				if (src.type() == CV_16UC3) {
					if (((src.at<cv::Vec3s>(jjj,iii)[0]) == (src.at<cv::Vec3s>(jjj,iii)[1])) && ((src.at<cv::Vec3s>(jjj,iii)[2]) == (src.at<cv::Vec3s>(jjj,iii)[1]))) {
						dst.at<unsigned short>(jjj,iii) = src.at<cv::Vec3s>(jjj,iii)[0];
					} else {
						dst.at<unsigned short>(jjj,iii) = (unsigned short) (0.299 * (double) src.at<cv::Vec3s>(jjj,iii)[0] + 0.587 * (double) src.at<cv::Vec3s>(jjj,iii)[1] + 0.114 * (double) src.at<cv::Vec3s>(jjj,iii)[2]);
					}

				} else if (src.type() == CV_8UC3) {
					if (((src.at<cv::Vec3b>(jjj,iii)[0]) == (src.at<cv::Vec3b>(jjj,iii)[1])) && ((src.at<cv::Vec3b>(jjj,iii)[2]) == (src.at<cv::Vec3b>(jjj,iii)[1]))) {
						dst.at<unsigned char>(jjj,iii) = src.at<cv::Vec3b>(jjj,iii)[0];
					} else {
						dst.at<unsigned char>(jjj,iii) = (unsigned char) (0.299 * (double) src.at<cv::Vec3b>(jjj,iii)[0] + 0.587 * (double) src.at<cv::Vec3b>(jjj,iii)[1] + 0.114 * (double) src.at<cv::Vec3b>(jjj,iii)[2]);
					}

				}

			}
		}
}

void addBorder(cv::Mat& inputMat, int borderSize) {
    cv::Mat newMat = cv::Mat::zeros(inputMat.rows + 2*borderSize, inputMat.cols + 2*borderSize, CV_8UC3);

    for (int i = borderSize; i < newMat.rows-borderSize; i++) {
        for (int j = borderSize; j < newMat.cols-borderSize; j++) {
            newMat.at<cv::Vec3b>(i,j) = inputMat.at<cv::Vec3b>(i-borderSize, j-borderSize);
        }
    }

    newMat.copyTo(inputMat);

}

void normalize_64_vec(cv::Mat& dst, cv::Mat& src) {

    //printf("%s << Entered...\n", __FUNCTION__);

    dst = cv::Mat(src.size(), src.type());

    double minVal = 9e50, maxVal = -9e50;

    for (int i = 0; i < src.rows; i++) {
        for (int j = 0; j < src.cols; j++) {
            for (int k = 0; k < src.channels(); k++) {
                if (src.at<cv::Vec3d>(i,j)[k] > maxVal) {
                    maxVal = src.at<cv::Vec3d>(i,j)[k];
                }

                if (src.at<cv::Vec3d>(i,j)[k] < minVal) {
                    minVal = src.at<cv::Vec3d>(i,j)[k];
                }
            }


        }
    }

    for (int i = 0; i < src.rows; i++) {
        for (int j = 0; j < src.cols; j++) {
            for (int k = 0; k < src.channels(); k++) {
                dst.at<cv::Vec3d>(i,j)[k] = max(min(((src.at<cv::Vec3d>(i,j)[k] - minVal) / (maxVal - minVal)), 1.0),0.0);
            }


        }
    }
}

void normalize_16(cv::Mat& dst, const cv::Mat& src, double dblmin, double dblmax) {

    unsigned short minv, maxv;
    bool presetLimits;

    double lower_bound = 0.0, upper_bound = 65535.0;

    if ((dblmin > -1.0) && (dblmax > -1.0)) {
        presetLimits = true;
        //cout << "preset == true" << endl;
    } else {
		presetLimits = false;
	}

    minv = 65535;
	maxv = 0;

    unsigned short val; //, new_val;

    cv::Size matSize = src.size();
    src.copyTo(dst);

    // Safety checks
	if ((src.size().height != dst.size().height) || (src.size().width != dst.size().width)) {
		printf("svLib::normalize_16() << Image dimensions don't match.\n");
		return;
	}

    if (1) { // (!presetLimits) {

        for (int i = 0; i < matSize.height; i++) {
            for (int j = 0; j < matSize.width; j++) {
                val = src.at<unsigned short>(i,j);             // how do I incorporate channels?

                if (val < minv) {
                    minv = val;
                }

                if (val > maxv) {
                    maxv = val;
                }
            }
        }

	} else {
		minv = (unsigned short) dblmin;
		maxv = (unsigned short) dblmax;
	}
	
	if (!presetLimits) {
		dblmin = double(minv);
		dblmax = double(maxv);
	}

	#pragma omp parallel for
	for (int i = 0; i < matSize.height; i++) {
        for (int j = 0; j < matSize.width; j++) {
            dst.at<unsigned short>(i,j) = std::max((unsigned short) dblmin, std::min((unsigned short) dblmax, src.at<unsigned short>(i,j)));
		}
	}

	double total_range = dblmax - dblmin;

	if (double(minv) > dblmin) {
		lower_bound = 65535.0 * (abs(double(minv) - dblmin) / total_range);
	}

	if (double(maxv) < dblmax) {
		upper_bound = 65535.0 - 65535.0 * (abs(double(maxv) - dblmax) / total_range);
	}

	normalize(dst, dst, lower_bound, upper_bound, cv::NORM_MINMAX);

	return;

}

void generateHistogram(cv::Mat& src, cv::Mat& dst, double* im_hist, double* im_summ, double* im_stat) {

    cv::MatND hist;
    cv::Mat img;

    int nPixels = src.cols*src.rows;

    //printf("%s << src.depth() = %d\n", __FUNCTION__, src.depth());

    src.convertTo(img, CV_32FC1);

    //svLib::normalize_16(src, img);  // no longer old

    //imshow("converted", img);
    //waitKey(0);

    double minIntensity = 9e50, maxIntensity=0;
    minMaxLoc(img, &minIntensity, &maxIntensity, 0, 0);

    double midIntensity = (maxIntensity + minIntensity) / 2.0;
    double intensityRange = maxIntensity-minIntensity;

    if (intensityRange < 64) {
        minIntensity = midIntensity - 32;
        maxIntensity = midIntensity + 32;
    }

    //double newIntensityRange = maxIntensity-minIntensity;

    printf("%s << intensity range = %f (%f, %f)\n", __FUNCTION__, intensityRange, minIntensity, maxIntensity);

    int intensityBins = 64;
    int histSize[] = {intensityBins};
    float intensityRanges[] = {float(minIntensity), float(maxIntensity)};
    const float* ranges[] = {intensityRanges};
    int channels[] = {0};

    calcHist(&img, 1, channels, cv::Mat(), hist, 1, histSize, ranges, true, false);

    double minVal = 9e50, maxVal=0.0;
    minMaxLoc(hist, &minVal, &maxVal, 0, 0);

    //printf("%s << minVal = %f; maxVal = %f\n", __FUNCTION__, minVal, maxVal);

    int horizontalScale = 640 / intensityBins;

    //printf("%s << horizontalScale = %d\n", __FUNCTION__, horizontalScale);
    int verticalScale = 480;
    cv::Mat histImg = cv::Mat::zeros(verticalScale, intensityBins*horizontalScale, CV_8UC3); //, cv::Vec3b::all(255));

    cv::Scalar col = cv::Scalar(255, 255, 255);

    histImg.setTo(col);

    //int quant01, quant05, quant50, quant95, quant99;

    double quantileCount = 0.0;

    for(int iii = 0; iii < intensityBins; iii++ ) {
        float binVal = hist.at<float>(iii, 0);
        float count = binVal/float(maxVal);

        quantileCount += binVal;

        /*
        printf("%s << iii = %d\n", __FUNCTION__, iii);
        printf("%s << binVal = %f\n", __FUNCTION__, binVal);
        printf("%s << fullCount = %f\n", __FUNCTION__, fullCount);
        */

        if (quantileCount < 0.01*double(nPixels)) {
            //quant01 = minIntensity + int(double(iii)*newIntensityRange/double(intensityBins));
        }
        if (quantileCount < 0.05*double(nPixels)) {
            //quant05 = minIntensity + int(double(iii)*newIntensityRange/double(intensityBins));
        }
        if (quantileCount < 0.50*double(nPixels)) {
            //quant50 = minIntensity + int(double(iii)*newIntensityRange/double(intensityBins));
        }
        if (quantileCount < 0.95*double(nPixels)) {
            //quant95 = minIntensity + int(double(iii)*newIntensityRange/double(intensityBins));
        }
        if (quantileCount < 0.99*double(nPixels)) {
            //quant99 = minIntensity + int(double(iii)*newIntensityRange/double(intensityBins));
        }

        //printf("%s << count = %f\n", __FUNCTION__, count);

		#ifdef _OPENCV_VERSION_3_PLUS_
		rectangle(histImg, cv::Point(int(iii*horizontalScale+1), int(verticalScale)), cv::Point(int((iii+1)*horizontalScale-2), int((verticalScale-1)*(1-count)+1)), cv::Scalar(0, 64, 192), -1);
		#else
		rectangle(histImg, cv::Point(int(iii*horizontalScale+1), int(verticalScale)), cv::Point(int((iii+1)*horizontalScale-2), int((verticalScale-1)*(1-count)+1)), cv::Scalar(0, 64, 192), CV_FILLED);
		#endif

    }

    histImg.copyTo(dst);

    double histMean = 0, histDev = 0, histRMS = 0, histSkew = 0;

    double pixelCount = img.rows * img.cols;

    // Calculate histogram statistics:
    for (int iii = 0; iii < img.rows; iii++) {
        for (int jjj = 0; jjj < img.cols; jjj++) {
            histMean += img.at<float>(iii,jjj) / pixelCount;
        }
    }

    //printf("%s << histMean = %f\n", __FUNCTION__, histMean);
    im_stat[0] = histMean;

    for (int iii = 0; iii < img.rows; iii++) {
        for (int jjj = 0; jjj < img.cols; jjj++) {
            histDev += pow((img.at<float>(iii,jjj) - histMean), 2) / pixelCount;
        }
    }

    histDev = pow(histDev, 0.5);

    //printf("%s << histDev = %f\n", __FUNCTION__, histDev);
    im_stat[1] = histDev;

    for (int iii = 0; iii < img.rows; iii++) {
        for (int jjj = 0; jjj < img.cols; jjj++) {
            histRMS += pow(img.at<float>(iii,jjj), 2) / pixelCount;
        }
    }

    histRMS = pow(histRMS, 0.5);

    //printf("%s << histRMS = %f\n", __FUNCTION__, histRMS);

    im_stat[2] = histRMS;

    for (int iii = 0; iii < img.rows; iii++) {
        for (int jjj = 0; jjj < img.cols; jjj++) {
            histSkew += pow((img.at<float>(iii,jjj)- histMean) / histDev, 3) / pixelCount;
        }
    }

    //printf("%s << histSkew = %f\n", __FUNCTION__, histSkew);

    im_stat[3] = histSkew;

    //printf("%s << qrange_0_100 = %d\n", __FUNCTION__, int(intensityRange));

    //printf("%s << qrange_1_99 = %d\n", __FUNCTION__, quant99-quant01);

    //printf("%s << qrange_5_95 = %d\n", __FUNCTION__, quant95-quant05);

}

cv::Mat normForDisplay(cv::Mat origMat) {
    double minVal = std::numeric_limits<double>::max(), maxVal = -std::numeric_limits<double>::max();

    for (int iii = 0; iii < origMat.rows; iii++) {
        for (int jjj = 0; jjj < origMat.cols; jjj++) {
            if (origMat.at<double>(iii,jjj) > maxVal)  maxVal = origMat.at<double>(iii,jjj);
            if (origMat.at<double>(iii,jjj) < minVal) minVal = origMat.at<double>(iii,jjj);
        }
    }

    cv::Mat displayMat(origMat.size(), CV_8UC1);

    for (int iii = 0; iii < origMat.rows; iii++) {
        for (int jjj = 0; jjj < origMat.cols; jjj++) {
            displayMat.at<unsigned char>(iii,jjj) = (unsigned char) ((origMat.at<double>(iii,jjj) - minVal) * 255.0 / (maxVal - minVal));
        }
    }

    cv::Mat largerMat;
	displayMat.copyTo(largerMat);

	int scale_factor = 2;
	while ((largerMat.rows < MIN_PIXELS_PER_AXIS_FOR_DISPLAY) && (largerMat.cols < MIN_PIXELS_PER_AXIS_FOR_DISPLAY)) {
		resize(displayMat, largerMat, cv::Size(origMat.cols*scale_factor, origMat.rows*scale_factor), 0, 0, cv::INTER_NEAREST);
		scale_factor++;
	}

    return largerMat;
}
