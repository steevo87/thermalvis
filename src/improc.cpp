/*! \file	improc.cpp
 *  \brief	Definitions for image processing.
*/

#include "improc.hpp"

double scoreColorImage(const cv::Mat& src) {
	
	double score = 0.00;
	
	double percentileVals[3] = { 0.001, 0.500, 0.999 };
	double intensityVals[3];
	findPercentiles(src, intensityVals, percentileVals, 3);

	
	double range = max(abs(intensityVals[2] - intensityVals[1]), abs(intensityVals[0] - intensityVals[1]));
	
	//printf("%s << range = (%f)\n", __FUNCTION__, range);
	
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

	//printf("%s << coord = (%f, %f)\n", __FUNCTION__, coord.x, coord.y);

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
	//printf("%s << coord[3] = (%d, %d); (%f); (%f)\n", __FUNCTION__, dcoord[3].x, dcoord[3].y, val[3], dist[3]);

	//printf("%s << (%f, %f, %f, %f) : (%f, %f, %f, %f) : (%f)\n", __FUNCTION__, val[0], val[1], val[2], val[3], dist[0], dist[1], dist[2], dist[3], total_dist);

	cv::Point ref_coord = cv::Point(int(floor(coord.x)), int(floor(coord.y)));

	if (total_dist == 0.0) {
		retVal = val[0];
		//printf("%s << returning reference val...\n", __FUNCTION__);
		//printf("%s << (%f, %f, %f, %f) : (%f, %f) vs (%d, %d) : (%f)\n", __FUNCTION__, val[0], val[1], val[2], val[3], coord.x, coord.y, ref_coord.x, ref_coord.y, retVal);
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

	if (mat1.rows != mat2.rows) {
		return false;
	}

	if (mat1.cols != mat2.cols) {
		return false;
	}

	if (mat1.type() != mat2.type()) {
		return false;
	}

	if ((mat1.type() != CV_16UC1) && (mat1.type() != CV_16SC1) && (mat1.type() != CV_8UC1) && (mat1.type() != CV_8UC3) && (mat1.type() != CV_8SC1) && (mat1.type() != CV_16UC3) && (mat1.type() != CV_16SC3) && (mat1.type() != CV_64FC1) && (mat1.type() != CV_32FC1))  {
		printf("%s << ERROR! Equality check for this type (%d) has not been implemented!\n", __FUNCTION__, mat1.type());
		return false;
	}
	
	bool isStillValid = true;

	#pragma omp parallel for
	for (int iii = 0; iii < mat1.rows; iii++) {
		for (int jjj = 0; jjj < mat1.cols; jjj++) {
			
			if (!isStillValid) {
				break;
			}

			switch (mat1.type()) {
				case CV_64FC1:
					//printf("%s << type: CV_64FC1\n", __FUNCTION__);
					if (mat1.at<double>(iii,jjj) != mat2.at<double>(iii,jjj)) {
						//printf("%s << (%d) (%d) \n", __FUNCTION__, mat1.at<double>(iii,jjj), mat2.at<double>(iii,jjj));
						isStillValid = false;
					}
					break;
				case CV_32FC1:
					//printf("%s << type: CV_32FC1\n", __FUNCTION__);
					if (mat1.at<float>(iii,jjj) != mat2.at<float>(iii,jjj)) {
						//printf("%s << (%d) (%d) \n", __FUNCTION__, mat1.at<float>(iii,jjj), mat2.at<float>(iii,jjj));
						isStillValid = false;
					}
					break;
				case CV_16UC1:
					//printf("%s << type: CV_16UC1\n", __FUNCTION__);
					if (mat1.at<unsigned short>(iii,jjj) != mat2.at<unsigned short>(iii,jjj)) {
						//printf("%s << (%d) (%d) \n", __FUNCTION__, mat1.at<unsigned short>(iii,jjj), mat2.at<unsigned short>(iii,jjj));
						isStillValid = false;
					}
					break;
				case CV_16SC1:
					//printf("%s << type: CV_16SC1\n", __FUNCTION__);
					if (mat1.at<short>(iii,jjj) != mat2.at<short>(iii,jjj)) {
						isStillValid = false;
					}
					break;
				case CV_8UC1:
					//printf("%s << type: CV_8UC1\n", __FUNCTION__);
					if (mat1.at<unsigned char>(iii,jjj) != mat2.at<unsigned char>(iii,jjj)) {
						//printf("%s << (%d) (%d) \n", __FUNCTION__, mat1.at<unsigned char>(iii,jjj), mat2.at<unsigned char>(iii,jjj));
						isStillValid = false;
					}
					break;
				case CV_8UC3:
					//printf("%s << type: CV_8UC3\n", __FUNCTION__);
					if ((mat1.at<cv::Vec3b>(iii,jjj)[0] != mat2.at<cv::Vec3b>(iii,jjj)[0]) || (mat1.at<cv::Vec3b>(iii,jjj)[1] != mat2.at<cv::Vec3b>(iii,jjj)[1]) || (mat1.at<cv::Vec3b>(iii,jjj)[2] != mat2.at<cv::Vec3b>(iii,jjj)[2])) {
						isStillValid = false;
					}
					break;
				case CV_8SC1:
					//printf("%s << type: CV_8SC1\n", __FUNCTION__);
					if (mat1.at<char>(iii,jjj) != mat2.at<char>(iii,jjj)) {
						isStillValid = false;
					}
					break;
				case CV_16UC3:
					//printf("%s << type: CV_16UC3\n", __FUNCTION__);
					if ((mat1.at<cv::Vec3s>(iii,jjj)[0] != mat2.at<cv::Vec3s>(iii,jjj)[0]) || (mat1.at<cv::Vec3s>(iii,jjj)[1] != mat2.at<cv::Vec3s>(iii,jjj)[1]) || (mat1.at<cv::Vec3s>(iii,jjj)[2] != mat2.at<cv::Vec3s>(iii,jjj)[2])) {
						isStillValid = false;
					}
					break;
				case CV_16SC3:
					//printf("%s << type: CV_16SC3\n", __FUNCTION__);
					if ((mat1.at<cv::Vec3s>(iii,jjj)[0] != mat2.at<cv::Vec3s>(iii,jjj)[0]) || (mat1.at<cv::Vec3s>(iii,jjj)[1] != mat2.at<cv::Vec3s>(iii,jjj)[1]) || (mat1.at<cv::Vec3s>(iii,jjj)[2] != mat2.at<cv::Vec3s>(iii,jjj)[2])) {
						isStillValid = false;
					}
					break;
				default:
					break;
			}
		}
	}

	if (!isStillValid) {
		return false;
	}
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
	
	if ((int(ksize) % 2) == 0) {
		ksize++;
	}
	
	//double bilateralVal = param;
	
	if (filter == GAUSSIAN_FILTERING) {
		GaussianBlur(src, dst, cv::Size(ksize, ksize), sqrt(param));
	} else if (filter == BILATERAL_FILTERING) {
		bilateralFilter(src, dst, ksize, param * 2.0, param * 2.0); //, double sigmaColor, double sigmaSpace, int borderType=BORDER_DEFAULT );
	} else {
		src.copyTo(dst);
	}
	
}

/*
void straightCLAHE(const cv::Mat& src, cv::Mat& dst, double factor) {
	
	int xdivs = 2;
	int ydivs = 2;
	int bins = 256;
	//int limit_counter = 255.0 * factor;
	
	IplImage tmp_ipl(src);
	
	
	IplImage* dst_ipl = cvCreateImage(cvSize(tmp_ipl.width,tmp_ipl.height), tmp_ipl.depth, 1);
	dst_ipl->origin = tmp_ipl.origin;
	  
	//IplImage dst_ipl;
	
	#ifdef USE_CLAHE
	cvCLAdaptEqualize(&tmp_ipl, dst_ipl, (unsigned int) xdivs, (unsigned int) ydivs, 
					(unsigned int) bins, (1.0 + factor * 14.0), CV_CLAHE_RANGE_FULL);
	// (float) limit_counter * 0.1
	#endif
	
	
	dst = cv::Mat(dst_ipl);
	
}
*/

/*
void downsampleCLAHE(const cv::Mat& src, cv::Mat& dst, double factor) {
	
	cv::Mat tmp;
	
	if (factor == 0.0) {
		
		adaptiveDownsample(src, dst, NORMALIZATION_STANDARD, 0.001);
	
		return;
	}
	
	// from: https://github.com/joshdoe/opencv-clahe

	// ...
	
	//printf("%s << entered...\n", __FUNCTION__);
	
	cv::Mat tmp_2;
	
	adaptiveDownsample(src, tmp_2, NORMALIZATION_STANDARD, 0.001);
	
	straightCLAHE(tmp_2, dst, factor);
	
	
	//int xdivs = 2;
	//int ydivs = 2;
	//int bins = 256;
	////int limit_counter = 255.0 * factor;
	
	//IplImage tmp_ipl(tmp_2);
	
	
	//IplImage* dst_ipl = cvCreateImage(cvSize(tmp_ipl.width,tmp_ipl.height), tmp_ipl.depth, 1);
	//dst_ipl->origin = tmp_ipl.origin;
	  
	////IplImage dst_ipl;
	
	//cvCLAdaptEqualize(&tmp_ipl, dst_ipl, (unsigned int) xdivs, (unsigned int) ydivs, 
					//(unsigned int) bins, (1.0 + factor * 14.0), CV_CLAHE_RANGE_FULL);
	//// (float) limit_counter * 0.1
	
	
	//dst = Mat(dst_ipl);
	
	////tmp.copyTo(dst);
	
}
*/

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

void temperatureRangeBasedDownsample(const cv::Mat& src, cv::Mat& dst, int newMedian, double degreesPerGraylevel, double desiredDegreesPerGraylevel) {

	if (dst.rows == 0) dst = cv::Mat::zeros(src.size(), CV_8UC1);

	if (newMedian == -1) {
		double percentile_levels[1], percentile_values[1];
		percentile_levels[0] = 0.5;
		findPercentiles(src, percentile_values, percentile_levels, 2);
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

    if ((exLeft > exRight) || (exTop > exBot)) {
        return 0.0;
    }

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

    //imshow("cropping", image);
    //waitKey(0);

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

        //imshow("cropping", dst);
        //waitKey(0);

        // cut down height to desired height
        int leftX = (dst.cols - width)/2;
        int rightX = leftX + width;

        printf("%s << leftX  = %d; rightX = %d\n", __FUNCTION__, leftX , rightX);

        cropImage(dst, cv::Point(leftX, 0), cv::Point(rightX, height));

        //imshow("cropping", dst);
        //waitKey(0);

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

	printf("%s << percentileVals(%f, %f, %f, %f, %f) = (%d, %d, %d, %d, %d)\n", __FUNCTION__, percentileVals[0], percentileVals[1], percentileVals[2], percentileVals[3], percentileVals[4], intensityVals[0], intensityVals[1], intensityVals[2], intensityVals[3], intensityVals[4]);

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
	} else {
		src.convertTo(dst, CV_8UC1);
	}
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

	vals[0] = 9e99;
	vals[1] = -9e99;
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

void cScheme::setupLookupTable(unsigned int depth) {

	//printf("%s << Entered\n", __FUNCTION__);

	unsigned int maxIndex, level;

	if (depth == 2) {
		maxIndex = 65535;
	} else if (depth == 1) {
		maxIndex = 255;
	} else {
		maxIndex = 255;
	}

	for (unsigned int iii = 0; iii <= maxIndex; iii++) {

		level = (int) floor(double((iii*(MAP_LENGTH-1))/((double) (maxIndex)))); // for 16 bits/pixel

		if (depth == 2) {
			lookupTable_2[iii][2] = (unsigned short) (255.0 * red[level]);
			lookupTable_2[iii][1] = (unsigned short) (255.0 * green[level]);
			lookupTable_2[iii][0] = (unsigned short) (255.0 * blue[level]);

			//printf("%s << LT(%d) = (%d, %d, %d)\n", __FUNCTION__, iii, lookupTable_2[iii][2], lookupTable_2[iii][1], lookupTable_2[iii][0]);
		} else if (depth == 1) {
			lookupTable_1[iii][2] = (unsigned char) (255.0 * red[level]);
			lookupTable_1[iii][1] = (unsigned char) (255.0 * green[level]);
			lookupTable_1[iii][0] = (unsigned char) (255.0 * blue[level]);
			
			/*
			if (iii == int(maxIndex/2)) {
				printf("%s << LT(%d)/(%d) = (%f, %f, %f) -> (%d, %d, %d)\n", __FUNCTION__, iii, level, red[level], green[level], blue[level], lookupTable_1[iii][2], lookupTable_1[iii][1], lookupTable_1[iii][0]);
			}
			*/
		}


	}

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

cScheme::cScheme() {
	load_standard(100);
}

cScheme::~cScheme() { }

void cScheme::create_long_map() {

	int element;

	int segs;
	double fact;

	segs = length-1;
	//M = (double) MAP_LENGTH;

	

	// Fill in initial elements
	red[0] = rx[0];
	green[0] = gx[0];
	blue[0] = bx[0];

	//printf("%s << segs = (%d), length = (%d), sample(%d) = (%f, %f, %f)\n", __FUNCTION__, segs, length, int(length / 2), red[int(length / 2)], green[int(length / 2)], blue[int(length / 2)]);
	
	int start_el = 0, finish_el;
	
	//cout << "Starting interpolation..." << endl;

	// Interpolate
	for (int iii = 0; iii < segs; iii++) {
	
		//cout << "start_el = " << start_el << endl;
		
		//cout << "dx[iii] = " << dx[iii] << endl;
		//cout << "dx[iii+1] = " << dx[iii+1] << endl;
		//cout << "MAP_LENGTH = " << MAP_LENGTH << endl;
		
		finish_el = int(dx[iii+1] * double(MAP_LENGTH-1));
		
		//cout << "finish_el = " << finish_el << endl;
		
		for (int jjj = start_el; jjj < finish_el; jjj++) {
			
			element = iii;
			
			//e = (double) element;
			//iy = (double) jjj;
			fact = (double(jjj) - double(start_el)) / (double(finish_el) - double(start_el));
			
			//cout << "jjj = " << jjj << endl;
			//cout << "fact = " << fact << endl;
			
			
			//cout << "start_el = " << start_el << endl;
			
			//cout << "rx[element] = " << rx[element] << endl;
			//cout << "gx[element] = " << gx[element] << endl;
			//cout << "bx[element] = " << bx[element] << endl;

			red[jjj] = rx[element] + fact*(rx[element+1] - rx[element]);
			green[jjj] = gx[element] + fact*(gx[element+1] - gx[element]);
			blue[jjj] = bx[element] + fact*(bx[element+1] - bx[element]);
			
			//cout << "red[jjj] = " << red[jjj] << endl;
			//cout << "green[jjj] = " << green[jjj] << endl;
			//cout << "blue[jjj] = " << blue[jjj] << endl;
			
		}
		
		start_el = finish_el;
	}
	
	//cout << "Finishing interpolation..." << endl;
	

	// Fill in final elements
	red[MAP_LENGTH-1] = rx[length-1];
	green[MAP_LENGTH-1] = gx[length-1];
	blue[MAP_LENGTH-1] = bx[length-1];

}

// Should be:
// Every ten units from 100 - 990 corresponds to a new colour scheme
// 100*N + 2*v + a: v corresponds to a different 'variation'; a can equal 1 or 0 - if 1, can include black
// and/or white (a = 0 is basically a safe version [if needed] for image fusion)

cScheme::cScheme(int mapCode, int mapParam) {
	load_standard(mapCode, mapParam);
}

void cScheme::load_standard(int mapCode, int mapParam) {

	code = mapCode + mapParam;
	//printf("%s << Loading scheme (%d, %d) = (%d)\n", __FUNCTION__, mapCode, mapParam, code);

	bool flexibleMarkers = false;
	// monkey
	length = 256;
	{
		double rr[MAX_INPUT_LENGTH] = { 0.842177, 0.794977, 0.750163, 0.707767, 0.667803, 0.630272, 0.595162, 0.562454, 0.532123, 0.504139, 0.478468, 0.455072, 0.433912, 0.414943, 0.398116, 0.383375, 0.370656, 0.359886, 0.350982, 0.343846, 0.338373, 0.334443, 0.331931, 0.330701, 0.330613, 0.331523, 0.333288, 0.335765, 0.338815, 0.342302, 0.346096, 0.350074, 0.354119, 0.358123, 0.361987, 0.365620, 0.368941, 0.371881, 0.374382, 0.376396, 0.377888, 0.378834, 0.379224, 0.379055, 0.378337, 0.377089, 0.375339, 0.373119, 0.370469, 0.367432, 0.364056, 0.360387, 0.356475, 0.352367, 0.348177, 0.344033, 0.339930, 0.335863, 0.331828, 0.327822, 0.323845, 0.319895, 0.315974, 0.312085, 0.308239, 0.304624, 0.301316, 0.298312, 0.295589, 0.292946, 0.290334, 0.287754, 0.285221, 0.282813, 0.280575, 0.278556, 0.276814, 0.275416, 0.274438, 0.273965, 0.274086, 0.274893, 0.276466, 0.278358, 0.276193, 0.269380, 0.262945, 0.256883, 0.251183, 0.245828, 0.240792, 0.236049, 0.231567, 0.227315, 0.223260, 0.219371, 0.215616, 0.211965, 0.208390, 0.204865, 0.201365, 0.197869, 0.194355, 0.190492, 0.185659, 0.179914, 0.173325, 0.166000, 0.158005, 0.149365, 0.140095, 0.130191, 0.120303, 0.112076, 0.105348, 0.099834, 0.095248, 0.091329, 0.088284, 0.085891, 0.081563, 0.074709, 0.065822, 0.055525, 0.043346, 0.029178, 0.014689, 0.000000, 0.000251, 0.018106, 0.036585, 0.053895, 0.068328, 0.081397, 0.093947, 0.106180, 0.118242, 0.131384, 0.146221, 0.162413, 0.179542, 0.197562, 0.215541, 0.232589, 0.248725, 0.263953, 0.278274, 0.291681, 0.304591, 0.317481, 0.330359, 0.343226, 0.356086, 0.368940, 0.381791, 0.394642, 0.407495, 0.420352, 0.433217, 0.446090, 0.458975, 0.471875, 0.485069, 0.498636, 0.512175, 0.525691, 0.539190, 0.552677, 0.566156, 0.579633, 0.593113, 0.606599, 0.620097, 0.633612, 0.647158, 0.660734, 0.674337, 0.687965, 0.701617, 0.715291, 0.728989, 0.742711, 0.756461, 0.770242, 0.784060, 0.797928, 0.811853, 0.825837, 0.839878, 0.853975, 0.868129, 0.882345, 0.896639, 0.911064, 0.925714, 0.940606, 0.952238, 0.960612, 0.968771, 0.976875, 0.984960, 0.993082, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 0.995905, 0.987520, 0.978928, 0.970117, 0.961069, 0.951756, 0.942144, 0.932186, 0.921823, 0.910991, 0.899620 };
		double gg[MAX_INPUT_LENGTH] = { 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 0.997193, 0.987772, 0.977945, 0.967751, 0.957223, 0.946391, 0.935282, 0.923916, 0.912314, 0.900494, 0.888469, 0.876254, 0.863862, 0.851304, 0.838592, 0.825738, 0.812754, 0.799653, 0.786450, 0.773159, 0.759799, 0.746387, 0.732946, 0.719498, 0.706067, 0.692679, 0.679361, 0.666139, 0.653042, 0.640094, 0.627321, 0.614744, 0.602383, 0.590254, 0.578368, 0.566732, 0.555350, 0.544221, 0.533338, 0.522692, 0.512272, 0.502062, 0.492045, 0.482203, 0.472517, 0.462933, 0.453383, 0.443871, 0.434400, 0.424976, 0.415601, 0.406280, 0.397015, 0.387812, 0.378673, 0.369599, 0.360503, 0.351344, 0.342116, 0.332815, 0.323467, 0.314071, 0.304619, 0.295099, 0.285488, 0.275762, 0.265891, 0.255836, 0.245550, 0.234967, 0.224007, 0.212563, 0.200497, 0.187641, 0.174266, 0.164865, 0.160603, 0.156434, 0.152357, 0.148375, 0.144486, 0.140688, 0.136979, 0.133355, 0.129811, 0.126341, 0.122938, 0.119595, 0.116304, 0.113057, 0.109845, 0.106661, 0.103495, 0.100341, 0.097456, 0.095335, 0.093882, 0.092971, 0.092454, 0.092203, 0.092108, 0.092078, 0.092034, 0.091657, 0.090349, 0.088259, 0.085536, 0.082298, 0.078629, 0.074017, 0.068165, 0.061625, 0.054555, 0.046727, 0.037788, 0.028338, 0.018891, 0.009446, 0.000000, 0.000160, 0.008872, 0.017691, 0.026453, 0.035166, 0.043509, 0.050663, 0.056788, 0.062048, 0.065920, 0.068030, 0.068653, 0.068291, 0.066748, 0.064598, 0.062725, 0.061358, 0.060744, 0.061117, 0.062683, 0.064964, 0.067306, 0.069710, 0.072180, 0.074716, 0.077319, 0.079991, 0.082730, 0.085535, 0.088406, 0.091340, 0.094335, 0.097389, 0.100497, 0.105193, 0.111848, 0.118233, 0.124379, 0.130311, 0.136052, 0.141624, 0.147045, 0.152335, 0.157511, 0.162589, 0.167591, 0.172561, 0.177501, 0.182411, 0.187293, 0.192148, 0.196982, 0.201804, 0.206624, 0.211456, 0.216317, 0.221229, 0.226253, 0.231410, 0.236711, 0.242158, 0.247746, 0.253489, 0.259410, 0.265607, 0.272360, 0.280060, 0.288737, 0.297592, 0.306705, 0.315913, 0.324994, 0.333996, 0.343098, 0.352261, 0.361527, 0.370938, 0.380468, 0.389927, 0.399285, 0.408544, 0.417698, 0.426737, 0.435925, 0.445700, 0.456026, 0.466869, 0.478203, 0.490003, 0.502247, 0.514911, 0.527972, 0.541405, 0.555182, 0.569274, 0.583734, 0.599017, 0.614443, 0.629991, 0.645638, 0.661362, 0.677139, 0.692949, 0.708770, 0.724580, 0.740345, 0.756043, 0.771656, 0.787163, 0.802548, 0.817793, 0.832881, 0.847796, 0.862520, 0.877037, 0.891329, 0.905377, 0.919160, 0.932656, 0.945842, 0.958689, 0.971169, 0.983249, 0.994896, 1.000000, 1.000000, 1.000000, 1.000000 };
		double bb[MAX_INPUT_LENGTH] = { 0.843510, 0.862611, 0.880857, 0.898336, 0.915130, 0.931314, 0.946956, 0.962114, 0.976843, 0.991186, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 0.990786, 0.979782, 0.968809, 0.957866, 0.946955, 0.936080, 0.925245, 0.914461, 0.903738, 0.893093, 0.882543, 0.872100, 0.861432, 0.847487, 0.829818, 0.811864, 0.793625, 0.775109, 0.756325, 0.737289, 0.718016, 0.698526, 0.678841, 0.658982, 0.638974, 0.618840, 0.598604, 0.578291, 0.557925, 0.537530, 0.517130, 0.496748, 0.476263, 0.455416, 0.434246, 0.412803, 0.391125, 0.369266, 0.347289, 0.325252, 0.303210, 0.281218, 0.259336, 0.237606, 0.216067, 0.194752, 0.173693, 0.153010, 0.133050, 0.115371, 0.100014, 0.086122, 0.072022, 0.056948, 0.038945, 0.019175, 0.000000, 0.000000, 0.014793, 0.029022, 0.042462, 0.053753, 0.062949, 0.070501, 0.076757, 0.081933, 0.086202, 0.089686, 0.092423, 0.094438, 0.095797, 0.096470, 0.096419, 0.095634, 0.094106, 0.091820, 0.088757, 0.085165, 0.081301, 0.077127, 0.072619, 0.067746, 0.062465, 0.056720, 0.050433, 0.043491, 0.035809, 0.027973, 0.020130, 0.012305, 0.004521, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.001181, 0.010993, 0.020202, 0.028872, 0.037072, 0.044670, 0.051287, 0.057190, 0.062583, 0.068320, 0.077287, 0.089581, 0.104867, 0.122799, 0.143094, 0.165557, 0.190073, 0.216592, 0.245115, 0.275679, 0.308349, 0.343211, 0.380366, 0.419924, 0.462001, 0.506715, 0.554177, 0.604492, 0.657741, 0.713974, 0.773186, 0.835275, 0.899958 };
		for (int iii = 0; iii < length; iii++) {
			rx[iii] = rr[iii];
			gx[iii] = gg[iii];
			bx[iii] = bb[iii];
		}
	}

	if (code == GRAYSCALE) {
		length = 2;
		double rr[MAX_INPUT_LENGTH] = { 0, 1 };
		double gg[MAX_INPUT_LENGTH] = { 0, 1 };
		double bb[MAX_INPUT_LENGTH] = { 0, 1 };
		for (int iii = 0; iii < length; iii++) {
			rx[iii] = rr[iii];
			gx[iii] = gg[iii];
			bx[iii] = bb[iii];
		}
	} else if (code == (GRAYSCALE + 1)) {
		length = 2;
		double rr[MAX_INPUT_LENGTH] = { 0.2, 0.8 };
		double gg[MAX_INPUT_LENGTH] = { 0.2, 0.8 };
		double bb[MAX_INPUT_LENGTH] = { 0.2, 0.8 };
		for (int iii = 0; iii < length; iii++) {
			rx[iii] = rr[iii];
			gx[iii] = gg[iii];
			bx[iii] = bb[iii];
		}
	} else if (code == HIGHLIGHTED) {
		length = 4;
		flexibleMarkers = true;
		double da[MAX_INPUT_LENGTH] = { 0.0, 0.5, 0.5, 1.0 };
		double rr[MAX_INPUT_LENGTH] = { 0, 1.0, 1.0, 1.0 };
		double gg[MAX_INPUT_LENGTH] = { 0, 1.0, 0.0, 0.0 };
		double bb[MAX_INPUT_LENGTH] = { 0, 1.0, 0.0, 0.0 };
		for (int iii = 0; iii < length; iii++) {
			dx[iii] = da[iii];
			rx[iii] = rr[iii];
			gx[iii] = gg[iii];
			bx[iii] = bb[iii];
		}
	} else if (code == (HIGHLIGHTED + 1)) {
		length = 4;
		flexibleMarkers = true;
		double da[MAX_INPUT_LENGTH] = { 0.25, 0.5, 0.5, 1.0 };
		double rr[MAX_INPUT_LENGTH] = { 0.25, 0.75, 1.0, 1.0 };
		double gg[MAX_INPUT_LENGTH] = { 0.25, 0.75, 0.0, 0.0 };
		double bb[MAX_INPUT_LENGTH] = { 0.25, 0.75, 0.0, 0.0 };
		for (int iii = 0; iii < length; iii++) {
			dx[iii] = da[iii];
			rx[iii] = rr[iii];
			gx[iii] = gg[iii];
			bx[iii] = bb[iii];
		}
	} else if (code == CIECOMP) {
		length = 33;
		double rr[MAX_INPUT_LENGTH] = { 1.000000, 0.765154, 0.514649, 0.264229, 0.082532, 0.004867, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000541, 0.062517, 0.246214, 0.493080, 0.743824, 0.931596, 0.996408, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000 };
		double gg[MAX_INPUT_LENGTH] = { 1.000000, 1.000000, 1.000000, 1.000000, 0.964432, 0.859914, 0.717154, 0.572201, 0.435801, 0.319831, 0.214279, 0.109202, 0.031115, 0.002438, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000889, 0.020277, 0.077613, 0.155367, 0.234768, 0.334034, 0.475222, 0.637346, 0.798860, 0.922554, 0.971514, 0.982362, 0.991155, 0.999981 };
		double bb[MAX_INPUT_LENGTH] = { 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 0.938789, 0.756126, 0.508812, 0.258234, 0.070001, 0.004301, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.001793, 0.069474, 0.251770, 0.498992, 0.749491, 1.000000 };
		for (int iii = 0; iii < length; iii++) {
			rx[iii] = rr[iii];
			gx[iii] = gg[iii];
			bx[iii] = bb[iii];
		}
	} else if (code == (CIECOMP + 1)) {
		length = 27;
		double rr[MAX_INPUT_LENGTH] = { 0.264229, 0.082532, 0.004867, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000541, 0.062517, 0.246214, 0.493080, 0.743824, 0.931596, 0.996408, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000 };
		double gg[MAX_INPUT_LENGTH] = { 1.000000, 0.964432, 0.859914, 0.717154, 0.572201, 0.435801, 0.319831, 0.214279, 0.109202, 0.031115, 0.002438, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000889, 0.020277, 0.077613, 0.155367, 0.234768, 0.334034, 0.475222, 0.637346, 0.798860, 0.922554, 0.971514 };
		double bb[MAX_INPUT_LENGTH] = { 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 0.938789, 0.756126, 0.508812, 0.258234, 0.070001, 0.004301, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.001793, 0.069474, 0.251770 };
		for (int iii = 0; iii < length; iii++) {
			rx[iii] = rr[iii];
			gx[iii] = gg[iii];
			bx[iii] = bb[iii];
		}
	} else if (code == CIELUV) {
		
		length = 256;
		double rr[MAX_INPUT_LENGTH] = { 0.842177, 0.794977, 0.750163, 0.707767, 0.667803, 0.630272, 0.595162, 0.562454, 0.532123, 0.504139, 0.478468, 0.455072, 0.433912, 0.414943, 0.398116, 0.383375, 0.370656, 0.359886, 0.350982, 0.343846, 0.338373, 0.334443, 0.331931, 0.330701, 0.330613, 0.331523, 0.333288, 0.335765, 0.338815, 0.342302, 0.346096, 0.350074, 0.354119, 0.358123, 0.361987, 0.365620, 0.368941, 0.371881, 0.374382, 0.376396, 0.377888, 0.378834, 0.379224, 0.379055, 0.378337, 0.377089, 0.375339, 0.373119, 0.370469, 0.367432, 0.364056, 0.360387, 0.356475, 0.352367, 0.348177, 0.344033, 0.339930, 0.335863, 0.331828, 0.327822, 0.323845, 0.319895, 0.315974, 0.312085, 0.308239, 0.304624, 0.301316, 0.298312, 0.295589, 0.292946, 0.290334, 0.287754, 0.285221, 0.282813, 0.280575, 0.278556, 0.276814, 0.275416, 0.274438, 0.273965, 0.274086, 0.274893, 0.276466, 0.278358, 0.276193, 0.269380, 0.262945, 0.256883, 0.251183, 0.245828, 0.240792, 0.236049, 0.231567, 0.227315, 0.223260, 0.219371, 0.215616, 0.211965, 0.208390, 0.204865, 0.201365, 0.197869, 0.194355, 0.190492, 0.185659, 0.179914, 0.173325, 0.166000, 0.158005, 0.149365, 0.140095, 0.130191, 0.120303, 0.112076, 0.105348, 0.099834, 0.095248, 0.091329, 0.088284, 0.085891, 0.081563, 0.074709, 0.065822, 0.055525, 0.043346, 0.029178, 0.014689, 0.000000, 0.000251, 0.018106, 0.036585, 0.053895, 0.068328, 0.081397, 0.093947, 0.106180, 0.118242, 0.131384, 0.146221, 0.162413, 0.179542, 0.197562, 0.215541, 0.232589, 0.248725, 0.263953, 0.278274, 0.291681, 0.304591, 0.317481, 0.330359, 0.343226, 0.356086, 0.368940, 0.381791, 0.394642, 0.407495, 0.420352, 0.433217, 0.446090, 0.458975, 0.471875, 0.485069, 0.498636, 0.512175, 0.525691, 0.539190, 0.552677, 0.566156, 0.579633, 0.593113, 0.606599, 0.620097, 0.633612, 0.647158, 0.660734, 0.674337, 0.687965, 0.701617, 0.715291, 0.728989, 0.742711, 0.756461, 0.770242, 0.784060, 0.797928, 0.811853, 0.825837, 0.839878, 0.853975, 0.868129, 0.882345, 0.896639, 0.911064, 0.925714, 0.940606, 0.952238, 0.960612, 0.968771, 0.976875, 0.984960, 0.993082, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 0.995905, 0.987520, 0.978928, 0.970117, 0.961069, 0.951756, 0.942144, 0.932186, 0.921823, 0.910991, 0.899620 };
		double gg[MAX_INPUT_LENGTH] = { 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 0.997193, 0.987772, 0.977945, 0.967751, 0.957223, 0.946391, 0.935282, 0.923916, 0.912314, 0.900494, 0.888469, 0.876254, 0.863862, 0.851304, 0.838592, 0.825738, 0.812754, 0.799653, 0.786450, 0.773159, 0.759799, 0.746387, 0.732946, 0.719498, 0.706067, 0.692679, 0.679361, 0.666139, 0.653042, 0.640094, 0.627321, 0.614744, 0.602383, 0.590254, 0.578368, 0.566732, 0.555350, 0.544221, 0.533338, 0.522692, 0.512272, 0.502062, 0.492045, 0.482203, 0.472517, 0.462933, 0.453383, 0.443871, 0.434400, 0.424976, 0.415601, 0.406280, 0.397015, 0.387812, 0.378673, 0.369599, 0.360503, 0.351344, 0.342116, 0.332815, 0.323467, 0.314071, 0.304619, 0.295099, 0.285488, 0.275762, 0.265891, 0.255836, 0.245550, 0.234967, 0.224007, 0.212563, 0.200497, 0.187641, 0.174266, 0.164865, 0.160603, 0.156434, 0.152357, 0.148375, 0.144486, 0.140688, 0.136979, 0.133355, 0.129811, 0.126341, 0.122938, 0.119595, 0.116304, 0.113057, 0.109845, 0.106661, 0.103495, 0.100341, 0.097456, 0.095335, 0.093882, 0.092971, 0.092454, 0.092203, 0.092108, 0.092078, 0.092034, 0.091657, 0.090349, 0.088259, 0.085536, 0.082298, 0.078629, 0.074017, 0.068165, 0.061625, 0.054555, 0.046727, 0.037788, 0.028338, 0.018891, 0.009446, 0.000000, 0.000160, 0.008872, 0.017691, 0.026453, 0.035166, 0.043509, 0.050663, 0.056788, 0.062048, 0.065920, 0.068030, 0.068653, 0.068291, 0.066748, 0.064598, 0.062725, 0.061358, 0.060744, 0.061117, 0.062683, 0.064964, 0.067306, 0.069710, 0.072180, 0.074716, 0.077319, 0.079991, 0.082730, 0.085535, 0.088406, 0.091340, 0.094335, 0.097389, 0.100497, 0.105193, 0.111848, 0.118233, 0.124379, 0.130311, 0.136052, 0.141624, 0.147045, 0.152335, 0.157511, 0.162589, 0.167591, 0.172561, 0.177501, 0.182411, 0.187293, 0.192148, 0.196982, 0.201804, 0.206624, 0.211456, 0.216317, 0.221229, 0.226253, 0.231410, 0.236711, 0.242158, 0.247746, 0.253489, 0.259410, 0.265607, 0.272360, 0.280060, 0.288737, 0.297592, 0.306705, 0.315913, 0.324994, 0.333996, 0.343098, 0.352261, 0.361527, 0.370938, 0.380468, 0.389927, 0.399285, 0.408544, 0.417698, 0.426737, 0.435925, 0.445700, 0.456026, 0.466869, 0.478203, 0.490003, 0.502247, 0.514911, 0.527972, 0.541405, 0.555182, 0.569274, 0.583734, 0.599017, 0.614443, 0.629991, 0.645638, 0.661362, 0.677139, 0.692949, 0.708770, 0.724580, 0.740345, 0.756043, 0.771656, 0.787163, 0.802548, 0.817793, 0.832881, 0.847796, 0.862520, 0.877037, 0.891329, 0.905377, 0.919160, 0.932656, 0.945842, 0.958689, 0.971169, 0.983249, 0.994896, 1.000000, 1.000000, 1.000000, 1.000000 };
		double bb[MAX_INPUT_LENGTH] = { 0.843510, 0.862611, 0.880857, 0.898336, 0.915130, 0.931314, 0.946956, 0.962114, 0.976843, 0.991186, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 0.990786, 0.979782, 0.968809, 0.957866, 0.946955, 0.936080, 0.925245, 0.914461, 0.903738, 0.893093, 0.882543, 0.872100, 0.861432, 0.847487, 0.829818, 0.811864, 0.793625, 0.775109, 0.756325, 0.737289, 0.718016, 0.698526, 0.678841, 0.658982, 0.638974, 0.618840, 0.598604, 0.578291, 0.557925, 0.537530, 0.517130, 0.496748, 0.476263, 0.455416, 0.434246, 0.412803, 0.391125, 0.369266, 0.347289, 0.325252, 0.303210, 0.281218, 0.259336, 0.237606, 0.216067, 0.194752, 0.173693, 0.153010, 0.133050, 0.115371, 0.100014, 0.086122, 0.072022, 0.056948, 0.038945, 0.019175, 0.000000, 0.000000, 0.014793, 0.029022, 0.042462, 0.053753, 0.062949, 0.070501, 0.076757, 0.081933, 0.086202, 0.089686, 0.092423, 0.094438, 0.095797, 0.096470, 0.096419, 0.095634, 0.094106, 0.091820, 0.088757, 0.085165, 0.081301, 0.077127, 0.072619, 0.067746, 0.062465, 0.056720, 0.050433, 0.043491, 0.035809, 0.027973, 0.020130, 0.012305, 0.004521, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.001181, 0.010993, 0.020202, 0.028872, 0.037072, 0.044670, 0.051287, 0.057190, 0.062583, 0.068320, 0.077287, 0.089581, 0.104867, 0.122799, 0.143094, 0.165557, 0.190073, 0.216592, 0.245115, 0.275679, 0.308349, 0.343211, 0.380366, 0.419924, 0.462001, 0.506715, 0.554177, 0.604492, 0.657741, 0.713974, 0.773186, 0.835275, 0.899958 };
		for (int iii = 0; iii < length; iii++) {
			rx[iii] = rr[iii];
			gx[iii] = gg[iii];
			bx[iii] = bb[iii];
		}
	} else if (code == (CIELUV + 1)) {
		
		length = 256;
		double rr[MAX_INPUT_LENGTH] = { 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.054719, 0.099579, 0.132524, 0.160076, 0.184437, 0.206645, 0.227268, 0.246651, 0.265009, 0.282477, 0.299142, 0.315054, 0.330238, 0.344699, 0.358430, 0.371414, 0.383626, 0.395038, 0.405620, 0.415342, 0.424175, 0.432096, 0.439085, 0.445130, 0.450225, 0.454373, 0.457583, 0.459874, 0.461272, 0.461810, 0.461525, 0.460461, 0.458665, 0.456187, 0.453077, 0.449386, 0.445162, 0.440454, 0.435306, 0.429820, 0.424110, 0.418167, 0.411980, 0.405537, 0.398828, 0.391839, 0.384558, 0.376969, 0.369056, 0.360811, 0.352436, 0.344007, 0.335514, 0.326940, 0.318272, 0.309490, 0.300575, 0.291539, 0.282529, 0.273618, 0.264889, 0.256442, 0.248393, 0.240877, 0.234041, 0.228044, 0.223040, 0.219163, 0.219144, 0.220152, 0.218614, 0.218585, 0.220007, 0.222757, 0.226671, 0.231552, 0.237187, 0.243365, 0.249884, 0.256560, 0.263229, 0.269748, 0.275999, 0.281881, 0.287316, 0.292238, 0.296600, 0.300365, 0.303508, 0.306014, 0.307876, 0.309091, 0.309665, 0.309606, 0.308927, 0.307644, 0.305773, 0.303334, 0.300347, 0.296833, 0.292814, 0.288311, 0.283352, 0.278286, 0.273324, 0.268495, 0.263833, 0.259371, 0.255145, 0.251190, 0.247543, 0.244237, 0.241305, 0.241363, 0.245876, 0.251343, 0.257741, 0.265034, 0.273171, 0.282099, 0.291756, 0.302079, 0.313007, 0.324481, 0.336297, 0.348161, 0.360057, 0.371973, 0.383899, 0.395826, 0.407749, 0.419660, 0.431554, 0.443427, 0.455276, 0.467099, 0.478894, 0.490660, 0.502398, 0.514107, 0.525789, 0.537446, 0.549080, 0.560693, 0.572289, 0.583869, 0.595439, 0.607000, 0.618557, 0.630113, 0.641671, 0.653236, 0.664810, 0.676396, 0.687998, 0.699618, 0.711258, 0.722921, 0.734609, 0.746329, 0.758080, 0.769860, 0.781665, 0.793496, 0.805349, 0.817225, 0.829122, 0.841041, 0.852983, 0.864950, 0.876944, 0.888968, 0.901021, 0.913100, 0.925203, 0.937325, 0.949466, 0.961619, 0.973774, 0.985928, 0.998102, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 0.991921, 0.982232, 0.972473, 0.962648, 0.952767, 0.942903, 0.933098 };
		double gg[MAX_INPUT_LENGTH] = { 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 0.988214, 0.972972, 0.957391, 0.943527, 0.929950, 0.916226, 0.902349, 0.888316, 0.874124, 0.859773, 0.845260, 0.830589, 0.815761, 0.800784, 0.785664, 0.770412, 0.755043, 0.739572, 0.724020, 0.708409, 0.692766, 0.677118, 0.661496, 0.645934, 0.630463, 0.615118, 0.599932, 0.584935, 0.570155, 0.555619, 0.541346, 0.527353, 0.513651, 0.500245, 0.487135, 0.474318, 0.461783, 0.449519, 0.437507, 0.425730, 0.414166, 0.402794, 0.391546, 0.380338, 0.369176, 0.358063, 0.347005, 0.336007, 0.325073, 0.314209, 0.303421, 0.292714, 0.282090, 0.271423, 0.260650, 0.249758, 0.238731, 0.227549, 0.216190, 0.204627, 0.192817, 0.180682, 0.168141, 0.155084, 0.141361, 0.126758, 0.110953, 0.093423, 0.073213, 0.048211, 0.016697, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.001846, 0.043999, 0.079778, 0.110058, 0.135213, 0.157403, 0.177619, 0.196537, 0.214580, 0.231988, 0.248938, 0.265564, 0.281971, 0.298244, 0.314450, 0.330646, 0.346876, 0.363178, 0.379581, 0.396107, 0.412772, 0.429587, 0.446557, 0.463681, 0.480955, 0.498372, 0.515920, 0.533583, 0.551346, 0.569189, 0.587092, 0.605034, 0.622993, 0.640948, 0.658878, 0.676762, 0.694581, 0.712318, 0.729956, 0.747481, 0.764878, 0.782138, 0.799251, 0.816208, 0.833003, 0.849631, 0.866090, 0.882499, 0.899587, 0.916476, 0.933164, 0.949649, 0.965932, 0.982013, 0.997891, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000 };
		double bb[MAX_INPUT_LENGTH] = { 0.945402, 0.957936, 0.970650, 0.983509, 0.996407, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 0.991458, 0.974268, 0.956565, 0.938351, 0.919639, 0.900449, 0.880809, 0.860756, 0.840330, 0.819579, 0.798553, 0.777306, 0.755894, 0.734375, 0.712805, 0.691241, 0.669738, 0.648349, 0.627124, 0.606111, 0.585354, 0.564893, 0.544766, 0.525006, 0.505643, 0.486701, 0.468204, 0.450169, 0.432612, 0.415545, 0.398976, 0.382913, 0.367360, 0.352446, 0.338263, 0.324839, 0.312197, 0.300362, 0.289354, 0.279190, 0.269884, 0.261446, 0.253880, 0.253758, 0.259407, 0.264843, 0.270034, 0.274960, 0.279608, 0.283974, 0.288060, 0.291875, 0.295431, 0.298748, 0.301747, 0.304273, 0.306336, 0.307953, 0.309141, 0.309918, 0.310301, 0.310309, 0.309960, 0.309275, 0.308272, 0.306974, 0.305400, 0.303572, 0.301513, 0.299245, 0.296790, 0.294170, 0.291408, 0.288526, 0.285543, 0.282481, 0.279357, 0.276188, 0.272989, 0.269771, 0.266544, 0.263314, 0.260084, 0.256851, 0.253613, 0.250360, 0.247080, 0.243757, 0.240359, 0.236799, 0.233074, 0.229189, 0.225147, 0.220944, 0.216568, 0.211998, 0.207200, 0.202128, 0.196719, 0.190889, 0.184426, 0.177218, 0.169168, 0.160198, 0.150252, 0.139185, 0.126802, 0.112619, 0.095469, 0.073767, 0.040440, 0.000000, 0.000000, 0.002703, 0.012629, 0.022977, 0.032503, 0.041849, 0.050711, 0.058910, 0.066584, 0.073818, 0.080671, 0.087179, 0.093367, 0.099248, 0.104830, 0.110113, 0.115095, 0.119768, 0.124123, 0.128147, 0.131827, 0.135146, 0.138088, 0.140635, 0.142768, 0.144467, 0.145711, 0.146481, 0.146753, 0.146507, 0.145717, 0.144359, 0.142406, 0.139826, 0.136582, 0.132633, 0.127925, 0.122391, 0.115944, 0.108464, 0.099782, 0.089648, 0.077664, 0.063131, 0.044608, 0.021258, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000 };
		for (int iii = 0; iii < length; iii++) {
			rx[iii] = rr[iii];
			gx[iii] = gg[iii];
			bx[iii] = bb[iii];
		}
	} else if (code == CIECOMP_ALT_1) {
		length = 33;
		double rr[MAX_INPUT_LENGTH] = { 0.999951, 0.764763, 0.513833, 0.262280, 0.070539, 0.004449, 0.000982, 0.000000, 0.014764, 0.079511, 0.164042, 0.247835, 0.342405, 0.474597, 0.620949, 0.766563, 0.888141, 0.936537, 0.955818, 0.977795, 0.996042, 1.000000, 0.999773, 0.999860, 1.000000, 0.999750, 0.999432, 0.999660, 0.999868, 0.999382, 0.999179, 0.999381, 0.999959 };
		double gg[MAX_INPUT_LENGTH] = { 1.000000, 1.000000, 1.000000, 0.997055, 0.970402, 0.863627, 0.716516, 0.570543, 0.432488, 0.318623, 0.214564, 0.108254, 0.027442, 0.002708, 0.000136, 0.000000, 0.000011, 0.000033, 0.000094, 0.000746, 0.017982, 0.076261, 0.155089, 0.233998, 0.330934, 0.473361, 0.636642, 0.802257, 0.931872, 0.974355, 0.982178, 0.991650, 1.000000 };
		double bb[MAX_INPUT_LENGTH] = { 1.000000, 1.000000, 1.000000, 1.000000, 0.999997, 0.999995, 0.999977, 0.999922, 0.999902, 0.999551, 0.999534, 0.998992, 0.999522, 0.999971, 0.999664, 0.999221, 0.950204, 0.758209, 0.505805, 0.252367, 0.056835, 0.003052, 0.000875, 0.000378, 0.000067, 0.000073, 0.000477, 0.004040, 0.061700, 0.249519, 0.498347, 0.748001, 0.997975 };
		for (int iii = 0; iii < length; iii++) {
			rx[iii] = rr[iii];
			gx[iii] = gg[iii];
			bx[iii] = bb[iii];
		}
	} else if (code == (CIECOMP_ALT_1 + 1)) {
		length = 33;
		double rr[MAX_INPUT_LENGTH] = { 0.000021, 0.000002, 0.000016, 0.000234, 0.000162, 0.003354, 0.038452, 0.102054, 0.165385, 0.228699, 0.291374, 0.369664, 0.475506, 0.586630, 0.695216, 0.803475, 0.893515, 0.930827, 0.944857, 0.961729, 0.978009, 0.993506, 0.999136, 0.998879, 0.999137, 0.999905, 0.999888, 0.999957, 0.999987, 1.000000, 1.000000, 1.000000, 1.000000 };
		double gg[MAX_INPUT_LENGTH] = { 1.000000, 0.898136, 0.788027, 0.679369, 0.570642, 0.461780, 0.370245, 0.292206, 0.213912, 0.134749, 0.055599, 0.007772, 0.000272, 0.000008, 0.000004, 0.000009, 0.000007, 0.000014, 0.000011, 0.000027, 0.000088, 0.004334, 0.037984, 0.096488, 0.155866, 0.214581, 0.274564, 0.358132, 0.474443, 0.596959, 0.719264, 0.841713, 0.964617 };
		double bb[MAX_INPUT_LENGTH] = { 0.999914, 1.000000, 1.000000, 0.999790, 0.998744, 0.998603, 0.999497, 0.999707, 0.999911, 0.999704, 0.999322, 0.999702, 1.000000, 0.999979, 0.999586, 0.998111, 0.956023, 0.819800, 0.630929, 0.440522, 0.253413, 0.086470, 0.011451, 0.000127, 0.000003, 0.000001, 0.000015, 0.000453, 0.001038, 0.000450, 0.000225, 0.000199, 0.000161 };
		for (int iii = 0; iii < length; iii++) {
			rx[iii] = rr[iii];
			gx[iii] = gg[iii];
			bx[iii] = bb[iii];
		}
	} else if (code == CIECOMP_ALT_2) {
		length = 33;
		double rr[MAX_INPUT_LENGTH] = { 0.999911, 0.764767, 0.513903, 0.262540, 0.070481, 0.004493, 0.000952, 0.000000, 0.014727, 0.079215, 0.164105, 0.250787, 0.300361, 0.256821, 0.171949, 0.088134, 0.087815, 0.263848, 0.518803, 0.760556, 0.943220, 0.993926, 0.999249, 0.999896, 1.000000, 0.999897, 0.999731, 0.999881, 0.999938, 0.999386, 0.999076, 0.999067, 0.999460 };
		double gg[MAX_INPUT_LENGTH] = { 1.000000, 0.999999, 0.999994, 0.997249, 0.969474, 0.863789, 0.716632, 0.570567, 0.432580, 0.318934, 0.214508, 0.108219, 0.027005, 0.001377, 0.000081, 0.000009, 0.000000, 0.000031, 0.000254, 0.000000, 0.018056, 0.077171, 0.155264, 0.234004, 0.331232, 0.473361, 0.636852, 0.802178, 0.931685, 0.974284, 0.982440, 0.991755, 1.000000 };
		double bb[MAX_INPUT_LENGTH] = { 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 0.999990, 0.999945, 0.999878, 0.999520, 0.999207, 0.994201, 0.946503, 0.762066, 0.509103, 0.254220, 0.060146, 0.006524, 0.000870, 0.000094, 0.000032, 0.000035, 0.000022, 0.000005, 0.000019, 0.000073, 0.000473, 0.005081, 0.060686, 0.248644, 0.498482, 0.748191, 0.998655 };
		for (int iii = 0; iii < length; iii++) {
			rx[iii] = rr[iii];
			gx[iii] = gg[iii];
			bx[iii] = bb[iii];
		}
	} else if (code == (CIECOMP_ALT_2 + 1)) {
		length = 26;
		double rr[MAX_INPUT_LENGTH] = { 0.513903, 0.262540, 0.070481, 0.004493, 0.000952, 0.000000, 0.014727, 0.079215, 0.164105, 0.250787, 0.300361, 0.256821, 0.171949, 0.518803, 0.760556, 0.943220, 0.993926, 0.999249, 0.999896, 1.000000, 0.999897, 0.999731, 0.999881, 0.999938, 0.999386, 0.999076 };
		double gg[MAX_INPUT_LENGTH] = { 0.999994, 0.997249, 0.969474, 0.863789, 0.716632, 0.570567, 0.432580, 0.318934, 0.214508, 0.108219, 0.027005, 0.001377, 0.000081, 0.000254, 0.000000, 0.018056, 0.077171, 0.155264, 0.234004, 0.331232, 0.473361, 0.636852, 0.802178, 0.931685, 0.974284, 0.982440 };
		double bb[MAX_INPUT_LENGTH] = { 1.000000, 1.000000, 1.000000, 1.000000, 0.999990, 0.999945, 0.999878, 0.999520, 0.999207, 0.994201, 0.946503, 0.762066, 0.509103, 0.000870, 0.000094, 0.000032, 0.000035, 0.000022, 0.000005, 0.000019, 0.000073, 0.000473, 0.005081, 0.060686, 0.248644, 0.498482 };
		for (int iii = 0; iii < length; iii++) {
			rx[iii] = rr[iii];
			gx[iii] = gg[iii];
			bx[iii] = bb[iii];
		}
	} else if (code == CIECOMP_ALT_3) {
		length = 33;
		double rr[MAX_INPUT_LENGTH] = { 0.000021, 0.000002, 0.000016, 0.000234, 0.000162, 0.003354, 0.038452, 0.102054, 0.165385, 0.228699, 0.291374, 0.369664, 0.475506, 0.586630, 0.695216, 0.803475, 0.893515, 0.930827, 0.944857, 0.961729, 0.978009, 0.993506, 0.999136, 0.998879, 0.999137, 0.999905, 0.999888, 0.999957, 0.999987, 1.000000, 1.000000, 1.000000, 1.000000 };
		double gg[MAX_INPUT_LENGTH] = { 1.000000, 0.898136, 0.788027, 0.679369, 0.570642, 0.461780, 0.370245, 0.292206, 0.213912, 0.134749, 0.055599, 0.007772, 0.000272, 0.000008, 0.000004, 0.000009, 0.000007, 0.000014, 0.000011, 0.000027, 0.000088, 0.004334, 0.037984, 0.096488, 0.155866, 0.214581, 0.274564, 0.358132, 0.474443, 0.596959, 0.719264, 0.841713, 0.964617 };
		double bb[MAX_INPUT_LENGTH] = { 0.999914, 1.000000, 1.000000, 0.999790, 0.998744, 0.998603, 0.999497, 0.999707, 0.999911, 0.999704, 0.999322, 0.999702, 1.000000, 0.999979, 0.999586, 0.998111, 0.956023, 0.819800, 0.630929, 0.440522, 0.253413, 0.086470, 0.011451, 0.000127, 0.000003, 0.000001, 0.000015, 0.000453, 0.001038, 0.000450, 0.000225, 0.000199, 0.000161 };
		for (int iii = 0; iii < length; iii++) {
			rx[iii] = rr[iii];
			gx[iii] = gg[iii];
			bx[iii] = bb[iii];
		}
	} else if (code == (CIECOMP_ALT_3 + 1)) {
		length = 27;
		double rr[MAX_INPUT_LENGTH] = { 0.000234, 0.000162, 0.003354, 0.038452, 0.102054, 0.165385, 0.228699, 0.291374, 0.369664, 0.475506, 0.586630, 0.695216, 0.803475, 0.893515, 0.930827, 0.944857, 0.961729, 0.978009, 0.993506, 0.999136, 0.998879, 0.999137, 0.999905, 0.999888, 0.999957, 0.999987, 1.000000 };
		double gg[MAX_INPUT_LENGTH] = { 0.679369, 0.570642, 0.461780, 0.370245, 0.292206, 0.213912, 0.134749, 0.055599, 0.007772, 0.000272, 0.000008, 0.000004, 0.000009, 0.000007, 0.000014, 0.000011, 0.000027, 0.000088, 0.004334, 0.037984, 0.096488, 0.155866, 0.214581, 0.274564, 0.358132, 0.474443, 0.596959 };
		double bb[MAX_INPUT_LENGTH] = { 0.999790, 0.998744, 0.998603, 0.999497, 0.999707, 0.999911, 0.999704, 0.999322, 0.999702, 1.000000, 0.999979, 0.999586, 0.998111, 0.956023, 0.819800, 0.630929, 0.440522, 0.253413, 0.086470, 0.011451, 0.000127, 0.000003, 0.000001, 0.000015, 0.000453, 0.001038, 0.000450 };
		for (int iii = 0; iii < length; iii++) {
			rx[iii] = rr[iii];
			gx[iii] = gg[iii];
			bx[iii] = bb[iii];
		}
	} else if (code == BLACKBODY) {
		length = 33;
		double rr[MAX_INPUT_LENGTH] = { 1.000000, 1.000000, 1.000000, 0.999999, 0.999996, 0.999675, 0.999319, 0.999516, 0.999489, 0.999604, 0.999995, 1.000000, 1.000000, 0.999997, 0.999865, 1.000000, 0.983105, 0.909532, 0.813694, 0.720501, 0.626450, 0.532048, 0.438575, 0.343988, 0.250932, 0.157250, 0.062752, 0.007126, 0.000412, 0.000324, 0.000000, 0.000046, 0.000258 };
		double gg[MAX_INPUT_LENGTH] = { 0.000000, 0.086618, 0.180586, 0.274778, 0.368677, 0.462417, 0.555966, 0.649320, 0.743804, 0.838791, 0.930766, 0.984766, 1.000000, 0.999999, 0.999942, 0.996948, 0.982010, 0.911528, 0.815235, 0.720867, 0.626425, 0.531084, 0.437229, 0.344078, 0.250634, 0.156937, 0.066379, 0.010972, 0.000155, 0.000066, 0.000106, 0.000057, 0.000028 };
		double bb[MAX_INPUT_LENGTH] = { 0.000000, 0.000028, 0.000047, 0.000017, 0.000006, 0.000011, 0.000034, 0.000020, 0.000028, 0.000211, 0.011424, 0.076038, 0.239387, 0.427148, 0.616124, 0.803833, 0.950133, 0.997841, 0.997676, 0.997875, 0.997774, 0.997078, 0.997007, 0.996407, 0.996357, 0.997493, 0.996829, 0.944413, 0.813917, 0.672345, 0.531919, 0.390918, 0.251743 };
		for (int iii = 0; iii < length; iii++) {
			rx[iii] = rr[iii];
			gx[iii] = gg[iii];
			bx[iii] = bb[iii];
		}
	} else if (code == (BLACKBODY + 1)) {
		length = 33;
		double rr[MAX_INPUT_LENGTH] = { 1.000000, 0.999906, 0.999708, 0.999539, 0.999552, 0.999235, 0.999463, 0.999773, 0.999862, 0.999950, 0.999895, 0.999494, 0.999077, 0.999141, 0.998829, 1.000000, 0.986819, 0.938395, 0.875514, 0.812872, 0.750901, 0.688290, 0.625432, 0.562228, 0.499214, 0.436045, 0.374025, 0.311043, 0.250516, 0.188187, 0.125189, 0.062544, 0.000095 };
		double gg[MAX_INPUT_LENGTH] = { 0.000000, 0.058804, 0.121131, 0.183065, 0.245874, 0.308954, 0.371402, 0.433194, 0.494831, 0.557439, 0.619737, 0.682929, 0.746057, 0.808921, 0.872426, 0.935061, 0.969323, 0.940452, 0.876631, 0.813228, 0.749850, 0.687075, 0.624668, 0.561989, 0.498919, 0.436440, 0.374136, 0.311701, 0.249296, 0.187608, 0.125145, 0.062669, 0.000080 };
		double bb[MAX_INPUT_LENGTH] = { 0.000012, 0.000028, 0.000042, 0.000152, 0.000239, 0.000069, 0.000387, 0.001237, 0.001583, 0.001240, 0.000291, 0.000238, 0.000232, 0.000184, 0.000516, 0.002223, 0.027364, 0.120060, 0.246693, 0.371988, 0.497032, 0.622486, 0.747998, 0.875008, 0.971356, 0.998454, 0.997949, 0.997694, 0.997953, 0.998069, 0.998122, 0.998256, 0.998504 };
		for (int iii = 0; iii < length; iii++) {
			rx[iii] = rr[iii];
			gx[iii] = gg[iii];
			bx[iii] = bb[iii];
		}
	} else if (code == RAINBOW) {
		length = 33;
		double rr[MAX_INPUT_LENGTH] = { 0.000000, 0.086173, 0.179998, 0.275159, 0.369505, 0.431644, 0.351353, 0.118157, 0.019516, 0.000785, 0.000248, 0.000176, 0.000039, 0.000017, 0.000213, 0.000960, 0.058315, 0.229594, 0.461803, 0.696663, 0.901663, 0.985886, 0.999456, 0.999834, 0.999881, 0.999986, 0.999991, 0.998931, 0.999086, 0.999625, 0.999794, 0.999880, 1.000000 };
		double gg[MAX_INPUT_LENGTH] = { 0.000129, 0.000054, 0.000053, 0.000064, 0.000062, 0.000257, 0.000501, 0.024325, 0.133515, 0.350071, 0.583836, 0.813183, 0.960441, 0.999336, 0.999975, 1.000000, 0.999995, 0.999990, 0.999973, 0.999093, 0.970101, 0.819314, 0.598721, 0.365028, 0.137422, 0.001499, 0.000000, 0.072536, 0.247121, 0.436213, 0.623769, 0.811851, 1.000000 };
		double bb[MAX_INPUT_LENGTH] = { 0.000000, 0.173973, 0.362471, 0.550963, 0.739751, 0.909659, 0.991103, 0.999353, 0.999848, 0.999933, 0.998977, 0.991456, 0.909874, 0.713164, 0.478570, 0.243157, 0.060479, 0.004108, 0.000011, 0.000015, 0.000009, 0.000124, 0.000658, 0.001394, 0.016480, 0.119105, 0.342446, 0.521194, 0.625726, 0.717459, 0.810571, 0.904202, 0.998013 };
		for (int iii = 0; iii < length; iii++) {
			rx[iii] = rr[iii];
			gx[iii] = gg[iii];
			bx[iii] = bb[iii];
		}
	} else if (code == (RAINBOW + 1)) {
		length = 25;
		double rr[MAX_INPUT_LENGTH] = { 0.369505, 0.431644, 0.351353, 0.118157, 0.019516, 0.000785, 0.000248, 0.000176, 0.000039, 0.000017, 0.000213, 0.000960, 0.058315, 0.229594, 0.461803, 0.696663, 0.901663, 0.985886, 0.999456, 0.999834, 0.999881, 0.999986, 0.999991, 0.998931, 0.999086 };
		double gg[MAX_INPUT_LENGTH] = { 0.000062, 0.000257, 0.000501, 0.024325, 0.133515, 0.350071, 0.583836, 0.813183, 0.960441, 0.999336, 0.999975, 1.000000, 0.999995, 0.999990, 0.999973, 0.999093, 0.970101, 0.819314, 0.598721, 0.365028, 0.137422, 0.001499, 0.000000, 0.072536, 0.247121 };
		double bb[MAX_INPUT_LENGTH] = { 0.739751, 0.909659, 0.991103, 0.999353, 0.999848, 0.999933, 0.998977, 0.991456, 0.909874, 0.713164, 0.478570, 0.243157, 0.060479, 0.004108, 0.000011, 0.000015, 0.000009, 0.000124, 0.000658, 0.001394, 0.016480, 0.119105, 0.342446, 0.521194, 0.625726 };
		for (int iii = 0; iii < length; iii++) {
			rx[iii] = rr[iii];
			gx[iii] = gg[iii];
			bx[iii] = bb[iii];
		}
	} else if (code == RAINBOW_ALT_1) {
		length = 11;
		double rr[MAX_INPUT_LENGTH] = { 0.00, 0.50, 0.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 1.00, 1.00 };
		double gg[MAX_INPUT_LENGTH] = { 0.00, 0.00, 0.00, 0.50, 1.00, 1.00, 1.00, 0.50, 0.00, 0.00, 1.00 };
		double bb[MAX_INPUT_LENGTH] = { 0.00, 1.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 0.00, 0.50, 1.00 };
		for (int iii = 0; iii < length; iii++) {
			rx[iii] = rr[iii];
			gx[iii] = gg[iii];
			bx[iii] = bb[iii];
		}
	} else if (code == (RAINBOW_ALT_1 + 1)) {
		length = 9;
		double rr[MAX_INPUT_LENGTH] = { 0.50, 0.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 1.00 };
		double gg[MAX_INPUT_LENGTH] = { 0.00, 0.00, 0.50, 1.00, 1.00, 1.00, 0.50, 0.00, 0.00 };
		double bb[MAX_INPUT_LENGTH] = { 1.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 0.00, 0.50 };
		for (int iii = 0; iii < length; iii++) {
			rx[iii] = rr[iii];
			gx[iii] = gg[iii];
			bx[iii] = bb[iii];
		}
	} else if (code == RAINBOW_ALT_2) {
		length = 13;
		double rr[MAX_INPUT_LENGTH] = { 0.00, 0.50, 1.00, 0.37, 0.00, 0.00, 0.00, 0.14, 0.76, 1.00, 1.00, 1.00, 1.00 };
		double gg[MAX_INPUT_LENGTH] = { 0.00, 0.00, 0.00, 0.00, 0.26, 1.00, 1.00, 1.00, 1.00, 0.61, 0.00, 0.50, 1.00 };
		double bb[MAX_INPUT_LENGTH] = { 0.00, 0.50, 1.00, 1.00, 1.00, 1.00, 0.50, 0.00, 0.00, 0.00, 0.00, 0.50, 1.00 };
		for (int iii = 0; iii < length; iii++) {
			rx[iii] = rr[iii];
			gx[iii] = gg[iii];
			bx[iii] = bb[iii];
		}
	} else if (code == (RAINBOW_ALT_2 + 1)) {
		length = 9;
		double rr[MAX_INPUT_LENGTH] = { 1.00, 0.37, 0.00, 0.00, 0.00, 0.14, 0.76, 1.00, 1.00 };
		double gg[MAX_INPUT_LENGTH] = { 0.00, 0.00, 0.26, 1.00, 1.00, 1.00, 1.00, 0.61, 0.00 };
		double bb[MAX_INPUT_LENGTH] = { 1.00, 1.00, 1.00, 1.00, 0.50, 0.00, 0.00, 0.00, 0.00 };
		for (int iii = 0; iii < length; iii++) {
			rx[iii] = rr[iii];
			gx[iii] = gg[iii];
			bx[iii] = bb[iii];
		}
	} else if (code == RAINBOW_ALT_3) {
		length = 13;
		double rr[MAX_INPUT_LENGTH] = { 0.00, 0.50, 1.00, 0.37, 0.00, 0.00, 0.00, 0.14, 0.76, 1.00, 1.00, 1.00, 1.00 };
		double gg[MAX_INPUT_LENGTH] = { 0.00, 0.00, 0.00, 0.00, 0.26, 1.00, 1.00, 1.00, 1.00, 0.61, 0.00, 0.50, 1.00 };
		double bb[MAX_INPUT_LENGTH] = { 0.00, 0.50, 1.00, 1.00, 1.00, 1.00, 0.50, 0.00, 0.00, 0.00, 0.00, 0.50, 1.00 };
		for (int iii = 0; iii < length; iii++) {
			rx[iii] = rr[iii];
			gx[iii] = gg[iii];
			bx[iii] = bb[iii];
		}
	} else if (code == (RAINBOW_ALT_3 + 1)) {
		length = 9;
		double rr[MAX_INPUT_LENGTH] = { 1.00, 0.37, 0.00, 0.00, 0.00, 0.14, 0.76, 1.00, 1.00 };
		double gg[MAX_INPUT_LENGTH] = { 0.00, 0.00, 0.26, 1.00, 1.00, 1.00, 1.00, 0.61, 0.00 };
		double bb[MAX_INPUT_LENGTH] = { 1.00, 1.00, 1.00, 1.00, 0.50, 0.00, 0.00, 0.00, 0.00 };
		for (int iii = 0; iii < length; iii++) {
			rx[iii] = rr[iii];
			gx[iii] = gg[iii];
			bx[iii] = bb[iii];
		}
	} else if (code == RAINBOW_ALT_4) {
		length = 33;
		double rr[MAX_INPUT_LENGTH] = { 0.000000, 0.086173, 0.179998, 0.275159, 0.369505, 0.431644, 0.351353, 0.118157, 0.019516, 0.000785, 0.000248, 0.000176, 0.000039, 0.000017, 0.000213, 0.000960, 0.058315, 0.229594, 0.461803, 0.696663, 0.901663, 0.985886, 0.999456, 0.999834, 0.999881, 0.999986, 0.999991, 0.998931, 0.999086, 0.999625, 0.999794, 0.999880, 1.000000 };
		double gg[MAX_INPUT_LENGTH] = { 0.000129, 0.000054, 0.000053, 0.000064, 0.000062, 0.000257, 0.000501, 0.024325, 0.133515, 0.350071, 0.583836, 0.813183, 0.960441, 0.999336, 0.999975, 1.000000, 0.999995, 0.999990, 0.999973, 0.999093, 0.970101, 0.819314, 0.598721, 0.365028, 0.137422, 0.001499, 0.000000, 0.072536, 0.247121, 0.436213, 0.623769, 0.811851, 1.000000 };
		double bb[MAX_INPUT_LENGTH] = { 0.000000, 0.173973, 0.362471, 0.550963, 0.739751, 0.909659, 0.991103, 0.999353, 0.999848, 0.999933, 0.998977, 0.991456, 0.909874, 0.713164, 0.478570, 0.243157, 0.060479, 0.004108, 0.000011, 0.000015, 0.000009, 0.000124, 0.000658, 0.001394, 0.016480, 0.119105, 0.342446, 0.521194, 0.625726, 0.717459, 0.810571, 0.904202, 0.998013 };
		for (int iii = 0; iii < length; iii++) {
			rx[iii] = rr[iii];
			gx[iii] = gg[iii];
			bx[iii] = bb[iii];
		}
	} else if (code == (RAINBOW_ALT_4 + 1)) {
		length = 22;
		double rr[MAX_INPUT_LENGTH] = { 0.275159, 0.369505, 0.431644, 0.351353, 0.118157, 0.019516, 0.000785, 0.000248, 0.000176, 0.000039, 0.000017, 0.461803, 0.696663, 0.901663, 0.999456, 0.999834, 0.999881, 0.999986, 0.999991, 0.998931, 0.999086, 0.999625 };
		double gg[MAX_INPUT_LENGTH] = { 0.000064, 0.000062, 0.000257, 0.000501, 0.024325, 0.133515, 0.350071, 0.583836, 0.813183, 0.960441, 0.999336, 0.999973, 0.999093, 0.970101, 0.598721, 0.365028, 0.137422, 0.001499, 0.000000, 0.072536, 0.247121, 0.436213 };
		double bb[MAX_INPUT_LENGTH] = { 0.550963, 0.739751, 0.909659, 0.991103, 0.999353, 0.999848, 0.999933, 0.998977, 0.991456, 0.909874, 0.713164, 0.000011, 0.000015, 0.000009, 0.000658, 0.001394, 0.016480, 0.119105, 0.342446, 0.521194, 0.625726, 0.717459 };
		for (int iii = 0; iii < length; iii++) {
			rx[iii] = rr[iii];
			gx[iii] = gg[iii];
			bx[iii] = bb[iii];
		}
	} else if (code == IRON) {
		length = 33;
		double rr[MAX_INPUT_LENGTH] = { 0.000137, 0.044733, 0.091636, 0.138374, 0.185449, 0.235309, 0.310152, 0.409810, 0.509259, 0.608080, 0.707532, 0.784229, 0.832226, 0.874605, 0.914548, 0.957131, 0.990374, 0.999789, 0.999559, 0.999753, 0.999893, 0.999713, 0.999243, 0.999138, 0.998799, 0.998982, 0.999794, 0.999992, 0.999997, 0.999947, 0.998754, 0.998419, 0.998206 };
		double gg[MAX_INPUT_LENGTH] = { 0.000218, 0.000655, 0.001318, 0.002240, 0.002696, 0.004508, 0.015631, 0.033312, 0.051963, 0.070414, 0.088750, 0.129074, 0.197559, 0.271724, 0.346554, 0.421253, 0.486973, 0.528391, 0.559335, 0.591508, 0.623114, 0.657761, 0.707739, 0.770047, 0.832696, 0.895595, 0.958334, 0.994681, 0.999982, 1.000000, 1.000000, 1.000000, 1.000000 };
		double bb[MAX_INPUT_LENGTH] = { 0.000000, 0.087421, 0.182004, 0.276794, 0.372322, 0.463058, 0.506880, 0.513951, 0.520935, 0.528776, 0.535606, 0.503864, 0.411574, 0.308960, 0.206618, 0.103307, 0.024739, 0.001474, 0.003034, 0.003601, 0.005707, 0.007447, 0.006697, 0.005752, 0.004690, 0.002699, 0.007463, 0.077716, 0.244948, 0.434054, 0.622376, 0.810788, 0.998917 };
		for (int iii = 0; iii < length; iii++) {
			rx[iii] = rr[iii];
			gx[iii] = gg[iii];
			bx[iii] = bb[iii];
		}
	} else if (code == (IRON + 1)) {
		length = 25;
		double rr[MAX_INPUT_LENGTH] = { 0.185449, 0.235309, 0.310152, 0.409810, 0.509259, 0.608080, 0.707532, 0.784229, 0.832226, 0.874605, 0.914548, 0.957131, 0.990374, 0.999789, 0.999559, 0.999753, 0.999893, 0.999713, 0.999243, 0.999138, 0.998799, 0.998982, 0.999794, 0.999992, 0.999997 };
		double gg[MAX_INPUT_LENGTH] = { 0.002696, 0.004508, 0.015631, 0.033312, 0.051963, 0.070414, 0.088750, 0.129074, 0.197559, 0.271724, 0.346554, 0.421253, 0.486973, 0.528391, 0.559335, 0.591508, 0.623114, 0.657761, 0.707739, 0.770047, 0.832696, 0.895595, 0.958334, 0.994681, 0.999982 };
		double bb[MAX_INPUT_LENGTH] = { 0.372322, 0.463058, 0.506880, 0.513951, 0.520935, 0.528776, 0.535606, 0.503864, 0.411574, 0.308960, 0.206618, 0.103307, 0.024739, 0.001474, 0.003034, 0.003601, 0.005707, 0.007447, 0.006697, 0.005752, 0.004690, 0.002699, 0.007463, 0.077716, 0.244948 };
		for (int iii = 0; iii < length; iii++) {
			rx[iii] = rr[iii];
			gx[iii] = gg[iii];
			bx[iii] = bb[iii];
		}
	} else if (code == IRON_ALT_1) {
		length = 12;
		double rr[MAX_INPUT_LENGTH] = { 0.00, 0.05, 0.10, 0.40, 0.70, 0.80, 0.90, 0.95, 1.00, 1.00, 1.00, 1.00 };
		double gg[MAX_INPUT_LENGTH] = { 0.00, 0.00, 0.00, 0.00, 0.00, 0.10, 0.25, 0.45, 0.60, 0.80, 0.95, 1.00 };
		double bb[MAX_INPUT_LENGTH] = { 0.00, 0.25, 0.50, 0.60, 0.60, 0.50, 0.05, 0.00, 0.00, 0.05, 0.45, 1.00 };
		for (int iii = 0; iii < length; iii++) {
			rx[iii] = rr[iii];
			gx[iii] = gg[iii];
			bx[iii] = bb[iii];
		}
	} else if (code == (IRON_ALT_1 + 1)) {
		length = 10;
		double rr[MAX_INPUT_LENGTH] = { 0.05, 0.10, 0.40, 0.70, 0.80, 0.90, 0.95, 1.00, 1.00, 1.00 };
		double gg[MAX_INPUT_LENGTH] = { 0.00, 0.00, 0.00, 0.00, 0.10, 0.25, 0.45, 0.60, 0.80, 0.95 };
		double bb[MAX_INPUT_LENGTH] = { 0.25, 0.50, 0.60, 0.60, 0.50, 0.05, 0.00, 0.00, 0.05, 0.45 };
		for (int iii = 0; iii < length; iii++) {
			rx[iii] = rr[iii];
			gx[iii] = gg[iii];
			bx[iii] = bb[iii];
		}
	} else if (code == IRON_ALT_2) {
		length = 11;
		double rr[MAX_INPUT_LENGTH] = { 0.00, 0.05, 0.10,  0.40, 0.80, 0.90, 0.95, 1.00,   1.00, 1.00, 1.00 };
		double gg[MAX_INPUT_LENGTH] = { 0.00, 0.00, 0.00,  0.00, 0.10, 0.25, 0.45, 0.80,   0.95, 1.00, 1.00 };
		double bb[MAX_INPUT_LENGTH] = { 0.00, 0.25, 0.50,  0.60, 0.50, 0.05, 0.00, 0.05,   0.45, 0.70, 1.00 };
		for (int iii = 0; iii < length; iii++) {
			rx[iii] = rr[iii];
			gx[iii] = gg[iii];
			bx[iii] = bb[iii];
		}
	} else if (code == (IRON_ALT_2 + 1)) {
		length = 5;
		double rr[MAX_INPUT_LENGTH] = { 0.40, 0.80, 0.90, 0.95, 1.00 };
		double gg[MAX_INPUT_LENGTH] = { 0.60, 0.50, 0.05, 0.00, 0.05 };
		double bb[MAX_INPUT_LENGTH] = { 0.00, 0.10, 0.25, 0.45, 0.80 };
		for (int iii = 0; iii < length; iii++) {
			rx[iii] = rr[iii];
			gx[iii] = gg[iii];
			bx[iii] = bb[iii];
		}
	} else if (code == (IRON_ALT_3)) {
		length = 11;
		double rr[MAX_INPUT_LENGTH] = { 0.00, 0.10, 0.40, 0.70, 0.80, 0.90, 0.95, 1.00, 1.00, 1.00, 1.00 };
		double gg[MAX_INPUT_LENGTH] = { 0.00, 0.00, 0.00, 0.00, 0.10, 0.25, 0.45, 0.60, 0.80, 0.95, 1.00 };
		double bb[MAX_INPUT_LENGTH] = { 0.00, 0.50, 0.60, 0.60, 0.50, 0.05, 0.00, 0.00, 0.05, 0.45, 1.00 };
		for (int iii = 0; iii < length; iii++) {
			rx[iii] = rr[iii];
			gx[iii] = gg[iii];
			bx[iii] = bb[iii];
		}
	} else if (code == (IRON_ALT_3 + 1)) {
		length = 9;
		double rr[MAX_INPUT_LENGTH] = { 0.10, 0.40, 0.70, 0.80, 0.90, 0.95, 1.00, 1.00, 1.00 };
		double gg[MAX_INPUT_LENGTH] = { 0.00, 0.00, 0.00, 0.10, 0.25, 0.45, 0.60, 0.80, 0.95 };
		double bb[MAX_INPUT_LENGTH] = { 0.50, 0.60, 0.60, 0.50, 0.05, 0.00, 0.00, 0.05, 0.45 };
		for (int iii = 0; iii < length; iii++) {
			rx[iii] = rr[iii];
			gx[iii] = gg[iii];
			bx[iii] = bb[iii];
		}
	} else if (code == BLUERED) {
		length = 3;
		double rr[MAX_INPUT_LENGTH] = { 0.00, 1.00, 1.00 };
		double gg[MAX_INPUT_LENGTH] = { 0.00, 1.00, 0.00 };
		double bb[MAX_INPUT_LENGTH] = { 1.00, 1.00, 0.00 };
		for (int iii = 0; iii < length; iii++) {
			rx[iii] = rr[iii];
			gx[iii] = gg[iii];
			bx[iii] = bb[iii];
		}
	} else if (code == (BLUERED + 1)) {
		length = 2;
		double rr[MAX_INPUT_LENGTH] = { 0.00, 1.00 };
		double gg[MAX_INPUT_LENGTH] = { 0.00, 0.00 };
		double bb[MAX_INPUT_LENGTH] = { 1.00, 0.00 };
		for (int iii = 0; iii < length; iii++) {
			rx[iii] = rr[iii];
			gx[iii] = gg[iii];
			bx[iii] = bb[iii];
		}
	} else if (code == BLUERED_ALT_1) {
		length = 4;
		double rr[MAX_INPUT_LENGTH] = { 0.00, 0.00, 1.00, 1.00 };
		double gg[MAX_INPUT_LENGTH] = { 0.00, 0.00, 0.00, 1.00 };
		double bb[MAX_INPUT_LENGTH] = { 0.00, 1.00, 0.00, 1.00 };
		for (int iii = 0; iii < length; iii++) {
			rx[iii] = rr[iii];
			gx[iii] = gg[iii];
			bx[iii] = bb[iii];
		}
	} else if (code == (BLUERED_ALT_1 + 1)) {
		length = 2;
		double rr[MAX_INPUT_LENGTH] = { 0.00, 1.00 };
		double gg[MAX_INPUT_LENGTH] = { 0.00, 0.00 };
		double bb[MAX_INPUT_LENGTH] = { 1.00, 0.00 };
		for (int iii = 0; iii < length; iii++) {
			rx[iii] = rr[iii];
			gx[iii] = gg[iii];
			bx[iii] = bb[iii];
		}
	} else if (code == BLUERED_ALT_2) {
		length = 7;
		double rr[MAX_INPUT_LENGTH] = { 0.00, 0.50, 0.80, 1.00, 1.00, 1.00, 1.00 };
		double gg[MAX_INPUT_LENGTH] = { 0.00, 0.50, 0.80, 1.00, 0.80, 0.50, 0.00 };
		double bb[MAX_INPUT_LENGTH] = { 1.00, 1.00, 1.00, 1.00, 0.80, 0.50, 0.00 };
		for (int iii = 0; iii < length; iii++) {
			rx[iii] = rr[iii];
			gx[iii] = gg[iii];
			bx[iii] = bb[iii];
		}
	} else if (code == (BLUERED_ALT_2 + 1)) {
		length = 4;
		double rr[MAX_INPUT_LENGTH] = { 0.00, 0.50, 1.00, 1.00 };
		double gg[MAX_INPUT_LENGTH] = { 0.00, 0.50, 0.50, 0.00 };
		double bb[MAX_INPUT_LENGTH] = { 1.00, 1.00, 0.50, 0.00 };
		for (int iii = 0; iii < length; iii++) {
			rx[iii] = rr[iii];
			gx[iii] = gg[iii];
			bx[iii] = bb[iii];
		}
	} else if (code == JET) {
		length = 13;
		double rr[MAX_INPUT_LENGTH] = { 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.70, 1.00, 1.00, 1.00, 1.00, 1.00 };
		double gg[MAX_INPUT_LENGTH] = { 0.00, 0.00, 0.00, 0.20, 0.80, 1.00, 1.00, 1.00, 0.80, 0.20, 0.00, 0.50, 1.00 };
		double bb[MAX_INPUT_LENGTH] = { 0.00, 0.50, 1.00, 1.00, 1.00, 0.70, 0.00, 0.00, 0.00, 0.00, 0.00, 0.50, 1.00 };
		for (int iii = 0; iii < length; iii++) {
			rx[iii] = rr[iii];
			gx[iii] = gg[iii];
			bx[iii] = bb[iii];
		}
	} else if (code == (JET + 1)) {
		length = 11;
		double rr[MAX_INPUT_LENGTH] = { 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.70, 1.00, 1.00, 1.00, 1.00 };
		double gg[MAX_INPUT_LENGTH] = { 0.00, 0.00, 0.20, 0.80, 1.00, 1.00, 1.00, 0.80, 0.20, 0.00, 0.50 };
		double bb[MAX_INPUT_LENGTH] = { 0.50, 1.00, 1.00, 1.00, 0.70, 0.00, 0.00, 0.00, 0.00, 0.00, 0.50 };
		for (int iii = 0; iii < length; iii++) {
			rx[iii] = rr[iii];
			gx[iii] = gg[iii];
			bx[iii] = bb[iii];
		}
	} else if (code == JET_ALT_1) {
		length = 11;
		double rr[MAX_INPUT_LENGTH] = { 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.70, 1.00, 1.00, 1.00, 1.00 };
		double gg[MAX_INPUT_LENGTH] = { 0.00, 0.00, 0.20, 0.80, 1.00, 1.00, 1.00, 0.80, 0.20, 0.00, 1.00 };
		double bb[MAX_INPUT_LENGTH] = { 0.00, 1.00, 1.00, 1.00, 0.70, 0.00, 0.00, 0.00, 0.00, 0.00, 1.00 };
		for (int iii = 0; iii < length; iii++) {
			rx[iii] = rr[iii];
			gx[iii] = gg[iii];
			bx[iii] = bb[iii];
		}
	} else if (code == (JET_ALT_1 + 1)) {
		length = 9;
		double rr[MAX_INPUT_LENGTH] = { 0.00, 0.00, 0.00, 0.00, 0.00, 0.70, 1.00, 1.00, 1.00 };
		double gg[MAX_INPUT_LENGTH] = { 0.00, 0.20, 0.80, 1.00, 1.00, 1.00, 0.80, 0.20, 0.00 };
		double bb[MAX_INPUT_LENGTH] = { 1.00, 1.00, 1.00, 0.70, 0.00, 0.00, 0.00, 0.00, 0.00 };
		for (int iii = 0; iii < length; iii++) {
			rx[iii] = rr[iii];
			gx[iii] = gg[iii];
			bx[iii] = bb[iii];
		}
	} else if (code == ICE) {
		length = 10;
		double rr[MAX_INPUT_LENGTH] = { 1.00, 0.25, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00 };
		double gg[MAX_INPUT_LENGTH] = { 1.00, 1.00, 0.88, 0.75, 0.68, 0.50, 0.38, 0.25, 0.13, 0.00 };
		double bb[MAX_INPUT_LENGTH] = { 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 0.75, 0.50, 0.25, 0.00 };
		for (int iii = 0; iii < length; iii++) {
			rx[iii] = rr[iii];
			gx[iii] = gg[iii];
			bx[iii] = bb[iii];
		}
	} else if (code == (ICE + 1)) {
		length = 6;
		double rr[MAX_INPUT_LENGTH] = { 0.00, 0.00, 0.00, 0.00, 0.00, 0.00 };
		double gg[MAX_INPUT_LENGTH] = { 0.88, 0.75, 0.68, 0.50, 0.38, 0.25 };
		double bb[MAX_INPUT_LENGTH] = { 1.00, 1.00, 1.00, 1.00, 0.75, 0.50 };
		for (int iii = 0; iii < length; iii++) {
			rx[iii] = rr[iii];
			gx[iii] = gg[iii];
			bx[iii] = bb[iii];
		}
	} else if (code == ICE_ALT_1) {
		length = 11;
		double rr[MAX_INPUT_LENGTH] = { 1.00, 0.67, 0.33,     0.00, 0.00, 0.00, 0.00, 0.00,    0.00, 0.00, 0.00 };
		double gg[MAX_INPUT_LENGTH] = { 1.00, 1.00, 1.00,     1.00, 0.75, 0.50, 0.25, 0.00,    0.00, 0.00, 0.00 };
		double bb[MAX_INPUT_LENGTH] = { 1.00, 1.00, 1.00,     1.00, 1.00, 1.00, 1.00, 1.00,    0.67, 0.33, 0.00 };
		for (int iii = 0; iii < length; iii++) {
			rx[iii] = rr[iii];
			gx[iii] = gg[iii];
			bx[iii] = bb[iii];
		}
	} else if (code == (ICE_ALT_1 + 1)) {
		length = 7;
		double rr[MAX_INPUT_LENGTH] = { 0.33,     0.00, 0.00, 0.00, 0.00, 0.00,    0.00 };
		double gg[MAX_INPUT_LENGTH] = { 1.00,     1.00, 0.75, 0.50, 0.25, 0.00,    0.00 };
		double bb[MAX_INPUT_LENGTH] = { 1.00,     1.00, 1.00, 1.00, 1.00, 1.00,    0.67 };
		for (int iii = 0; iii < length; iii++) {
			rx[iii] = rr[iii];
			gx[iii] = gg[iii];
			bx[iii] = bb[iii];
		}
	} else if (code == ICE_ALT_2) {
		length = 11;
		double rr[MAX_INPUT_LENGTH] = { 1.00, 0.64, 0.36, 0.16, 0.04, 0.01, 0.00, 0.00, 0.00, 0.00, 0.00 };
		double gg[MAX_INPUT_LENGTH] = { 1.00, 0.90, 0.80, 0.70, 0.60, 0.50, 0.40, 0.30, 0.20, 0.10, 0.00 };
		double bb[MAX_INPUT_LENGTH] = { 1.00, 1.00, 1.00, 1.00, 1.00, 0.99, 0.89, 0.78, 0.63, 0.48, 0.00 };
		for (int iii = 0; iii < length; iii++) {
			rx[iii] = rr[iii];
			gx[iii] = gg[iii];
			bx[iii] = bb[iii];
		}
	} else if (code == (ICE_ALT_2 + 1)) {
		length = 7;
		double rr[MAX_INPUT_LENGTH] = { 0.36, 0.16, 0.04, 0.01, 0.00, 0.00, 0.00 };
		double gg[MAX_INPUT_LENGTH] = { 0.80, 0.70, 0.60, 0.50, 0.40, 0.30, 0.20 };
		double bb[MAX_INPUT_LENGTH] = { 1.00, 1.00, 1.00, 0.99, 0.89, 0.78, 0.63 };
		for (int iii = 0; iii < length; iii++) {
			rx[iii] = rr[iii];
			gx[iii] = gg[iii];
			bx[iii] = bb[iii];
		}
	} else if (code == ICE_ALT_3) {
		length = 9;
		double rr[MAX_INPUT_LENGTH] = { 1.00, 0.50, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00 };
		double gg[MAX_INPUT_LENGTH] = { 1.00, 1.00, 1.00, 0.50, 0.00, 0.50, 1.00, 0.50, 0.00 };
		double bb[MAX_INPUT_LENGTH] = { 1.00, 1.00, 1.00, 1.00, 1.00, 0.50, 0.00, 0.00, 0.00 };
		for (int iii = 0; iii < length; iii++) {
			rx[iii] = rr[iii];
			gx[iii] = gg[iii];
			bx[iii] = bb[iii];
		}
	} else if (code == (ICE_ALT_3 + 1)) {
		length = 7;
		double rr[MAX_INPUT_LENGTH] = { 0.50, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00 };
		double gg[MAX_INPUT_LENGTH] = { 1.00, 1.00, 0.50, 0.00, 0.50, 1.00, 0.50 };
		double bb[MAX_INPUT_LENGTH] = { 1.00, 1.00, 1.00, 1.00, 0.50, 0.00, 0.00 };
		for (int iii = 0; iii < length; iii++) {
			rx[iii] = rr[iii];
			gx[iii] = gg[iii];
			bx[iii] = bb[iii];
		}
	} else if (code == ICEIRON) {
		length = 19;
		double rr[MAX_INPUT_LENGTH] = { 1.00, 0.67, 0.33,     0.00, 0.00, 0.00, 0.00,    0.00, 0.00,     0.00,       0.33, 0.67,     0.67, 0.86, 0.93, 1.00,   1.00, 1.00, 1.00 };
		double gg[MAX_INPUT_LENGTH] = { 1.00, 1.00, 1.00,     1.00, 0.67, 0.33, 0.00,    0.00, 0.00,     0.00,       0.00, 0.00,     0.00, 0.15, 0.40, 0.80,   0.95, 1.00, 1.00 };
		double bb[MAX_INPUT_LENGTH] = { 1.00, 1.00, 1.00,     1.00, 1.00, 1.00, 1.00,    0.67, 0.33,     0.00,       0.00, 0.00,     0.33, 0.40, 0.10, 0.05,   0.45, 0.70, 1.00 };
		for (int iii = 0; iii < length; iii++) {
			rx[iii] = rr[iii];
			gx[iii] = gg[iii];
			bx[iii] = bb[iii];
		}
	} else if (code == (ICEIRON + 1)) {
		length = 12;
		double rr[MAX_INPUT_LENGTH] = { 0.33,     0.00, 0.00, 0.00, 0.00,    0.00, 0.67,     0.67, 0.86, 0.93, 1.00,   1.00 };
		double gg[MAX_INPUT_LENGTH] = { 1.00,     1.00, 0.67, 0.33, 0.00,    0.00, 0.00,     0.00, 0.15, 0.40, 0.80,   0.95 };
		double bb[MAX_INPUT_LENGTH] = { 1.00,     1.00, 1.00, 1.00, 1.00,    0.67, 0.00,     0.33, 0.40, 0.10, 0.05,   0.45 };
		for (int iii = 0; iii < length; iii++) {
			rx[iii] = rr[iii];
			gx[iii] = gg[iii];
			bx[iii] = bb[iii];
		}
	} else if (code == ICEIRON_ALT_1) {
		length = 21;
		double rr[MAX_INPUT_LENGTH] = { 1.00, 0.67, 0.33,     0.00, 0.00, 0.00, 0.00, 0.00,    0.00, 0.00,     0.00,       0.05, 0.10,     0.40, 0.80, 0.90, 0.95, 1.00,   1.00, 1.00, 1.00 };
		double gg[MAX_INPUT_LENGTH] = { 1.00, 1.00, 1.00,     1.00, 0.75, 0.50, 0.25, 0.00,    0.00, 0.00,     0.00,       0.00, 0.00,     0.00, 0.10, 0.25, 0.45, 0.80,   0.95, 1.00, 1.00 };
		double bb[MAX_INPUT_LENGTH] = { 1.00, 1.00, 1.00,     1.00, 1.00, 1.00, 1.00, 1.00,    0.67, 0.33,     0.00,       0.25, 0.50,     0.60, 0.50, 0.05, 0.00, 0.05,   0.45, 0.70, 1.00 };
		for (int iii = 0; iii < length; iii++) {
			rx[iii] = rr[iii];
			gx[iii] = gg[iii];
			bx[iii] = bb[iii];
		}
	} else if (code == (ICEIRON_ALT_1 + 1)) {
		length = 10;
		double rr[MAX_INPUT_LENGTH] = { 0.00, 0.00, 0.00, 0.00, 0.00,          0.40, 0.80, 0.90, 0.95, 1.00 };
		double gg[MAX_INPUT_LENGTH] = { 1.00, 0.75, 0.50, 0.25, 0.00,          0.00, 0.10, 0.25, 0.45, 0.80 };
		double bb[MAX_INPUT_LENGTH] = { 1.00, 1.00, 1.00, 1.00, 1.00,          0.60, 0.50, 0.05, 0.00, 0.05 };
		for (int iii = 0; iii < length; iii++) {
			rx[iii] = rr[iii];
			gx[iii] = gg[iii];
			bx[iii] = bb[iii];
		}
	} else if (code == ICEIRON_ALT_2) {
		length = 19;
		double rr[MAX_INPUT_LENGTH] = { 1.00,      0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00,      0.00,        0.10, 0.40, 0.70, 0.80, 0.90, 0.95, 1.00, 1.00, 1.00,   1.00 };
		double gg[MAX_INPUT_LENGTH] = { 1.00,      0.88, 0.75, 0.68, 0.50, 0.38, 0.25, 0.13,      0.00,        0.00, 0.00, 0.00, 0.10, 0.25, 0.45, 0.60, 0.80, 0.95,   1.00 };
		double bb[MAX_INPUT_LENGTH] = { 1.00,      1.00, 1.00, 1.00, 1.00, 0.75, 0.50, 0.25,      0.00,        0.50, 0.60, 0.60, 0.50, 0.05, 0.00, 0.00, 0.05, 0.45,   1.00 };
		for (int iii = 0; iii < length; iii++) {
			rx[iii] = rr[iii];
			gx[iii] = gg[iii];
			bx[iii] = bb[iii];
		}
	} else if (code == (ICEIRON_ALT_2 + 1)) {
		length = 17;
		double rr[MAX_INPUT_LENGTH] = { 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00,      0.00,        0.10, 0.40, 0.70, 0.80, 0.90, 0.95, 1.00, 1.00, 1.00 };
		double gg[MAX_INPUT_LENGTH] = { 0.88, 0.75, 0.68, 0.50, 0.38, 0.25, 0.13,      0.00,        0.00, 0.00, 0.00, 0.10, 0.25, 0.45, 0.60, 0.80, 0.95 };
		double bb[MAX_INPUT_LENGTH] = { 1.00, 1.00, 1.00, 1.00, 0.75, 0.50, 0.25,      0.00,        0.50, 0.60, 0.60, 0.50, 0.05, 0.00, 0.00, 0.05, 0.45 };
		for (int iii = 0; iii < length; iii++) {
			rx[iii] = rr[iii];
			gx[iii] = gg[iii];
			bx[iii] = bb[iii];
		}
	} else if (code == ICEFIRE) {
		length = 19;
		double rr[MAX_INPUT_LENGTH] = { 1.00, 0.67, 0.33,     0.00, 0.00, 0.00, 0.00,    0.00, 0.00,     0.00,       0.33, 0.67,     1.00, 1.00, 1.00, 1.00,   1.00, 1.00, 1.00 };
		double gg[MAX_INPUT_LENGTH] = { 1.00, 1.00, 1.00,     1.00, 0.67, 0.33, 0.00,    0.00, 0.00,     0.00,       0.00, 0.00,     0.00, 0.33, 0.67, 1.00,   1.00, 1.00, 1.00 };
		double bb[MAX_INPUT_LENGTH] = { 1.00, 1.00, 1.00,     1.00, 1.00, 1.00, 1.00,    0.67, 0.33,     0.00,       0.00, 0.00,     0.00, 0.00, 0.00, 0.00,   0.33, 0.67, 1.00 };
		for (int iii = 0; iii < length; iii++) {
			rx[iii] = rr[iii];
			gx[iii] = gg[iii];
			bx[iii] = bb[iii];
		}
	} else if (code == (ICEFIRE + 1)) {
		length = 12;
		double rr[MAX_INPUT_LENGTH] = { 0.33,     0.00, 0.00, 0.00, 0.00,    0.00, 0.67,     1.00, 1.00, 1.00, 1.00,   1.00 };
		double gg[MAX_INPUT_LENGTH] = { 1.00,     1.00, 0.67, 0.33, 0.00,    0.00, 0.00,     0.00, 0.33, 0.67, 1.00,   1.00 };
		double bb[MAX_INPUT_LENGTH] = { 1.00,     1.00, 1.00, 1.00, 1.00,    0.67, 0.00,     0.00, 0.00, 0.00, 0.00,   0.33 };
		for (int iii = 0; iii < length; iii++) {
			rx[iii] = rr[iii];
			gx[iii] = gg[iii];
			bx[iii] = bb[iii];
		}
	} else if (code == ICEFIRE_ALT_1) {
		length = 17;
		double rr[MAX_INPUT_LENGTH] = { 1.00, 0.67, 0.33,     0.00, 0.00, 0.00,    0.00, 0.00,     0.00,       0.33, 0.67,     1.00, 1.00, 1.00,   1.00, 1.00, 1.00 };
		double gg[MAX_INPUT_LENGTH] = { 1.00, 1.00, 1.00,     1.00, 0.50, 0.00,    0.00, 0.00,     0.00,       0.00, 0.00,     0.00, 0.50, 1.00,   1.00, 1.00, 1.00 };
		double bb[MAX_INPUT_LENGTH] = { 1.00, 1.00, 1.00,     1.00, 1.00, 1.00,    0.67, 0.33,     0.00,       0.00, 0.00,     0.00, 0.00, 0.00,   0.33, 0.67, 1.00 };
		for (int iii = 0; iii < length; iii++) {
			rx[iii] = rr[iii];
			gx[iii] = gg[iii];
			bx[iii] = bb[iii];
		}
	} else if (code == (ICEFIRE_ALT_1 + 1)) {
		length = 10;
		double rr[MAX_INPUT_LENGTH] = { 0.33,     0.00, 0.00, 0.00,    0.00, 0.67,     1.00, 1.00, 1.00,   1.00 };
		double gg[MAX_INPUT_LENGTH] = { 1.00,     1.00, 0.50, 0.00,    0.00, 0.00,     0.00, 0.50, 1.00,   1.00 };
		double bb[MAX_INPUT_LENGTH] = { 1.00,     1.00, 1.00, 1.00,    0.67, 0.00,     0.00, 0.00, 0.00,   0.33 };
		for (int iii = 0; iii < length; iii++) {
			rx[iii] = rr[iii];
			gx[iii] = gg[iii];
			bx[iii] = bb[iii];
		}
	} else if (code == ICEFIRE_ALT_2) {
		length = 17;
		double rr[MAX_INPUT_LENGTH] = { 1.00, 0.50, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00,    0.00,   0.25, 0.50, 0.75, 1.00, 1.00, 1.00, 1.00, 1.00 };
		double gg[MAX_INPUT_LENGTH] = { 1.00, 1.00, 1.00, 0.50, 0.00, 0.50, 1.00, 0.50,    0.00,   0.00, 0.00, 0.00, 0.00, 0.50, 1.00, 1.00, 1.00 };
		double bb[MAX_INPUT_LENGTH] = { 1.00, 1.00, 1.00, 1.00, 1.00, 0.50, 0.00, 0.00,    0.00,   0.00, 0.00, 0.50, 0.00, 0.00, 0.00, 0.50, 1.00 };
		for (int iii = 0; iii < length; iii++) {
			rx[iii] = rr[iii];
			gx[iii] = gg[iii];
			bx[iii] = bb[iii];
		}
	} else if (code == (ICEFIRE_ALT_2 + 1)) {
		length = 12;
		double rr[MAX_INPUT_LENGTH] = { 0.50, 0.00, 0.00, 0.00, 0.00, 0.00, 0.50, 0.75, 1.00, 1.00, 1.00, 1.00 };
		double gg[MAX_INPUT_LENGTH] = { 1.00, 1.00, 0.50, 0.00, 0.50, 1.00, 0.00, 0.00, 0.00, 0.50, 1.00, 1.00 };
		double bb[MAX_INPUT_LENGTH] = { 1.00, 1.00, 1.00, 1.00, 0.50, 0.00, 0.00, 0.50, 0.00, 0.00, 0.00, 0.50 };
		for (int iii = 0; iii < length; iii++) {
			rx[iii] = rr[iii];
			gx[iii] = gg[iii];
			bx[iii] = bb[iii];
		}
	} else if (code == ICEFIRE_ALT_3) {
		length = 17;
		//                  WHITE           CYAN            LBLUE           DBLUE           PURPLE          RED             ORANGE          YELLOW          WHITE
		double rr[MAX_INPUT_LENGTH] = {    1.00, 0.50,     0.00, 0.00,     0.50, 0.25,     0.00, 0.25,     0.50, 0.75,     1.00, 1.00,     1.00, 1.00,     1.00, 1.00,     1.00 };
		double gg[MAX_INPUT_LENGTH] = {    1.00, 1.00,     1.00, 0.50,     0.50, 0.25,     0.00, 0.00,     0.00, 0.00,     0.00, 0.25,     0.50, 0.75,     1.00, 1.00,     1.00 };
		double bb[MAX_INPUT_LENGTH] = {    1.00, 1.00,     1.00, 1.00,     1.00, 0.75,     0.50, 0.50,     0.50, 0.25,     0.00, 0.00,     0.00, 0.00,     0.00, 0.50,     1.00 };
		for (int iii = 0; iii < length; iii++) {
			rx[iii] = rr[iii];
			gx[iii] = gg[iii];
			bx[iii] = bb[iii];
		}
	} else if (code == (ICEFIRE_ALT_3 + 1)) {
		length = 13;
		//                  CYAN            LBLUE           DBLUE           PURPLE          RED             ORANGE          YELLOW
		double rr[MAX_INPUT_LENGTH] = {    0.00, 0.00,     0.50, 0.25,     0.00, 0.25,     0.50, 0.75,     1.00, 1.00,     1.00, 1.00,     1.00 };
		double gg[MAX_INPUT_LENGTH] = {    1.00, 0.50,     0.50, 0.25,     0.00, 0.00,     0.00, 0.00,     0.00, 0.25,     0.50, 0.75,     1.00 };
		double bb[MAX_INPUT_LENGTH] = {    1.00, 1.00,     1.00, 0.75,     0.50, 0.50,     0.50, 0.25,     0.00, 0.00,     0.00, 0.00,     0.00 };
		for (int iii = 0; iii < length; iii++) {
			rx[iii] = rr[iii];
			gx[iii] = gg[iii];
			bx[iii] = bb[iii];
		}
	} else if (code == REPEATED) {
		length = 16;
		double rr[MAX_INPUT_LENGTH] = {    0.00, 1.00, 1.00, 1.00, 0.00, 1.00, 1.00, 1.00, 0.00, 1.00, 1.00, 1.00, 0.00, 1.00, 1.00, 1.00 };
		double gg[MAX_INPUT_LENGTH] = {    0.00, 1.00, 0.00, 1.00, 0.00, 1.00, 0.00, 1.00, 0.00, 1.00, 0.00, 1.00, 0.00, 1.00, 0.00, 1.00, };
		double bb[MAX_INPUT_LENGTH] = {    1.00, 1.00, 0.00, 1.00, 1.00, 1.00, 0.00, 1.00, 1.00, 1.00, 0.00, 1.00, 1.00, 1.00, 0.00, 1.00, };
		for (int iii = 0; iii < length; iii++) {
			rx[iii] = rr[iii];
			gx[iii] = gg[iii];
			bx[iii] = bb[iii];
		}
	} else if (code == (REPEATED + 1)) {
		length = 12;
		double rr[MAX_INPUT_LENGTH] = {    0.00, 1.00, 0.00, 1.00, 0.00, 1.00, 0.00, 1.00, 0.00, 1.00, 0.00, 1.00 };
		double gg[MAX_INPUT_LENGTH] = {    0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00 };
		double bb[MAX_INPUT_LENGTH] = {    1.00, 0.00, 1.00, 0.00, 1.00, 0.00, 1.00, 0.00, 1.00, 0.00, 1.00, 0.00 };
		for (int iii = 0; iii < length; iii++) {
			rx[iii] = rr[iii];
			gx[iii] = gg[iii];
			bx[iii] = bb[iii];
		}
	} else if (code == REPEATED_ALT_1) {
		length = 6;
		double rr[MAX_INPUT_LENGTH] = { 0.00, 0.00, 0.00, 1.00, 1.00, 1.00 };
		double gg[MAX_INPUT_LENGTH] = { 0.00, 1.00, 1.00, 1.00, 0.00, 0.00 };
		double bb[MAX_INPUT_LENGTH] = { 1.00, 1.00, 0.00, 0.00, 0.00, 1.00 };
		for (int iii = 0; iii < length; iii++) {
			rx[iii] = rr[iii];
			gx[iii] = gg[iii];
			bx[iii] = bb[iii];
		}
	} else if (code == (REPEATED_ALT_1 + 1)) {
		length = 6;
		double rr[MAX_INPUT_LENGTH] = { 0.00, 0.00, 0.00, 1.00, 1.00, 1.00 };
		double gg[MAX_INPUT_LENGTH] = { 0.00, 1.00, 1.00, 1.00, 0.00, 0.00 };
		double bb[MAX_INPUT_LENGTH] = { 1.00, 1.00, 0.00, 0.00, 0.00, 1.00 };
		for (int iii = 0; iii < length; iii++) {
			rx[iii] = rr[iii];
			gx[iii] = gg[iii];
			bx[iii] = bb[iii];
		}
	} else if (code == REPEATED_ALT_2) {
		length = 48;
		double rr[MAX_INPUT_LENGTH] = { 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00 };
		double gg[MAX_INPUT_LENGTH] = { 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00 };
		double bb[MAX_INPUT_LENGTH] = { 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00 };
		for (int iii = 0; iii < length; iii++) {
			rx[iii] = rr[iii];
			gx[iii] = gg[iii];
			bx[iii] = bb[iii];
		}
	} else if (code == (REPEATED_ALT_2 + 1)) {
		length = 48;
		double rr[MAX_INPUT_LENGTH] = { 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00 };
		double gg[MAX_INPUT_LENGTH] = { 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00 };
		double bb[MAX_INPUT_LENGTH] = { 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00 };
		for (int iii = 0; iii < length; iii++) {
			rx[iii] = rr[iii];
			gx[iii] = gg[iii];
			bx[iii] = bb[iii];
		}
	} else if (code == REPEATED_ALT_3) {
		length = 41;
		double rr[MAX_INPUT_LENGTH] = { 1.00, 0.67, 0.33,     0.00, 0.00, 0.00, 0.00, 0.00,    0.00, 0.00,     0.00,       0.05, 0.10,     0.40, 0.80, 0.90, 0.95, 1.00,   1.00, 1.00, 1.00, 0.67, 0.33,     0.00, 0.00, 0.00, 0.00, 0.00,    0.00, 0.00,     0.00,       0.05, 0.10,     0.40, 0.80, 0.90, 0.95, 1.00,   1.00, 1.00, 1.00 };
		double gg[MAX_INPUT_LENGTH] = { 1.00, 1.00, 1.00,     1.00, 0.75, 0.50, 0.25, 0.00,    0.00, 0.00,     0.00,       0.00, 0.00,     0.00, 0.10, 0.25, 0.45, 0.80,   0.95, 1.00, 1.00, 1.00, 1.00,     1.00, 0.75, 0.50, 0.25, 0.00,    0.00, 0.00,     0.00,       0.00, 0.00,     0.00, 0.10, 0.25, 0.45, 0.80,   0.95, 1.00, 1.00 };
		double bb[MAX_INPUT_LENGTH] = { 1.00, 1.00, 1.00,     1.00, 1.00, 1.00, 1.00, 1.00,    0.67, 0.33,     0.00,       0.25, 0.50,     0.60, 0.50, 0.05, 0.00, 0.05,   0.45, 0.70, 1.00, 1.00, 1.00,     1.00, 1.00, 1.00, 1.00, 1.00,    0.67, 0.33,     0.00,       0.25, 0.50,     0.60, 0.50, 0.05, 0.00, 0.05,   0.45, 0.70, 1.00 };
		for (int iii = 0; iii < length; iii++) {
			rx[iii] = rr[iii];
			gx[iii] = gg[iii];
			bx[iii] = bb[iii];
		}
	} else if (code == (REPEATED_ALT_3 + 1)) {
		length = 28;
		double rr[MAX_INPUT_LENGTH] = { 0.00, 0.00, 0.00, 0.00, 0.00, 0.00,    0.40, 0.70, 0.80, 0.90, 0.95, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00,    0.40, 0.70, 0.80, 0.90, 0.95, 1.00, 1.00, 1.00 };
		double gg[MAX_INPUT_LENGTH] = { 0.88, 0.75, 0.68, 0.50, 0.38, 0.25,    0.00, 0.00, 0.10, 0.25, 0.45, 0.60, 0.80, 0.95, 0.88, 0.75, 0.68, 0.50, 0.38, 0.25,    0.00, 0.00, 0.10, 0.25, 0.45, 0.60, 0.80, 0.95 };
		double bb[MAX_INPUT_LENGTH] = { 1.00, 1.00, 1.00, 1.00, 0.75, 0.50,    0.60, 0.60, 0.50, 0.05, 0.00, 0.00, 0.05, 0.45, 1.00, 1.00, 1.00, 1.00, 0.75, 0.50,    0.60, 0.60, 0.50, 0.05, 0.00, 0.00, 0.05, 0.45 };
		for (int iii = 0; iii < length; iii++) {
			rx[iii] = rr[iii];
			gx[iii] = gg[iii];
			bx[iii] = bb[iii];
		}
	} else if (code == REPEATED_ALT_4) {
		length = 12;
        double rr[MAX_INPUT_LENGTH] = { 0.00, 1.00, 0.00, 1.00, 0.00, 1.00, 0.00, 1.00, 0.00, 1.00, 0.00, 1.00 };
        double gg[MAX_INPUT_LENGTH] = { 0.00, 1.00, 0.00, 1.00, 0.00, 1.00, 0.00, 1.00, 0.00, 1.00, 0.00, 1.00 };
        double bb[MAX_INPUT_LENGTH] = { 0.00, 1.00, 0.00, 1.00, 0.00, 1.00, 0.00, 1.00, 0.00, 1.00, 0.00, 1.00 };
		for (int iii = 0; iii < length; iii++) {
			rx[iii] = rr[iii];
			gx[iii] = gg[iii];
			bx[iii] = bb[iii];
		}
	} else if (code == (REPEATED_ALT_4 + 1)) {
		length = 12;
        double rr[MAX_INPUT_LENGTH] = { 0.25, 0.75, 0.25, 0.75, 0.25, 0.75, 0.25, 0.75, 0.25, 0.75, 0.25, 0.75 };
        double gg[MAX_INPUT_LENGTH] = { 0.25, 0.75, 0.25, 0.75, 0.25, 0.75, 0.25, 0.75, 0.25, 0.75, 0.25, 0.75 };
        double bb[MAX_INPUT_LENGTH] = { 0.25, 0.75, 0.25, 0.75, 0.25, 0.75, 0.25, 0.75, 0.25, 0.75, 0.25, 0.75 };
 		for (int iii = 0; iii < length; iii++) {
			rx[iii] = rr[iii];
			gx[iii] = gg[iii];
			bx[iii] = bb[iii];
		}
	} else if (code == REPEATED_ALT_5) {
		length = 6;
		double rr[MAX_INPUT_LENGTH] = { 0.00, 0.00, 0.00, 1.00, 1.00, 1.00 };
		double gg[MAX_INPUT_LENGTH] = { 0.00, 1.00, 1.00, 1.00, 0.00, 0.00 };
		double bb[MAX_INPUT_LENGTH] = { 1.00, 1.00, 0.00, 0.00, 0.00, 1.00 };
		for (int iii = 0; iii < length; iii++) {
			rx[iii] = rr[iii];
			gx[iii] = gg[iii];
			bx[iii] = bb[iii];
		}
	} else if (code == (REPEATED_ALT_5 + 1)) {
		length = 6;
		double rr[MAX_INPUT_LENGTH] = { 0.00, 0.00, 0.00, 1.00, 1.00, 1.00 };
		double gg[MAX_INPUT_LENGTH] = { 0.00, 1.00, 1.00, 1.00, 0.00, 0.00 };
		double bb[MAX_INPUT_LENGTH] = { 1.00, 1.00, 0.00, 0.00, 0.00, 1.00 };
		for (int iii = 0; iii < length; iii++) {
			rx[iii] = rr[iii];
			gx[iii] = gg[iii];
			bx[iii] = bb[iii];
		}
	} else if (code == REPEATED_ALT_6) {
		length = 48;
		double rr[MAX_INPUT_LENGTH] = { 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00 };
		double gg[MAX_INPUT_LENGTH] = { 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00 };
		double bb[MAX_INPUT_LENGTH] = { 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00 };
		for (int iii = 0; iii < length; iii++) {
			rx[iii] = rr[iii];
			gx[iii] = gg[iii];
			bx[iii] = bb[iii];
		}
	} else if (code == (REPEATED_ALT_6 + 1)) {
		length = 48;
		double rr[MAX_INPUT_LENGTH] = { 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00 };
		double gg[MAX_INPUT_LENGTH] = { 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00 };
		double bb[MAX_INPUT_LENGTH] = { 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00, 1.00, 1.00, 0.00, 0.00, 0.00, 1.00 };
		for (int iii = 0; iii < length; iii++) {
			rx[iii] = rr[iii];
			gx[iii] = gg[iii];
			bx[iii] = bb[iii];
		}
	}

	if (!flexibleMarkers) {
		
		for (int iii = 0; iii < length; iii++) {
			dx[iii] = (double(iii) / double((length-1.0)));
		}
	}

	/*
	for (int iii = 0; iii < length; iii++) {
		printf("%s << (dx,rx,gx,bx)(%d) = (%1.2f, %1.2f, %1.2f, %1.2f)\n", __FUNCTION__, iii, dx[iii], rx[iii], gx[iii], bx[iii]);
	}
	*/
	
	create_long_map();		// Maybe this line is causing the problems...


	setupLookupTable(1);	// This line is exhibiting some problems...
	setupLookupTable(2);

}

int cScheme::current_scheme() {
	return code;
}

void cScheme::falsify_image(const cv::Mat& thermIm, cv::Mat& outputIm) {

	if ((outputIm.size() != thermIm.size()) || (outputIm.type() != CV_8UC3)) {
		outputIm = cv::Mat::zeros(thermIm.size(), CV_8UC3);
	}

	int lookupIndex = 0;

	#pragma omp parallel for
	for (int i = 0; i < thermIm.size().height; i++)	{
		for (int j = 0; j < thermIm.size().width; j++) {
			if (thermIm.depth() == 2) {
				lookupIndex = thermIm.at<unsigned short>(i,j);
				outputIm.at<cv::Vec3b>(i,j)[2] = uchar(lookupTable_2[lookupIndex][2]);
				outputIm.at<cv::Vec3b>(i,j)[1] = uchar(lookupTable_2[lookupIndex][1]);
				outputIm.at<cv::Vec3b>(i,j)[0] = uchar(lookupTable_2[lookupIndex][0]);
			} else if (thermIm.depth() == 0) {
				lookupIndex = thermIm.at<unsigned char>(i,j);
				outputIm.at<cv::Vec3b>(i,j)[2] = lookupTable_1[lookupIndex][2];
				outputIm.at<cv::Vec3b>(i,j)[1] = lookupTable_1[lookupIndex][1];
				outputIm.at<cv::Vec3b>(i,j)[0] = lookupTable_1[lookupIndex][0];
			}
		}
	}
}

void cScheme::image_resize(cv::Mat& inputIm, int dim_i, int dim_j) {
	cv::Mat newIm;
	newIm = cv::Mat(dim_i,dim_j,inputIm.type());

	double val;

	int ni, nj;

	ni = inputIm.size().height;
	nj = inputIm.size().width;

	for (int i = 0; i < dim_i; i++) {
		for (int j = 0; j < dim_j; j++) {
			//val = (*inputIm)(((int)((i*inputIm->ni())/dim_i)),((int)((j*inputIm->nj())/dim_j)));
			val = inputIm.at<unsigned char>(((int)((i*ni)/dim_i)),((int)((j*nj)/dim_j)));

			newIm.at<unsigned char>(i,j) = (unsigned char)(val);
		}
	}

	//inputIm->set_size(dim_i, dim_j);
	//inputIm->deep_copy(*newIm);
    inputIm = cv::Mat(newIm);

}

void cScheme::forge_image(cv::Mat& thermIm, cv::Mat& visualIm, cv::Mat& outputIm, double* params, double thresh) {
	
	outputIm = cv::Mat(thermIm.size(), CV_8UC3);
	
	double working_params[2];
	
	if (params == NULL) {
		working_params[0] = 0.2;
		working_params[1] = 0.8;
	} else {
		working_params[0] = params[0];
		working_params[1] = params[1];
	}
	
	double vals[2], percentiles[2];
	percentiles[0] = thresh;
	percentiles[1] = 1 - thresh;
	
	findPercentiles(thermIm, vals, percentiles, 2);
	
	/*
	printf("%s << percentiles[0] = (%f), percentiles[1] = (%f)\n", __FUNCTION__, vals[0], vals[1]);
	printf("%s << red[0] = (%d), red[1] = (%d)\n", __FUNCTION__, lookupTable_1[0][0], lookupTable_1[65535][0]);
	printf("%s << green[0] = (%d), green[1] = (%d)\n", __FUNCTION__, lookupTable_1[0][1], lookupTable_1[65535][1]);
	printf("%s << blue[0] = (%d), blue[1] = (%d)\n", __FUNCTION__, lookupTable_1[0][2], lookupTable_1[65535][2]);
	*/
	
	double lumChange[3];
	double maxLumChange;
	
	for (int iii = 0; iii < thermIm.rows; iii++) {
		for (int jjj = 0; jjj < thermIm.cols; jjj++) {
			
			if (thermIm.depth() == 2) {
				if (thermIm.at<unsigned short>(iii,jjj) < (unsigned short) vals[0]) {
					outputIm.at<cv::Vec3b>(iii,jjj)[0] = uchar(lookupTable_2[thermIm.at<unsigned short>(iii,jjj)][0]);
					outputIm.at<cv::Vec3b>(iii,jjj)[1] = uchar(lookupTable_2[thermIm.at<unsigned short>(iii,jjj)][1]);
					outputIm.at<cv::Vec3b>(iii,jjj)[2] = uchar(lookupTable_2[thermIm.at<unsigned short>(iii,jjj)][2]);
				} else if (thermIm.at<unsigned short>(iii,jjj) > (unsigned short) vals[1]) {
					outputIm.at<cv::Vec3b>(iii,jjj)[0] = uchar(lookupTable_2[thermIm.at<unsigned short>(iii,jjj)][0]);
					outputIm.at<cv::Vec3b>(iii,jjj)[1] = uchar(lookupTable_2[thermIm.at<unsigned short>(iii,jjj)][1]);
					outputIm.at<cv::Vec3b>(iii,jjj)[2] = uchar(lookupTable_2[thermIm.at<unsigned short>(iii,jjj)][2]);
				} else {
					
					lumChange[0] = 2.0 * (((double) visualIm.at<cv::Vec3b>(iii,jjj)[0])/255.0 - 0.5) * (working_params[1] - working_params[0]);
					lumChange[1] = 2.0 * (((double) visualIm.at<cv::Vec3b>(iii,jjj)[1])/255.0 - 0.5) * (working_params[1] - working_params[0]);
					lumChange[2] = 2.0 * (((double) visualIm.at<cv::Vec3b>(iii,jjj)[2])/255.0 - 0.5) * (working_params[1] - working_params[0]);
					
					maxLumChange = max(lumChange[0], lumChange[1]);
					maxLumChange = max(maxLumChange, lumChange[2]);
					
					outputIm.at<cv::Vec3b>(iii,jjj)[0] = uchar(((double) visualIm.at<cv::Vec3b>(iii, jjj)[0]) * (working_params[1] - working_params[0]) + (working_params[0] * 255.0));
					outputIm.at<cv::Vec3b>(iii,jjj)[1] = uchar(((double) visualIm.at<cv::Vec3b>(iii, jjj)[1]) * (working_params[1] - working_params[0]) + (working_params[0] * 255.0));
					outputIm.at<cv::Vec3b>(iii,jjj)[2] = uchar(((double) visualIm.at<cv::Vec3b>(iii, jjj)[2]) * (working_params[1] - working_params[0]) + (working_params[0] * 255.0));
					/*
					if (lumChange[0] > 0) {
						outputIm.at<cv::Vec3b>(iii,jjj)[0] = outputIm.at<cv::Vec3b>(iii, jjj)[0] + (255.0 - outputIm.at<cv::Vec3b>(iii, jjj)[0]) * maxLumChange;
					} else {
						outputIm.at<cv::Vec3b>(iii,jjj)[0] = outputIm.at<cv::Vec3b>(iii, jjj)[0] + (outputIm.at<cv::Vec3b>(iii, jjj)[0] * maxLumChange);
					}
					
					if (lumChange[1] > 0) {
						outputIm.at<cv::Vec3b>(iii,jjj)[1] = outputIm.at<cv::Vec3b>(iii, jjj)[1] + (255.0 - outputIm.at<cv::Vec3b>(iii, jjj)[0]) * maxLumChange;
					} else {
						outputIm.at<cv::Vec3b>(iii,jjj)[1] = outputIm.at<cv::Vec3b>(iii, jjj)[1] + (outputIm.at<cv::Vec3b>(iii, jjj)[0] * maxLumChange);
					}
					
					if (lumChange[2] > 0) {
						outputIm.at<cv::Vec3b>(iii,jjj)[2] = outputIm.at<cv::Vec3b>(iii, jjj)[2] + (255.0 - outputIm.at<cv::Vec3b>(iii, jjj)[0]) * maxLumChange;
					} else {
						outputIm.at<cv::Vec3b>(iii,jjj)[2] = outputIm.at<cv::Vec3b>(iii, jjj)[2] + (outputIm.at<cv::Vec3b>(iii, jjj)[0] * maxLumChange);
					}
					*/
					/*
					if (lumChange[0] > 0) {
						outputIm.at<cv::Vec3b>(iii,jjj)[0] = outputIm.at<cv::Vec3b>(iii, jjj)[0] + (255.0 - outputIm.at<cv::Vec3b>(iii, jjj)[0]) * lumChange[0];
					} else {
						outputIm.at<cv::Vec3b>(iii,jjj)[0] = outputIm.at<cv::Vec3b>(iii, jjj)[0] + (outputIm.at<cv::Vec3b>(iii, jjj)[0] * lumChange[0]);
					}
					
					if (lumChange[1] > 0) {
						outputIm.at<cv::Vec3b>(iii,jjj)[1] = outputIm.at<cv::Vec3b>(iii, jjj)[1] + (255.0 - outputIm.at<cv::Vec3b>(iii, jjj)[0]) * lumChange[1];
					} else {
						outputIm.at<cv::Vec3b>(iii,jjj)[1] = outputIm.at<cv::Vec3b>(iii, jjj)[1] + (outputIm.at<cv::Vec3b>(iii, jjj)[0] * lumChange[1]);
					}
					
					if (lumChange[2] > 0) {
						outputIm.at<cv::Vec3b>(iii,jjj)[2] = outputIm.at<cv::Vec3b>(iii, jjj)[2] + (255.0 - outputIm.at<cv::Vec3b>(iii, jjj)[0]) * lumChange[2];
					} else {
						outputIm.at<cv::Vec3b>(iii,jjj)[2] = outputIm.at<cv::Vec3b>(iii, jjj)[2] + (outputIm.at<cv::Vec3b>(iii, jjj)[0] * lumChange[2]);
					}
					*/
					
					/*
					outputIm.at<cv::Vec3b>(iii,jjj)[0] = visualIm.at<cv::Vec3b>(iii,jjj)[0];
					outputIm.at<cv::Vec3b>(iii,jjj)[1] = visualIm.at<cv::Vec3b>(iii,jjj)[1];
					outputIm.at<cv::Vec3b>(iii,jjj)[2] = visualIm.at<cv::Vec3b>(iii,jjj)[2];
					*/
				}
				

			}
			
			
		}
	}
	
}

void cScheme::fuse_image(cv::Mat& thermIm, cv::Mat& visualIm, cv::Mat& outputIm, double* params) {

	double working_params[2];
	
	bool alreadyMapped = false;
	
	if (params == NULL) {
		working_params[0] = DEFAULT_LOWER_VISIBLE_FUSION_LIMIT;
		working_params[1] = DEFAULT_UPPER_VISIBLE_FUSION_LIMIT;
	} else {
		working_params[0] = params[0];
		working_params[1] = params[1];
	}

	double lumChange;

	unsigned char sr, sg, sb;

	cv::Mat newTherm;
	
	outputIm = cv::Mat(newTherm.size(), CV_8UC3);

	if (thermIm.channels() > 1) {
		
		if (checkIfActuallyGray(thermIm)) {
			#ifdef _OPENCV_VERSION_3_PLUS_
			cvtColor(thermIm, newTherm, cv::COLOR_RGB2GRAY);
			#else
			cvtColor(thermIm, newTherm, CV_RGB2GRAY);
			#endif
		} else {
			thermIm.copyTo(outputIm);
			alreadyMapped = true;
		}
        
    } else {
        thermIm.copyTo(newTherm);
    }

	cv::Mat newVisual;

    if (visualIm.channels() > 1) {
		#ifdef _OPENCV_VERSION_3_PLUS_
		cvtColor(visualIm, newVisual, cv::COLOR_RGB2GRAY);
		#else
		cvtColor(visualIm, newVisual, CV_RGB2GRAY);
		#endif
    } else {
        visualIm.copyTo(newVisual);
    }

	// If images aren't the same size, resize
	if (thermIm.size() != visualIm.size()) {
		// Determine which one is bigger and resize accordingly
		if (thermIm.size().height > thermIm.size().width) {
			image_resize(newVisual, thermIm.size().height, thermIm.size().width);
		} else {
			image_resize(newTherm, visualIm.size().height, visualIm.size().width);
		}
	}

	if (!alreadyMapped) {
		falsify_image(newTherm, outputIm);
	}

	// For each pixel
	for (int i = 0; i < outputIm.size().height; i++)	{
		for (int j = 0; j < outputIm.size().width; j++) {

			lumChange = 2.0 * (((double) newVisual.at<unsigned char>(i,j))/255.0 - 0.5) * (working_params[1] - working_params[0]);

			if (lumChange > 0) {
				sr = (unsigned char)(outputIm.at<cv::Vec3b>(i, j)[0] + (255.0 - outputIm.at<cv::Vec3b>(i, j)[0]) * lumChange);
				sg = (unsigned char)(outputIm.at<cv::Vec3b>(i, j)[1] + (255.0 - outputIm.at<cv::Vec3b>(i, j)[1]) * lumChange);
				sb = (unsigned char)(outputIm.at<cv::Vec3b>(i, j)[2] + (255.0 - outputIm.at<cv::Vec3b>(i, j)[2]) * lumChange);
			} else {
				sr = (unsigned char)(outputIm.at<cv::Vec3b>(i, j)[0] + (outputIm.at<cv::Vec3b>(i, j)[0] * lumChange));
				sg = (unsigned char)(outputIm.at<cv::Vec3b>(i, j)[1] + (outputIm.at<cv::Vec3b>(i, j)[1] * lumChange));
				sb = (unsigned char)(outputIm.at<cv::Vec3b>(i, j)[2] + (outputIm.at<cv::Vec3b>(i, j)[2] * lumChange));
			}

			outputIm.at<cv::Vec3b>(i,j)[0] = sr;
			outputIm.at<cv::Vec3b>(i,j)[1] = sg;
			outputIm.at<cv::Vec3b>(i,j)[2] = sb;
		}
	}

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
    double minVal = 9e99, maxVal = -9e99;

    for (int iii = 0; iii < origMat.rows; iii++) {
        for (int jjj = 0; jjj < origMat.cols; jjj++) {

            // printf("%s << origMat.val = %f\n", __FUNCTION__, origMat.at<double>(iii,jjj));


            if (origMat.at<double>(iii,jjj) > maxVal) {
                maxVal = origMat.at<double>(iii,jjj);
            }

            if (origMat.at<double>(iii,jjj) < minVal) {
                minVal = origMat.at<double>(iii,jjj);
            }
        }
    }

    cv::Mat displayMat(origMat.size(), CV_8UC1);

    for (int iii = 0; iii < origMat.rows; iii++) {
        for (int jjj = 0; jjj < origMat.cols; jjj++) {
            displayMat.at<unsigned char>(iii,jjj) = (unsigned char) ((origMat.at<double>(iii,jjj) - minVal) * 255.0 / (maxVal - minVal));
        }
    }

    cv::Mat largerMat;

    resize(displayMat, largerMat, cv::Size(origMat.rows*1, origMat.cols*1), 0, 0, cv::INTER_NEAREST);

    return largerMat;
}
