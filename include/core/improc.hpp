/*! \file	improc.hpp
 *  \brief	Declarations for image processing.
*/

#ifndef THERMALVIS_IMPROC_H
#define THERMALVIS_IMPROC_H

#include "opencv2/opencv.hpp"

#include "core/tools.hpp"
#include "core/opencv_redefinitions.hpp"

#ifdef _OPENCV_VERSION_3_PLUS_
#define CV_COLOR_RED		RGB_(0,		0,		255)
#define CV_COLOR_GREEN		RGB_(0,		255,	0)
#define CV_COLOR_BLUE		RGB_(255,	0,		0)
#define CV_COLOR_LRED		RGB_(128,	128,	255)
#define CV_COLOR_YELLOW		RGB_(0,		255,	255)
#define CV_COLOR_PURPLE		RGB_(255,	0,		255)
#else
#define CV_COLOR_RED		RGB_(255,	0,		0)
#define CV_COLOR_GREEN		RGB_(0,		255,	0)
#define CV_COLOR_BLUE		RGB_(0,		0,		255)
#define CV_COLOR_LRED		RGB_(255,	128,	128)
#define CV_COLOR_YELLOW		RGB_(255,	255,	0)
#define CV_COLOR_PURPLE		RGB_(255,	0,		255)
#endif

#include "streamer/streamer_defines.hpp"

#define MIN_PIXELS_PER_AXIS_FOR_DISPLAY 400

#ifdef _USE_QT_
#include <QApplication>
#include <QLabel>
QImage Mat2QImage(const cv::Mat &src);
#endif

enum E_ImageDatatype
{
  DATATYPE_INVALID  = -1,
  DATATYPE_8BIT     =  0,
  DATATYPE_RAW      =  1,
  DATATYPE_MM       =  2,
  DATATYPE_DEPTH    =  3,
  NO_OF_DATATYPES   // Must be the last value
};

void denoiseImage(const cv::Mat& src, cv::Mat& dst, int denoisingMode = DENOISING_MODE_X);

void fadeImage(const cv::Mat& src, cv::Mat& dst, double frac = 0.65);

void applyFilter(const cv::Mat& src, cv::Mat& dst, int filter = NO_FILTERING, double param = 2.0);

//void straightCLAHE(const cv::Mat& src, cv::Mat& dst, double factor);

double findBestAlpha(const cv::Mat& K, const cv::Mat& coeff, const cv::Size& camSize);

void weightedMixture(cv::Mat& dst, const std::vector<cv::Mat>& srcs, const std::vector<double>& weightings);

void addBorder(cv::Mat& inputMat, int borderSize);

/// \brief 		Generates a point that's half way in between the two input points
cv::Point2f meanPoint(cv::Point2f& P1, cv::Point2f& P2);

/// \brief      Finds the minimum separation between any two points in a set
double findMinimumSeparation(vector<cv::Point2f>& pts);

void readPoints(const char *filename, vector<cv::Point2f>& pts);
void writePoints(const char *filename, const vector<cv::Point2f>& pts);

void normalize_64_vec(cv::Mat& dst, cv::Mat& src);

/// \brief      Redistorts points using an established distortion model
void redistortPoints(const vector<cv::Point2f>& src, vector<cv::Point2f>& dst, const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs, const cv::Mat& newCamMat=cv::Mat::eye(3,3,CV_64FC1));

/// \brief 		Calculates the perpindicular distance between a line (P1 to P2) and a point (P3)
double perpDist(cv::Point2f& P1, cv::Point2f& P2, cv::Point2f& P3);

/// \brief      Distance between two 3D points in double format
double distBetweenPts(cv::Point3d& P1, cv::Point3d& P2);
cv::Scalar getRandomColour();
double lookupValue(double xi, double yi, double maxVal, const cv::Mat& lookupMat);

/// \brief 		Calculates the distance between two "Point" points
double distBetweenPts(cv::Point& P1, cv::Point& P2);
/// \brief      Calculates the distance between two "Point2f" points
double distBetweenPts2f(cv::Point2f& P1, cv::Point2f& P2);

bool matricesAreEqual(cv::Mat& mat1, cv::Mat& mat2);

void mixImages(cv::Mat& dst, std::vector<cv::Mat>& images);

double getInterpolatedVal(const cv::Mat& img, cv::Point2f& coord);

bool rectangle_contains_centroid(cv::Rect mainRectangle, cv::Rect innerRectangle);
cv::Rect meanRectangle(vector<cv::Rect>& rectangles);
void clusterRectangles(vector<cv::Rect>& rectangles, double minOverlap);

cv::Point rectangle_center(cv::Rect input);

void combineImages(const cv::Mat& im1, const cv::Mat& im2, cv::Mat& dst);

/// \brief      Stores a 16-bit monochrome image in the first 2 channels of a CV_8UC3 mat
void unpackTo8bit3Channel(const cv::Mat& src, cv::Mat& dst);

double rectangleOverlap(cv::Rect rectangle1, cv::Rect rectangle2);

void obtainEightBitRepresentation(cv::Mat& src, cv::Mat& dst);
void obtainColorRepresentation(cv::Mat& src, cv::Mat& dst);

/// \brief      Trims the top/bottom or sides of the image to get the correct ratio
void trimToDimensions(cv::Mat& image, int width, int height);

/// \brief      Very basic image cropping
void cropImage(cv::Mat& image, cv::Point tl, cv::Point br);

void drawGrid(const cv::Mat& src, cv::Mat& dst, int mode = 0);

/// \brief 		Convert a vector from 'Point' format to 'Point2f' format
void convertVectorToPoint2f(vector<cv::Point>& input, vector<cv::Point2f>& output);

/// \brief      Convert a vector from 'Point2f' format to 'Point' format
void convertVectorToPoint(vector<cv::Point2f>& input, vector<cv::Point>& output);

void splitMultimodalImage(const cv::Mat& src, cv::Mat& therm, cv::Mat& vis);

/// \brief      Finds centroid of a contour
cv::Point findCentroid(vector<cv::Point>& contour);

/// \brief      Finds centroid of a contour
cv::Point2f findCentroid2f(vector<cv::Point>& contour);

/// \brief      Resizes an image without interpolation
void simpleResize(cv::Mat& src, cv::Mat& dst, cv::Size size);

/// \brief      Sets pixel values in 16-bit image that are above or below limits to the limits themselves
void thresholdRawImage(cv::Mat& img, double *vals);

/// \brief      Creates a gaussian intensity distribution in a given matrix
void createGaussianMatrix(cv::Mat& gaussianMat, double sigmaFactor = 1.0);

/// \brief      Makes a copy of a contour
void copyContour(vector<cv::Point>& src, vector<cv::Point>& dst);

/// \brief      Inverts the pixel intensities of a matrix
void invertMatIntensities(const cv::Mat& src, cv::Mat& dst);

/// \brief 		Swaps the position of two elements in a point vector
void swapElements(vector<cv::Point2f>& corners, int index1, int index2);

/// \brief      Determine the width and height of a contour (in x and y directions at this stage)
void contourDimensions(vector<cv::Point> contour, double& width, double& height);

/// \brief      Draws lines between initial and corrected points
void drawLinesBetweenPoints(cv::Mat& image, const vector<cv::Point2f>& src, const vector<cv::Point2f>& dst);

/// \brief      Returns a score which reflects the information content of the image for visualization
double scoreColorImage(const cv::Mat& src);

/// \brief      Returns a score which reflects the information content of the image for visualization
double scoreThermalImage(const cv::Mat& src);

/// \brief 		Move a point from one vector to another, resize and shift old vector
void transferElement(vector<cv::Point2f>& dst, vector<cv::Point2f>& src, int index);

/// \brief 		Stretches the histogram to span the whole 16-bit scale (16-bit to 16-bit)
void normalize_16(cv::Mat& dst, const cv::Mat& src, double dblmin = -1.0, double dblmax = -1.0);

void histExpand8(const cv::Mat& src, cv::Mat& dst);
void reduceToPureImage(cv::Mat& dst, cv::Mat& src);
void fix_bottom_right(cv::Mat& mat);

/// \brief 		Converts a 16-bit matrix to a 3 channel 8 bit matrix, with the third channel assigned confidence values
void convert16bitTo8bitConfidence(const cv::Mat& src, const cv::Mat& conf, cv::Mat& dst);

void fixedDownsample(const cv::Mat& src, cv::Mat& dst, double center, double range);


/// \brief 		Downsamples a temperature matrix to an 8-bit image
void temperatureDownsample(const cv::Mat& src, cv::Mat& dst, double minVal, double maxVal);

/// \brief 		Converts a 16-bit image encoding temperature into floating point format
void convertToTemperatureMat(const cv::Mat& src, cv::Mat& dst, double grad = 10, int intercept = 1000);

/// \brief 		Converts the DPG (Degrees Per Graylevel) of an 8-bit image but preserves the median value
void temperatureRangeBasedResample(const cv::Mat& src, cv::Mat& dst, double degreesPerGraylevel, double desiredDegreesPerGraylevel);

/// \brief 		Converts a 16-bit raw image into an 8-bit image based on desired temperature range and provided median value
void temperatureRangeBasedDownsample(const cv::Mat& src, cv::Mat& dst, int newMedian = -1, double degreesPerGraylevel = 0.01, double desiredDegreesPerGraylevel = 0.05);

/// \brief 		Reads an image from a specified path 
cv::Mat read_image_from_file(std::string path);

/// \brief 		Determine the type of frame based on the matrix
E_ImageDatatype determineFrameType( cv::Mat& frame );

/// \brief 		Downsamples a temperature matrix to a 16-bit image using Optris-like linear scaling
void temperatureDownsample16(const cv::Mat& src, cv::Mat& dst);

//void downsampleCLAHE(const cv::Mat& src, cv::Mat& dst, double factor);
void adaptiveDownsample(const cv::Mat& src, cv::Mat& dst, int code = NORM_MODE_FULL_STRETCHING, double factor = 0.0);

void process8bitImage(const cv::Mat& src, cv::Mat& dst, int code = NORM_MODE_FULL_STRETCHING, double factor = 0.0);

bool checkIfActuallyGray(const cv::Mat& im);

void findIntensityValues(double *vals, cv::Mat& im, cv::Mat& mask);
void shiftIntensities(cv::Mat& im, double scaler, double shifter, double downer);
void findPercentiles(const cv::Mat& img, double *vals, double *percentiles, unsigned int num);

/// \brief      Obtains histogram and other image statistics
void generateHistogram(cv::Mat& src, cv::Mat& dst, double* im_hist, double* im_summ, double* im_stat);

/// \brief 		16-bit to 8-bit
void down_level(cv::Mat& dst, cv::Mat& src);

void applyIntensityShift(const cv::Mat& src1, cv::Mat& dst1, const cv::Mat& src2, cv::Mat& dst2, double grad, double shift);

cv::Mat normForDisplay(cv::Mat origMat);




#endif // THERMALVIS_IMPROC_H
