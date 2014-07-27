/*! \file	improc.hpp
 *  \brief	Declarations for image processing.
*/

#ifndef _THERMALVIS_IMPROC_H_
#define _THERMALVIS_IMPROC_H_

#include "opencv2/opencv.hpp"

#include "tools.hpp"

#include "streamer/streamer_defines.hpp"
//#include "input_stream_config.hpp"

/// \brief		ID for custom mapping
namespace tv {
	#define MAP_LENGTH		1024

	#define MAX_INPUT_LENGTH	256

	#define GRAYSCALE		0

	#define CIECOMP			100
	#define CIECOMP_ALT_1	110
	#define CIECOMP_ALT_2	120
	#define CIECOMP_ALT_3	130

	#define CIELUV			140

	#define BLACKBODY		150

	#define HIGHLIGHTED		180

	#define RAINBOW			200
	#define RAINBOW_ALT_1	210
	#define RAINBOW_ALT_2	220
	#define RAINBOW_ALT_3	230
	#define RAINBOW_ALT_4	240

	#define IRON			300
	#define IRON_ALT_1		310
	#define IRON_ALT_2		320
	#define IRON_ALT_3		330

	#define BLUERED			400
	#define BLUERED_ALT_1	410
	#define BLUERED_ALT_2	420

	#define JET				500
	#define JET_ALT_1		510

	#define ICE				600
	#define ICE_ALT_1		610
	#define ICE_ALT_2		620
	#define ICE_ALT_3		630

	#define ICEIRON			700
	#define ICEIRON_ALT_1	710
	#define ICEIRON_ALT_2	720

	#define ICEFIRE			800
	#define ICEFIRE_ALT_1	810
	#define ICEFIRE_ALT_2	820
	#define ICEFIRE_ALT_3	830

	#define REPEATED		900
	#define REPEATED_ALT_1	910
	#define REPEATED_ALT_2	920
	#define REPEATED_ALT_3	930
	#define REPEATED_ALT_4	940
	#define REPEATED_ALT_5	950
	#define REPEATED_ALT_6	960

	#define MIN_PROP_THRESHOLD 0.002

	#define DEFAULT_LOWER_VISIBLE_FUSION_LIMIT 		0.2
	#define DEFAULT_UPPER_VISIBLE_FUSION_LIMIT 		0.8
}

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

/// \brief 		Converts a 16-bit raw image into an 8-bit image based on desired temperature range and provided median value
void temperatureRangeBasedDownsample(const cv::Mat& src, cv::Mat& dst, int newMedian = -1, double degreesPerGraylevel = 0.01, double desiredDegreesPerGraylevel = 0.05);

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


/// \brief		For configuring and applying false-colormapping or modality-fusion schemes
class cScheme {
protected:
	
	/// \brief		Short array of the intensities corresponding to the color mappings (rx, gx, bx)
	double dx[MAX_INPUT_LENGTH];
	
	/// \brief		Short array of red-weightings of intensity mappings
	double rx[MAX_INPUT_LENGTH];
	/// \brief		Long array of red-weightings of intensity mappings
	double red[MAP_LENGTH];
	/// \brief		Short array of green-weightings of intensity mappings
	double gx[MAX_INPUT_LENGTH];
	/// \brief		Long array of green-weightings of intensity mappings
	double green[MAP_LENGTH];
	/// \brief		Short array of blue-weightings of intensity mappings
	double bx[MAX_INPUT_LENGTH];
	/// \brief		Long of blue-weightings of intensity mappings
	double blue[MAP_LENGTH];
	/// \brief		Number of elements in shortened colour arrays
	int		length;
	/// \brief		Code identifying which map is being used [see load_standard() function]
	int		code;

	/// \brief		Lookup table which shows what colour intensities correspond to what raw intensities
	unsigned char	lookupTable_1[256][3];
	unsigned short	lookupTable_2[65536][3];

public:
	/// \brief 		Constructor.
	cScheme();

	/// \brief 		Constructor.
	/// \param 		mapCode: 1 for safe, 0 for unsafe (with blacks and whites)
	cScheme(int mapCode, int mapParam = 0);

	/// \brief 		Destructor.
	~cScheme();

	/// \brief 		Load a standard colour map in as the scheme.
	/// \param 		mapCode: 1 for safe, 0 for unsafe (with blacks and whites)
	void load_standard(int mapCode, int mapParam = 0);

	/// \brief 		Creates a long map using the shortened map.
	void create_long_map();

	/// \brief		Recreates lookup table
	void setupLookupTable(unsigned int depth = 2);

	/// \brief 		Create and apply a new colour map as the scheme.
	/// \param 		r		Pointer to red weightings
	/// \param 		g		Pointer to green weightings
	/// \param 		b		Pointer to blue weightings
	/// \param 		len		Length of all three arrays
	// void customize(double* d, double* r, double* g, double* b, int len);

	/// \brief 		Create a false colour version of a monochromatic thermal image.
	/// \param 		thermIm		Monochrocv::Matic Thermal Image
	/// \param 		outputIm	Colour output Image
	void falsify_image(const cv::Mat& thermIm, cv::Mat& outputIm);

	/// \brief 		Create a false colour combination of a colour or monochrome visual image and a monochrome thermal image.
	/// \param 		thermIm		Monochromatic Thermal Image
	/// \param 		visualIm	Colour or monochromatic Visual Image
	/// \param 		outputIm	Colour output Image
	/// \param 		params		Pointer to double values dictating the following parameters:
	///							[0] : minimum lightness (default = 0.2)
	///							[1] : maximum lightness (default = 0.8)
	void fuse_image(cv::Mat& thermIm, cv::Mat& visualIm, cv::Mat& outputIm, double *params = NULL);
	
	/// \brief 		Combines RGB and thermal image using some other scheme
	void forge_image(cv::Mat& thermIm, cv::Mat& visualIm, cv::Mat& outputIm, double* params = NULL, double thresh = 0.05);

	/// \brief 		Returns code for current scheme.
	/// \param 		output		0 = rainbow, 1 = iron, 2 = jet, 3 = custom
	int current_scheme();

	/// \brief 		Resizes the given image, preserving the underlying data (to some extent).
	/// \param 		inputIm		Input image
	/// \param 		dim_i		Width
	/// \param 		dim_j		Height
	void image_resize(cv::Mat& inputIm, int dim_i, int dim_j);


};

#endif
