/*! \file	calibration.hpp
 *  \brief	Declarations for generic geometric calibration.
*/

#ifndef THERMALVIS_CALIBRATION_HPP
#define THERMALVIS_CALIBRATION_HPP

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "core/tools.hpp"
#include "core/improc.hpp"
#include "core/opencv_redefinitions.hpp"

#include <sys/stat.h>
#include <stdio.h>

#ifdef _WIN32
#include <ctime>
#endif

#define CHESSBOARD_FINDER_CODE          0
#define MASK_FINDER_CODE                1
#define HEATED_CHESSBOARD_FINDER_CODE   2

#define MIN_CORNER_SEPARATION 3
#define MAX_CORNER_SEPARATION 100

#define ALL_PATTERNS_OPTIMIZATION_CODE              0
#define RANDOM_SET_OPTIMIZATION_CODE                1
#define FIRST_N_PATTERNS_OPTIMIZATION_CODE          2
#define ENHANCED_MCM_OPTIMIZATION_CODE              3
#define BEST_OF_RANDOM_PATTERNS_OPTIMIZATION_CODE   4
#define EXHAUSTIVE_SEARCH_OPTIMIZATION_CODE         5
#define RANDOM_SEED_OPTIMIZATION_CODE               6
#define SCORE_BASED_OPTIMIZATION_CODE               7

#define DEBUG_MODE 									0

#define MINIMUM_CORRECTION_DISTANCE 				1

#define PATTERN_FINDER_CV_CORNER_SUBPIX_FLAG 4

#define TRACKING 0
#define RADIAL_LENGTH 1000
#define FOLD_COUNT 1

#define PI 3.14159265

#define MAX_CAMS 3
#define MAX_SEARCH_DIST 3

#define REGULAR_OPENCV_CHESSBOARD_FINDER    0
#define MASK_FINDER                         3
#define MASK_FINDER_INNERS                  8
#define EXTENDED_CHESSBOARD_FINDER          5
#define INVERTED_OPENCV_CHESSBOARD_FINDER   10

#define MIN_DISTANCE_FROM_EDGE 2

#define MAX_FRAMES_PER_INPUT    5000
#define MAX_FRAMES_TO_LOAD      1000
#define MAX_PATTERNS_TO_KEEP    500
#define MAX_CANDIDATE_PATTERNS  100
#define MAX_PATTERNS_PER_SET    10

#define MINIMUM_MSER_AREA		50

#define PATCH_CORRECTION_INTRINSICS_FLAGS cv::CALIB_RATIONAL_MODEL

/// \brief		Class enables for storing an MSER feature beyond simply its bounding points
class mserPatch
{
public:
    /// \brief		Point vector representing the bounding points of the patch
    vector<cv::Point> hull;
    /// \brief		Integer pixel location of the patches centroid
    cv::Point centroid;
    /// \brief		Integer pixel location of the patches centroid
    cv::Point2f centroid2f;
    /// \brief		A vector storing all of the moments associated with the patch
    cv::Moments momentSet;
    /// \brief		The total area in pixels of the patch
    double area;
    /// \brief		An estimate of the mean intensity of pixels within the patch
    double meanIntensity;
    /// \brief		An estimate of the variance of intensity of pixels within the patch
    double varIntensity;

    /// \brief 		Default Constructor.
    mserPatch();

    /// \brief 		Constructor from a point vector and a pointer to the matrix (for intensities).
    mserPatch(vector<cv::Point>& inputHull, const cv::Mat& image);
};

/// \brief      Generates a random set of indices from a valid range
void generateRandomIndexArray(int * randomArray, int maxElements, int maxVal);

/// \brief      Blurs and enhances a distribution map to make it more displayable
void prepForDisplay(const cv::Mat& distributionMap, cv::Mat& distributionDisplay);

/// \brief      Adds a pointset to a radial distribution array
void addToRadialDistribution(double *radialDistribution, std::vector<cv::Point2f>& cornerSet, cv::Size imSize);

/// \brief      Add a cornerset to the tally matrix
void addToBinMap(cv::Mat& binMap, std::vector<cv::Point2f>& cornerSet, cv::Size imSize);

/// \brief      Calculate the score for a pointset in terms of its contribution to calibration
double obtainSetScore(cv::Mat& distributionMap,
                      cv::Mat& binMap,
                      cv::Mat& gaussianMat,
                      std::vector<cv::Point2f>& cornerSet,
                      double *radialDistribution);

/// \brief 		Verifies that the final patches do actually represent a grid pattern
bool verifyCorners(cv::Size imSize, cv::Size patternSize, vector<cv::Point2f>& patternPoints, double minDist = MIN_CORNER_SEPARATION, double maxDist = MAX_CORNER_SEPARATION);

/// \brief 		Move a vector of points from one vector of vectors to another
void transferMserElement(vector<vector<cv::Point> >& dst, vector<vector<cv::Point> >& src, int index);
/// \brief 		Move an MSER-patch from one vector to another, resizing and shifting the old vector
void transferPatchElement(vector<mserPatch>& dst, vector<mserPatch>& src, int index);

/// \brief      Filters out MSERs that cannot possibly be part of the geometric pattern
void clusterFilter(vector<mserPatch>& patches, vector<vector<cv::Point> >& msers, int totalPatches);
/// \brief      Filters out most deviant patches to retain only the correct number for the pattern
void reduceCluster(vector<mserPatch>& patches, vector<vector<cv::Point> >& msers, int totalPatches);

/// \brief      Filters out MSERs that are enclosed by or enclose true patches
void enclosureFilter(vector<mserPatch>& patches, vector<vector<cv::Point> >& msers);

/// \brief      Filters out MSERs that have a large amount of internal pixel intensity variance
void varianceFilter(vector<mserPatch>& patches, vector<vector<cv::Point> >& msers);

/// \brief      Filters out MSERs that do not have a realistic shape
void shapeFilter(vector<mserPatch>& patches, vector<vector<cv::Point> >& msers);

/// \brief      Measures the coverage of a cameras field of view by analysing the distribution tally
void addToDistributionMap(cv::Mat& distributionMap, vector<cv::Point2f>& corners);

///// \brief      Redistorts points using an established distortion model
//void redistortPoints(const vector<Point2f>& src, vector<Point2f>& dst, const Mat& cameraMatrix, const Mat& distCoeffs, const Mat& newCamMat=Mat::eye(3,3,CV_64FC1));

/// \brief 		Generic grid verification function
bool verifyPattern(cv::Size imSize, cv::Size patternSize, vector<cv::Point2f>& patternPoints, double minDist, double maxDist);

/// \brief      Finds just the MSER centres from an image
bool findPatternCentres(const cv::Mat& image, cv::Size patternSize, vector<cv::Point2f>& centres, int mode);

/// \brief 		Sorts the patches into a standard order to prepare for homography
void sortPatches(cv::Size imageSize, cv::Size patternSize, vector<cv::Point2f>& patchCentres, int mode);

/// \brief 		Re-orders patches into row by row, left to right
void reorderPatches(cv::Size patternSize, int mode, int *XVec, int *YVec, vector<cv::Point2f>& patchCentres);

/// \brief      Estimate the locations of patch corners based on patch centroids
void interpolateCornerLocations2(const cv::Mat& image, int mode, cv::Size patternSize, vector<cv::Point2f>& vCentres, vector<cv::Point2f>& vCorners);

/// \brief      Converts points ordering from row-by-row to quad-clustered
void groupPointsInQuads(cv::Size patternSize, vector<cv::Point2f>& corners);

/// \brief      Refines positions of corners through iterative local homography mappings
void refineCornerPositions(const cv::Mat& image, cv::Size patternSize, vector<cv::Point2f>& vCorners);

/// \brief      Initial attempt to correct locations of all corners based on estimates from MSER centroids
void initialRefinementOfCorners(const cv::Mat& imGrey, vector<cv::Point2f>& src, cv::Size patternSize);

/// \brief 		Sorts the corners into a standard order to prepare for calibration
void sortCorners(cv::Size imageSize, cv::Size patternSize, vector<cv::Point2f>& corners);

/// \brief      Returns the subpixel location of the corner given a decent estimate
int findBestCorners(const cv::Mat& image, vector<cv::Point2f>& src, vector<cv::Point2f>& dst, cv::Size patternSize, int detector, int searchDist = MAX_SEARCH_DIST);

/// \brief      Visually pattern on image
void debugDisplayPattern(const cv::Mat& image, cv::Size patternSize, cv::Mat& corners, bool mode = true, double delay = 0.0);

/// \brief 		Verifies that the final patches do actually represent a grid pattern
bool verifyPatches(cv::Size imSize, cv::Size patternSize, vector<cv::Point2f>& patchCentres, int mode, double minDist, double maxDist);

/// \brief      Uses contrast enhancement etc to improve positional estimates of patch centres
bool correctPatchCentres(const cv::Mat& image, cv::Size patternSize, vector<cv::Point2f>& patchCentres, int mode);

/// \brief 		Estimates the co-ordinates of the corners of the patches
bool findPatchCorners(const cv::Mat& image, cv::Size patternSize, cv::Mat& homography, vector<cv::Point2f>& corners, vector<cv::Point2f>& patchCentres2f, int mode, int detector = 0);

/// \brief 		MSER-clustering mask corner locater
bool findMaskCorners(const cv::Mat& image, cv::Size patternSize, vector<cv::Point2f>& corners, int detector = 0, int mserDelta = 8, double max_var = 0.25, double min_div = 0.2, double area_threshold = 1.01);

/// \brief 		Core pattern-finding function
bool findPatternCorners(const cv::Mat& image, cv::Size patternSize, vector<cv::Point2f>& corners, int mode, int detector = 0, int mserDelta = 8, double max_var = 0.25, double min_div = 0.2, double area_threshold = 1.01);

/// \brief 		Find all patches (MSERS - using default settings) in an image
void findAllPatches(const cv::Mat& image, cv::Size patternSize, vector<vector<cv::Point> >& msers, int mserDelta = 8, float max_var = 0.25, float min_div = 0.2, double area_threshold = 1.01);


/// \brief 		Finds (or simulates) the edge patches in a pattern
void findEdgePatches(cv::Size patternSize, int mode, int *XVec, int *YVec, vector<cv::Point2f>& patchCentres, vector<cv::Point2f>& remainingPatches);

/// \brief 		Finds (or simulates) the interior patches in a pattern
void findInteriorPatches(cv::Size patternSize, int mode, int *XVec, int *YVec, vector<cv::Point2f>& patchCentres, vector<cv::Point2f>& remainingPatches);

/// \brief      Randomly cuts down a list of input files
void randomCulling(vector<std::string>& inputList, int maxSearch);

/// \brief      Culls some patterns from a vector, and from the corresponding names list
void randomCulling(vector<std::string>& inputList, int maxSearch, vector<vector<cv::Point2f> >& patterns);

/// \brief      Culls some sets of patterns from a vector vector, and from the corresponding names list
void randomCulling(vector<std::string>& inputList, int maxSearch, vector<vector<vector<cv::Point2f> > >& patterns);

/// \brief      Checks validity of image for calibration
bool checkAcutance();

/// \brief 		Determines how many patches can be found in each row and extreme columns
void determineFindablePatches(cv::Size patternSize, int mode, int *XVec, int *YVec);

/// \brief 		Finds (or simulates) the four corners patches in a pattern
void findCornerPatches(cv::Size imageSize, cv::Size patternSize, int mode, int *XVec, int *YVec, vector<cv::Point2f>& patchCentres, vector<cv::Point2f>& remainingPatches);

/// \brief      Determines the distribution of patches based on pattern dimensions and pattern type
void determinePatchDistribution(cv::Size patternSize, int mode, int& rows, int& cols, int& quant);

/// \brief      Visually display patches on image
void debugDisplayPatches(const cv::Mat& image, vector<vector<cv::Point> >& msers);

/// \brief 		Applies various area, colour and positional filters to reduce no. of patches to desired amount
bool refinePatches(const cv::Mat& image, cv::Size patternSize, vector<vector<cv::Point> >& msers, vector<cv::Point2f>& patchCentres, int mode);

/// \brief          Checks that the entire pattern is inside the frame by at least a specified margin.
bool patternInFrame(cv::Size imSize, vector<cv::Point2f>& patternPoints, int minBorder = 2);

#endif
