#ifndef __EVALUATION_HPP_H__
#define __EVALUATION_HPP_H__

#include <stdio.h>

/****************************************************************************************\
*                                  Copyright notices                                     *
\****************************************************************************************/

/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                        Intel License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000, Intel Corporation, all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of Intel Corporation may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/

// Some of this code has been sourced from:
// - ../OpenCV/OpenCV-2.3.0/modules/features2d/src/evaluation.cpp
// - ../OpenCV/OpenCV-2.3.0/samples/cpp/detector_descriptor_evaluation.cpp
// Additional code is provided with absolutely no warranty.

/****************************************************************************************\
*                                  Includes                                              *
\****************************************************************************************/

#ifdef _IS_LINUX_

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgproc/imgproc_c.h"
//#include "opencv2/core/internal.hpp"
#include "opencv2/calib3d/calib3d.hpp"
//#include "opencv2/nonfree/nonfree.hpp"

#include <limits>
#include <cstdio>
#include <iostream>
#include <fstream>
#include <cmath>

#ifdef _IS_LINUX_
#include <dirent.h>
#endif

#include <limits>
#include <sys/stat.h>

#include <boost/filesystem.hpp>

#include "core/tools.hpp"
#include "core/features.hpp"

// eccBlock
//#include "eccblock/eccBlock/eccBlock.h"

/****************************************************************************************\
*                                  Namespaces                                            *
\****************************************************************************************/

using namespace std;
using namespace cv;
using namespace boost::filesystem;

/****************************************************************************************\
*                                  Settings                                              *
\****************************************************************************************/

static int globalCounter = 0;

#define ABSOLUTE_MAX_KEYPOINTS_TO_RETAIN 1000
#define EVALUATION_OVERLAP_THRESHOLD 0.2

const string DETECTOR_FOR_PRECISION_RECALL = "SURF";
const string matchesParent = "/home/steve/svidaslib/svm-matching/data";

const int DD_DATASETS_COUNT = 10;
const string DD_DATASET_NAMES[DD_DATASETS_COUNT] = { "aquavist", "balcony", "pipes", "nitrogen", "driveway", "pavement", "office", "library", "desk" }; // "oven",

//const string DD_DATASET_NAMES[DD_DATASETS_COUNT] = { "shed" };

//const string DD_DATASET_NAMES[DD_DATASETS_COUNT] = { "aquaciat" };

//const string DD_DATASET_NAMES[DD_DATASETS_COUNT] = { "building-4", "gazebo", "trees-2", "pavement"}; // "shed",
//const string DD_DATASET_NAMES[DD_DATASETS_COUNT] = { "apartments" , "aquaciat", "building-1", "building-2", "building-3" };
//const string DD_DATASET_NAMES[DD_DATASETS_COUNT] = { "apartments" , "aquaciat", "building-1", "building-2", "building-3", "building-4", "gazebo", "shed", "trees-2", "pavement"};
//const string DD_DATASET_NAMES[DD_DATASETS_COUNT] = { "chairs", "construction", "building-3", "building-4"};
//const string DD_DATASET_NAMES[DD_DATASETS_COUNT] = { "gazebo", "park", "shed", "street-1"};
//const string DD_DATASET_NAMES[DD_DATASETS_COUNT] = { "trees-2"};
// "street-2",

//const string DD_DATASET_NAMES[DD_DATASETS_COUNT] = { "desk"}; // "nitrogen", "desk", "driveway"
// -i /home/steve/features-data -m visible -s profile -b -x
// -i /home/steve/features-data -m visible -s profile -g -c
const int DD_MODALITIES_COUNT = 2;
const string DD_MODALITY_NAMES[DD_MODALITIES_COUNT] = { "visible", "thermal" };

const int DD_DIGITAL_TRANSFORMATIONS_COUNT = 6;
const string DD_DIGITAL_TRANSFORMATIONS[DD_DIGITAL_TRANSFORMATIONS_COUNT] = { "gau", "cmp", "gab", "qnt", "nrm", "sap"};

const int DD_ANALOG_TRANSFORMATIONS_COUNT = 5;
const string DD_ANALOG_TRANSFORMATIONS[DD_ANALOG_TRANSFORMATIONS_COUNT] = { "ofb", "nuc", "vpt", "tod", "raz"};

const int DD_SUBSETS_COUNT = 12;
const string DD_SUBSET_NAMES[DD_SUBSETS_COUNT] = { "profile", "ofb", "nuc", "vpt", "tod", "raz", "gau", "cmp", "sap", "gab", "qnt", "nrm"};

const int DD_TYPES_COUNT = 2;
const string DD_TYPE_NAMES[DD_TYPES_COUNT] = { "same-level", "base-ref" };

const int DD_PROFILE_LEVELS = 29;
const int DD_PROFILE_LIST[DD_PROFILE_LEVELS] = { 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100, 110, 120, 130, 140, 150, 160, 170, 180, 190, 200, 250, 300}; //, 350, 400, 450, 500 };

const int DD_DETECTORS_COUNT = 1; // , "ORB", "SURF", "STAR", "FAST", "SIFT"
const string DD_DETECTOR_NAMES[DD_DETECTORS_COUNT] = { "SURF" };

const int DD_DESCRIPTORS_COUNT = 3; // "SIFTX", "SURF", // GLOHX", "SCX", "BRIEF", "PCAX, MM-SEP-VIS", "MM-SEP-THM",
//const string DD_DESCRIPTORS_NAMES[DD_DESCRIPTORS_COUNT] = { "SURF" }; // "Sc8Tn64", "HAAR-n", "DCT-n", "RAD-n", "NGC"
//const string DD_DESCRIPTORS_NAMES[DD_DESCRIPTORS_COUNT] = { "BRIEF" };
const string DD_DESCRIPTORS_NAMES[DD_DESCRIPTORS_COUNT] = { "SIFTX", "GLOHX", "SCX" };
//const string DD_DESCRIPTORS_NAMES[DD_DESCRIPTORS_COUNT] = { "MM-JNT-ALL", "MM-SEP-VIS", "MM-SEP-THM" };

const double linear_grads[10] = {-10.0, -1.1537, -1.4850, -1.0067, -2.0817, -0.8194, -1.7680, -1.3612, -0.9326, -1.1902 };
const double inverse_grads[10] = { 8.5659, 4.9802, 5.9680, 3.0563, 9.5934, 2.7436, 6.0952, 5.3390, 5.3234, 4.3279 };

const double double_inverse_grads[10] = { -1.508, -0.9029, -2.4547, -0.9156, -1.0, -1.0818, -0.9776, -1.3606, -1.0932, -0.604 };

// For FUSION
// -i /home/steve/features-data -m visible -s profile -c -g -e //  -g -e
// FOR PLAIN
// -i /home/steve/features-data -m visible -s profile -g -e
// -i /home/steve/features-data -m thermal -d balcony -s tod
const int MAX_IMAGES_PER_FOLDER = 5;
const int MAX_TRANSFORM_LEVELS = 20;
const int DEFAULT_MAX_FEATURES = 300;
const int DEFAULT_FEATURE_COUNT = 300;

/****************************************************************************************\
*                                  Global constants                                      *
\****************************************************************************************/
#undef _S
// Match ranking metric
#define DISTANCE_MATCHING           0
#define DISTANCE_RATIO_MATCHING     1
#define SVM_MATCHING                2

// Matching method
#define ONE_WAY_MATCHING            0
#define TWO_WAY_RANK_MATCHING       1   // Not true two-way matching, but uses average rank in both directions...
#define TWO_WAY_PRIORITY_MATCHING   2

#define LINEAR_FUSION               0
#define RECIPROCAL_DIST_FUSION      1

#define MAX_STRING_LENGTH           256

extern char matchesFilename[MAX_STRING_LENGTH];
extern bool printMatches;
const string data_path;

#ifdef _IS_LINUX_
const mode_t DEFAULT_MKDIR_PERMISSIONS = S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH;
#endif

#define MIN_WEIGHTING_CODE          -1.00
#define MAX_WEIGHTING_CODE          -2.00
#define RADIAL_DISTANCE             3.33
#define MULTIPLY_MODALITIES         9.99
#define THERMAL_ONLY                5.00
#define DO_RECIPROCAL_WEIGHTING     1.04
#define DO_LINEAR_WEIGHTING         1.05
#define DOUBLE_RECIPROCAL           1.06
#define LOOKUP_WEIGHTING            1.07
#define DO_DEPENDENT_WEIGHTING      6.66
#define STAGGERED_WEIGHTING         10.00
/****************************************************************************************\
*                                  Class declarations                                 *
\****************************************************************************************/

class EllipticKeyPoint {
public:
    EllipticKeyPoint();
    EllipticKeyPoint( const Point2f& _center, const Scalar& _ellipse );

    static void convert( const vector<KeyPoint>& src, vector<EllipticKeyPoint>& dst );
    static void convert( const vector<EllipticKeyPoint>& src, vector<KeyPoint>& dst );

    static Mat_<double> getSecondMomentsMatrix( const Scalar& _ellipse );
    Mat_<double> getSecondMomentsMatrix() const;

    void calcProjection( const Mat_<double>& H, EllipticKeyPoint& projection ) const;
    static void calcProjection( const vector<EllipticKeyPoint>& src, const Mat_<double>& H, vector<EllipticKeyPoint>& dst );

    Point2f center;
    Scalar ellipse; // 3 elements a, b, c: ax^2+2bxy+cy^2=1
    Size_<float> axes; // half lenght of elipse axes
    Size_<float> boundingBox; // half sizes of bounding box which sides are parallel to the coordinate axes
};

/*
struct IntersectAreaCounter {
    IntersectAreaCounter() : bua(0), bna(0) {}
    IntersectAreaCounter( float _dr, int _minx,
                          int _miny, int _maxy,
                          const Point2f& _diff,
                          const Scalar& _ellipse1, const Scalar& _ellipse2 ) :
               dr(_dr), bua(0), bna(0), minx(_minx), miny(_miny), maxy(_maxy),
               diff(_diff), ellipse1(_ellipse1), ellipse2(_ellipse2) {}
    IntersectAreaCounter( const IntersectAreaCounter& counter, Split )
    {
        *this = counter;
        bua = 0;
        bna = 0;
    }

    void operator()( const BlockedRange& range )
    {
        int temp_bua = bua, temp_bna = bna;
        for( int i = range.begin(); i != range.end(); i++ )
        {
            float rx1 = minx + i*dr;
            float rx2 = rx1 - diff.x;
            for( float ry1 = (float)miny; ry1 <= (float)maxy; ry1 += dr )
            {
                float ry2 = ry1 - diff.y;
                //compute the distance from the ellipse center
                float e1 = (float)(ellipse1[0]*rx1*rx1 + 2*ellipse1[1]*rx1*ry1 + ellipse1[2]*ry1*ry1);
                float e2 = (float)(ellipse2[0]*rx2*rx2 + 2*ellipse2[1]*rx2*ry2 + ellipse2[2]*ry2*ry2);
                //compute the area
                if( e1<1 && e2<1 ) temp_bna++;
                if( e1<1 || e2<1 ) temp_bua++;
            }
        }
        bua = temp_bua;
        bna = temp_bna;
    }

    void join( IntersectAreaCounter& ac )
    {
        bua += ac.bua;
        bna += ac.bna;
    }

    float dr;
    int bua, bna;

    int minx;
    int miny, maxy;

    Point2f diff;
    Scalar ellipse1, ellipse2;

};
*/

struct SIdx {
    SIdx() : S(-1), i1(-1), i2(-1) {}
    SIdx(float _S, int _i1, int _i2) : S(_S), i1(_i1), i2(_i2) {}
    float S;
    int i1;
    int i2;

    bool operator<(const SIdx& v) const { return S > v.S; }

    struct UsedFinder
    {
        UsedFinder(const SIdx& _used) : used(_used) {}
        const SIdx& used;
        bool operator()(const SIdx& v) const { return  (v.i1 == used.i1 || v.i2 == used.i2); }
        UsedFinder& operator=(const UsedFinder&);
    };
};

struct DMatchForEvaluation : public DMatch {
    uchar isCorrect;
    DMatchForEvaluation( const DMatch &dm ) : DMatch( dm ) {}
};

class BaseQualityEvaluator {
public:
    BaseQualityEvaluator( const char* _algName, const char* _testName ) : algName(_algName), testName(_testName)
    {
        //TODO: change this
        isWriteGraphicsData = true;
    }

    void run();

protected:
    virtual string getRunParamsFilename() const = 0;
    virtual string getResultsFilename() const = 0;
    virtual string getPlotPath() const = 0;

    virtual void calcQualityClear( int datasetIdx ) = 0;
    virtual bool isCalcQualityEmpty( int datasetIdx ) const = 0;

    void readAllDatasetsRunParams();
    virtual void readDatasetRunParams( FileNode& fn, int datasetIdx ) = 0;
    void writeAllDatasetsRunParams() const;
    virtual void writeDatasetRunParams( FileStorage& fs, int datasetIdx ) const = 0;
    void setDefaultAllDatasetsRunParams();
    virtual void setDefaultDatasetRunParams( int datasetIdx ) = 0;
    virtual void readDefaultRunParams( FileNode& /*fn*/ ) {}
    virtual void writeDefaultRunParams( FileStorage& /*fs*/ ) const {}

    bool readDataset( const string& datasetName, vector<Mat>& Hs, vector<Mat>& imgs );

    virtual void readAlgorithm() {}
    virtual void processRunParamsFile() {}
    virtual void runDatasetTest( const vector<Mat>& /*imgs*/, const vector<Mat>& /*Hs*/, int /*di*/, int& /*progress*/ ) {}

    virtual void processResults( int datasetIdx );
    virtual void processResults();
    virtual void writePlotData( int /*datasetIdx*/ ) const {}

    string algName, testName;
    bool isWriteParams, isWriteGraphicsData;
};

class sortKeypointsByResponse { // http://stackoverflow.com/questions/4066576/passing-a-parameter-to-a-comparison-function
public:
    sortKeypointsByResponse() {};

    bool operator()(const cv::KeyPoint &a, cv::KeyPoint &b) {
        if ( a.response > b.response ) {
            return true;
        } else {
            return false;
        }
    }
};

class configData {
public:
    char *dataPath;

    bool singleDatasetOnly, singleModalityOnly, singleSubsetOnly;
    char *datasetString, *modalityString, *subsetString;

    bool regenerateHomographies, testAcrossModalities, displayMode, writeMode;
    bool regenerateDetectorResults, regenerateDescriptorResults, regenerateFeatures, regenerateDescriptors;
    bool regenerateAllResults;

    bool singleTypeOnly;
    int typeCode;

    bool fusionMode;

    configData();

    bool processArguments(int argc, char** argv);
    void printParameters();

};

class fileTree {
public:
    vector<string> datasets, modalities, subsets;

    void prepareTree(configData& iD);
    void displayTree();
    void makeDirectories(const configData& iD, unsigned int level, unsigned int idx_0, unsigned int idx_1 = 0, unsigned int idx_2 = 0);
};

class addressNode {
public:
    char imagePath[MAX_STRING_LENGTH];
    char imageAddress[MAX_STRING_LENGTH];
    char homogPath[MAX_STRING_LENGTH];
    char homogAddress[MAX_STRING_LENGTH];
    char resultsPath[MAX_STRING_LENGTH];
    char resultsAddress[MAX_STRING_LENGTH];
    char compResultsAddress[MAX_STRING_LENGTH];
    char featuresAddress[MAX_STRING_LENGTH];
    char featuresPath[MAX_STRING_LENGTH];
    char matchResultsPath[MAX_STRING_LENGTH];
    char matchResultsAddress[MAX_STRING_LENGTH];
    char newDirectoryPath[MAX_STRING_LENGTH];
    char descriptorsPath[MAX_STRING_LENGTH];
    char descResultsPath[MAX_STRING_LENGTH];
    char descriptorsAddress[MAX_STRING_LENGTH];
    char compdescriptorsAddress[MAX_STRING_LENGTH];
    char compImagePath[MAX_STRING_LENGTH];
    char compImageName[MAX_STRING_LENGTH];
    char fusionHomogPath[MAX_STRING_LENGTH];
    char fusionHomogAddress[MAX_STRING_LENGTH];
    char baseFeaturesPath[MAX_STRING_LENGTH];

    void processImage(string imageName, vector<Mat>& rawImageVec, vector<Mat>& grayImageVec, vector<Mat>& colImageVec);

};

class homographyHandler {
public:
    Ptr<FeatureDetector> homographyDetector;

    vector<KeyPoint> levelHomographyKeypoints[MAX_TRANSFORM_LEVELS];
    vector<KeyPoint> homographyKeypoints[MAX_IMAGES_PER_FOLDER];
    Ptr<DescriptorExtractor> homographyExtractor;
    Ptr<DescriptorMatcher> descriptorMatcher;
    double ransacReprojThreshold;
    vector<Mat> homographyDescriptors;

    homographyHandler();
};

class eccBlockWrapper {
public:
    // eccBlock variables
    int          number_of_iterations;
    double       termination_eps;
    const char * motion;
    const char * warp_init ;
    const char * warp_to_file;
    const char * image_to_file;
    bool         verbose;
    IplImage   * target_image;
    IplImage   * template_image;
    CvMat      * warp_matrix;
    //WARP_MODE warp_mode;

    eccBlockWrapper();

};

/****************************************************************************************\
*                                  Function declarations                                 *
\****************************************************************************************/
void initializeKeypointsTripleVec(vector< vector< vector<KeyPoint> > > *kpVec);

static inline Point2f applyHomography( const Mat_<double>& H, const Point2f& pt );

static inline void linearizeHomographyAt( const Mat_<double>& H, const Point2f& pt, Mat_<double>& A );

void calculateRepeatability( const Mat& img1, const Mat& img2, const Mat& H1to2,
                                    const vector<KeyPoint>& _keypoints1, const vector<KeyPoint>& _keypoints2,
                                    float& repeatability, int& correspondencesCount,
                                    float overlapThreshold,
                                    Mat* thresholdedOverlapMask=0  );

void smoothAndSubsamplePrecisionRecallPoints(vector<Point2f>& pts, double resolution = 0.001);

void calculateMatchability( const Mat& img1, const Mat& img2, const Mat& H1to2,
                                    const vector<KeyPoint>& _keypoints1, const vector<KeyPoint>& _keypoints2,
                                    float& matchability, int& correspondencesCount,
                                    float overlapThreshold,
                                    Mat* thresholdedOverlapMask=0  );

static void filterEllipticKeyPointsByImageSize( vector<EllipticKeyPoint>& keypoints, const Size& imgSize );

static void computeOneToOneMatchedOverlaps( const vector<EllipticKeyPoint>& keypoints1, const vector<EllipticKeyPoint>& keypoints2t,
                                            bool commonPart, vector<SIdx>& overlaps, float minOverlap );

void cv::evaluateFeatureDetector( const Mat& img1, const Mat& img2, const Mat& H1to2,
                              vector<KeyPoint>* _keypoints1, vector<KeyPoint>* _keypoints2,
                              float& repeatability, int& correspCount,
                              const Ptr<FeatureDetector>& _fdetector );

static inline float recall( int correctMatchCount, int correspondenceCount );

static inline float precision( int correctMatchCount, int falseMatchCount );

double calculatePrecisionRecallScore(vector<Point2f>& recallPrecisionCurve);

void computeRecallPrecisionCurveModified( const vector<vector<DMatch> >& matches1to2,
                                      const vector<vector<uchar> >& correctMatches1to2Mask,
                                      const vector<vector<DMatch> >& matches2to1,
                                      const vector<vector<uchar> >& correctMatches2to1Mask,
                                      vector<Point2f>& recallPrecisionCurve,
                                      int correspCount );

float cv::getRecall( const vector<Point2f>& recallPrecisionCurve, float l_precision );

int cv::getNearestPoint( const vector<Point2f>& recallPrecisionCurve, float l_precision );

void evaluateGenericDescriptorMatcherModified( const Mat& img1, const Mat& img2, const Mat& H1to2,
                                           vector<KeyPoint>& keypoints1, vector<KeyPoint>& keypoints2,
                                           vector<vector<DMatch> >* _matches1to2, vector<vector<uchar> >* _correctMatches1to2Mask,
                                           vector<vector<DMatch> >* _matches2to1, vector<vector<uchar> >* _correctMatches2to1Mask,
                                           vector<Point2f>& recallPrecisionCurve,
#ifdef _OPENCV_VERSION_3_PLUS_
                                           const Ptr<DescriptorMatcher>& _dmatcher );
#else
                                           const Ptr<GenericDescriptorMatcher>& _dmatcher );
#endif

/// Brief Function to evaluate affine covariant detectors and descriptors
static void calcKeyPointProjections( const vector<KeyPoint>& src, const Mat_<double>& H, vector<KeyPoint>& dst );

static void filterKeyPointsByImageSize( vector<KeyPoint>& keypoints, const Size& imgSize );

inline void writeKeypoints( FileStorage& fs, const vector<KeyPoint>& keypoints, int imgIdx );

void writeKeypoints_MATLAB( char *filename, const vector<KeyPoint>& keypoints );

void readKeypoints_MATLAB( char *filename, vector<KeyPoint>& keypoints );

void writeDescriptors_MATLAB( char *filename, const vector<KeyPoint>& keypoints, const Mat& descriptors );

void readDescriptors_MATLAB( char *filename, vector<KeyPoint>& keypoints, Mat& descriptors, unsigned int maxFeatures );

inline void readKeypoints( FileStorage& fs, vector<KeyPoint>& keypoints, int imgIdx );

void convertBackToMatchVec(Mat& matchingMat, vector<vector<DMatch> >& matches);

int update_progress( const string& /*name*/, int progress, int test_case_idx, int count, double dt );

/// Brief Just one-way matching
void createMatchingMatrix(Mat& matchingMatrix, const vector<vector<DMatch> >& matches);

void createMatchingMatrix(Mat& matchingMatrix, const vector<vector<DMatch> >& matches1to2, const vector<vector<DMatch> >& matches2to1);

void getPriorityScores(const Mat& matchingMatrix, vector<vector<double> >& priorityScores, vector<vector<int> >& priorityMatches);

void getOneWayPriorityScores(Mat& matchingMatrix, vector<vector<double> >& priorityScores, vector<vector<int> >& priorityMatches);

void testLog( bool isBadAccuracy );

int countAndSortFiles(char *input, vector<string> &inputList);

void initializeDetectors(vector< Ptr<FeatureDetector> > &allFeatureDetectors);

void initializeDescriptors(vector< Ptr<DescriptorExtractor> > &allFeatureDescriptors);

//void sortKeypoints(vector<KeyPoint>& keypoints, unsigned int maxKeypoints = ABSOLUTE_MAX_KEYPOINTS_TO_RETAIN);

void obtainSubset(vector<KeyPoint>& src, vector<KeyPoint>& dst, double threshold = 0.0);

bool compareMatrices(Mat& mat1, Mat& mat2);

void trainedFASTDetection(Mat& image, vector<KeyPoint> &keypoints, int maxFeatures);

void randomDetection(Mat& image, vector<KeyPoint> &keypoints, int maxFeatures, bool randomScales = false);

void applyRandomDescription(vector<KeyPoint> &pts, Mat& descriptors);

void calculatePrecisionRecall( Mat& im1, Mat& im2, Mat& homography, vector<KeyPoint>& pts1, vector<KeyPoint>& pts2, Mat& desc1, Mat& desc2, vector<Point2f>& results, int correspCount, float overlapThreshold);

double calculateEntropy(const Mat& im1, const Mat& im2, const vector<KeyPoint>& pts1, const vector<KeyPoint>& pts2);

void combineMatches(vector<vector<DMatch> >& matches1, vector<vector<DMatch> >& matches2, vector<vector<DMatch> >& combinedMatches, double weighting, double entropy = 0.00);

void calculateFusionPR( Mat& im1, Mat& im2, Mat& im1b, Mat& im2b, Mat& homography, vector<KeyPoint>& pts1, vector<KeyPoint>& pts2, Mat& desc1, Mat& desc2, Mat& desc1b, Mat& desc2b, vector<Point2f>& results, int correspCount, float overlapThreshold, double weighting, double entropy = 0.00);

void writeDescriptorResults(char* fileAddress, vector<Point2f>& curve);

bool sort_point2f_vector_by_x_val (const Point2f& pt1, const Point2f& pt2);

bool sort_point2f_vector_by_y_val (const Point2f& pt1, const Point2f& pt2);

//double reweightDistanceWithLinearSVM(double dist, double ratio, double gradient, double *shift);

//double calcDistance(double dist, double ratio, double *coeffs);

//double calcLinePerpDistance(double *line1, double *line2);

void sortMatches(const vector<vector<DMatch> >& matches, vector<vector<double> >& scores, vector<vector<unsigned int> >& indices);

double applyPolynomialCoefficients(double val, double* polyCoeffs, int polyOrder);

int determineCorrectMatches(Mat& flagMat, const vector<vector<DMatch> >& matches1to2, const vector<vector<uchar> >& correctMatches1to2Mask, const vector<vector<DMatch> >& matches2to1, const vector<vector<uchar> >& correctMatches2to1Mask);

void summarizeScores(vector<float> &scores, float *summ);

void initializeSubsetStrings();

void prepVector(vector<string>& stringVector, char* inputString, bool singleOnly, const string *nameStrings, const int stringCount);

void listTestSets(string debugString, vector<string> setList);

void makeAnnotationPair(Mat& annotationPair, Mat im1, Mat im2);

void assignChronologicalResponses(vector<KeyPoint>& pts);

void assignMinimumRadii(vector<KeyPoint>& pts);

//double estimateSalience(Mat& img, Point2f& center, double radius);
//double estimateStability(Mat& img, Point2f& center, double radius);
//bool constructPatch(Mat& img, Mat& patch, Point2f& center, double radius, int cells = 3);
//double getInterpolatedVal(Mat& img, Point2f& coord);
//double getValueFromPatch(Mat& patch);
//double getPatchVariance(const Mat& patch) ;
void assignEigenVectorResponses(const Mat& img, vector<KeyPoint>& pts);
void assignStabilityResponses(const Mat& img, vector<KeyPoint>& pts);

void combineRecallVectors(vector<vector<Point2f> >& totalPrecRecall, vector<Point2f>& finalRecall);

bool retainMostAppropriateHomography(Mat& originalImage, Mat& targetImage, Mat& originalCandidate, Mat newCandidate);

static void usage(const char *argv0);

void assembleSubDirectories(char *imageParentPath, vector<string>& subDirectories);

void analyzeMatches(Mat& visMat, Mat& thermMat, Mat& mask);

void displayMatchesNeatly(Mat& im1, Mat& im2, vector<KeyPoint>& pts1, vector<KeyPoint>& pts2, vector<DMatch>& matches, Mat& dst);
#endif

#endif
