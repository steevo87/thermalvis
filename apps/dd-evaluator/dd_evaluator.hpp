#ifndef __DD_EVALUATOR_H__
#define __DD_EVALUATOR_H__

/****************************************************************************************
                                  Example arguments
****************************************************************************************/

// Program arguments:
// -i /home/steve/OpenCV/opencv_extras/testdata/cv -d aquavist -m visible -r

// Example for x-modality:
// -i /home/steve/OpenCV/opencv_extras/testdata/cv -d aquavist  -s profile -x

// Another example:
// -i /home/steve/features-data -m thermal -s profile -g -e

/****************************************************************************************
                                  Includes
****************************************************************************************/
// OpenCV
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp" // #include "opencv/highgui.h"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
//#include "opencv2/nonfree/nonfree.hpp"

// Generic
#include <limits>
#include <cstdio>
#include <iostream>
#include <fstream>

#ifdef _IS_LINUX_
#include <dirent.h>

#endif

#include <limits>
#include <sys/stat.h>
#include <boost/filesystem.hpp>

// Intra-project
#include "evaluation.hpp"

#include "core/improc.hpp"
#include "core/features.hpp"

/****************************************************************************************
                                  Namespaces
****************************************************************************************/
using namespace std;
using namespace cv;
using namespace boost::filesystem;

/****************************************************************************************
                                  Constants
****************************************************************************************/
#define MAX_IMAGES_FOR_SINGLE_DESCRIPTOR_TEST   5
#define MAX_KEYPOINTS_TO_RETAIN                 300
#define MAX_POINTS_FOR_DESCRIPTORS              300
#define WEIGHTING                               1.07 // 1.00

/****************************************************************************************
                                  Globals
****************************************************************************************/
vector<Point2f> leftAnnotationPoints, rightAnnotationPoints;
Mat testImage_1, testImage_2;
bool globswitch = true;
int imageWidth = 640;
int imageHeight = 480;
Mat annotationImage;

/****************************************************************************************
                                  Function declarations
****************************************************************************************/
void onMouse( int event, int x, int y, int, void* );

#endif
