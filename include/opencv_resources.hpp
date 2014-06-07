/*! \file	opencv_resources.hpp
 *  \brief	For OpenCV inclusion requirements.
*/

#ifndef _THERMALVIS_OPENCV_RESOURCES_H_
#define _THERMALVIS_OPENCV_RESOURCES_H_

#include "general_resources.hpp"

// OpenCV
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/video/video.hpp"

#ifdef _USE_OPENCV_GPU_
#include "opencv2/gpu/gpu.hpp"
#endif

// #include <cv_bridge/CvBridge.h>

//using namespace cv;

#endif
