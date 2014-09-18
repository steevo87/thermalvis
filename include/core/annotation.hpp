/*! \file	annotation.hpp
 *  \brief	Declarations for drawing on images.
*/

#ifndef THERMALVIS_ANNOTATION_H
#define THERMALVIS_ANNOTATION_H

#include "opencv2/opencv.hpp"

#include "core/improc.hpp"

/// \brief		Draws KeyPoints to scale with coloring proportional to feature strength
void drawRichKeyPoints(const cv::Mat& src, std::vector<cv::KeyPoint>& kpts, cv::Mat& dst);

/// \brief		Draw a nice circle
void draw_circle(cv::Mat& img, cv::Point center, int radius, const cv::Scalar& color, int thickness, int shift);

/// \brief		Display matches between two images
void showMatches(const cv::Mat& pim1, std::vector<cv::Point2f>& pts1, const cv::Mat& pim2, std::vector<cv::Point2f>& pts2, cv::Mat& drawImg);

#endif // THERMALVIS_ANNOTATION_H
