/*! \file	input_stream.hpp
 *  \brief	Declarations for generic video input.
*/

#ifndef _THERMALVIS_INPUT_STREAM_H_
#define _THERMALVIS_INPUT_STREAM_H_

#include "program.hpp"
#include "improc.hpp"

#define DEFAULT_COLORSCHEME_INDEX	3
#define DEFAULT_COLORSCHEME_CODE	CIELUV

#define DEFAULT_MIN_TEMP			25.0
#define DEFAULT_MAX_TEMP			35.0

class inputStream : public GenericOptions {

protected:
	
	int colormap_index;

	bool autoscaleTemps;

	double minTemp, maxTemp;

	

	short FrameWidth, FrameHeight, FrameDepth;
	double FrameRatio;
	int FrameSize;

	cv::Mat *rawImage, *scaledImage, *_8bitImage;
	cScheme *cMapping;
	
public:
	inputStream();
	bool processFrame();
	void colorizeFrame();
	void displayFrame();
	bool accessLatestFrame(cv::Mat& latestFrame);
	virtual bool writeImageToDisk();
	
};

#endif
