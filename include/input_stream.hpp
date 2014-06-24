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
	void colorizeFrame() { cMapping->falsify_image(*_8bitImage, *displayImage, 0); }		// THIS IS SLOWING THINGS DOWN!!
	void displayFrame();
	bool accessLatestRawFrame(cv::Mat& latestFrame);
	bool accessLatest8bitFrame(cv::Mat& latestFrame);
	virtual bool writeImageToDisk();

	void load_standard(int mapCode, int mapParam = 0) { cMapping->load_standard(mapCode, mapParam); }
	int get_colormap_index() { return colormap_index; }
	void set_colormap_index(int index) { colormap_index = index; }
	void set_autoscaleTemps(bool setting) { autoscaleTemps = setting; }

	void assignMemoryToRawImage(int height, int width) { rawImage = new cv::Mat(height, width, CV_16UC1); }
	void assignDataToRawImage(uchar *buff) { rawImage->data = buff; }
	bool getPauseMode() { return pauseMode; }
	void switchPauseMode() { pauseMode = !pauseMode; }

	void setMinTemp(double temperature) { minTemp = temperature; }
	void setMaxTemp(double temperature) { maxTemp = temperature; }
	
};

#endif
