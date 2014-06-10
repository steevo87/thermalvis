#ifndef __DIR_READER_H__
#define __DIR_READER_H__

//#include <iostream>

#include "improc.hpp"

#include <boost/algorithm/string/replace.hpp>
#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

#define DEFAULT_COLORSCHEME_INDEX	3
#define DEFAULT_COLORSCHEME_CODE	CIELUV

#define DEFAULT_MIN_TEMP			25.0
#define DEFAULT_MAX_TEMP			35.0



class directoryManager {

protected:
	bool wantsToOutput;
	bool isValid;
	char *input_directory, *output_directory;

	int colormap_index;

	vector<std::string> file_list;

	bool pauseMode, autoscaleTemps;

	double minTemp, maxTemp;

	int FrameCounter0, FrameCounter1, LastFrameCounter, FC0, FC1;

	short FrameWidth, FrameHeight, FrameDepth;
	double FrameRatio;
	int FrameSize;

	cv::Mat rawImage, scaledImage, _8bitImage, colorImage;
	cScheme cMapping;
	
public:
	directoryManager();
	bool initialize(char* input_dir);
	bool setOutputDir(char* output_dir);
	bool grabFrame();
	bool wantsToRun();
};

#endif // __DIR_READER_H__