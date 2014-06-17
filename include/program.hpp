/*! \file	program.hpp
 *  \brief	Declarations for generic programs.
*/

#ifndef _THERMALVIS_PROGRAM_H_
#define _THERMALVIS_PROGRAM_H_

#include "tools.hpp"
#include "opencv2/opencv.hpp"

#define MAX_INPUT_ARG_LENGTH 256

class GenericOptions {

protected:
	int FrameCounter1, LastFrameCounter, FC0, FC1;
	bool pauseMode;
	bool isValid;
	bool wantsToOutput;
	bool writeInColor;
	cv::Mat *displayImage;
	char *output_directory;
	bool debugMode;

public:
	GenericOptions();
	bool wantsToRun();
	bool setOutputDir(char* output_dir);
	bool initializeOutput(int argc, char* argv[]);
	virtual void setDebugMode(bool val) { debugMode = val; }
	void setWriteMode(bool mode);
	bool writeImageToDisk();
	virtual void displayFrame();
	void updateFrameCounter(int newCount) { FrameCounter1 = newCount; }
};

#endif