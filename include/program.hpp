/*! \file	program.hpp
 *  \brief	Declarations for generic programs.
*/

#ifndef _THERMALVIS_PROGRAM_H_
#define _THERMALVIS_PROGRAM_H_

#include "opencv2/opencv.hpp"

#ifdef _USE_BOOST_
#include "boost/date_time/posix_time/posix_time.hpp"
#endif

#include "tools.hpp"
#include "camera.hpp"

#define MAX_INPUT_ARG_LENGTH 256

class GenericOptions {

protected:
	int FrameCounter1, LastFrameCounter, FC0, FC1;
	bool pauseMode, isValid;
	bool debugMode, wantsToOutput, writeInColor;
	cv::Mat *displayImage;
	char *output_directory;
	std::ofstream timestamps_stream;

public:
	GenericOptions();
	~GenericOptions();
	bool wantsToRun();
	bool setOutputDir(char* output_dir);
	bool initializeOutput(char *output_dir);
	virtual void setDebugMode(bool val) { debugMode = val; }
	void setWriteMode(bool mode);
	virtual bool writeImageToDisk();
	virtual void displayCurrentFrame();
	void updateFrameCounter(int newCount) { FrameCounter1 = newCount; }
};

struct commonData {
	cameraParameters cameraData;
	string topic, topicParent, read_addr, outputFolder;
	bool *wantsToTerminate;

	commonData() : outputFolder("outputFolder"), read_addr("./") { };
	void setTerminationTrigger(bool* trigger) { wantsToTerminate = trigger; }
};

#endif
