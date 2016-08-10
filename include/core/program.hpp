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

#ifdef _IS_LINUX_
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#endif

#define MAX_INPUT_ARG_LENGTH    256
#define MAX_PATH_SIZE           512

class GenericOptions {

protected:
	int FrameCounter1, LastFrameCounter, FC0, FC1;
	bool pauseMode, isValid, wantsToShutdown;
	bool debugMode, wantsToOutput, writeInColor;
	cv::Mat *displayImage;
	char *output_directory;
	char nodeName[256];
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

/**
 * A structure to represent data common to all nodes
 */
struct commonData {
    cameraParameters cameraData; /**< the camera parameters corresponding to the video stream */
    string topic, topicParent, read_addr;
    string outputFolder; /**< directory where output information will be written */
	bool *wantsToTerminate;
	

	commonData() : outputFolder("outputFolder"), read_addr("./") { };
	void setTerminationTrigger(bool* trigger) { wantsToTerminate = trigger; }
};

#endif
