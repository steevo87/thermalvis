/*! \file	program.cpp
 *  \brief	Definitions for generic programs.
*/

#include "program.hpp"


GenericOptions::GenericOptions() : pauseMode(false), isValid(true), wantsToOutput(false), FrameCounter1(0), writeInColor(true) {
	output_directory = new char[MAX_INPUT_ARG_LENGTH];
	displayImage = new cv::Mat();
}

GenericOptions::~GenericOptions() {
	timestamps_stream.close();
}

void GenericOptions::displayFrame() {
	if (displayImage->rows != 0) {
		!pauseMode ? cv::imshow("display", *displayImage) : 0;
		char key = cv::waitKey(1);
		if (key == 'q') isValid = false;
	}
}

bool GenericOptions::wantsToRun() {
	return isValid;
}

bool GenericOptions::setOutputDir(char* output_dir) {
	printf("%s << User has opted to output images to the following directory: <%s>\n", __FUNCTION__, output_dir);
	wantsToOutput = true;
	sprintf(output_directory, "%s", output_dir);
	return true;
}

bool GenericOptions::initializeOutput(char *output_dir) {
	if (output_dir != NULL) {
		printf("%s << Using data output directory of <%s>.\n", __FUNCTION__, output_dir);
		wantsToOutput = true;
		sprintf(output_directory, "%s", output_dir);
		char timestamps_file[256];
		sprintf(timestamps_file, "%s-timestamps.txt", output_dir);
		timestamps_stream.open(timestamps_file);
		return true;
	}
	return false;
}

void GenericOptions::setWriteMode(bool mode) {
	writeInColor = mode;
}

bool GenericOptions::writeImageToDisk() {
	if (wantsToOutput) {
		char imFilename[256];
		sprintf(imFilename, "%s/frame%06d.png", output_directory, FrameCounter1);
		std::string imageFilename(imFilename);
		
		if (writeInColor) {
			cv::imwrite(imageFilename, *displayImage);
		} else {
			ROS_WARN("Cannot write in raw mode, because this class does not have access to that data!");
			return false;
		}
			
		return true;
	}
	return false;
}