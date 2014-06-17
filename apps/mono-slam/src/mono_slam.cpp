/*! \file	mono_slam.cpp
 *  \brief	Monocular SLAM demonstration app
*/

#include "directory_stream.hpp"
#include "sparse_flow.hpp"

int main(int argc, char* argv[]) {
	
	displayMessage("Launching Monocular SLAM Demo App!", MESSAGE_NORMAL, __FUNCTION__);

	directoryManager dM;

	if (!dM.initializeInput(argc, argv)) return -1;
	dM.initialize();

	flowManager fM;
	fM.initializeOutput(argc, argv);
	fM.setWriteMode(!(argc >= 4));
	
	cv::Mat workingFrame;

	while (dM.wantsToRun()) {
		dM.grabFrame();
		dM.processFrame();
		dM.accessLatestFrame(workingFrame);
		fM.applyToFrame(workingFrame);
		fM.displayFrame();
	}
	
	return S_OK;
}
