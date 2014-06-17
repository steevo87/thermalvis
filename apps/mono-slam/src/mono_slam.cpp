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

	cameraInfoStruct camInfo;

	trackerData startupData;
	flowConfig fcData;
	
	fcData.debugMode = true;
	fcData.showTrackHistory = true;
		
	featureTrackerNode *fM;

	bool configurationDataProvided = false;

	if (configurationDataProvided) {
		fM = new featureTrackerNode(startupData);
		fM->initializeOutput(argc, argv);
		fM->setWriteMode(!(argc >= 4));
	}
	
	
	cv::Mat workingFrame;

	while (dM.wantsToRun()) {
		dM.grabFrame();
		dM.processFrame();
		dM.accessLatest8bitFrame(workingFrame);
		
		if (!configurationDataProvided) {
			startupData.cameraData.cameraSize.width = workingFrame.cols;
			startupData.cameraData.cameraSize.height = workingFrame.rows;
			startupData.cameraData.imageSize.at<unsigned short>(0, 0) = startupData.cameraData.cameraSize.width;
			startupData.cameraData.imageSize.at<unsigned short>(0, 1) = startupData.cameraData.cameraSize.height;
			startupData.cameraData.updateCameraParameters();
			configurationDataProvided = true;
			fM = new featureTrackerNode(startupData);
			fM->initializeOutput(argc, argv);
			fM->setWriteMode(!(argc >= 4));
		}
		fM->serverCallback(fcData);
		fM->handle_camera(workingFrame, &camInfo);
		fM->features_loop();

	}
	
	return S_OK;
}
