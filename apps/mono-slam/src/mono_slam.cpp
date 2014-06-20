/*! \file	mono_slam.cpp
 *  \brief	Monocular SLAM demonstration app
*/

#include "launch.hpp"
#include "directory_stream.hpp"
#include "sparse_flow.hpp"

int main(int argc, char* argv[]) {
	
	ROS_INFO("Launching Monocular SLAM Demo App!");

	xmlParameters xP;
	bool inputXmlSupplied = false;
	if (argc > 1) {
		xP.parseInputXML(argv[1]);
		ROS_INFO("About to print XML summary..");
		xP.printInputSummary();
		inputXmlSupplied = true;
	}

	directoryManager dM;

	if (!dM.initializeInput(!inputXmlSupplied ? argc : 1, argv)) return -1;
	dM.initialize();
	dM.setLoopMode(true);

	cameraInfoStruct camInfo;

	// Preliminary settings
	trackerData startupData;
	{
		if (!startupData.assignFromXml(xP)) return -1;
		startupData.outputForAnalysis = true;

		ROS_INFO("startupData.debugMode = (%d)", startupData.debugMode);
	}

	// Real-time changeable variables
	flowConfig fcData;
	{
		fcData.assignStartingData(startupData);
		//fcData.setDetector1(DETECTOR_FAST);
	}

	#ifdef _DEBUG
	if (
		((fcData.getDetector1() != DETECTOR_FAST) && (fcData.getDetector1() != DETECTOR_OFF)) || 
		((fcData.getDetector2() != DETECTOR_FAST) && (fcData.getDetector2() != DETECTOR_OFF)) || 
		((fcData.getDetector3() != DETECTOR_FAST) && (fcData.getDetector3() != DETECTOR_OFF))
	) {
		ROS_WARN("The GFTT/HARRIS detector is EXTREMELY slow in the Debug build configuration, so consider switching to an alternative while you are debugging.");
	}
	#endif

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
