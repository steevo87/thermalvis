/*! \file	mono_slam.cpp
 *  \brief	Monocular SLAM demonstration app
*/

#include "launch.hpp"
#include "directory_stream.hpp"
#include "sparse_flow.hpp"

#define DEFAULT_LAUNCH_XML "Documents/GitHub/thermalvis/launch/windows_test.launch"

int main(int argc, char* argv[]) {
	
	ROS_INFO("Launching Monocular SLAM Demo App!");

	char xmlAddress[256];
	
	if (argc > 1) {
		sprintf(xmlAddress, "%s", argv[1]);
		ROS_INFO("Using XML file provided at (%s)", xmlAddress);
	} else {
#ifdef _WIN32
		sprintf(xmlAddress, "%s/%s", std::getenv("USERPROFILE"), DEFAULT_LAUNCH_XML);
#else
		sprintf(xmlAddress, "~/%s", DEFAULT_LAUNCH_XML);
#endif
		ROS_INFO("No XML config file provided, therefore using default at (%s)", xmlAddress);
	}

	xmlParameters xP;
	
	xP.parseInputXML(xmlAddress);
	ROS_INFO("About to print XML summary..");
	xP.printInputSummary();

	// Preliminary settings
	streamerData streamerStartupData;
	{
		if (!streamerStartupData.assignFromXml(xP)) return -1;
		streamerStartupData.outputForAnalysis = true;

		ROS_INFO("streamerStartupData.debugMode = (%d)", streamerStartupData.debugMode);
	}

	// Real-time changeable variables
	streamerConfig scData;
	{
		scData.assignStartingData(streamerStartupData);
	}

	streamerNode *sM;

	sM = new streamerNode(streamerStartupData);
	sM->initializeOutput(argc, argv);
	sM->setupDevice();
	
	cameraInfoStruct camInfo;

	// Preliminary settings
	trackerData trackerStartupData;
	{
		if (!trackerStartupData.assignFromXml(xP)) return -1;
		trackerStartupData.outputForAnalysis = true;

		ROS_INFO("trackerStartupData.debugMode = (%d)", trackerStartupData.debugMode);
	}

	// Real-time changeable variables
	flowConfig fcData;
	{
		fcData.assignStartingData(trackerStartupData);
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
		fM = new featureTrackerNode(trackerStartupData);
		fM->initializeOutput(argc, argv);
		fM->setWriteMode(!(argc >= 4));
	}
	
	cv::Mat workingFrame;

	while (sM->wantsToRun()) {
		
		sM->serverCallback(scData);
		sM->streamCallback();
		sM->processImage();
		sM->handle_camera(workingFrame, &camInfo); // Want to retrieve frame
		
		//dM.grabFrame();
		//dM.processFrame();
		//dM.accessLatest8bitFrame(workingFrame);
		
		if (!configurationDataProvided) {
			trackerStartupData.cameraData.cameraSize.width = workingFrame.cols;
			trackerStartupData.cameraData.cameraSize.height = workingFrame.rows;
			trackerStartupData.cameraData.imageSize.at<unsigned short>(0, 0) = trackerStartupData.cameraData.cameraSize.width;
			trackerStartupData.cameraData.imageSize.at<unsigned short>(0, 1) = trackerStartupData.cameraData.cameraSize.height;
			trackerStartupData.cameraData.updateCameraParameters();
			configurationDataProvided = true;
			fM = new featureTrackerNode(trackerStartupData);
			fM->initializeOutput(argc, argv);
			fM->setWriteMode(!(argc >= 4));
		}
		fM->serverCallback(fcData);
		fM->handle_camera(workingFrame, &camInfo);
		fM->features_loop();
	}
	
	return S_OK;
}
