/*! \file	mono_slam.cpp
 *  \brief	Monocular SLAM demonstration app
*/

#include "launch.hpp"
#include "directory_stream.hpp"
#include "sparse_flow.hpp"

#define DEFAULT_LAUNCH_XML "Documents/GitHub/thermalvis/launch/windows_test.launch"

#ifdef _USE_QT_
#include "mainwindow_streamer.h"
#include <QApplication>

#include <mutex>
#include <thread>

QApplication *a;;
void qt_streamer_thread();
std::mutex qt_streamer_mutex;
#endif

int main(int argc, char* argv[]) {

#ifdef _USE_QT_
	a = new QApplication(argc, argv);
	MainWindow_streamer w;
    w.show();
#endif

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

	// === STREAMER NODE === //
	// Preliminary settings
	streamerData streamerStartupData;
	if (!streamerStartupData.assignFromXml(xP)) return -1;

	// Real-time changeable variables
	streamerConfig scData;
	if (!scData.assignStartingData(streamerStartupData)) return -1;

	streamerNode *sM;
	sM = new streamerNode(streamerStartupData);
	sM->initializeOutput(argc, argv);
	
	cameraInfoStruct camInfo;

	// === FLOW NODE === //
	// Preliminary settings
	trackerData trackerStartupData;
	if (!trackerStartupData.assignFromXml(xP)) return -1;

	// Real-time changeable variables
	flowConfig fcData;
	fcData.assignStartingData(trackerStartupData);

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

	bool calibrationDataProcessed = false;
	cv::Mat workingFrame;

	#ifdef _USE_QT_
		std::thread t_streamer(qt_streamer_thread);
	#endif

	while (sM->wantsToRun()) {
		sM->serverCallback(scData);
		if (!sM->retrieveRawFrame()) continue;
		sM->imageLoop();
		if (!sM->get8bitImage(workingFrame)) continue;
			
		if (!calibrationDataProcessed) {
			trackerStartupData.cameraData.cameraSize.width = workingFrame.cols;
			trackerStartupData.cameraData.cameraSize.height = workingFrame.rows;
			trackerStartupData.cameraData.imageSize.at<unsigned short>(0, 0) = trackerStartupData.cameraData.cameraSize.width;
			trackerStartupData.cameraData.imageSize.at<unsigned short>(0, 1) = trackerStartupData.cameraData.cameraSize.height;
			trackerStartupData.cameraData.updateCameraParameters();
			calibrationDataProcessed = true;
			fM = new featureTrackerNode(trackerStartupData);
			fM->initializeOutput(argc, argv);
			fM->setWriteMode(!(argc >= 4));
		}
		fM->serverCallback(fcData);
		fM->handle_camera(workingFrame, &camInfo);
		fM->features_loop();
	}
	
#ifdef _USE_QT_
	t_streamer.join();
#endif

	return S_OK;
}

#ifdef _USE_QT_
void qt_streamer_thread() {
	while (a->exec()) {
		if (qt_streamer_mutex.try_lock()) {
			// ...
			qt_streamer_mutex.unlock();
		}
	}
}
#endif