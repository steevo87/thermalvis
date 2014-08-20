/*! \file	mono_slam.cpp
 *  \brief	Monocular SLAM demonstration app
*/

#include "launch.hpp"
#include "streamer/directory_stream.hpp"
#include "flow/sparse_flow.hpp"
#include "slam/monocular_slam.hpp"

#define DEFAULT_LAUNCH_XML "Documents/GitHub/thermalvis/launch/windows_demo.launch"

#ifdef _USE_QT_
#include "mainwindow_streamer.h"
#include "mainwindow_flow.h"
#include "mainwindow_slam.h"

#include <QApplication>
#include <QThread>
#include <QtCore>

#include <mutex>
#include <thread>
#endif

#ifdef _USE_QT_
class ProcessingThread : public QThread {
#else
class ProcessingThread {
#endif
public:
	ProcessingThread() : 
		streamerIsLinked(false),
		flowIsLinked(false),
		wantsToOutput(false), 
		writeMode(false), 
		output_directory(NULL), 
		xmlAddress(NULL), 
		wantsFlow(true),
		wantsSlam(true)
	{ 
		scData = new streamerConfig;
		streamerStartupData = new streamerData;
		fcData = new flowConfig;
		trackerStartupData = new trackerData;
		_slamData = new slamConfig;
		slamStartupData = new slamData;
	}
	bool initialize(int argc, char* argv[]);
	void run();
#ifndef _USE_QT_
	void start() { run(); }
#else
	void establishStreamerLink(MainWindow_streamer *gui);
	bool wantsStreamerGUI() { return streamerStartupData->displayGUI; }
	void establishFlowLink(MainWindow_flow *gui);
	bool wantsFlowGUI() { return trackerStartupData->displayGUI; }
	void establishSlamLink(MainWindow_slam *gui);
	bool wantsSlamGUI() { return slamStartupData->displayGUI; }
#endif
private:
	bool wantsToOutput, writeMode, wantsFlow, wantsSlam;
	char *output_directory;
	char *xmlAddress;
	xmlParameters xP;
	ros::sensor_msgs::CameraInfo camInfo;
	cv::Mat workingFrame;

	bool streamerIsLinked;
	streamerConfig *scData;
	streamerData *streamerStartupData;
	streamerNode *sM;

	bool flowIsLinked;
	flowConfig *fcData;
	trackerData *trackerStartupData;
	featureTrackerNode *fM;

	bool slamIsLinked;
	slamConfig *_slamData;
	slamData *slamStartupData;
	slamNode *_slamNode;
};

int main(int argc, char* argv[]) {

	ROS_INFO("Launching Monocular SLAM Demo App!");

#ifdef _USE_QT_
	QApplication a(argc, argv);
#endif

	ProcessingThread mainThread;
	
#ifdef _USE_QT_
	QObject::connect(&mainThread, SIGNAL(finished()), &a, SLOT(quit()));
#endif

	if (!mainThread.initialize(argc, argv)) return S_FALSE;
	mainThread.start();
	
#ifdef _USE_QT_
	MainWindow_streamer* w_streamer;
	if (mainThread.wantsStreamerGUI()) {
		w_streamer = new MainWindow_streamer;
		mainThread.establishStreamerLink(w_streamer);
		w_streamer->show();
	}

	MainWindow_flow* w_tracker;
	if (mainThread.wantsFlowGUI()) {
		w_tracker = new MainWindow_flow;
		mainThread.establishFlowLink(w_tracker);
		w_tracker->show();
	}

	MainWindow_slam* w_slam;
	if (mainThread.wantsSlamGUI()) {
		w_slam = new MainWindow_slam;
		mainThread.establishSlamLink(w_slam);
		w_slam->show();
	}

	if (a.exec()) return 1;
	std::cin.get();
	return S_OK;
#endif

	std::cin.get();
	return S_OK;
}

#ifdef _USE_QT_
void ProcessingThread::establishStreamerLink(MainWindow_streamer *gui) {
	gui->linkRealtimeVariables(scData);
	streamerIsLinked = true;
}
void ProcessingThread::establishFlowLink(MainWindow_flow *gui) {
	gui->linkRealtimeVariables(fcData);
	flowIsLinked = true;
}
void ProcessingThread::establishSlamLink(MainWindow_slam *gui) {
	gui->linkRealtimeVariables(_slamData);
	flowIsLinked = true;
}
#endif

void ProcessingThread::run() {

	while (sM->wantsToRun()) {
		sM->serverCallback(*scData);
		if (!sM->retrieveRawFrame()) return;
		sM->imageLoop();
		
		if (wantsFlow) {
			if (!sM->get8bitImage(workingFrame, camInfo)) continue;
			
			fM->serverCallback(*fcData);
			fM->handle_camera(workingFrame, &camInfo);
			fM->features_loop();

			if (wantsSlam) {
				int a = 1;
			}
		}
		
	}
}

bool ProcessingThread::initialize(int argc, char* argv[]) {

	xmlAddress = new char[256];
	output_directory = new char[256];

	wantsToOutput = false;
	if (argc >= 3) {
		printf("%s << Using data output directory of <%s>.\n", __FUNCTION__, argv[2]);
		wantsToOutput = true;
		sprintf(output_directory, "%s", argv[2]);
	} else {
		output_directory = NULL;
	}

	writeMode = !(argc >= 4);
	
	if (argc > 1) {
		sprintf(xmlAddress, "%s", argv[1]);
		ROS_INFO("Using XML file provided at (%s)", xmlAddress);
	} else {
#ifdef _WIN32
		sprintf(xmlAddress, "%s/%s", std::getenv("USERPROFILE"), DEFAULT_LAUNCH_XML);
#else
		sprintf(xmlAddress, "%s/%s", std::getenv("HOME"), DEFAULT_LAUNCH_XML);
#endif
		ROS_INFO("No XML config file provided, therefore using default at (%s)", xmlAddress);
	}

	xP.parseInputXML(xmlAddress);
	ROS_INFO("About to print XML summary..");
	xP.printInputSummary();

	// === STREAMER NODE === //
	// Preliminary settings
	if (!streamerStartupData->assignFromXml(xP)) return false;

	// Real-time changeable variables
	if (!scData->assignStartingData(*streamerStartupData)) return false;

	sM = new streamerNode(*streamerStartupData);
	sM->initializeOutput(output_directory);

	// === FLOW NODE === //
	// Preliminary settings
	wantsFlow = trackerStartupData->assignFromXml(xP);

	if (wantsFlow) {
		// Real-time changeable variables
		fcData->assignStartingData(*trackerStartupData);

		#ifdef _DEBUG
		if (
			((fcData->getDetector1() != DETECTOR_FAST) && (fcData->getDetector1() != DETECTOR_OFF)) || 
			((fcData->getDetector2() != DETECTOR_FAST) && (fcData->getDetector2() != DETECTOR_OFF)) || 
			((fcData->getDetector3() != DETECTOR_FAST) && (fcData->getDetector3() != DETECTOR_OFF))
		) {
			ROS_WARN("The GFTT/HARRIS detector is EXTREMELY slow in the Debug build configuration, so consider switching to an alternative while you are debugging.");
		}
		#endif

		fM = new featureTrackerNode(*trackerStartupData);
		fM->initializeOutput(output_directory);
		fM->setWriteMode(writeMode);
	}

	// === SLAM NODE === //
	// Preliminary settings
	wantsSlam = slamStartupData->assignFromXml(xP);

	if (wantsSlam) {
		// Real-time changeable variables
		if (!_slamData->assignStartingData(*slamStartupData)) return false;

		_slamNode = new slamNode(*slamStartupData);
		_slamNode->initializeOutput(output_directory);
	}

	return true;
}
