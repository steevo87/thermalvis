/*! \file	mm_calibrator.cpp
 *  \brief	Multi-modal calibration tool.
*/

#include "core/launch.hpp"
#include "streamer/directory_stream.hpp"
#include "calibrator/calib.hpp"

#define DEFAULT_LAUNCH_XML "Documents/GitHub/thermalvis/launch/calibrator_demo.launch"

#ifdef _USE_QT_
#include "mainwindow_calibrator.h"
#include "mainwindow_streamer.h"

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
		wantsToOutput(false), 
		writeMode(false), 
		output_directory(NULL), 
		xmlAddress(NULL)
	{ 
		scData = new streamerConfig;
		streamerStartupData = new streamerData;
		caData = new calibratorConfig;
		calibratorStartupData = new calibratorData;
	}
	bool initialize(int argc, char* argv[]);
	void run();
#ifndef _USE_QT_
	void start() { run(); }
#else
	void establishStreamerLink(MainWindow_streamer *gui);
	bool wantsStreamerGUI() { return streamerStartupData->displayGUI; }
	void establishCalibratorLink(MainWindow_calibrator *gui);
	bool wantsCalibratorGUI() { return calibratorStartupData->displayGUI; }
#endif
private:
	bool wantsToOutput, writeMode;
	char *output_directory;
	char *xmlAddress;
	xmlParameters xP;
	sensor_msgs::CameraInfo camInfo;
	cv::Mat workingFrame;

	bool streamerIsLinked;
	streamerConfig *scData;
	streamerData *streamerStartupData;
	streamerNode *sM;

	bool calibratorIsLinked;
	calibratorConfig *caData;
	calibratorData *calibratorStartupData;
	calibratorNode *cA;

};

int main(int argc, char* argv[]) {

	ROS_INFO("Launching MM-Calibrator Calibration Tool!");

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

	MainWindow_calibrator* w_calibrator;
	if (mainThread.wantsCalibratorGUI()) {
		w_calibrator = new MainWindow_calibrator;
		mainThread.establishCalibratorLink(w_calibrator);
		w_calibrator->show();
	}

	if (a.exec()) return 1;
#endif

	return S_OK;
}

#ifdef _USE_QT_
void ProcessingThread::establishStreamerLink(MainWindow_streamer *gui) {
	gui->linkRealtimeVariables(scData);
	streamerIsLinked = true;
}
void ProcessingThread::establishCalibratorLink(MainWindow_calibrator *gui) {
	gui->linkRealtimeVariables(caData);
	calibratorIsLinked = true;
}
#endif

void ProcessingThread::run() {

	while (sM->wantsToRun()) {
		sM->serverCallback(*scData);
		if (!sM->loopCallback()) return;
		sM->imageLoop();
		if (!sM->get8bitImage(workingFrame, camInfo)) continue;
		cA->serverCallback(*caData);
		cA->handle_camera(workingFrame, &camInfo);
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
		string xmlString = string(argv[1]);
#ifdef _WIN32
		if (xmlString.size() > 0) {
			if (xmlString[0] == '~') {
				xmlString.erase(xmlString.begin());
				xmlString = std::getenv("USERPROFILE") + xmlString;
			}
		}
#endif
		sprintf(xmlAddress, "%s", xmlString.c_str());
		ROS_INFO("Using XML file provided at (%s)", xmlAddress);
	} else {
#ifdef _WIN32
		sprintf(xmlAddress, "%s/%s", std::getenv("USERPROFILE"), DEFAULT_LAUNCH_XML);
#else
		sprintf(xmlAddress, "%s/%s", std::getenv("HOME"), DEFAULT_LAUNCH_XML);
#endif
		ROS_INFO("No XML config file provided, therefore using default at (%s)", xmlAddress);
	}

	if (!xP.parseInputXML(xmlAddress)) return false;
	ROS_INFO("About to print XML summary..");
	xP.printInputSummary();

	// === STREAMER NODE === //
	// Preliminary settings
	if (!streamerStartupData->assignFromXml(xP)) return false;

	// Real-time changeable variables
	if (!scData->assignStartingData(*streamerStartupData)) return false;

	sM = new streamerNode(*streamerStartupData);
	//sM->initializeOutput(output_directory);

	// === CALIBRATOR NODE === //
	// Preliminary settings
	if (!calibratorStartupData->assignFromXml(xP)) return true;

	// Real-time changeable variables
	if (!caData->assignStartingData(*calibratorStartupData)) return false;

	cA = new calibratorNode(*calibratorStartupData);
	cA->initializeOutput(output_directory);

	return true;
}
