#include "input_stream.hpp"

streamerSharedData::streamerSharedData() : 
	maxReadAttempts(0), 
	framerate(-1.0),
	normMode(NORM_MODE_STANDARD),
	normFactor(0.0),
	output16bit(false), 
	output8bit(true),
	outputColor(false),
	map(CONFIG_MAP_CODE_CIELUV),
	undistortImages(false), 
	fusionFactor(0.6),  
	serialPollingRate(25.0),
	maxNucInterval(45),
	maxNucThreshold(0.2),
	verboseMode(false), 
	autoTemperature(false),
	minTemperature(20.0), 
	maxTemperature(40.0),
	debugMode(false)
{ }

streamerData::streamerData() : 
	detectorMode(DETECTOR_MODE_INS), 
	filterMode(IMAGE_FILTER_NONE), 
	filterParam(2.0),
	usbMode(USB_MODE_16),
	inputDatatype(DATATYPE_RAW),
	outputType(OUTPUT_TYPE_CV_16UC1)
{ }

bool streamerData::assignFromXml(xmlParameters& xP) {

	int countOfFlowNodes = 0;

	BOOST_FOREACH(boost::property_tree::ptree::value_type &v, xP.pt.get_child("launch")) {
		if (v.first.compare("node")) continue; // only progresses if its a "node" tag
		if (!v.second.get_child("<xmlattr>.type").data().compare("flow")) countOfFlowNodes++;
	}

	if (countOfFlowNodes == 0) {
		ROS_ERROR("No flow nodes found in XML config!");
		return false;
	}

	if (countOfFlowNodes > 1) {
		ROS_ERROR("More than 1 flow node found in XML config! This functionality is not supported in Windows..");
		return false;
	}

	BOOST_FOREACH(boost::property_tree::ptree::value_type &v, xP.pt.get_child("launch")) { // Within tree (pt), finds launch, and loops all tags within it
		if (v.first.compare("node")) continue;
		if (v.second.get_child("<xmlattr>.type").data().compare("flow")) continue;

		BOOST_FOREACH(boost::property_tree::ptree::value_type &v2, v.second) { // Traverses the subtree...
			if (v2.first.compare("param")) continue;

			if (!v2.second.get_child("<xmlattr>.name").data().compare("debugMode")) debugMode = !v2.second.get_child("<xmlattr>.value").data().compare("true");
			if (!v2.second.get_child("<xmlattr>.name").data().compare("verboseMode")) verboseMode = !v2.second.get_child("<xmlattr>.value").data().compare("true");
			
			// ...

        }
	}

	return true;

}

#ifndef _BUILD_FOR_ROS_
streamerConfig::streamerConfig() { }

void streamerConfig::assignStartingData(streamerData& startupData) {

	/*
	maxFeatures = startupData.maxFeatures;
	minFeatures = startupData.minFeatures;
	drawingHistory = startupData.drawingHistory;
	matchingMode = startupData.matchingMode;

	maxFrac = startupData.maxFrac;
	flowThreshold = startupData.flowThreshold;
	minSeparation = startupData.minSeparation;
	maxVelocity = startupData.maxVelocity;
	newFeaturesPeriod = startupData.newFeaturesPeriod;
	delayTimeout = startupData.delayTimeout;
	
	verboseMode = startupData.verboseMode;
	debugMode = startupData.debugMode;
	showTrackHistory = startupData.showTrackHistory;

	adaptiveWindow = startupData.adaptiveWindow;
	velocityPrediction = startupData.velocityPrediction;
	attemptHistoricalRecovery = startupData.attemptHistoricalRecovery;
	autoTrackManagement = startupData.autoTrackManagement;
	attemptMatching = startupData.attemptMatching;
	detectEveryFrame = startupData.detectEveryFrame;

	sensitivity_1 = startupData.sensitivity[0];
	sensitivity_2 = startupData.sensitivity[1];
	sensitivity_3 = startupData.sensitivity[2];

	if ((!startupData.detector[0].compare("FAST")) || (!startupData.detector[0].compare("fast"))) {
		detector_1 = DETECTOR_FAST;
	} else if ((!startupData.detector[0].compare("GFTT")) || (!startupData.detector[0].compare("gftt"))) {
		detector_1 = DETECTOR_GFTT;
	} else if ((!startupData.detector[0].compare("HARRIS")) || (!startupData.detector[0].compare("harris"))) {
		detector_1 = DETECTOR_HARRIS;
	} else {
		ROS_ERROR("Could not identify provided detector..");
		detector_1 = DETECTOR_FAST;
	}

	if ((!startupData.detector[1].compare("FAST")) || (!startupData.detector[1].compare("fast"))) {
		detector_2 = DETECTOR_FAST;
	} else if ((!startupData.detector[1].compare("GFTT")) || (!startupData.detector[1].compare("gftt"))) {
		detector_2 = DETECTOR_GFTT;
	} else if ((!startupData.detector[1].compare("HARRIS")) || (!startupData.detector[1].compare("harris"))) {
		detector_2 = DETECTOR_HARRIS;
	} else {
		detector_2 = DETECTOR_OFF;
	}

	if ((!startupData.detector[2].compare("FAST")) || (!startupData.detector[2].compare("fast"))) {
		detector_3 = DETECTOR_FAST;
	} else if ((!startupData.detector[2].compare("GFTT")) || (!startupData.detector[2].compare("gftt"))) {
		detector_3 = DETECTOR_GFTT;
	} else if ((!startupData.detector[2].compare("HARRIS")) || (!startupData.detector[2].compare("harris"))) {
		detector_3 = DETECTOR_HARRIS;
	} else {
		detector_3 = DETECTOR_OFF;
	}
	*/

}
#endif

inputStream::inputStream() {
	cMapping = new cScheme(DEFAULT_COLORSCHEME_CODE);
	colormap_index = DEFAULT_COLORSCHEME_CODE;

	minTemp = DEFAULT_MIN_TEMP; 
	maxTemp = DEFAULT_MAX_TEMP;

	autoscaleTemps = true;

	rawImage = new cv::Mat();
	scaledImage = new cv::Mat();
	_8bitImage = new cv::Mat();
}

bool inputStream::accessLatestRawFrame(cv::Mat& latestFrame) {
	if (rawImage->rows == 0) return false;
	latestFrame = *rawImage;
	return true;
}

bool inputStream::accessLatest8bitFrame(cv::Mat& latestFrame) {
	if (_8bitImage->rows == 0) return false;
	latestFrame = *_8bitImage;
	return true;
}

void inputStream::displayFrame() {
	if (displayImage->rows != 0) {
		!pauseMode ? cv::imshow("display", *displayImage) : 0;
	} else if (_8bitImage->rows != 0) {
		!pauseMode ? cv::imshow("display", *_8bitImage) : 0;
	} else {
		!pauseMode ? cv::imshow("display", *rawImage) : 0;
	}
	
	char key = cv::waitKey(1);
	if (key == 'q') isValid = false;
}

bool inputStream::processFrame() {

	if (scaledImage->rows == 0) {
		scaledImage = new cv::Mat(rawImage->rows, rawImage->cols, CV_16UC1);
		_8bitImage = new cv::Mat(rawImage->rows, rawImage->cols, CV_8UC1);
		displayImage = new cv::Mat(rawImage->rows, rawImage->cols, CV_8UC3);
	}

	if (autoscaleTemps) {
		normalize_16(*scaledImage, *rawImage);
	} else {
		double minLevel = minTemp * 100.0;
		double maxLevel = maxTemp * 100.0;
		normalize_16(*scaledImage, *rawImage, minLevel, maxLevel);
	}
	
	down_level(*_8bitImage, *scaledImage);

	return 0;
}

bool inputStream::writeImageToDisk() {
	if (wantsToOutput) {
		char imFilename[256];
		sprintf(imFilename, "%s/frame%06d.png", output_directory, FrameCounter1);
		std::string imageFilename(imFilename);

		if (writeInColor && (displayImage->rows != 0)) {
			cv::imwrite(imageFilename, *displayImage);
		} else if (writeInColor && (_8bitImage->rows != 0)) {
			cv::imwrite(imageFilename, *_8bitImage);
		} else {
			cv::imwrite(imageFilename, *rawImage);
		}

		return true;
	}
	return false;
}