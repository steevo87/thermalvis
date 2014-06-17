#include "input_stream.hpp"

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

void inputStream::colorizeFrame() {
	cMapping->falsify_image(*_8bitImage, *displayImage, 0);		// THIS IS SLOWING THINGS DOWN!!
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