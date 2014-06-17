#include "optris_stream.hpp"

void winOptrisStream::assignMemoryToRawImage(int height, int width) {
	rawImage = new cv::Mat(height, width, CV_16UC1);
}

void winOptrisStream::assignDataToRawImage(uchar *buff) {
	rawImage->data = buff;
}

bool winOptrisStream::getPauseMode() {
	return pauseMode;
}

bool winOptrisStream::switchPauseMode() {
	pauseMode = !pauseMode;
	return pauseMode;
}

void winOptrisStream::setMinTemp(double temperature) {
	minTemp = temperature;
}

void winOptrisStream::setMaxTemp(double temperature) {
	maxTemp = temperature;
}

void winOptrisStream::load_standard(int mapCode, int mapParam) {
	cMapping->load_standard(mapCode, mapParam);
}

int winOptrisStream::get_colormap_index() {
	return colormap_index;
}

void winOptrisStream::set_colormap_index(int index) {
	colormap_index = index;
}

void winOptrisStream::set_autoscaleTemps(bool setting) {
	autoscaleTemps = setting;
}