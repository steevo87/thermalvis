#include "mono_slam.h"

directoryManager::directoryManager() : wantsToOutput(false), FrameCounter1(0), isValid(true) {
	cMapping = cScheme(DEFAULT_COLORSCHEME_CODE);
	colormap_index = DEFAULT_COLORSCHEME_CODE;

	minTemp = DEFAULT_MIN_TEMP; 
	maxTemp = DEFAULT_MAX_TEMP;

	pauseMode = false;
	autoscaleTemps = true;

	tD.debugMode = true;
	fD = new featureTrackerNode(tD);

	//fD = new cv::FastFeatureDetector( 15 );

}

bool directoryManager::grabFrame() {
	
	if (FrameCounter1 >= int(file_list.size())) return false;

	
		
	std::string full_path = std::string(input_directory) + "/" + file_list.at(FrameCounter1);

	

	if (rawImage.rows == 0) {

		#ifdef _OPENCV_VERSION_3_PLUS_
		rawImage = cv::imread(full_path, cv::IMREAD_ANYDEPTH);
		#else
		rawImage = cv::imread(full_path, CV_LOAD_IMAGE_ANYDEPTH);
		#endif

		scaledImage = cv::Mat(rawImage.rows, rawImage.cols, CV_16UC1);
		_8bitImage = cv::Mat(rawImage.rows, rawImage.cols, CV_8UC1);
		colorImage = cv::Mat(rawImage.rows, rawImage.cols, CV_8UC3);

	} else {
		#ifdef _OPENCV_VERSION_3_PLUS_
		rawImage = cv::imread(full_path, cv::IMREAD_ANYDEPTH);
		#else
		rawImage = cv::imread(full_path, CV_LOAD_IMAGE_ANYDEPTH);
		#endif
	}
	
	if (FrameCounter1 == 0) {
		tD.cameraData.cameraSize.width = rawImage.cols;
		tD.cameraData.cameraSize.height = rawImage.rows;
		tD.cameraData.imageSize.at<unsigned short>(0, 0) = tD.cameraData.cameraSize.width;
		tD.cameraData.imageSize.at<unsigned short>(0, 1) = tD.cameraData.cameraSize.height;
		tD.cameraData.updateCameraParameters();
		fD->updateTrackerData(tD);
		fD->process_info();
	}

	// Read in image to rawImage!

	if (autoscaleTemps) {
		normalize_16(scaledImage, rawImage);
	} else {
		double minLevel = minTemp * 100.0;
		double maxLevel = maxTemp * 100.0;
		normalize_16(scaledImage, rawImage, minLevel, maxLevel);
	}
	
	down_level(_8bitImage, scaledImage);

	// Try local feature tracking here?
	
	//if (FrameCounter1 == 0) {
		//fD->detect(_8bitImage, detectedKeypoints);
		//cv::KeyPoint::convert(detectedKeypoints, newlySensedPoints);
	//}
	
	fD->handle_camera(_8bitImage);
	fD->features_loop();
	fD->getDisplayImage(colorImage);
	
	!pauseMode ? cv::imshow("optrisVideo", colorImage) : 0;

	char key = cv::waitKey(1);
	
	if (key == 'q') isValid = false;
	
	if (wantsToOutput) {
		char imFilename[256];
		sprintf(imFilename, "%s/frame%06d.png", output_directory, FrameCounter1);
		string imageFilename(imFilename);
		cv::imwrite(imageFilename, rawImage);	
	}

	FrameCounter1++;
	
	return 0;
}

bool directoryManager::wantsToRun() {
	return isValid;
}

bool directoryManager::setOutputDir(char* output_dir) {
	printf("%s << User has opted to output images to the following directory: <%s>\n", __FUNCTION__, output_dir);
	wantsToOutput = true;
	output_directory = new char[256];
	sprintf(output_directory, "%s", output_dir);
	return true;
}

bool directoryManager::initialize(char* input_dir) {
	input_directory = new char[256];
	sprintf_s(input_directory, 256, "%s", input_dir);
	
	// Get list of files in directory!

	std::string full_dir = string(input_directory) + "/";

	fs::path someDir(full_dir);
		
	if ( fs::exists(someDir) && fs::is_directory(someDir)) {

		fs::directory_iterator end_iter;

				
		for( fs::directory_iterator dir_iter(someDir) ; dir_iter != end_iter ; ++dir_iter) {
			if (fs::is_regular_file(dir_iter->status()) ) {

				std::stringstream temp;
				temp << dir_iter->path().filename();
				string name;

				name = temp.str();

				

				boost::replace_all(name, "\"", "");

				if ((name == ".") || (name == "..") || (name[0] == '.') || (name.size() < 5)) {
					continue;
				}

				if (name.size() > 5) {
					if ((name[name.size()-4] != '.') && (name[name.size()-5] != '.')) {
						continue;
					}
				} else {
					if (name[name.size()-4] != '.') {
						continue;
					}
				}

				file_list.push_back(name);
				//printf("%s << Filename = (%s)\n", __FUNCTION__, name.c_str());

				/*
				string candidateName = source + "/" + name;
				
				bool alreadyFound = false;
				
				for (unsigned int iii = 0; iii < filenames.size(); iii++) {
					if (candidateName == filenames.at(iii)) {
						alreadyFound = true;
					}
				}
				
				if (!alreadyFound) {
					//cout << "Pushing back..." << candidateName << endl;
					filenames.push_back(candidateName);
					im = imread(source + "/" + name);
					//imshow("test", im);
					//waitKey();
					ratings[images.size()] = 0.0;
					images.push_back(im);
					
				}
				*/
			}
		}
		// skips dodgy items (e.g. hidden files)
		// checks if candidate name already exists in list
		// if not already found, adds to list

		//std::printf("%s << DONE.\n", __FUNCTION__);

	} else {
		return false;
	}

	return true;
}

int main(int argc, char* argv[]) {
	printf("%s << Launching Directory Reader Demo App!\n", __FUNCTION__);
	
	if (argc < 2) {
		printf("%s << Error! A directory must be specified for input images.\n", __FUNCTION__);
		return -1;
	}
		
	directoryManager *dM;

	dM = new directoryManager();
	if (!dM->initialize(argv[1])) return -1;
	if (argc > 2) dM->setOutputDir(argv[2]);
	while (dM->wantsToRun()) {
		dM->grabFrame();
	}
	
	return S_OK;
}
