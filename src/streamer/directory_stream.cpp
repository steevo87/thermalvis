#include "streamer/directory_stream.hpp"

directoryManager::directoryManager() : loopMode(false) {
	input_directory = new char[MAX_INPUT_ARG_LENGTH];
}

void directoryManager::setLoopMode(bool val) {
	loopMode = val;
}

bool directoryManager::grabFrame() {
	
	if (FrameCounter1 >= int(file_list.size())) {
		if (loopMode) {
			FrameCounter1 = 0;
		} else {
			return false;
		}
	}

	std::string full_path = std::string(input_directory) + "/" + file_list.at(FrameCounter1);

	*rawImage = read_image_from_file(full_path);
	FrameCounter1++;
	
	return true;
}

bool directoryManager::initializeInput(int argc, char* argv[]) {
	if (argc < 2) {
		printf("%s << Warning! No input directory specified so using default data sample directory of <%s>.\n", __FUNCTION__, _DEFAULT_SAMPLE_DATA_);
		sprintf(input_directory, "%s", _DEFAULT_SAMPLE_DATA_);
	} else {
		printf("%s << Using data input directory of <%s>.\n", __FUNCTION__, argv[1]);
		sprintf(input_directory, "%s", argv[1]);
	}
	return true;
}

bool directoryManager::initialize() {
	
	// Get list of files in directory!
	std::string full_dir = string(input_directory) + "/";

#ifdef _USING_BOOST_
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

				if ((name == ".") || (name == "..") || (name[0] == '.') || (name.size() < 5)) continue;

				if (name.size() > 5) {
					if ((name[name.size()-4] != '.') && (name[name.size()-5] != '.')) continue;
				} else {
					if (name[name.size()-4] != '.') continue;
				}
				file_list.push_back(name);
			}
		}
		

	} else return false;
	return true;
#else
	return false;
#endif
}
