/*! \file	directory_stream.hpp
 *  \brief	Declarations for directory video input.
*/

#ifndef _THERMALVIS_DIRECTORY_STREAM_H_
#define _THERMALVIS_DIRECTORY_STREAM_H_
	
#include "input_stream.hpp"

#include <boost/algorithm/string/replace.hpp>
#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

class directoryManager : public inputStream {

protected:
	char *input_directory;

	std::vector<std::string> file_list;

public:
	directoryManager();
	bool grabFrame();
	bool initialize();
	bool initializeInput(int argc, char* argv[]);
};
	
#endif
