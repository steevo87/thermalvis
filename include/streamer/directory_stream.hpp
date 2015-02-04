/*! \file	directory_stream.hpp
 *  \brief	Declarations for directory video input.
*/

#ifndef _THERMALVIS_DIRECTORY_STREAM_H_
#define _THERMALVIS_DIRECTORY_STREAM_H_
	
#include "input_stream.hpp"

#ifdef _USE_BOOST_
#include <boost/algorithm/string/replace.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/path.hpp>
namespace fs = boost::filesystem;
#endif

class directoryManager : public inputStream {

protected:
	char *input_directory;
	bool loopMode;
	std::vector<std::string> file_list;

public:
	directoryManager();
	bool grabFrame();
	void setLoopMode(bool val);
	bool initialize();
	bool initializeInput(int argc, char* argv[]);
};

#endif
