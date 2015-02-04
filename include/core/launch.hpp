/*! \file	launch.hpp
 *  \brief	Declarations for parsing XML launch files outside of ROS.
*/

#ifndef THERMALVIS_LAUNCH_H
#define THERMALVIS_LAUNCH_H

#include "tools.hpp"

#ifdef _USE_BOOST_

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>

//#include "flow/sparse_flow.hpp"

#define ATTR_SET ".<xmlattr>"

class xmlParameters {
	friend class trackerData;
	friend class streamerData;
	friend class slamData;
	friend class calibratorData;

protected:
	boost::property_tree::ptree pt;

public:
	bool parseInputXML(char *file_address);
	void printInputSummary();
};
	
#endif

#endif
