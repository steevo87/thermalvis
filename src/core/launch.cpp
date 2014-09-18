/*! \file	launch.cpp
 *  \brief	Definitions for processing XML launch files.
*/

#include "core/launch.hpp"

#ifdef _USE_BOOST_

bool xmlParameters::parseInputXML(char *file_address) {

	if (!boost::filesystem::exists(file_address)) {
		ROS_ERROR("Launch file (%s) doesn't exist. Please check.", file_address);
		return false;
	}
	boost::property_tree::xml_parser::read_xml(std::string(file_address), pt);

	return true;
}

void xmlParameters::printInputSummary() {

	std::cout << std::endl << "<<< XML SUMMARY >>>" << std::endl; 

	BOOST_FOREACH(boost::property_tree::ptree::value_type &v, pt.get_child("launch")) { // Within tree (pt), finds launch, and loops all tags within it
		if (v.first.compare("node")) continue; // only progresses if its a "node" tag
		
		std::cout << "Extracted attributes from <" << v.second.get_child("<xmlattr>.name").data() << "> " << v.first << ":" << std::endl;

		BOOST_FOREACH(boost::property_tree::ptree::value_type &v2, v.second.get_child("<xmlattr>")) {
            std::cout << "\t// " << v2.first.data() << " = " << v2.second.data() << std::endl;
        }

		BOOST_FOREACH(boost::property_tree::ptree::value_type &v2, v.second) { // Traverses the subtree...
			if (v2.first.compare("param")) continue; // only progresses if its a "param" tag

			std::cout << "\t" << v2.second.get_child("<xmlattr>.name").data() << " = " << v2.second.get_child("<xmlattr>.value").data() << std::endl;
        }
	}

	std::cout << std::endl << "<<< END SUMMARY >>>" << std::endl; 

}

#endif
