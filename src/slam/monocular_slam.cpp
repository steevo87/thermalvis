/*! \file	monocular_slam.cpp
 *  \brief	Definitions for monocular slam.
*/

#include "slam/monocular_slam.hpp"

#ifndef _BUILD_FOR_ROS_
bool slamConfig::assignStartingData(slamData& startupData) {

	verboseMode = startupData.verboseMode;
	debugMode = startupData.debugMode;
	
	return true;
}
#endif

slamData::slamData()
{
	
}

#ifdef _USE_BOOST_ 
#ifndef _BUILD_FOR_ROS_
bool slamData::assignFromXml(xmlParameters& xP) {

	int countOfNodes = 0;

	BOOST_FOREACH(boost::property_tree::ptree::value_type &v, xP.pt.get_child("launch")) {
		if (v.first.compare("node")) continue; // only progresses if its a "node" tag
		if (!v.second.get_child("<xmlattr>.type").data().compare("slam")) countOfNodes++;
	}

	if (countOfNodes == 0) {
		ROS_ERROR("No relevant nodes found in XML config!");
		return false;
	}

	if (countOfNodes > 1) {
		ROS_ERROR("More than 1 relevant node found in XML config! This functionality is not supported in Windows..");
		return false;
	}

	BOOST_FOREACH(boost::property_tree::ptree::value_type &v, xP.pt.get_child("launch")) { // Within tree (pt), finds launch, and loops all tags within it
		if (v.first.compare("node")) continue;
		if (v.second.get_child("<xmlattr>.type").data().compare("slam")) {
			if (!v.second.get_child("<xmlattr>.type").data().compare("reconfigure_gui")) {
				if (!v.second.get_child("<xmlattr>.args").data().compare("slam")) displayGUI = true;
				if (!v.second.get_child("<xmlattr>.args").data().compare("/slam")) displayGUI = true;
			}
			continue;
		}

		BOOST_FOREACH(boost::property_tree::ptree::value_type &v2, v.second) { // Traverses the subtree...
			if (v2.first.compare("param")) continue;

			if (!v2.second.get_child("<xmlattr>.name").data().compare("debugMode")) debugMode = !v2.second.get_child("<xmlattr>.value").data().compare("true");
			if (!v2.second.get_child("<xmlattr>.name").data().compare("verboseMode")) verboseMode = !v2.second.get_child("<xmlattr>.value").data().compare("true");
			
        }

		// Substitute tildes if in Windows
#ifdef _WIN32
		if (outputFolder.size() > 0) {
			if (outputFolder[0] == '~') {
				outputFolder.erase(outputFolder.begin());
				outputFolder = std::getenv("USERPROFILE") + outputFolder;
			}
		}
#endif

	}

	return true;

}
#endif
#endif

#ifdef _BUILD_FOR_ROS_
void slamNode::serverCallback(thermalvis::slamConfig &config, uint32_t level) {
#else
void slamNode::serverCallback(slamConfig &config) {
#endif

	
}

#ifdef _BUILD_FOR_ROS_
slamNode::slamNode(ros::NodeHandle& nh, slamData startupData)
#else
slamNode::slamNode(slamData startupData)
#endif
{
	configData = startupData;

	#ifdef _BUILD_FOR_ROS_
	ROS_INFO("Establishing server callback...");
	f = boost::bind (&slamNode::serverCallback, this, _1, _2);
    server.setCallback (f);
	#endif
}
