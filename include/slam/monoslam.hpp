#ifndef _THERMALVIS_MONOSLAM_HPP_
#define _THERMALVIS_MONOSLAM_HPP_

#ifdef _FOR_REF_ONLY_

#include "feature_tracks.h"
#include "monoslamConfig.h"
typedef dynamic_reconfigure::Server < thermalvis::monoslamConfig > Server;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

const char __PROGRAM__[] = "THERMALVIS_MONOSLAM";

bool wantsToShutdown = false;
void mySigintHandler(int sig);


/// \brief		Manages the SLAM procedure
class slamNode {
private:

	
	
public:

	slamNode(ros::NodeHandle& nh, slamData startupData);
	
	
	void prepareForTermination();
	
};

boost::shared_ptr < slamNode > *globalNodePtr;

#endif

#endif // _THERMALVIS_MONOSLAM_HPP_