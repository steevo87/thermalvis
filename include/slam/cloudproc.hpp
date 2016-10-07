/*! \file	cloudproc.hpp
 *  \brief	Declarations for cloud processing.
*/

#ifndef THERMALVIS_CLOUDPROC_H
#define THERMALVIS_CLOUDPROC_H

#ifdef _USE_PCL_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "core/improc.hpp"
#include "core/colormapping.hpp"

#include "slam/triangulation.hpp"

#define DPM_GRAY					0
#define DPM_AXES					1

void findThermalCloudPercentiles(pcl::PointCloud<pcl::RGB>::Ptr& thermalCloud, double *vals, double *percentiles, unsigned int num);
	
class cCloudScheme : public cScheme {
	
public:

	/// \brief 		Create a false colour version of a monochromatic thermal cloud
	/// \param 		ioCloud			Base cloud, for color update
	/// \param 		thermalCloud	Cloud containing thermal data
	/// \param 		deadpointMode	Instruction for how to colorize dead points
	///					[0] : color dead points as a midtone gray
	///					[1] : color dead points according to their position along an axis
	/// \param 		volumeSize		Size of volume (needed for some dead point modes)
	void falsify_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ioCloud, pcl::PointCloud<pcl::RGB>::Ptr& thermalCloud, double minTemp = 20.0, double maxTemp = 45.0, int deadPointMode = DPM_GRAY, double volumeSize = 3.0, double minConfidence = 0.001, bool clampOutliers = true);
	void falsify_cloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& ioCloud, pcl::PointCloud<pcl::RGB>::Ptr& thermalCloud, double minTemp = 20.0, double maxTemp = 45.0, int deadPointMode = DPM_GRAY, double volumeSize = 3.0, double minConfidence = 0.001, bool clampOutliers = true);
	
	/// \brief 		Create a representation of the cloud that shows the confidence of each point in thermal
	/// \param 		ioCloud			Base cloud, for color update
	/// \param 		thermalCloud	Cloud containing thermal data
	void confidence_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ioCloud, pcl::PointCloud<pcl::RGB>::Ptr& thermalCloud);
	
	/// \brief 		Create a false colour combination of a color or monochrome visual cloud and a monochrome thermal cloud.
	/// \param 		ioCloud			Base cloud, for color update
	/// \param 		colorCloud		Cloud containing the RGB data
	/// \param 		thermalCloud	Cloud containing thermal data
	/// \param 		deadpointMode	Instruction for how to colorize dead points
	///					[0] : color dead points as a midtone gray
	///					[1] : color dead points according to their position along an axis
	/// \param 		volumeSize		Size of volume (needed for some dead point modes)
	/// \param 		params		Pointer to double values dictating the following parameters:
	///					[0] : minimum lightness (default = 0.2)
	///					[1] : maximum lightness (default = 0.8)
	void fuse_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ioCloud, pcl::PointCloud<pcl::RGB>::Ptr& colorCloud, pcl::PointCloud<pcl::RGB>::Ptr& thermalCloud, double minTemp = 20.0, double maxTemp = 45.0, int deadPointMode = DPM_GRAY, double volumeSize = 3.0, double minConfidence = 0.001, bool clampOutliers = true, double *params = NULL);
	void fuse_cloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& ioCloud, pcl::PointCloud<pcl::RGB>::Ptr& colorCloud, pcl::PointCloud<pcl::RGB>::Ptr& thermalCloud, double minTemp = 20.0, double maxTemp = 45.0, int deadPointMode = DPM_GRAY, double volumeSize = 3.0, double minConfidence = 0.001, bool clampOutliers = true, double *params = NULL);
	
};

#endif // THERMALVIS_CLOUDPROC_H

#endif // _USE_PCL_
