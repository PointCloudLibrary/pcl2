/** \file conversions.h 
 * \brief Declares several functions for converting between pcl and pcl2 data structures
 */

#ifndef PCL2_CONVERSIONS_H
#define PCL2_CONVERSIONS_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>

#include "pcl2/cloud.h"
#include "pcl2/typed_matrix.h"

namespace pcl2
{

pcl::PointCloud<pcl::PointXYZ>::Ptr
convertToPointCloudXYZ (const MatF & xyz);

boost::shared_ptr<sensor_msgs::PointCloud2>
convertCloudToPointCloud2 (const Cloud & input);

Cloud
convertPointCloud2ToCloud (const sensor_msgs::PointCloud2 & input);

}
#endif
