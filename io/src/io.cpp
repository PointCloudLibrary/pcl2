/** \file io.cpp 
 * \brief Defines functions declared in io.h
 */

#include "pcl2/io/io.h"
#include "pcl2/conversions.h"

#include <pcl/io/pcd_io.h>

pcl2::Cloud pcl2::loadCloud (const std::string & filename)
{
  sensor_msgs::PointCloud2 pc2;
  pcl::io::loadPCDFile (filename, pc2);
  return (pcl2::convertPointCloud2ToCloud (pc2));
}

void pcl2::saveCloud (const std::string & filename, const pcl2::Cloud & cloud)
{
  boost::shared_ptr<sensor_msgs::PointCloud2> pc2 = convertCloudToPointCloud2 (cloud);
  pcl::io::savePCDFile (filename, *pc2);
}
