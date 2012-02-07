/** \file fit.cpp 
 * \brief Defines functions declared in fit.h
 */

#include "pcl2/registration/fit.h"
#include "pcl2/eigen_matrix.h"
#include "pcl2/conversions.h"

#include <pcl/features/normal_3d.h>

pcl2::MatF
pcl2::fitPlaneLLS (Cloud cloud)
{
  return (fitPlaneLLS (cloud["xyz"]));
}

pcl2::MatF
pcl2::fitPlaneLLS (MatF points)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud = convertToPointCloudXYZ (points);

  Eigen::Vector4f plane_parameters;
  float curvature;
  pcl::computePointNormal (*point_cloud, plane_parameters, curvature);

  EigenMat<float> output (1,4);
  for (int i = 0; i < 4; ++i)
    output (0,i) = plane_parameters (i);

  return (output);
}

pcl2::MatF
pcl2::fitPlaneRANSAC (Cloud cloud, float inlier_threshold)
{
  return (fitPlaneRANSAC (cloud["xyz"], inlier_threshold));
}

pcl2::MatF
pcl2::fitPlaneRANSAC (MatF points, float inlier_threshold)
{
  bool SORRY_NOT_IMPLEMENTED_YET = false;
  assert (SORRY_NOT_IMPLEMENTED_YET);
}
