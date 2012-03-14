/** \file fit.hpp
 * \brief Defines functions declared in fit.h
 */

#include "pcl2/registration/fit.h"

#include "pcl2/core.h"
#include "pcl2/eigen_matrix.h"
#include "pcl2/stats.h"
#include "pcl2/math.h"

#include "pcl2/conversions.h"
#include <pcl/features/normal_3d.h>

template <typename T>
pcl2::TypedMat<T>
pcl2::fitPlaneLLS (const TypedMat<T> & points)
{
  typedef TypedMat<T> MatT;
  MatT cov = computeCovariance (points);
  MatT eigvecs = computeEigenvectors3x3 (cov);
  MatT normal = eigvecs (0).copy ();

  ////// Double-check
  pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud = convertToPointCloudXYZ (points);
  Eigen::Vector4f plane_parameters;
  float curvature;
  pcl::computePointNormal (*point_cloud, plane_parameters, curvature);
  for (int i = 0; i < 3; ++i)
  {
    assert (fabs(normal (0, i) - plane_parameters (i)) < 1e-3 ||
            fabs(normal (0, i) + plane_parameters (i)) < 1e-3);
  }
  //////

  return (normal);
}


template <typename T>
pcl2::TypedMat<T>
pcl2::fitPlaneRANSAC (const TypedMat<T> & points, float inlier_threshold)
{
  const bool SORRY_NOT_IMPLEMENTED_YET = false;
  assert (SORRY_NOT_IMPLEMENTED_YET);
}
