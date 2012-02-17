/** \file fit.cpp
 * \brief Defines functions declared in fit.h
 */

#include "pcl2/registration/fit.h"
#include "pcl2/registration/impl/fit.hpp"

pcl2::MatF
pcl2::fitPlaneLLS (const Cloud & cloud)
{
  MatF points = cloud["xyz"];
  return (fitPlaneLLS (points));
}

pcl2::MatF
pcl2::fitPlaneRANSAC (const Cloud & cloud, float inlier_threshold)
{
  MatF points = cloud["xyz"];
  return (fitPlaneRANSAC (points, inlier_threshold));
}
