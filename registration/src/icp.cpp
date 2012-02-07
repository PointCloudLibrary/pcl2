/** \file icp.cpp 
 * \brief Defines functions and classes declared in icp.h
 */

#include "pcl2/registration/icp.h"
#include "pcl2/registration/transform.h"

pcl2::ICP::ICP (size_t max_iterations) :
  max_iterations_ (max_iterations),
  correspondence_identification_ (new CorrespondenceIdentification),
  transformation_estimation_ (new RigidTransformationEstimation),
  converged_ (false),
  transformation_ (4, 4)
{
  // todo: set transformation to the identity matrix
}

pcl2::Cloud
pcl2::ICP::align (const Cloud & source, const Cloud & target)
{
  Cloud aligned_source = source;
  for (size_t i = 0; i < max_iterations_; ++i)
  {
    // Identify correspondences
    PointCorrespondences correspondences = 
      correspondence_identification_->identifyCorrespondences (aligned_source, target);

    // Estimate a rigid transformation between
    transformation_ *= transformation_estimation_->estimateTransformation (correspondences);
    
    // Transform source cloud
    aligned_source = transform (source, transformation_);

    // Check for convergence
    converged_ = false; /// \todo checkForConvergenceSomehow (); 
    if (converged_)
      break;
  }
  return (aligned_source);
}



/// \todo Move me to a different file!

pcl2::RigidTransformationEstimation::RigidTransformationEstimation () {}

pcl2::MatF
pcl2::RigidTransformationEstimation::estimateTransformation (const PointCorrespondences & correspondences)
{
  EigenMat<float> tform (4,4);
  return (tform);
}
