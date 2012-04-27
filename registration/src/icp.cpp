/**
 * Software License Agreement (BSD License)
 * 
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2009-2012, Willow Garage, Inc.
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met: 
 * 
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Willow Garage, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

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
