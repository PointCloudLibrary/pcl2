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

/** \file icp.h 
 * \brief Declares classes and functions for aligning point clouds using the Iterative Closest Point (ICP) algorithm.
 */

// This is still just an outline, which I doubt will even compile.
// It also contains lots of stuff that belongs in other files. 

#ifndef PCL2_ICP_H
#define PCL2_ICP_H

#include "pcl2/cloud.h"
#include "pcl2/registration/correspondence_identification.h"
#include "pcl2/eigen_matrix.h"

namespace pcl2
{

class RigidTransformationEstimation
{
public:
  typedef boost::shared_ptr<RigidTransformationEstimation> Ptr;

  RigidTransformationEstimation ();
  virtual MatF estimateTransformation (const PointCorrespondences & correspondences);
};

class ICP
{
public:
  /** \brief \b ICP is an implementation of the classic Iterative Closest Point algorithm.
   * This implementation allows for multiple configurations "Efficient Variants of the ICP Algorithm"
   */
  ICP (size_t max_iterations = 100);

  /** \brief Align a source cloud to a target cloud by solving for the rigid transformation that minimizes the 
   * registration error between the two clouds.
   * \param source The cloud that will be aligned
   * \param target The cloud to which the source will be aligned
   * \return A copy of the source cloud that has been transformed to align with the target.
   */
  Cloud align (const Cloud & source, const Cloud & target);

  /** \todo write me */
  void setCorrespondenceIdentification (const CorrespondenceIdentification::Ptr & ci)
  {
    correspondence_identification_ = ci;
  }

  /** \todo write me */
  void setRigidTransformationEstimation (const RigidTransformationEstimation::Ptr & rte)
  {
    transformation_estimation_ = rte;
  }

  /** \brief Get the source-to-target transform estimated by the most recent alignment. 
   * \return A 4x4 matrix containing the estimated rigid transformation from the source cloud to the target cloud.
   */
  inline MatF getFinalTransformation () const
  { 
    return (transformation_); 
  }

  /** \brief Get the estimated registration error of the most recent alignment.
   * \return The registration error of the last alignment, as determined by the specified error metric.
   */
  inline float getRegistrationError () const
  {
    return (registration_error_);
  }

  /** \brief Determine if the convergence criteria were satisfied by the most recent alignment.
   * \return The convergence state of the last alignment.  This return value will be true if the specified convergence
   * critera were met; it will be false if the maximum number of iterations was reached.
   */
  inline bool converged () const
  {
    return (converged_); 
  }

protected:
  size_t max_iterations_;
  CorrespondenceIdentification::Ptr correspondence_identification_;
  RigidTransformationEstimation::Ptr transformation_estimation_;

  bool converged_;
  EigenMat<float> transformation_;
  float registration_error_;

};

}

#endif // PCL2_ICP_H
