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

/** \file fit.h 
 * \brief Declares example functions for fitting planes to point clouds
 */

/// \todo Move this file (fit.h) to a different module (not registration)

#ifndef PCL2_FIT_H
#define PCL2_FIT_H

#include "pcl2/cloud.h"
#include "pcl2/typed_matrix.h"

namespace pcl2
{

/** \brief Fit a plane to a given cloud of 3D points using Linear Least Squares (LLS)
 * \param cloud A cloud containing an "xyz" channel of 3D points
 * \return A MatF containing the 4 coefficients of the best fitting plane, 
 * given in \f$ c_0 x + c_1 y + c_2 z = c_3 \f$ form
 */
MatF fitPlaneLLS (const Cloud & cloud);

/** \brief Fit a plane to a channel of 3D points using Linear Least Squares (LLS)
 * \param points An n by 3 MatF representing n 3D points
 * \return A MatF containing the 4 coefficients of the best fitting plane, 
 * given in \f$ c_0 x + c_1 y + c_2 z = c_3 \f$ form
 */
template <typename T>
TypedMat<T> fitPlaneLLS (const TypedMat<T> & points);

/** \brief Fit a plane to a given cloud of 3D points using Random Sample Consensus (RANSAC)
 * \param cloud A cloud containing an "xyz" channel of 3D points
 * \param inlier_threshold The maximum point-to-plane distance at which a point will be considered an inlier
 * \return A MatF containing the 4 coefficients of the best fitting plane, 
 * given in \f$ c_0 x + c_1 y + c_2 z = c_3 \f$ form
 */
MatF fitPlaneRANSAC (const Cloud & cloud, float inlier_threshold);

/** \brief Fit a plane to a channel of 3D points using Random Sample Consensus (RANSAC)
 * \param points An n by 3 MatF representing n 3D points
 * \param inlier_threshold The maximum point-to-plane distance at which a point will be considered an inlier
 * \return A MatF containing the 4 coefficients of the best fitting plane, 
 * given in \f$ c_0 x + c_1 y + c_2 z = c_3 \f$ form
 */
template <typename T>
TypedMat<T> fitPlaneRANSAC (const TypedMat<T> & points, float inlier_threshold);

}

#endif
