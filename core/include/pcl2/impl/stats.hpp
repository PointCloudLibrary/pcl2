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

/** \file stats.hpp
 * \brief Defines functions for computing statistical properties of a matrix
 */

#include "pcl2/stats.h"
#include "pcl2/row.h"
#include "pcl2/eigen_matrix.h"
#include "pcl2/math.h"

template<typename T>
pcl2::TypedMat<T>
pcl2::computeMean (const TypedMat<T> & input)
{
  EigenMat<T> sum = computeSum (input);
  return (sum / input.rows ());
}

template<typename T>
pcl2::TypedMat<T>
pcl2::computeCovariance (const TypedMat<T> & input)
{
  TypedMat<T> mean = computeMean (input);
  return (computeCovariance (input, mean));
}

template<typename T>
pcl2::TypedMat<T>
pcl2::computeCovariance (const TypedMat<T> & input, const TypedMat<T> & mean)
{
  EigenMat<T> unnormalized_cov (input.cols (), input.cols ());
  unnormalized_cov.fill (0);
  for (typename TypedMat<T>::Row row = input (0); row.hasNext (); row.advance ())
  {
    TypedMat<T> demeaned_row = row - mean;
    unnormalized_cov += computeOuterProduct (demeaned_row, demeaned_row);
  }
  return (unnormalized_cov / (input.rows () - 1));
}

template<typename T>
void
pcl2::computeMeanAndCovariance (const TypedMat<T> & input, TypedMat<T> & mean, TypedMat<T> & covariance)
{
  mean = computeMean (input);
  covariance = computeCovariance (input, mean);
}
