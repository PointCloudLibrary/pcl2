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

/** \file eigen_matrix.hpp
 * \brief Contains class definitions for pcl2::EigenMat and pcl2::ConstEigenMat 
 */

#ifndef PCL2_EIGEN_MATRIX_HPP
#define PCL2_EIGEN_MATRIX_HPP

#include "pcl2/eigen_matrix.h"

#include "pcl2/impl/typed_matrix.hpp"
#include "pcl2/impl/eigen_matrix_impl.hpp"

template <typename T>
pcl2::EigenMat<T>::EigenMat (core::MatImpl::Ptr matrix) : TypedMat<T> (matrix)
{
  eigen_matrix_ptr_ = boost::dynamic_pointer_cast<core::EigenMatImpl<T> > (matrix);
  assert (eigen_matrix_ptr_);
}

template <typename T>
pcl2::EigenMat<T>::EigenMat (const Mat & shared_matrix) : TypedMat<T> (shared_matrix)
{
  eigen_matrix_ptr_ = boost::dynamic_pointer_cast<core::EigenMatImpl<T> > (matrix_ptr_);
  assert (eigen_matrix_ptr_);
}

template <typename T>
pcl2::EigenMat<T>::EigenMat (size_t rows, size_t cols) :
  TypedMat<T> (typename core::EigenMatImpl<T>::Ptr (new core::EigenMatImpl<T> (rows, cols)))
{
  eigen_matrix_ptr_ = boost::dynamic_pointer_cast<core::EigenMatImpl<T> > (matrix_ptr_);
  assert (eigen_matrix_ptr_);
}

#endif
