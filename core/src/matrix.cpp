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

/** \file matrix.cpp
 * \brief Contains class definition for \ref pcl2::Mat
 */

#include <assert.h>
#include "pcl2/matrix.h"
#include "pcl2/matrix_impl.h"
#include "pcl2/typed_matrix.h"

#include "pcl2/matrix_view_impl.h"

pcl2::Mat::Mat (const core::MatImpl::Ptr matrix_ptr) : matrix_ptr_ (matrix_ptr) {}

pcl2::core::MatImpl::Ptr 
pcl2::Mat::getPtr ()
{
  return (matrix_ptr_);
}

const pcl2::core::MatImpl::Ptr 
pcl2::Mat::getPtr () const
{
  return (matrix_ptr_);
}

pcl2::Mat
pcl2::Mat::copy () const
{
  return (Mat (matrix_ptr_->copy ()));
}

size_t
pcl2::Mat::rows () const
{
  return (matrix_ptr_->rows ());
}

size_t
pcl2::Mat::cols () const
{
  return (matrix_ptr_->cols ());
}

pcl2::Mat
pcl2::Mat::operator () (const TypedMat<int> & indices)
{
  typedef core::TypedMatImpl<int> Impl;
  Impl::ConstPtr idx = boost::dynamic_pointer_cast<const Impl> (indices.getPtr ());
  return (Mat (matrix_ptr_->createView (idx)));
}
