/**
 * Software License Agreement (BSD License)
 * 
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2009-2011, Willow Garage, Inc.
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

/** \file spatial_index.h 
 * \brief Contains class declaration for pcl2::SpatialIndex
 */

#ifndef PCL2_SPATIAL_INDEX_H
#define PCL2_SPATIAL_INDEX_H

// see http://en.wikipedia.org/wiki/Spatial_database#Spatial_Index

#include "pcl2/typed_matrix.h"
#include <boost/shared_ptr.hpp>

namespace pcl2
{

template <typename T>
class SpatialIndex
{
public:
  typedef TypedMat<T> MatT;
  typedef boost::shared_ptr<pcl2::SpatialIndex<T> > Ptr;

  virtual void
  buildIndex (const MatT & input) = 0;

  virtual MatI
  findKNearestNeighbors (const MatT & query, size_t k) const = 0;

  virtual MatI
  findFixedRadiusNeighbors (const MatT & query, float r) const = 0;

  virtual std::pair<MatI, MatT>
  findKNearestNeighborsAndDistances (const MatT & query, size_t k) const = 0;
  
  virtual std::pair<MatI, MatT>
  findFixedRadiusNeighborsAndDistances (const MatT & query, float r) const = 0;
};

}

#endif
