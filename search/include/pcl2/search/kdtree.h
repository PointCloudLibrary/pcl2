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

/** \file kdtree.h 
 * \brief Contains class declaration for pcl2::SpatialIndex
 */

#ifndef PCL2_KDTREE_H
#define PCL2_KDTREE_H


#include "pcl2/spatial_index.h"

#include <pcl/kdtree/kdtree_flann.h>

namespace pcl2
{

namespace search
{

template <typename T>
class KDTree : public SpatialIndex<T>
{
public:
  typedef TypedMat<T> MatT;

  virtual void
  buildIndex (const MatT & input)
  {
    assert (input.rows () >= 1);
    assert (input.cols () == 3);

    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_pointcloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl_pointcloud->width = input.rows ();
    pcl_pointcloud->height = 1;
    pcl_pointcloud->is_dense = false;
    pcl_pointcloud->points.resize (input.rows ());
    for (size_t i = 0; i < input.rows (); ++i)
    {
      pcl_pointcloud->points[i].x = input (i, 0);
      pcl_pointcloud->points[i].y = input (i, 1);
      pcl_pointcloud->points[i].z = input (i, 2);
    }
    
    kdtree_.setInputCloud (pcl_pointcloud);
  }

  virtual MatI
  findKNearestNeighbors (const MatT & query, size_t k) const
  {
    // Convert query point
    assert (query.rows () == 1);
    assert (query.cols () == 3);
    pcl::PointXYZ q;
    q.x = query (0, 0);
    q.y = query (0, 1);
    q.z = query (0, 2);

    std::vector<int> indices (k);
    std::vector<float> dists (k);
    k = (size_t) kdtree_.nearestKSearch (q, k, indices, dists);
    assert (k == indices.size ());

    // Convert output
    EigenMat<int> output (k, 1);
    for (size_t i = 0; i < indices.size (); ++i)
      output (i, 0) = indices[i];

    return (output);
  }

  virtual MatI
  findFixedRadiusNeighbors (const MatT & query, float r) const
  {
    // Convert query point
    assert (query.rows () == 1);
    assert (query.cols () == 3);
    pcl::PointXYZ q;
    q.x = query (0, 0);
    q.y = query (0, 1);
    q.z = query (0, 2);

    std::vector<int> indices;
    std::vector<float> dists;
    int k = (size_t) kdtree_.radiusSearch (q, r, indices, dists);

    // Convert output
    EigenMat<int> output (k, 1);
    for (size_t i = 0; i < indices.size (); ++i)
      output (i, 0) = indices[i];

    return (output);
  };

  virtual std::pair<MatI, MatT>
  findKNearestNeighborsAndDistances (const MatT & query, size_t k) const
  {
    assert (false);
  };
  
  virtual std::pair<MatI, MatT>
  findFixedRadiusNeighborsAndDistances (const MatT & query, float r) const
  {
    assert (false);
  };

protected:
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_;
};

}

}

#endif
