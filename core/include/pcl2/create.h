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

/** \file create.h
 * \brief Declares functions for creating matrices
 */

#ifndef PCL2_CREATE_H
#define PCL2_CREATE_H

#include "pcl2/eigen_matrix.h"
#include <boost/random.hpp>
#include <ctime>

namespace pcl2
{

template <typename T>
TypedMat<T>
createZeros (size_t rows, size_t cols)
{
  EigenMat<T> output (rows, cols); output << 0; return (output);
}

template <typename T>
TypedMat<T>
createOnes (size_t rows, size_t cols)
{
  EigenMat<T> output (rows, cols); output << 1; return (output);
}

template <typename T>
TypedMat<T>
createIdentity (size_t n)
{
  EigenMat<T> output (n, n);
  output << 0;
  for (size_t i = 0; i < n; ++i)
    output (i, i) = 1;
  return (output); 
}

template <typename T>
TypedMat<T>
createRandom (size_t rows, size_t cols)
{
  unsigned seed = static_cast<unsigned> (std::time(0));
  boost::mt19937 generator (seed);                
  boost::uniform_01<> uniform_dist;    
  boost::variate_generator<boost::mt19937, boost::uniform_01<> > rand (generator, uniform_dist);  

  EigenMat<T> output (rows, cols);
  for (size_t j = 0; j < cols; ++j)
    for (size_t i = 0; i < rows; ++i)
      output (i, j) = rand (); 
  return (output);
}

template <typename T>
TypedMat<T>
createSeries (T start, T end, T step=1)
{
  size_t nr_elements = ceil(1.0*(end - start) / step);
  EigenMat<T> output (nr_elements, 1);

  size_t i = 0;
  for (T x = start; x < end; x += step)
    output (i++, 0) = x;

  return (output);
}

}

#endif
