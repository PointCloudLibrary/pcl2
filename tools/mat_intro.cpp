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

#include <iostream>
#include "pcl2/matrix.h"
#include "pcl2/eigen_matrix.h"
#include "pcl2/row.h"
#include "pcl2/math.h"
#include "pcl2/stats.h"
#include "pcl2/io/io.h"
#include "pcl2/search/neighbors.h"
#include "pcl2/registration/fit.h"
#include "pcl2/create.h"

int 
main (int argc, char ** argv)
{
  // Create an N by M matrix of floats
  int n = 8;
  int m = 3;
  pcl2::EigenMat<float> mat = pcl2::EigenMat<float> (n, m);

  // By default, the values of a matrix are uninitialized
  // We can change all the values in the array using 'fill' or the '<<' operator
  mat << 10.0;

  // We can access its elements with the () operator 
  // (it's a matrix, so we follow mathematical convention: rows by columns)
  int i = 1;
  int j = 2;
  std::cout << "The element stored at row " << i+1 << " and column " << j+1 << " is: ";
  std::cout << mat (i, j) << std::endl;
  
  // We can also iterate over the rows and fill in each row individually
  for (pcl2::MatF::Row row_i = mat (0); row_i.hasNext (); row_i.advance ())
    row_i << row_i.getIndex () * 100;

  std::cout << "mat = " << std::endl 
            << mat << std::endl << std::endl;
  
  // We can create special matrices using the functions declared in create.h
  pcl2::MatF ones = pcl2::createOnes<float> (2, 3);
  pcl2::MatF zeros = pcl2::createZeros<float> (1, 5);
  pcl2::MatF eye = pcl2::createIdentity<float> (3);
  pcl2::MatF rand = pcl2::createRandom<float> (2, 4);

  std::cout << "ones = " << std::endl << ones << std::endl;
  std::cout << "zeros = " << std::endl << zeros << std::endl;
  std::cout << "eye = " << std::endl << eye << std::endl;
  std::cout << "rand = " << std::endl << rand << std::endl;

  // A "series" is a useful kind of matrix.  It creates a vector of sequential elements
  // starting with a value, x0, and increasing in increments of d, up to---but not including-- the value, xN
  //   S = [x0, x0+d, x0+2d, x0+3d, ..., xN)
  pcl2::MatI idx = pcl2::createSeries (0, 8, 2); // will contain: 0, 2, 4, 6 (but not 8!)

  std::cout << "idx = " << std::endl << idx << std::endl;

  // You can use a vector indices to create a "view" into another matrix.
  // The view will contain a subset of the the original matrix's rows ---
  // one for each element of the provided index vector of the original matrix.
  // The view can be operated on just like any other matrix, 
  // but changes to the view will affect the original

  pcl2::MatF view = mat (idx);
  std::cout << "view of mat [mat (idx)] = " << std::endl << view << std::endl;
  view << pcl2::createRandom<float> (view.rows (), view.cols ());

  std::cout << "After filling the view with random values, view = " << std::endl
            << view << std::endl << std::endl
            << "and mat = " << std::endl 
            << mat << std::endl << std::endl;

  // Now we'll perform a few operations on matrices
  std::cout << "The mean and covariance of mat: " << std::endl;
  std::cout << pcl2::computeMean (mat) << std::endl;
  std::cout << pcl2::computeCovariance (mat) << std::endl;

  // This works on views, too
  std::cout << pcl2::computeMean (mat (idx)) << std::endl;
  std::cout << pcl2::computeCovariance (mat (idx)) << std::endl;

  std::cout << "Cumulative sums and products:" << std::endl << std::endl;

  mat << 10.0;
  mat = pcl2::computeCumulativeSum (mat);
  std::cout << mat << std::endl;

  mat.fill (2.0);
  std::cout << pcl2::computeCumulativeProduct (mat) << std::endl;

  std::cout << "Outer sums and products:" << std::endl << std::endl;

  mat << pcl2::computeOuterSum (pcl2::createSeries<float> (0, 80, 10), pcl2::createSeries<float> (0, 3));
  std::cout << mat << std::endl << std::endl;

  pcl2::MatF vec = pcl2::createSeries<float> (0.0, 3.0);
  pcl2::MatF mat3x3 = pcl2::computeOuterProduct (vec, vec);
  mat (pcl2::createSeries (3, 6)) << mat3x3;
  std::cout << mat << std::endl << std::endl;


  return (0);
}
