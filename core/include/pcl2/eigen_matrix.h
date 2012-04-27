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

/** \file eigen_matrix.h 
 * \brief Contains class declarations for EigenMat and ConstEigenMat 
 */

#ifndef PCL2_EIGEN_MATRIX_H
#define PCL2_EIGEN_MATRIX_H

#include "pcl2/typed_matrix.h"

namespace pcl2
{

namespace core
{
  template <typename T> class EigenMatImpl;
}

///////////////////////////////////////////////////////////////////////////////
/** \brief A shared wrapper of an Eigen matrix
 *
 * Extends the TypedMat class to add constructors for creating new Eigen 
 * matrices
 * \see ConstEigenMat
 */
template <typename T>
class EigenMat : public TypedMat<T>
{
protected:
  using TypedMat<T>::matrix_ptr_;

private:
  EigenMat ();

protected:
  /** \brief A shared pointer to the implementation */
  typedef boost::shared_ptr<core::MatImpl> MatImplPtr;

  /** \brief Construct a EigenMat around the provided MatImpl 
   *
   * \throws Throws a BadCastException if the provided MatImpl cannot be cast 
   * to an EigenMatrixImpl.
   * \todo Make this throw an exception instead of an assertion failure
   */
  EigenMat (MatImplPtr matrix);

public:
  /** \brief Construct a EigenMat from a generic Mat.  
   *
   * \throws Throws a BadCastException if the provided matrix cannot be cast to an
   * EigenMatrix.
   * \todo Make this throw an exception instead of an assertion failure
   */
  EigenMat (const Mat & shared_matrix);

  /** \brief Construct a new EigenMat with the specified number of rows and columns 
   *
   * \param rows The number of rows
   * \param cols The number of cols
   */
  EigenMat (size_t rows, size_t cols);

protected:
  /** \brief A shared pointer to the matrix implementation */
  boost::shared_ptr<core::EigenMatImpl<T> > eigen_matrix_ptr_;
};

}

#endif
