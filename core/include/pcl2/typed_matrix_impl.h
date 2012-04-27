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

/** \file typed_matrix_impl.h 
 * \brief Contains class declaration for pcl2::TypedMatImpl
 */

#ifndef PCL2_TYPED_MATRIX_IMPL_H
#define PCL2_TYPED_MATRIX_IMPL_H
#include <iostream>
#include "pcl2/matrix_impl.h"
#include "pcl2/spatial_index.h"

namespace pcl2
{

template <typename T> class SpatialIndex;

namespace core
{

/** \brief An abstract matrix implementation templated on scalar type 
 * (e.g., float, int)
 *
 * The TypedMatImpl class extends the base MatImpl class to add element
 * accessors.  It must be templated on the matrix's scalar type.
 */
template <typename T>
class TypedMatImpl : public MatImpl
{

public:
  /** \brief A shared pointer to a TypedMatImpl */
  typedef boost::shared_ptr<TypedMatImpl<T> > Ptr;

  /** \brief A const shared pointer to a TypedMatImpl */
  typedef boost::shared_ptr<const TypedMatImpl<T> > ConstPtr;

  /** \brief Fill in the values of this matrix using the values from the provided
   * matrix
   * \param matrix_ptr The matrix whose values will be copied into this matrix.
   * The size of the input matrix must be compatible with the matrix being filled.
   *
   * \throws IncompatibleSizeException
   * \see Mat::fill (const Mat & matrix)
   */
  virtual void fill (const TypedMatImpl<T> & matrix) = 0; 

  virtual void fill (const T & value) = 0;

  /** \brief Access an element in the matrix
   * \param i The row of the element
   * \param j The column of the element
   * \return A reference to the element in the ith row and jth column
   */
  virtual T & operator () (size_t i, size_t j) = 0;

  /** \brief Access an element in the matrix
   * \param i The row of the element
   * \param j The column of the element
   * \return A const reference to the element in the ith row and jth column
   */
  virtual const T & operator () (size_t i, size_t j) const = 0;

  /** \brief Perform scalar addition */
  virtual typename TypedMatImpl<T>::Ptr operator + (const T & operand) const = 0;
  /** \brief Perform scalar subtraction */
  virtual typename TypedMatImpl<T>::Ptr operator - (const T & operand) const = 0;
  /** \brief Perform scalar multiplication */
  virtual typename TypedMatImpl<T>::Ptr operator * (const T & operand) const = 0;
  /** \brief Perform scalar division */
  virtual typename TypedMatImpl<T>::Ptr operator / (const T & operand) const = 0;

  /** \brief Perform element-wise addition */
  virtual typename TypedMatImpl<T>::Ptr operator + (const TypedMatImpl<T> & operand) const = 0;
  /** \brief Perform element-wise subtraction */
  virtual typename TypedMatImpl<T>::Ptr operator - (const TypedMatImpl<T> & operand) const = 0;

  /** \brief Perform in-place element-wise addition */
  virtual void operator += (const TypedMatImpl<T> & operand) = 0;
  /** \brief Perform in-place element-wise subtraction */
  virtual void operator -= (const TypedMatImpl<T> & operand) = 0;
  /** \brief Perform in-place element-wise multiplication */
  virtual void operator *= (const TypedMatImpl<T> & operand) = 0;
  /** \brief Perform in-place element-wise division */
  virtual void operator /= (const TypedMatImpl<T> & operand) = 0;

  mutable typename SpatialIndex<T>::Ptr spatial_index_;
};

} // namespace core
} // namespace pcl2

#endif
