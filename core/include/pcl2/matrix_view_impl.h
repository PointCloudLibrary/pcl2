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

/** \file matrix_view_impl.h 
 * \brief Contains class declaration for pcl2::MatViewImpl
 */

#ifndef PCL2_MATRIX_VIEW_IMPL_H
#define PCL2_MATRIX_VIEW_IMPL_H

#include "typed_matrix_impl.h"

namespace pcl2
{
namespace core
{

/** \brief A "view" into a matrix, appearing to contain only the specified subset
 * and/or permutation of rows from the original matrix.
 *
 * A MatViewImpl wraps a source matrix and a list of indices and creates a new
 * matrix that appears to contain a row for each index in the provided indices, 
 * where the elements in each row of the view correspond to the elements at that
 * row in the source matrix. Note that although a view can be used like a true 
 * matrix, it is not an independent copy of the source matrix.  Thus, any changes
 * made to a view's data will also affect the corresponding values in the source
 * matrix and vice versa.
 */
template <typename T>
class MatViewImpl : public TypedMatImpl<T>, public boost::enable_shared_from_this<MatViewImpl<T> >
{
public:
  /** \brief A shared pointer to a MatViewImpl */
  typedef boost::shared_ptr<MatViewImpl<T> > Ptr;

protected:
  MatViewImpl ();
  MatViewImpl (const MatViewImpl & a);

public:
  /** \brief Create a view into the specified rows of the provided matrix 
   *
   * This constructor creates a view from a source \a matrix and a matrix of 
   * \a indices. The resulting view will contain a row for each row index 
   * defined in  \a indices, and the elements in each row of the view will be 
   * references to the corresponding elements in the original matrix.
   *
   * \param matrix The source matrix (i.e., the matrix that is to be "viewed")
   * \param indices A matrix of integers indexing rows of the source \a matrix.
   */
  MatViewImpl (typename TypedMatImpl<T>::Ptr matrix_ptr, TypedMatImpl<int>::ConstPtr indices_ptr);

  virtual MatImpl::Ptr copy () const;

  virtual MatImpl::Ptr createNew (size_t rows, size_t cols) const;
  virtual MatImpl::Ptr createView (const TypedMatImpl<int>::ConstPtr & indices);

  virtual size_t rows () const;
  virtual size_t cols () const;

  virtual void fill (const TypedMatImpl<T> & matrix);
  virtual void fill (const T & value);

  virtual typename TypedMatImpl<T>::Ptr operator + (const T & operand) const;
  virtual typename TypedMatImpl<T>::Ptr operator - (const T & operand) const;
  virtual typename TypedMatImpl<T>::Ptr operator * (const T & operand) const;
  virtual typename TypedMatImpl<T>::Ptr operator / (const T & operand) const;

  virtual typename TypedMatImpl<T>::Ptr operator + (const TypedMatImpl<T> & operand) const;
  virtual typename TypedMatImpl<T>::Ptr operator - (const TypedMatImpl<T> & operand) const;

  virtual void operator += (const TypedMatImpl<T> & operand);
  virtual void operator -= (const TypedMatImpl<T> & operand);
  virtual void operator *= (const TypedMatImpl<T> & operand);
  virtual void operator /= (const TypedMatImpl<T> & operand);

  virtual T & operator () (size_t i, size_t j);
  virtual const T & operator () (size_t i, size_t j) const;

protected:
  /** The matrix being "viewed" */
  typename TypedMatImpl<T>::Ptr matrix_ptr_;

  /** The list of indices into matrix_ that defines this view */
  TypedMatImpl<int>::ConstPtr indices_ptr_;

};

} // namespace core
} // namespace pcl2

#endif
