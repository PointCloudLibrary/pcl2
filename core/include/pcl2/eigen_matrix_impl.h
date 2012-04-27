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

/** \file eigen_matrix_impl.h 
 * \brief Contains class declaration for pcl2::EigenMatImpl
 */

#ifndef PCL2_EIGEN_MATRIX_IMPL_H
#define PCL2_EIGEN_MATRIX_IMPL_H

#include "pcl2/eigen_matrix.h"

#include "pcl2/matrix_impl.h"
#include "pcl2/typed_matrix_impl.h"

#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <Eigen/Core>

namespace pcl2
{

template <typename T> class EigenMat;
template <typename T> class ConstEigenMat;

namespace core
{

/** \brief The implementation for a shared Eigen matrix */
template<typename T>
class EigenMatImpl : public TypedMatImpl<T>, public boost::enable_shared_from_this<EigenMatImpl<T> > 
{
public:
  /** \brief a shared pointer to a EigenMatImpl */
  typedef boost::shared_ptr<EigenMatImpl<T> > Ptr;

  /** \brief a const shared pointer to a EigenMatImpl */
  typedef boost::shared_ptr<const EigenMatImpl<T> > ConstPtr;

private:
  /** \brief This constructor is disabled */
  EigenMatImpl ();

  /** \brief The copy constructor is disabled */
  EigenMatImpl (const EigenMatImpl<T> & f);

protected:
  /** \brief Construct a new EigenMatImpl with the specified number of rows and columns 
   *
   * \param rows The number of rows
   * \param cols The number of cols
   */
  EigenMatImpl (size_t rows, size_t cols);

public:
  virtual MatImpl::Ptr copy () const;

  /** \brief Create a new EigenMatImpl of the given size 
   *
   * \param rows The number of rows in the new matrix
   * \param cols The number of cols in the new matrix
   * \return A shared pointer to a new matrix
   */
  virtual MatImpl::Ptr createNew (size_t rows, size_t cols) const;

  /** \brief Create a new MatViewImpl from this EigenMatImpl and the 
   * provided indices 
   *
   * \param indices A matrix of integers indexing rows of this matrix
   * \return A shared pointer to a new MatViewImpl
   */
  virtual MatImpl::Ptr createView (const typename TypedMatImpl<int>::ConstPtr & indices);

  virtual size_t rows () const;
  virtual size_t cols () const;

  virtual void fill (const TypedMatImpl<T> & matrix_ptr);
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

  /** \brief Access an element in the matrix
   * \param i The row of the element
   * \param j The column of the element
   * \return A reference to the element in the ith row and jth column
   */
  virtual T & operator () (size_t i, size_t j);

  /** \brief Access an element in the matrix
   * \param i The row of the element
   * \param j The column of the element
   * \return A const reference to the element in the ith row and jth column
   */
  virtual const T & operator () (size_t i, size_t j) const;

protected:

  /** \brief The number of rows */
  size_t rows_;

  /** \brief The number of columns */
  size_t cols_;

  /** \brief The matrix data (stored in column major form) */
  Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> data_;

  friend class pcl2::EigenMat<T>;
  friend class pcl2::ConstEigenMat<T>;
};

} // namespace core
} // namespace pcl2

#endif
