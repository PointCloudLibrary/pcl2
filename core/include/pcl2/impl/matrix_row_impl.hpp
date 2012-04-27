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

/** \file matrix_row_impl.hpp
 * \brief Contains class definition for pcl2::MatRowImpl
 */

#ifndef PCL2_MATRIX_ROW_IMPL_HPP
#define PCL2_MATRIX_ROW_IMPL_HPP

#include "pcl2/matrix_row_impl.h"

template <typename T>
pcl2::core::MatRowImpl<T>::MatRowImpl () { assert (false); }

template <typename T>
pcl2::core::MatRowImpl<T>::MatRowImpl (const MatRowImpl & a) { assert (false); }

template <typename T>
pcl2::core::MatRowImpl<T>::MatRowImpl (typename TypedMatImpl<T>::Ptr matrix_ptr, size_t row_index) : 
  matrix_ptr_ (matrix_ptr), 
  row_index_ (row_index)
{}

template <typename T>
pcl2::core::MatImpl::Ptr
pcl2::core::MatRowImpl<T>::copy () const 
{
  MatImpl::Ptr output_matrix_ptr = matrix_ptr_->createNew (1, cols ());
  TypedMatImpl<T> & output_matrix_ref = *boost::static_pointer_cast<TypedMatImpl<T> > (output_matrix_ptr);
  for (size_t i = 0; i < cols (); ++i)
  {
    output_matrix_ref (0, i) = (*matrix_ptr_) (row_index_, i);
  }
 
  return (output_matrix_ptr);
}

template <typename T>
pcl2::core::MatImpl::Ptr
pcl2::core::MatRowImpl<T>::createNew (size_t rows, size_t cols) const 
{ 
  return (matrix_ptr_->createNew (rows, cols));
}

template <typename T>
pcl2::core::MatImpl::Ptr 
pcl2::core::MatRowImpl<T>::createView (const TypedMatImpl<int>::ConstPtr & indices)
{
  assert (false); /// \todo Implement this method!
}

template <typename T>
size_t 
pcl2::core::MatRowImpl<T>::rows () const { return (1); }

template <typename T>
size_t 
pcl2::core::MatRowImpl<T>::cols () const { return (matrix_ptr_->cols ()); }

template <typename T>
void
pcl2::core::MatRowImpl<T>::fill (const TypedMatImpl<T> & matrix)
{
  ///\todo Replace this assert with a throw
  assert (matrix.rows () == 1 && matrix.cols () == cols ()); 
  for (size_t i = 0; i < cols (); ++i)
  {
    (*matrix_ptr_) (row_index_, i) = matrix (0, i);
  }
}

template <typename T>
void 
pcl2::core::MatRowImpl<T>::fill (const T & value)
{
  for (size_t i = 0; i < cols (); ++i)
  {
    (*matrix_ptr_) (row_index_, i) = value;
  }
}

template <typename T>
typename pcl2::core::TypedMatImpl<T>::Ptr
pcl2::core::MatRowImpl<T>::operator + (const T & operand) const
{
  typename TypedMatImpl<T>::Ptr output_matrix_ptr = 
    boost::static_pointer_cast<core::TypedMatImpl<T> > (matrix_ptr_->createNew (1, cols ()));
  for (size_t i = 0; i < cols (); ++i)
  {
    (*output_matrix_ptr) (0, i) = (*matrix_ptr_) (row_index_, i) + operand;
  }
  return (output_matrix_ptr);
}

template <typename T>
typename pcl2::core::TypedMatImpl<T>::Ptr
pcl2::core::MatRowImpl<T>::operator - (const T & operand) const
{
  typename TypedMatImpl<T>::Ptr output_matrix_ptr = 
    boost::static_pointer_cast<core::TypedMatImpl<T> > (matrix_ptr_->createNew (1, cols ()));
  for (size_t i = 0; i < cols (); ++i)
  {
    (*output_matrix_ptr) (0, i) = (*matrix_ptr_) (row_index_, i) - operand;
  }
  return (output_matrix_ptr);
}

template <typename T>
typename pcl2::core::TypedMatImpl<T>::Ptr
pcl2::core::MatRowImpl<T>::operator * (const T & operand) const
{
  typename TypedMatImpl<T>::Ptr output_matrix_ptr = 
    boost::static_pointer_cast<core::TypedMatImpl<T> > (matrix_ptr_->createNew (1, cols ()));
  for (size_t i = 0; i < cols (); ++i)
  {
    (*output_matrix_ptr) (0, i) = (*matrix_ptr_) (row_index_, i) * operand;
  }
  return (output_matrix_ptr);
}

template <typename T>
typename pcl2::core::TypedMatImpl<T>::Ptr
pcl2::core::MatRowImpl<T>::operator / (const T & operand) const
{
  typename TypedMatImpl<T>::Ptr output_matrix_ptr = 
    boost::static_pointer_cast<core::TypedMatImpl<T> > (matrix_ptr_->createNew (1, cols ()));
  for (size_t i = 0; i < cols (); ++i)
  {
    (*output_matrix_ptr) (0, i) = (*matrix_ptr_) (row_index_, i) / operand;
  }
  return (output_matrix_ptr);
}

template <typename T>
typename pcl2::core::TypedMatImpl<T>::Ptr
pcl2::core::MatRowImpl<T>::operator + (const TypedMatImpl<T> & operand) const
{
  typename TypedMatImpl<T>::Ptr output_matrix_ptr = 
    boost::static_pointer_cast<core::TypedMatImpl<T> > (matrix_ptr_->createNew (1, cols ()));
  for (size_t i = 0; i < cols (); ++i)
  {
    (*output_matrix_ptr) (0, i) = (*matrix_ptr_) (row_index_, i) + operand (0, i);
  }
  return (output_matrix_ptr);
}

template <typename T>
typename pcl2::core::TypedMatImpl<T>::Ptr
pcl2::core::MatRowImpl<T>::operator - (const TypedMatImpl<T> & operand) const
{
  typename TypedMatImpl<T>::Ptr output_matrix_ptr = 
    boost::static_pointer_cast<core::TypedMatImpl<T> > (matrix_ptr_->createNew (1, cols ()));
  for (size_t i = 0; i < cols (); ++i)
  {
    (*output_matrix_ptr) (0, i) = (*matrix_ptr_) (row_index_, i) - operand (0, i);
  }
  return (output_matrix_ptr);
}

template <typename T>
void 
pcl2::core::MatRowImpl<T>::operator += (const TypedMatImpl<T> & operand)
{
  assert (operand.rows () == 1);
  assert (operand.cols () == cols ());
  for (size_t i = 0; i < cols (); ++i)
  {
    (*matrix_ptr_) (row_index_, i) += operand (0, i);
  }
}

template <typename T>
void 
pcl2::core::MatRowImpl<T>::operator -= (const TypedMatImpl<T> & operand)
{
  assert (operand.rows () == 1);
  assert (operand.cols () == cols ());
  for (size_t i = 0; i < cols (); ++i)
  {
    (*matrix_ptr_) (row_index_, i) -= operand (0, i);
  }
}

template <typename T>
void 
pcl2::core::MatRowImpl<T>::operator *= (const TypedMatImpl<T> & operand)
{
  assert (operand.rows () == 1);
  assert (operand.cols () == cols ());
  for (size_t i = 0; i < cols (); ++i)
  {
    (*matrix_ptr_) (row_index_, i) *= operand (0, i);
  }
}

template <typename T>
void 
pcl2::core::MatRowImpl<T>::operator /= (const TypedMatImpl<T> & operand)
{
  assert (operand.rows () == 1);
  assert (operand.cols () == cols ());
  for (size_t i = 0; i < cols (); ++i)
  {
    (*matrix_ptr_) (row_index_, i) /= operand (0, i);
  }
}

template <typename T>
T & 
pcl2::core::MatRowImpl<T>::operator () (size_t i, size_t j)
{
  assert (i == 0);
  return ((*matrix_ptr_) (row_index_, j));
}

template <typename T>
const T & 
pcl2::core::MatRowImpl<T>::operator () (size_t i, size_t j) const
{
  assert (i == 0);
  return ((*matrix_ptr_) (row_index_, j));
}

#endif
