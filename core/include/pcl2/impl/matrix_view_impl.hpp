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

/** \file matrix_view_impl.hpp
 * \brief Contains class definition for pcl2::MatViewImpl
 */

#ifndef PCL2_MATRIX_VIEW_IMPL_HPP
#define PCL2_MATRIX_VIEW_IMPL_HPP

#include "pcl2/matrix_view_impl.h"

template <typename T>
pcl2::core::MatViewImpl<T>::MatViewImpl () { assert (false); }

template <typename T>
pcl2::core::MatViewImpl<T>::MatViewImpl (const MatViewImpl & a) { assert (false); }

template <typename T>
pcl2::core::MatViewImpl<T>::MatViewImpl (typename TypedMatImpl<T>::Ptr matrix_ptr, 
                                         TypedMatImpl<int>::ConstPtr indices_ptr) : 
  matrix_ptr_ (matrix_ptr), 
  indices_ptr_ (indices_ptr)
{}

template <typename T>
pcl2::core::MatImpl::Ptr
pcl2::core::MatViewImpl<T>::copy () const 
{
  size_t r = rows ();
  size_t c = cols ();
  MatImpl::Ptr output_matrix_ptr = matrix_ptr_->createNew (r, c);
  TypedMatImpl<T> & output_matrix_ref = *boost::dynamic_pointer_cast<TypedMatImpl<T> > (output_matrix_ptr);
  for (size_t i = 0; i < r; ++i)
  {
    for (size_t j = 0; j < c; ++j)
    {
      output_matrix_ref (i, j) = operator () (i, j);
     }
  }
  
  return (output_matrix_ptr);
}

template <typename T>
pcl2::core::MatImpl::Ptr
pcl2::core::MatViewImpl<T>::createNew (size_t rows, size_t cols) const 
{ 
  return (matrix_ptr_->createNew (rows, cols));
}

template <typename T>
pcl2::core::MatImpl::Ptr 
pcl2::core::MatViewImpl<T>::createView (const TypedMatImpl<int>::ConstPtr & indices)
{
  assert (false); /// \todo Implement this method! (Views of views...)
}

template <typename T>
size_t 
pcl2::core::MatViewImpl<T>::rows () const
{
  return (indices_ptr_->rows ());
}

template <typename T>
size_t 
pcl2::core::MatViewImpl<T>::cols () const
{
  return (matrix_ptr_->cols ());
}

template <typename T>
void
pcl2::core::MatViewImpl<T>::fill (const TypedMatImpl<T> & matrix)
{
  assert (matrix.rows () == rows () && matrix.cols () == cols ());

  for (size_t i = 0; i < rows (); ++i)
  {
    const size_t i2 = (*indices_ptr_) (i, 0);
    for (size_t j = 0; j < cols (); ++j)
    {
      (*matrix_ptr_) (i2, j) = matrix (i, j);
    }
  }
}

template <typename T>
void 
pcl2::core::MatViewImpl<T>::fill (const T & value)
{
  for (size_t i = 0; i < rows (); ++i)
  {
    const size_t i2 = (*indices_ptr_) (i, 0);
    for (size_t j = 0; j < cols (); ++j)
    {
      (*matrix_ptr_) (i2, j) = value;
    }
  }
}

template <typename T>
typename pcl2::core::TypedMatImpl<T>::Ptr
pcl2::core::MatViewImpl<T>::operator + (const T & operand) const
{
  typename TypedMatImpl<T>::Ptr output_matrix_ptr = 
    boost::static_pointer_cast<core::TypedMatImpl<T> > (matrix_ptr_->createNew (rows (), cols ()));

  for (size_t i = 0; i < rows (); ++i)
  {
    const size_t i2 = (*indices_ptr_) (i, 0);
    for (size_t j = 0; j < cols (); ++j)
    {
      (*output_matrix_ptr) (i, j) = (*matrix_ptr_) (i2, j) + operand;
    }
  }
  return (output_matrix_ptr);
}

template <typename T>
typename pcl2::core::TypedMatImpl<T>::Ptr
pcl2::core::MatViewImpl<T>::operator - (const T & operand) const
{
  typename TypedMatImpl<T>::Ptr output_matrix_ptr = 
    boost::static_pointer_cast<core::TypedMatImpl<T> > (matrix_ptr_->createNew (rows (), cols ()));

  for (size_t i = 0; i < rows (); ++i)
  {
    const size_t i2 = (*indices_ptr_) (i, 0);
    for (size_t j = 0; j < cols (); ++j)
    {
      (*output_matrix_ptr) (i, j) = (*matrix_ptr_) (i2, j) - operand;
    }
  }
  return (output_matrix_ptr);
}

template <typename T>
typename pcl2::core::TypedMatImpl<T>::Ptr
pcl2::core::MatViewImpl<T>::operator * (const T & operand) const
{
  typename TypedMatImpl<T>::Ptr output_matrix_ptr = 
    boost::static_pointer_cast<core::TypedMatImpl<T> > (matrix_ptr_->createNew (rows (), cols ()));

  for (size_t i = 0; i < rows (); ++i)
  {
    const size_t i2 = (*indices_ptr_) (i, 0);
    for (size_t j = 0; j < cols (); ++j)
    {
      (*output_matrix_ptr) (i, j) = (*matrix_ptr_) (i2, j) * operand;
    }
  }
  return (output_matrix_ptr);
}

template <typename T>
typename pcl2::core::TypedMatImpl<T>::Ptr
pcl2::core::MatViewImpl<T>::operator / (const T & operand) const
{
  typename TypedMatImpl<T>::Ptr output_matrix_ptr = 
    boost::static_pointer_cast<core::TypedMatImpl<T> > (matrix_ptr_->createNew (rows (), cols ()));

  for (size_t i = 0; i < rows (); ++i)
  {
    const size_t i2 = (*indices_ptr_) (i, 0);
    for (size_t j = 0; j < cols (); ++j)
    {
      (*output_matrix_ptr) (i, j) = (*matrix_ptr_) (i2, j) / operand;
    }
  }
  return (output_matrix_ptr);
}

template <typename T>
typename pcl2::core::TypedMatImpl<T>::Ptr
pcl2::core::MatViewImpl<T>::operator + (const TypedMatImpl<T> & operand) const
{
  assert (operand.rows () == rows () && operand.cols () == cols ());

  typename TypedMatImpl<T>::Ptr output_matrix_ptr = 
    boost::static_pointer_cast<core::TypedMatImpl<T> > (matrix_ptr_->createNew (rows (), cols ()));

  for (size_t i = 0; i < rows (); ++i)
  {
    const size_t i2 = (*indices_ptr_) (i, 0);
    for (size_t j = 0; j < cols (); ++j)
    {
      (*output_matrix_ptr) (i, j) = (*matrix_ptr_) (i2, j) + operand (i, j);
    }
  }
  return (output_matrix_ptr);
}

template <typename T>
typename pcl2::core::TypedMatImpl<T>::Ptr
pcl2::core::MatViewImpl<T>::operator - (const TypedMatImpl<T> & operand) const
{
  assert (operand.rows () == rows () && operand.cols () == cols ());

  typename TypedMatImpl<T>::Ptr output_matrix_ptr = 
    boost::static_pointer_cast<core::TypedMatImpl<T> > (matrix_ptr_->createNew (rows (), cols ()));

  for (size_t i = 0; i < rows (); ++i)
  {
    const size_t i2 = (*indices_ptr_) (i, 0);
    for (size_t j = 0; j < cols (); ++j)
    {
      (*output_matrix_ptr) (i, j) = (*matrix_ptr_) (i2, j) - operand (i, j);
    }
  }
  return (output_matrix_ptr);
}

template <typename T>
void 
pcl2::core::MatViewImpl<T>::operator += (const TypedMatImpl<T> & operand)
{
  assert (operand.rows () == rows () && operand.cols () == cols ());

  for (size_t i = 0; i < rows (); ++i)
  {
    const size_t i2 = (*indices_ptr_) (i, 0);
    for (size_t j = 0; j < cols (); ++j)
    {
      (*matrix_ptr_) (i2, j) += operand (i, j);
    }
  }
}

template <typename T>
void 
pcl2::core::MatViewImpl<T>::operator -= (const TypedMatImpl<T> & operand)
{
  assert (operand.rows () == rows () && operand.cols () == cols ());

  for (size_t i = 0; i < rows (); ++i)
  {
    const size_t i2 = (*indices_ptr_) (i, 0);
    for (size_t j = 0; j < cols (); ++j)
    {
      (*matrix_ptr_) (i2, j) -= operand (i, j);
    }
  }
}

template <typename T>
void 
pcl2::core::MatViewImpl<T>::operator *= (const TypedMatImpl<T> & operand)
{
  assert (operand.rows () == rows () && operand.cols () == cols ());

  for (size_t i = 0; i < rows (); ++i)
  {
    const size_t i2 = (*indices_ptr_) (i, 0);
    for (size_t j = 0; j < cols (); ++j)
    {
      (*matrix_ptr_) (i2, j) *= operand (i, j);
    }
  }
}

template <typename T>
void 
pcl2::core::MatViewImpl<T>::operator /= (const TypedMatImpl<T> & operand)
{
  assert (operand.rows () == rows () && operand.cols () == cols ());

  for (size_t i = 0; i < rows (); ++i)
  {
    const size_t i2 = (*indices_ptr_) (i, 0);
    for (size_t j = 0; j < cols (); ++j)
    {
      (*matrix_ptr_) (i2, j) /= operand (i, j);
    }
  }
}

template <typename T>
T & 
pcl2::core::MatViewImpl<T>::operator () (size_t i, size_t j)
{
  const int i2 = (*indices_ptr_) (i, 0);
  return ((*matrix_ptr_) (i2, j));
}

template <typename T>
const T & 
pcl2::core::MatViewImpl<T>::operator () (size_t i, size_t j) const
{
  const int i2 = (*indices_ptr_) (i, 0);
  return ((*matrix_ptr_) (i2, j));
}

#endif
