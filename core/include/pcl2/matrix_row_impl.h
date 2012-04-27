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

/** \file matrix_row_impl.h 
 * \brief Contains class declaration for pcl2::MatRowImpl
 */

#ifndef PCL2_MATRIX_ROW_IMPL_H
#define PCL2_MATRIX_ROW_IMPL_H

#include "typed_matrix_impl.h"

#include <assert.h>
#include <boost/enable_shared_from_this.hpp>

namespace pcl2
{
namespace core
{

/** \todo Document this class */
template <typename T>
class MatRowImpl : public TypedMatImpl<T>, public boost::enable_shared_from_this<MatRowImpl<T> >
{
public:
  /** \brief A shared pointer to a MatRowImpl */
  typedef boost::shared_ptr<MatRowImpl<T> > Ptr;
  typedef boost::shared_ptr<const MatRowImpl<T> > ConstPtr;

protected:
  MatRowImpl ();
  MatRowImpl (const MatRowImpl & a);

public:
  /** \todo document this */
  MatRowImpl (typename TypedMatImpl<T>::Ptr matrix, size_t row_index);

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

  inline size_t getIndex () const 
  { 
    return (row_index_); 
  }
  
  inline bool hasNext () const 
  {
    return (row_index_ < matrix_ptr_->rows ());
  }

  inline void advance () 
  {
    assert (hasNext ());
    ++row_index_;
  }

protected:
  /** \todo document this */
  typename TypedMatImpl<T>::Ptr matrix_ptr_;

  /** The index of the row being referenced by this MatRowImpl */
  size_t row_index_;

};

} // namespace core
} // namespace pcl2

#endif
