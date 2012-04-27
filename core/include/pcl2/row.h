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

/** \file row.h 
 * \brief Contains class declarations for Row<T> and ConstRow<T> 
 */

#ifndef PCL2_ROW_H
#define PCL2_ROW_H

#include "pcl2/typed_matrix.h"

#include <iostream>
#include <boost/shared_ptr.hpp>
#include <boost/pointer_cast.hpp>

namespace pcl2
{
template <typename T> class TypedMat;
namespace core
{
  template <typename T> class TypedMatImpl;
  template <typename T> class MatRowImpl;
}

///////////////////////////////////////////////////////////////////////////////
/** \brief \todo Write me
 *
 * Extends the TypedMat class to...
 */
template <typename T>
class Row : public TypedMat<T>
{
protected:
  using TypedMat<T>::matrix_ptr_;

private:
  Row ();

protected:
  /** \brief Construct a Row view into the provided TypedMatImpl
   */
  Row (boost::shared_ptr<core::TypedMatImpl<T> > matrix_ptr, size_t row);

public:
  int
  getIndex () const;

  bool
  hasNext () const;

  void
  advance ();

protected:
  /** \brief A shared pointer to the matrix implementation */
  boost::shared_ptr<core::MatRowImpl<T> > mat_row_ptr_;

  friend class TypedMat<T>;
};

template <typename T>
std::ostream& operator << (std::ostream& out, const pcl2::Row<T> & row)
{
  for (size_t i = 0; i < row.cols (); ++i)
    out << row (0, i) << " ";

  return (out);
}

}

#endif
