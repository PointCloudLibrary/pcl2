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

/** \file matrix_impl.h 
 * \brief Contains class declaration for pcl2::MatImpl
 */

#ifndef PCL2_MATRIX_IMPL_H
#define PCL2_MATRIX_IMPL_H

#include <boost/shared_ptr.hpp>

namespace pcl2
{
namespace core
{

template <typename T> class TypedMatImpl;

/** \brief An abstract class defining the matrix implementation interface. 
 * For internal use only.
 *
 * Subclasses of MatImpl will provide the actual implementation for the Mat
 * class. They are responsible for storing the actual matrix data and providing
 * various methods for operating on it. 
 *
 * \note These MatImpl classes are intended to be used exclusively by Mat
 * and its subclasses. PCL users are not meant to use MatImpl or any of its 
 * subclasses in their code.
 */
class MatImpl
{
public:
  /** \brief A shared pointer to an MatImpl */
  typedef boost::shared_ptr<MatImpl> Ptr;

  /** \brief A const shared pointer to an MatImpl */
  typedef boost::shared_ptr<const MatImpl> ConstPtr;

  /** \brief Create a new copy of this matrix and its data
   *
   * \return A shared pointer to a new copy of this matrix
   */
  virtual MatImpl::Ptr copy () const = 0;

  /** \brief Create a new MatImpl of the given size
   *
   * \param rows The number of rows in the new matrix
   * \param cols The number of cols in the new matrix
   * \return A shared pointer to a new matrix
   */
  virtual MatImpl::Ptr createNew (size_t rows, size_t cols) const = 0;

  /** \brief Create a new MatViewImpl from this MatImpl and the provided 
   * indices 
   *
   * \param indices An matrix of integers indexing rows of this matrix
   * \return A shared pointer to a new MatViewImpl
   */
  virtual MatImpl::Ptr createView (const boost::shared_ptr<const TypedMatImpl<int> > & indices) = 0;

  /** \brief Get the number of rows in the matrix
   *
   * \return The number of rows in the matrix
   */
  virtual size_t rows () const = 0;

  /** \brief Get the number of columns in the matrix
   *
   * \return The number of columns in the matrix
   */
  virtual size_t cols () const = 0;

};

} // namespace core
} // namespace pcl2

#endif
