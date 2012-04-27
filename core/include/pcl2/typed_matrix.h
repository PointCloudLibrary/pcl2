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

/** \file typed_matrix.h 
 * \brief Contains class declarations for TypedMat and ConstTypedMat 
 */

#ifndef PCL2_TYPED_MATRIX_H
#define PCL2_TYPED_MATRIX_H


namespace pcl2
{
template <typename T> class TypedMat;
typedef TypedMat<int> MatI;
typedef TypedMat<float> MatF;
typedef TypedMat<double> MatD;
}

#include "pcl2/matrix.h"

#include <boost/shared_ptr.hpp>

namespace pcl2
{

template <typename T> class SpatialIndex;
template <typename T> class Row;
namespace core
{
  template <typename T> class TypedMatImpl;
}

///////////////////////////////////////////////////////////////////////////////
/** \brief 
 *
 * Extends the Mat class to add type-specific accessor operators
 */
template <typename T>
class TypedMat : public Mat
{
public:
  typedef pcl2::Row<T> Row;

private:
  TypedMat ();

protected:
  /** \brief A shared pointer to the implementation */
  typedef boost::shared_ptr<core::MatImpl> MatImplPtr;

  /** \brief Construct a TypedMat around the provided MatImpl 
   *
   * \throws Throws a BadCastException if the provided MatImpl cannot be cast 
   * to a matrix of the specified type, T.
   * \todo Make this throw an exception instead of an assertion failure
   */
  TypedMat (MatImplPtr matrix);

public:
  /** \brief Construct a TypedMat from a generic Mat.  
   *
   * \throws Throws a BadCastException if the provided matrix cannot be cast to a
   * matrix of the specified type, T.
   * \todo Make this throw an exception instead of an assertion failure
   */
  TypedMat (const Mat & shared_matrix);

  /** \brief Access an element in the matrix
   *
   * \param i The row of the element
   * \param j The column of the element
   * \return A reference to the element in the ith row and jth column
   */
  T & operator () (size_t i, size_t j);

  /** \brief Access an element in the matrix
   *
   * \param i The row of the element
   * \param j The column of the element
   * \return A const reference to the element in the ith row and jth column
   */
  const T & operator () (size_t i, size_t j) const;

  /** \brief Access an element in the matrix
   *
   * \param i The index of the element in row-major order
   * \return A const reference to the ith element
   */
  const T & operator [] (size_t i) const;

  Row operator () (size_t row_index);
  const Row operator () (size_t row_index) const;

  TypedMat operator () (const TypedMat<int> & indices);

  /** \brief Fill in this matrix with the provided scalar value
   * \param value The scalar value that will be copied into every element of this matrix
   */
  void fill (const T & value);

  /** \brief Fill in this matrix with the provided values
   * \param matrix The matrix whose values will be copied into the corresponding elements of this matrix
   * The size of the input matrix (i.e., the source matrix) must be compatible with the
   * matrix being filled (i.e., the destination).
   *
   * \throws IncompatibleSizeException
   */
  void fill (const TypedMat<T> & matrix);

  /** \todo Decide if the << operator is better than a "fill" method; we'll have both for now */
  inline TypedMat<T> & operator << (const T & value) { fill (value); return (*this); }
  inline TypedMat<T> & operator << (const TypedMat<T> & matrix) { fill (matrix); return (*this); }

  /** \brief Perform scalar addition */
  TypedMat
  operator + (const T & operand) const;

  /** \brief Perform scalar subtraction */
  TypedMat
  operator - (const T & operand) const;

  /** \brief Perform scalar multiplication */
  TypedMat
  operator * (const T & operand) const;

  /** \brief Perform scalar division */
  TypedMat
  operator / (const T & operand) const;

  /** \brief Perform element-wise addition */
  TypedMat
  operator + (const TypedMat<T> & operand) const;

  /** \brief Perform element-wise subtraction */
  TypedMat
  operator - (const TypedMat<T> & operand) const;

  /** \brief Perform in-place element-wise addition */
  TypedMat &
  operator += (const TypedMat<T> & operand);

  /** \brief Perform in-place element-wise subtraction */
  TypedMat &
  operator -= (const TypedMat<T> & operand);

  /** \brief Perform in-place element-wise multiplication */
  TypedMat &
  operator *= (const TypedMat<T> & operand);

  /** \brief Perform in-place element-wise division */
  TypedMat &
  operator /= (const TypedMat<T> & operand);

  void
  buildSpatialIndex (const boost::shared_ptr<SpatialIndex<T> > & spatial_index) const;

  //typename SpatialIndex<T>::Ptr
  boost::shared_ptr<SpatialIndex<T> >
  getSpatialIndex () const;

protected:
  /** \brief A shared pointer to the matrix implementation */
  boost::shared_ptr<core::TypedMatImpl<T> > typed_matrix_ptr_;
};


template <typename T> std::ostream&
operator << (std::ostream& out, const pcl2::TypedMat<T> & mat);

}

#endif
