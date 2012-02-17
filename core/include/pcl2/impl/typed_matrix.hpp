/** \file typed_matrix.hpp
 * \brief Contains class definitions for pcl2::TypedMat and pcl2::ConstTypedMat 
 */

#ifndef PCL2_TYPED_MATRIX_HPP
#define PCL2_TYPED_MATRIX_HPP

#include "pcl2/typed_matrix.h"
#include "pcl2/impl/matrix_row_impl.hpp"

template <typename T>
pcl2::TypedMat<T>::TypedMat (core::MatImpl::Ptr matrix) : Mat (matrix)
{
  typed_matrix_ptr_ = boost::dynamic_pointer_cast<core::TypedMatImpl<T> > (matrix);
  assert (typed_matrix_ptr_);
}

template <typename T>
pcl2::TypedMat<T>::TypedMat (const Mat & shared_matrix) : Mat (shared_matrix)
{
  typed_matrix_ptr_ = boost::dynamic_pointer_cast<core::TypedMatImpl<T> > (matrix_ptr_);
  assert (typed_matrix_ptr_);
}

/// \deprecated ?
template <typename T>
T &
pcl2::TypedMat<T>::operator () (size_t i, size_t j)
{
  return ((*typed_matrix_ptr_) (i, j));
}

template <typename T>
const T & 
pcl2::TypedMat<T>::operator () (size_t i, size_t j) const
{
  return ((*typed_matrix_ptr_) (i, j));
}

template <typename T>
const T & 
pcl2::TypedMat<T>::operator [] (size_t i) const
{
  size_t i2 = i % rows ();
  size_t j = i / rows ();
  return ((*typed_matrix_ptr_) (i2, j));
}

template <typename T>
pcl2::Row<T>
pcl2::TypedMat<T>::operator () (size_t index)
{
  return (Row (typed_matrix_ptr_, index));
}

template <typename T>
const pcl2::Row<T>
pcl2::TypedMat<T>::operator () (size_t index) const
{
  /// \todo: Add a "const" flag to the constructor
  return (Row (typed_matrix_ptr_, index));
}

template <typename T>
pcl2::TypedMat<T>
pcl2::TypedMat<T>::operator () (const TypedMat<int> & indices)
{
  typedef core::TypedMatImpl<int> Impl;
  Impl::ConstPtr idx = boost::dynamic_pointer_cast<const Impl> (indices.getPtr ());
  return (TypedMat (matrix_ptr_->createView (idx)));
}

template <typename T>
void 
pcl2::TypedMat<T>::fill (const T & value)
{
  typed_matrix_ptr_->fill (value);
}

template <typename T>
void 
pcl2::TypedMat<T>::fill (const TypedMat<T> & matrix)
{
  typed_matrix_ptr_->fill (*boost::static_pointer_cast<const core::TypedMatImpl<T> > (matrix.getPtr ()));
}

template <typename T>
pcl2::TypedMat<T>
pcl2::TypedMat<T>::operator + (const T & operand) const
{
  return (TypedMat<T>((*typed_matrix_ptr_) + operand));
}

template <typename T>
pcl2::TypedMat<T>
pcl2::TypedMat<T>::operator - (const T & operand) const
{
  return (TypedMat<T>((*typed_matrix_ptr_) - operand));
}

template <typename T>
pcl2::TypedMat<T>
pcl2::TypedMat<T>::operator * (const T & operand) const
{
  return (TypedMat<T>((*typed_matrix_ptr_) * operand));
}

template <typename T>
pcl2::TypedMat<T>
pcl2::TypedMat<T>::operator / (const T & operand) const
{
  return (TypedMat<T>((*typed_matrix_ptr_) / operand));
}

template <typename T>
pcl2::TypedMat<T>
pcl2::TypedMat<T>::operator + (const TypedMat<T> & operand) const
{
  typedef core::TypedMatImpl<T> Impl;
  typename Impl::ConstPtr operand_ptr = boost::static_pointer_cast<const Impl> (operand.getPtr ());
  return (TypedMat<T> ((*typed_matrix_ptr_) + (*operand_ptr)));
}

template <typename T>
pcl2::TypedMat<T>
pcl2::TypedMat<T>::operator - (const TypedMat<T> & operand) const
{
  typedef core::TypedMatImpl<T> Impl;
  typename Impl::ConstPtr operand_ptr = boost::static_pointer_cast<const Impl> (operand.getPtr ());
  return (TypedMat<T> ((*typed_matrix_ptr_) - (*operand_ptr)));
}

template <typename T>
pcl2::TypedMat<T> &
pcl2::TypedMat<T>::operator += (const TypedMat<T> & operand)
{
  typedef core::TypedMatImpl<T> Impl;
  typename Impl::ConstPtr operand_ptr = boost::static_pointer_cast<const Impl> (operand.getPtr ());
  (*typed_matrix_ptr_) += (*operand_ptr);

  return (*this);
}

template <typename T>
pcl2::TypedMat<T> &
pcl2::TypedMat<T>::operator -= (const TypedMat<T> & operand)
{
  typedef core::TypedMatImpl<T> Impl;
  typename Impl::ConstPtr operand_ptr = boost::static_pointer_cast<const Impl> (operand.getPtr ());
  (*typed_matrix_ptr_) -= (*operand_ptr);

  return (*this);
}

template <typename T>
pcl2::TypedMat<T> &
pcl2::TypedMat<T>::operator *= (const TypedMat<T> & operand)
{
  typedef core::TypedMatImpl<T> Impl;
  typename Impl::ConstPtr operand_ptr = boost::static_pointer_cast<const Impl> (operand.getPtr ());
  (*typed_matrix_ptr_) *= (*operand_ptr);

  return (*this);
}

template <typename T>
pcl2::TypedMat<T> &
pcl2::TypedMat<T>::operator /= (const TypedMat<T> & operand)
{
  typedef core::TypedMatImpl<T> Impl;
  typename Impl::ConstPtr operand_ptr = boost::static_pointer_cast<const Impl> (operand.getPtr ());
  (*typed_matrix_ptr_) /= (*operand_ptr);

  return (*this);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
template <typename T>
pcl2::ConstTypedMat<T>::ConstTypedMat (core::MatImpl::Ptr matrix) : Mat (matrix)
{
  typed_matrix_ptr_ = boost::dynamic_pointer_cast<core::TypedMatImpl<T> > (matrix);
  assert (typed_matrix_ptr_);
}

template <typename T>
pcl2::ConstTypedMat<T>::ConstTypedMat (const Mat & shared_matrix) : Mat (shared_matrix)
{
  typed_matrix_ptr_ = boost::dynamic_pointer_cast<core::TypedMatImpl<T> > (matrix_ptr_);
  assert (typed_matrix_ptr_);
}

template <typename T>
const T & 
pcl2::ConstTypedMat<T>::operator () (size_t i, size_t j) const
{
  return ((*typed_matrix_ptr_) (i, j));
}

template <typename T>
pcl2::ConstRow<T>
pcl2::ConstTypedMat<T>::operator () (size_t index) const
{
  return (Row (typed_matrix_ptr_, index));
}

template <typename T>
pcl2::ConstTypedMat<T>
pcl2::ConstTypedMat<T>::operator () (const ConstTypedMat<int> & indices)
{
  typedef core::TypedMatImpl<int> Impl;
  Impl::ConstPtr idx = boost::dynamic_pointer_cast<const Impl> (indices.getPtr ());
  return (ConstTypedMat (matrix_ptr_->createView (idx)));
}
*/
#endif
