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

/// \deprecated
template <typename T>
T &
pcl2::TypedMat<T>::operator () (size_t i, size_t j)
{
  return ((*typed_matrix_ptr_) (i, j));
}

/// \deprecated
template <typename T>
const T & 
pcl2::TypedMat<T>::operator () (size_t i, size_t j) const
{
  return ((*typed_matrix_ptr_) (i, j));
}

template <typename T>
pcl2::Row<T>
pcl2::TypedMat<T>::operator () (size_t index)
{
  return (Row (typed_matrix_ptr_, index));
}

template <typename T>
pcl2::TypedMat<T>
pcl2::TypedMat<T>::operator () (const ConstTypedMat<int> & indices)
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


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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

#endif
