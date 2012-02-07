/** \file eigen_matrix_impl.cpp
 * \brief Contains class definition for pcl2::core::EigenMatImpl
 */

#ifndef PCL2_EIGEN_MATRIX_IMPL_HPP
#define PCL2_EIGEN_MATRIX_IMPL_HPP

#include "pcl2/eigen_matrix_impl.h"
#include "pcl2/impl/matrix_view_impl.hpp"

template <typename T>
pcl2::core::EigenMatImpl<T>::EigenMatImpl () { assert (false); }

template <typename T>
pcl2::core::EigenMatImpl<T>::EigenMatImpl (const EigenMatImpl & f) { assert (false); }

template <typename T>
pcl2::core::EigenMatImpl<T>::EigenMatImpl (size_t rows, size_t cols) : 
  rows_ (rows), cols_ (cols), data_ (rows, cols)
{
}

template <typename T>
pcl2::core::MatImpl::Ptr 
pcl2::core::EigenMatImpl<T>::copy () const
{
  EigenMatImpl<T>::Ptr matrix_copy (new EigenMatImpl (rows_, cols_));
  matrix_copy->data_ = data_;
  return (matrix_copy);
}

template <typename T>
pcl2::core::MatImpl::Ptr 
pcl2::core::EigenMatImpl<T>::createNew (size_t i, size_t j) const
{
  return (Ptr (new EigenMatImpl (i, j)));
}

template <typename T>
pcl2::core::MatImpl::Ptr 
pcl2::core::EigenMatImpl<T>::createView (const TypedMatImpl<int>::ConstPtr & indices)
{
  return (MatImpl::Ptr (new MatViewImpl<T> (this->shared_from_this (), indices)));
}

template <typename T>
size_t 
pcl2::core::EigenMatImpl<T>::rows () const
{
  return (rows_);
}

template <typename T>
size_t 
pcl2::core::EigenMatImpl<T>::cols () const
{
  return (cols_);
}

template <typename T>
void 
pcl2::core::EigenMatImpl<T>::fill (const MatImpl::ConstPtr & matrix_ptr)
{
  ///\todo Write this method!
  assert (false);

  // Check to make sure the sizes are compatible
  /// \todo Replace these asserts with the appropriate if-statements and throws
  assert (matrix_ptr->rows () == rows_);
  assert (matrix_ptr->cols () == cols_);

  // Cast the incoming pointer to a typed matrix
  typedef core::TypedMatImpl<T> Impl;
  typename Impl::ConstPtr typed_matrix_ptr = boost::dynamic_pointer_cast<const Impl> (matrix_ptr);
  assert (typed_matrix_ptr);

  for (int j = 0; j < cols_; ++j)
    for (int i = 0; i < rows_; ++i)
      data_ (i, j) = (*typed_matrix_ptr) (i,j);
}

template <typename T>
void 
pcl2::core::EigenMatImpl<T>::fill (const T & value)
{
  // Assign the given value to every element of the data matrix
  data_ = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>::Constant(rows_, cols_, value);
}

template <typename T>
T & 
pcl2::core::EigenMatImpl<T>::operator () (size_t i, size_t j)
{
  return (data_ (i, j));
}

template <typename T>
const T & 
pcl2::core::EigenMatImpl<T>::operator () (size_t i, size_t j) const
{
  return (data_ (i, j));
}

#endif
