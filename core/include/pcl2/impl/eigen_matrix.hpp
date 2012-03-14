/** \file eigen_matrix.hpp
 * \brief Contains class definitions for pcl2::EigenMat and pcl2::ConstEigenMat 
 */

#ifndef PCL2_EIGEN_MATRIX_HPP
#define PCL2_EIGEN_MATRIX_HPP

#include "pcl2/eigen_matrix.h"

#include "pcl2/impl/typed_matrix.hpp"
#include "pcl2/impl/eigen_matrix_impl.hpp"

template <typename T>
pcl2::EigenMat<T>::EigenMat (core::MatImpl::Ptr matrix) : TypedMat<T> (matrix)
{
  eigen_matrix_ptr_ = boost::dynamic_pointer_cast<core::EigenMatImpl<T> > (matrix);
  assert (eigen_matrix_ptr_);
}

template <typename T>
pcl2::EigenMat<T>::EigenMat (const Mat & shared_matrix) : TypedMat<T> (shared_matrix)
{
  eigen_matrix_ptr_ = boost::dynamic_pointer_cast<core::EigenMatImpl<T> > (matrix_ptr_);
  assert (eigen_matrix_ptr_);
}

template <typename T>
pcl2::EigenMat<T>::EigenMat (size_t rows, size_t cols) :
  TypedMat<T> (typename core::EigenMatImpl<T>::Ptr (new core::EigenMatImpl<T> (rows, cols)))
{
  eigen_matrix_ptr_ = boost::dynamic_pointer_cast<core::EigenMatImpl<T> > (matrix_ptr_);
  assert (eigen_matrix_ptr_);
}

#endif
