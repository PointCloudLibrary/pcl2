/** \file row.hpp
 * \brief Contains class definitions for pcl2::EigenMat and pcl2::ConstEigenMat 
 */

#ifndef PCL2_ROW_HPP
#define PCL2_ROW_HPP

#include "pcl2/row.h"
#include "pcl2/impl/matrix_row_impl.hpp"


template <typename T>
pcl2::Row<T>::Row (boost::shared_ptr<core::TypedMatImpl<T> > matrix_ptr, size_t row) :
  TypedMat<T> (typename core::MatRowImpl<T>::Ptr (new core::MatRowImpl<T> (matrix_ptr, row)))
{
  typedef core::MatRowImpl<T> Impl;
  mat_row_ptr_ = boost::dynamic_pointer_cast<Impl> (matrix_ptr_);
}

template <typename T> int
pcl2::Row<T>::getIndex () const
{
  return (mat_row_ptr_->getIndex ());
}

template <typename T> bool
pcl2::Row<T>::hasNext () const
{
  return (mat_row_ptr_->hasNext ());
}

template <typename T> void
pcl2::Row<T>::advance ()
{
  mat_row_ptr_->advance ();
}

#endif
