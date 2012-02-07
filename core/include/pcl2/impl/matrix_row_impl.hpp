/** \file matrix_row_impl.hpp
 * \brief Contains class definition for pcl2::MatRowImpl
 */

#ifndef PCL2_MATRIX_ROW_IMPL_HPP
#define PCL2_MATRIX_ROW_IMPL_HPP

#include "pcl2/matrix_row_impl.h"

template <typename T>
pcl2::core::MatRowImpl<T>::MatRowImpl () { assert (false); }

template <typename T>
pcl2::core::MatRowImpl<T>::MatRowImpl (const MatRowImpl & a) { assert (false); }

template <typename T>
pcl2::core::MatRowImpl<T>::MatRowImpl (typename TypedMatImpl<T>::Ptr matrix_ptr, size_t row_index) : 
  matrix_ptr_ (matrix_ptr), 
  row_index_ (row_index)
{}

template <typename T>
pcl2::core::MatImpl::Ptr
pcl2::core::MatRowImpl<T>::copy () const 
{
  MatImpl::Ptr output_matrix_ptr = matrix_ptr_->createNew (1, cols ());
  TypedMatImpl<T> & output_matrix_ref = *boost::dynamic_pointer_cast<TypedMatImpl<T> > (output_matrix_ptr);
  for (size_t i = 0; i < cols (); ++i)
  {
    output_matrix_ref (row_index_, i) = (*matrix_ptr_) (row_index_, i);
  }
 
  return (output_matrix_ptr);
}

template <typename T>
pcl2::core::MatImpl::Ptr
pcl2::core::MatRowImpl<T>::createNew (size_t rows, size_t cols) const 
{ 
  return (matrix_ptr_->createNew (rows, cols));
}

template <typename T>
pcl2::core::MatImpl::Ptr 
pcl2::core::MatRowImpl<T>::createView (const TypedMatImpl<int>::ConstPtr & indices)
{
  assert (false); /// \todo Implement this method!
}

template <typename T>
size_t 
pcl2::core::MatRowImpl<T>::rows () const { return (1); }

template <typename T>
size_t 
pcl2::core::MatRowImpl<T>::cols () const { return (matrix_ptr_->cols ()); }

template <typename T>
void
pcl2::core::MatRowImpl<T>::fill (const MatImpl::ConstPtr & matrix_ptr)
{
  ///\todo Replace this assert with a throw
  assert (matrix_ptr->rows () == 1 && matrix_ptr->cols () == cols ()); 
  typename TypedMatImpl<T>::ConstPtr typed_matrix_ptr = 
    boost::dynamic_pointer_cast<const core::TypedMatImpl<T> > (matrix_ptr);
  for (int i = 0; i < cols (); ++i)
  {
    (*matrix_ptr_) (row_index_, i) = (*typed_matrix_ptr) (0, i);
  }
}

template <typename T>
void 
pcl2::core::MatRowImpl<T>::fill (const T & value)
{
  for (int i = 0; i < cols (); ++i)
  {
    (*matrix_ptr_) (row_index_, i) = value;
  }
}

template <typename T>
T & 
pcl2::core::MatRowImpl<T>::operator () (size_t i, size_t j)
{
  assert (i == 0);
  return ((*matrix_ptr_) (row_index_, j));
}

template <typename T>
const T & 
pcl2::core::MatRowImpl<T>::operator () (size_t i, size_t j) const
{
  assert (i == 0);
  return ((*matrix_ptr_) (row_index_, j));
}

#endif
