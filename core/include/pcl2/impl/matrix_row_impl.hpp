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
  TypedMatImpl<T> & output_matrix_ref = *boost::static_pointer_cast<TypedMatImpl<T> > (output_matrix_ptr);
  for (size_t i = 0; i < cols (); ++i)
  {
    output_matrix_ref (0, i) = (*matrix_ptr_) (row_index_, i);
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
pcl2::core::MatRowImpl<T>::fill (const TypedMatImpl<T> & matrix)
{
  ///\todo Replace this assert with a throw
  assert (matrix.rows () == 1 && matrix.cols () == cols ()); 
  for (size_t i = 0; i < cols (); ++i)
  {
    (*matrix_ptr_) (row_index_, i) = matrix (0, i);
  }
}

template <typename T>
void 
pcl2::core::MatRowImpl<T>::fill (const T & value)
{
  for (size_t i = 0; i < cols (); ++i)
  {
    (*matrix_ptr_) (row_index_, i) = value;
  }
}

template <typename T>
typename pcl2::core::TypedMatImpl<T>::Ptr
pcl2::core::MatRowImpl<T>::operator + (const T & operand) const
{
  typename TypedMatImpl<T>::Ptr output_matrix_ptr = 
    boost::static_pointer_cast<core::TypedMatImpl<T> > (matrix_ptr_->createNew (1, cols ()));
  for (size_t i = 0; i < cols (); ++i)
  {
    (*output_matrix_ptr) (0, i) = (*matrix_ptr_) (row_index_, i) + operand;
  }
  return (output_matrix_ptr);
}

template <typename T>
typename pcl2::core::TypedMatImpl<T>::Ptr
pcl2::core::MatRowImpl<T>::operator - (const T & operand) const
{
  typename TypedMatImpl<T>::Ptr output_matrix_ptr = 
    boost::static_pointer_cast<core::TypedMatImpl<T> > (matrix_ptr_->createNew (1, cols ()));
  for (size_t i = 0; i < cols (); ++i)
  {
    (*output_matrix_ptr) (0, i) = (*matrix_ptr_) (row_index_, i) - operand;
  }
  return (output_matrix_ptr);
}

template <typename T>
typename pcl2::core::TypedMatImpl<T>::Ptr
pcl2::core::MatRowImpl<T>::operator * (const T & operand) const
{
  typename TypedMatImpl<T>::Ptr output_matrix_ptr = 
    boost::static_pointer_cast<core::TypedMatImpl<T> > (matrix_ptr_->createNew (1, cols ()));
  for (size_t i = 0; i < cols (); ++i)
  {
    (*output_matrix_ptr) (0, i) = (*matrix_ptr_) (row_index_, i) * operand;
  }
  return (output_matrix_ptr);
}

template <typename T>
typename pcl2::core::TypedMatImpl<T>::Ptr
pcl2::core::MatRowImpl<T>::operator / (const T & operand) const
{
  typename TypedMatImpl<T>::Ptr output_matrix_ptr = 
    boost::static_pointer_cast<core::TypedMatImpl<T> > (matrix_ptr_->createNew (1, cols ()));
  for (size_t i = 0; i < cols (); ++i)
  {
    (*output_matrix_ptr) (0, i) = (*matrix_ptr_) (row_index_, i) / operand;
  }
  return (output_matrix_ptr);
}

template <typename T>
typename pcl2::core::TypedMatImpl<T>::Ptr
pcl2::core::MatRowImpl<T>::operator + (const TypedMatImpl<T> & operand) const
{
  typename TypedMatImpl<T>::Ptr output_matrix_ptr = 
    boost::static_pointer_cast<core::TypedMatImpl<T> > (matrix_ptr_->createNew (1, cols ()));
  for (size_t i = 0; i < cols (); ++i)
  {
    (*output_matrix_ptr) (0, i) = (*matrix_ptr_) (row_index_, i) + operand (0, i);
  }
  return (output_matrix_ptr);
}

template <typename T>
typename pcl2::core::TypedMatImpl<T>::Ptr
pcl2::core::MatRowImpl<T>::operator - (const TypedMatImpl<T> & operand) const
{
  typename TypedMatImpl<T>::Ptr output_matrix_ptr = 
    boost::static_pointer_cast<core::TypedMatImpl<T> > (matrix_ptr_->createNew (1, cols ()));
  for (size_t i = 0; i < cols (); ++i)
  {
    (*output_matrix_ptr) (0, i) = (*matrix_ptr_) (row_index_, i) - operand (0, i);
  }
  return (output_matrix_ptr);
}

template <typename T>
void 
pcl2::core::MatRowImpl<T>::operator += (const TypedMatImpl<T> & operand)
{
  assert (operand.rows () == 1);
  assert (operand.cols () == cols ());
  for (size_t i = 0; i < cols (); ++i)
  {
    (*matrix_ptr_) (row_index_, i) += operand (0, i);
  }
}

template <typename T>
void 
pcl2::core::MatRowImpl<T>::operator -= (const TypedMatImpl<T> & operand)
{
  assert (operand.rows () == 1);
  assert (operand.cols () == cols ());
  for (size_t i = 0; i < cols (); ++i)
  {
    (*matrix_ptr_) (row_index_, i) -= operand (0, i);
  }
}

template <typename T>
void 
pcl2::core::MatRowImpl<T>::operator *= (const TypedMatImpl<T> & operand)
{
  assert (operand.rows () == 1);
  assert (operand.cols () == cols ());
  for (size_t i = 0; i < cols (); ++i)
  {
    (*matrix_ptr_) (row_index_, i) *= operand (0, i);
  }
}

template <typename T>
void 
pcl2::core::MatRowImpl<T>::operator /= (const TypedMatImpl<T> & operand)
{
  assert (operand.rows () == 1);
  assert (operand.cols () == cols ());
  for (size_t i = 0; i < cols (); ++i)
  {
    (*matrix_ptr_) (row_index_, i) /= operand (0, i);
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
