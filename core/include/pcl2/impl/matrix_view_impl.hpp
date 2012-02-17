/** \file matrix_view_impl.hpp
 * \brief Contains class definition for pcl2::MatViewImpl
 */

#ifndef PCL2_MATRIX_VIEW_IMPL_HPP
#define PCL2_MATRIX_VIEW_IMPL_HPP

#include "pcl2/matrix_view_impl.h"

template <typename T>
pcl2::core::MatViewImpl<T>::MatViewImpl () { assert (false); }

template <typename T>
pcl2::core::MatViewImpl<T>::MatViewImpl (const MatViewImpl & a) { assert (false); }

template <typename T>
pcl2::core::MatViewImpl<T>::MatViewImpl (typename TypedMatImpl<T>::Ptr matrix_ptr, 
                                         TypedMatImpl<int>::ConstPtr indices_ptr) : 
  matrix_ptr_ (matrix_ptr), 
  indices_ptr_ (indices_ptr)
{}

template <typename T>
pcl2::core::MatImpl::Ptr
pcl2::core::MatViewImpl<T>::copy () const 
{
  size_t r = rows ();
  size_t c = cols ();
  MatImpl::Ptr output_matrix_ptr = matrix_ptr_->createNew (r, c);
  TypedMatImpl<T> & output_matrix_ref = *boost::dynamic_pointer_cast<TypedMatImpl<T> > (output_matrix_ptr);
  for (size_t i = 0; i < r; ++i)
  {
    for (size_t j = 0; j < c; ++j)
    {
      output_matrix_ref (i, j) = operator () (i, j);
     }
  }
  
  return (output_matrix_ptr);
}

template <typename T>
pcl2::core::MatImpl::Ptr
pcl2::core::MatViewImpl<T>::createNew (size_t rows, size_t cols) const 
{ 
  return (matrix_ptr_->createNew (rows, cols));
}

template <typename T>
pcl2::core::MatImpl::Ptr 
pcl2::core::MatViewImpl<T>::createView (const TypedMatImpl<int>::ConstPtr & indices)
{
  assert (false); /// \todo Implement this method! (Views of views...)
}

template <typename T>
size_t 
pcl2::core::MatViewImpl<T>::rows () const
{
  return (indices_ptr_->rows ());
}

template <typename T>
size_t 
pcl2::core::MatViewImpl<T>::cols () const
{
  return (matrix_ptr_->cols ());
}

template <typename T>
void
pcl2::core::MatViewImpl<T>::fill (const TypedMatImpl<T> & matrix)
{
  assert (matrix.rows () == rows () && matrix.cols () == cols ());

  for (size_t i = 0; i < rows (); ++i)
  {
    const size_t i2 = (*indices_ptr_) (i, 0);
    for (size_t j = 0; j < cols (); ++j)
    {
      (*matrix_ptr_) (i2, j) = matrix (i, j);
    }
  }
}

template <typename T>
void 
pcl2::core::MatViewImpl<T>::fill (const T & value)
{
  for (size_t i = 0; i < rows (); ++i)
  {
    const size_t i2 = (*indices_ptr_) (i, 0);
    for (size_t j = 0; j < cols (); ++j)
    {
      (*matrix_ptr_) (i2, j) = value;
    }
  }
}

template <typename T>
typename pcl2::core::TypedMatImpl<T>::Ptr
pcl2::core::MatViewImpl<T>::operator + (const T & operand) const
{
  typename TypedMatImpl<T>::Ptr output_matrix_ptr = 
    boost::static_pointer_cast<core::TypedMatImpl<T> > (matrix_ptr_->createNew (rows (), cols ()));

  for (size_t i = 0; i < rows (); ++i)
  {
    const size_t i2 = (*indices_ptr_) (i, 0);
    for (size_t j = 0; j < cols (); ++j)
    {
      (*output_matrix_ptr) (i, j) = (*matrix_ptr_) (i2, j) + operand;
    }
  }
  return (output_matrix_ptr);
}

template <typename T>
typename pcl2::core::TypedMatImpl<T>::Ptr
pcl2::core::MatViewImpl<T>::operator - (const T & operand) const
{
  typename TypedMatImpl<T>::Ptr output_matrix_ptr = 
    boost::static_pointer_cast<core::TypedMatImpl<T> > (matrix_ptr_->createNew (rows (), cols ()));

  for (size_t i = 0; i < rows (); ++i)
  {
    const size_t i2 = (*indices_ptr_) (i, 0);
    for (size_t j = 0; j < cols (); ++j)
    {
      (*output_matrix_ptr) (i, j) = (*matrix_ptr_) (i2, j) - operand;
    }
  }
  return (output_matrix_ptr);
}

template <typename T>
typename pcl2::core::TypedMatImpl<T>::Ptr
pcl2::core::MatViewImpl<T>::operator * (const T & operand) const
{
  typename TypedMatImpl<T>::Ptr output_matrix_ptr = 
    boost::static_pointer_cast<core::TypedMatImpl<T> > (matrix_ptr_->createNew (rows (), cols ()));

  for (size_t i = 0; i < rows (); ++i)
  {
    const size_t i2 = (*indices_ptr_) (i, 0);
    for (size_t j = 0; j < cols (); ++j)
    {
      (*output_matrix_ptr) (i, j) = (*matrix_ptr_) (i2, j) * operand;
    }
  }
  return (output_matrix_ptr);
}

template <typename T>
typename pcl2::core::TypedMatImpl<T>::Ptr
pcl2::core::MatViewImpl<T>::operator / (const T & operand) const
{
  typename TypedMatImpl<T>::Ptr output_matrix_ptr = 
    boost::static_pointer_cast<core::TypedMatImpl<T> > (matrix_ptr_->createNew (rows (), cols ()));

  for (size_t i = 0; i < rows (); ++i)
  {
    const size_t i2 = (*indices_ptr_) (i, 0);
    for (size_t j = 0; j < cols (); ++j)
    {
      (*output_matrix_ptr) (i, j) = (*matrix_ptr_) (i2, j) / operand;
    }
  }
  return (output_matrix_ptr);
}

template <typename T>
typename pcl2::core::TypedMatImpl<T>::Ptr
pcl2::core::MatViewImpl<T>::operator + (const TypedMatImpl<T> & operand) const
{
  assert (operand.rows () == rows () && operand.cols () == cols ());

  typename TypedMatImpl<T>::Ptr output_matrix_ptr = 
    boost::static_pointer_cast<core::TypedMatImpl<T> > (matrix_ptr_->createNew (rows (), cols ()));

  for (size_t i = 0; i < rows (); ++i)
  {
    const size_t i2 = (*indices_ptr_) (i, 0);
    for (size_t j = 0; j < cols (); ++j)
    {
      (*output_matrix_ptr) (i, j) = (*matrix_ptr_) (i2, j) + operand (i, j);
    }
  }
  return (output_matrix_ptr);
}

template <typename T>
typename pcl2::core::TypedMatImpl<T>::Ptr
pcl2::core::MatViewImpl<T>::operator - (const TypedMatImpl<T> & operand) const
{
  assert (operand.rows () == rows () && operand.cols () == cols ());

  typename TypedMatImpl<T>::Ptr output_matrix_ptr = 
    boost::static_pointer_cast<core::TypedMatImpl<T> > (matrix_ptr_->createNew (rows (), cols ()));

  for (size_t i = 0; i < rows (); ++i)
  {
    const size_t i2 = (*indices_ptr_) (i, 0);
    for (size_t j = 0; j < cols (); ++j)
    {
      (*output_matrix_ptr) (i, j) = (*matrix_ptr_) (i2, j) - operand (i, j);
    }
  }
  return (output_matrix_ptr);
}

template <typename T>
void 
pcl2::core::MatViewImpl<T>::operator += (const TypedMatImpl<T> & operand)
{
  assert (operand.rows () == rows () && operand.cols () == cols ());

  for (size_t i = 0; i < rows (); ++i)
  {
    const size_t i2 = (*indices_ptr_) (i, 0);
    for (size_t j = 0; j < cols (); ++j)
    {
      (*matrix_ptr_) (i2, j) += operand (i, j);
    }
  }
}

template <typename T>
void 
pcl2::core::MatViewImpl<T>::operator -= (const TypedMatImpl<T> & operand)
{
  assert (operand.rows () == rows () && operand.cols () == cols ());

  for (size_t i = 0; i < rows (); ++i)
  {
    const size_t i2 = (*indices_ptr_) (i, 0);
    for (size_t j = 0; j < cols (); ++j)
    {
      (*matrix_ptr_) (i2, j) -= operand (i, j);
    }
  }
}

template <typename T>
void 
pcl2::core::MatViewImpl<T>::operator *= (const TypedMatImpl<T> & operand)
{
  assert (operand.rows () == rows () && operand.cols () == cols ());

  for (size_t i = 0; i < rows (); ++i)
  {
    const size_t i2 = (*indices_ptr_) (i, 0);
    for (size_t j = 0; j < cols (); ++j)
    {
      (*matrix_ptr_) (i2, j) *= operand (i, j);
    }
  }
}

template <typename T>
void 
pcl2::core::MatViewImpl<T>::operator /= (const TypedMatImpl<T> & operand)
{
  assert (operand.rows () == rows () && operand.cols () == cols ());

  for (size_t i = 0; i < rows (); ++i)
  {
    const size_t i2 = (*indices_ptr_) (i, 0);
    for (size_t j = 0; j < cols (); ++j)
    {
      (*matrix_ptr_) (i2, j) /= operand (i, j);
    }
  }
}

template <typename T>
T & 
pcl2::core::MatViewImpl<T>::operator () (size_t i, size_t j)
{
  const int i2 = (*indices_ptr_) (i, 0);
  return ((*matrix_ptr_) (i2, j));
}

template <typename T>
const T & 
pcl2::core::MatViewImpl<T>::operator () (size_t i, size_t j) const
{
  const int i2 = (*indices_ptr_) (i, 0);
  return ((*matrix_ptr_) (i2, j));
}

#endif
