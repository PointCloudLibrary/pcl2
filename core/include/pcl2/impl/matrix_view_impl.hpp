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
pcl2::core::MatViewImpl<T>::MatViewImpl (typename TypedMatImpl<T>::Ptr matrix, TypedMatImpl<int>::ConstPtr indices) : 
  matrix_ (matrix), 
  indices_ (indices)
{}

template <typename T>
pcl2::core::MatImpl::Ptr
pcl2::core::MatViewImpl<T>::copy () const 
{
  size_t r = rows ();
  size_t c = cols ();
  MatImpl::Ptr output_matrix_ptr = matrix_->createNew (r, c);
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
  return (matrix_->createNew (rows, cols));
}

template <typename T>
pcl2::core::MatImpl::Ptr 
pcl2::core::MatViewImpl<T>::createView (const TypedMatImpl<int>::ConstPtr & indices)
{
  assert (false); /// \todo Implement this method! (Views of views...)
}

template <typename T>
size_t 
pcl2::core::MatViewImpl<T>::rows () const { return (indices_->rows ()); }

template <typename T>
size_t 
pcl2::core::MatViewImpl<T>::cols () const { return (matrix_->cols ()); }

template <typename T>
void
pcl2::core::MatViewImpl<T>::fill (const MatImpl::ConstPtr & matrix_ptr)
{
  ///\todo Write this method!
  assert (false);
}

template <typename T>
void 
pcl2::core::MatViewImpl<T>::fill (const T & value)
{
  for (int i = 0; i < rows (); ++i)
  {
    const int i2 = (*indices_) (i, 0);
    for (int j = 0; j < cols (); ++j)
    {
      (*matrix_) (i2, j) = value;
    }
  }
}

template <typename T>
T & 
pcl2::core::MatViewImpl<T>::operator () (size_t i, size_t j)
{
  const int i2 = (*indices_) (i, 0);
  return ((*matrix_) (i2, j));
}

template <typename T>
const T & 
pcl2::core::MatViewImpl<T>::operator () (size_t i, size_t j) const
{
  const int i2 = (*indices_) (i, 0);
  return ((*matrix_) (i2, j));
}

#endif
