/** \file matrix.cpp
 * \brief Contains class definition for \ref pcl2::Mat
 */

#include <assert.h>
#include "pcl2/matrix.h"
#include "pcl2/typed_matrix.h"

#include "pcl2/matrix_view_impl.h"

pcl2::Mat::Mat (const core::MatImpl::Ptr matrix_ptr) : matrix_ptr_ (matrix_ptr) {}

pcl2::core::MatImpl::Ptr 
pcl2::Mat::getPtr ()
{
  return (matrix_ptr_);
}

pcl2::core::MatImpl::ConstPtr 
pcl2::Mat::getPtr () const
{
  return (matrix_ptr_);
}

pcl2::Mat
pcl2::Mat::copy () const
{
  return (Mat (matrix_ptr_->copy ()));
}

size_t
pcl2::Mat::rows () const
{
  return (matrix_ptr_->rows ());
}

size_t
pcl2::Mat::cols () const
{
  return (matrix_ptr_->cols ());
}

pcl2::Mat
pcl2::Mat::operator () (const ConstTypedMat<int> & indices)
{
  typedef core::TypedMatImpl<int> Impl;
  Impl::ConstPtr idx = boost::dynamic_pointer_cast<const Impl> (indices.getPtr ());
  return (Mat (matrix_ptr_->createView (idx)));
}

void 
pcl2::Mat::fill (const Mat & matrix)
{
  matrix_ptr_->fill (matrix.matrix_ptr_);
}


pcl2::Mat & 
pcl2::Mat::operator += (const Mat & matrix)
{
  /// \todo Implement this operator!
  assert (false);
}

pcl2::Mat &
pcl2::Mat::operator *= (const Mat & matrix)
{
  /// \todo Implement this operator!
  assert (false);
}
