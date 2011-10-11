#include "pcl2/float_array.h"

pcl2::FloatArray::FloatArray () { assert (false); }

pcl2::FloatArray::FloatArray (const FloatArray & f) { assert (false); }

pcl2::FloatArray::FloatArray (size_t rows, size_t cols)
{
  rows_ = rows;
  cols_ = cols;
  data_.resize (rows*cols);
}

pcl2::FloatArray::Ptr
pcl2::FloatArray::create (size_t rows, size_t cols)
{
  return (Ptr (new FloatArray (rows, cols)));
}

size_t 
pcl2::FloatArray::rows () const
{
  return (rows_);
}

size_t 
pcl2::FloatArray::cols () const
{
  return (cols_);
}


float & 
pcl2::FloatArray::operator () (size_t i, size_t j)
{
  // check inputs
  return (data_[j*rows_+i]);
}

const float & 
pcl2::FloatArray::operator () (size_t i, size_t j) const
{
  // check inputs
  return (data_[j*rows_+i]);
}

pcl2::Array::Ptr 
pcl2::FloatArray::getSharedPtr ()
{
  return (boost::static_pointer_cast<pcl2::Array> (shared_from_this ()));
}

pcl2::Array::Ptr 
pcl2::FloatArray::copy () const
{
  FloatArray::Ptr array_copy (new FloatArray (rows_, cols_));
  array_copy->data_ = data_;
  return (array_copy);
}
