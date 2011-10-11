#include "pcl2/int_array.h"

pcl2::IntArray::IntArray () { assert (false); }

pcl2::IntArray::IntArray (const IntArray & f) { assert (false); }

pcl2::IntArray::IntArray (size_t rows, size_t cols)
{
  rows_ = rows;
  cols_ = cols;
  data_.resize (rows*cols);
}

pcl2::IntArray::Ptr
pcl2::IntArray::create (size_t rows, size_t cols)
{
  return (Ptr (new IntArray (rows, cols)));
}

size_t 
pcl2::IntArray::rows () const
{
  return (rows_);
}

size_t 
pcl2::IntArray::cols () const
{
  return (cols_);
}


int & 
pcl2::IntArray::operator () (size_t i, size_t j)
{
  // check inputs
  return (data_[j*rows_+i]);
}

const int & 
pcl2::IntArray::operator () (size_t i, size_t j) const
{
  // check inputs
  return (data_[j*rows_+i]);
}

pcl2::Array::Ptr 
pcl2::IntArray::getSharedPtr ()
{
  return (boost::static_pointer_cast<pcl2::Array> (shared_from_this ()));
}

pcl2::Array::Ptr 
pcl2::IntArray::copy () const
{
  IntArray::Ptr array_copy (new IntArray (rows_, cols_));
  array_copy->data_ = data_;
  return (array_copy);
}
