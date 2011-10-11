#ifndef PCL2_FLOAT_ARRAY_H
#define PCL2_FLOAT_ARRAY_H

#include "pcl2/array.h"

#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <vector>

namespace pcl2
{

/** \brief An array of floats */
class FloatArray : public Array, public boost::enable_shared_from_this<FloatArray> 
{
public:
  typedef boost::shared_ptr<FloatArray> Ptr;

private:
  FloatArray ();
  FloatArray (const FloatArray & f);
  FloatArray (size_t rows, size_t cols);

public:

  /** \brief Create a new FloatArray
   * \param rows The number of rows in the array
   * \param cols The number of columns in the array
   * \return A shared pointer to a new FloatArray
   */
  static FloatArray::Ptr create (size_t rows, size_t cols);

  virtual size_t rows () const;
  virtual size_t cols () const;

  /** \brief Access an element in the array
   * \param i The row of the element
   * \param j The column of the element
   * \return A reference to the element in the ith row and jth column
   */
  float & operator () (size_t i, size_t j);

  /** \brief Access an element in the array
   * \param i The row of the element
   * \param j The column of the element
   * \return A const reference to the element in the ith row and jth column
   */
  const float & operator () (size_t i, size_t j) const;

  virtual Array::Ptr getSharedPtr ();
  virtual Array::Ptr copy () const;

protected:

  /** \brief The number of rows */
  size_t rows_;

  /** \brief The number of columns */
  size_t cols_;

  /** \brief The array data (stored in column major form) */
  std::vector<float> data_;
};

}

#endif
