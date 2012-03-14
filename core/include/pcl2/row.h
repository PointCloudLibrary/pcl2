/** \file row.h 
 * \brief Contains class declarations for Row<T> and ConstRow<T> 
 */

#ifndef PCL2_ROW_H
#define PCL2_ROW_H

#include "pcl2/typed_matrix.h"

#include <iostream>
#include <boost/shared_ptr.hpp>
#include <boost/pointer_cast.hpp>

namespace pcl2
{
template <typename T> class TypedMat;
namespace core
{
  template <typename T> class TypedMatImpl;
  template <typename T> class MatRowImpl;
}

///////////////////////////////////////////////////////////////////////////////
/** \brief \todo Write me
 *
 * Extends the TypedMat class to...
 */
template <typename T>
class Row : public TypedMat<T>
{
protected:
  using TypedMat<T>::matrix_ptr_;

private:
  Row ();

protected:
  /** \brief Construct a Row view into the provided TypedMatImpl
   */
  Row (boost::shared_ptr<core::TypedMatImpl<T> > matrix_ptr, size_t row);

public:
  int
  getIndex () const;

  bool
  hasNext () const;

  void
  advance ();

protected:
  /** \brief A shared pointer to the matrix implementation */
  boost::shared_ptr<core::MatRowImpl<T> > mat_row_ptr_;

  friend class TypedMat<T>;
};

template <typename T>
std::ostream& operator << (std::ostream& out, const pcl2::Row<T> & row)
{
  for (size_t i = 0; i < row.cols (); ++i)
    out << row (0, i) << " ";

  return (out);
}

}

#endif
