/** \file row.h 
 * \brief Contains class declarations for Row<T> and ConstRow<T> 
 */

#ifndef PCL2_ROW_H
#define PCL2_ROW_H

#include "pcl2/typed_matrix.h"
#include "pcl2/matrix_row_impl.h"

namespace pcl2
{

template <typename T> class TypedMat;

///////////////////////////////////////////////////////////////////////////////
/** \brief \todo Write me
 *
 * Extends the TypedMat class to...
 * \see ConstRow
 */
template <typename T>
class Row : public TypedMat<T>
{
  using Mat::matrix_ptr_;

private:
  Row () {};

protected:
  /** \brief Construct a Row view into the provided TypedMatImpl
   */
  Row (typename core::TypedMatImpl<T>::Ptr matrix_ptr, size_t row) :
    TypedMat<T> (typename core::MatRowImpl<T>::Ptr (new core::MatRowImpl<T> (matrix_ptr, row)))
  {
    mat_row_ptr_ = boost::dynamic_pointer_cast<core::MatRowImpl<T> > (matrix_ptr_);
  }

public:
  inline int getIndex () const { return (mat_row_ptr_->getIndex ()); }
  inline bool hasNext () const { return (mat_row_ptr_->hasNext ()); }
  inline void advance () { mat_row_ptr_->advance (); }

protected:
  /** \brief A shared pointer to the matrix implementation */
  typename core::MatRowImpl<T>::Ptr mat_row_ptr_;

  friend class TypedMat<T>;
};

///////////////////////////////////////////////////////////////////////////////
/** \brief A const version of Row 
 *
 * \see Row
 */
template <typename T>
class ConstRow : public ConstTypedMat<T>
{
  using Mat::matrix_ptr_;

private:
  ConstRow () {};

public:
  ConstRow (const Row<T> & row) : 
    ConstTypedMat<T> (row)
  {
    mat_row_ptr_ = boost::dynamic_pointer_cast<core::MatRowImpl<T> > (matrix_ptr_);
  }

protected:
  /** \brief Construct a Row view into the provided TypedMatImpl
   */
  ConstRow (typename core::TypedMatImpl<T>::Ptr matrix_ptr, int row) :
    ConstTypedMat<T> (typename core::MatRowImpl<T>::Ptr (new core::MatRowImpl<T> (matrix_ptr, row)))
  {
    mat_row_ptr_ = boost::dynamic_pointer_cast<core::MatRowImpl<T> > (matrix_ptr_);
  }

public:
  inline int getIndex () const { return (mat_row_ptr_->getIndex ()); }
  inline bool hasNext () const { return (mat_row_ptr_->hasNext ()); }
  inline void advance () { mat_row_ptr_->advance (); }

protected:

  typename core::MatRowImpl<T>::Ptr mat_row_ptr_;

  friend class ConstTypedMat<T>;
};

template <typename T>
std::ostream& operator << (std::ostream& out, const pcl2::Row<T> row)
{
  const pcl2::ConstRow<T> const_row (row);
  return (out << const_row);
}

template <typename T>
std::ostream& operator << (std::ostream& out, const pcl2::ConstRow<T> row)
{
  for (size_t i = 0; i < row.cols (); ++i)
    out << row (0, i) << " ";

  return (out);
}

}

#endif
