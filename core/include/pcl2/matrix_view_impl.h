/** \file matrix_view_impl.h 
 * \brief Contains class declaration for pcl2::MatViewImpl
 */

#ifndef PCL2_MATRIX_VIEW_IMPL_H
#define PCL2_MATRIX_VIEW_IMPL_H

#include "typed_matrix_impl.h"

namespace pcl2
{
namespace core
{

/** \brief A "view" into a matrix, appearing to contain only the specified subset
 * and/or permutation of rows from the original matrix.
 *
 * A MatViewImpl wraps a source matrix and a list of indices and creates a new
 * matrix that appears to contain a row for each index in the provided indices, 
 * where the elements in each row of the view correspond to the elements at that
 * row in the source matrix. Note that although a view can be used like a true 
 * matrix, it is not an independent copy of the source matrix.  Thus, any changes
 * made to a view's data will also affect the corresponding values in the source
 * matrix and vice versa.
 */
template <typename T>
class MatViewImpl : public TypedMatImpl<T>, public boost::enable_shared_from_this<MatViewImpl<T> >
{
public:
  /** \brief A shared pointer to a MatViewImpl */
  typedef boost::shared_ptr<MatViewImpl<T> > Ptr;

protected:
  MatViewImpl ();
  MatViewImpl (const MatViewImpl & a);

public:
  /** \brief Create a view into the specified rows of the provided matrix 
   *
   * This constructor creates a view from a source \a matrix and a matrix of 
   * \a indices. The resulting view will contain a row for each row index 
   * defined in  \a indices, and the elements in each row of the view will be 
   * references to the corresponding elements in the original matrix.
   *
   * \param matrix The source matrix (i.e., the matrix that is to be "viewed")
   * \param indices A matrix of integers indexing rows of the source \a matrix.
   */
  MatViewImpl (typename TypedMatImpl<T>::Ptr matrix_ptr, TypedMatImpl<int>::ConstPtr indices_ptr);

  virtual MatImpl::Ptr copy () const;

  virtual MatImpl::Ptr createNew (size_t rows, size_t cols) const;
  virtual MatImpl::Ptr createView (const TypedMatImpl<int>::ConstPtr & indices);

  virtual size_t rows () const;
  virtual size_t cols () const;

  virtual void fill (const TypedMatImpl<T> & matrix);
  virtual void fill (const T & value);

  virtual typename TypedMatImpl<T>::Ptr operator + (const T & operand) const;
  virtual typename TypedMatImpl<T>::Ptr operator - (const T & operand) const;
  virtual typename TypedMatImpl<T>::Ptr operator * (const T & operand) const;
  virtual typename TypedMatImpl<T>::Ptr operator / (const T & operand) const;

  virtual typename TypedMatImpl<T>::Ptr operator + (const TypedMatImpl<T> & operand) const;
  virtual typename TypedMatImpl<T>::Ptr operator - (const TypedMatImpl<T> & operand) const;

  virtual void operator += (const TypedMatImpl<T> & operand);
  virtual void operator -= (const TypedMatImpl<T> & operand);
  virtual void operator *= (const TypedMatImpl<T> & operand);
  virtual void operator /= (const TypedMatImpl<T> & operand);

  virtual T & operator () (size_t i, size_t j);
  virtual const T & operator () (size_t i, size_t j) const;

protected:
  /** The matrix being "viewed" */
  typename TypedMatImpl<T>::Ptr matrix_ptr_;

  /** The list of indices into matrix_ that defines this view */
  TypedMatImpl<int>::ConstPtr indices_ptr_;

};

} // namespace core
} // namespace pcl2

#endif
