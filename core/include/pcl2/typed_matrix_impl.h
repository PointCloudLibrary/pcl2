/** \file typed_matrix_impl.h 
 * \brief Contains class declaration for pcl2::TypedMatImpl
 */

#ifndef PCL2_TYPED_MATRIX_IMPL_H
#define PCL2_TYPED_MATRIX_IMPL_H
#include <iostream>
#include "matrix_impl.h"

namespace pcl2
{
namespace core
{

/** \brief An abstract matrix implementation templated on scalar type 
 * (e.g., float, int)
 *
 * The TypedMatImpl class extends the base MatImpl class to add element
 * accessors.  It must be templated on the matrix's scalar type.
 */
template <typename T>
class TypedMatImpl : public MatImpl
{
public:
  /** \brief A shared pointer to a TypedMatImpl */
  typedef boost::shared_ptr<TypedMatImpl<T> > Ptr;

  /** \brief A const shared pointer to a TypedMatImpl */
  typedef boost::shared_ptr<const TypedMatImpl<T> > ConstPtr;

  /** \brief Fill in the values of this matrix using the values from the provided
   * matrix
   * \param matrix_ptr The matrix whose values will be copied into this matrix.
   * The size of the input matrix must be compatible with the matrix being filled.
   *
   * \throws IncompatibleSizeException
   * \see Mat::fill (const Mat & matrix)
   */
  virtual void fill (const MatImpl::ConstPtr & matrix_ptr) = 0; 
  virtual void fill (const T & value) = 0;

  /** \brief Access an element in the matrix
   * \param i The row of the element
   * \param j The column of the element
   * \return A reference to the element in the ith row and jth column
   */
  virtual T & operator () (size_t i, size_t j) = 0;

  /** \brief Access an element in the matrix
   * \param i The row of the element
   * \param j The column of the element
   * \return A const reference to the element in the ith row and jth column
   */
  virtual const T & operator () (size_t i, size_t j) const = 0;
};

} // namespace core
} // namespace pcl2

#endif
