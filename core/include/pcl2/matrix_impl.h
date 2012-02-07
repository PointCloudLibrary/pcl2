/** \file matrix_impl.h 
 * \brief Contains class declaration for pcl2::MatImpl
 */

#ifndef PCL2_MATRIX_IMPL_H
#define PCL2_MATRIX_IMPL_H

#include <boost/shared_ptr.hpp>

namespace pcl2
{
namespace core
{

template <typename T> class TypedMatImpl;

/** \brief An abstract class defining the matrix implementation interface. 
 * For internal use only.
 *
 * Subclasses of MatImpl will provide the actual implementation for the Mat
 * class. They are responsible for storing the actual matrix data and providing
 * various methods for operating on it. 
 *
 * \note These MatImpl classes are intended to be used exclusively by Mat
 * and its subclasses. PCL users are not meant to use MatImpl or any of its 
 * subclasses in their code.
 */
class MatImpl
{
public:
  /** \brief A shared pointer to an MatImpl */
  typedef boost::shared_ptr<MatImpl> Ptr;

  /** \brief A const shared pointer to an MatImpl */
  typedef boost::shared_ptr<const MatImpl> ConstPtr;

  /** \brief Create a new copy of this matrix and its data
   *
   * \return A shared pointer to a new copy of this matrix
   */
  virtual MatImpl::Ptr copy () const = 0;

  /** \brief Create a new MatImpl of the given size
   *
   * \param rows The number of rows in the new matrix
   * \param cols The number of cols in the new matrix
   * \return A shared pointer to a new matrix
   */
  virtual MatImpl::Ptr createNew (size_t rows, size_t cols) const = 0;

  /** \brief Create a new MatViewImpl from this MatImpl and the provided 
   * indices 
   *
   * \param indices An matrix of integers indexing rows of this matrix
   * \return A shared pointer to a new MatViewImpl
   */
  virtual MatImpl::Ptr createView (const boost::shared_ptr<const TypedMatImpl<int> > & indices) = 0;

  /** \brief Get the number of rows in the matrix
   *
   * \return The number of rows in the matrix
   */
  virtual size_t rows () const = 0;

  /** \brief Get the number of columns in the matrix
   *
   * \return The number of columns in the matrix
   */
  virtual size_t cols () const = 0;

  /** \brief Fill in the values of this matrix using the values from the provided
   * matrix
   * \param matrix The matrix whose values will be copied into this matrix.
   * The size of the input matrix must be compatible with the matrix being filled.
   *
   * \throws IncompatibleSizeException
   * \see Mat::fill (const Mat & matrix)
   */
  virtual void fill (const MatImpl::ConstPtr & matrix) = 0;

};

} // namespace core
} // namespace pcl2

#endif
