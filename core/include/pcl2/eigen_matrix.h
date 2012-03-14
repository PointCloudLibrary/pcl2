/** \file eigen_matrix.h 
 * \brief Contains class declarations for EigenMat and ConstEigenMat 
 */

#ifndef PCL2_EIGEN_MATRIX_H
#define PCL2_EIGEN_MATRIX_H

#include "pcl2/typed_matrix.h"

namespace pcl2
{

namespace core
{
  template <typename T> class EigenMatImpl;
}

///////////////////////////////////////////////////////////////////////////////
/** \brief A shared wrapper of an Eigen matrix
 *
 * Extends the TypedMat class to add constructors for creating new Eigen 
 * matrices
 * \see ConstEigenMat
 */
template <typename T>
class EigenMat : public TypedMat<T>
{
protected:
  using TypedMat<T>::matrix_ptr_;

private:
  EigenMat ();

protected:
  /** \brief A shared pointer to the implementation */
  typedef boost::shared_ptr<core::MatImpl> MatImplPtr;

  /** \brief Construct a EigenMat around the provided MatImpl 
   *
   * \throws Throws a BadCastException if the provided MatImpl cannot be cast 
   * to an EigenMatrixImpl.
   * \todo Make this throw an exception instead of an assertion failure
   */
  EigenMat (MatImplPtr matrix);

public:
  /** \brief Construct a EigenMat from a generic Mat.  
   *
   * \throws Throws a BadCastException if the provided matrix cannot be cast to an
   * EigenMatrix.
   * \todo Make this throw an exception instead of an assertion failure
   */
  EigenMat (const Mat & shared_matrix);

  /** \brief Construct a new EigenMat with the specified number of rows and columns 
   *
   * \param rows The number of rows
   * \param cols The number of cols
   */
  EigenMat (size_t rows, size_t cols);

protected:
  /** \brief A shared pointer to the matrix implementation */
  boost::shared_ptr<core::EigenMatImpl<T> > eigen_matrix_ptr_;
};

}

#endif
