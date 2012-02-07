/** \file eigen_matrix.h 
 * \brief Contains class declarations for EigenMat and ConstEigenMat 
 */

#ifndef PCL2_EIGEN_MATRIX_H
#define PCL2_EIGEN_MATRIX_H

#include "pcl2/typed_matrix.h"
#include "pcl2/eigen_matrix_impl.h"

namespace pcl2
{

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
  using Mat::matrix_ptr_;

private:
  EigenMat ();

protected:
  /** \brief Construct a EigenMat around the provided MatImpl 
   *
   * \throws Throws a BadCastException if the provided MatImpl cannot be cast 
   * to an EigenMatrixImpl.
   * \todo Make this throw an exception instead of an assertion failure
   */
  EigenMat (core::MatImpl::Ptr matrix);

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
  typename core::EigenMatImpl<T>::Ptr eigen_matrix_ptr_;
};

///////////////////////////////////////////////////////////////////////////////
/** \brief A const version of EigenMat 
 *
 * \see EigenMat
 * \see Mat 
 */
template <typename T>
class ConstEigenMat : public ConstTypedMat<T>
{
  using Mat::matrix_ptr_;

private:
  ConstEigenMat ();

protected:
  /** \brief Construct a ConstEigenMat around the provided MatImpl 
   *
   * \throws Throws a BadCastException if the provided MatImpl cannot be cast 
   * to a const EigenMatrixImpl.
   * \todo Make this throw an exception instead of an assertion failure
   */
  ConstEigenMat (core::MatImpl::Ptr matrix);

public:
  /** \brief Construct a ConstEigenMat from a generic Mat.  
   *
   * \throws Throws a BadCastException if the provided matrix cannot be cast to a
   * const EigenMatrix.
   * \todo Make this throw an exception instead of an assertion failure
   */
  ConstEigenMat (Mat & shared_matrix);

protected:
  /** A const shared pointer to the matrix implementation */
  typename core::EigenMatImpl<T>::ConstPtr eigen_matrix_ptr_;
};

}

#endif
