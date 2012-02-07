/** \file typed_matrix.h 
 * \brief Contains class declarations for TypedMat and ConstTypedMat 
 */

#ifndef PCL2_TYPED_MATRIX_H
#define PCL2_TYPED_MATRIX_H

#include "pcl2/matrix.h"
#include "pcl2/typed_matrix_impl.h"
#include "pcl2/row.h"

namespace pcl2
{

///////////////////////////////////////////////////////////////////////////////
/** \brief 
 *
 * Extends the Mat class to add type-specific accessor operators
 * \see ConstTypedMat Mat
 */
template <typename T>
class TypedMat : public Mat
{
public:
  typedef pcl2::Row<T> Row;

private:
  TypedMat ();

protected:
  /** \brief Construct a TypedMat around the provided MatImpl 
   *
   * \throws Throws a BadCastException if the provided MatImpl cannot be cast 
   * to a matrix of the specified type, T.
   * \todo Make this throw an exception instead of an assertion failure
   */
  TypedMat (core::MatImpl::Ptr matrix);

public:
  /** \brief Construct a TypedMat from a generic Mat.  
   *
   * \throws Throws a BadCastException if the provided matrix cannot be cast to a
   * matrix of the specified type, T.
   * \todo Make this throw an exception instead of an assertion failure
   */
  TypedMat (const Mat & shared_matrix);

  /** \brief Access an element in the matrix
   *
   * \param i The row of the element
   * \param j The column of the element
   * \return A reference to the element in the ith row and jth column
   */
  T & operator () (size_t i, size_t j);

  /** \brief Access an element in the matrix
   *
   * \param i The row of the element
   * \param j The column of the element
   * \return A const reference to the element in the ith row and jth column
   */
  const T & operator () (size_t i, size_t j) const;

  Row operator () (size_t row_index);

  TypedMat operator () (const ConstTypedMat<int> & indices);

  /** Fill in this matrix with the provided scalar value
   * \param value The scalar value that will be copied into every element of this matrix
   *
   * \throws IncompatibleSizeException
   */
  void fill (const T & value);

protected:
  /** \brief A shared pointer to the matrix implementation */
  typename core::TypedMatImpl<T>::Ptr typed_matrix_ptr_;
};

///////////////////////////////////////////////////////////////////////////////
/** \brief A const version of TypedMat 
 *
 * \see TypedMat
 * \see Mat 
 */
template <typename T>
class ConstTypedMat : public Mat
{
public:
  typedef pcl2::ConstRow<T> Row;

private:
  ConstTypedMat ();

protected:
  /** \brief Construct a ConstTypedMat around the provided MatImpl 
   *
   * \throws Throws a BadCastException if the provided MatImpl cannot be cast 
   * to a const matrix of the specified type, T.
   * \todo Make this throw an exception instead of an assertion failure
   */
  ConstTypedMat (core::MatImpl::Ptr matrix);

public:
  /** \brief Construct a ConstTypedMat from a generic Mat.  
   *
   * \throws Throws a BadCastException if the provided matrix cannot be cast to a
   * const matrix of the specified type, T.
   * \todo Make this throw an exception instead of an assertion failure
   */
  ConstTypedMat (const Mat & shared_matrix);

  /** \brief Access an element in the matrix
   *
   * \param i The row of the element
   * \param j The column of the element
   * \return A const reference to the element in the ith row and jth column
   */
  const T & operator () (size_t i, size_t j) const;

  Row operator () (size_t row_index) const;

  ConstTypedMat operator () (const ConstTypedMat<int> & indices);

protected:
  /** A const shared pointer to the matrix implementation */
  typename core::TypedMatImpl<T>::Ptr typed_matrix_ptr_;
};

typedef TypedMat<int> MatI;
typedef ConstTypedMat<int> ConstMatI;
typedef TypedMat<float> MatF;
typedef ConstTypedMat<float> ConstMatF;
typedef TypedMat<double> MatD;
typedef ConstTypedMat<double> ConstMatD;

template <typename T>
std::ostream& operator << (std::ostream& out, const pcl2::TypedMat<T> & mat)
{
  const pcl2::ConstTypedMat<T> const_mat (mat);
  return (out << const_mat);
}

template <typename T>
std::ostream& operator << (std::ostream& out, const pcl2::ConstTypedMat<T> & mat)
{
  for (typename pcl2::ConstRow<T> r = mat (0); r.hasNext (); r.advance ())
    out << r << std::endl;
  return (out);
}

}

#endif
