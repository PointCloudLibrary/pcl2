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
class TypedMat : public /*virtual*/ Mat//, public virtual ConstTypedMat<T>
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

  /** \brief Access an element in the matrix
   *
   * \param i The index of the element in row-major order
   * \return A const reference to the ith element
   */
  const T & operator [] (size_t i) const;

  Row operator () (size_t row_index);
  const Row operator () (size_t row_index) const;

  TypedMat operator () (const TypedMat<int> & indices);

  /** \brief Fill in this matrix with the provided scalar value
   * \param value The scalar value that will be copied into every element of this matrix
   */
  void fill (const T & value);

  /** \brief Fill in this matrix with the provided values
   * \param matrix The matrix whose values will be copied into the corresponding elements of this matrix
   * The size of the input matrix (i.e., the source matrix) must be compatible with the
   * matrix being filled (i.e., the destination).
   *
   * \throws IncompatibleSizeException
   */
  void fill (const TypedMat<T> & matrix);

  /** \todo Decide if the << operator is better than a "fill" method; we'll have both for now */
  inline TypedMat<T> & operator << (const T & value) { fill (value); return (*this); }
  inline TypedMat<T> & operator << (const TypedMat<T> & matrix) { fill (matrix); return (*this); }

  /** \brief Perform scalar addition */
  TypedMat
  operator + (const T & operand) const;

  /** \brief Perform scalar subtraction */
  TypedMat
  operator - (const T & operand) const;

  /** \brief Perform scalar multiplication */
  TypedMat
  operator * (const T & operand) const;

  /** \brief Perform scalar division */
  TypedMat
  operator / (const T & operand) const;

  /** \brief Perform element-wise addition */
  TypedMat
  operator + (const TypedMat<T> & operand) const;

  /** \brief Perform element-wise subtraction */
  TypedMat
  operator - (const TypedMat<T> & operand) const;

  /** \brief Perform in-place element-wise addition */
  TypedMat &
  operator += (const TypedMat<T> & operand);

  /** \brief Perform in-place element-wise subtraction */
  TypedMat &
  operator -= (const TypedMat<T> & operand);

  /** \brief Perform in-place element-wise multiplication */
  TypedMat &
  operator *= (const TypedMat<T> & operand);

  /** \brief Perform in-place element-wise division */
  TypedMat &
  operator /= (const TypedMat<T> & operand);

protected:
  /** \brief A shared pointer to the matrix implementation */
  typename core::TypedMatImpl<T>::Ptr typed_matrix_ptr_;
};

// ///////////////////////////////////////////////////////////////////////////////
// /** \brief A const version of TypedMat 
//  *
//  * \see TypedMat
//  * \see Mat 
//  */
// template <typename T>
// class ConstTypedMat : public Mat
// {
//   friend class TypedMat<T>;

// public:
//   typedef pcl2::ConstRow<T> Row;

// private:
//   ConstTypedMat ();

// protected:
//   /** \brief Construct a ConstTypedMat around the provided MatImpl 
//    *
//    * \throws Throws a BadCastException if the provided MatImpl cannot be cast 
//    * to a const matrix of the specified type, T.
//    * \todo Make this throw an exception instead of an assertion failure
//    */
//   ConstTypedMat (core::MatImpl::Ptr matrix);

// public:
//   /** \brief Construct a ConstTypedMat from a generic Mat.  
//    *
//    * \throws Throws a BadCastException if the provided matrix cannot be cast to a
//    * const matrix of the specified type, T.
//    * \todo Make this throw an exception instead of an assertion failure
//    */
//   ConstTypedMat (const Mat & shared_matrix);

//   /** \brief Access an element in the matrix
//    *
//    * \param i The row of the element
//    * \param j The column of the element
//    * \return A const reference to the element in the ith row and jth column
//    */
//   const T & operator () (size_t i, size_t j) const;

//   Row operator () (size_t row_index) const;

//   ConstTypedMat operator () (const ConstTypedMat<int> & indices);

// protected:
//   /** A const shared pointer to the matrix implementation */
//   typename core::TypedMatImpl<T>::Ptr typed_matrix_ptr_;
// };

typedef TypedMat<int> MatI;
typedef TypedMat<float> MatF;
typedef TypedMat<double> MatD;
//typedef ConstTypedMat<int> ConstMatI;
//typedef ConstTypedMat<float> ConstMatF;
//typedef ConstTypedMat<double> ConstMatD;

template <typename T>
std::ostream& operator << (std::ostream& out, const pcl2::TypedMat<T> & mat)
{
  for (typename pcl2::Row<T> r = mat (0); r.hasNext (); r.advance ())
    out << r << std::endl;
  return (out);
}

// template <typename T>
// std::ostream& operator << (std::ostream& out, const pcl2::ConstTypedMat<T> & mat)
// {
//   for (typename pcl2::ConstRow<T> r = mat (0); r.hasNext (); r.advance ())
//     out << r << std::endl;
//   return (out);
// }

}

#endif
