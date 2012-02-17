/** \file math.h 
 * \brief Declares functions for performing basic arithmethic on matrices
 */

#ifndef PCL2_MATH_H
#define PCL2_MATH_H

#include "pcl2/typed_matrix.h"

namespace pcl2
{

template <typename T>
TypedMat<T>
computeSum (const TypedMat<T> & input);

template <typename T>
TypedMat<T>
computeProduct (const TypedMat<T> & input);

template <typename T>
TypedMat<T>
computeCumulativeSum (const TypedMat<T> & input);

template <typename T>
TypedMat<T>
computeCumulativeProduct (const TypedMat<T> & input);

template <typename T>
TypedMat<T>
computeOuterSum (const TypedMat<T> & vec1, const TypedMat<T> & vec2);

template <typename T>
TypedMat<T>
computeOuterProduct (const TypedMat<T> & vec1, const TypedMat<T> & vec2);

// --- Move these to a different file ---
template <typename T>
TypedMat<T>
computeEigenvalues3x3 (const TypedMat<T> & input);

template <typename T>
TypedMat<T>
computeEigenvectors3x3 (const TypedMat<T> & input);

template <typename T>
void
computeEigendecomposition3x3 (const TypedMat<T> & input, TypedMat<T> & eigenvalues, TypedMat<T> & eigenvectors);



}
#endif
