/** \file stats.hpp
 * \brief Defines functions for computing statistical properties of a matrix
 */

#include "pcl2/stats.h"
#include "pcl2/row.h"
#include "pcl2/eigen_matrix.h"
#include "pcl2/math.h"

template<typename T>
pcl2::TypedMat<T>
pcl2::computeMean (const TypedMat<T> & input)
{
  EigenMat<T> sum = computeSum (input);
  return (sum / input.rows ());
}

template<typename T>
pcl2::TypedMat<T>
pcl2::computeCovariance (const TypedMat<T> & input)
{
  TypedMat<T> mean = computeMean (input);
  return (computeCovariance (input, mean));
}

template<typename T>
pcl2::TypedMat<T>
pcl2::computeCovariance (const TypedMat<T> & input, const TypedMat<T> & mean)
{
  EigenMat<T> unnormalized_cov (input.cols (), input.cols ());
  unnormalized_cov.fill (0);
  for (typename TypedMat<T>::Row row = input (0); row.hasNext (); row.advance ())
  {
    TypedMat<T> demeaned_row = row - mean;
    unnormalized_cov += computeOuterProduct (demeaned_row, demeaned_row);
  }
  return (unnormalized_cov / (input.rows () - 1));
}

template<typename T>
void
pcl2::computeMeanAndCovariance (const TypedMat<T> & input, TypedMat<T> & mean, TypedMat<T> & covariance)
{
  mean = computeMean (input);
  covariance = computeCovariance (input, mean);
}
