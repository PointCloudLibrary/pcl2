/** \file stats.h 
 * \brief Declares functions for computing statistical properties of a matrix
 */

#ifndef PCL2_STATS_H
#define PCL2_STATS_H

#include "pcl2/typed_matrix.h"

namespace pcl2
{

template <typename T>
TypedMat<T>
computeMean (const TypedMat<T> & input);

template <typename T>
TypedMat<T>
computeCovariance (const TypedMat<T> & input);

template <typename T>
TypedMat<T>
computeCovariance (const TypedMat<T> & input, const TypedMat<T> & mean);

template <typename T>
void
computeMeanAndCovariance (const TypedMat<T> & input, TypedMat<T> & mean, TypedMat<T> & covariance);


}
#endif
