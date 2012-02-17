/** \file math.h 
 * \brief Instantiate functions for performing basic arithmethic on matrices
 */

#include "pcl2/impl/math.hpp"

template pcl2::TypedMat<int> pcl2::computeSum<int> (const TypedMat<int> &);
template pcl2::TypedMat<float> pcl2::computeSum<float> (const TypedMat<float> &);
template pcl2::TypedMat<double> pcl2::computeSum<double> (const TypedMat<double> &);

template pcl2::TypedMat<int> pcl2::computeProduct<int> (const TypedMat<int> &);
template pcl2::TypedMat<float> pcl2::computeProduct<float> (const TypedMat<float> &);
template pcl2::TypedMat<double> pcl2::computeProduct<double> (const TypedMat<double> &);

template pcl2::TypedMat<int> pcl2::computeCumulativeSum<int> (const TypedMat<int> &);
template pcl2::TypedMat<float> pcl2::computeCumulativeSum<float> (const TypedMat<float> &);
template pcl2::TypedMat<double> pcl2::computeCumulativeSum<double> (const TypedMat<double> &);

template pcl2::TypedMat<int> pcl2::computeCumulativeProduct<int> (const TypedMat<int> &);
template pcl2::TypedMat<float> pcl2::computeCumulativeProduct<float> (const TypedMat<float> &);
template pcl2::TypedMat<double> pcl2::computeCumulativeProduct<double> (const TypedMat<double> &);

template pcl2::TypedMat<int> pcl2::computeOuterSum<int> (const TypedMat<int> &, const TypedMat<int> &);
template pcl2::TypedMat<float> pcl2::computeOuterSum<float> (const TypedMat<float> &, const TypedMat<float> &);
template pcl2::TypedMat<double> pcl2::computeOuterSum<double> (const TypedMat<double> &, const TypedMat<double> &);

template pcl2::TypedMat<int> pcl2::computeOuterProduct<int> (const TypedMat<int> &, const TypedMat<int> &);
template pcl2::TypedMat<float> pcl2::computeOuterProduct<float> (const TypedMat<float> &, const TypedMat<float> &);
template pcl2::TypedMat<double> pcl2::computeOuterProduct<double> (const TypedMat<double> &, const TypedMat<double> &);

template pcl2::TypedMat<float> pcl2::computeEigenvalues3x3<float> (const TypedMat<float> &);
template pcl2::TypedMat<double> pcl2::computeEigenvalues3x3<double> (const TypedMat<double> &);

template pcl2::TypedMat<float> pcl2::computeEigenvectors3x3<float> (const TypedMat<float> &);
template pcl2::TypedMat<double> pcl2::computeEigenvectors3x3<double> (const TypedMat<double> &);

template void pcl2::computeEigendecomposition3x3<float> (const TypedMat<float> &, TypedMat<float> &, TypedMat<float> &);
template void pcl2::computeEigendecomposition3x3<double> (const TypedMat<double> &, TypedMat<double> &, TypedMat<double> &);
