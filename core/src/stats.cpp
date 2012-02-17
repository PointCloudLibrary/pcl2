/** \file stats.cpp 
 * \brief Instantiate functions for computing statistical properties of a matrix
 */

#include "pcl2/impl/stats.hpp"

template pcl2::TypedMat<float> pcl2::computeMean<float> (const TypedMat<float> &);
template pcl2::TypedMat<double> pcl2::computeMean<double> (const TypedMat<double> &);

template pcl2::TypedMat<float> pcl2::computeCovariance<float> (const TypedMat<float> &);
template pcl2::TypedMat<double> pcl2::computeCovariance<double> (const TypedMat<double> &);

template pcl2::TypedMat<float> pcl2::computeCovariance<float> (const TypedMat<float> &, const TypedMat<float> &);
template pcl2::TypedMat<double> pcl2::computeCovariance<double> (const TypedMat<double> &, const TypedMat<double> &);

template void pcl2::computeMeanAndCovariance<float> (const TypedMat<float> &, TypedMat<float> &, TypedMat<float> &);
template void pcl2::computeMeanAndCovariance<double> (const TypedMat<double> &, TypedMat<double> &, TypedMat<double> &);
