#include "pcl2/impl/eigen_matrix.hpp"

template class pcl2::TypedMat<int>;
template class pcl2::TypedMat<float>;
template class pcl2::TypedMat<double>;

template class pcl2::EigenMat<int>;
template class pcl2::EigenMat<float>;
template class pcl2::EigenMat<double>;

/// \todo Move this!
template std::ostream& pcl2::operator << (std::ostream&, const pcl2::TypedMat<int> &);
template std::ostream& pcl2::operator << (std::ostream&, const pcl2::TypedMat<float> &);
template std::ostream& pcl2::operator << (std::ostream&, const pcl2::TypedMat<double> &);
