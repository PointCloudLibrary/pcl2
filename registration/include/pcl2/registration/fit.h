/** \file fit.h 
 * \brief Declares example functions for fitting planes to point clouds
 */

/// \todo Move this file (fit.h) to a different module (not registration)

#ifndef PCL2_FIT_H
#define PCL2_FIT_H

#include "pcl2/cloud.h"
#include "pcl2/typed_matrix.h"

namespace pcl2
{

/** \brief Fit a plane to a given cloud of 3D points using Linear Least Squares (LLS)
 * \param cloud A cloud containing an "xyz" channel of 3D points
 * \return A MatF containing the 4 coefficients of the best fitting plane, 
 * given in \f$ c_0 x + c_1 y + c_2 z = c_3 \f$ form
 */
MatF fitPlaneLLS (const Cloud & cloud);

/** \brief Fit a plane to a channel of 3D points using Linear Least Squares (LLS)
 * \param points An n by 3 MatF representing n 3D points
 * \return A MatF containing the 4 coefficients of the best fitting plane, 
 * given in \f$ c_0 x + c_1 y + c_2 z = c_3 \f$ form
 */
template <typename T>
TypedMat<T> fitPlaneLLS (const TypedMat<T> & points);

/** \brief Fit a plane to a given cloud of 3D points using Random Sample Consensus (RANSAC)
 * \param cloud A cloud containing an "xyz" channel of 3D points
 * \param inlier_threshold The maximum point-to-plane distance at which a point will be considered an inlier
 * \return A MatF containing the 4 coefficients of the best fitting plane, 
 * given in \f$ c_0 x + c_1 y + c_2 z = c_3 \f$ form
 */
MatF fitPlaneRANSAC (const Cloud & cloud, float inlier_threshold);

/** \brief Fit a plane to a channel of 3D points using Random Sample Consensus (RANSAC)
 * \param points An n by 3 MatF representing n 3D points
 * \param inlier_threshold The maximum point-to-plane distance at which a point will be considered an inlier
 * \return A MatF containing the 4 coefficients of the best fitting plane, 
 * given in \f$ c_0 x + c_1 y + c_2 z = c_3 \f$ form
 */
template <typename T>
TypedMat<T> fitPlaneRANSAC (const TypedMat<T> & points, float inlier_threshold);

}

#endif
