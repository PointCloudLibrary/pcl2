/** \file transform.h 
 * \brief Declares functions for transforming point clouds
 */

#ifndef PCL2_TRANSFORM_H
#define PCL2_TRANSFORM_H

#include "pcl2/cloud.h"
#include "pcl2/typed_matrix.h"

namespace pcl2
{

Cloud transform (Cloud cloud, const MatF & tform);

}

#endif // PCL2_TRANSFORM_H
