/** \file normals.h 
 * \brief Declares functions for estimating surface normals
 */

#ifndef PCL2_NEIGHBORS_H
#define PCL2_NEIGHBORS_H

#include "pcl2/cloud.h"
#include "pcl2/typed_matrix.h"


namespace pcl2
{

Cloud estimateSurfaceNormals (const Cloud & cloud, float nhood_radius);
Cloud estimateSurfaceNormals (const MatF & points, float nhood_radius);

Cloud estimateSurfaceNormals (const MatF & surface_points, const MatF & query_points, float nhood_radius);

Cloud estimateSurfaceNormalsKNN (const Cloud & cloud, int k);

// 
// Cloud estimateSurfaceNormals (Cloud cloud, NeighborhoodSpec ns);
// e.g., cld += estimateSurfaceNormals (cld, FixedRadius (0.1));
//       cld += estimateSurfaceNormals (cld, KNearest (0.1));

}
