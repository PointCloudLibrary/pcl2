/** \file neighbors.h 
 * \brief Declares functions for finding nearest neighbors
 */

#ifndef PCL2_NEIGHBORS_H
#define PCL2_NEIGHBORS_H

#include "pcl2/cloud.h"
#include "pcl2/typed_matrix.h"


namespace pcl2
{

/** \todo Document this class */
class Neighborhood : public Cloud
{
public:
  Neighborhood (Cloud & cloud, const TypedMat<int> & indices) : Cloud (cloud (indices)), indices_ (indices)
  {

  }
protected:
  MatI indices_;
};

/** \brief Find the nearest neighbor to a query point in a given cloud 
 * \param cloud An input cloud containing a 3D "xyz" channel
 * \param query A 3D query point
 * \return The index of the nearest neighbor.
 * \see http://en.wikipedia.org/wiki/Nearest_neighbor_search#k-nearest_neighbor
 */
int findNearestNeighbor (const Cloud & cloud, const MatF & query);

/** \brief Find the k nearest neighbors that surround a query point in a given cloud 
 * \param cloud An input cloud containing a 3D "xyz" channel
 * \param query A 3D query point
 * \param k the number of nearest neighbors to return
 * \return A MatI containing the indices of the k nearest neighbors.
 * \see http://en.wikipedia.org/wiki/Nearest_neighbor_search#k-nearest_neighbor
 */
MatI findKNearestNeighbors (const Cloud & cloud, const MatF & query, size_t k);

/** \brief Find all points within a specified radius surrounding a query point in a given cloud 
 * \param cloud An input cloud containing a 3D "xyz" channel
 * \param query A 3D query point
 * \param r the radius of the neighborhood
 * \return A MatI containing the indices of all points that fell within the given radius
 * \see http://en.wikipedia.org/wiki/Fixed-radius_near_neighbors
 * \see computeFixedRadiusNeighborhood
 */
MatI findFixedRadiusNeighbors (Cloud & cloud, const MatF & query, float r);

/** \brief Compute the subset points that fall within a specified radius surrounding a query point in a given cloud 
 * \param cloud An input cloud containing a 3D "xyz" channel
 * \param query A 3D query point
 * \param r the radius of the neighborhood
 * \return A neighborhood of points that lie within the given radius around the query point
 * \see http://en.wikipedia.org/wiki/Fixed-radius_near_neighbors
 * \see findFixedRadiusNeighbors
 */
Neighborhood computeFixedRadiusNeighborhood (Cloud & cloud, const MatF & query, float r);

}

#endif
