/** \file neighbors.cpp 
 * \brief Defines functions declared in neighbors.h
 */

#include "pcl2/search/neighbors.h"
#include "pcl2/eigen_matrix.h"
#include "pcl2/search/kdtree.h"

#include <pcl/kdtree/kdtree_flann.h>


int
pcl2::findNearestNeighbor (const Cloud & cloud, const MatF & query)
{
  // Convert point cloud
  MatF xyz = cloud["xyz"];
  assert (xyz.rows () >= 1);
  assert (xyz.cols () == 3);
  pcl::PointCloud<pcl::PointXYZ>::Ptr input (new pcl::PointCloud<pcl::PointXYZ>);
  input->width = cloud.size ();
  input->height = 1;
  input->is_dense = false;
  input->points.resize (cloud.size ());
  for (size_t i = 0; i < xyz.rows (); ++i)
  {
    input->points[i].x = xyz (i, 0);
    input->points[i].y = xyz (i, 1);
    input->points[i].z = xyz (i, 2);
  }

  // Convert query point
  assert (query.rows () == 1);
  assert (query.cols () == 3);
  pcl::PointXYZ q;
  q.x = query (0, 0);
  q.y = query (0, 1);
  q.z = query (0, 2);
  
  // Perform neighbor search
  pcl::KdTreeFLANN<pcl::PointXYZ> tree;
  tree.setInputCloud (input);

  std::vector<int> indices (1);
  std::vector<float> dists (1);
  int k = (size_t) tree.nearestKSearch (q, 1, indices, dists);
  assert (k == 1);

  return (indices[0]);
}

pcl2::TypedMat<int>
pcl2::findKNearestNeighbors (const Cloud & cloud, const MatF & query, size_t k)
{
  // Convert point cloud
  MatF xyz = cloud["xyz"];
  assert (xyz.rows () >= 1);
  assert (xyz.cols () == 3);
  pcl::PointCloud<pcl::PointXYZ>::Ptr input (new pcl::PointCloud<pcl::PointXYZ>);
  input->width = cloud.size ();
  input->height = 1;
  input->is_dense = false;
  input->points.resize (cloud.size ());
  for (size_t i = 0; i < xyz.rows (); ++i)
  {
    input->points[i].x = xyz (i, 0);
    input->points[i].y = xyz (i, 1);
    input->points[i].z = xyz (i, 2);
  }

  // Convert query point
  assert (query.rows () == 1);
  assert (query.cols () == 3);
  pcl::PointXYZ q;
  q.x = query (0, 0);
  q.y = query (0, 1);
  q.z = query (0, 2);
  
  // Perform neighbor search
  pcl::KdTreeFLANN<pcl::PointXYZ> tree;
  tree.setInputCloud (input);

  std::vector<int> indices (k);
  std::vector<float> dists (k);
  k = (size_t) tree.nearestKSearch (q, k, indices, dists);
  assert (k == indices.size ());

  // Convert output
  EigenMat<int> output (k, 1);
  for (size_t i = 0; i < indices.size (); ++i)
    output (i, 0) = indices[i];

  return (output);
}

pcl2::TypedMat<int>
pcl2::findFixedRadiusNeighbors (Cloud & cloud, const MatF & query, float r)
{
  // Search in the xyx channel
  MatF xyz = cloud["xyz"];

  // Look for a pre-computed spatial search index
  SpatialIndex<float>::Ptr spatial_index = xyz.getSpatialIndex ();
  if (!spatial_index)
  {
    // If no spatial index is present, create one
    /// \todo: Replace this with buildDefaultSpatialIndex, getSpatialIndex
    spatial_index.reset (new search::KDTree<float> ());
    xyz.buildSpatialIndex (spatial_index); 
  }

  // Use the search index to perform the radius search and return the neigbhor indices
  return (spatial_index->findFixedRadiusNeighbors (query, r));
}


pcl2::Neighborhood
pcl2::computeFixedRadiusNeighborhood (Cloud & cloud, const MatF & query, float r)
{
  // Convert point cloud
  MatF xyz = cloud["xyz"];
  assert (xyz.rows () >= 1);
  assert (xyz.cols () == 3);
  pcl::PointCloud<pcl::PointXYZ>::Ptr input (new pcl::PointCloud<pcl::PointXYZ>);
  input->width = cloud.size ();
  input->height = 1;
  input->is_dense = false;
  input->points.resize (cloud.size ());
  for (size_t i = 0; i < xyz.rows (); ++i)
  {
    input->points[i].x = xyz (i, 0);
    input->points[i].y = xyz (i, 1);
    input->points[i].z = xyz (i, 2);
  }

  // Convert query point
  assert (query.rows () == 1);
  assert (query.cols () == 3);
  pcl::PointXYZ q;
  q.x = query (0, 0);
  q.y = query (0, 1);
  q.z = query (0, 2);
  
  // Perform neighbor search
  pcl::KdTreeFLANN<pcl::PointXYZ> tree;
  tree.setInputCloud (input);

  std::vector<int> idx_vec;
  std::vector<float> dist_vec;
  size_t k = (size_t) tree.radiusSearch (q, r, idx_vec, dist_vec);
  assert (k == idx_vec.size ());

  // Convert output
  EigenMat<int> neighbor_indices (k, 1);
  EigenMat<float> squared_distances (k, 1);
  for (size_t i = 0; i < k; ++i)
  {
    neighbor_indices (i, 0) = idx_vec[i];
    squared_distances (i, 0) = dist_vec[i];
  }

  //Cloud neighborhood = cloud (neighbor_indices);
  Neighborhood neighborhood (cloud, neighbor_indices);
  neighborhood.insert ("dist", squared_distances);
  return (neighborhood);
}
