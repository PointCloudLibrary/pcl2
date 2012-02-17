#include <iostream>
#include "pcl2/typed_matrix.h"
#include "pcl2/eigen_matrix.h"
#include "pcl2/row.h"
#include "pcl2/io/io.h"
#include "pcl2/search/neighbors.h"
#include "pcl2/registration/fit.h"

int 
main (int argc, char ** argv)
{
  // Load a point cloud
  pcl2::Cloud bunny = pcl2::loadCloud ("../data/bunny.pcd");

  // Get the xyz channel from the cloud
  pcl2::MatF pts = bunny["xyz"];

  // Create a new mat to store the surface normals in
  pcl2::EigenMat<float> normals (bunny.size (), 3);

  // Loop over every point in the cloud
  pcl2::MatF::Row q_normal = normals (0);
  for (pcl2::MatF::Row q = pts (0); q.hasNext (); q.advance (), q_normal.advance ())
  {
    // Find the neighbors
    const float radius = 0.03;
    pcl2::MatI nn_idx = pcl2::findFixedRadiusNeighbors (bunny, q, radius);

    // Fit a plane to the neighborhood and store the result
    q_normal << pcl2::fitPlaneLLS (pts (nn_idx));
  }

  // Add the new normals channel to the cloud...
  bunny += pcl2::Cloud ("normals", normals);
  // ... and add a dummy curvature channel, too, because pcd_viewer expects one
  bunny += pcl2::Cloud ("curvature", pcl2::createZeros<float> (bunny.size (), 1));

  // Save the cloud for visualization 
  pcl2::saveCloud ("./bunny_out.pcd", bunny);

  return (0);
}
