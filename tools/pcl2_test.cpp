#include <iostream>
#include <assert.h>
#include "pcl2/cloud.h"
#include "pcl2/matrix.h"
#include "pcl2/eigen_matrix.h"
#include "pcl2/io/io.h"
#include "pcl2/search/neighbors.h"
#include "pcl2/registration/fit.h"

using namespace std;

pcl2::TypedMat<int> createIndices (int i1, int i2, int step=1)
{
  assert (step > 0);
  assert (i2 >= i1);
  size_t n = ceil (1.0 * (i2-i1) / step);

  pcl2::EigenMat<int> indices (n, 1);
  int index = i1;
  for (size_t i = 0; i < n; ++i)
  {
    indices (i, 0) = index;
    index += step;
  }
  return (indices);
}

void printChannelNames (const pcl2::Cloud & cloud)
{
  vector<string> channel_names = cloud.getChannelNames ();
  for (size_t i = 0; i < channel_names.size (); ++i)
    cout << " - " << channel_names[i] << endl;
}

void print (const pcl2::Mat & matrix)
{
  pcl2::MatF matf = matrix;
  for (size_t i = 0; i < matf.rows (); ++i)
  {
    cout << matf (i, 0);
    for (size_t j = 1; j < matf.cols (); ++j)
      cout << ", " << matf (i, j);
    cout << endl;
  }
}

void printRow (const pcl2::Mat & matrix, size_t i)
{
  pcl2::MatF matf = matrix;
  cout << matf (i, 0);
  for (size_t j = 1; j < matf.cols (); ++j)
    cout << ", " << matf (i, j);
  cout << endl;
}

void examples ()
{
  // First, I'll create an empty Cloud
  pcl2::Cloud cloud;

  // This cloud is just an empty container, so it won't have any channels.
  cout << "Created a cloud with " << cloud.channelCount () << "channels.  "
       << "cloud.empty () = " << cloud.empty () << endl;

  // Now I'll load some data from disk and store it in the cloud.
  cloud = pcl2::loadCloud ("bunny.pcd");

  // It will now have some channels
  cout << "This cloud has " << cloud.channelCount () << "channels and " << cloud.size () << " points." << endl;

  // I'll get a list of the channel names and print them out
  vector<string> channel_names = cloud.getChannelNames ();
  cout << "The channel names are: " << endl;  
  for (size_t i = 0; i < channel_names.size (); ++i)
    cout << " - " << channel_names[i] << endl;

  // Now I'll create a new Nx3 matrix
  int n = cloud.size ();
  pcl2::EigenMat<float> a (n, 3);

  // 

}

int main (int argc, char ** argv)
{
  // Creating empty cloud
  pcl2::Cloud cloud;
  cout << "Created cloud: cloud.channelCount () = " << cloud.channelCount () << ", "
       << "cloud.empty () = " << cloud.empty () << endl;

  // Creating matrix
  pcl2::EigenMat<float> a (100, 3);
  cout << "Created matrix: " << a.rows () << " by " << a.cols () << endl;

  // Adding channel to cloud
  cloud.insert ("xyz", a);
  cout << "Added matrix to cloud: cloud.channelCount () = " << cloud.channelCount () << endl;
  cout << "List of channels contained in cloud:" << endl;
  printChannelNames (cloud);

  // Accessing/editing channel data
  pcl2::Mat & matrix_ref = cloud["xyz"];
  cout << "Accessed matrix from cloud: " << matrix_ref.rows () << " by " << matrix_ref.cols () << endl;

  cout << "Filling in point data..." << endl;
  pcl2::MatF pts = cloud["xyz"];
  for (size_t i = 0; i < pts.rows (); ++i)
    for (size_t j = 0; j < pts.cols (); ++j)
      pts (i, j) = (i+1)+0.1*(j+1);
  cout << "...done." << endl;

  for (size_t i = 0; i < 10; ++i)
  {
    cout << "The " << (i+1) << "th row of pts: " 
         << pts (i,0) << ", " << pts (i, 1) << ", " << pts (i,2) << endl;
  }
  cout << "..." << endl;

  // Creating a view
  pcl2::TypedMat<int> idx = createIndices (0, 100, 10);
  pcl2::MatF view = pts (idx);
  for (size_t i = 0; i < view.rows (); ++i)
  {
    cout << "The " << (i+1) << "th row of view: " 
         << view (i,0) << ", " << view (i, 1) << ", " << view (i,2) << endl; 
  }
  
  // Creating a copy of a view
  pcl2::Mat view_copy = view.copy ();
  print (view_copy);

  // Changing data will change the view but not the copy
  pts (0,0) = 1000.0;
  printRow (pts, 0);
  printRow (view, 0);
  printRow (view_copy, 0);


  pcl2::Cloud bunny = pcl2::loadCloud ("../data/bunny.pcd");

  pcl2::EigenMat<float> query_point (1, 3);
  query_point (0,0) = -0.01;
  query_point (0,1) = 0.1;
  query_point (0,2) = 0.04;


  pcl2::Cloud nhood;
  if (false)
  {
    //pcl2::TypedMat<int> nn_idx = findKNearestNeighbors (bunny, query_point, 20);
    pcl2::TypedMat<int> nn_idx = pcl2::findFixedRadiusNeighbors (bunny, query_point, 0.025);
    nhood = bunny (nn_idx);
  }
  else
  {
    nhood = pcl2::computeFixedRadiusNeighborhood (bunny, query_point, 0.05);
  }
  pcl2::saveCloud ("foo.pcd", nhood);

  cout << "before!" << endl;
  pcl2::MatF plane = pcl2::fitPlaneLLS (nhood);
  cout << "after!" << endl;

  pcl2::EigenMat<float> plane_normal (1, 3);
  pcl2::EigenMat<float> curvature (1,1);
  for (int i = 0; i < 3; ++i) plane_normal (0, i) = plane (0, i);
  pcl2::Cloud surf_norm;
  surf_norm.insert ("xyz", query_point);
  surf_norm.insert ("normals", plane_normal);
  surf_norm.insert ("curvature", curvature);
  pcl2::saveCloud ("foo2.pcd", surf_norm);

  return (0);
}
