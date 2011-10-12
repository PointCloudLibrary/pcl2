#include <iostream>
#include "pcl2/cloud.h"
#include "pcl2/float_array.h"
#include "pcl2/int_array.h"
#include "pcl2/kernels.h"

using namespace std;

void printChannelNames (const pcl2::Cloud & cloud)
{
  vector<string> channel_names = cloud.getChannelNames ();
  for (size_t i = 0; i < channel_names.size (); ++i)
    cout << " - " << channel_names[i] << endl;
}

void printFloatArrayRow (const pcl2::FloatArray & array, size_t i)
{
  cout << array (i, 0);
  for (size_t j = 1; j < array.cols (); ++j)
    cout << ", " << array (i, j);
  cout << endl;
}

typedef sse TURBO;
typedef TURBO::tfloat TURBOf;

float dot(const pcl2::FloatArray &a)
{
  const float *x = &a(0,0);
  const float *y = &a(0,1);
  const float *z = &a(0,2);

  float m[3] = { 1, 0, 0 };
  dot3<TURBO> wrk(m);
  return xyz_sum((TURBOf*)x, (TURBOf*)y, (TURBOf*)z, a.rows(), wrk);
}

int main (int argc, char ** argv)
{
  // Creating empty cloud
  pcl2::Cloud cloud;
  cout << "Created cloud: cloud.channelCount () = " << cloud.channelCount () << ", "
       << "cloud.empty () = " << cloud.empty () << endl;

  // Creating array
  pcl2::FloatArray::Ptr a = pcl2::FloatArray::create (100, 3);
  cout << "Created array: " << a->rows () << " by " << a->cols () << endl;

  // Adding channel to cloud
  cloud.insert ("xyz", *a);
  cout << "Added array to cloud: cloud.channelCount () = " << cloud.channelCount () << endl;
  cout << "List of channels contained in cloud:" << endl;
  printChannelNames (cloud);

  // Accessing/editing channel data
  pcl2::Array & array_ref = cloud["xyz"];
  cout << "Accessed array from cloud: " << array_ref.rows () << " by " << array_ref.cols () << endl;

  cout << "Filling in point data..." << endl;
  pcl2::FloatArray & pts = cloud["xyz"];
  for (size_t i = 0; i < pts.rows (); ++i)
    for (size_t j = 0; j < pts.cols (); ++j)
      pts (i, j) = (i+1)+0.1*(j+1);
  cout << "...done." << endl;

  for (size_t i = 9; i < pts.rows (); i += 10)
  {
    cout << "The " << (i+1) << "th row of pts: " 
         << pts (i,0) << ", " << pts (i, 1) << ", " << pts (i,2) << endl;
  }

  // Run a simple kernel
  cout << "dot " << dot(cloud["xyz"]) << endl;

  // Creating an IntArray and adding it to a cloud
  pcl2::IntArray::Ptr ia = pcl2::IntArray::create (100, 3);
  cloud.insert ("rgb", *ia);
  cout << "Added 'rgb' channel to cloud; list of channels:" << endl;
  printChannelNames (cloud);

  // Merging channels from another cloud  
  pcl2::Cloud cloud2;
  cout << "Created cloud2: cloud2.channelCount () = " << cloud2.channelCount () << endl;

  cloud2.insert ("normals", *pcl2::FloatArray::create (100, 3));
  cloud2.insert ("curvature", *pcl2::FloatArray::create (100, 1));
  cout << "Added 'normals' and 'curvature' channels to cloud2; list of channels:" << endl;
  printChannelNames (cloud2);

  cloud += cloud2;
  cout << "Added channels from cloud2 to cloud; list of channels:" << endl;
  printChannelNames (cloud);

  // Removing a channel
  cloud.remove ("curvature");
  cout << "Removed 'curvature' channel from cloud; list of channels:" << endl;
  printChannelNames (cloud);

  // Copy
  pcl2::Cloud cloud_copy = cloud.copy ();
  dynamic_cast<pcl2::FloatArray&> (cloud_copy["xyz"]) (0,0) = 100;
  cout << "Made a deep copy of cloud and modified it" << endl;
  cout << "First row of the original: " << endl;
  printFloatArrayRow (cloud["xyz"], 0);
  cout << "First row of the copy: " << endl;
  printFloatArrayRow (cloud_copy["xyz"], 0);

  // Inserting channel with incompatible size
  cout << "Trying to insert channel with incompatible size..." << endl;
  try
  {
    cloud.insert ("fail", *pcl2::FloatArray::create (99, 3));
  }
  catch (pcl2::Exception &e)
  {
    cout << "Caught " << e.what () << endl;
  }

  // Accessing a non-existent channel
  cout << "Trying to access channel that isn't there..." << endl;
  try
  {
    pcl2::FloatArray & curv = cloud["curvature"];
  }
  catch (pcl2::Exception &e)
  {
    cout << "Caught " << e.what () << endl;
  }

  // Accessing a FloatArray as an IntArray and vice versa
  cout << "Trying to access a FloatArray as an IntArray and vice versa" << endl;
  try
  {
    pcl2::FloatArray & a = cloud["rgb"];
  }
  catch (pcl2::Exception &e)
  {
    cout << "Caught " << e.what () << endl;
  }
  try
  {
    pcl2::IntArray & a = cloud["xyz"];
  }
  catch (pcl2::Exception &e)
  {
    cout << "Caught " << e.what () << endl;
  }

  return (0);
}
