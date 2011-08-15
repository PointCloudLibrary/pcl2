/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/channel.h>
#include <pcl/const_channel.h>
#include <boost/shared_ptr.hpp>
#include <stdio.h>
#include <string>
#include <vector>
#include <map>
#include <iostream>

using namespace pcl;
using namespace std;


int main (int argc, char** argv)
{
  // Make a PointCloud of 3 points
  PointCloud cloud (3);

  // Create some new point data
  cloud.createChannel<PointXYZ> ("xyz");
  cloud.createChannel<ColorRGB> ("rgb");

  Channel<PointXYZ> xyz_channel = cloud["xyz"];

  //*
  for (size_t i = 0; i < cloud.size (); ++i)
  {
	  //cloud["xyz"][i] =  PointXYZ(); // this does not compile!! this is desired behaviour since we dont want to
                         // access the map each time we acces a single point!!!
    xyz_channel[i] = PointXYZ();
  }
  //*/

  // Fill them in with some values

  Channel<PointXYZ>  xyzChannel = cloud["xyz"];
  Channel<ColorRGB>  rgbChannel = cloud["rgb"];

  for (size_t i = 0; i < cloud.size (); ++i)
  {
    PointXYZ & p = xyzChannel[i];
    p.x = 10*i + 1;
    p.y = 10*i + 2;
    p.z = 10*i + 3;

    ColorRGB & c = rgbChannel[i];
    c.r = i;
    c.g = i + 100;
    c.b = i + 200;
  }

  // Now make the cloud bigger
  cloud.resize (5);

  // Let's add a third type of data to the mix

  cloud.createChannel<Normal> ("normals");
  Channel<Normal>  normals = cloud["normals"];

  for (size_t i = 0; i < cloud.size (); ++i)
  {
    const PointXYZ & p = xyzChannel[i];

    normals[i].x = 0.1*p.x;
    normals[i].y = 0.1*p.y;
    normals[i].z = 0.1*i;
  }
// Finally, we'll print out some values
  for (size_t i = 0; i < cloud.size (); ++i)
  {
    const PointXYZ & p = xyzChannel[i];
    const Normal &n = normals[i];
    const ColorRGB & c = rgbChannel[i];
    printf("%zu:\n", i);
    printf("  point: (%f, %f, %f)\n", p.x, p.y, p.z);
    printf("normals: (%f, %f, %f)\n", n.x, n.y, n.z);
    printf("  color: (%d, %d, %d)\n", c.r, c.g, c.b);
  }

  const PointCloud const_cloud = cloud;
  ConstChannel<PointXYZ> const_xyz_channel = const_cloud["xyz"];
  ConstChannel<Normal>  const_normals = const_cloud["normals"];
  ConstChannel<ColorRGB>  const_rgbChannel = const_cloud["rgb"];
  ConstChannel<Normal> const_normals2 = normals;
  ConstChannel<PointXYZ> const_xyz_channel2 = cloud["xyz"];
  // Finally, we'll print out some values
  for (size_t i = 0; i < cloud.size (); ++i)
  {
    const PointXYZ & p = const_xyz_channel[i];
    const Normal &n = const_normals2[i];
    const ColorRGB & c = const_rgbChannel[i];
    printf("%zu:\n", i);
    printf("  point: (%f, %f, %f)\n", p.x, p.y, p.z);
    printf("normals: (%f, %f, %f)\n", n.x, n.y, n.z);
    printf("  color: (%d, %d, %d)\n", c.r, c.g, c.b);
  }

  // Oh, and I guess we should test a few error cases
  printf("The following lines should be error messages...\n");

  try
  {
    Channel<PointXYZ> type_mismatch = cloud["normals"];
  }
  catch (PCLException& e)
  {
  	cout << e.what () << endl;
  };

  try
  {
    ConstChannel<PointXYZ> type_mismatch = cloud["normals"];
  }
  catch (PCLException& e)
  {
  	cout << e.what () << endl;
  };

  try
  {
    Channel<PointXYZ>  bad_key = cloud["not_a_key"];
  }
  catch (PCLException& e)
  {
    cout << e.what () << endl;
  };
  try
  {
    ConstChannel<PointXYZ>  bad_key = cloud["not_a_key"];
  }
  catch (PCLException& e)
  {
    cout << e.what () << endl;
  };

  try
  {
    cloud.createChannel<PointXYZ> ("xyz");
  }
  catch (PCLException& e)
  {
    cout << e.what () << endl;
  };

  return (0);
}
