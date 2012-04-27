/**
 * Software License Agreement (BSD License)
 * 
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2009-2012, Willow Garage, Inc.
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met: 
 * 
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Willow Garage, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/** \file conversions.cpp 
 * \brief Defines functions declared in conversions.h
 */

#include "pcl2/conversions.h"
#include "pcl2/eigen_matrix.h"

pcl::PointCloud<pcl::PointXYZ>::Ptr
pcl2::convertToPointCloudXYZ (const pcl2::MatF & xyz)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr output (new pcl::PointCloud<pcl::PointXYZ>);
  output->width = xyz.rows ();
  output->height = 1;
  output->is_dense = false;
  output->points.resize (xyz.rows ());
  for (size_t i = 0; i < xyz.rows (); ++i)
  {
    output->points[i].x = xyz (i, 0);
    output->points[i].y = xyz (i, 1);
    output->points[i].z = xyz (i, 2);
  }

  return (output);
}

boost::shared_ptr<sensor_msgs::PointCloud2>
pcl2::convertCloudToPointCloud2 (const pcl2::Cloud & input)
{
  // Get a listing of the channels in the cloud
  std::vector<std::string> channels = input.getChannelNames ();
  for (size_t i = 0; i < channels.size (); ++i)
    if (channels[i].compare ("xyz") == 0)
      swap (channels[i], channels[0]);

  // Determine the number of bytes in each point
  int point_step = 0;
  for (size_t i = 0; i < channels.size (); ++i)
  {
    pcl2::Mat channel = input[channels[i]];
    point_step += 4*channel.cols ();
  }

  // Create an empty PointCloud2
  boost::shared_ptr<sensor_msgs::PointCloud2> pc2 (new sensor_msgs::PointCloud2);

  // Fill in header data
  pc2->height = 1;
  pc2->width = input.size ();
  pc2->point_step = point_step;
  pc2->row_step = pc2->point_step * pc2->width;
  pc2->is_dense = false;

  // Resize the PointCloud2's data and get a pointer to it
  pc2->data.resize (pc2->height * pc2->row_step);
  float * data = reinterpret_cast<float *> (&pc2->data[0]); // Warning: Assumes all channels are floats

  // For each channel...
  int offset = 0;
  for (size_t i = 0; i < channels.size (); ++i)
  {
    pcl2::Mat channel = input[channels[i]];

    std::vector<std::string> fields;
    std::vector<size_t> counts;
    if (channels[i].compare ("xyz") == 0)
    {
      fields.push_back ("x");
      fields.push_back ("y");
      fields.push_back ("z");
      counts.push_back (1);
      counts.push_back (1);
      counts.push_back (1);
    }
    else if (channels[i].compare ("normals") == 0)
    {
      fields.push_back ("normal_x");
      fields.push_back ("normal_y");
      fields.push_back ("normal_z");
      counts.push_back (1);
      counts.push_back (1);
      counts.push_back (1);
    }
    else
    {
      fields.push_back (channels[i]);
      counts.push_back (channel.cols ());
    }

    int point_offset = offset;

    // Fill in fields
    sensor_msgs::PointField field;
    for (size_t j = 0; j < fields.size (); ++j)
    {
      field.name = fields[j];
      field.count = counts[j];
      field.datatype = sensor_msgs::PointField::FLOAT32; // Warning: Assumes all channels are floats
      field.offset = offset;
      pc2->fields.push_back (field);

      offset += 4*field.count; // Warning: Assumes all channels are floats
    }

    MatF float_matrix = channel;
    for (size_t r = 0; r < channel.rows (); ++r)
    {
      for (size_t c = 0; c < channel.cols (); ++c)
      {
        data[point_offset/4+c] = float_matrix (r,c);
      }
      point_offset += point_step;
    }

  }
  pc2->point_step = offset;

  return (pc2);
}

pcl2::Cloud
pcl2::convertPointCloud2ToCloud (const sensor_msgs::PointCloud2 & input)
{
  size_t nr_points = input.width * input.height;
  const float * data = reinterpret_cast<const float *> (&input.data[0]); // Warning: Assumes all channels are floats

  // xyz is a hacky special case: 
  //   we'll search for individual "x", "y", and "z" fields and replace them with one "xyz" field
  std::vector<sensor_msgs::PointField> fields = input.fields;
  for (size_t i = 0; i < fields.size (); ++i)
  {
    const std::string name = fields[i].name;
    if (i < fields.size () - 2 &&
        fields[i+0].name.compare("x") == 0 &&
        fields[i+1].name.compare("y") == 0 &&
        fields[i+2].name.compare("z") == 0)
    {
      sensor_msgs::PointField field;
      field.name = "xyz";
      field.count = 3;
      field.datatype = sensor_msgs::PointField::FLOAT32;
      field.offset = fields[i].offset;

      // Erase "x", "y", and "z", and add "xyz"
      fields.erase (fields.begin ()+i, fields.begin ()+i+3);
      fields.push_back (field);
    }
    // (And likewise for "normal_x", "normal_y", and "normal_z" => "normals")
    else if (i < fields.size () - 2 &&
             fields[i+0].name.compare("normal_x") == 0 &&
             fields[i+1].name.compare("normal_y") == 0 &&
             fields[i+2].name.compare("normal_z") == 0)
    {
      sensor_msgs::PointField field;
      field.name = "normals";
      field.count = 3;
      field.datatype = sensor_msgs::PointField::FLOAT32;
      field.offset = fields[i].offset;

      // Erase "normal_x", "normal_y", and "normal_z", and add "normals"
      fields.erase (fields.begin ()+i, fields.begin ()+i+3);
      fields.push_back (field);
    }
  }

  Cloud output;
  for (size_t i = 0; i < fields.size (); ++i)
  {
    EigenMat<float> float_matrix (nr_points, fields[i].count);
    
    size_t point_offset = fields[i].offset;
    for (size_t r = 0; r < float_matrix.rows (); ++r)
    {
      for (size_t c = 0; c < float_matrix.cols (); ++c)
      {
        float_matrix (r,c) = data[point_offset/4+c];
      }
      point_offset += input.point_step;
    }
    output.insert (fields[i].name, float_matrix);
  }
  
  return (output);
}
