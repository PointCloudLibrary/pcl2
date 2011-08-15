/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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
 *
 * Author: Suat Gedikli (gedikli@willowgarage.com)
 */

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/channel.h>
#include <pcl/const_channel.h>
#include <boost/shared_ptr.hpp>
#include <pcl/io/openni_grabber.h>
#include <pcl/common/time.h>

/**
 * @brief example of how to use a global callback function with the Grabber interface
 * @param cloud the cloud to be processed
 */
void globalFunction (pcl::PointCloud cloud)
{
  static unsigned count = 0;
  static double last = pcl::getTime ();
  double now = pcl::getTime ();
  ++count;
  if (now - last >= 1.0)
  {
    pcl::ConstChannel<pcl::PointXYZ> points = cloud["xyz"];
    std::cout << "distance of center pixel :" << points [(cloud.getWidth ()) * (cloud.getHeight() + 1)].z << " mm. Average frame rate: " << double(count)/double(now - last) << " Hz" <<  std::endl;
    count = 0;
    last = now;
  }
}

/**
 * @brief example of how to use a method as  a callback function for the Grabber interface
 */
class SimpleOpenNIProcessor
{
public:
  void method (pcl::PointCloud cloud);
};

void SimpleOpenNIProcessor::method(pcl::PointCloud cloud)
{
  static unsigned count = 0;
  static double last = pcl::getTime ();
  double now = pcl::getTime ();
  ++count;
  if (now - last >= 1.0)
  {
    pcl::ConstChannel<pcl::PointXYZ> points = cloud["xyz"];
    std::cout << "distance of center pixel :" << points [(cloud.getWidth ()) * (cloud.getHeight() + 1)].z << " mm. Average frame rate: " << double(count)/double(now - last) << " Hz" <<  std::endl;
    count = 0;
    last = now;
  }
}


int
main ()
{

  SimpleOpenNIProcessor v;
  pcl::Grabber* interface = new pcl::OpenNIGrabber();

  boost::function<void (pcl::PointCloud)> f = boost::bind (&SimpleOpenNIProcessor::method, &v, _1);
  interface->registerCallback (f);

  interface->start ();
  while(true);
  interface->stop ();

  delete interface;
  return 0;
}