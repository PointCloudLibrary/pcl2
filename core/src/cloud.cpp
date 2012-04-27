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

/** \file cloud.cpp
 * \brief Contains class definition for \ref pcl2::Cloud
 */

#include "pcl2/cloud.h"

pcl2::Cloud::Cloud () : size_ (0)
{

}

pcl2::Cloud::Cloud (const std::string & channel_name, Mat channel_data) : size_ (channel_data.rows ())
{
  channels_.emplace (channel_name, channel_data);
}

size_t
pcl2::Cloud::size () const
{
  return (size_);
}

size_t 
pcl2::Cloud::channelCount () const
{
  return (channels_.size ());
}

bool
pcl2::Cloud::empty () const
{
  return (channelCount () == 0);
}

std::vector<std::string>
pcl2::Cloud::getChannelNames () const
{
  std::vector<std::string> names (channelCount ());
  ChannelMap::const_iterator itr = channels_.begin ();
  for (size_t i = 0; i < names.size (); ++i)
  {
    names[i] = itr->first;
    ++itr;
  }
  return (names);
}

pcl2::Mat &
pcl2::Cloud::operator[] (const std::string & channel_name)
{
  ChannelMap::iterator itr = channels_.find (channel_name);
  if (itr == channels_.end ())
  {
    throw ChannelNotFoundException ();
  }
  return (itr->second);
}

const pcl2::Mat & 
pcl2::Cloud::operator[] (const std::string & channel_name) const
{
  ChannelMap::const_iterator itr = channels_.find (channel_name);
  if (itr == channels_.end ())
  {
    throw ChannelNotFoundException ();
  }
  return (itr->second);
}

#include <iostream>
pcl2::Cloud
pcl2::Cloud::operator() (const TypedMat<int> & indices)
{
  Cloud output;
  for (ChannelMap::iterator itr = channels_.begin (); itr != channels_.end (); ++itr)
  {
    output.insert (itr->first, itr->second (indices));
  }
  return (output);
}

bool
pcl2::Cloud::hasChannel (const std::string & channel_name) const
{
  return (channels_.count (channel_name) > 0);
}

void
pcl2::Cloud::insert (const std::string & channel_name, Mat channel_data)
{
  if (!empty ())
  {
    // Check if sizes match
    if (channel_data.rows () != size_)
    {
      throw IncompatibleSizeException ();
    }
  }
  else
  {
    size_ = channel_data.rows ();
  }
  channels_.emplace (channel_name, channel_data);
}

void
pcl2::Cloud::remove (const std::string & channel_name)
{
  size_t n = channels_.erase (channel_name);
  if (n == 0)
    throw pcl2::ChannelNotFoundException ();
}

pcl2::Cloud &
pcl2::Cloud::operator += (const Cloud & cloud)
{
  /// \todo Implement a cloud.begin () and cloud.end ()
  for (ChannelMap::const_iterator itr = cloud.channels_.begin ();
       itr != cloud.channels_.end ();
       ++itr)
  {
    channels_.emplace (itr->first, itr->second);
  }
  return (*this);
}

pcl2::Cloud
pcl2::Cloud::copy () const
{
  Cloud cloud_copy;
  for (ChannelMap::const_iterator itr = channels_.begin ();
       itr != channels_.end ();
       ++itr)
  {
    cloud_copy.channels_.emplace (itr->first, itr->second.copy ());
  }
  return (cloud_copy);
}
