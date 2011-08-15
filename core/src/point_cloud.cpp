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
#include <pcl/channel_impl.h>
#include <map>
#include <bits/stl_map.h>

struct pcl::PointCloud::PointCloudImpl
{
  unsigned int width_;
  unsigned int height_;
  std::map<std::string, ChannelBase::Ptr > map_;
};

pcl::PointCloud::PointCloud(unsigned int width, unsigned int height)
  : cloud_(new PointCloudImpl)
{
  cloud_->width_ = width;
  cloud_->height_ = height;
}

void pcl::PointCloud::addChannel(const std::string& channel_name, const ChannelBase::Ptr& channel)
{
  if (channel->size() != size ())
  {
    THROW_PCL_EXCEPTION("channel size does not match point cloud size");
  }
  else if (hasChannel(channel_name))
  {
    THROW_PCL_EXCEPTION("channel with name \"%s\" already exists.", channel_name.c_str());
  }

  cloud_->map_[channel_name] = channel;
}

void pcl::PointCloud::removeChannel(const std::string& channel_name)
{
  std::map<std::string, ChannelBase::Ptr >::iterator it = cloud_->map_.find(channel_name);
  if (it == cloud_->map_.end())
  {
    THROW_PCL_EXCEPTION("No channel with name \"%s\" exists in the cloud!\n", channel_name.c_str());
  }
  cloud_->map_.erase(it);
}

bool pcl::PointCloud::hasChannel(const std::string& channel_name) const
{
  return (cloud_->map_.count(channel_name) > 0);
}

pcl::ChannelBase::ConstPtr pcl::PointCloud::operator [] (const std::string& channel_name) const
{
  std::map<std::string, ChannelBase::Ptr >::const_iterator it = cloud_->map_.find(channel_name);
  if (it == cloud_->map_.end())
  {
    THROW_PCL_EXCEPTION("No channel with name \"%s\" exists in the cloud!\n", channel_name.c_str());
  }

  pcl::ChannelBase::ConstPtr channel = boost::const_pointer_cast<ChannelBase> (it->second);
  return channel;
}

pcl::ChannelBase::Ptr pcl::PointCloud::operator [] (const std::string& channel_name)
{
  std::map<std::string, ChannelBase::Ptr >::const_iterator it = cloud_->map_.find(channel_name);
  if (it == cloud_->map_.end())
  {
    THROW_PCL_EXCEPTION("No channel with name \"%s\" exists in the cloud!\n", channel_name.c_str());
  }

  return it->second;
}

unsigned int pcl::PointCloud::size() const
{
  return (cloud_->height_?cloud_->width_*cloud_->height_:cloud_->width_);
}

unsigned int pcl::PointCloud::getWidth () const
{
  return cloud_->width_;
}

unsigned int pcl::PointCloud::getHeight () const
{
  return cloud_->height_;
}

void pcl::PointCloud::resize(unsigned int width, unsigned int height)
{
  cloud_->width_ = width;
  cloud_->height_ = height;
  unsigned new_size = size ();
  // Resize all of the vectors in the cloud
  std::map<std::string, ChannelBase::Ptr >::iterator it;
  for (it = cloud_->map_.begin(); it != cloud_->map_.end(); ++it)
    it->second->resize(new_size);
}

//deep copy

void pcl::PointCloud::copyTo(PointCloud& other) const
{
  other.cloud_->map_.clear();
  other.resize (getWidth(), getHeight());
  std::map<std::string, ChannelBase::Ptr >::iterator it;
  for (it = cloud_->map_.begin(); it != cloud_->map_.end(); ++it)
  {
    other.addChannel(it->first, it->second->clone());
  }
}

pcl::PointCloud* pcl::PointCloud::clone () const
{
  pcl::PointCloud* copy = new pcl::PointCloud (getWidth(), getHeight());
  std::map<std::string, ChannelBase::Ptr >::iterator it;
  for (it = cloud_->map_.begin(); it != cloud_->map_.end(); ++it)
  {
    copy->addChannel(it->first, it->second->clone());
  }
  return copy;
}
