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
#pragma once
#include <boost/shared_ptr.hpp>
#include <string>
#include "exception.h"
#include "channel_impl.h"

namespace pcl
{
  /**
   * @brief Actual point cloud class. Data is organized as named channels and can be accessed with e.g. pointcloud["RGB"]
   */
  class PointCloud
  {
  public:
    /**
     * @brief Constructor
     * @note No channel will be created.
     * @param width in case of an ordered point cloud -> width of the point cloud.
     * @param width in case of an ordered point cloud -> height of the point cloud.
     */
    PointCloud (unsigned int width, unsigned int height = 0);

    /**
     * @brief indicates whether a channel with given name exists or not
     * @param channel_name
     * @return true if channel with given name exists
     */
    bool hasChannel (const std::string& channel_name) const;

    /**
     * @brief access operator for a channel with given name
     * @param channel_name
     * @return channel proxy object for given channel or exception if channel does not exist
     */
    ChannelBase::ConstPtr operator [] (const std::string & channel_name) const;

    /**
     * @brief access operator for a channel with given name (non-const version)
     * @param channel_name
     * @return channel proxy object for given channel or exception if channel does not exist
     */
    ChannelBase::Ptr operator [] (const std::string & channel_name);

    /**
     * @brief returns the size of the point cloud (size of the channel(s) )
     * @return size of the point cloud
     */
    unsigned int size () const;

    /**
     * @return width of an ordered point cloud
     */
    unsigned int getWidth () const;

    /**
     * @return height of the point cloud. If 0, then point cloud is not ordered otherwise its ordered
     */
    unsigned int getHeight () const;

    /**
     * @return wheter the point cloud is ordered or not
     */
    bool isOrdered () const;
    /**
     * @brief resizes all channels of the cloud
     * @param nr_points new size of the point cloud
     */
    void resize (unsigned int width, unsigned int height = 0);

    /**
     * @brief makes a deep copy of the point cloud
     * @param other destination point cloud
     */
    void copyTo (PointCloud& other) const;

    /**
     * @brief creates a deep copy of the point cloud
     * @return copy of the point cloud
     */
    PointCloud* clone () const;
    /**
     * @brief adds a new channel into the point cloud
     * @attention the size of the channel must be exactly the same as the size of the point cloud.
     * @param channel_name the name of the channel in the point cloud
     * @param channel data
     * @return reference to the channel in the point cloud
     */
    void addChannel (const std::string& channel_name, const ChannelBase::Ptr& channel);

    /**
     * @brief removes a channel from the point cloud
     * @param channel_name the name of the channel to be removed
     */
    void removeChannel (const std::string& channel_name);

    /**
     * @brief creates a new channel with given name
     * @param channel_name
     * @return reference to the newly created channel
     */
    template <typename T> void
    createChannel (const std::string& channel_name)
    {
      // boost::shared_ptr<ChannelBase> ptr = Channel<T>::create (size());
      ChannelBase::Ptr channel(new ChannelImpl<T> (size ()));
      addChannel (channel_name, channel);
    }

  private:
    /** @brief pimpl idiom
     */
    struct PointCloudImpl;
    boost::shared_ptr<PointCloudImpl> cloud_;
  } ;
}
