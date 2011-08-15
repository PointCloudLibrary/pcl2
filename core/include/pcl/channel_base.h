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

namespace pcl
{
  class PointCloud;
  /**
   * @brief untyped base class for channel objects that just supports basic operations
   */
  class ChannelBase
  {
    friend class PointCloud;
    template<typename> friend class Channel;
    template<typename> friend class ConstChannel;
    public:

      typedef boost::shared_ptr<ChannelBase> Ptr;
      typedef boost::shared_ptr<const ChannelBase> ConstPtr;
      /**
       * @brief returns the size of the channel
       * @return size of the channel
       */
      virtual size_t size () const = 0;

      /**
       * @brief virtual destructor
       */
      virtual ~ChannelBase() {}


      /**
       * @brief make a deep copy of the channel
       * @return copy of the channel
       */
      virtual Ptr clone () const = 0;
    protected:
      /**
       * @brief resizes the channel
       * @param size new size of the channel
       */
      virtual void resize (size_t size) = 0;

      /**
       * @brief returns the Type of the Elements of the channel.
       */
      virtual const char* getTypeName () const = 0;
  };
}
