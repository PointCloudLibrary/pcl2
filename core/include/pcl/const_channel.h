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
#include "channel_impl.h"
#include "channel.h"
#include <vector>

namespace pcl
{
  /**
   * @brief Proxy class to access elements
   */
  template <typename Type>
  class ConstChannel
  {
    public:

      ConstChannel (const ConstChannel& other)
      : channel_ (other.channel_)
      {
      }

      ConstChannel (const Channel<Type>& other)
      : channel_ (other.channel_)
      {
      }

      ConstChannel (ChannelBase::ConstPtr base)
      {
        channel_ = boost::dynamic_pointer_cast <const ChannelImpl<Type> >(base);
        if (!channel_.get ())
          THROW_PCL_EXCEPTION ("Type mismatch. %s requested, but actual channel elements are of type %s", typeid(Type).name (), base->getTypeName());
      }

      ConstChannel (ChannelBase::Ptr base)
      {
        channel_ = boost::dynamic_pointer_cast <const ChannelImpl<Type> >(base);
        if (!channel_.get ())
          THROW_PCL_EXCEPTION ("Type mismatch. %s requested, but actual channel elements are of type %s", typeid(Type).name (), base->getTypeName());
      }
      /**
       * @brief returns the size of the channel
       * @return size of the channel
       */
      virtual size_t size () const
      {
        return (channel_->size ());
      }

      /**
       * @brief access operator for element in channel (constant reference version)
       * @param index index of the element to be accessed
       * @attention does not make a bound check!
       * @return constant reference to the element at position "index"
       */
      const Type & operator [] (size_t index) const
      {
        return channel_->data_[index];
      }

      ConstChannel<Type>& operator=(const ChannelBase::ConstPtr& base)
      {
        channel_ = boost::dynamic_pointer_cast <const ChannelImpl<Type> >(base);
        return *this;
      }
  private:
      /**
       * @brief the actual data
       */
      typename ChannelImpl<Type>::ConstPtr channel_;
  };
}
