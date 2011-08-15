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
#include "channel_base.h"
#include <vector>

namespace pcl
{
  template <class Type> class Channel;
  template <class Type> class ConstChannel;
  /**
   * @brief Typed channel class
   */
  template <typename Type>
  class ChannelImpl : public ChannelBase
  {
    public:
      friend class Channel<Type>;
      friend class ConstChannel<Type>;
      friend class PointCloud;

      typedef boost::shared_ptr<ChannelImpl<Type> > Ptr;
      typedef boost::shared_ptr<const ChannelImpl<Type> > ConstPtr;
      /**
       * @brief returns the size of the channel
       * @return size of the channel
       */
      virtual size_t size () const
      {
        return (data_.size ());
      }

      virtual ChannelBase::Ptr clone () const
      {
        return ChannelBase::Ptr(new ChannelImpl<Type> (*this));
      }
      protected:
      /**
       * @brief access operator for element in channel
       * @param index index of the element to be accessed
       * @attention does not make a bound check!
       * @return reference to the element at position "index"
       */
      Type & operator [] (size_t i)
      {
        return data_[i];
      }

      /**
       * @brief access operator for element in channel (constant reference version)
       * @param index index of the element to be accessed
       * @attention does not make a bound check!
       * @return constant reference to the element at position "index"
       */
      const Type & operator [] (size_t index) const
      {
        return data_[index];
      }

      ChannelImpl (){}
      ChannelImpl (size_t size) : data_ (size) {}

      virtual void resize (size_t size)
      {
        data_.resize (size);
      }

      virtual const char* getTypeName () const
      {
        return typeid(Type).name ();
      }
      /**
       * @brief the actual data
       */
      std::vector<Type> data_;
  };
}
