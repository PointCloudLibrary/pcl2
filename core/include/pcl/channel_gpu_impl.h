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
 *  Author: Anatoly Baskeheev, Itseez Ltd, (myname.mysurname@mycompany.com)
 */


#pragma once

#include "channel_base.h"

//////  [!!!!!!!!!!!!!!!!!!!!!!!!!]
//////  "gpu_containers" is a module from GPU part of PCL. You can find its interface draft in gpu/containers subfolder.
//////  The module will compile *with* and *without* Cuda Toolkit installed (HAVE_CUDA define must be passed to pcl_config.h).
//////  If the module is compiled without Cuda support, functions from it will report error at runtime (throw something).
//////
//////   Containers from this module can also be implelemented using OpenCL in future.
#include <pcl/gpu/containers/device_array.hpp>




/////// Anatoly Baksheev: 
/////// Below is some draft of ChannelGpuImpl to take it into consideration. I don't clearly understand final design of channel conception now.


namespace pcl
{
    template <class Type> class Channel;
    template <class Type> class ConstChannel;
    /**
    * @brief Typed channel class
    */
    template <typename Type>
    class ChannelGpuImpl : public ChannelBase
    {
    public:
        friend class Channel<Type>;
        friend class ConstChannel<Type>;
        friend class PointCloud;

        typedef boost::shared_ptr<      ChannelGpuImpl<Type> > Ptr;
        typedef boost::shared_ptr<const ChannelGpuImpl<Type> > ConstPtr;
        
        virtual size_t size () const;        
        virtual ChannelBase::Ptr clone () const;
    protected:      
        ChannelGpuImpl (){}
        ChannelGpuImpl (size_t size) : data_ (size) {}

        virtual void resize (size_t size);        
        virtual const char* getTypeName () const;
        
        /**
        * @brief the actual data. BTW, DeviceArray2D supports reference counting. 
        * 2D-verion of DeviceArray to be able to store organized clouds.
        */
        pcl::gpu::DeviceArray2D<Type> data_;
       
    };
}
