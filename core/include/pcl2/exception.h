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

/** \file exception.h 
 * \brief Contains class definitions for pcl2::Exception and its various subclasses
 */

#ifndef PCL2_EXCEPTION_H
#define PCL2_EXCEPTION_H

#include <string>

namespace pcl2
{

/** \brief The base exception class */
class Exception
{
public:
  /** \brief Get the exception's description
   * \return A string describing the exception
   */
  virtual std::string what () { return ("GenericException"); }
};

/** \brief Bad cast exception 
 *
 * Thrown when attempting to convert a pointer to an improper type
 */
class BadCastException : public Exception
{
public:
  virtual std::string what () { return ("BadCastException"); }
};

/** \brief Channel not found exception 
 *
 * Thrown when attempting to access a non-existent channel
 */
class ChannelNotFoundException : public Exception
{
public:
  virtual std::string what () { return ("ChannelNotFoundException"); }
};

/** \brief Incompatible size exception 
 *
 * Thrown when attempting to perform an operation with clouds/matrices whose 
 * sizes are mismatched.
 */
class IncompatibleSizeException : public Exception
{
public:
  virtual std::string what () { return ("IncompatibleSizesException"); }
};

}

#endif // PCL2_EXCEPTION_H
