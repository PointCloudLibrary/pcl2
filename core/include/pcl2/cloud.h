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


/** \file cloud.h 
 * \brief Contains class declaration for pcl2::Cloud
 */
#ifndef PCL2_CLOUD_H
#define PCL2_CLOUD_H

#include "pcl2/matrix.h"
#include "pcl2/exception.h"

#include <vector>
#include <string>
#include <boost/shared_ptr.hpp>
#include <boost/unordered_map.hpp>

#include <stdio.h>

/** \brief The primary PCL namespace */
namespace pcl2
{

/** \brief The core point cloud data structure
 *
 * \todo Write a more detailed description
 */
class Cloud
{
  public:
    /** \brief A shared pointer to a Cloud */
    typedef boost::shared_ptr<Cloud> Ptr;
  
    /** \brief Construct an empty cloud */
    Cloud ();

    /** \brief Construct a single-channel cloud 
     *
     * \param channel_name The name of the channel (e.g., "xyz" for 3D points)
     * \param channel_data An n x m Mat representing a set of n m-dimensional points
     */
    Cloud (const std::string & channel_name, Mat channel_data);
  

    /** \brief Get the number of points in the cloud
     *
     * \return The number of points in the cloud
     */
    size_t 
    size () const;

    /** \brief Get the number of channels in the cloud
     *
     * \return The number of channels in the cloud
     */
    size_t 
    channelCount () const;

    /** \brief Determine if the cloud contains any channels
     *
     * \return True if the cloud contains no channels, false otherwise
     */
    bool 
    empty () const;

    /** \brief Get a list of the channel names in the cloud
     *
     * \return a vector of strings describing the channels contained in the cloud
     */
    std::vector<std::string> 
    getChannelNames () const;

    /** \brief Access a given channel
     *
     * This method returns a reference to the channel keyed on the given \a channel_name. 
     * If the specified key is not present in the cloud, a \a ChannelNotFoundException 
     * will be thrown.
     *
     * \param[in] channel_name The name of the desired channel
     * \return A reference to the \a Mat containing the channel's data
     * \throw A ChannelNotFoundException will be thrown if the given \a channel_name is not found
     */
    Mat & 
    operator [] (const std::string & channel_name);

    /** \brief Access a given channel
     *
     * This method returns a const reference to the channel keyed on the given \a channel_name.
     * If the specified key is not present in the cloud, a \a ChannelNotFoundException 
     * will be thrown.
     *
     * \param[in] channel_name The name of the desired channel
     * \return A const reference to the \a Mat containing the channel's data
     * \throw A ChannelNotFoundException will be thrown if the given \a channel_name is not found
     */
    const Mat & 
    operator [] (const std::string & channel_name) const;

    /** \brief Create a view of a subset of points
     * 
     * This operator creates a view of the cloud based on a matrix of indices.  The
     * resulting view contains the same channels as the original cloud, and each channel
     * references a subset of the rows in the original channel data.
     * Note that the channel data is not copied; any changes made to the view's data
     * will also affect the corresponding points in the original cloud, and vice versa.
     *
     * \param[in] indices A matrix of integer indices defining a subset of the points 
     * in the cloud.  Each element in the matrix must be a valid point index in the cloud.
     * \return A virtual subcloud representing only the points indexed by \a indices
     */
    Cloud 
    operator () (const TypedMat<int> & indices);

    /** \brief Determine if a channel is present in the cloud
     * 
     * \param[in] channel_name A string describing a possible channel in the cloud
     * \return A boolean indicating whether the given channel was present
     */
    bool 
    hasChannel (const std::string & channel_name) const;

    /** \brief Add a channel to the cloud 
     * 
     * \note This method will probably be replaced with the [] operator
     * \param[in] channel_name The string under which the new channel will be stored 
     * \param[in] channel_data The channel data to be added
     * \throw An IncompatibleSizeException will be thrown if the size of the given \a channel_data does is not consistent
     *        with existing channels
     *
     * \todo If the specified key is already present, should it be overwritten or should an exception be thrown?
     */
    void 
    insert (const std::string & channel_name, Mat channel_data);

    /** \brief Remove a channel from the cloud
     *
     * \param[in] channel_name The name of the channel to be removed
     * \note If the specified channel is not present, a \a ChannelNotFoundException will be thrown
     */
    void 
    remove (const std::string & channel_name);

    /** \brief Add all the channels from another cloud
     *
     * This method will add all the channels from the specified \a cloud.
     * \param[in] cloud The cloud to be added to this one
     * \throw A ChannelNotFoundException will be thrown if the given \a channel_name is not found
     */
    Cloud & 
    operator += (const Cloud & cloud);

    /** \brief Create a new copy of this cloud and its data 
     *
     * This method will create a deep copy of this cloud.  This will make a new copy
     * of the point data for every channel in the cloud.
     * \return A deep copy of the cloud and its channel data 
     * \note Any views will copied into full matrices (i.e., only the view's subset will be copied)
     */
    Cloud 
    copy () const;

  protected:
    /** \brief A map from strings to Mats */
    typedef boost::unordered_map<std::string, Mat> ChannelMap;

    /** \brief A map containing channel names and shared pointers to their associated data matrices */
    ChannelMap channels_;

    /** \brief The number of points in the cloud
     *
     * All of the channels in the cloud must have exactly this many rows */
    size_t size_;

};

}

#endif //PCL2_CLOUD_H
