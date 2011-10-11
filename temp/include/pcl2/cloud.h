#ifndef PCL2_CLOUD_H
#define PCL2_CLOUD_H

#include "pcl2/array.h"
#include "pcl2/exception.h"

#include <vector>
#include <string>
#include <boost/shared_ptr.hpp>
#include <boost/unordered_map.hpp>

#include <stdio.h>


namespace pcl2
{
class Array;
class IntArray;

/** \brief The core point cloud data structure
 *
 *
 */
class Cloud
{
public:

  /** \brief A shared pointer to a Cloud */
  typedef boost::shared_ptr<Cloud> Ptr;

  /** \brief Construct an empty cloud */
  Cloud ();

  /** \brief Get the number of points in the cloud
   *
   * \return The number of points in the cloud
   */
  size_t size () const;

  /** \brief Get the number of channels in the cloud
   *
   * \return The number of channels in the cloud
   */
  size_t channelCount () const;

  /** \brief Determine if the cloud contains any channels
   *
   * \return True if the cloud contains no channels, false otherwise
   */
  bool empty () const;

  /** \brief Get a list of the channel names in the cloud
   *
   * \return a vector of strings describing the channels contained in the cloud
   */
  std::vector<std::string> getChannelNames () const;

  /** \brief Access a given channel
   *
   * This method returns a reference to the channel keyed on the given \a channel_name. 
   * If the specified key is not present in the cloud, a \a ChannelNotFoundException 
   * will be thrown.
   *
   * \param channel_name The name of the desired channel
   * \return A reference to the \a Array containing the channel's data
   * \throw A ChannelNotFoundException will be thrown if the given \a channel_name is not found
   */
  Array & operator [] (const std::string & channel_name);

  /** \brief Access a given channel
   *
   * This method returns a const reference to the channel keyed on the given \a channel_name.
   * If the specified key is not present in the cloud, a \a ChannelNotFoundException 
   * will be thrown.
   *
   * \param channel_name The name of the desired channel
   * \return A const reference to the \a Array containing the channel's data
   * \throw A ChannelNotFoundException will be thrown if the given \a channel_name is not found
   */
  const Array & operator [] (const std::string & channel_name) const;

  /** \brief Create a view of a subset of points
   * 
   * This operator creates a view of the cloud based on an array of indices.  The
   * resulting view contains the same channels as the original cloud, and each channel
   * references a subset of the rows in the original channel data.
   * Note that the channel data is not copied; any changes made to the view's data
   * will also affect the corresponding points in the original cloud, and vice versa.
   *
   * \param indices An array of integer indices defining a subset of the points 
   * in the cloud.  Each element in the array must be a valid point index in the cloud.
   * \return A virtual subcloud representing only the points indexed by \a indices
   */
  Cloud operator () (const IntArray & indices) const;

  /** \brief Determine if a channel is present in the cloud
   * 
   * \param channel_name A string describing a possible channel in the cloud
   * \return A boolean indicating whether the given channel was present
   */
  bool hasChannel (const std::string & channel_name) const;

  /** \brief Add a channel to the cloud 
   * 
   * \note This method will probably be replaced with the [] operator
   * \param channel_name The string under which the new channel will be stored 
   * \param channel_data The channel data to be added
   * \throw An IncompatibleSizeException will be thrown if the size of the given \a channel_data does is not consistent
   *        with existing channels
   *
   * \todo If the specified key is already present, should it be overwritten or should an exception be thrown?
   */
  void insert (const std::string & channel_name, Array & channel_data);

  /** \brief Remove a channel from the cloud
   *
   * \param channel_name The name of the channel to be removed
   * \note If the specified channel is not present, a \a ChannelNotFoundException will be thrown
   */
  void remove (const std::string & channel_name);

  /** \brief Add all the channels from another cloud
   *
   * This method will add all the channels from the specified \a cloud.
   * \param cloud The cloud to be added to this one
   * \throw A ChannelNotFoundException will be thrown if the given \a channel_name is not found
   */
  Cloud & operator += (const Cloud & cloud);

  /** \brief Create a new copy of this cloud and its data 
   *
   * This method will create a deep copy of this cloud.  This will make a new copy
   * of the point data for every channel in the cloud.
   * \return A deep copy of the cloud and its channel data 
   * \note Any views will copied into full arrays (i.e., only the view's subset will be copied)
   */
  Cloud copy () const;

protected:
  /** \brief A map from strings to Array pointers */
  typedef boost::unordered_map<std::string, Array::Ptr> ChannelMap;

  /** \brief A map containing channel names and shared pointers to their associated data arrays */
  ChannelMap channels_;

  /** \brief The number of points in the cloud
   *
   * All of the channels in the cloud must have exactly this many rows */
  size_t size_;

};

}

#endif //PCL2_CLOUD_H
