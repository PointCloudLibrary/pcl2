#include "pcl2/cloud.h"

pcl2::Cloud::Cloud () : size_ (0)
{

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

pcl2::Array &
pcl2::Cloud::operator[] (const std::string & channel_name)
{
  ChannelMap::iterator itr = channels_.find (channel_name);
  if (itr == channels_.end ())
  {
    throw ChannelNotFoundException ();
  }
  return (*itr->second);
}

const pcl2::Array &
pcl2::Cloud::operator[] (const std::string & channel_name) const
{
  ChannelMap::const_iterator itr = channels_.find (channel_name);
  if (itr == channels_.end ())
  {
    throw ChannelNotFoundException ();
  }
  return (*itr->second);
}

bool
pcl2::Cloud::hasChannel (const std::string & channel_name) const
{
  return (channels_.count (channel_name) > 0);
}

void
pcl2::Cloud::insert (const std::string & channel_name, Array & channel_data)
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
  channels_.emplace (channel_name, channel_data.getSharedPtr ());
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
  // !!! Implement a cloud.begin () and cloud.end ()
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
    cloud_copy.channels_.emplace (itr->first, itr->second->copy ());
  }
  return (cloud_copy);
}
