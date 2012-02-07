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
