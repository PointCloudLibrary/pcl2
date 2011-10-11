#ifndef PCL2_ARRAY_H
#define PCL2_ARRAY_H

#include <boost/shared_ptr.hpp>

namespace pcl2
{
class FloatArray;
class IntArray;

/** \brief The base array class
 *
 * \todo Write a detailed description
 */
class Array
{
public:
  /** \brief A shared pointer to an Array */
  typedef boost::shared_ptr<Array> Ptr;

  /** \brief Get the number of rows in the array
   *
   * \return The number of rows in the array
   */
  virtual size_t rows () const = 0;

  /** \brief Get the number of columns in the array
   *
   * \return The number of columns in the array
   */
  virtual size_t cols () const = 0;

  /** \brief Get the shared pointer to this array
   *
   * \return A shared pointer to this array
   */
  virtual Array::Ptr getSharedPtr () = 0;

  /** \brief Create a new copy of this array 
   *
   * \return A shared pointer to a new copy of this array
   */
  virtual Array::Ptr copy () const = 0;

  /** \brief Downcast from an Array to a FloatArray 
   * \throw A BadCastException will be thrown if the downcast is invalid
   */
  operator FloatArray& ();

  /** \brief Downcast from an Array to an IntArray 
   * \throw A BadCastException will be thrown if the downcast is invalid
   */
  operator IntArray& ();
};

}

#endif
