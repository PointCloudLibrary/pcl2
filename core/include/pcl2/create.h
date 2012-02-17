/** \file create.h
 * \brief Declares functions for creating matrices
 */

#ifndef PCL2_CREATE_H
#define PCL2_CREATE_H

#include "pcl2/eigen_matrix.h"
#include <boost/random.hpp>
#include <ctime>

namespace pcl2
{

template <typename T>
TypedMat<T>
createZeros (size_t rows, size_t cols)
{
  EigenMat<T> output (rows, cols); output << 0; return (output);
}

template <typename T>
TypedMat<T>
createOnes (size_t rows, size_t cols)
{
  EigenMat<T> output (rows, cols); output << 1; return (output);
}

template <typename T>
TypedMat<T>
createIdentity (size_t n)
{
  EigenMat<T> output (n, n);
  output << 0;
  for (size_t i = 0; i < n; ++i)
    output (i, i) = 1;
  return (output); 
}

template <typename T>
TypedMat<T>
createRandom (size_t rows, size_t cols)
{
  unsigned seed = static_cast<unsigned> (std::time(0));
  boost::mt19937 generator (seed);                
  boost::uniform_01<> uniform_dist;    
  boost::variate_generator<boost::mt19937, boost::uniform_01<> > rand (generator, uniform_dist);  

  EigenMat<T> output (rows, cols);
  for (size_t j = 0; j < cols; ++j)
    for (size_t i = 0; i < rows; ++i)
      output (i, j) = rand (); 
  return (output);
}

template <typename T>
TypedMat<T>
createSeries (T start, T end, T step=1)
{
  size_t nr_elements = ceil(1.0*(end - start) / step);
  EigenMat<T> output (nr_elements, 1);

  size_t i = 0;
  for (T x = start; x < end; x += step)
    output (i++, 0) = x;

  return (output);
}

}

#endif
