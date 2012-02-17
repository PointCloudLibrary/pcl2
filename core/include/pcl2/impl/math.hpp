/** \file math.hpp
 * \brief Defines functions for performing basic arithmethic on matrices
 */

#include "pcl2/math.h"
#include "pcl2/row.h"
#include "pcl2/eigen_matrix.h"

#include <pcl/common/eigen.h> /// \attention Using PCL 1

template<typename T>
pcl2::TypedMat<T>
pcl2::computeSum (const TypedMat<T> & input)
{
  EigenMat<T> sum (1, input.cols ());
  sum << 0.0;
  for (typename TypedMat<T>::Row row = input (0); row.hasNext (); row.advance ())
  {
    sum += row;
  }
  
  return (sum);
}

template<typename T>
pcl2::TypedMat<T>
pcl2::computeProduct (const TypedMat<T> & input)
{
  EigenMat<T> product (1, input.cols ());
  product << 1.0;
  for (typename TypedMat<T>::Row row = input (0); row.hasNext (); row.advance ())
  {
    product *= row;
  }
  
  return (product);
}

template<typename T>
pcl2::TypedMat<T>
pcl2::computeCumulativeSum (const TypedMat<T> & input)
{
  EigenMat<T> sum (1, input.cols ());
  sum << 0.0;
  
  EigenMat<T> output (input.rows (), input.cols ());
  typename TypedMat<T>::Row output_row = output (0);  

  for (typename TypedMat<T>::Row input_row = input (0); input_row.hasNext (); input_row.advance ())
  {
    output_row << (sum += input_row);
    output_row.advance ();
  }

  return (output);
}


template<typename T>
pcl2::TypedMat<T>
pcl2::computeCumulativeProduct (const TypedMat<T> & input)
{
  EigenMat<T> product (1, input.cols ());
  product << 1.0;
  
  EigenMat<T> output (input.rows (), input.cols ());
  typename TypedMat<T>::Row output_row = output (0);  

  for (typename TypedMat<T>::Row input_row = input (0); input_row.hasNext (); input_row.advance ())
  {
    output_row << (product *= input_row);
    output_row.advance ();
  }

  return (output);
}

template<typename T>
pcl2::TypedMat<T>
pcl2::computeOuterSum (const TypedMat<T> & vec1, const TypedMat<T> & vec2)
{
  assert (vec1.rows () == 1 || vec1.cols () == 1);
  assert (vec2.rows () == 1 || vec2.cols () == 1);

  size_t n = vec1.rows () * vec1.cols ();
  size_t m = vec2.rows () * vec2.cols ();
  EigenMat<T> output (n, m);
  for (size_t i = 0; i < n; ++i)
    for (size_t j = 0; j < m; ++j)
      output (i, j) = vec1 [i] + vec2 [j];
  return (output);
}

template<typename T>
pcl2::TypedMat<T>
pcl2::computeOuterProduct (const TypedMat<T> & vec1, const TypedMat<T> & vec2)
{
  assert (vec1.rows () == 1 || vec1.cols () == 1);
  assert (vec2.rows () == 1 || vec2.cols () == 1);
  
  size_t n = vec1.rows () * vec1.cols ();
  size_t m = vec2.rows () * vec2.cols ();
  EigenMat<T> output (n, m);
  for (size_t i = 0; i < n; ++i)
    for (size_t j = 0; j < m; ++j)
      output (i, j) = vec1 [i] * vec2 [j];
  return (output);
}

template<typename T>
pcl2::TypedMat<T>
pcl2::computeEigenvalues3x3 (const TypedMat<T> & input)
{
  // This function is hideous, but I'm just trying to get things to run

  assert (input.rows () == 3 && input.cols () == 3);

  // Copy the input matrix into a 3x3 Eigen::Matrix
  TypedMat<T> input_copy = input.copy ();
  Eigen::Matrix<T, 3, 3> matrix (Eigen::Map<const Eigen::Matrix<T, 3, 3> > (&input_copy (0, 0)));

  // Compute the eigenvalues
  Eigen::Matrix<T, 1, 3> eigenvalues;
  Eigen::Matrix<T, 3, 3> eigenvectors;
  pcl::eigen33 (matrix, eigenvectors, eigenvalues);

  // Copy the eigenvalues into the output matrix
  TypedMat<T> output = input (0).copy (); /// \todo, come up with a way to create new TypedMat<T>s 
  Eigen::Map<Eigen::Matrix<T, 1, 3> > output_map (&output (0, 0));
  output_map = eigenvalues;

  return (output);
}

template<typename T>
pcl2::TypedMat<T>
pcl2::computeEigenvectors3x3 (const TypedMat<T> & input)
{
  // This function is hideous, but I'm just trying to get things to run

  assert (input.rows () == 3 && input.cols () == 3);

  // Copy the input matrix into a 3x3 Eigen::Matrix
  TypedMat<T> input_copy = input.copy ();
  Eigen::Matrix<T, 3, 3> matrix (Eigen::Map<const Eigen::Matrix<T, 3, 3> > (&input_copy (0, 0)));

  // Compute the eigenvalues
  Eigen::Matrix<T, 1, 3> eigenvalues;
  Eigen::Matrix<T, 3, 3> eigenvectors;
  pcl::eigen33 (matrix, eigenvectors, eigenvalues);

  // Copy the eigenvectors into the output matrix
  TypedMat<T> output = input.copy (); /// \todo, come up with a way to create new TypedMat<T>s 
  Eigen::Map<Eigen::Matrix<T, 3, 3> > output_map (&output (0, 0));
  output_map = eigenvectors.transpose ();

  return (output);
}

template<typename T>
void
pcl2::computeEigendecomposition3x3 (const TypedMat<T> & input, TypedMat<T> & eigenvalues, TypedMat<T> & eigenvectors)
{
  // This function is hideous, but I'm just trying to get things to run

  assert (input.rows () == 3 && input.cols () == 3);

  // Copy the input matrix into a 3x3 Eigen::Matrix
  TypedMat<T> input_copy = input.copy ();
  Eigen::Matrix<T, 3, 3> matrix (Eigen::Map<const Eigen::Matrix<T, 3, 3> > (&input_copy (0, 0)));

  // Compute the eigenvalues
  Eigen::Matrix<T, 1, 3> eigen_eigenvalues;
  Eigen::Matrix<T, 3, 3> eigen_eigenvectors;
  pcl::eigen33 (matrix, eigen_eigenvectors, eigen_eigenvalues);

  // Copy the eigenvectors into the output matrix
  Eigen::Map<Eigen::Matrix<T, 1, 3> > eigenvalues_map (&eigenvalues (0, 0));
  Eigen::Map<Eigen::Matrix<T, 3, 3> > eigenvectors_map (&eigenvectors (0, 0));
  eigenvalues_map = eigen_eigenvalues;
  eigenvectors_map = eigen_eigenvectors.transpose ();
}

