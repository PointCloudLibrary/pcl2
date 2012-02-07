#include <iostream>
#include "pcl2/matrix.h"
#include "pcl2/eigen_matrix.h"
#include "pcl2/row.h"

template <class MatT, typename ScalarT>
MatT createRange (ScalarT start, ScalarT end, ScalarT step)
{
  int nr_elements = ceil(1.0*(end - start) / step);
  MatT out (nr_elements, 1);

  int i = 0;
  for (ScalarT x = start; x < end; x += step)
    out (i++, 0) = x;

  return (out);
}

int
main (int argc, char ** argv)
{
  // Create an N by M matrix of floats
  int n = 8;
  int m = 3;
  pcl2::EigenMat<float> mat (n, m);

  // We can access its elements with the () operator 
  // (it's a matrix, so we follow mathematical convention: rows by columns)
  int i = 1;
  int j = 2;
  std::cout << "The element stored at row " << i+1 << " and column " << j+1 << " is: ";
  std::cout << mat (i, j) << std::endl;
  
  // As you likely guessed, we can change individual elements by assigning to the returned reference
  mat (i, j) = 123;
  std::cout << "After assignment, mat = " << std::endl << mat << std::endl;

  // By default, the values of an EigenMat are initialized to zero. 
  // We can change all the values in the array using 'fill'
  mat.fill (1.0);
  std::cout << "After fill, mat = " << std::endl << mat << std::endl;

  // We can also iterate over the rows and fill in each row individually
  pcl2::ConstMatF foo = mat;
  pcl2::ConstMatF::Row foo2 = mat (0);
  for (pcl2::MatF::Row r = mat (0); r.hasNext (); r.advance ())
  {
    r.fill (100.0*r.getIndex ());
  }
  std::cout << "After row-by-row fill, mat = " << std::endl << mat << std::endl;

  pcl2::MatI idx = createRange<pcl2::EigenMat<int>, int> (0, n, 2); // What should this function really look like?
  std::cout << "idx = " << std::endl << idx << std::endl;

  pcl2::MatF view = mat (idx);
  std::cout << "view of mat [mat (idx)] = " << std::endl << view << std::endl;
  view.fill (2);

  std::cout << "After filling the view, mat = " << std::endl << mat << std::endl;


  return (0);
}
