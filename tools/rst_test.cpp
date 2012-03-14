#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include "pcl2/matrix.h"
#include "pcl2/eigen_matrix.h"
#include "pcl2/row.h"
#include "pcl2/math.h"
#include "pcl2/stats.h"
#include "pcl2/io/io.h"
#include "pcl2/search/neighbors.h"
#include "pcl2/registration/fit.h"
#include "pcl2/create.h"

#define START_RST_CODEBLOCK(rst) rst.openCodeblock(__FILE__, __LINE__+1);
#define END_RST_CODEBLOCK(rst) rst.closeCodeblock(__LINE__-1); 

class RSTWriter
{
public:
  RSTWriter () : codeblock_open_ (false) {}

  void addRaw (const std::string & str)
  {
    strstream_ << str;
  }
  void addTitle (const std::string & title) 
  {
    int len = title.length ();
    strstream_ << std::setw (len) << std::setfill('=') << "" << std::endl
               << title << std::endl 
               << std::setw (len) << "" << std::endl 
               << std::endl; 
  }
  void addSubtitle (const std::string & subtitle, size_t depth=1)
  {
    char fill;
    switch (depth)
    {
    case 0: fill = '-';
      break;
    case 1: fill = '*';
      break;
    case 2: fill = '^';
      break;
    case 3: fill = '%';
      break;
    case 4: fill = '#';
      break;
    default: fill = '~';
    }
    int len = subtitle.length ();
    strstream_ << subtitle << std::endl 
               << std::setfill (fill) << std::setw (len) << "" << std::endl 
               << std::endl; 
  }
  void addText (const std::string & text)
  {
    strstream_ << text << std::endl << std::endl;
  }
  template <typename T> void addMat (const pcl2::TypedMat<T> & mat) 
  {
    const int w = 15;
    int prec = strstream_.precision ();
    strstream_.precision (3);

    strstream_ << ".. class:: matrix" << std::endl << std::endl;
    for (size_t j = 0; j < mat.cols (); ++j)
      strstream_ << std::setfill ('=') << std::setw (w+1) << " ";
    strstream_ << std::endl;
    for (size_t i = 0; i < mat.rows (); ++i)
    {
      for (size_t j = 0; j < mat.cols (); ++j)
      {
        strstream_ << std::setfill (' ') << std::setw (w) << std::fixed << mat (i, j) << " ";
      }
      strstream_ << std::endl;
    }
    for (size_t j = 0; j < mat.cols (); ++j)
      strstream_ << std::setfill ('=') << std::setw (w+1) << " ";
    strstream_ << std::endl << std::endl;

    strstream_.precision (prec);
  }

  void addImage (const std::string & filename, int other_stuff=false)
  {
    strstream_ << ".. image:: " << filename << std::endl << std::endl;
  }

  void openCodeblock (const std::string & filename, size_t start)
  {
    assert (!codeblock_open_);
    code_filename_ = filename;
    code_start_ = start;
    codeblock_open_ = true;
  }
  void closeCodeblock (size_t end) 
  {
    assert (codeblock_open_);
    code_end_ = end;
    codeblock_open_ = false;
    // write the code block out to rst
    strstream_ << ".. literalinclude:: " << code_filename_ << std::endl
               << "   :language: cpp" << std::endl
               << "   :lines: " << code_start_ << " - " << code_end_ << std::endl 
               << std::endl;
  }
  void write (const std::string filename)
  {
    std::ofstream output_file;
    output_file.open (filename.c_str ());
    output_file << strstream_.str ();
    output_file.close();
  }

protected:
  std::stringstream strstream_;
  bool codeblock_open_;
  std::string code_filename_;
  size_t code_start_, code_end_;
};

int 
main (int argc, char ** argv)
{
  RSTWriter rst;
  
  rst.addTitle ("Example title");

  rst.addText ("This is a simple example of how to write a tutorial in a single cpp file");

  rst.addSubtitle ("Example subtitle");

  rst.addText ("The following example shows how to create "
               "a matrix and fill it with random values.");

  START_RST_CODEBLOCK(rst);
  // Create an N by M matrix of floats
  int n = 8;
  int m = 3;
  pcl2::EigenMat<float> mat = pcl2::EigenMat<float> (n, m);
  mat << pcl2::createRandom<float> (n, m);
  END_RST_CODEBLOCK(rst);

  rst.addText ("The next line displays the matrix");

  START_RST_CODEBLOCK(rst);
  std::cout << mat << std::endl;
  END_RST_CODEBLOCK(rst);

  rst.addText ("and the output should look like this:");
  rst.addMat (mat);

  rst.addText ("And if we wanted to display an image, we could.");
  rst.addImage ("./image.png");

  rst.write ("rst_test.rst");

}
