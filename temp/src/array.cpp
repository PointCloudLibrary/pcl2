#include "pcl2/array.h"
#include "pcl2/float_array.h"
#include "pcl2/int_array.h"
#include "pcl2/exception.h"

pcl2::Array::operator FloatArray & ()
{
  FloatArray * p = dynamic_cast<FloatArray*> (this);
  if (p == 0)
    throw pcl2::BadCastException ();
  else
    return (*p);
}

pcl2::Array::operator IntArray & ()
{
  IntArray * p = dynamic_cast<IntArray*> (this);
  if (p == 0)
    throw pcl2::BadCastException ();
  else
    return (*p);
}
