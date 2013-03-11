/*
 * EuclideanFilterD.cpp
 *
 *  Created on: 29.11.2012
 *      Author: phil
 */

#include "obcore/filter/EuclideanFilterD.h"
#include "obcore/math/mathbase.h"

using namespace obvious;

FILRETVAL EuclideanFilterD::applyFilter(void)
{
  // check if input and output are set properly
	if((!_input)||(!_output))
	{
	  LOGMSG(DBG_ERROR, "Pointer to input or output invalid");
		return(FILTER_ERROR);
	}
	double depthVar = 0;
	double *dPtr    = _input;
	_validSize      = 0;

	// init output with zeros
	for(unsigned int i=0 ; i<_size ; i++)
		_output[i]=0.0;

  for(unsigned int i=0 ; i<_size ; i+=Point3D::sizeP) {
    depthVar =  euklideanDistance<double>((double*)dPtr, NULL, Point3D::sizeP);
    if(((depthVar >  _threshold) && (_direction == FILTER_BIGGER)) ||
       ((depthVar <= _threshold) && (_direction == FILTER_SMALLER)))
    {
      dPtr += Point3D::sizeP;
    }
    else
    {
      for(unsigned int j=0 ; j<Point3D::sizeP ; j++)
        *_output++=*dPtr++;
      _validSize += Point3D::sizeP;
    }
  }
	return(FILTER_OK);
}

/*
 * Function to set centroid
 */
void EuclideanFilterD::setCentroid(const Point3D& center)
{
  setCentroidDefault(center);
}

