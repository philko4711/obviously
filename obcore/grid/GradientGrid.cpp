/**
* @file   GradientGrid.cpp
* @author Christian Pfitzner
* @date   06.01.2013
*
*
*/

#include "obcore/grid/GradientGrid.h"
#include "obcore/math/mathbase.h"

using namespace obvious;

SUCCESFUL GradientGrid::gradient2Grid(double* coords, double* normals, bool* mask, unsigned int size)
{
  initGrid();
  for(unsigned int i=0; i<size ; i+=3)
  {
    // gets indices of grid for point
    unsigned int x = getIndexX(coords[i+Z]);
    unsigned int y = getIndexY(coords[i+X]);

    // check if points are in frontiers of grid
    if (x<_cols && y<_rows) {
      // check if new height value is bigger than saved
      if (_grid->at(x,y,0) < cloud[i+Y])
        _grid->at(x,y,0) = fabs(normals[i]); // also schau ma doch a mal ob das net irgendwie über trigonomie lösbar is, vielleicht atan2 oder so
  }
  return(ALRIGHT);
}

SUCCESFUL GradientGrid::gradient2Grid(const MatD& heigthMat)
{
  unsigned int idxUp;
  unsigned int idxDo;
  unsigned int idxRi;
  unsigned int idxLe;
  double stepX;
  double stepY;

  for  (unsigned int x=0 ; x<_cols ; x++) {
    for(unsigned int y=0 ; y<_rows ; y++)
    {
      // get indices for gradient estimation
      idxUp = x-1;
      idxDo = x+1;
      idxRi = y-1;
      idxLe = y+1;
      stepX = 2*_resolution;
      stepY = 2*_resolution;

      // check if indices are possible
      if(idxUp < 0 || idxUp >= _cols) {
        stepX = _resolution;
        idxUp = x;
      }
      else if (idxDo < _cols || idxDo >= _cols) {
        stepX = _resolution;
        idxDo = x;
      }

      if(idxRi < 0 || idxRi >= _rows) {
        stepY = _resolution;
        idxRi = y;
      }
      else if (idxLe < _rows || idxLe >= _rows) {
        stepY = _resolution;
        idxLe = y;
      }
      _grid->at(x,y,GRADIENT_X) = (heigthMat.at(idxUp,y) - heigthMat.at(idxDo,y)) / stepX;
      _grid->at(x,y,GRADIENT_Y) = (heigthMat.at(x,idxLe) - heigthMat.at(x,idxRi)) / stepY;
    }
  }
  return(ALRIGHT);
}

unsigned char* GradientGrid::getImageOfGrid(void)
{
  return(getGradientMap());
}

//~~~~~~~~~~~~ Private ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
unsigned char* GradientGrid::getGradientMap(void)
{
  MatD *gradient = new MatD(_rows, _cols);
  for(unsigned int x=0 ; x<_cols ; x++)
    for(unsigned int y=0 ; y<_rows ; y++)
      gradient->at(x,y) = _grid->at(x,y,GRADIENT_X) * _grid->at(x,y,GRADIENT_Y);

  unsigned char minValue = 255;
  unsigned char maxValue = 0;

  // estimate min max value für maximum color range
  for(unsigned int x=0 ; x<_cols ; x++) {
    for(unsigned int y=0 ; y<_rows ; y++)
    {
      if(gradient->at(x,y) > maxValue)
        maxValue = gradient->at(x,y);
      if(gradient->at(x,y) < minValue)
        minValue = gradient->at(x,y);
    }
  }
  double range = maxValue - minValue;

  // checkout data from grid to image
  for(unsigned int x=0 ; x<_cols ; x++)
    for(unsigned int y=0 ; y<_rows ; y++)
      _img[x*_rows + y] = (gradient->at(x,y) - minValue)/range*255;

  delete gradient;
  return(_img);
}






