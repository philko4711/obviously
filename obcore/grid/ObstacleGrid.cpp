/**
* @file   ObstacleGrid.cpp
* @author Christian Pfitzner
* @date   04.01.2013
*
*
*/

#include "obcore/grid/ObstacleGrid.h"

#include <math.h>
#include "obcore/Point.h"
#include "obcore/Point3D.h"
#include "obcore/base/RGBImage.h"
#include "obcore/rgbColor.h"

using namespace obvious;

ObstacleGrid::ObstacleGrid(double resolution, double length, double width)
: Grid2D(resolution, length, width, 4),
  _obstaclesInGrid(0)
{
  _hCrit   = 1.0;
  _sCrit   = 1.0;
  _rCrit   = 1.0;

  _hGrid = new HeightGrid(resolution, length, width);
  _gGrid = new GradientGrid(resolution, length, width);
  _sGrid = new GradientGrid(resolution, length, width);
}

ObstacleGrid::~ObstacleGrid()
{
  delete _hGrid;
  delete _gGrid;
  delete _sGrid;
}

SUCCESFUL ObstacleGrid::normals2Grid(double* coords, double* normals, bool* mask, unsigned int size)
{
  return(_sGrid->gradient2Grid(coords, normals, mask, size));
}

SUCCESFUL ObstacleGrid::height2Grid(double* coords, bool* mask, unsigned int size)
{
  if(!_hGrid->height2Grid(coords, mask, size))
    return(ERROR);
  return(_gGrid->gradient2Grid(dynamic_cast<MatD&>(_hGrid->getMat())));
}

bool ObstacleGrid::getObstacles()
{
  for(unsigned int x=0 ; x<_rows; x++) {
    for(unsigned int y=0 ; y<_cols; y++)
    {
//      if(_gGrid->getMat().at(x,y) >= _heightTH || _sGrid->getMat().at(x,y) >= _slopeTH)
      if(_gGrid->getMat().at(x,y)/_hCrit*_hWeight + _sGrid->getMat().at(x,y)/_sCrit * _sWeight >= 1.0)
      {
        _grid->at(x,y) = 1.0;
      }
      else
        _grid->at(x,y) = _gGrid->getMat().at(x,y);
    }
  }
  return(true);
}

unsigned char* ObstacleGrid::getImageOfGrid( void)
{
  return(getObstacleMap());
}

bool ObstacleGrid::getNearestObstacle(double& x, double& y) const
{
  unsigned int idxX, idxY;
  // estimate maximum square calculation
  unsigned int squareMax;
  if (_cols >= _rows)
    squareMax = _cols/2;
  else
    squareMax = _rows/2;

  // loop for whole grid
  for(unsigned int square = 1 ; square <= squareMax ; square++ )
  {
    enum DIRECTION {DOWN, LEFT, UP, RIGHT};

    unsigned int nrPerRow_Col = square * 2;
    // estimate start position
    unsigned int idxStartY    = (_rows/2) - square;
    unsigned int idxStartX    = (_cols/2) - square;

    // loop for one square
    for(unsigned int dir = DOWN ; dir <= RIGHT ; dir++)
    {
      for(unsigned int i = 1 ; i<nrPerRow_Col ; i++)
      {
        if(dir == DOWN) {
          idxX = idxStartX + i;
          idxY = idxStartY;
        }
        if(dir == LEFT) {
          idxX = idxStartX + nrPerRow_Col - 1;
          idxY = idxStartY + i;
        }
        if(dir == UP) {
          idxX = idxStartX + nrPerRow_Col - i - 1;
          idxY = idxStartY + nrPerRow_Col - 1;
        }
        if(dir == RIGHT) {
          idxX = idxStartX;
          idxY = idxStartY + nrPerRow_Col - i;
        }

        // check for obstacle
        if (_grid->at(idxX,idxY) != 0.0)
        {
          x = getCoord2idxX(idxX);
          y = getCoord2idxY(idxY);
          return(true);
        }
      }
    }
  }
  // no obstacle found
  return(false);
}

unsigned char* ObstacleGrid::getObstacleMap(void)
{
  MatD *obstacles = new MatD(_rows, _cols);
  RGBImage* img   = new RGBImage(_rows, _cols);

  unsigned char minValue = 255;
  unsigned char maxValue = 0;

  // estimate min max value für maximum color range
  for(unsigned int x=0 ; x<_cols ; x++) {
    for(unsigned int y=0 ; y<_rows ; y++)
    {
      if(obstacles->at(x,y) > maxValue)
        maxValue = obstacles->at(x,y);
      if(obstacles->at(x,y) < minValue)
        minValue = obstacles->at(x,y);
    }
  }
  enum ColorEnum {RED, GREEN, BLUE};

  // checkout data from grid to image
  for(unsigned int x=0 ; x<_cols ; x++)
    for(unsigned int y=0 ; y<_rows ; y++)
    {
      // obstacles
      if(_grid->at(x,y) >= 1.0)
      {
        img->at(x,y,RED)    = 255;
        img->at(x,y,GREEN)  = 0;
        img->at(x,y,BLUE)   = 0;
      }
      // terrain
      else if (_grid->at(x,y) > 0.0)
      {
        for(unsigned int i=RED ; i<=BLUE ; i++)
          img->at(x,y,i) = 255*(1-_grid->at(x,y));
      }
      // no obstacles
      else if (_grid->at(x,y) == -1.0)
      {
        for(unsigned int i=RED ; i<=BLUE ; i++)
          img->at(x,y,i) = 255;
      }
      // unknown
      else
      {
        for(unsigned int i=RED ; i<=BLUE ; i++)
          img->at(x,y,i) = 180;
      }
    }

  _img = img->getImg();

  delete img;
  delete obstacles;
  return(_img);
}




