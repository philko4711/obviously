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

using namespace obvious;

ObstacleGrid::ObstacleGrid(double resolution, double length, double width)
: Grid2D(resolution, length, width, 4),
  _obstaclesInGrid(0)
{
  _heightTH = 1.0;
  _hGrid = new HeightGrid(resolution, length, width);
  _gGrid = new GradientGrid(resolution, length, width);
}

ObstacleGrid::~ObstacleGrid()
{
  delete _hGrid;
  delete _gGrid;
}

//SUCCESFUL ObstacleGrid::normals2Grid(double* cloud, unsigned int size, double* normals)
//{
//  for(unsigned int i=0; i<size ; i+=3)
//  {
//    // gets indices of grid for point
//    unsigned int x = getIndexX(cloud[i+X]);
//    unsigned int y = getIndexY(cloud[i+Y]);
//
//    // check if points are in frontiers of grid
//    if (x<_cols && y<_rows) {
//      // check if new height value is bigger than saved
//      if (_grid->at(x,y,GRADIENT_X) < fabs(normals[i+GRADIENT_X]))
//        _grid->at(x,y,GRADIENT_X) = normals[i+GRADIENT_X];
//
//      if (_grid->at(x,y,GRADIENT_Y) < fabs(normals[i+GRADIENT_Y]))
//        _grid->at(x,y,GRADIENT_Y) = normals[i+GRADIENT_Y];
//    }
//  }
//  _pointsEstimated = false;
//  return(ALRIGHT);
//}

SUCCESFUL ObstacleGrid::height2Grid(double* cloud, unsigned int size)
{
  if(!_hGrid->height2Grid(cloud, size))
    return(ERROR);
  return(_gGrid->gradient2Grid(dynamic_cast<MatD&>(_hGrid->getMat())));
}

bool ObstacleGrid::getObstacles()
{
  std::cout << "1" << std::endl;
  for(unsigned int x=0 ; x<_rows; x++) {
    for(unsigned int y=0 ; y<_cols; y++)
    {
      if(_gGrid->getMat().at(x,y) >= 0.2)
      {
        _grid->at(x,y) = 1.0;
      }
      else
        _grid->at(x,y) = 0.0;
    }
  }
//  double* obstacles = new double[_obstaclesInGrid];
//  return(obstacles);
  return(true);
}

unsigned char* ObstacleGrid::getImageOfGrid( void)
{
//  _hGrid->getImageOfGrid(img);
  return(getObstacleMap());
  //return(_gGrid->getImageOfGrid());

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

  // checkout data from grid to image
  for(unsigned int x=0 ; x<_cols ; x++)
    for(unsigned int y=0 ; y<_rows ; y++)
    {
      if(_grid->at(x,y) == 1.0)
        _img[x*_rows + y] = 255; //(obstacles->at(x,y) - minValue)/range*255;
      else
        _img[x*_rows + y] = 0;
    }

  delete obstacles;
  return(_img);
}




