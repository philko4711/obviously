/*
 * tmpSearch.cpp
 *
 *  Created on: 28.01.2013
 *      Author: christian
 */

#include "obcore/grid/ObstacleGrid.h"
#include "iostream"

using namespace obvious;

int main(void)
{
  ObstacleGrid*       _G = new ObstacleGrid(0.1, 1.0, 1.0);
  unsigned int      size = 3;
  double       coords[3] = {0.3, -0.44, 0.44};


  _G->cloud2Grid(coords, size);
  double x = 0.0;
  double y = 0.0;
  _G->getObstacles();
  _G->getNearestObstacle(x,y);
  std::cout << "Obstacle found at " << x << ", " << y << std::endl;

  return(0);
}


