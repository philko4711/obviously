#include "obvision/raycast/Raycast2D.h"
#include "obcore/math/mathbase.h"


using namespace obvious;

/**
 * Thanks to: http://dev.opera.com/articles/view/creating-pseudo-3d-games-with-html-5-can-1/
 */
Raycast2D::Raycast2D(void)
{
  _grid       = NULL;
  _clearFree  = false;
}

Raycast2D::~Raycast2D(void)
{
  delete _grid;
}

void Raycast2D::estimateFreeSpace(bool clear)
{
  _clearFree = clear;
//  castSingleRay(-M_PI/16);
  castRays(-M_PI/6, M_PI/6, M_PI/128);
}

//-------------------------- PRIVATE--------------------------------
bool Raycast2D::castRays(const double& minAngle, const double& maxAngle, const double& step)
{
  for(double ray=minAngle ; ray<=maxAngle ; ray+=step )
  {
    castSingleRay(ray);
  }

  return(true);
}

inline double Raycast2D::castSingleRay(double angle)
{
  if (angle > 0)
    angle += 2*M_PI;


  double slope = sin(angle) / cos(angle);
  bool up      = (angle>-0.5*M_PI || angle<0.5*M_PI);
  bool right   = (angle>0         || angle<M_PI);

  double  dX = up ? 1 : -1;
  double  dY = dX * slope;

  ///@todo implement function in grid to estimate viewing point
  unsigned int idxStartX = 1;
  unsigned int idxStartY = 1;

  int wallX = 1;
  int wallY = 1;

  double x = right ? ceil(idxStartX) : floor(idxStartX);
  double y = idxStartY + (x-idxStartX) * slope;

  std::cout << "test" << std::endl;

  while(_grid->idxValid(wallX,wallY))
  {
    wallX = floor(x + (right ? 0 : -1));
    wallY = floor(y);

    std::cout << "x: " << wallX << ", y: " << wallY << std::endl;
    // check if cell is declared as obstacle
    if(_grid->at(wallX,wallY) >= 1.0)
    {
      unsigned int distX = x - idxStartX;
      unsigned int distY = y - idxStartY;
      return(sqrt(distX*distX + distY*distY));
    }
    // clear out free space
    else if(_clearFree)
    {
      _grid->at(wallX,wallY) = -1.0;  // = FREE;
    }

    // set up next step
    x += dX;
    y += dY;
  }
  // nothing hit in this ray
  return(false);
}
