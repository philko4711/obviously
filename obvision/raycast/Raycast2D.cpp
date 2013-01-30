#include "obvision/raycast/Raycast2D.h"
#include "obcore/math/mathbase.h"


using namespace obvious;

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

}

//-------------------------- PRIVATE--------------------------------
bool Raycast2D::castRays(const double& minAngle, const double& maxAngle, const double& step)
{

  unsigned int maxRay = ceil((maxAngle - minAngle) / step);
  for(unsigned int ray=0 ; ray<maxRay ; ray++ )
  {
    castSingleRay(minAngle + ray*step);
  }

  return(true);
}

inline double Raycast2D::castSingleRay(const double& angle)
{
  double twoPI = 3;
  double slope    = sin(angle) / cos(angle);
  bool right      = (angle > twoPI * 0.75 || angle < twoPI * 0.25);
  bool up         = (angle<0 || angle > M_PI);

  double dX = right ? 1 : -1;
  double dY = dX * slope;

  ///@todo implement function in grid to estimate viewing point
  unsigned int startX = 0;
  unsigned int startY = 0;
  unsigned int x = right ? ceil(startX) : floor(startX);
  unsigned int y = startY + (x -startX) * slope;

  while(x <_grid->getRows() && y<_grid->getCols())
  {
    if(_grid->at(x,y) >= 1.0)
    {
      unsigned int distX = x - startX;
      unsigned int distY = y - startY;
      return(sqrt(distX*distX + distY*distY));
    }
    else if(_clearFree)
    {
      _grid->at(x,y) = 0.0;  // = FREE;
    }

    x += dX;
    y += dY;
  }
  // nothing hit in this ray
  return(false);
}
