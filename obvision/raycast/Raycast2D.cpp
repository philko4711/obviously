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
//  castRays(-0.25, 0.25, 40);
  castSingleRay(0.2);
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
  double slope    = sin(angle) / cos(angle);
  bool up      = (angle>-0.5*M_PI || angle<0.5*M_PI);
  bool right   = (angle>0         || angle<M_PI);

  float dX = up ? 1 : -1;
  float dY = dY * slope;

  ///@todo implement function in grid to estimate viewing point
  unsigned int idxStartX = 1;
  unsigned int idxStartY = 1;

  unsigned int x = right ? ceil(idxStartX) : floor(idxStartX);
  unsigned int y = idxStartY + (x-idxStartX) * slope;

  std::cout << "test" << std::endl;

  while(_grid->idxValid(x,y))
  {
    int wallX = floor(x + (right ? 0 : -1));
    int wallY = floor(y);

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
