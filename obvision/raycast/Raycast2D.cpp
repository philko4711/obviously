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
  castRays(-M_PI/6, M_PI/6, M_PI/512);
}

//-------------------------- PRIVATE--------------------------------
bool Raycast2D::castRays(const double& minAngle, const double& maxAngle, const double& step)
{
  for(double ray=minAngle ; ray<=maxAngle ; ray+=step )
    castSingleRay(ray);
  return(true);
}

inline double Raycast2D::castSingleRay(double angle)
{
  double slope = sin(angle) / cos(angle);
  bool      up = (angle>-0.5*M_PI && angle<0.5*M_PI);
  bool   right = (angle>0         && angle<M_PI);
  double    dX = up ? 1 : -1;
  double    dY = -dX * slope;

  ///@todo implement function in grid to estimate viewing point
  unsigned int idxStartX = 1;
  unsigned int idxStartY = 1;

  int wallX, wallY, wXold, wYold = 1;

  double x = up ? ceil(idxStartX) : floor(idxStartX);
  double y = right ? (idxStartY + (x-idxStartX) * slope - 1) : 0.1;

  bool diagStep;
  while(_grid->idxValid(wallX = floor(x) + (up ? 0 : -1),
                        wallY = right ? floor(y) : ceil(y)))
  {
    // necessary to avoid diagonal crossing
    if(wallX != wXold && wallY != wYold)
      diagStep = true;
    else
      diagStep = false;

    // loop for diagonal crossing
    for(unsigned int i=0 ; i<=diagStep ; i++)
    {
      // check if cell is declared as obstacle
      if(_grid->at(wallX-i,wallY) >= 1.0)
      {
        unsigned int distX = x - idxStartX;
        unsigned int distY = y - idxStartY;
        return(sqrt(distX*distX + distY*distY));
      }
      // clear out free space
      else if(_clearFree)
      {
        _grid->at(wallX-i,wallY) = -1.0;  // = FREE;
      }
    }
    // save old to new
    wXold = wallX;  wYold = wallY;
    // set up next step
    x += dX;        y += dY;
  }
  // nothing hit in this ray
  return(false);
}
