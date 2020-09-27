#include "Tilt3d.h"

namespace obvious
{

Tilt3d::Tilt3d(unsigned int raysIncl, double inclMin, double inclMax, double inclRes, double azimMin, double azimMax, double azimRes)
    : SensorPolar(raysIncl, inclMin, inclMax, inclRes, azimMin, azimMax, azimRes)
{
  std::cout << __PRETTY_FUNCTION__ << "Hi." << std::endl;
}

Tilt3d::~Tilt3d() {}

int Tilt3d::lookupIndex(int inclIndex)
{
//not necessary!
}
  return indexVelodyneROS;
}

} // namespace obvious