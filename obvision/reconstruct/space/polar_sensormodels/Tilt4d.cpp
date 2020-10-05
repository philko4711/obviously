#include "Tilt4d.h"

namespace obvious
{

Tilt4d::Tilt4d(unsigned int raysIncl, double inclMin, double inclMax, double inclRes, double azimMin, double azimMax, double azimRes)
    : SensorPolar(raysIncl, inclMin, inclMax, inclRes, azimMin, azimMax, azimRes)
{
  std::cout << __PRETTY_FUNCTION__ << "Hi." << std::endl;
}

Tilt4d::~Tilt4d() {}

} // namespace obvious