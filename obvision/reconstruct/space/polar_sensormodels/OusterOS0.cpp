#include "OusterOS0.h"

namespace obvious
{

OusterOS0::OusterOS0(unsigned int raysIncl, double inclMin, double inclMax, double inclRes, double azimMin, double azimMax, double azimRes)
    : SensorPolar(raysIncl, inclMin, inclMax, inclRes, azimMin, azimMax, azimRes)
{
  std::cout << __PRETTY_FUNCTION__ << "Hi." << std::endl;
}

OusterOS0::~OusterOS0() {}

int OusterOS0::lookupIndex(int inclIndex)
{
  // not necessary if ROS node orders rays from inclMin to inclMax in +inclRes steps for each azimuth from 0°-360° in +azimRes steps
}

} // namespace obvious