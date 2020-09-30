#include "VelodyneVLP16.h"

namespace obvious {

VelodyneVLP16::VelodyneVLP16(unsigned int raysIncl, double inclMin,
                             double inclMax, double inclRes, double azimMin,
                             double azimMax, double azimRes, std::vector<int> firingSeq)
    : SensorPolar(raysIncl, inclMin, inclMax, inclRes, azimMin, azimMax,
                  azimRes, firingSeq) {
  std::cout << __PRETTY_FUNCTION__ << "Hi." << std::endl;
}

VelodyneVLP16::~VelodyneVLP16() {}

} // namespace obvious