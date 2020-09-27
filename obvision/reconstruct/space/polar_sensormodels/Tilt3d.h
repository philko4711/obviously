#include "obvision/reconstruct/space/SensorPolar.h"

namespace obvious
{

class Tilt3d : public SensorPolar
{
public:
  /**
   * Standard Constructor
   * @param[in] raysIncl number of inclination rays of scanning device (vertical): number of "tilts" = ( |_angleStart - _angleStop| ) / unit (check
   * dyn.reconf. params of ohm_sensors/ohm_tilt_scanner_3d/cfg/ReconfigureScan.cfg)
   * @param[in] inclMin lowest inclination angle in RAD, i.e. inclination start angle (first data captured): given by dynamic reconfigure param _angleStart
   * @param[in] inclMax highest inclination angle in RAD, i.e. inclination stop angle (last data captured): given by dynamic reconfigure param _angleStop
   * @param[in] inclRes resolution of inclination rays in RAD, i.e. angle between two vertical rays: "unit" in
   * ohm_sensors/ohm_tilt_scanner_3d/src/TiltScannerNode.cpp
   * @param[in] azimMin lowest azimuth angle in RAD (0.0 = 0°)
   * @param[in] azimMax highest azimuth angle in RAD (4.71239 = 270°)
   * @param[in] azimRes resolution of azimuth rays in RAD (0.005760 = 0.33°), angle between two horizontal rays in 360° plane
   */
  Tilt3d(unsigned int raysIncl, double inclMin, double inclMax, double inclRes, double azimMin, double azimMax, double azimRes);

  /**
   * Destructor
   */
  virtual ~Tilt3d();
};

} // namespace obvious