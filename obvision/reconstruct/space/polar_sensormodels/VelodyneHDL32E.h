#include "obvision/reconstruct/space/SensorPolar.h"

namespace obvious
{

class VelodyneHDL32E : public SensorPolar
{
public:
  /**
   * Standard Constructor
   * @param[in] raysIncl number of inclination rays of scanning device (vertical): 32
   * @param[in] inclMin lowest inclination angle in RAD, i.e. inclination start angle (first data captured) (-0.535292 = -30.67°)
   * @param[in] inclMax highest inclination angle in RAD, i.e. inclination stop angle (last data captured) (0.186227 = +10.67°)
   * @param[in] inclRes resolution of inclination rays in RAD (0.0232129 = 1.33°), i.e. angle between two vertical rays
   * @param[in] azimMin lowest azimuth angle in RAD (0.0 = 0°)
   * @param[in] azimMax highest azimuth angle in RAD (6.28319 = 360°)
   * @param[in] azimRes resolution of azimuth rays in RAD (0.002897 = 0.166°), angle between two horizontal rays in 360° plane, depending on rotation speed
   * (RPM) of sensor, default RPM = 600 --> azimRes = 0.166° (300RPM --> 0.083°, 900RPM --> 0.249°, 1200RPM --> 0.332°)
   * @param[in] firingSeq order in which vertical lasers are fired (see SensorPolar class Constructor+lookupIndex methods for more details), is by default empty. If not empty, it will trigger a call to lookupIndex in function backProject() of C SensorPolar
   * firingSeq for Velodyne HDL-32E: firingSeq={0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30,  1, 3, 5, 7, 9, 11, 13, 15, 17, 19, 21, 23, 25, 27, 29, 31}; (check HDL-32E User Manual p. 62ff)
   * note: since the firing sequence (scan order) of the HDL-32E does not increment evenly from inclMin to inclMax in inclRes steps, the lookupIndex method
   * is necessary to match the scan order to the raycast order
   */
  VelodyneHDL32E(unsigned int raysIncl, double inclMin, double inclMax, double inclRes, double azimMin, double azimMax, double azimRes, std::vector<int> firingSeq);

  /**
   * Destructor
   */
  virtual ~VelodyneHDL32E();

};

} // namespace obvious