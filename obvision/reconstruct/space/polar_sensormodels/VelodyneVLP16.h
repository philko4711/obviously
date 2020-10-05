#include "obvision/reconstruct/space/SensorPolar.h"

namespace obvious {

class VelodyneVLP16 : public SensorPolar {
public:
  /**
   * Standard Constructor
   * @param[in] raysIncl number of inclination rays of scanning device
   * (vertical): 16
   * @param[in] inclMin lowest inclination angle in RAD, i.e. inclination start
   * angle (first data captured) (-0.26180 = -15°)
   * @param[in] inclMax highest inclination angle in RAD, i.e. inclination stop
   * angle (last data captured) (0.26180 = +15°)
   * @param[in] inclRes resolution of inclination rays in RAD (0.034907 = 2°),
   * i.e. angle between two vertical rays
   * @param[in] azimMin lowest azimuth angle in RAD (0.0 = 0°)
   * @param[in] azimMax highest azimuth angle in RAD (6.28319 = 360°)
   * @param[in] azimRes resolution of azimuth rays in RAD (0.00349066 = 0.2°),
   * angle between two horizontal rays in 360° plane, depending on rotation
   * speed
   * (RPM) of sensor, default RPM = 600 --> azimRes = 0.2° (300RPM --> 0.1°,
   * 900RPM --> 0.3°, 1200RPM --> 0.4°)
   * @param[in] firingSeq order in which vertical lasers are fired (see SensorPolar class Constructor+lookupIndex methods for more details), is by default empty. If not empty, it will trigger a call to lookupIndex in function backProject() of C SensorPolar
   * firingSeq for Velodyne: firingSeq={0, 2, 4, 6, 8, 10, 12, 14, 1, 3, 5, 7, 9, 11, 13, 15}; (check VLP16 User Manual p. 54ff)
   * @note: since the firing sequence (scan order) of the VLP16 does not
   * increment evenly from inclMin to inclMax in inclRes steps, the lookupIndex
   * method is
   * necessary to match the scan order to the raycast order
   */
  VelodyneVLP16(unsigned int raysIncl, double inclMin, double inclMax,
                double inclRes, double azimMin, double azimMax, double azimRes, std::vector<int> firingSeq);

  /**
   * Destructor
   */
  virtual ~VelodyneVLP16();

};

} // namespace obvious