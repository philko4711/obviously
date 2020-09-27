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
   * note: since the firing sequence (scan order) of the HDL-32E does not increment evenly from inclMin to inclMax in inclRes steps, the lookupIndex method
   * is necessary to match the scan order to the raycast order
   */
  VelodyneHDL32E(unsigned int raysIncl, double inclMin, double inclMax, double inclRes, double azimMin, double azimMax, double azimRes);

  /**
   * Destructor
   */
  virtual ~VelodyneHDL32E();

  /**
   * lookupIndex
   * @param[in] inclIndex calculated in terms of sensormodel, i.e. SensorPolar builds up the Raycast and _indexMap for each azimuth angle from the lowest
   * inclination angle (inclMin) up to the highest inclination angle (inclMax), in resolution (inclRes) steps
   * @return indexCorrected returns the corresponding inclination index of the real sensor data, taking into account its firing sequence
   * This function matches the firing sequence of real sensor data to _indexMap. This generic model builds the _indexMap for each azimuth starting from the
   * lowest inclination value and incrementing +inclRes until inclMax is reached. Then the current azimuth is incremented +azimRes & the said is repeated for
   * all azimuths in the plane The real sensor data might come in a differnt order though! I.e. in the case of the Velodyne PUCK VLP16, the firing sequence
   * "lists lasers in the order they are fired. Though the VLP-16's lasers are organized in a single, vertical column, they are not fired from one end to the
   * other. Instead, the firing sequence "hops around." This is to avoid "cross-talk" or interference." (see VLP-16 User Manual, page 54ff) Firing sequence
   * for Velodyne HDL-32E: see HDL-32E User Manual, page 62ff
   */
  int lookupIndex(int inclIndex);
};

} // namespace obvious