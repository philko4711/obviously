#include "obvision/reconstruct/space/SensorPolar.h"

namespace obvious
{

class Tilt4d : public SensorPolar
{
public:
  /**
   * Standard Constructor
   * This class uses the Velodyne VLP16 sensor.
   * @param[in] raysIncl number of inclination measurement slots in vertical scanning area:  ( |tiltStart - tiltStop| ) / inclRes or even more accurate depending on step_res (see below in note)
   * @param[in] inclMin lowest inclination angle in RAD, i.e. first inclination angle at tilting start position
   * @param[in] inclMax highest inclination angle in RAD, i.e. last inclination angle at tilting stop position
   * @param[in] inclRes resolution of inclination rays in RAD (0.034907 = 2°), i.e. angle between two vertical rays
   * @param[in] azimMin lowest azimuth angle in RAD (0.0 = 0°)
   * @param[in] azimMax highest azimuth angle in RAD (6.28319 = 360°) --> check horizontal field of view --> probably reduced because robot covers a specific range of the horizontal field of view during the tilting process
   * @param[in] azimRes resolution of azimuth rays in RAD (0.00349066 = 0.2°), angle between two horizontal rays in 360° plane, depending on rotation speed (RPM) of sensor, default RPM = 600 --> azimRes = 0.2° (300RPM --> 0.1°, 900RPM --> 0.3°, 1200RPM --> 0.4°)
   * @note the tilting process will result in multiple measurements per inclination slot. Example: tilt start inclination angle = 180°, tilt stop angle = 0°. In the entire inclination span of 180°, we have 180°/2° = 90 inclination slots/buckets. 
   * When we start tilting at 180°, we cover a vertical aperture of 30° with the VLP16, so we get measurements from 180° to 150° and with a inclRes of 2° we receive 16 measurements/buckets (180°, 178°, 176°...). 
   * Now, the sensor is tilted by a step_res of for example 0.1° (see ohm_sensors/ohm_4d_scanner/cfg/FdScanner.cfg), capturing the vertical area from 179.9° - 149.9°. In the next tilt, again +0.1°, 179.8° - 149.8° and so on.
   * At one point, the sensors tilting position will arrive exactly at 178°, so the 16 measurements in inclRes 2° distance from each other will return values for the inclination angles 178°, 176°, 174°....
   * But for these values we already have measurements from the first scan (see above). 
   * What needs to be done now: in ROS node polar_sensor_models (https://github.com/JuicyJay/polar_sensor_models), implement a "bucket" sort function that collects multiple measurements in a vector of vectors
   * and calculates the average distance so the result is one single depth measurement value per inclination measurement slot
   * IMPORTANT: The number of measurement slots must be the same number as the size of the sensor which is calculated by raysIncl*raysAzim (=width*height) 
   */
  Tilt4d(unsigned int raysIncl, double inclMin, double inclMax, double inclRes, double azimMin, double azimMax, double azimRes);

  /**
   * Destructor
   */
  virtual ~Tilt4d();
};

} // namespace obvious