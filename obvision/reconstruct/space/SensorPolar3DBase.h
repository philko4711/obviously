#ifndef OBVISION_RECONSTRUCT_SPACE_SENSORPOLAR3DBASE_H_
#define OBVISION_RECONSTRUCT_SPACE_SENSORPOLAR3DBASE_H_

#include "obcore/math/linalg/eigen/Matrix.h"
#include "obvision/reconstruct/Sensor.h"

namespace obvious
{
/**
 * @class SensorPolar3DBase
 * @brief generic sensor model class for polar sensing units (to be tested for: Velodyne VLP16 PUCK, Velodyne HDL-32E, ohm_tilt_scanner_3d, SICK LDMRS8, ohm_4d_scanner, Ouster OS0-128, & many more)
 * @brief all polar sensor model classes inherit from this class & implement lookupIndex
 * @brief this class itself inherits basic sensor modalities from class Sensor 
 * @author Jasmin Ziegler
 */
class SensorPolar3DBase : public Sensor
{
public:

/**
 * Standard Constructor
 * @param[in] inclMin lowest inclination angle [RAD] (vertical)
 * @param[in] inclMax highest inclination angle in [RAD] (vertical)
 * @param[in] inclRes vertical angular resolution [RAD]
 * @param[in] azimMin lowest azimuth angle [RAD] (horizontal)
 * @param[in] azimMax highest azimuth angle in [RAD] (horizontal)
 * @param[in] azimRes horizontal angular resolution [RAD]
 */
SensorPolar3DBase(double inclMin, double inclMax, double inclRes, double azimMin, double azimMax, double azimRes, double maxRange = INFINITY, double minRange = 0.0, double lowReflectivityRange = INFINITY);
 
/**
 * Destructor
 */
virtual ~SensorPolar3DBase();

  /**
   * Project all coordinates (center of each voxel in tsd space) back to sensor index: which sensor ray comes closest to the coordinate?
   * @param[in] M matrix of 3D coordinates of all voxel center points in tsd space (homogeneous)
   * @param[out] indices vector of projection results (must be allocated outside)
   * @param[in] T temporary transformation matrix of coordinates
   */
void backProject(obvious::Matrix* M, int* indices, obvious::Matrix* T = NULL);

private:
double _azimRes;
double _inclRes;

};

} /*namespace obvious */

#endif /* OBVISION_RECONSTRUCT_SPACE_SENSORPOLAR3DBASE_H_ */