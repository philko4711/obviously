#include "obcore/math/linalg/eigen/Matrix.h"
#include "obvision/reconstruct/Sensor.h"

namespace obvious {

/**
 * @class SensorPolar
 * @brief base class for polar 3D laser scanners (VLP16 PUCK, E32) / tilting
 * systems (tilt_3d, tilt4d)
 * @brief inherit from this class and implement method lookupIndex for each new
 * sensor you want to integrate
 * @author Jasmin Ziegler
 */
class SensorPolar : public Sensor {
public:
  /**
   * Standard constructor
   * @param[in] raysIncl number of inclination rays of scanning device
   * (vertical)
   * @param[in] inclMin lowest inclination angle in RAD, i.e. inclination start
   * angle (first data captured)
   * @param[in] inclMax highest inclination angle in RAD, i.e. inclination stop
   * angle (last data captured)
   * @param[in] inclRes resolution of inclination rays in RAD, i.e. angle
   * between two vertical rays
   * @param[in] azimMin lowest azimuth angle in RAD
   * @param[in] azimMax highest azimuth angle in RAD
   * @param[in] azimRes resolution of azimuth rays in RAD, angle between two
   * horizontal rays in 360° plane
   * @param[in] firingSeq order in which vertical laser rays are fired, count
   * from inclMin to inclMax ind inclRes steps (e.g.: inclMin = -2°, inclMax =
   * 2°, inclRes = 2°. Find out in which order the laser measures the following
   * angles: -2° (idx 0), 0° (idx 1), 2° (idx 2). If laser measures 2° (idx 0)
   * inclination first, followed by -2° (idx 1) and 0° (idx 2) last, the firing
   * sequence to translate to the sensor model order from inclMin to inclMax
   * would be: {1, 2, 0}); The firing sequence is empty by default because some
   * sensors do not need this index mapping bec. they already fire from inclMin
   * to inclMax or bec. the sensor data was sorted previously. If firingSeq is
   * empty, function lookupIndex is ignored (default).
   * firingSeq for Velodyne: firingSeq={0, 2, 4, 6, 8, 10, 12, 14, 1, 3, 5, 7,
   * 9, 11, 13, 15}; (check VLP16 User Manual p. 54ff)
   * firingSeq for Velodyne HDL-32E: firingSeq={0, 2, 4, 6, 8, 10, 12, 14, 16,
   * 18, 20, 22, 24, 26, 28, 30,  1, 3, 5, 7, 9, 11, 13, 15, 17, 19, 21, 23, 25,
   * 27, 29, 31}; (check HDL-32E User Manual p. 62ff)
   * @param[in] maxRange maximum measurement range (inherited from class Sensor)
   * @param[in] minRange minimum measurement range, i.e. a dead zone (inherited
   * from class Sensor)
   * @param[in] lowReflectivityRange range for objects with low remission
   * (inherited from class Sensor)
   * note when integrating a new sensor: if the scan order (firing sequence)
   * increments evenly from inclMin to inclMax in inclRes steps, you can proceed
   * without class INDEXMATCH BUT: if the scan order (firing sequence) of the
   * lasers is not evenly, you have to match indices. An example is given with
   * Velodyne VLP16 and HDL32-E, check for more details.
   */
  SensorPolar(unsigned int raysIncl, double inclMin, double inclMax,
              double inclRes, double azimMin, double azimMax, double azimRes,
              std::vector<int> firingSeq = {}, double maxRange = INFINITY,
              double minRange = 0.0, double lowReflectivityRange = INFINITY);

  /**
   * Destructor
   */
  virtual ~SensorPolar();

  /**
   * match firing sequence of real sensor data to _indexMap. This generic model
   * builds the _indexMap for each azimuth starting from the lowest inclination
   * value and incrementing +inclRes until inclMax is reached. Then the current
   * azimuth is incremented +azimRes & the said is repeated for all azimuths in
   * the plane The real sensor data might come in a differnt order though! I.e.
   * in the case of the Velodyne PUCK VLP16, the firing sequence "lists lasers
   * in
   * the order they are fired. Though the VLP-16's lasers are organized in a
   * single, vertical column, they are not fired from one end to the other.
   * Instead,
   * the firing sequence "hops around." This is to avoid "cross-talk" or
   * interference." (see VLP-16 User Manual, page 54ff)
   * the firing sequence is defined as a std::vec in each Sensor Interface. It
   * is empty by default, which means the firing seq is "in order" from inclMin
   * to inclMax.
   * If it is empty, lookupIndex is ignored. If it has values, lookupIndex is
   * called from backProject before backprojecting indices.
   * NOTE: this function currently has 32 switch cases because of Velodyne
   * HDL32-E and its 32 vertical channels. VLP16 with 16 vertical channels also
   * uses this function right now. For future integrations: increase number of
   * switch cases if you want to integrate a new sensor with more than 32
   * vertical channels!
   */
  int lookupIndex(int inclIndex);

  /**
   * Project all coordinates (center of each voxel in tsd space) back to sensor
   * index: which sensor ray comes closest to the coordinate?
   * @param[in] M matrix of 3D coordinates of all voxel center points in tsd
   * space (homogeneous)
   * @param[out] indices vector of projection results (must be allocated
   * outside)
   * @param[in] T temporary transformation matrix of coordinates
   */
  void backProject(obvious::Matrix *M, int *indices, obvious::Matrix *T = NULL);

private:
  double _inclRes;
  double _inclMin;
  double _inclMax;
  double _inclNegSpan;
  double _azimRes;
  double _azimMin;
  std::vector<int> _firingSeq;
  int **_indexMap;
};

} // namespace obvious