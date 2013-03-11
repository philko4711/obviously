/*
 * LaserDevice.h
 *
 *  Created on: 11.03.2013
 *      Author: christian
 */

#ifndef LASERDEVICE_H_
#define LASERDEVICE_H_

/**
 * @namespace obvious
 */
namespace obvious
{
/**
 * @class LaserDevice
 */
class LaserDevice
{
public:
  /**
   * Standard Constructor
   */
  LaserDevice(double minAngle, double maxAngle, unsigned int rays)
    : _minAngle(minAngle), _maxAngle(maxAngle), _nrOfRays(rays) { }
  /**
   * Default Destructor
   */
  virtual   ~LaserDevice()
  {
    delete[] _distances;
    delete[] _intensity;
    delete[] _angle;
    delete[] _mask;
  };
  /**
   * Function to grab new data
   * @return  TRUE if success
   */
  virtual bool      grab(void) = 0;
  /**
   * Function to return Distances
   * @return  Distance in meters
   */
  double*   getDistance()   { return _distances; }
  /**
   * Function to return intensities
   * @return  Intensity value
   */
  double*   getIntensity()  { return _intensity; }
  /**
   * Function to return angles
   * @return  angles in rad
   */
  double*   getAngle()       { return _angle; }
  /**
   * Function to return mask of valid points
   * @return  mask with valid points (TRUE)
   */
  double*   getMask()         { return _mask; }
private:
  double*   _distances;       //!< Distance in meters
  double*   _intensity;       //!< Intensities
  double*   _angle;           //!< Angles in rad
  bool*     _mask;            //!< mask for valid or invalid points
  /*~~~~~~~~~~~~~~~~~~~~~~~~~~ Configuration ~~~~~~~~~~~~~~~~~~~~~~~~~*/
  double            _minAngle;
  double            _maxAngle;
  unsigned int      _nrOfRays;
  double            _angRes;


};

};  //namespace


#endif /* LASERDEVICE_H_ */
