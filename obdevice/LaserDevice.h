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
    : _minAngle(minAngle), _maxAngle(maxAngle), _nrOfRays(rays),
      _ranges(0), _intensities(0), _angles(0), _mask(0),
      _angRes(estimateAngularRes()) { }
  /**
   * Default Destructor
   */
  virtual   ~LaserDevice()
  {
    delete[] _ranges;
    delete[] _intensities;
    delete[] _angles;
    delete[] _mask;
  };
  /**
   * Function to grab new data
   * @return  TRUE if success
   */
  virtual bool      grab(void) = 0;

  unsigned int getNumberOfRays() const { return _nrOfRays; }
  /**
   * Function to return Distances
   * @return  Distance in meters
   */
  double*   getDistance() const    { return _ranges; }
  /**
   * Function to return intensities
   * @return  Intensity value
   */
  double*   getIntensity() const   { return _intensities; }
  /**
   * Function to return angles
   * @return  angles in rad
   */
  double*   getAngle() const        { return _angles; }
  /**
   * Function to return mask of valid points
   * @return  mask with valid points (TRUE)
   */
  bool*   getMask()                  { return _mask; }
protected:
  /**
   * Function to estimate ranges in scan
   */
  virtual void estimateRanges(void) { }
  /**
   * Function to estimate intensities in scan
   */
  virtual void estimateIntensities(void) { }
  /**
   * Function to estimate single angles for every ray
   */
  virtual void estimateAngles(void) { }
  /**
   * Function to estimate 2D coords
   */
  virtual void estimateCoords2D(void) { }
  /**
   * Function to estimate mask
   */
  virtual void estimateMask();
  double*   _ranges;            //!< Distance in meters
  double*   _intensities;       //!< Intensities
  double*   _coords2D;          //!< 2D coords
  double*   _angles;            //!< Angles in rad
  bool*     _mask;              //!< mask for valid or invalid points
  /*~~~~~~~~~~~~~~~~~~~~~~~~~~ Configuration ~~~~~~~~~~~~~~~~~~~~~~~~~*/
  double            _minAngle;
  double            _maxAngle;
  unsigned int      _nrOfRays;
  double            _angRes;

  double estimateAngularRes(void)      { return(1.0); }//(double)(_maxAngle-_minAngle)/_nrOfRays); }



};

};  //namespace


#endif /* LASERDEVICE_H_ */
