/*
 * SickLMS100.h
 *
 *  Created on: 11.03.2013
 *      Author: christian
 */



#ifndef SICKLMS100_H_
#define SICKLMS100_H_

#include "obdevice/LaserDevice.h"
#include <LMS1xx.h>

/**
 * @namespace obvious
 */
namespace obvious
{
/**
 * @class LaserDevice
 */
class SickLMS100 : public LaserDevice
{
public:
  /**
   * Standard Constructor
   */
  SickLMS100(double minAngle, double maxAngle, unsigned int rays);
  /**
   * Default Destructor
   */
  virtual   ~SickLMS100();
  /**
   * Function to grab new data
   * @return  TRUE if success
   */
  bool      grab(void);
private:
  inline double estimateAngularRes()      { return((_maxAngle-_minAngle)/_nrOfRays); }
  /**
   * Function to estimate ranges in scan
   */
  void estimateRanges();
  /**
   * Function to estimate intensities in scan
   */
  virtual void estimateIntensities();
  /**
   * Function to estimate single angles for every ray
   */
  virtual void estimateAngles();
  /**
   * Function to estimate 2D coords
   */
  virtual void estimateCoords2D();
  /**
   * Function to estimate mask
   */
  virtual void estimateMask();

  LMS1xx      _laser;
  scanCfg     _cfg;
  scanDataCfg _dataCfg;
  scanData    _data;


};

};  //namespace


#endif /* SICKLMS100_H_ */
