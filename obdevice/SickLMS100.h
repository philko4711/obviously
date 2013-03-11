/*
 * SickLMS100.h
 *
 *  Created on: 11.03.2013
 *      Author: christian
 */



#ifndef SICKLMS100_H_
#define SICKLMS100_H_

#include "obdevice/LaserDevice.h"

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


};

};  //namespace


#endif /* SICKLMS100_H_ */
