/**
 * @file TsdSensorFusion.h
 * @autor christian
 * @date  09.11.2013
 *
 *
 */
#ifndef TSDSENSORFUSION_H_
#define TSDSENSORFUSION_H_

#include "obvision/reconstruct/Sensor.h"
#include "obvision/reconstruct/SensorProjective3D.h"
#include <vector>

#include "obvision/reconstruct/TsdSpace.h"
#include "obvision/icp/icp_def.h"
#include "obvision/reconstruct/RayCast3D.h"
#include "obcore/base/Timer.h"
#include "obcore/base/Logger.h"
#include "obcore/base/tools.h"
#include "obcore/math/mathbase.h"

#include "obgraphic/VtkCloud.h"

namespace obvious
{
/**
 * @class   TsdSensorFusion
 * @date    2013-11-09
 * @author  Christian Pfitzner
 */
class TsdSensorFusion
{
public:
  /**
   * Default constructor
   */
  TsdSensorFusion(TsdSpace* space, Icp* icp,  OutOfBoundsFilter3D* outBountFilter);
  /**
   * Default destructor
   */
  virtual
  ~TsdSensorFusion();
  /**
   * Function to add a sensor to the sensor array
   * @param   sensor
   * @param   id of sensor
   */
  void
  addSensor(Sensor* sensor, unsigned int* id);

  Sensor* getCurrentSensor(void);

  unsigned int getCurrentSensorID(void);
  /**
   * Function to switch between sensors
   * @param number
   */
  void
  switchSensor(unsigned int number = 0);
  /**
   * Function to register sensor to
   * @return  success
   */
  bool
  localiseCurrentSensor(double* data, unsigned int size, bool* mask = NULL);
  /**
   * Function to push sensor data to gri
   * @return  success
   */
  bool
  pushCurrentSensor(double* data, bool* mask = NULL);
  /**
   * Function to return point cloud from current sensor view
   * @param coords
   * @param normals
   * @param rgb
   * @param size
   */
  void getPointCloudFromSensorView(double* coords, double* normals, unsigned char* rgb, unsigned int* size);
  /**
   * Function to return vtkcloud by raycasting from space
   * @param cloud
   */
  void getPointCloudFromSensorView(VtkCloud* cloud);
private:
  TsdSpace*             _space;
  Icp*                  _icp;
  OutOfBoundsFilter3D*  _outboundFilter;
  std::vector<Sensor*>    _sensors;
  Sensor*                _current_sensor;
  unsigned int         _current_sensor_idx;
};

} /* namespace obvious */


#endif /* TSDSENSORFUSION_H_ */
