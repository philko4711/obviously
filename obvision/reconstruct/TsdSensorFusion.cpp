/**
 * @file TsdSensorFusion.cpp
 * @autor christian
 * @date  09.11.2013
 *
 *
 */
#include "TsdSensorFusion.h"

using namespace obvious;

TsdSensorFusion::TsdSensorFusion(TsdSpace* space, Icp* icp, OutOfBoundsFilter3D* outBountFilter)
{
  _space              = space;
  _icp                = icp;
  _outboundFilter     = outBountFilter;
  _current_sensor     = NULL;
  _current_sensor_idx = 0;
}

TsdSensorFusion::~TsdSensorFusion()
{
  // TODO Auto-generated destructor stub
}

void
TsdSensorFusion::addSensor(Sensor* sensor, unsigned int* id)
{
  _sensors.push_back(sensor);         // add sensor to vector
  _current_sensor = _sensors.back();  // point on last object in vector
}

Sensor*
TsdSensorFusion::getCurrentSensor(void)
{
  return(_current_sensor);
}

unsigned int
TsdSensorFusion::getCurrentSensorID(void)
{
  return _current_sensor_idx;
}

void
TsdSensorFusion::switchSensor(unsigned int number)
{
  if(_sensors.size() <= number)
    _current_sensor == _sensors[number];
  else
    LOGMSG(DBG_ERROR, "Sensor not found. Current size of sensors " << _sensors.size());
}

bool
TsdSensorFusion::localiseCurrentSensor(double* data, unsigned int size, bool* mask)
{
  double P[12];

  // get model from space
  unsigned int sizeRaycast;
  double* coords      = new double        [_current_sensor->getWidth() * _current_sensor->getHeight() * 3];
  double* normals     = new double        [_current_sensor->getWidth() * _current_sensor->getHeight() * 3];
  unsigned char* rgb = new unsigned char[_current_sensor->getWidth() * _current_sensor->getHeight() * 3];
  this->getPointCloudFromSensorView(coords, normals, rgb, &sizeRaycast);

  // put everything into icp for localization
  _outboundFilter->setPose(_current_sensor->getPose());
  _icp->reset();
  _icp->setScene(data, NULL, size);
  _icp->setModel(coords, normals, sizeRaycast/3);

  // Perform ICP registration
  double       rms        = 0;
  unsigned int pairs      = 0;
  unsigned int iterations = 0;
  EnumIcpState state = _icp->iterate(&rms, &pairs, &iterations);

  if(((state == ICP_SUCCESS)       && (rms < 0.1)) ||
     ((state == ICP_MAXITERATIONS) && (rms < 0.1)))
  {
    Matrix* T = _icp->getFinalTransformation();
    T->print();
    _current_sensor->transform(T);
    _current_sensor->getPose()->getData(P);
  }
  else
  {
    LOGMSG(DBG_DEBUG, "Registration failed, RMS " << rms);
  }
  return(true);
}

bool
TsdSensorFusion::pushCurrentSensor(double* data, bool* mask)
{
  _current_sensor->setRealMeasurementData(data);
  _current_sensor->setRealMeasurementMask(mask);

  // only push if not registration
  _space->push(_current_sensor);

  return(true);
}

void
obvious::TsdSensorFusion::getPointCloudFromSensorView(double* coords,
    double* normals, unsigned char* rgb, unsigned int* size)
{
  RayCast3D raycaster(_space);
  raycaster.calcCoordsFromCurrentPose(_current_sensor, coords, normals, rgb, size);
//  for(unsigned int i=0 ; i<*size ; i+=3) {
//        coords[i]   = -coords[i];
//        coords[i+1] = -coords[i+1];
//      }
}

void
obvious::TsdSensorFusion::getPointCloudFromSensorView(VtkCloud* cloud)
{
  RayCast3D raycaster(_space);
  unsigned int size  = _current_sensor->getWidth() * _current_sensor->getHeight() * 3;
  double* coords      = new double[size];
  double* normals     = new double[size];
  unsigned char* rgb = new unsigned char[size];

  unsigned int sizeFromRayCast = 0;
  raycaster.calcCoordsFromCurrentPose(_current_sensor, coords, normals, rgb, &sizeFromRayCast);

//  for(unsigned int i=0 ; i<sizeFromRayCast ; i+=3) {
//        coords[i]   = -coords[i];
//        coords[i+1] = -coords[i+1];
//      }

  double P[16];
  cloud->setCoords(coords, sizeFromRayCast/3, 3);
  _current_sensor->getPose()->getData(P);
  cloud->transform(P);

  delete [] coords;
  delete [] normals;
  delete [] rgb;
}
