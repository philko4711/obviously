/**
* @file  CamNano.cpp
* @autor Christian Pfitzner
* @date  08.11.2012
*
*/


#include "CamNano.h"
#include "obcore/base/Logger.h"
#include "obcore/math/mathbase.h"
#include <math.h>


using namespace obvious;

/*
 * Standard constructor of class CamNano
 */
CamNano::CamNano()
  : ParentDevice3D(165,120)
{
   _res = pmdOpen(&_hnd, SOURCE_PLUGIN, SOURCE_PARAM, PROC_PLUGIN, PROC_PARAM);
   sleep(1);
   if (_res != PMD_OK)
   {
     std::cout << "Error code: " <<_res << std::endl;
     LOGMSG(DBG_ERROR, "Error openning sensor");
     exit(1);
   }
   else
     LOGMSG(DBG_ERROR, "Opened sensor");

  /*
  * config pmd processing
  */
  char lens[128];
  pmdProcessingCommand(_hnd, lens, 128,  "GetLensParameters");
  pmdProcessingCommand(_hnd, 0, 0,       "SetAveraging On");
  pmdProcessingCommand(_hnd, 0, 0,       "SetAveragingFrames 2");
  pmdProcessingCommand(_hnd, 0, 0,       "SetConsistencyThreshold 0.5");
  pmdProcessingCommand(_hnd, 0, 0,       "SetBilateralFilterKernelSize 10");
  pmdProcessingCommand(_hnd, 0, 0,       "SetBilateralFilter on");
  pmdProcessingCommand(_hnd, 0, 0,       "SetBilateralFitlerSigmaRange 15");
  pmdProcessingCommand(_hnd, 0, 0,       "SetBilateralFilterenhanceImage off");
  pmdProcessingCommand(_hnd, 0, 0,       "SetSignalStrengthCheck On");

  /*
  * config logging messages
  */
  LOGMSG_CONF("CamNano.log", Logger::file_off|Logger::screen_on, DBG_DEBUG, DBG_ERROR);

  _points    = _rows*_cols;
  _coordsV   = new double[_points*3];
  _coordsF   = new float [_points*3];
  _image     = new unsigned char [3*_points];
  _imageF    = new float [_points];
  _dist      = new float [_points];
  _amp       = new float [_points];

  _meanAmp      = 0.0f;
  _intTime      = 0.0f;
  _autoIntegrat = true;
  _rawSet       = false;
  _intrinsic    = true;
  _init         = true;
  _debug        = false;

  // init of pid controller
  _ctrl.setDebug(_debug);
  _ctrl.setP(2.0f);
  _ctrl.setI(0.5f);
  _ctrl.setD(0.0f);
  _ctrl.setAWU(30.f);
  _ctrl.setSetValue(250);
  _ctrl.setMinValue(MIN_INTEGRATIONTIME);
  _ctrl.setMaxValue(MAX_INTEGRATIONTIME);
}

/*
 * Standard destructor of class CamNano
 */
CamNano::~CamNano()
{
  //delete [] _coords;
  delete [] _coordsF;
  delete [] _image;
  delete [] _imageF;
  delete [] _coordsV;
  delete [] _dist;
  delete [] _amp;
  pmdClose(_hnd);
}

double* CamNano::getDistImage()
{
  double* _distD = new double [_cols * _rows];
  for(unsigned int i=0 ; i<_cols*_rows ; i++)
    _distD[i] = _dist[i];
  return _distD;
}

unsigned int CamNano::getValidSize(void) const
{
  return _points;
}

/*
 * Function to grab raw data from camera
 */
void CamNano::setRaw(bool raw)
{
  _rawSet = raw;
}

/*
 * Function to grab data from camera
 */
bool CamNano::grab()
{
  // check if sensor initialized
  if (!_init) {
    LOGMSG(DBG_ERROR, "Sensor uninitialized");
    pmdClose(_hnd);
    return(false);
  }

  this->estimateFrameRate();
  this->updateSensor();
  this->grabDistance();
  this->grabAmplitudes();
  this->grabCoordinates();

  _imageF = _amp;
  unsigned int maxval = 0;
  unsigned int minval = 10000;
  for (unsigned int i=0; i<_rows*_cols; i++) {
    if(_amp[i] > maxval)
      maxval = _amp[i];
    if(_amp[i] < minval)
      minval = _amp[i];
  }

  unsigned int k=0;
  for(unsigned int i=0 ; i<_rows*_cols*3 ; i+=3, k++)
  {
    _coords[i]    = -(double)_coordsF[i];
    _coords[i+1]  = (double)_coordsF[i+1];
    _coords[i+2]  = (double)_coordsF[i+2];
    _rgb[i]       = (_amp[k]-minval) * 255 / (maxval);
    _rgb[i+1]     = (_amp[k]-minval) * 255 / (maxval);
    _rgb[i+2]     = (_amp[k]-minval) * 255 / (maxval);
    _z[k]         = _coords[i+2] / 1000.0;

    double n[24];

    int idx[8];
    idx[0] = {i-_rows-3};       // up left
    idx[1] = {i-_rows  };       // up
    idx[2] = {i-_rows+3};       // up right
    idx[3] = {i-3};             // left
    idx[4] = {i+3};             // right
    idx[5] = {i+_rows-3};       // down left
    idx[6] = {i+_rows  };       // down
    idx[7] = {i+_rows+3};       // down right

    // caluclate indices of neighbours
    for(unsigned int j=0 ; j<8 ; j++) {
      if(idx[j]>=0 && idx[j] <=_rows*_cols*3)
        memcpy(&n[3*j],      &(_coords[idx[j]]), 3*sizeof(double));
    }

    double dist      = abs3D(&_coords[i]);
    double alpha_max = 0.0;
    // check for eigth neighbours
    for(unsigned int j=0 ; j<8 ; j++)
    {
      double n2[3];
      n2[0] = n[3*j+0]  - _coords[i+0];
      n2[1] = n[3*j+1]  - _coords[i+1];
      n2[2] = n[3*j+2]  - _coords[i+2];

      double dotProduct = dot3<double>(&_coords[i], n2);
      if(dotProduct<0)
      {
        double alpha = acos(dotProduct / (abs3D(n2) * dist));
        if(alpha_max<alpha) alpha_max = alpha;
      }
    }
    bool angleFilter = false;
    if(alpha_max>deg2rad(150.0))
      angleFilter = true;

    bool edge = false;
    if (i%(3*_cols)==0 || i%(3*_cols)==3 || i%(3*_cols)==6 || i%(3*_cols)==9 || i%(3*_cols)==12)
        edge = true;

    _mask[k]      = (!isnan(_z[k]))                &&
                    (_amp[k]>AMP_THRESHOLD)        &&
                    (_z[k] < DIST_THRESHOLD_MAX)   &&
                    !angleFilter && !edge;
                    /*
                    (_z[k] > DIST_THRESHOLD_MIN); */
  }

  if (_autoIntegrat)
    this->setAutoIntegration();
  return(true);
}

/*
 * Function to get valid coords
 */
double* CamNano::getValidCoords(void)
{
  unsigned int v=0;
  for(unsigned int i=0 ; i<_rows*_cols ; i++)
  {
    if(_mask[i] == true)
    {
      _coordsV[v]   = _coords[3*i];
      _coordsV[v+1] = _coords[3*i+1];
      _coordsV[v+2] = _coords[3*i+2];
      v+=3;
    }
  }
  _points = v/3;
  return(_coordsV);
}

/*
 * Function to set integration time
 */
void CamNano::setIntegrationTime(unsigned value)
{
  _res = pmdSetIntegrationTime(_hnd, 0, value);
  if (_res != PMD_OK)
  {
    LOGMSG(DBG_ERROR, "Wrong integration time");
  }
  else
    _intTime = value;

  _autoIntegrat = false;
}

/*
 * Function to switch between autonomous and static integration time
 */
void CamNano::setIntegrationAuto(bool autom)
{
  _autoIntegrat = autom;
}

/*
 * Function to get integration Time
 */
float CamNano::getIntegrationTime(void) const
{
  return (_intTime);
}

/*
 * Function to show set parameters of camera
 */
void CamNano::showParameters(void)
{
   unsigned i;
   char serialNr[128];
   _res = pmdGetIntegrationTime (_hnd, &i, 0);
   _res = pmdSourceCommand(_hnd, serialNr, sizeof(serialNr), "GetSerialNumber");

   std::cout << "Serial number of device: " << serialNr           << std::endl;
   std::cout << "Integration time: "     << i <<  " microseconds" << std::endl;
   std::cout << "Modulation frequency: 30 MHz"                    << std::endl;
}

/*
 * Function to return image array
 */
unsigned char* CamNano::getImage(void) const
{
  float minMag = NAN;
  float maxMag = 0;
  for (unsigned int i=0 ; i<_rows*_cols ; i++)
  {
    if (_imageF[i] < minMag)
      minMag = _imageF[i];
    if (_imageF[i] > maxMag)
      maxMag = _imageF[i];
  }

  float range = minMag - maxMag;
  for (unsigned int i=0 ; i < _rows*_cols ; i++)
  {
     _image[3*i]   = (unsigned char)((_imageF[3*i]   - minMag)/range*255);
     _image[3*i+1] = (unsigned char)((_imageF[3*i+1] - minMag)/range*255);
     _image[3*i+2] = (unsigned char)((_imageF[3*i+2] - minMag)/range*255);
  }
  return _image;
}

/*
 * Function to activate debug mode
 */
void CamNano::setDebug(bool debug)
{
  _debug = debug;
}


bool
CamNano::updateSensor(void)
{
  _res = pmdUpdate(_hnd);
  if (_res != PMD_OK)
  {
    LOGMSG(DBG_ERROR, "Error updating sensor");
    pmdClose (_hnd);
    return(false);
  }

  _res = pmdGetSourceDataDescription(_hnd, &_dd);
  if (_res != PMD_OK)
  {
    LOGMSG(DBG_ERROR, "Error updating data description");
    pmdClose (_hnd);
    return(false);
  }
  return(true);
}

bool
CamNano::grabDistance()
{
  _res = pmdGetDistances(_hnd, _dist, _rows*_cols * sizeof(float));
  if (_res != PMD_OK)
  {
    LOGMSG(DBG_ERROR, "Error getting the distance");
    pmdClose(_hnd);
    return(false);
  }
  else
  {
    return(true);
  }
}

bool
CamNano::grabAmplitudes()
{
  // get array with amplitudes from sensor
  _res = pmdGetAmplitudes(_hnd, _amp, _rows*_cols * sizeof(float));
  if (_res != PMD_OK)
  {
    LOGMSG(DBG_ERROR, "Error getting the amplitudes");
    pmdClose(_hnd);
    return(false);
  }
  else
    return(true);
}

bool
CamNano::grabCoordinates(void)
{
  // calculate 3d coordinates
  _res = pmdGet3DCoordinates(_hnd, _coordsF, _rows*_cols * sizeof(float) * 3);
  if (_res != PMD_OK)
  {
    LOGMSG(DBG_ERROR, "Error getting coordinates");
    pmdClose(_hnd);
    return(false);
  }
  else
    return(true);
}

/*
 * Function to set automatic integration time
 */
void
CamNano::setAutoIntegration(void)
{
  _ctrl.setSetValue(300);
  _intTime = _ctrl.controll(_meanAmp);
  setIntegrationTime(_intTime);
  if (_debug)
    this->showParameters();
}


