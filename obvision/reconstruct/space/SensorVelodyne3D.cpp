#include "SensorVelodyne3D.h"
#include "obcore/base/System.h"
#include "obcore/math/mathbase.h"
#include <limits>

/**
// NEW - rows and cols switched
namespace obvious
{
SensorVelodyne3D::SensorVelodyne3D(unsigned int raysIncl, double inclMin, double inclRes, double azimRes, double maxRange, double minRange,
                                   double lowReflectivityRange)
    : Sensor(3, maxRange, minRange, lowReflectivityRange)
{
  _azimRes  = azimRes;                    // 0.2° in RAD
  _inclRes  = inclRes;                    // 2° in RAD
  _raysIncl = raysIncl;                   // 16
  _inclMin  = inclMin;                    //-15° in RAD
  _inclSpan = _inclRes * (_raysIncl - 1); // CHANGE THIS IN GENERIC MODEL !!!!
  _azimMin  = 0.0;

  // unsigned int raysAzim = round(static_cast<unsigned>(2 * M_PI / azimRes));
  unsigned int raysAzim = round(2 * M_PI / azimRes);

  std::cout << "raysAzim HEHEHEHEH = " << raysAzim << std::endl;

  _width  = raysAzim + 1; // weil 0° und 360°
  _height = raysIncl;     // VORSICHT das ist hier dann 16, würde nicht auch 15 reichen?
  _size   = _width * _height;

  _data = new double[_size];
  _mask = new bool[_size];
  for(unsigned int i = 0; i < _size; i++)
    _mask[i] = true;

  obvious::System<int>::allocate(_height, _width, _indexMap);

  // set IndexMap - row col switched here
  unsigned int column = 0;
  for(unsigned int row = 0; row < _height; row++) // INCL
  {
    for(column = 0; column < _width; column++) // AZIM
    {
      _indexMap[row][column] = row * (_width) + column;
      // std::cout << "row = " << row << " , col = " << column << ", indexMap = " << _indexMap[row][column] << std::endl;
    }
    column = 0; // iterate over azim rays for each incl ray
  }

  _rays             = new obvious::Matrix(3, _size);
  obvious::Matrix R = obvious::Matrix(*_T, 0, 0, 3, 3);

  unsigned int n = 0;
  // for(unsigned int i = 0; i < _width; i++) // AZIM 0 - 1800 : passt weil 0° und 360° gell?
  // {
  //   double currentAzim = _azimMin + i * _azimRes;
  //   std::cout << "currentAzim = " << rad2deg(currentAzim) << std::endl;

  //   for(unsigned int j = 0; j < _height; j++, n++) // INCL 0 - 15 : passt weil 16 rays
  //   {
  //     double currentIncl = _inclMin + j * _inclRes;
  //     std::cout << "currentIncl = " << rad2deg(currentIncl) << " @ n = " << n << " !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
  for(unsigned int i = 0; i < _height; i++) // INCL
  {
    double currentIncl = _inclMin + i * _inclRes;
    // std::cout << "currentIncl = " << rad2deg(currentIncl) << std::endl;

    for(unsigned int j = 0; j < _width; j++, n++) // AZIM
    {
      double currentAzim = _azimMin + j * _azimRes;
      // std::cout << "currentAzim = " << rad2deg(currentAzim) << std::endl;
      // std::cout << "row i = " << i << " , col j = " << j << ", indexMap = " << _indexMap[i][j] << std::endl;
      // std::cout << " @ n = " << n << " !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;

      obvious::Matrix calcRay(3, 1);

      // ich muss den inclination angle korrigieren sonst geht dit nit :(((((((((((((((())))))))))))))))
      double inclCorr = 0.0;

      if(currentIncl <= 0) // DAS GLEICH HAB ICH ERGÄNZT für wenn =0
      {
        inclCorr = deg2rad(90.0) + (-1.0 * currentIncl);
      }
      else
      {
        inclCorr = deg2rad(90.0) - currentIncl;
      }

      calcRay(0, 0) = sin(inclCorr) * cos(currentAzim);
      calcRay(1, 0) = cos(inclCorr);
      calcRay(2, 0) = sin(inclCorr) * sin(currentAzim);

      const double length    = sqrt(calcRay(0, 0) * calcRay(0, 0) + calcRay(1, 0) * calcRay(1, 0) + calcRay(2, 0) * calcRay(2, 0));
      const double lengthInv = 1.0 / length;

      calcRay = R * calcRay;

      (*_rays)(0, n) = calcRay(0, 0) * lengthInv;
      (*_rays)(1, n) = calcRay(1, 0) * lengthInv;
      (*_rays)(2, n) = calcRay(2, 0) * lengthInv;
    }
  }

  _raysLocal  = new obvious::Matrix(3, _size);
  *_raysLocal = *_rays;
}

SensorVelodyne3D::~SensorVelodyne3D()
{
  delete _rays;
  delete _raysLocal;
  delete[] _data;
  delete[] _mask;
  System<int>::deallocate(_indexMap);
}

// todo - adapt this for E32 and all other sensors
// Sensormodel - VLP (old version)
// unsigned int SensorVelodyne3D::lookupIndex(int indexSensormodel)
// {
//   unsigned int indexVelodyneROS = 0;
//   switch(indexSensormodel)
//   {
//   case 0:
//     indexVelodyneROS = 0;
//     break;
//   case 1:
//     indexVelodyneROS = 2;
//     break;
//   case 2:
//     indexVelodyneROS = 4;
//     break;
//   case 3:
//     indexVelodyneROS = 6;
//     break;
//   case 4:
//     indexVelodyneROS = 8;
//     break;
//   case 5:
//     indexVelodyneROS = 10;
//     break;
//   case 6:
//     indexVelodyneROS = 12;
//     break;
//   case 7:
//     indexVelodyneROS = 14;
//     break;
//   case 8:
//     indexVelodyneROS = 1;
//     break;
//   case 9:
//     indexVelodyneROS = 3;
//     break;
//   case 10:
//     indexVelodyneROS = 5;
//     break;
//   case 11:
//     indexVelodyneROS = 7;
//     break;
//   case 12:
//     indexVelodyneROS = 9;
//     break;
//   case 13:
//     indexVelodyneROS = 11;
//     break;
//   case 14:
//     indexVelodyneROS = 13;
//     break;
//   case 15:
//     indexVelodyneROS = 15;
//     break;
//   }
//   return indexVelodyneROS;
// }

// VLP-Sensormodel (new version)
unsigned int SensorVelodyne3D::lookupIndex(int indexSensormodel)
{
  unsigned int indexVelodyneROS = 0;
  switch(indexSensormodel)
  {
  case 0:
    indexVelodyneROS = 0;
    break;
  case 1:
    indexVelodyneROS = 8;
    break;
  case 2:
    indexVelodyneROS = 1;
    break;
  case 3:
    indexVelodyneROS = 6;
    break;
  case 4:
    indexVelodyneROS = 2;
    break;
  case 5:
    indexVelodyneROS = 10;
    break;
  case 6:
    indexVelodyneROS = 3;
    break;
  case 7:
    indexVelodyneROS = 11;
    break;
  case 8:
    indexVelodyneROS = 4;
    break;
  case 9:
    indexVelodyneROS = 12;
    break;
  case 10:
    indexVelodyneROS = 5;
    break;
  case 11:
    indexVelodyneROS = 13;
    break;
  case 12:
    indexVelodyneROS = 6;
    break;
  case 13:
    indexVelodyneROS = 14;
    break;
  case 14:
    indexVelodyneROS = 7;
    break;
  case 15:
    indexVelodyneROS = 15;
    break;
  }
  return indexVelodyneROS;
}

// M sind die Koordinaten des TSD SPACES! von allen VOXELN die Mittelpunkte!
void SensorVelodyne3D::backProject(obvious::Matrix* M, int* indices, obvious::Matrix* T)
{
  obvious::Matrix PoseInv = getTransformation();
  PoseInv.invert();
  if(T)
    PoseInv *= *T;

  // multiply PoseInv with M where poseInv is not transposed but M is transposed
  // (true)
  obvious::Matrix coords3D = obvious::Matrix::multiply(PoseInv, *M, false, true);

  for(unsigned int i = 0; i < M->getRows(); i++)
  {
    double x = coords3D(0, i);
    double y = coords3D(1, i);
    double z = coords3D(2, i);

    double r                = sqrt(x * x + y * y + z * z);
    double inclinationAngle = acos(y / r);

    // ich glaub die verzerrung liegt am atan - pls check JA ES MUSS DARAN LIEGEN PLS HALP ME!!!!!!!!!!!
    double azimuthAngle = atan2(z, x);
    if(azimuthAngle < 0.0)
      azimuthAngle += 2 * M_PI;

    unsigned int azimIndex  = 0;
    unsigned int inclIndex  = 0;
    unsigned int inclMapped = 0;
    unsigned int indexCheck = 0;

    if((inclinationAngle < deg2rad(75.0)) || (inclinationAngle > deg2rad(105.0))) // ACHTUNG FOR GENERIC MODEL: HIER MIT INCLSPAN RECHNEN
    {
      indices[i] = -1;
      continue;
    }
    else
    {
      // std::cout << "from valid non -1 indices from backProject: inclinationAngle = " << rad2deg(inclinationAngle)
      //           << " , azimuthAngle = " << rad2deg(azimuthAngle) << std::endl;
      // calculate azimuth = col && inclination = row & returnRayIndex
      azimIndex  = round(azimuthAngle / _azimRes);
      inclIndex  = round(abs(((inclinationAngle - M_PI / 2) - _inclSpan + abs(_inclMin))) / _inclRes);
      inclMapped = lookupIndex(inclIndex);

      // von oben: _indexMap[row][column] = row * (_width) + column
      indexCheck = inclMapped * _width + azimIndex;
      // std::cout << "indexCheck = " << indexCheck << std::endl;
      // push current value of indexMap[row][column] into int* indices which backProject() will return to push()
      indices[i] = _indexMap[inclMapped][azimIndex];

      std::cout << "inclIndex = " << inclIndex << ", inclMapped = " << inclMapped << ", azimIndex = " << azimIndex
                << " -- _indexMap[inclMapped][azimIndex] = " << _indexMap[inclMapped][azimIndex] << std::endl;
    }
  }
}

} // namespace obvious
**/

// another save before switch rows and cols
/**
namespace obvious
{
SensorVelodyne3D::SensorVelodyne3D(unsigned int raysIncl, double inclMin, double inclRes, double azimRes, double maxRange, double minRange,
                                   double lowReflectivityRange)
    : Sensor(3, maxRange, minRange, lowReflectivityRange)
{
  _azimRes  = azimRes;                    // 0.2° in RAD
  _inclRes  = inclRes;                    // 2° in RAD
  _raysIncl = raysIncl;                   // 16
  _inclMin  = inclMin;                    //-15° in RAD
  _inclSpan = _inclRes * (_raysIncl - 1); // CHANGE THIS IN GENERIC MODEL !!!!
  _azimMin  = 0.0;

  unsigned int raysAzim = round(static_cast<unsigned>(2 * M_PI / azimRes));
  std::cout << "raysAzim = " << raysAzim << std::endl;

  _width  = raysAzim + 1; // weil 0° und 360°
  _height = raysIncl;     // VORSICHT das ist hier dann 16, würde nicht auch 15 reichen?
  _size   = _width * _height;

  _data = new double[_size];
  _mask = new bool[_size];
  for(unsigned int i = 0; i < _size; i++)
    _mask[i] = true;

  obvious::System<int>::allocate(_width, _height, _indexMap);

  // set IndexMap
  unsigned int column = 0;
  for(unsigned int row = 0; row < _width; row++) // AZIM 0 - 1800 : passt weil 0° und 360° gell?
  {
    for(column = 0; column < _height; column++) // INCL 0 - 15 : passt weil height = 16 rays --> später aufpassen in generic model
    {
      _indexMap[row][column] = row * (_height) + column;
    }
    column = 0; // iterate over 16 vertical rays for each azimuth ray
  }

  _rays             = new obvious::Matrix(3, _size);
  obvious::Matrix R = obvious::Matrix(*_T, 0, 0, 3, 3);

  unsigned int n = 0;
  for(unsigned int i = 0; i < _width; i++) // AZIM 0 - 1800 : passt weil 0° und 360° gell?
  {
    double currentAzim = _azimMin + i * _azimRes;
    std::cout << "currentAzim = " << rad2deg(currentAzim) << std::endl;

    for(unsigned int j = 0; j < _height; j++, n++) // INCL 0 - 15 : passt weil 16 rays
    {
      double currentIncl = _inclMin + j * _inclRes;
      std::cout << "currentIncl = " << rad2deg(currentIncl) << " @ n = " << n << " !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;

      obvious::Matrix calcRay(3, 1);

      // ich muss den inclination angle korrigieren sonst geht dit nit :(((((((((((((((())))))))))))))))
      double inclCorr = 0.0;

      if(currentIncl <= 0) // DAS GLEICH HAB ICH ERGÄNZT für wenn =0
      {
        inclCorr = deg2rad(90.0) + (-1.0 * currentIncl);
      }
      else
      {
        inclCorr = deg2rad(90.0) - currentIncl;
      }

      calcRay(0, 0) = sin(inclCorr) * cos(currentAzim);
      calcRay(1, 0) = cos(inclCorr);
      calcRay(2, 0) = sin(inclCorr) * sin(currentAzim);

      const double length    = sqrt(calcRay(0, 0) * calcRay(0, 0) + calcRay(1, 0) * calcRay(1, 0) + calcRay(2, 0) * calcRay(2, 0));
      const double lengthInv = 1.0 / length;

      calcRay = R * calcRay;

      (*_rays)(0, n) = calcRay(0, 0) * lengthInv;
      (*_rays)(1, n) = calcRay(1, 0) * lengthInv;
      (*_rays)(2, n) = calcRay(2, 0) * lengthInv;
    }
  }

  _raysLocal  = new obvious::Matrix(3, _size);
  *_raysLocal = *_rays;
}

SensorVelodyne3D::~SensorVelodyne3D()
{
  delete _rays;
  delete _raysLocal;
  delete[] _data;
  delete[] _mask;
  System<int>::deallocate(_indexMap);
}

// todo - adapt this for E32 and all other sensors
// OLD VERSION
// unsigned int SensorVelodyne3D::lookupIndex(int indexSensormodel)
// {
//   unsigned int indexVelodyneROS = 0;
//   switch(indexSensormodel)
//   {
//   case 0:
//     indexVelodyneROS = 0;
//     break;
//   case 1:
//     indexVelodyneROS = 2;
//     break;
//   case 2:
//     indexVelodyneROS = 4;
//     break;
//   case 3:
//     indexVelodyneROS = 6;
//     break;
//   case 4:
//     indexVelodyneROS = 8;
//     break;
//   case 5:
//     indexVelodyneROS = 10;
//     break;
//   case 6:
//     indexVelodyneROS = 12;
//     break;
//   case 7:
//     indexVelodyneROS = 14;
//     break;
//   case 8:
//     indexVelodyneROS = 1;
//     break;
//   case 9:
//     indexVelodyneROS = 3;
//     break;
//   case 10:
//     indexVelodyneROS = 5;
//     break;
//   case 11:
//     indexVelodyneROS = 7;
//     break;
//   case 12:
//     indexVelodyneROS = 9;
//     break;
//   case 13:
//     indexVelodyneROS = 11;
//     break;
//   case 14:
//     indexVelodyneROS = 13;
//     break;
//   case 15:
//     indexVelodyneROS = 15;
//     break;
//   }
//   return indexVelodyneROS;
// }

// VLP-Sensormodel (new version)
unsigned int SensorVelodyne3D::lookupIndex(int indexSensormodel)
{
  unsigned int indexVelodyneROS = 0;
  switch(indexSensormodel)
  {
  case 0:
    indexVelodyneROS = 0;
    break;
  case 1:
    indexVelodyneROS = 8;
    break;
  case 2:
    indexVelodyneROS = 1;
    break;
  case 3:
    indexVelodyneROS = 6;
    break;
  case 4:
    indexVelodyneROS = 2;
    break;
  case 5:
    indexVelodyneROS = 10;
    break;
  case 6:
    indexVelodyneROS = 3;
    break;
  case 7:
    indexVelodyneROS = 11;
    break;
  case 8:
    indexVelodyneROS = 4;
    break;
  case 9:
    indexVelodyneROS = 12;
    break;
  case 10:
    indexVelodyneROS = 5;
    break;
  case 11:
    indexVelodyneROS = 13;
    break;
  case 12:
    indexVelodyneROS = 6;
    break;
  case 13:
    indexVelodyneROS = 14;
    break;
  case 14:
    indexVelodyneROS = 7;
    break;
  case 15:
    indexVelodyneROS = 15;
    break;
  }
  return indexVelodyneROS;
}

// M sind die Koordinaten des TSD SPACES! von allen VOXELN die Mittelpunkte!
void SensorVelodyne3D::backProject(obvious::Matrix* M, int* indices, obvious::Matrix* T)
{
  obvious::Matrix PoseInv = getTransformation();
  PoseInv.invert();
  if(T)
    PoseInv *= *T;

  // multiply PoseInv with M where poseInv is not transposed but M is transposed
  // (true)
  obvious::Matrix coords3D = obvious::Matrix::multiply(PoseInv, *M, false, true);

  for(unsigned int i = 0; i < M->getRows(); i++)
  {
    double x = coords3D(0, i);
    double y = coords3D(1, i);
    double z = coords3D(2, i);

    double r                = sqrt(x * x + y * y + z * z);
    double inclinationAngle = acos(y / r);

    // ich glaub die verzerrung liegt am atan - pls check JA ES MUSS DARAN LIEGEN PLS HALP ME!!!!!!!!!!!
    double azimuthAngle = atan2(z, x);
    if(azimuthAngle < 0.0)
      azimuthAngle += 2 * M_PI;

    unsigned int azimIndex  = 0;
    unsigned int inclIndex  = 0;
    unsigned int inclMapped = 0;
    unsigned int indexCheck = 0;

    // if((inclinationAngle < deg2rad(75.0)) || (inclinationAngle > deg2rad(105.0))) // ACHTUNG FOR GENERIC MODEL: HIER MIT INCLSPAN RECHNEN
    if((inclinationAngle < deg2rad(35.0)) || (inclinationAngle > deg2rad(155.0))) // TEST: größere range um zu sehen ob ich mehr layer krieg aus der
                                                                                  // schachtel - ja klappt

    {
      indices[i] = -1;
      continue;
    }
    else
    {
      // calculate azimuth = row && inclination = column & returnRayIndex
      azimIndex  = round(azimuthAngle / _azimRes);
      inclIndex  = abs(((inclinationAngle - M_PI / 2) - _inclSpan + abs(_inclMin))) / _inclRes;
      inclMapped = lookupIndex(inclIndex);

      // prode: index ausrechnen = raysIncl * row + col
      indexCheck = _height * azimIndex + inclIndex;

      // push current value of indexMap[row][column] into int* indices which backProject() will return to push()
      indices[i] = _indexMap[azimIndex][inclMapped];
    }
  }
}

} // namespace obvious
**/

// OLD SAVE

namespace obvious
{
// my constructor
SensorVelodyne3D::SensorVelodyne3D(unsigned int raysIncl, double inclMin, double inclRes, double azimRes, double maxRange, double minRange,
                                   double lowReflectivityRange)
    : Sensor(3, maxRange, minRange, lowReflectivityRange)
{
  _azimRes                  = azimRes;
  _inclRes                  = inclRes;
  unsigned int raysAzim     = 0;
  double       azimAngle    = 0.0;
  const double resetInclMin = inclMin; // to reset variable inclMin each time
                                       // after exiting inner for-loop (=-15°
                                       // here for VLP16)

  raysAzim = round(static_cast<unsigned>(2 * M_PI / azimRes));

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /// PLEASE LET SOMEONE CHECK IF THE AZIM+1 THING IS CORRECT! DO I REALLY HAVE
  /// 361 values FOR AZIMUT? yes right?
  /// +1 bei allocate _indexMap und bei _width
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  // inherited from class sensor
  // Size of first dimension, i.e., # of samples of a 2D scan or width of image
  _width = raysAzim + 1;
  // Size of second dimension, i.e., 1 for a 2D scan or height of image sensor
  _height = raysIncl;
  // Number of measurement samples, i.e. _width x _height
  _size = _width * _height;

  _data = new double[_size];
  _mask = new bool[_size];
  for(unsigned int i = 0; i < _size; i++)
    _mask[i] = true;
  // for (unsigned int i = 0; i <= _size; i++)
  //   _mask[i] = true;                                                     // MACHT KEINEN UNTERSCHIED?

  obvious::System<int>::allocate(_width, _height, _indexMap);

  setIndexMap(_width, _height);

  _rays = new obvious::Matrix(3, _size);

  obvious::Matrix R = obvious::Matrix(*_T, 0, 0, 3, 3);

  unsigned int n = 0; // counts all rays of inner loop and outer loop to store
                      // in matrix _rays
  for(unsigned int i = 0; i < _width; i++)
  {
    for(unsigned int j = 0; j < _height; j++, n++)
    {
      obvious::Matrix calcRay(3, 1);
      // to follow definition of theta (inclination angle) in spherical
      // coordinate system
      double       thetaSphere = 0.0;
      const double piHalf      = deg2rad(90.0);

      if(inclMin < 0)
      {
        thetaSphere = piHalf + (inclMin) * (-1);
      }
      else
      {
        thetaSphere = piHalf - inclMin;
      }

      // x y z
      calcRay(0, 0) = sin(thetaSphere) * cos(azimAngle);
      calcRay(1, 0) = sin(thetaSphere) * sin(azimAngle);
      calcRay(2, 0) = cos(thetaSphere);

      // // May's CS
      // calcRay(0, 0) = sin(thetaSphere) * cos(azimAngle);
      // calcRay(1, 0) = cos(thetaSphere);
      // calcRay(2, 0) = sin(thetaSphere) * sin(azimAngle);
      // normalize rays
      const double length    = sqrt(calcRay(0, 0) * calcRay(0, 0) + calcRay(1, 0) * calcRay(1, 0) + calcRay(2, 0) * calcRay(2, 0));
      const double lengthInv = 1.0 / length;
      calcRay                = R * calcRay;

      // store end points of current ray in matrix* _rays inherited from class
      // Sensor
      (*_rays)(0, n) = calcRay(0, 0) * lengthInv;
      (*_rays)(1, n) = calcRay(1, 0) * lengthInv;
      (*_rays)(2, n) = calcRay(2, 0) * lengthInv;

      inclMin += inclRes;
    }
    inclMin = resetInclMin; // reset inclination angle to minimum
    azimAngle += azimRes;
  }

  _raysLocal  = new obvious::Matrix(3, _size);
  *_raysLocal = *_rays;
}

SensorVelodyne3D::~SensorVelodyne3D()
{
  delete _rays;
  delete _raysLocal;
  delete[] _data;
  delete[] _mask;
  System<int>::deallocate(_indexMap);
}

// return by reference - inclAngle u azimuth in DEGREEs -- changed to RAD
void SensorVelodyne3D::returnAngles(double xCoord, double yCoord, double zCoord, double* inclAngle, double* azimAngle)
{
  // Inclination
  double theta = 0.0; // careful love, this is the angle between z-axis and x-y-plane as
                      // defined in polar coordinates --> no distinction of
                      // cases for acos necessary, bec. only values from 75° -
                      // 105° for VLP16
  double length    = sqrt(xCoord * xCoord + yCoord * yCoord + zCoord * zCoord);
  double lengthInv = 1.0 / length;

  theta = acos(zCoord * lengthInv);
  // May's CS
  // theta = acos(yCoord * lengthInv);

  if(theta > deg2rad(90.0)) // translate theta into inclination angle "aperture angle"
                            // from -15° to +15°
  {
    *inclAngle = -(theta - deg2rad(90.0)); // -15° -> 0°
  }
  else
  {
    *inclAngle = deg2rad(90.0) - theta; // 0° -> +15°
  }

  // Azimuth
  *azimAngle = atan2(yCoord, xCoord);
  // May's CS - substracted M_PI for a test like chefe
  // *azimAngle = atan2(zCoord, xCoord) - M_PI;

  if(*azimAngle < 0)
  {
    *azimAngle += 2.0 * M_PI; // express angles positively in 3rd and 4th quadrant
  }
}

void SensorVelodyne3D::returnRayIndex(double azimAngle, double inclAngle, unsigned int* azimIndex, unsigned int* inclIndex)
{
  *azimIndex = round(azimAngle / _azimRes);

  // assignment will always be the same for VLP16 - inclination resolution is
  // fixed 2° and nbr of rays is 16
  double mapInclination = inclAngle + deg2rad(15.0); // map inclination angles (-15° -> +15°) up to positive
  // range 0° - 30° --> TO DO change so this also works for E32
  *inclIndex = round(mapInclination / _inclRes);
}

void SensorVelodyne3D::setIndexMap(unsigned int width, unsigned int height)
{
  unsigned int column = 0;
  for(unsigned int row = 0; row < width; row++)
  {
    for(column = 0; column < height; column++)
    {
      _indexMap[row][column] = row * (height) + column;
    }
    column = 0; // iterate over 16 vertical rays for each azimuth ray
  }
}

// todo - adapt this for E32
unsigned int SensorVelodyne3D::lookupIndex(int indexSensormodel)
{
  unsigned int indexVelodyneROS = 0;
  switch(indexSensormodel)
  {
  case 0:
    indexVelodyneROS = 0;
    break;
  case 1:
    indexVelodyneROS = 2;
    break;
  case 2:
    indexVelodyneROS = 4;
    break;
  case 3:
    indexVelodyneROS = 6;
    break;
  case 4:
    indexVelodyneROS = 8;
    break;
  case 5:
    indexVelodyneROS = 10;
    break;
  case 6:
    indexVelodyneROS = 12;
    break;
  case 7:
    indexVelodyneROS = 14;
    break;
  case 8:
    indexVelodyneROS = 1;
    break;
  case 9:
    indexVelodyneROS = 3;
    break;
  case 10:
    indexVelodyneROS = 5;
    break;
  case 11:
    indexVelodyneROS = 7;
    break;
  case 12:
    indexVelodyneROS = 9;
    break;
  case 13:
    indexVelodyneROS = 11;
    break;
  case 14:
    indexVelodyneROS = 13;
    break;
  case 15:
    indexVelodyneROS = 15;
    break;
  }
  return indexVelodyneROS;
}

//   // M sind die Koordinaten des TSD SPACES! von allen VOXELN die Mittelpunkte!
void SensorVelodyne3D::backProject(obvious::Matrix* M, int* indices, obvious::Matrix* T)
{
  obvious::Matrix PoseInv = getTransformation();
  PoseInv.invert();
  if(T)
    PoseInv *= *T;

  // multiply PoseInv with M where poseInv is not transposed but M is transposed
  // (true)
  obvious::Matrix coords3D = obvious::Matrix::multiply(PoseInv, *M, false, true);

  double       inclAngle    = 0.0;
  double       azimAngle    = 0.0;
  unsigned int row          = 0;
  unsigned int column       = 0;
  unsigned int columnMapped = 0;
  unsigned int idxCheck     = 0;

  for(unsigned int i = 0; i < M->getRows(); i++)
  {
    // x y z
    double x = coords3D(0, i);
    double y = coords3D(1, i);
    double z = coords3D(2, i);

    // returnAngles(x, y, z, &inclAngle, &azimAngle);

    // x z y
    // double x = coords3D(0, i);
    // double z = coords3D(1, i);
    // double y = coords3D(2, i);

    returnAngles(x, y, z, &inclAngle, &azimAngle);

    // leave current loop if incl angle out of measurement area -15° --> +15.0°
    if((inclAngle < deg2rad(-15.0)) || (inclAngle > deg2rad(15.0)))
    {
      indices[i] = -1;
      continue;
    }
    else
    {
      // 1: calculate incoming azimuth = ROW index of indexMap
      // 2: calculate incoming inclination = COLUMN of indexMap
      returnRayIndex(azimAngle, inclAngle, &row, &column);

      // ROW CORRECTED weil row= azimindex max zb 359,9 / 0.2 = 1799 == 1800 -->
      // index 1799 weil 0 anfängt? ist das richtig?
      // 0 / 0.2 = 0
      // 0.2 / 0.2 = 1
      // muss nur beim letzten wert passieren gell? was versteh ich hier grad
      // nicht
      // warum passiert das dann bei col nicht?
      // ich brauch einen index mehr gell? in allocate

      // map column from sensor model to Velodyne firing sequence (order of
      // vertical rays differs between sensormodel and velodyne ros input)
      columnMapped = lookupIndex(column);

      // probe: index ausrechnen
      idxCheck = columnMapped + 16 * row;

      // push current value of current indexMap[row][column] into int* indices
      // (returned by backProject to push())
      indices[i] = _indexMap[row][columnMapped];
    }
  }
}
} // namespace obvious
  //** / // OLD SAVE