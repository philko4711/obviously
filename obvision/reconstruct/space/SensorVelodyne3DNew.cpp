#include "SensorVelodyne3DNew.h"
#include "obcore/base/System.h"
#include "obcore/math/mathbase.h"
#include <limits>

namespace obvious
{
SensorVelodyne3DNew::SensorVelodyne3DNew(unsigned int raysIncl, double inclMin, double inclMax, double inclRes, double azimMin, double azimMax,
                                         double azimRes, double maxRange, double minRange, double lowReflectivityRange)
    : Sensor(3, maxRange, minRange, lowReflectivityRange)
{
  _inclRes                  = inclRes;
  _inclMin                  = inclMin;
  _inclSpan                 = static_cast<double>(_inclRes * (raysIncl - 1)); // todo CHECK FOR OTHER MODELS
  _inclNegSpan              = abs(inclMin);
  _azimRes                  = azimRes;
  _azimMin                  = azimMin;
  const double resetInclMin = inclMin; // to reset variable inclMin each time

  int raysAzim = round(static_cast<int>(2 * M_PI / _azimRes));

  // from Sensor
  _width  = static_cast<unsigned>(raysAzim + 1); // check +1 here
  _height = static_cast<unsigned>(raysIncl);
  _size   = _width * _height;

  _data = new double[_size];
  _mask = new bool[_size];
  for(unsigned int i = 0; i < _size; i++)
    _mask[i] = true;

  // set index map
  obvious::System<int>::allocate(_width, _height, _indexMap);
  for(unsigned int row = 0; row < _width; row++) // AZIM
  {
    for(unsigned int column = 0; column < _height; column++) // INCL
    {
      _indexMap[row][column] = row * _height + column;
    }
  }

  _rays    = new Matrix(3, _size);
  Matrix R = Matrix(*_T, 0, 0, 3, 3);

  unsigned int n           = 0;
  double       currentAzim = 0.0;
  // double       currentIncl = _inclMin;
  // double currentIncl = 0.0;

  for(unsigned int i = 0; i < _width; i++)
  {
    currentAzim = _azimMin + i * _azimRes;

    for(unsigned int j = 0; j < _height; j++, n++)
    {
      // currentIncl = _inclMin + j * _inclRes;

      Matrix       calcRay(3, 1);
      double       thetaSphere = 0.0;
      const double piHalf      = deg2rad(90.0);
      // hier muss ich die inclination Winkel in thetaSphzere jetzt umrechnen, weil ich meinen Fächer ja von -15° bis +15° aufbaue und die Kugelkoord. theta
      // von der z-Achse runter auf die x-y-Ebene definieren
      // if(currentIncl <= 0)
      if(inclMin <= 0)

      {
        // thetaSphere = piHalf + currentIncl * (-1);
        thetaSphere = piHalf + inclMin * (-1);
      }
      else
      {
        // thetaSphere = piHalf - currentIncl;
        thetaSphere = piHalf - inclMin;
      }

      calcRay(0, 0) = sin(thetaSphere) * cos(currentAzim);
      calcRay(1, 0) = sin(thetaSphere) * sin(currentAzim);
      calcRay(2, 0) = cos(thetaSphere);

      const double length    = sqrt(calcRay(0, 0) * calcRay(0, 0) + calcRay(1, 0) * calcRay(1, 0) + calcRay(2, 0) * calcRay(2, 0));
      const double lengthInv = 1.0 / length;
      calcRay                = R * calcRay;

      (*_rays)(0, n) = calcRay(0, 0) * lengthInv;
      (*_rays)(1, n) = calcRay(1, 0) * lengthInv;
      (*_rays)(2, n) = calcRay(2, 0) * lengthInv;

      inclMin += _inclRes;
    }
    inclMin = resetInclMin;
  }

  _raysLocal  = new Matrix(3, _size);
  *_raysLocal = *_rays;
}

SensorVelodyne3DNew::~SensorVelodyne3DNew()
{
  delete[] _data;
  delete[] _mask;
  System<int>::deallocate(_indexMap);
  delete _rays;
  delete _raysLocal;
}

int SensorVelodyne3DNew::lookupIndex(int inclIndex)
{
  int indexVelodyneROS = 0;
  switch(inclIndex)
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

// M sind die Koordinaten des TSD SPACES! von allen VOXELN die Mittelpunkte!
void SensorVelodyne3DNew::backProject(obvious::Matrix* M, int* indices, obvious::Matrix* T)
{
  Matrix PoseInv = getTransformation();
  PoseInv.invert();
  if(T)
    PoseInv *= *T;

  Matrix coords3D = Matrix::multiply(PoseInv, *M, false, true);

  double inclAngle       = 0.0;
  int    lookupInclIndex = 0;

  for(unsigned int i = 0; i < M->getRows(); i++)
  {
    double length    = sqrt(coords3D(0, i) * coords3D(0, i) + coords3D(1, i) * coords3D(1, i) + coords3D(2, i) * coords3D(2, i));
    double lengthInv = 1.0 / length; // hilft das f. Perform.?

    double theta = acos(coords3D(2, i) * lengthInv);

    // theta in inclAngle umwandeln vor Indexberechnung
    if(theta >= deg2rad(90.0))
    {
      inclAngle = -(theta - deg2rad(90.0)); // for all angles from -15° to 0°
    }
    else
    {
      inclAngle = deg2rad(90.0) - theta; // for all angles bigger than 0° to 15°
    }

    // std::cout << "inclAngle = " << rad2deg(inclAngle) << std::endl;

    double azimAngle = atan2(coords3D(1, i), coords3D(0, i));
    if(azimAngle < 0)
    {
      azimAngle += 2 * M_PI; // express angles positively in 3rd and 4th quadrant bec atan2 expresses them negatively
    }

    // leave current loop if inclAngle out of vertical aperture/measurement area between -15° and +15°; set current index = -1 (invalid)
    if((inclAngle < deg2rad(-15.0)) || (inclAngle > deg2rad(15.0)))
    {
      indices[i] = -1;
      continue;
    }
    else
    {
      // shift inclAngle to positive 1st quadrant before index calculations so its easier to calculate indices with resolution vals
      double inclShifted = inclAngle + _inclNegSpan; // shifts all inclAngles upwards with +15°, so span from -15° to 15° is now shifted from 0° to 30°
      // std::cout << "inclShifted = " << rad2deg(inclShifted) << std::endl;
      int azimIndex = round(azimAngle / _azimRes);
      // std::cout << "azimIndex = " << azimIndex << std::endl;

      // ohne LOOKUP
      int inclIndex = round(inclShifted / _inclRes);
      // std::cout << "inclIndex = " << inclIndex << std::endl;
      int idxCheck = azimIndex * _height + inclIndex; // height = raysIncl
      // std::cout << "idxCheck = " << idxCheck << std::endl;
      indices[i] = _indexMap[azimIndex][inclIndex];
      // std::cout << "_indexMap[azimIndex][inclIndex] = " << _indexMap[azimIndex][inclIndex] << std::endl;
      // std::cout << "indices[i] = " << indices[i] << std::endl;

      // // mit LOOKUP
      // int inclIndex = round(static_cast<int>(inclShifted / _inclRes));
      // std::cout << "inclIndex = " << inclIndex << std::endl;

      // lookupInclIndex = lookupIndex(inclIndex);
      // std::cout << "lookupInclIndex = " << lookupInclIndex << std::endl;

      // indices[i] = _indexMap[azimIndex][lookupInclIndex];
      // std::cout << "_indexMap[azimIndex][lookupInclIndex] = " << _indexMap[azimIndex][lookupInclIndex] << std::endl;
    }
  }
}

} // namespace obvious