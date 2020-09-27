#include "SensorPolar.h"
#include "obcore/base/System.h"
#include "obcore/math/mathbase.h"
#include <limits>

namespace obvious
{

SensorPolar::SensorPolar(unsigned int raysIncl, double inclMin, double inclMax, double inclRes, double azimMin, double azimMax, double azimRes,
                         double maxRange, double minRange, double lowReflectivityRange)
    : Sensor(3, maxRange, minRange, lowReflectivityRange)
{
  _inclRes                  = inclRes;
  _inclMin                  = inclMin;
  _inclMax                  = inclMax;
  _inclNegSpan              = abs(inclMin);
  _azimRes                  = azimRes;
  _azimMin                  = azimMin;
  const double resetInclMin = inclMin;

  int raysAzim = static_cast<int>(round(2 * M_PI / _azimRes));

  // inherited from Sensor
  _width  = static_cast<unsigned>(raysAzim);
  _height = static_cast<unsigned>(raysIncl);
  _size   = _width * _height;
  _data   = new double[_size];
  _mask   = new bool[_size];
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

  for(unsigned int i = 0; i < _width; i++)
  {
    currentAzim = _azimMin + i * _azimRes;

    for(unsigned int j = 0; j < _height; j++, n++)
    {
      Matrix       calcRay(3, 1);
      double       thetaSphere = 0.0;
      const double piHalf      = deg2rad(90.0);
      // hier muss ich die inclination Winkel in thetaSphzere jetzt umrechnen, weil ich meinen Fächer ja von _inclMin bis _inclMax aufbaue und die
      // Kugelkoord. theta von der z-Achse runter auf die x-y-Ebene definieren
      if(inclMin < 0.0)

      {
        thetaSphere = piHalf + inclMin * (-1.0);
      }
      else
      {
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

SensorPolar::~SensorPolar()
{
  delete[] _data;
  delete[] _mask;
  System<int>::deallocate(_indexMap);
  delete _rays;
  delete _raysLocal;
}

void SensorPolar::backProject(obvious::Matrix* M, int* indices, obvious::Matrix* T)
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
    double lengthInv = 1.0 / length;

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

    double azimAngle = atan2(coords3D(1, i), coords3D(0, i)) + M_PI; // +PI because atan2 defines angles in 3rd and 4th quadrant clockwise = negatively. By
                                                                     // adding +pi, all results are positive -> index calculation works

    // leave current loop if inclAngle out of vertical aperture/measurement area between _inclMin and _inclMax; set current index = -1 (invalid)
    if((inclAngle < _inclMin) || (inclAngle > _inclMax))
    {
      indices[i] = -1;
      continue;
    }
    else
    {
      // shift inclAngle to positive 1st quadrant before index calculations so its easier to calculate indices with resolution vals
      double inclShifted = inclAngle + _inclNegSpan;
      int    azimIndex   = static_cast<int>(floor(azimAngle / _azimRes));

      // ohne LOOKUPINDEX --> testdata
      int inclIndex = static_cast<int>(floor(inclShifted / _inclRes));
      int idxCheck  = azimIndex * static_cast<int>(_height) + inclIndex;
      indices[i]    = _indexMap[azimIndex][inclIndex];

      /** mit LOOKUP --> realdata
      // int inclIndex = static_cast<int>(floor(inclShifted / _inclRes));
      // lookupInclIndex = lookupIndex(inclIndex);
      // indices[i] = _indexMap[azimIndex][lookupInclIndex];
      **/
    }
  }
}

} // namespace obvious