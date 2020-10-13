#include "SensorPolar.h"
#include "obcore/base/System.h"
#include "obcore/math/mathbase.h"
#include <limits>

namespace obvious
{

SensorPolar::SensorPolar(unsigned int raysIncl, double inclMin, double inclMax, double inclRes, double azimMin, double azimMax, double azimRes,
                         std::vector<int> firingSeq, double maxRange, double minRange, double lowReflectivityRange)
    : Sensor(3, maxRange, minRange, lowReflectivityRange)
{
  _inclRes                  = inclRes;
  _inclMin                  = inclMin;
  _inclMax                  = inclMax;
  _inclNegSpan              = abs(inclMin);
  _azimRes                  = azimRes;
  _azimMin                  = azimMin;
  _azimMax                  = azimMax;
  _azimNegSpan = abs(azimMin);
  const double resetInclMin = inclMin;
  _firingSeq                = firingSeq;

  double azimRange = abs(azimMin - azimMax);
  int    raysAzim  = static_cast<int>(round(azimRange / _azimRes));
  // int raysAzim = static_cast<int>(round(2 * M_PI / _azimRes));

  // inherited from Sensor
  _width  = static_cast<unsigned>(raysAzim);
  _height = static_cast<unsigned>(raysIncl);
  _size   = _width * _height;

  std::cout << __PRETTY_FUNCTION__ << "Sensor width = " << _width << std::endl;
  std::cout << __PRETTY_FUNCTION__ << "Sensor height = " << _height << std::endl;
  std::cout << __PRETTY_FUNCTION__ << "Sensor size = " << _size << std::endl;

  _data = new double[_size];
  _mask = new bool[_size];
  for(unsigned int i = 0; i < _size; i++)
    _mask[i] = true;

  // set index map
  obvious::System<int>::allocate(_width, _height, _indexMap);

  //indexmap wenn kein Vollwinkel (tilt3d zb)
  if(azimRange < (2.0*M_PI))    //krieg ich hier probleme wg. rundungen?
  {
    std::cout << __PRETTY_FUNCTION__ << "kein Vollwinkel :-)))))))))))))" << std::endl;
    //kein Vollwinkel, daher keine indexmap von 0° - 360°. Für Indexberechnugn werden in backProject neg. Azims nach oben geschoben, also Start bei 0° ist ok, aber +PI machen wie beim atan2 später
    unsigned int startAzimIdx = static_cast<unsigned>(floor(M_PI / _azimRes));  //in bsp: 180° / 1° = 180
    unsigned int endAzimIdx = static_cast<unsigned>(floor(azimRange + M_PI));   //in bsp: 270° + 180° = 450°

    for(startAzimIdx; startAzimIdx < endAzimIdx; startAzimIdx++)  //Azimvals von 180°-450°
    {
      for(unsigned int column = 0; column < _height; column++)
      {
        _indexMap[startAzimIdx][column] = startAzimIdx * _height + column;
      }
    }
  }

  else    //indexMap Vollwinkel
  {
    std::cout << __PRETTY_FUNCTION__ << "EIN voller Vollwinkel :-)))))))))))))" << std::endl;
  for(unsigned int row = 0; row < _width; row++) // AZIM
  {
    for(unsigned int column = 0; column < _height; column++) // INCL
    {
      _indexMap[row][column] = row * _height + column;
    }
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
      // hier muss ich die inclination Winkel in thetaSphzere jetzt umrechnen,
      // weil ich meinen Fächer ja von _inclMin bis _inclMax aufbaue und die
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

int SensorPolar::lookupIndex(int inclIndex)
{
  int inclMatch = 0;
  if(_firingSeq.size() == 32)
  {
    switch(inclIndex)
    {
    case 0:
      inclMatch = _firingSeq[0];
      break;
    case 1:
      inclMatch = _firingSeq[1];
      break;
    case 2:
      inclMatch = _firingSeq[2];
      break;
    case 3:
      inclMatch = _firingSeq[3];
      break;
    case 4:
      inclMatch = _firingSeq[4];
      break;
    case 5:
      inclMatch = _firingSeq[5];
      break;
    case 6:
      inclMatch = _firingSeq[6];
      break;
    case 7:
      inclMatch = _firingSeq[7];
      break;
    case 8:
      inclMatch = _firingSeq[8];
      break;
    case 9:
      inclMatch = _firingSeq[9];
      break;
    case 10:
      inclMatch = _firingSeq[10];
    case 11:
      inclMatch = _firingSeq[11];
      break;
    case 12:
      inclMatch = _firingSeq[12];
      break;
    case 13:
      inclMatch = _firingSeq[13];
      break;
    case 14:
      inclMatch = _firingSeq[14];
      break;
    case 15:
      inclMatch = _firingSeq[15];
      break;
    case 16:
      inclMatch = _firingSeq[16];
      break;
    case 17:
      inclMatch = _firingSeq[17];
      break;
    case 18:
      inclMatch = _firingSeq[18];
      break;
    case 19:
      inclMatch = _firingSeq[19];
      break;
    case 20:
      inclMatch = _firingSeq[20];
      break;
    case 21:
      inclMatch = _firingSeq[21];
      break;
    case 22:
      inclMatch = _firingSeq[22];
      break;
    case 23:
      inclMatch = _firingSeq[23];
      break;
    case 24:
      inclMatch = _firingSeq[24];
      break;
    case 25:
      inclMatch = _firingSeq[25];
      break;
    case 26:
      inclMatch = _firingSeq[26];
      break;
    case 27:
      inclMatch = _firingSeq[27];
      break;
    case 28:
      inclMatch = _firingSeq[28];
      break;
    case 29:
      inclMatch = _firingSeq[29];
      break;
    case 30:
      inclMatch = _firingSeq[30];
      break;
    case 31:
      inclMatch = _firingSeq[31];
      break;
    default:
      std::cout << __PRETTY_FUNCTION__ << " index not valid - aborting." << std::endl;
      std::abort();
    }
  }
  else if(_firingSeq.size() == 16)
  {
    switch(inclIndex)
    {
    case 0:
      inclMatch = _firingSeq[0];
      break;
    case 1:
      inclMatch = _firingSeq[1];
      break;
    case 2:
      inclMatch = _firingSeq[2];
      break;
    case 3:
      inclMatch = _firingSeq[3];
      break;
    case 4:
      inclMatch = _firingSeq[4];
      break;
    case 5:
      inclMatch = _firingSeq[5];
      break;
    case 6:
      inclMatch = _firingSeq[6];
      break;
    case 7:
      inclMatch = _firingSeq[7];
      break;
    case 8:
      inclMatch = _firingSeq[8];
      break;
    case 9:
      inclMatch = _firingSeq[9];
      break;
    case 10:
      inclMatch = _firingSeq[10];
    case 11:
      inclMatch = _firingSeq[11];
      break;
    case 12:
      inclMatch = _firingSeq[12];
      break;
    case 13:
      inclMatch = _firingSeq[13];
      break;
    case 14:
      inclMatch = _firingSeq[14];
      break;
    case 15:
      inclMatch = _firingSeq[15];
      break;
    default:
      std::cout << __PRETTY_FUNCTION__ << " index not valid - aborting." << std::endl;
      std::abort();
    }
  }
  else
  {
    std::cout << __PRETTY_FUNCTION__
              << "size of _firingSeq is not valid. Must contain either 16 rays "
                 "for VLP16 or 32 rays for HDL-32E. Wanna integrate a new "
                 "sensor? pls update this function if the firing sequence hops "
                 "around. aborting."
              << std::endl;
    std::abort();
  }

  return inclMatch;
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
      inclAngle = -(theta - deg2rad(90.0));
    }
    else
    {
      inclAngle = deg2rad(90.0) - theta;
    }

    double azimAngle = atan2(coords3D(1, i), coords3D(0, i)) + M_PI; // +PI because atan2 defines angles in 3rd and 4th
                                                                     // quadrant clockwise = negatively. By
    // adding +pi, all results are positive -> index calculation works

      // shift inclAngle to positive 1st quadrant before index calculations so
      // its easier to calculate indices with resolution vals
      double inclShifted = inclAngle + _inclNegSpan;
      double azimShifted = azimAngle + _azimNegSpan;
    
    // throw out invalid indices that are out of both inclination and azimuth field of view of sensor
    // leave current loop if inclAngle out of vertical aperture/measurement area
    // between _inclMin and _inclMax; set current index = -1 (invalid)
    if((inclAngle < _inclMin) || (inclAngle > _inclMax))
    {
      indices[i] = -1;
      continue;
    }

    //DAMIT HAU ICH AUCH GÜLTIGE WERTE RAUS
    // else if((azimAngle < (_azimMin + M_PI)) || (azimAngle > (_azimMax + M_PI))) //+PI here as well for boundaries!!
    // {
    //   indices[i] = -1;
    //   continue;
    // }

    else
    {

      // int    azimIndex   = static_cast<int>(floor(azimAngle / _azimRes)); // todo add azimuth bounds for non 360° scanners and set index -1
      int    azimIndex   = static_cast<int>(floor(azimShifted / _azimRes));
      // ohne LOOKUPINDEX --> testdata zb
      if(_firingSeq.empty())
      {
        int inclIndex = static_cast<int>(floor(inclShifted / _inclRes));
        int idxCheck  = azimIndex * static_cast<int>(_height) + inclIndex;
        indices[i]    = _indexMap[azimIndex][inclIndex];
      }
      else
      {
        // mit LOOKUP --> realdata
        int inclIndex   = static_cast<int>(floor(inclShifted / _inclRes));
        lookupInclIndex = lookupIndex(inclIndex);
        indices[i]      = _indexMap[azimIndex][lookupInclIndex];
      }
    }
  }
}

} // namespace obvious