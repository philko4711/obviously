/*
 * juicy_test.cpp
 *
 *      Author: jasmin
 */

#include "math.h"
#include "obcore/base/System.h"
#include "obcore/math/linalg/eigen/Matrix.h"
#include "obcore/math/mathbase.h"
#include "obgraphic/Obvious3D.h"
#include <iostream>

double       _azimRes = obvious::deg2rad(2.0); // eig 0.2
double       _azimMin = obvious::deg2rad(0.0);
double       _inclRes = obvious::deg2rad(2.0);
int**        _indexMap;
unsigned int _raysIncl    = 16;
double       _inclMin     = obvious::deg2rad(-15.0);    // smallest inclination angle - "lowest" ray
double       _inclSpan    = _inclRes * (_raysIncl - 1); // wirklich?
double       _inclNegSpan = abs(_inclMin);

void returnAngles(double xCoord, double yCoord, double zCoord, double* inclAngle, double* azimAngle)
{
  // INCLINATION
  double theta = 0.0; // careful - this is the angle between z-axis and x-y-plane as defined in polar coords --> not inclination angle

  double length    = sqrt(xCoord * xCoord + yCoord * yCoord + zCoord * zCoord);
  double lengthInv = 1.0 / length;
  double piHalf    = obvious::deg2rad(90.0);

  theta = acos(zCoord * lengthInv);
  std::cout << "incl theta = " << obvious::rad2deg(theta) << std::endl;

  if((obvious::rad2deg(theta) < 75.0) && (obvious::rad2deg(theta) > 105.0))
  {
    std::cout << __PRETTY_FUNCTION__ << "3D coordinates are not within field of vision of laser scanner" << std::endl;
  }
  else
  {
    if(obvious::rad2deg(theta) > 90.0)
    {
      *inclAngle = -(theta - piHalf);
    }
    else
    {
      *inclAngle = piHalf - theta;
    }
    std::cout << __PRETTY_FUNCTION__ << " inclAngle = " << obvious::rad2deg(*inclAngle) << std::endl;

    // AZIMUTH
    *azimAngle = atan2(yCoord, xCoord);
    std::cout << __PRETTY_FUNCTION__ << " before adding 2 pi-- azimAngle = " << obvious::rad2deg(*azimAngle) << std::endl;

    if(*azimAngle < 0)
    {
      *azimAngle += 2 * M_PI; // express angles positively in 3rd and 4th quadrant bec atan2 expresses them negatively
      std::cout << __PRETTY_FUNCTION__ << " after azimAngle = " << obvious::rad2deg(*azimAngle) << std::endl;
    }
  }
}

// /**
//  * returns ray index
//  * @param[in] azimAngle azimuth angle calculated in returnAngles()
//  * @param[in] inclAngle inclination angle calculated in returnAngles()
//  * @param[out] azimIndex index of azimuth ray number which is closest to 3D point to assign laser data to 3D point
//  * @param[out] inclIndex index of inclination ray number which is closest to 3D point to assign laser data to 3D point
//  * @todo make this function abstract so each velodyne sensor has to implement it since it may differ from sensor to sensor
//  */
// void returnRayIndex(double azimAngle, double inclAngle, unsigned int* azimIndex, unsigned int* inclIndex)
// {
//   // AZIMUTH
//   *azimIndex = round(azimAngle / _azimRes);
//   std::cout << __PRETTY_FUNCTION__ << " azimAngle = " << azimAngle << " , _ = " << _ << std::endl;

//   std::cout << __PRETTY_FUNCTION__ << " azimIndex = " << *azimIndex << std::endl;

//   // INCLINATION --> may differ for different sensors
//   // assignment will always be the same for VLP16 - inclination resolution is fixed 2° and nbr of rays is 16
//   double mapInclination = inclAngle + obvious::deg2rad(15.0); // map inclination angles (-15° -> +15°) up to positive range 0° - 30°
//   *inclIndex            = round(mapInclination / _inclRes);   // maybe exchange round with floor ? or ceiling? idk
//   std::cout << __PRETTY_FUNCTION__ << " inclAngle = " << inclAngle << " , _inclRes = " << _inclRes << std::endl;
//   std::cout << __PRETTY_FUNCTION__ << "das will ich weg haben: mapInclination = " << obvious::rad2deg(mapInclination);
//   std::cout << __PRETTY_FUNCTION__ << " inclIndex = " << *inclIndex << std::endl;

//   unsigned int inclIndexNew = 0;
//   double       inclSpan     = _inclRes * (_raysIncl - 1);
//   inclIndexNew              = round(abs(((inclAngle - M_PI / 2) - inclSpan + abs(__inclMin))) / __inclRes);
// }

void setIndexMap(unsigned int width, unsigned int height)
{
  unsigned int column = 0;
  // evtl mit _height u _width ersetzen
  obvious::System<int>::allocate(width, _raysIncl, _indexMap);
  for(unsigned int row = 0; row < width; row++) // r row = azimuth
  {
    for(column = 0; column <= _raysIncl; column++) // c column = incl
    {

      // _indexMap[row][column] = row*(verticalRays+1) + column;         //todo stimm tdas so mit +1? 2040 indices insgesamt 0-2039
      _indexMap[row][column] = row * height + column; // ohne +1

      // std::cout << "_indexMap = " << _indexMap[row][column] << std::endl;
    }
    // column=0;     //iterate over 16 vertical rays for each azimuth ray
  }
}

int main(void)
{
  double              bg[3]  = {1.0, 1.0, 1.0};
  obvious::Obvious3D* viewer = new obvious::Obvious3D("Juicy VLP16 Raycast Visualization Test", 1024, 768, 0, 0, bg);
  double**            coordsStart;
  double**            coordsEnd;

  double rgbGreen[3] = {0.0, 180.0, 153.0};
  double rgbRed[3]   = {255.0, 0.0, 51.0};
  double rgbBlue[3]  = {0.0, 0.0, 255.0};

  double       inclCorr     = 0.0; // für Definition des Winkels für Kugelkoordinaten
  double       angleAzimuth = 0.0;
  unsigned int raysAzim     = static_cast<unsigned>(2 * M_PI / _azimRes);

  unsigned int width  = raysAzim + 1; // weil 0° und 360°
  unsigned int height = _raysIncl;
  unsigned int size   = width * height;

  // unsigned int totalRays = (azimuthRays * verticalRays) - 1;	//Anzahl der totalRays -1 weil Zählung bei 0 beginnt

  obvious::System<double>::allocate(size, 3, coordsStart); //(row - for each point, column - xyz, nameArray)
  obvious::System<double>::allocate(size, 3, coordsEnd);

  viewer->showAxes();

  // TEST BACKPROJECTION
  // draw sphere
  double centerSphere[3] = {-0.2, 0.2, -0.077};
  double radiusSphere    = 0.05;
  viewer->addSphere(centerSphere, radiusSphere, rgbGreen);

  double       z         = centerSphere[2];
  double       y         = centerSphere[1];
  double       x         = centerSphere[0];
  const double piHalf    = obvious::deg2rad(90.0);
  double       inclAngle = 0.0;
  double       theta     = 0.0;
  double       azimAngle = 0.0;
  double       length    = sqrt(x * x + y * y + z * z);
  double       lengthInv = 1.0 / length;

  theta = acos(z * lengthInv);
  std::cout << "incl theta = " << obvious::rad2deg(theta) << std::endl;

  if((obvious::rad2deg(theta) < 75.0) && (obvious::rad2deg(theta) > 105.0))
  {
    std::cout << __PRETTY_FUNCTION__ << "3D coordinates are not within field of vision of laser scanner" << std::endl;
  }
  else
  {
    if(obvious::rad2deg(theta) > 90.0)
    {
      inclAngle = -(theta - piHalf);
    }
    else
    {
      inclAngle = piHalf - theta;
    }
    std::cout << __PRETTY_FUNCTION__ << " inclAngle = " << obvious::rad2deg(inclAngle) << std::endl;

    // AZIMUTH
    azimAngle = atan2(y, x);
    std::cout << __PRETTY_FUNCTION__ << " before adding 2 pi-- azimAngle = " << obvious::rad2deg(azimAngle) << std::endl;

    if(azimAngle < 0)
    {
      azimAngle += 2 * M_PI; // express angles positively in 3rd and 4th quadrant bec atan2 expresses them negatively
      std::cout << __PRETTY_FUNCTION__ << " after azimAngle = " << obvious::rad2deg(azimAngle) << std::endl;
    }
  }

  std::cout << __PRETTY_FUNCTION__ << " _inclSpan rad = " << _inclSpan << std::endl;
  std::cout << __PRETTY_FUNCTION__ << " _inclSpan deg = " << obvious::rad2deg(_inclSpan) << std::endl;
  std::cout << __PRETTY_FUNCTION__ << " _inclMin rad = " << _inclMin << std::endl;
  std::cout << __PRETTY_FUNCTION__ << " _inclMin deg = " << obvious::rad2deg(_inclMin) << std::endl;
  std::cout << __PRETTY_FUNCTION__ << " _inclRes rad = " << _inclRes << std::endl;
  std::cout << __PRETTY_FUNCTION__ << " _inclRes deg = " << obvious::rad2deg(_inclRes) << std::endl;

  std::cout << __PRETTY_FUNCTION__ << " inclAngle rad = " << inclAngle << std::endl;
  std::cout << __PRETTY_FUNCTION__ << " inclAngle deg = " << obvious::rad2deg(inclAngle) << std::endl;
  std::cout << __PRETTY_FUNCTION__ << " azimAngle rad = " << azimAngle << std::endl;
  std::cout << __PRETTY_FUNCTION__ << " azimAngle deg = " << obvious::rad2deg(azimAngle) << std::endl;

  unsigned int azimIndex = round(azimAngle / _azimRes);
  // shift up inclAngle here before idx calculations!!!
  double       inclShifted = inclAngle + _inclNegSpan;
  unsigned int inclIndex   = round(inclShifted / _inclRes);
  // unsigned int inclIndex = abs(((inclAngle - M_PI / 2) - _inclSpan + abs(_inclMin))) / _inclRes; //Das ist absoluter bullshit
  std::cout << __PRETTY_FUNCTION__ << "Sphere 1: azimIndex = " << azimIndex << std::endl;
  std::cout << __PRETTY_FUNCTION__ << "Sphere 1: inclIndex = " << inclIndex << std::endl;

  // returnRayIndex(azimAngle, inclAngle, &azimIndex, &inclIndex);

  std::cout << "......................................................." << std::endl;

  // draw second sphere
  double centerSphere2[3] = {0.0, 0.4, 0.0};
  double radiusSphere2    = 0.025;
  viewer->addSphere(centerSphere2, radiusSphere2, rgbRed);

  double x2         = centerSphere2[0];
  double y2         = centerSphere2[1];
  double z2         = centerSphere2[2];
  double inclAngle2 = 0.0;
  double azimAngle2 = 0.0;
  returnAngles(x2, y2, z2, &inclAngle2, &azimAngle2);
  std::cout << __PRETTY_FUNCTION__ << " inclAngle2 rad = " << inclAngle2 << std::endl;
  std::cout << __PRETTY_FUNCTION__ << " inclAngle2 deg = " << obvious::rad2deg(inclAngle2) << std::endl;
  std::cout << __PRETTY_FUNCTION__ << " azimAngle2 rad = " << azimAngle2 << std::endl;
  std::cout << __PRETTY_FUNCTION__ << " azimAngle2 deg = " << obvious::rad2deg(azimAngle2) << std::endl;

  unsigned int azimIndex2 = round(azimAngle2 / _azimRes);
  // shift up inclAngle here before idx calculations!!!
  double       inclShifted2 = inclAngle2 + _inclNegSpan;
  unsigned int inclIndex2   = round(inclShifted2 / _inclRes);
  std::cout << __PRETTY_FUNCTION__ << "Sphere 2: azimIndex = " << azimIndex2 << std::endl;
  std::cout << __PRETTY_FUNCTION__ << "Sphere 2: inclIndex = " << inclIndex2 << std::endl;

  std::cout << "......................................................." << std::endl;

  // draw third sphere
  double centerSphere3[3] = {0.23, 0.0, 0.01};
  double radiusSphere3    = 0.01;
  viewer->addSphere(centerSphere3, radiusSphere3, rgbBlue);

  double x3         = centerSphere3[0];
  double y3         = centerSphere3[1];
  double z3         = centerSphere3[2];
  double inclAngle3 = 0.0;
  double azimAngle3 = 0.0;
  returnAngles(x3, y3, z3, &inclAngle3, &azimAngle3);
  std::cout << __PRETTY_FUNCTION__ << " inclAngle3 rad = " << inclAngle3 << std::endl;
  std::cout << __PRETTY_FUNCTION__ << " inclAngle3 deg = " << obvious::rad2deg(inclAngle3) << std::endl;
  std::cout << __PRETTY_FUNCTION__ << " azimAngle3 rad = " << azimAngle3 << std::endl;
  std::cout << __PRETTY_FUNCTION__ << " azimAngle3 deg = " << obvious::rad2deg(azimAngle3) << std::endl;

  unsigned int azimIndex3 = round(azimAngle3 / _azimRes);
  // shift up inclAngle here before idx calculations!!!
  double       inclShifted3 = inclAngle3 + _inclNegSpan;
  unsigned int inclIndex3   = round(inclShifted3 / _inclRes);
  std::cout << __PRETTY_FUNCTION__ << "Sphere 3: azimIndex = " << azimIndex3 << std::endl;
  std::cout << __PRETTY_FUNCTION__ << "Sphere 3: inclIndex = " << inclIndex3 << std::endl;

  // setIndexMap(width, height);
  // set indexmap here
  obvious::System<int>::allocate(width, _raysIncl, _indexMap); // ACHTUNG width ist schon raysAzim +1 !!
  for(unsigned int row = 0; row < width; row++)                // r row = azimuth
  {
    for(unsigned int column = 0; column <= _raysIncl; column++) // c column = incl
    {
      _indexMap[row][column] = row * _raysIncl + column; // ohne +1
    }
  }

  obvious::Matrix* _rays;
  _rays = new obvious::Matrix(3, size);

  unsigned int n = 0;
  for(unsigned int i = 0; i < width; i++) // AZIM
  {
    double currentAzim = _azimMin + i * _azimRes;
    // std::cout << "currentAzim = " << obvious::rad2deg(currentAzim) << std::endl;

    for(unsigned int j = 0; j < height; j++, n++) // INCL ACHTUNG HEIGHT MUSS HIER INCLSPAN SEIN! SONST HAB ICHZU WENIGE!!! INDICES
    // for(unsigned int j = 0; j < _inclSpan; j++, n++) // INCL ACHTUNG HEIGHT MUSS HIER INCLSPAN SEIN! SONST HAB ICHZU WENIGE!!! INDICES

    {

      double currentIncl = _inclMin + j * _inclRes;
      // std::cout << "currentIncl = " << obvious::rad2deg(currentIncl) << " @ n = " << n << " !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;

      if(currentIncl <= 0)
      {
        inclCorr = piHalf + currentIncl * (-1);
      }
      else
      {
        inclCorr = piHalf - currentIncl;
      }
      coordsStart[n][0] = 0.0;
      coordsStart[n][1] = 0.0;
      coordsStart[n][2] = 0.0;

      // längere Rays, der durch Kugeln geht
      if((i == azimIndex) && (j == inclIndex))
      {
        std::cout << "1: azim of current ray is " << obvious::rad2deg(currentAzim) << std::endl;
        std::cout << "1: incl of current ray is " << obvious::rad2deg(inclCorr) << std::endl;
        std::cout << "val _indexMap[width=row=azim][height=col=incl] at azimIndex " << azimIndex << " and inclIndex " << inclIndex
                  << "is: " << _indexMap[azimIndex][inclIndex] << std::endl;
        coordsEnd[n][0] = 0.5 * sin(inclCorr) * cos(currentAzim);
        coordsEnd[n][1] = 0.5 * sin(inclCorr) * sin(currentAzim);
        coordsEnd[n][2] = 0.5 * cos(inclCorr);
      }
      else if((i == azimIndex2) && (j == inclIndex2))

      {
        std::cout << "2: azim of current ray is " << obvious::rad2deg(currentAzim) << std::endl;
        std::cout << "2: incl of current ray is " << obvious::rad2deg(inclCorr) << std::endl;
        coordsEnd[n][0] = 0.5 * sin(inclCorr) * cos(currentAzim);
        coordsEnd[n][1] = 0.5 * sin(inclCorr) * sin(currentAzim);
        coordsEnd[n][2] = 0.5 * cos(inclCorr);
      }
      else if((i == azimIndex3) && (j == inclIndex3))

      {
        std::cout << "2: azim of current ray is " << obvious::rad2deg(currentAzim) << std::endl;
        std::cout << "2: incl of current ray is " << obvious::rad2deg(inclCorr) << std::endl;
        coordsEnd[n][0] = 0.5 * sin(inclCorr) * cos(currentAzim);
        coordsEnd[n][1] = 0.5 * sin(inclCorr) * sin(currentAzim);
        coordsEnd[n][2] = 0.5 * cos(inclCorr);
      }
      else
      {
        coordsEnd[n][0] = 0.1 * sin(inclCorr) * cos(currentAzim);
        coordsEnd[n][1] = 0.1 * sin(inclCorr) * sin(currentAzim);
        coordsEnd[n][2] = 0.1 * cos(inclCorr);
      }

      // for length calc
      obvious::Matrix ray(3, 1);
      ray(0, 0) = coordsEnd[n][0];
      ray(1, 0) = coordsEnd[n][1];
      ray(2, 0) = coordsEnd[n][2];
      // normalize length
      const double length    = sqrt(ray(0, 0) * ray(0, 0) + ray(1, 0) * ray(1, 0) + ray(2, 0) * ray(2, 0));
      const double lengthInv = 1.0 / length;
      // store in pointer _rays which is later inherited from class Sensor
      (*_rays)(0, j) = ray(0, 0) * lengthInv;
      (*_rays)(1, j) = ray(1, 0) * lengthInv;
      (*_rays)(2, j) = ray(2, 0) * lengthInv;
      // todo: indexMap
    }
    // std::cout << "n = " << n << std::endl;
    viewer->addLines(coordsStart, coordsEnd, size, NULL);
  }

  viewer->startRendering();
  delete viewer;
} // main
