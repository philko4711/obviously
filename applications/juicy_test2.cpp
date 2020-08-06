/*
 * juicy_test.cpp
 *
 *      Author: jasmin
 */

#include "obgraphic/Obvious3D.h"
#include "obcore/base/System.h"
#include "math.h"
#include <iostream>
#include "obcore/math/linalg/eigen/Matrix.h"
#include "obcore/math/mathbase.h"


double azimRes = obvious::deg2rad(3.0);
double azimMin = obvious::deg2rad(0.0);
double inclRes = obvious::deg2rad(2.0);
int** _indexMap;


void returnAngles(double xCoord, double yCoord, double zCoord, double* inclAngle, double* azimAngle)
{
  //INCLINATION
  double theta = 0.0;   //careful - this is the angle between z-axis and x-y-plane as defined in polar coords --> not inclination angle

  double length = sqrt(xCoord*xCoord + yCoord*yCoord + zCoord*zCoord);
  double lengthInv = 1.0 / length;

  theta = acos(zCoord * lengthInv);
  std::cout << "incl theta = " << obvious::rad2deg(theta) << std::endl;


if((theta < 75.0) && (theta > 105.0))
{
  std::cout << __PRETTY_FUNCTION__ << "3D coordinates are not within field of vision of laser scanner" << std::endl;
}
else
{
  if(theta > 90.0)
  {
    *inclAngle = - (theta - 90.0);
  }
  else
  {
    *inclAngle = 90.0 - theta;
  }
  std::cout << __PRETTY_FUNCTION__ << " inclAngle = " << *inclAngle << std::endl;

  //AZIMUTH
  *azimAngle = atan2(yCoord,xCoord);
  if(*azimAngle < 0)
  {
    *azimAngle += 2 * M_PI;    //express angles positively in 3rd and 4th quadrant
  }

  std::cout << __PRETTY_FUNCTION__ << " azimAngle = " << obvious::rad2deg(*azimAngle) << std::endl;
  }
}


/**
 * returns ray index
 * @param[in] azimAngle azimuth angle calculated in returnAngles()
 * @param[in] inclAngle inclination angle calculated in returnAngles()
 * @param[out] azimIndex index of azimuth ray number which is closest to 3D point to assign laser data to 3D point
 * @param[out] inclIndex index of inclination ray number which is closest to 3D point to assign laser data to 3D point
 * @todo make this function abstract so each velodyne sensor has to implement it since it may differ from sensor to sensor
 */
void returnRayIndex(double azimAngle, double inclAngle, unsigned int* azimIndex, unsigned int* inclIndex)
{
  //AZIMUTH
  *azimIndex = round(azimAngle/azimRes);
  std::cout << __PRETTY_FUNCTION__ << " azimIndex = " << *azimIndex << std::endl;

  //INCLINATION --> may differ for different sensors
  //assignment will always be the same for VLP16 - inclination resolution is fixed 2° and nbr of rays is 16
  double mapInclination = inclAngle + obvious::deg2rad(15.0);                //map inclination angles (-15° -> +15°) up to positive range 0° - 30°
  *inclIndex = round(mapInclination / inclRes);    //maybe exchange round with floor ? or ceiling? idk

  std::cout << __PRETTY_FUNCTION__ << " inclIndex = " << *inclIndex << std::endl;
}

void setIndexMap(unsigned int width, unsigned int height)
{
  unsigned int column = 0;
  //evtl mit _height u _width ersetzen
  obvious::System<int>::allocate(width, height, _indexMap);
  for(unsigned int row=0; row < width; row++)     //r row = azimuth
  {
    for(column = 0; column <= height; column++)  //c column = incl
    {

      // _indexMap[row][column] = row*(verticalRays+1) + column;         //todo stimm tdas so mit +1? 2040 indices insgesamt 0-2039
      _indexMap[row][column] = row*height + column;         //ohne +1

      std::cout << "_indexMap = " << _indexMap[row][column] << std::endl;
    }
    // column=0;     //iterate over 16 vertical rays for each azimuth ray
  }
}


int main(void)
{
  double bg[3] = {1.0, 1.0, 1.0};
  obvious::Obvious3D* viewer = new obvious::Obvious3D("Juicy VLP16 Raycast Visualization Test", 1024, 768, 0, 0, bg);
  double** coordsStart;
  double** coordsEnd;

  double rgbGreen[3] = {0.0,180.0,153.0};
//  double rgbRed[3] = {255.0,0.0,51.0};

  double inclMin = obvious::deg2rad(-15.0);				//smallest inclination angle - "lowest" ray
  double inclCorr = 0.0;		//für Definition des Winkels für Kugelkoordinaten
  double angleAzimuth = 0.0;
  unsigned int raysAzim = static_cast<unsigned>(2 * M_PI / azimRes);
  unsigned int raysIncl = 16;

  unsigned int width = raysAzim + 1; //weil 0° und 360°
  unsigned int height = raysIncl;

  unsigned int size = width * height;

  // unsigned int totalRays = (azimuthRays * verticalRays) - 1;	//Anzahl der totalRays -1 weil Zählung bei 0 beginnt
  
  obvious::System<double>::allocate(size, 3, coordsStart);  //(row - for each point, column - xyz, nameArray)
  obvious::System<double>::allocate(size, 3, coordsEnd);

  viewer->showAxes();

  //TEST BACKPROJECTION
    //draw sphere
    double centerSphere[3] = {-0.2, 0.2, -0.077};
    double radiusSphere = 0.05;
    viewer->addSphere(centerSphere, radiusSphere, rgbGreen);

    double z = centerSphere[2];
    double y = centerSphere[1];
    double x = centerSphere[0];

    double inclAngle = 0.0;
    double azimAngle = 0.0;

    returnAngles(x, y, z, &inclAngle, &azimAngle);

    unsigned int azimIndex = 0;
    unsigned int inclIndex = 0;
    returnRayIndex(azimAngle, inclAngle, &azimIndex, &inclIndex);

    setIndexMap(width, height);

  obvious::Matrix* _rays;
  _rays = new obvious::Matrix(3, size);

  unsigned int n = 0;
  for(unsigned int i = 0; i < width; i++) //AZIM
  {
    double currentAzim = azimMin + i * azimRes;
    std::cout << "currentAzim = " << obvious::rad2deg(currentAzim) << std::endl;

    for(unsigned int j = 0; j < height; j++, n++)  // INCL
    {

      double currentIncl = inclMin + j * inclRes;
    // std::cout << "currentIncl = " << obvious::rad2deg(currentIncl) << " @ n = " << n << " !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
      const double piHalf      = obvious::deg2rad(90.0);

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

      //längerer Ray, der durch Kugel geht
      if( (i == azimIndex) && (j == inclIndex) )

      {
        std::cout << "azim of current ray is " << obvious::rad2deg(currentAzim) << std::endl;
        std::cout << "incl of current ray is " << obvious::rad2deg(inclCorr) << std::endl;
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

      //for length calc
      obvious::Matrix ray(3, 1);
      ray(0, 0) = coordsEnd[n][0];
      ray(1, 0) = coordsEnd[n][1];
      ray(2, 0) = coordsEnd[n][2];
      //normalize length
      const double length = sqrt(ray(0, 0) * ray(0, 0) + ray(1, 0) * ray(1, 0) + ray(2, 0) * ray(2, 0));
      const double lengthInv = 1.0 / length;
      //store in pointer _rays which is later inherited from class Sensor
      (*_rays)(0, j) = ray(0, 0) * lengthInv;
      (*_rays)(1, j) = ray(1, 0) * lengthInv;
      (*_rays)(2, j) = ray(2, 0) * lengthInv;
      //todo: indexMap
    }
    std::cout << "n = " << n << std::endl;
    viewer->addLines(coordsStart, coordsEnd, size, NULL);
  }

  viewer->startRendering();
  delete viewer;
}  //main









