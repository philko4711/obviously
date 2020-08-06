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


double azimuthResolution = 3.0;
double inclResolution = 2.0;
int** _indexMap;

/**
 * returns azimuth angle and inclination angle
 * @param[in] xCoord x coordinate of a 3D point
 * @param[in] yCoord y coordinate of a 3D point
 * @param[in] zCoord z coordinate of a 3D point
 * @param[out] inclAngle inclination angle in x-z-plane, 16 layers of rays from -15° to +15°, 2° resolution (VLP16 PUCK)
 * @param[out] azimAngle azimuth angle in x-y-plane, 0° to 360°
 */
void returnAngles(double xCoord, double yCoord, double zCoord, double* inclAngle, double* azimAngle)
{
  //INCLINATION
  double theta = 0.0;   //careful - this is the angle between z-axis and x-y-plane as defined in polar coords --> not inclination angle

  //das macht May mit acos
//  double r = sqrt(coords3D(0,i) * coords3D(0,i) + coords3D(1,i) * coords3D(1,i) + coords3D(2,i) * coords3D(2,i));
//  double theta = acos(coords3D(1,i) / r);
//  if(coords3D(2,i)>0)
//    theta = -theta;catkin

  double length = sqrt(xCoord*xCoord + yCoord*yCoord + zCoord*zCoord);
  double lengthInv = 1.0 / length;

  theta = obvious::rad2deg(acos(zCoord/length));
  std::cout << "theta = " << theta << std::endl;


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
  *azimAngle = obvious::rad2deg(atan2(yCoord,xCoord));
  if(*azimAngle < 0)
  {
    *azimAngle += 360.0;    //express angles positively in 3rd and 4th quadrant
  }

  std::cout << __PRETTY_FUNCTION__ << " azimAngle = " << *azimAngle << std::endl;
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
  *azimIndex = round(azimAngle/azimuthResolution);
  std::cout << __PRETTY_FUNCTION__ << " azimIndex = " << *azimIndex << std::endl;

  //INCLINATION --> may differ for different sensors
  //assignment will always be the same for VLP16 - inclination resolution is fixed 2° and nbr of rays is 16
  double mapInclination = inclAngle + 15.0;                //map inclination angles (-15° -> +15°) up to positive range 0° - 30°
  *inclIndex = round(mapInclination / inclResolution);    //maybe exchange round with floor ? or ceiling? idk

  std::cout << __PRETTY_FUNCTION__ << " inclIndex = " << *inclIndex << std::endl;
}

void setIndexMap(unsigned int azimuthRays, unsigned int verticalRays)
{
  unsigned int column = 0;
  //evtl mit _height u _width ersetzen
  obvious::System<int>::allocate(azimuthRays, verticalRays, _indexMap);
  for(unsigned int row=0; row < azimuthRays; row++)     //r row
  {
    for(column = 0; column <= verticalRays; column++)  //c column
    {

      _indexMap[row][column] = row*(verticalRays+1) + column;         //todo stimm tdas so mit +1? 2040 indices insgesamt 0-2039

      std::cout << "_indexMap = " << _indexMap[row][column] << std::endl;
    }
    column=0;     //iterate over 16 vertical rays for each azimuth ray
  }
}

void setDistanceMap(vector<float> phiAzim, vector<float> thetaIncl, vector<float> dist)
{

}


int main(void)
{
  double bg[3] = {1.0, 1.0, 1.0};
  obvious::Obvious3D* viewer = new obvious::Obvious3D("VLP16 Raycast", 1024, 768, 0, 0, bg);
  double** coordsStart;
  double** coordsEnd;

//  double** coordsStartColor;
//  double** coordsEndColor;

  double rgbGreen[3] = {0.0,180.0,153.0};
//  double rgbRed[3] = {255.0,0.0,51.0};

  float angleInclThetaMin = -15.0;				//smallest inclination angle - "lowest" ray
  double thetaSphere = 0.0;		//für Definition des Winkels für Kugelkoordinaten
  double angleAzimuth = 0.0;
  unsigned int azimuthRays = static_cast<unsigned>(360 / azimuthResolution);
  unsigned int verticalRays = 16;

  unsigned int totalRays = (azimuthRays * verticalRays) - 1;	//Anzahl der totalRays -1 weil Zählung bei 0 beginnt
  obvious::System<double>::allocate(totalRays, 3, coordsStart);  //(row - for each point, column - xyz, nameArray)
  obvious::System<double>::allocate(totalRays, 3, coordsEnd);

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

    setIndexMap(azimuthRays, verticalRays);

  obvious::Matrix* _rays;
  _rays = new obvious::Matrix(3, totalRays);

  for(unsigned int i = 0; i < azimuthRays; i++)
  {
    for(unsigned int j = 0; j < 16; j++)
    {
      if(angleInclThetaMin < 0)
      {
        thetaSphere = 90.0 + (angleInclThetaMin * (-1));
      }
      else
      {
        thetaSphere = 90.0 - angleInclThetaMin;
      }
      coordsStart[j][0] = 0.0;
      coordsStart[j][1] = 0.0;
      coordsStart[j][2] = 0.0;

      //längerer Ray, der durch Kugel geht
      if( (i == azimIndex) && (j == inclIndex) )

      {
        std::cout << "angleAzimuth of current ray is " << angleAzimuth << std::endl;
        std::cout << "angleInclination of current ray is " << angleInclThetaMin << std::endl;
      coordsEnd[j][0] = 0.5 * sin((thetaSphere*M_PI)/180) * cos((angleAzimuth*M_PI)/180);
      coordsEnd[j][1] = 0.5 * sin((thetaSphere*M_PI)/180) * sin((angleAzimuth*M_PI)/180);
      coordsEnd[j][2] = 0.5 * cos((thetaSphere*M_PI)/180);
      }
      else
      {
      coordsEnd[j][0] = 0.1 * sin((thetaSphere*M_PI)/180) * cos((angleAzimuth*M_PI)/180);
      coordsEnd[j][1] = 0.1 * sin((thetaSphere*M_PI)/180) * sin((angleAzimuth*M_PI)/180);
      coordsEnd[j][2] = 0.1 * cos((thetaSphere*M_PI)/180);
      }

      //for length calc
      obvious::Matrix ray(3, 1);
      ray(0, 0) = coordsEnd[j][0];
      ray(1, 0) = coordsEnd[j][1];
      ray(2, 0) = coordsEnd[j][2];
      //normalize length
      const double length = sqrt(ray(0, 0) * ray(0, 0) + ray(1, 0) * ray(1, 0) + ray(2, 0) * ray(2, 0));
      const double lengthInv = 1.0 / length;
      //store in pointer _rays which is later inherited from class Sensor
      (*_rays)(0, j) = ray(0, 0) * lengthInv;
      (*_rays)(1, j) = ray(1, 0) * lengthInv;
      (*_rays)(2, j) = ray(2, 0) * lengthInv;
      //todo: indexMap

      angleInclThetaMin += inclResolution;
    }
    angleInclThetaMin = -15.0;		//austauschen -15.0 gegen inclMin
    angleAzimuth += azimuthResolution;

    viewer->addLines(coordsStart, coordsEnd, totalRays, NULL);
  }

  viewer->startRendering();
  delete viewer;
}  //main









