#include "VelodynePUCKVLP16.h"

namespace obvious
{

VelodynePUCKVLP16::VelodynePUCKVLP16(unsigned int raysIncl, double inclMin, double inclMax, double inclRes, double azimMin, double azimMax, double azimRes)
    : SensorPolar3DBase(raysIncl, inclMin, inclMax, inclRes, azimMin, azimMax, azimRes)
{
  std::cout << __PRETTY_FUNCTION__ << "Hi." << std::endl;
}

VelodynePUCKVLP16::~VelodynePUCKVLP16() {}

int VelodynePUCKVLP16::lookupIndex(int inclIndex)
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
  default:
    std::cout << __PRETTY_FUNCTION__ << " index not valid - aborting." << std::endl;
    std::abort();
  }
  return indexVelodyneROS;
}

} // namespace obvious