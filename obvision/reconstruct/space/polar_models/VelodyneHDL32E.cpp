#include "VelodyneHDL32E.h"

namespace obvious
{

VelodyneHDL32E::VelodyneHDL32E(unsigned int raysIncl, double inclMin, double inclMax, double inclRes, double azimMin, double azimMax, double azimRes)
    : SensorPolar3DBase(raysIncl, inclMin, inclMax, inclRes, azimMin, azimMax, azimRes)
{
  std::cout << __PRETTY_FUNCTION__ << "Hi." << std::endl;
}

VelodyneHDL32E::~VelodyneHDL32E() {}

int VelodyneHDL32E::lookupIndex(int inclIndex)
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
    indexVelodyneROS = 16;
    break;
  case 9:
    indexVelodyneROS = 18;
    break;
  case 10:
    indexVelodyneROS = 20;
    break;
  case 11:
    indexVelodyneROS = 22;
    break;
  case 12:
    indexVelodyneROS = 24;
    break;
  case 13:
    indexVelodyneROS = 26;
    break;
  case 14:
    indexVelodyneROS = 28;
    break;
  case 15:
    indexVelodyneROS = 30;
    break;
  case 16:
    indexVelodyneROS = 1;
    break;
  case 17:
    indexVelodyneROS = 3;
    break;
  case 18:
    indexVelodyneROS = 5;
    break;
  case 19:
    indexVelodyneROS = 7;
    break;
  case 20:
    indexVelodyneROS = 9;
    break;
  case 21:
    indexVelodyneROS = 11;
    break;
  case 22:
    indexVelodyneROS = 13;
    break;
  case 23:
    indexVelodyneROS = 15;
    break;
  case 24:
    indexVelodyneROS = 17;
    break;
  case 25:
    indexVelodyneROS = 19;
    break;
  case 26:
    indexVelodyneROS = 21;
    break;
  case 27:
    indexVelodyneROS = 23;
    break;
  case 28:
    indexVelodyneROS = 25;
    break;
  case 29:
    indexVelodyneROS = 27;
    break;
  case 30:
    indexVelodyneROS = 29;
    break;
  case 31:
    indexVelodyneROS = 31;
    break;
  default:
    std::cout << __PRETTY_FUNCTION__ << " index not valid - aborting." << std::endl;
    std::abort();
  }
  return indexVelodyneROS;
}

} // namespace obvious