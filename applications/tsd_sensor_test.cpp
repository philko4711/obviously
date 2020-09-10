#include "obcore/math/mathbase.h"
#include "obgraphic/Obvious3D.h"
#include "obvision/reconstruct/space/SensorVelodyne3DNew.h"
#include "obvision/reconstruct/space/TsdSpace.h"
#include <Eigen/Dense>

#define VXLDIM 0.025
#define LAYOUTPARTITION obvious::LAYOUT_32x32x32
#define LAYOUTSPACE obvious::LAYOUT_1024x1024x1024
obvious::Matrix*              _T;
obvious::Matrix               _Tinit(4, 4);
obvious::Obvious3D*           _viewer3D;
obvious::VtkCloud*            _vInput;
obvious::VtkCloud*            _vRecon;
obvious::TsdSpace*            _space;
obvious::SensorVelodyne3DNew* _sensor;

typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > stdVecEig3d;
void                                                                             redBlueRenderSpace(obvious::VtkCloud& cloud);
unsigned int circles(double* coords, const double azimMin, const double azimMax, const double azimRes, const double inclMin, const double inclMax,
                     const double inclRes);
unsigned int spiral(double* coords, const double azimMin, const double azimMax, const double azimRes, const double inclMin, const double inclMax,
                    const double inclRes, const double absBase, const double absDiff);

int main(int argc, char** argv)
{
  _space = new obvious::TsdSpace(VXLDIM, LAYOUTPARTITION, LAYOUTSPACE);
  _space->setMaxTruncation(3.0 * VXLDIM);
  obvious::obfloat tr[3];
  _space->getCentroid(tr);
  tr[2]         = 0.0;
  double tf[16] = {1, 0, 0, tr[0], 0, 1, 0, tr[1], 0, 0, 1, tr[2], 0, 0, 0, 1};
  _Tinit.setData(tf);
  unsigned int raysIncl = 16;
  double       inclMin  = obvious::deg2rad(-15.0);
  double       inclMax  = obvious::deg2rad(15.0);
  double       inclRes  = obvious::deg2rad(2.0);
  double       azimMin  = obvious::deg2rad(0.0);
  double       azimMax  = obvious::deg2rad(359.8);
  double       azimRes  = obvious::deg2rad(0.2);
  _sensor               = new obvious::SensorVelodyne3DNew(raysIncl, inclMin, inclMax, inclRes, azimMin, azimMax, azimRes);

  _sensor->setTransformation(_Tinit);
  std::cout << __PRETTY_FUNCTION__ << " sensor initial transform " << std::endl;
  _sensor->getTransformation().print();

  const unsigned int nAzimutz = static_cast<unsigned int>(std::round((azimMax - azimMin) / azimRes));
  const unsigned int nIncl    = static_cast<unsigned int>(std::round((inclMax - inclMin) / inclRes));

  std::cout << __PRETTY_FUNCTION__ << " nAz " << nAzimutz << " nIncl " << nIncl << std::endl;
  std::cout << __PRETTY_FUNCTION__ << " sensor az" << _sensor->getWidth() << " nIncl " << _sensor->getHeight() << std::endl;
  const unsigned int size   = _sensor->getWidth() * _sensor->getHeight() * 3;
  double*            coords = new double[size];

  // create synthetic data
  unsigned int ctr = circles(coords, azimMin, azimMax, azimRes, inclMin, inclMax, inclRes);
  // unsigned int       ctr       = spiral(coords, azimMin, azimMax, azimRes, inclMin, inclMax, inclRes, 1.0, 0.005);

  double* depthData = new double[size / 3];
  bool*   mask      = new bool[size / 3];
  for(unsigned int i = 0; i < size / 3; i++)
  {
    Eigen::Vector3d vec(coords[i * 3], coords[i * 3 + 1], coords[i * 3 + 2]);
    depthData[i] = vec.norm();
    if(vec.norm() > 0)
      mask[i] = true;
    else
      mask[i] = false;
  }
  std::cout << __PRETTY_FUNCTION__ << " start push " << std::endl;
  _sensor->setRealMeasurementData(depthData);
  _sensor->setRealMeasurementMask(mask);
  _space->push(_sensor);
  std::cout << __PRETTY_FUNCTION__ << " end push" << std::endl;

  _vInput = new obvious::VtkCloud();
  _vInput->setCoords(coords, size / 3, 3, NULL);
  _vRecon = new obvious::VtkCloud();
  redBlueRenderSpace(*_vRecon);
  _viewer3D = new obvious::Obvious3D("3DMapper");
  _viewer3D->addCloud(_vInput);
  _viewer3D->addCloud(_vRecon);
  _viewer3D->addAxisAlignedCube(0, _space->getMaxX(), 0, _space->getMaxY(), 0, _space->getMaxZ());
  _viewer3D->showAxes(true);
  _viewer3D->startRendering();
}

void redBlueRenderSpace(obvious::VtkCloud& cloud)
{
  static unsigned int seq = 0;
  struct Color
  {
    Color(uint8_t r, uint8_t g, uint8_t b) : r(r), g(g), b(b) {}
    Color() : r(0), g(0), b(0) {}
    void    red(uint8_t val) { r = val; }
    void    blue(uint8_t val) { b = val; }
    uint8_t r;
    uint8_t g;
    uint8_t b;
  };

  obvious::Matrix*   cellCoordsHom = obvious::TsdSpacePartition::getCellCoordsHom();
  obvious::Matrix*   partCoords    = obvious::TsdSpacePartition::getPartitionCoords();
  unsigned int       partSize      = _space->getPartitions()[0][0][0]->getSize();
  stdVecEig3d        centers;
  std::vector<Color> colors;
  obvious::obfloat   tr[3];
  _space->getCentroid(tr);
  for(unsigned int pz = 0; pz < _space->getPartitionsInZ(); pz++)
  {
    for(unsigned int py = 0; py < _space->getPartitionsInY(); py++)
    {
      for(unsigned int px = 0; px < _space->getPartitionsInX(); px++)
      {
        obvious::TsdSpacePartition* part = _space->getPartitions()[pz][py][px];
        if(part->isInitialized() && !part->isEmpty())
        {
          obvious::obfloat t[3];
          part->getCellCoordsOffset(t);
          for(unsigned int c = 0; c < partSize; c++)
          {
            Eigen::Vector3d center;
            center(0) = (*cellCoordsHom)(c, 0) + t[0];
            center(1) = (*cellCoordsHom)(c, 1) + t[1];
            center(2) = (*cellCoordsHom)(c, 2) + t[2];
            // if(center(1) > _space->getMaxY() / 2.0)
            //   continue;
            obvious::obfloat tsd = part->getTsd((*partCoords)(c, 0), (*partCoords)(c, 1), (*partCoords)(c, 2));
            if((isnan(tsd)) || (tsd > std::abs(1.1)))
              continue;

            Color tsdColor; //(0, 0, 0);
            if(tsd < 0.0)   // red
              tsdColor.red(static_cast<unsigned char>(-1.0 * tsd * 255.0));
            else
              tsdColor.blue(static_cast<unsigned char>(tsd * 255.0));

            centers.push_back(center);
            colors.push_back(tsdColor);
          }
        }
      }
    }
  }
  double*        coords   = new double[centers.size() * 3];
  unsigned char* colorsOb = new unsigned char[centers.size() * 3];
  if((centers.size() == colors.size()) && (centers.size() != 0))
  {
    //_gui->drawGlyphs(colors, centers, space.getVoxelSize());
    for(unsigned int i = 0; i < centers.size(); i++)
    {
      coords[i * 3]     = centers[i].x();
      coords[i * 3 + 1] = centers[i].y();
      coords[i * 3 + 2] = centers[i].z();

      colorsOb[i * 3]     = colors[i].r;
      colorsOb[i * 3 + 1] = colors[i].g;
      colorsOb[i * 3 + 2] = colors[i].b;
    }
  }
  else
    std::cout << __PRETTY_FUNCTION__ << "nuttingham found " << centers.size() << " " << colors.size() << std::endl;
  cloud.setCoords(coords, centers.size(), 3, NULL);
  cloud.setColors(colorsOb, centers.size(), 3);
}

unsigned int circles(double* coords, const double azimMin, const double azimMax, const double azimRes, const double inclMin, const double inclMax,
                     const double inclRes)
{
  double       r   = 1.0;
  unsigned int ctr = 0;
  for(double azim = azimMin; azim < azimMax; azim += azimRes)
  {
    for(double incl = inclMin; incl < (inclMax + inclRes); incl += inclRes, ctr++)
    {
      if(azim < azimMax / 4.0)
        r = 0.25;
      else if(azim < azimMax / 2.0)
        r = 0.5;
      else if(azim < 3.0 * azimMax / 4.5)
        r = 0.75;
      else
        r = 1.0;

      double theta = 0.0;
      if(incl < 0.0)
        theta = M_PI / 2.0 + std::abs(incl);
      else
      {
        theta = M_PI / 2.0 - incl;
      }
      coords[ctr * 3]     = r * std::sin(theta) * std::cos(azim);
      coords[ctr * 3 + 1] = r * std::sin(theta) * std::sin(azim);
      coords[ctr * 3 + 2] = std::cos(theta);
    }
  }
  return ctr;
}

unsigned int spiral(double* coords, const double azimMin, const double azimMax, const double azimRes, const double inclMin, const double inclMax,
                    const double inclRes, const double absBase, const double absDiff)
{
  unsigned int ctr = 0;
  double       r   = absBase;
  for(double azim = azimMin; azim < azimMax; azim += azimRes, r += absDiff)
  {
    for(double incl = inclMin; incl < inclMax + inclRes; incl += inclRes, ctr++)
    {
      double theta = 0.0;
      if(incl < 0.0)
        theta = M_PI / 2.0 + std::abs(incl);
      else
      {
        theta = M_PI / 2.0 - incl;
      }
      coords[ctr * 3]     = r * std::sin(theta) * std::cos(azim);
      coords[ctr * 3 + 1] = r * std::sin(theta) * std::sin(azim);
      coords[ctr * 3 + 2] = std::cos(theta);
    }
  }
  return ctr;
}