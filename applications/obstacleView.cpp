/**
* @file obstacleView.cpp
* @autor christian
* @date  11.01.2013
*
*
*/



#include "obdevice/Xtion.h"
#include "obgraphic/Obvious3D.h"
#include "obgraphic/Obvious2D.h"
#include "obgraphic/Obvious2DMap.h"
#include "obcore/grid/ObstacleGrid.h"
#include "obcore/base/Image.h"
#include "obvision/raycast/Raycast2D.h"
#include "obvision/normals/NormalsEstimator.h"
#include "obcore/filter/CartesianFilter.h"

using namespace std;
using namespace obvious;


int main(int argc, char* argv[])
{
  Xtion*                  _xtion  = new Xtion(argv[1]);
  ObstacleGrid*                _G = new ObstacleGrid(0.04, 8.0, 8.0);
  Obvious2DMap*           _viewer = new Obvious2DMap(800, 800, "Obstacle streaming", 8.0, 8.0);
  Image*                     _img = new Image(_G->getCols(), _G->getRows(), Image::COLORED);
  NormalsEstimator*   _nestimator = new NormalsEstimator();
  CartesianFilter*        _filter = new CartesianFilter();

  unsigned int size            = _xtion->getRows()*_xtion->getCols();
  double*       _normals       = new double[size*3];
  Raycast2D*    _RC              = new Raycast2D();

  // config viewer
  _viewer->showCircle();
  _viewer->showAngles();

  // config filter
  _filter->setAxis(IFilter::y);
  _filter->setThreshold(0.1);

  // config obstacle map
  _G->setCriticalStepHeight(0.2); // 20 centimeter
  _G->setWeightStepHeight(0.4);
  _G->setCriticalSlope(0.42); //30 degrees
  _G->setWeightSlope(0.6);

  double x, y;
  while(1)
  {
    if(_xtion->grab())
    {
      unsigned int cols = _xtion->getCols();
      unsigned int rows = _xtion->getRows();
      double* coords    = _xtion->getCoords();
      bool*   mask      = _xtion->getMask();

      unsigned int j=0;
      for(unsigned int i=0 ; i<size*3 ; i+=3,j++)
      {
        if (coords[i+1] >= 0.2)
          mask[j] = false;
      }

      _nestimator->estimateNormals3DGrid(cols, rows, coords, mask, _normals);

      _G->normals2Grid(coords, _normals, mask, size*3);
      _G->height2Grid(coords, mask, size*3);

      _RC->setInput(_G);
      _G->getObstacles();
      _RC->estimateFreeSpace(true);

      _img->setImg(_G->getImageOfGrid());
    }
    _viewer->draw(_img->getImg(), _img->getWidth(), _img->getHeight(), _img->getType());
  }

  delete _normals;
  delete _nestimator;
  delete _xtion;
  delete _G;
  delete _img;
  delete _viewer;
  delete _RC;
  return 0;

}








