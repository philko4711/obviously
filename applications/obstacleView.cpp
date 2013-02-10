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

using namespace std;
using namespace obvious;


int main(int argc, char* argv[])
{
  Xtion         *_xtion         = new Xtion(argv[1]);
  ObstacleGrid  *_G             = new ObstacleGrid(/*0.015625*/0.04, 8.0, 8.0);
  Obvious2DMap  *_viewer        = new Obvious2DMap(800, 800, "Obstacle streaming", 8.0, 8.0);
  Image         *_img           = new Image(_G->getCols(), _G->getRows(), Image::COLORED);
  NormalsEstimator* _nestimator = new NormalsEstimator();
  unsigned int size            = _xtion->getRows()*_xtion->getCols();
  double*       _normals       = new double[size*3];
  Raycast2D*    _RC              = new Raycast2D();

  //_viewer->showGrid();
  _viewer->showCircle();
  _viewer->showAngles();

  _G->setThresholdGradient(0.5);
  double x, y;
  while(1)
  {
    if(_xtion->grab())
    {
      unsigned int cols = _xtion->getCols();
      unsigned int rows = _xtion->getRows();
      double* coords    = _xtion->getCoords();
      bool*   mask      = _xtion->getMask();

      _nestimator->estimateNormals3DGrid(cols, rows, coords, mask, _normals);

//      for (unsigned int i=100 ; i< 130 ; i++)
//        std::cout << _normals[i] << std::endl;

      _G->normals2Grid(_xtion->getCoords(), _normals, mask, size*3);
      _G->height2Grid(_xtion->getCoords(), size*3);
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








