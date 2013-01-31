#include "obcore/grid/Grid2D.h"
#include "obvision/raycast/Raycast2D.h"
#include "obgraphic/Obvious2DMap.h"
#include "obcore/base/Image.h"

using namespace obvious;

int main(void)
{
  ObstacleGrid*        grid = new ObstacleGrid(0.08, 8.0, 8.0);
  Raycast2D*             RC = new Raycast2D();
  Obvious2DMap*     _viewer = new Obvious2DMap(250, 250, "Obstacle streaming", 8.0, 8.0);
  Image*               _img = new Image(grid->getCols(), grid->getRows(), Image::COLORED);

  double coord[3] = {2.1, 2.1, 2.1};
  unsigned int size = 1;
  grid->height2Grid(coord, size*3);

  RC->setInput(grid);
  RC->estimateFreeSpace(true);
  grid->getObstacles();
  _img->setImg(grid->getImageOfGrid());


  while(1)
  {
    _viewer->draw(_img->getImg(), _img->getWidth(), _img->getHeight(), _img->getType());
  }

  return(0);
}
