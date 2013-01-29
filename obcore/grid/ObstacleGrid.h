/**
* @file   ObstacleGrid.h
* @author Christian Pfitzner
* @date   04.01.2013
*
*
*/

#ifndef OBSTACLEGRID_H_
#define OBSTACLEGRID_H_

#include "obcore/grid/Grid2D.h"
#include "obcore/grid/HeightGrid.h"
#include "obcore/grid/GradientGrid.h"
/**
 * @namespace obvious
 */
namespace obvious{
/**
 * @class ObstacleGrid
 * @brief Grid to detect obstacles
 *
 * The reduction of point clouds to a grid makes following algorithms easier
 * to detect obstacles. This is why the class is set up. The class is derived
 * from Grid2D
 * @see Grid2D
 */
class ObstacleGrid : public Grid2D
{
public:
  /**
   * Default constructor
   * @param[in]   resolution    length of one grid cell
   * @param[in]   length        length of grid
   * @param[in]   width         width of grid
   */
  ObstacleGrid(double resolution = 0.05, double length = 2.005, double width = 2.005);
  /**
   * Default destructor
   */
  virtual ~ObstacleGrid(void);
  /**
   * Function to get height map
   * @param[in]     cloud   cloud with points in double array
   * @param[in]     size    number of points in cloud
   * @return        TRUE if no error occurs @see SUCCESFUL
   */
  SUCCESFUL height2Grid(double* cloud, unsigned int size);
  /**
   * Function to return estimate obstacles out of grids
   * @return    true if everything went allright
   */
  bool getObstacles(void);
  /**
   * Function to return image of grid
   */
  virtual unsigned char* getImageOfGrid(void);
  /**
   * Function to return nearest obstacle to center
   * @param     x   coordinate for x in meters
   * @param     y   coordinate for y in meters
   * @return    TRUE if obstacle found
   */
  bool getNearestObstacle(double& x, double& y) const;
  /**
   * Function to set threshold for heigth grid
   * @param[in]   heightTH    threshold in meters for height grid
   */
  void setThresholdStepHeight(const double& heightTH) { _heightTH = heightTH; }
  /**
   * Function to set threshold for gradient grid
   * @param[in]   gradientTH  theshold in rad for gradient grid
   */
  void setThresholdGradient(const double& gradientTH) { _gradientTH = gradientTH; }
  /**
   * Function to set threshold for roughness grid
   * @param[in]   roughTH     @todo has to be defined
   */
  void setThresholdRough(const double& roughTH)       { _roughTH  = roughTH; }
private:

  unsigned char* getObstacleMap(void);
  /**
   * @enum CHANNEL
   */
  enum CHANNEL {
    HEIGHT = 1,       //!< HEIGHT
    GRADIENT_X = 2,   //!< GRADIENT_X
    GRADIENT_Y = 3    //!< GRADIENT_Y
  };

  //!< @param   OBSTACLE_COLOR    255
  static const unsigned char OBSTACLE_COLOR = 255;

  HeightGrid*           _hGrid;
  GradientGrid*         _gGrid;
  unsigned int          _obstaclesInGrid;       //!< number of obstacles in grid
  double                _heightTH;              //!< threshold for step height map
  double                _gradientTH;            //!< threshold for gradient map
  double                _roughTH;               //!< threshold for roughtness
};
} // namespace

#endif /* OBSTACLEGRID_H_ */
