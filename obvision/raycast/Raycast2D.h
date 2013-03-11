/**
* @file   Raycast2D.h
* @author christian
* @date   30.01.2013
*
*
*/

#ifndef _RAYCAST2D_
#define _RAYCAST2D_

#include "obcore/grid/ObstacleGrid.h"
/**
 * @namespace obvious
 */
namespace obvious {
/**
 * @class Raycast2D
 */
class Raycast2D
{
public:
  /**
   * Standard constructor
   */
  Raycast2D(void);
  /**
   * Default destructor
   */
  ~Raycast2D(void);
  /**
   * Function to set grid as input
   * @param[in]   grid    input grid
   */
  void setInput(ObstacleGrid* grid)    { _grid = grid; }
  /**
   * Function to estimate free space in grid
   * @param       clear       true to clear out free space (default:=TRUE)
   */
  void estimateFreeSpace(bool clear = true);
private:
  /**
   * Function to send rays from minimal to maximal angle
   * @param[in]   minAngle    minimal angle in radiant
   * @param[in]   maxAngle    maximum angle in radiant
   * @return      true if success
   */
  inline bool castRays(const double& minAngle, const double& maxAngle, const double& step);
  /**
   * Function to send one single ray through grid
   * @param[in]   angle       angle in radiant
   * @return      distance in meters
   */
  inline double castSingleRay(double angle);

  ObstacleGrid* _grid;
  bool          _clearFree;
};
}; // namespace

#endif

