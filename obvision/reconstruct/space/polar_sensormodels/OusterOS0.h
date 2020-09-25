#include "obvision/reconstruct/space/SensorPolar.h"

namespace obvious
{

class OusterOS0 : public SensorPolar
{
public:
  /**
   * Standard Constructor
   * @param[in] raysIncl number of inclination rays of scanning device (vertical): 128
   * @param[in] inclMin lowest inclination angle in RAD (-0.785398 = -45°)
   * @param[in] inclMax highest inclination angle in RAD (0.785398 = +45°)
   * @param[in] inclRes resolution of inclination rays in RAD (0.012217 = 0.70°), i.e. angle between two vertical rays
   * @param[in] azimMin lowest azimuth angle in RAD (0.0 = 0°)
   * @param[in] azimMax highest azimuth angle in RAD (6.28319 = 360°)
   * @param[in] azimRes resolution of azimuth rays in RAD (0.0061087 = 0.35°), angle between two horizontal rays in 360° plane, configurable 512 --> 0.70°,
   * 1024 --> 0.35°, 2048 --> 0.18°
   * @todo check with techsupport if backprojection really isn't necessary like they said "There is no no need to do back projection, the PointOS1 type
   * already includes a ring parameter that indicates which laser (i.e. which ring) it came from."
   */
  OusterOS0(unsigned int raysIncl, double inclMin, double inclMax, double inclRes, double azimMin, double azimMax, double azimRes);

  /**
   * Destructor
   */
  virtual ~OusterOS0();

  /**
   *
   * */
  int lookupIndex(int inclIndex);
};

} // namespace obvious