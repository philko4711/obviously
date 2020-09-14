#include "obvision/reconstruct/space/SensorPolar3DBase.h"

namespace obvious
{

class VelodyneHDL32E : public SensorPolar3DBase
{
public:
  /**
   * Standard Constructor
   */
  VelodyneHDL32E(unsigned int raysIncl = 32, double inclMin = -0.535292, double inclMax = 0.186227, double inclRes = 0.023213, double azimMin = 0.0,
                 double azimMax = 6.28319, double azimRes = 0.002897);

  /**
   * Destructor
   */
  virtual ~VelodyneHDL32E();

  int lookupIndex(int inclIndex);
};

} // namespace obvious