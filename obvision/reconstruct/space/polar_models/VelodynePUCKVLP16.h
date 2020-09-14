#include "obvision/reconstruct/space/SensorPolar3DBase.h"

namespace obvious
{

class VelodynePUCKVLP16 : public SensorPolar3DBase
{
public:
  /**
   * Standard Constructor
   */
  VelodynePUCKVLP16(unsigned int raysIncl = 16, double inclMin = -0.26180, double inclMax = 0.261799, double inclRes = 0.034907, double azimMin = 0.0,
                    double azimMax = 6.28319, double azimRes = 0.00349066);

  /**
   * Destructor
   */
  virtual ~VelodynePUCKVLP16();

  int lookupIndex(int inclIndex);
};

} // namespace obvious