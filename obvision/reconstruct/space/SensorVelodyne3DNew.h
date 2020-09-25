#include "obcore/math/linalg/eigen/Matrix.h"
#include "obvision/reconstruct/Sensor.h"

namespace obvious
{

class SensorVelodyne3DNew : public Sensor
{
public:
  SensorVelodyne3DNew(unsigned int raysIncl, double inclMin, double inclMax, double inclRes, double azimMin, double azimMax, double azimRes,
                      double maxRange = INFINITY, double minRange = 0.0, double lowReflectivityRange = INFINITY);

  virtual ~SensorVelodyne3DNew();

  int lookupIndex(int inclIndex);

  void backProject(obvious::Matrix* M, int* indices, obvious::Matrix* T = NULL);

private:
  double _inclRes;
  double _inclMin;
  double _inclMax;
  // double _inclSpan;
  double _inclNegSpan;
  double _azimRes;
  double _azimMin;
  int**  _indexMap;
};
} // namespace obvious