#include "RayCastProjective3D.h"

#include <string.h>

#include "obcore/base/System.h"
#include "obcore/math/mathbase.h"
#include "obcore/base/Timer.h"
#include "obcore/base/Logger.h"

namespace obvious
{

RayCastProjective3D::RayCastProjective3D(const unsigned int cols, const unsigned int rows, SensorProjective3D* sensor, TsdSpace* space) : RayCast3D(space)
{
  _cols = cols;
  _rows = rows;

  _space = space;
  _sensor = sensor;

  System<Matrix*>::allocate(_cols, _rows, _rays);
  for(unsigned int col=0; col<_cols; col++)
    for(unsigned int row=0; row<_rows; row++)
    {
      _rays[col][row] = new Matrix(4, 1);
      _sensor->project2Space(col, row, 1.0, _rays[col][row]);

      // Normalize ray to size of voxel
      Matrix* M = _rays[col][row];
      double len = sqrt((*M)[0][0]*(*M)[0][0] + (*M)[1][0]*(*M)[1][0] + (*M)[2][0]*(*M)[2][0]);
      len /= _space->getVoxelSize();
      (*M)[0][0] /= len;
      (*M)[1][0] /= len;
      (*M)[2][0] /= len;
      (*M)[3][0] = 0.0;
    }
}

RayCastProjective3D::~RayCastProjective3D()
{
  for(unsigned int col=0; col<_cols; col++)
    for(unsigned int row=0; row<_rows; row++)
      delete _rays[col][row];
  System<Matrix*>::deallocate(_rays);
}

void RayCastProjective3D::calcCoordsFromCurrentView(double* coords, double* normals, unsigned char* rgb, unsigned int* ctr, unsigned int subsampling)
{
  Timer t;
  *ctr = 0;

  Matrix* T = _sensor->getPose();
  Matrix Tinv(4, 4);
  Tinv = T->getInverse();

#pragma omp parallel
  {
    double depth = 0.0;
    double          c[3];
    double          n[3];
    unsigned char color[3];
    double* c_tmp             = new double[_rows*_cols*3];
    double* n_tmp             = new double[_rows*_cols*3];
    unsigned char* color_tmp = new unsigned char[_rows*_cols*3];
    unsigned int cnt_tmp     = 0;
    Matrix M(4,1);
    Matrix N(4,1);
    M[3][0] = 1.0;
    N[3][0] = 0.0; // no translation for normals

#pragma omp for schedule(dynamic)
    for (unsigned int row = 0; row < _rows; row+=subsampling)
    {
      for (unsigned int col = 0; col < _cols; col+=subsampling)
      {
        if (rayCastFromCurrentView(row, col, c, n, color, &depth)) // Ray returned with coordinates
        {
          M[0][0] = c[0];
          M[1][0] = c[1];
          M[2][0] = c[2];
          N[0][0] = n[0];
          N[1][0] = n[1];
          N[2][0] = n[2];
          M       = Tinv * M;
          N       = Tinv * N;
          for (unsigned int i = 0; i < 3; i++)
          {
            c_tmp[cnt_tmp]      = M[i][0];
            color_tmp[cnt_tmp]  = color[i];
            n_tmp[cnt_tmp++]    = N[i][0];
          }
        }
      }
    }
#pragma omp critical
    {
      memcpy(&coords[*ctr],  c_tmp,     cnt_tmp*sizeof(double));
      memcpy(&normals[*ctr], n_tmp,     cnt_tmp*sizeof(double));
      memcpy(&rgb[*ctr],     color_tmp, cnt_tmp*sizeof(unsigned char));
      *ctr += cnt_tmp;
    }
    delete[] c_tmp;
    delete[] n_tmp;
    delete[] color_tmp;
  }

  LOGMSG(DBG_DEBUG, "Elapsed TSDF projection: " << t.getTime() << "ms");
  LOGMSG(DBG_DEBUG, "Raycasting finished! Found " << *ctr << " coordinates");
}


bool RayCastProjective3D::rayCastFromCurrentView(const unsigned int row, const unsigned int col, double coordinates[3], double normal[3], unsigned char rgb[3], double* depth)
{
  double tr[3];
  _sensor->getPosition(tr);

  double dirVec[3];
  double position[3];
  double position_prev[3];

  calcRayFromCurrentView(row, col, dirVec);

  int xDim = _space->getXDimension();
  int yDim = _space->getYDimension();
  int zDim = _space->getZDimension();
  double voxelSize = _space->getVoxelSize();

  // Interpolation weight
  double interp;

  double xmin   = ((double)(dirVec[0] > 0.0 ? 0 : (xDim-1)*voxelSize) - tr[0]) / dirVec[0];
  double ymin   = ((double)(dirVec[1] > 0.0 ? 0 : (yDim-1)*voxelSize) - tr[1]) / dirVec[1];
  double zmin   = ((double)(dirVec[2] > 0.0 ? 0 : (zDim-1)*voxelSize) - tr[2]) / dirVec[2];
  double idxMin = max(max(xmin, ymin), zmin);
  idxMin        = max(idxMin, 0.0);

  double xmax   = ((double)(dirVec[0] > 0.0 ? (xDim-1)*voxelSize : 0) - tr[0]) / dirVec[0];
  double ymax   = ((double)(dirVec[1] > 0.0 ? (yDim-1)*voxelSize : 0) - tr[1]) / dirVec[1];
  double zmax   = ((double)(dirVec[2] > 0.0 ? (zDim-1)*voxelSize : 0) - tr[2]) / dirVec[2];
  double idxMax = min(min(xmax, ymax), zmax);

  if (idxMin >= idxMax)
    return false;

  double tsdf_prev;
  position[0] = tr[0] + idxMin * dirVec[0];
  position[1] = tr[1] + idxMin * dirVec[1];
  position[2] = tr[2] + idxMin * dirVec[2];
  _space->interpolateTrilinear(position, &tsdf_prev);

  bool found = false;
  for(int i=idxMin; i<idxMax; i++)
  {
    // calculate current position
    memcpy(position_prev, position, 3 * sizeof(*position));

    position[0] += dirVec[0];
    position[1] += dirVec[1];
    position[2] += dirVec[2];

    double tsdf;
    if (!_space->interpolateTrilinear(position, &tsdf))
      continue;

    // check sign change
    if(tsdf_prev > 0 && tsdf_prev < 0.99999 && tsdf < 0)
    {
      interp = tsdf_prev / (tsdf_prev - tsdf);
      if(_sensor->hasRealMeasurmentRGB()) _space->interpolateTrilinearRGB(position, rgb);
      found = true;
      break;
    }

    tsdf_prev = tsdf;
  }

  if(!found) return false;

  // interpolate between voxels when sign change happened
  for (unsigned int i = 0; i < 3; i++)
    coordinates[i] = position_prev[i] + dirVec[i] * interp;

  if(!_space->interpolateNormal(coordinates, normal))
    return false;

  return true;
}

void RayCastProjective3D::calcRayFromCurrentView(const unsigned int row, const unsigned int col, double dirVec[3])
{
  Matrix* T = _sensor->getPose();
  Matrix ray(4, 1);

  // bring peakpoint in map coordinate system
  ray = *_rays[col][row];
  ray = *T * ray;

  dirVec[0] = ray[0][0];
  dirVec[1] = ray[1][0];
  dirVec[2] = ray[2][0];
}

}