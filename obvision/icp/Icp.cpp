#include "Icp.h"
#include <gsl/gsl_blas.h>
#include "obcore/base/tools.h"
#include "obcore/base/Timer.h"
namespace obvious
{

const char* g_icp_states[] = {"ICP_IDLE", "ICP_PROCESSING", "ICP_NOTMATCHABLE", "ICP_MAXITERATIONS", "ICP_TIMEELAPSED", "ICP_SUCCESS", "ICP_ERROR"};

Icp::Icp(PairAssignment* assigner, IRigidEstimator* estimator)
{
  _assigner  = assigner;
  _estimator = estimator;

  _maxIterations       = 3;
  _dim                 = _assigner->getDimension();
  _maxRMS              = 0.1;
  _model               = NULL;
  _scene               = NULL;
  _normalsM            = NULL;
  _normalsS            = NULL;
  _sizeModelBuf        = 0;
  _sizeSceneBuf        = 0;
  _sizeModel           = 0;
  _sizeScene           = 0;

  _Tfinal4x4           = new Matrix(4, 4);
  _Tfinal              = new Matrix(_dim+1, _dim+1);
  _Tlast               = new Matrix(4, 4);
  _Tfinal4x4->setIdentity();
  _Tlast->setIdentity();

  _convCnt = 5;

  _fptrCallbackPairs = NULL;

  this->reset();
}

Icp::~Icp()
{
  if(_model != NULL) System<double>::deallocate(_model);
  if(_scene != NULL) System<double>::deallocate(_scene);
  delete(_Tlast);
  delete(_Tfinal);
}

void Icp::setAssignmentCallback(fptrAssignmentCallback fptr)
{
  _fptrCallbackPairs = fptr;
}

const char* Icp::state2char(EnumIcpState eState)
{
  return g_icp_states[eState];
};

PairAssignment* Icp::getPairAssigner()
{
  return _assigner;
}

IRigidEstimator* Icp::getRigidEstimator()
{
  return _estimator;
}

void Icp::setModel(double* coords, double* normals, const unsigned int size)
{
  unsigned int sizeNormals = _sizeModelBuf;

  checkMemory(size, _dim, _sizeModelBuf, _model);

  memcpy(&_model[0][0], coords, size*_dim*sizeof(double));
  if(normals)
  {
    checkMemory(size, _dim, sizeNormals, _normalsM);
    memcpy(_normalsM[0], normals, size*_dim*sizeof(double));
  }

  _sizeModel = size;

  _assigner->setModel(_model, _sizeModel);
  _estimator->setModel(_model, _sizeModel, _normalsM);
}

void Icp::setModel(gsl_matrix* coords, gsl_matrix* normals)
{
  if(coords->size2!=(size_t)_dim)
  {
    cout << "WARNING: Model is not of correct dimensionality. Needed: " << _dim << endl;
    return;
  }

  unsigned int size = coords->size1;
  unsigned int sizeNormals = _sizeModelBuf;

  checkMemory(size, _dim, _sizeModelBuf, _model);

  for(unsigned int i=0; i<size; i++)
  {
    double* c = gsl_matrix_ptr(coords, i, 0);
    memcpy(_model[i], c, _dim*sizeof(double));
  }

  if(normals)
  {
    checkMemory(size, _dim, sizeNormals, _normalsM);
    for(unsigned int i=0; i<size; i++)
    {
      double* c = gsl_matrix_ptr(normals, i, 0);
      memcpy(_normalsM[i], c, _dim*sizeof(double));
    }
  }

  _sizeModel = size;

  _assigner->setModel(_model, _sizeModel);
  _estimator->setModel(_model, _sizeModel, _normalsM);
}

void Icp::setScene(double* coords, double* normals, const unsigned int size)
{
  if(size==0)
  {
    cout << "Scene of size 0 passed ... ignoring" << endl;
    return;
  }
  _sizeScene = size;

  unsigned int sizeNormals = _sizeSceneBuf;

  checkMemory(_sizeScene, _dim, _sizeSceneBuf, _scene);
  memcpy(_scene[0], coords, size*_dim*sizeof(double));

  if(normals)
  {
    checkMemory(_sizeScene, _dim, sizeNormals, _normalsS);
    memcpy(_normalsS[0], normals, size*_dim*sizeof(double));
  }

  applyTransformation(_scene, _sizeScene, _dim, _Tfinal4x4);
  if(normals) applyTransformation(_normalsS, _sizeScene, _dim, _Tfinal4x4);
}

void Icp::setScene(gsl_matrix* coords, gsl_matrix* normals)
{
  if(coords->size2!=(size_t)_dim) {
    cout << "WARNING: Scene is not of correct dimensionality " << _dim << endl;
  }

  _sizeScene = coords->size1;

  unsigned int sizeNormals = _sizeSceneBuf;

  checkMemory(_sizeScene, _dim, _sizeSceneBuf, _scene);
  for(unsigned int i=0; i<_sizeScene; i++)
  {
    double* c = gsl_matrix_ptr(coords, i, 0);
    memcpy(_scene[i], c, _dim*sizeof(double));
  }

  if(normals)
  {
    checkMemory(_sizeScene, _dim, sizeNormals, _normalsS);

    for(unsigned int i=0; i<_sizeScene; i++)
    {
      double* c = gsl_matrix_ptr(normals, i, 0);
      memcpy(_normalsS[i], c, _dim*sizeof(double));
    }
  }

  applyTransformation(_scene, _sizeScene, _dim, _Tfinal4x4);
  if(normals) applyTransformation(_normalsS, _sizeScene, _dim, _Tfinal4x4);
}

void Icp::checkMemory(unsigned int rows, unsigned int cols, unsigned int &memsize, double** &mem)
{
  // first instantiation of buffer
  if(mem == NULL)
  {
    memsize = rows;
    System<double>::allocate(rows, cols, mem);
  }
  // resize buffer, if needed
  if(rows > memsize)
  {
    memsize = rows;
    if(mem != NULL) System<double>::deallocate(mem);
    System<double>::allocate(rows, cols, mem);
  }
}

void Icp::reset()
{
  _Tfinal4x4->setIdentity();
  _assigner->reset();
}

void Icp::setMaxRMS(double rms)
{
  _maxRMS = rms;
}

double Icp::getMaxRMS()
{
  return _maxRMS;
}

void Icp::setMaxIterations(unsigned int iterations)
{
  _maxIterations = iterations;
}

unsigned int Icp::getMaxIterations()
{
  return _maxIterations;
}

void Icp::setConvergenceCounter(unsigned int convCnt)
{
  _convCnt = convCnt;
}

unsigned int Icp::getConvergenceCounter()
{
  return _convCnt;
}

void Icp::applyTransformation(double** data, unsigned int size, unsigned int dim, Matrix* T)
{
  // Apply rotation
  gsl_matrix_view points = gsl_matrix_view_array(*data, size, dim);
  gsl_matrix* points_tmp = gsl_matrix_alloc(size, dim);
  gsl_matrix_view R = gsl_matrix_submatrix(T->getBuffer(), 0, 0, dim, dim);
  gsl_blas_dgemm(CblasNoTrans, CblasTrans, 1.0, &points.matrix, &R.matrix, 0.0, points_tmp);
  gsl_matrix_memcpy(&points.matrix, points_tmp);
  gsl_matrix_free(points_tmp);

  // Add translation
  gsl_vector_view x  = gsl_matrix_column(&points.matrix, 0);
  gsl_vector_view y  = gsl_matrix_column(&points.matrix, 1);
  gsl_vector_view tr = gsl_matrix_column(T->getBuffer(), 3);
  gsl_vector_add_constant(&x.vector, gsl_vector_get(&tr.vector,0));
  gsl_vector_add_constant(&y.vector, gsl_vector_get(&tr.vector,1));

  if(_dim >= 3)
  {
    gsl_vector_view z = gsl_matrix_column(&points.matrix, 2);
    gsl_vector_add_constant(&z.vector, gsl_vector_get(&tr.vector,2));
  }
}


EnumIcpState Icp::step(double* rms, unsigned int* pairs)
{
  if(_model==NULL || _scene == NULL) return ICP_ERROR;

  vector<StrCartesianIndexPair>* pvPairs;
  _estimator->setScene(_scene, _sizeScene, _normalsS);
  _assigner->determinePairs(_scene, _sizeScene);

  pvPairs = _assigner->getPairs();
  *pairs = pvPairs->size();

  if(_fptrCallbackPairs)
  {
    double** m;
    System<double>::allocate(pvPairs->size(), _dim, m);
    double** s;
    System<double>::allocate(pvPairs->size(), _dim, s);
    for(unsigned int i=0; i<pvPairs->size(); i++)
    {
      unsigned int idxModel = ((*pvPairs)[i]).indexFirst;
      unsigned int idxScene = ((*pvPairs)[i]).indexSecond;
      for(unsigned int j=0; j<(unsigned int)_dim; j++)
      {
        m[i][j] = _model[idxModel][j];
        s[i][j] = _scene[idxScene][j];
      }
    }
    _fptrCallbackPairs(m, s, pvPairs->size());
    System<double>::deallocate(m);
    System<double>::deallocate(s);
  }

  if(pvPairs->size()>2)
  {
    // Estimate transformation
    _estimator->setPairs(pvPairs);

    // get mapping error
    *rms = _estimator->getRMS();

    // estimate transformation
    _estimator->estimateTransformation(_Tlast->getBuffer());

    applyTransformation(_scene, _sizeScene, _dim, _Tlast);
    if(_normalsS)
      applyTransformation(_normalsS, _sizeScene, _dim, _Tlast);

    // update overall transformation
    (*_Tfinal4x4) = (*_Tlast) * (*_Tfinal4x4);
  }
  else
  {
    return ICP_NOTMATCHABLE;
  }

  return ICP_PROCESSING;
}

EnumIcpState Icp::iterate(double* rms, unsigned int* pairs, unsigned int* iterations)
{
  EnumIcpState eRetval = ICP_PROCESSING;
  unsigned int iter = 0;
  double rms_prev = 10e12;
  unsigned int conv_cnt = 0;
  while( eRetval == ICP_PROCESSING )
  {
    eRetval = step(rms, pairs);
    iter++;

    if(fabs(*rms-rms_prev) < 10e-10)
      conv_cnt++;
    else
      conv_cnt = 0;
    if((*rms <= _maxRMS || conv_cnt>=_convCnt ))
      eRetval = ICP_SUCCESS;
    else if(iter >= _maxIterations)
      eRetval = ICP_MAXITERATIONS;

    rms_prev = *rms;
  }
  *iterations = iter;
  return eRetval;
}	

Matrix* Icp::getFinalTransformation4x4()
{
  return _Tfinal4x4;
}

Matrix* Icp::getFinalTransformation()
{
   for(int r=0; r<_dim; r++)
   {
      for(int c=0; c<_dim; c++)
      {
         (*_Tfinal)[r][c] = (*_Tfinal4x4)[r][c];
      }
      (*_Tfinal)[r][_dim] = (*_Tfinal4x4)[r][3];
   }

   for(int c=0; c<_dim; c++)
      (*_Tfinal)[_dim][c] = 0;

   (*_Tfinal)[_dim][_dim] = 1;

  return _Tfinal;
}

Matrix* Icp::getLastTransformation()
{
  return _Tlast;
}

void Icp::getRotationTransformVec(double* rotAngles, double* translation)
{
  _estimator->getRotationTransformVec(rotAngles, translation);
}

}
