

/**
* @file   LaserTSDNode.cpp
* @author christian
* @date   24.09.2013
*
*
*/

#include "obvision/reconstruct/TsdSensorFusion.h"

#include "obcore/base/Timer.h"
#include "obcore/base/Logger.h"
#include "obcore/base/tools.h"
#include "obcore/math/mathbase.h"
#include "obgraphic/Obvious3D.h"
#include "obdevice/CamNano.h"
#include "obdevice/Xtion.h"

using namespace obvious;
using namespace std;

Obvious3D* _viewer    = NULL;
CamNano*   _nano      = NULL;
Xtion*     _xtion     = NULL;

VtkCloud*                        _cloud        = NULL;
VtkCloud*                        _cloudScene   = NULL;
VtkCloud*                        _liveSensor   = NULL;
VtkCloud*                        _xtion_live   = NULL;
obvious::Sensor*                 _sensor       = NULL;
obvious::Sensor*                 _sensor2      = NULL;
obvious::TsdSpace*               _space        = NULL;
obvious::Icp*                    _icp          = NULL;
obvious::OutOfBoundsFilter3D*    _filterBounds = NULL;
obvious::TsdSensorFusion*        _sensor_fusion = NULL;

bool _push;
bool _reg;
bool _showSpace;
bool _contiRegister;

void calc(void);

class vtkTimerCallback : public vtkCommand
{
public:
  static vtkTimerCallback *New()
  {
    vtkTimerCallback *cb = new vtkTimerCallback;
    return cb;
  }

  virtual void Execute(vtkObject *vtkNotUsed(caller), unsigned long eventId,  void *vtkNotUsed(callData))
  {
    if(_nano->grab())
    {
      unsigned int cols = _nano->getCols();
      unsigned int rows = _nano->getRows();
      double* coords  = _nano->getCoords();

      // Convert to Euklidean distances
      double* dist = new double[cols*rows];
      for(unsigned int i=0; i<cols*rows; i++)
        dist[i] = abs3D(&coords[3*i]);

      // flip y coords for correct visualization in vtk
      for(unsigned int i=0 ; i<cols*rows*3 ; i+=3)
      {
        coords[i]   = -coords[i];
        coords[i+1] = -coords[i+1];
      }

      unsigned char* colorArr = new unsigned char[_nano->getValidSize()*3];
      for (unsigned int i=0 ; i<_nano->getValidSize()*3 ; i+=3) {
        colorArr[i] = 0;  colorArr[i+1] = 255;  colorArr[i+2] = 0;
      }
      _liveSensor->setCoords(_nano->getValidCoords(), _nano->getValidSize(), 3);
      _liveSensor->setColors(colorArr, _nano->getValidSize()*3, 3);

      double P[16];
      _sensor->getPose()->getData(P);
      _liveSensor->transform(P);
      _viewer->update();

      calc();
      delete [] colorArr;
      delete [] dist;
    }
    else
    {
      LOGMSG(DBG_WARN, "Can't grab Kinect.");
    }
  }

private:

};

void _newReg(void)
{
  std::cout << __PRETTY_FUNCTION__ << std::endl;
  _reg = true;
}


void _newPush(void)
{
  std::cout << __PRETTY_FUNCTION__ << std::endl;
  _push = true;
}

void _clearSpace(void)
{
  std::cout << "Space cleared." << std::endl;
  _space->reset();
}

void _saveTsdToTmp(void)
{
  _space->serialize("/tmp/tsd_save");
}

void _contiReg(void)
{
  if(_contiRegister)
    _contiRegister = false;
  else
    _contiRegister = true;
}

void init(void)
{
  const double volumeSizeX = 0.5;
  const double volumeSizeY = 0.5;
  const double volumeSizeZ = 0.5;
  const double voxelSize   = 0.001;
  double d_maxIterations   = 50;
  double d_subSampling     = 31;

  const unsigned int maxIterations = (unsigned int)(d_maxIterations);
  const unsigned int subSampling   = (unsigned int)(d_subSampling);

  // TSD Space configuration
  _space = new TsdSpace(volumeSizeX, volumeSizeY, volumeSizeZ, voxelSize);
  _space->setMaxTruncation(2.0*voxelSize);

  // configure sensor
  double Pdata[12] = {90.1, 0.0,  82.0, 0.0,
                      0.0,   90.1, 59.5, 0.0,
                      0.0,   0.0,  1.0,  0.0};
  _sensor  = new SensorProjective3D(_nano->getCols(), _nano->getRows(), Pdata, _space->getVoxelSize());
  _sensor2 = new SensorProjective3D(_xtion->getCols(), _xtion->getRows(), Pdata, _space->getVoxelSize());

  // translation of sensor
  double tx = volumeSizeX/2.0;
  double ty = volumeSizeY/2.0;
  double tz = volumeSizeZ/3.0;
  double tf[16]={1, 0, 0, tx,
                  0, 1, 0, ty,
                  0, 0, 1, tz,
                  0, 0, 0, 1};
  Matrix T(4, 4);
  T.setData(tf);
  _sensor->transform(&T);
  _sensor2->transform(&T);

  // ICP configuration
  PairAssignment*  assigner  = (PairAssignment*) new FlannPairAssignment(3, 0.0, true);
  IRigidEstimator* estimator = (IRigidEstimator*)new PointToPlaneEstimator3D();

  // Out-of-Bounds filter to remove measurements outside TSD space
  _filterBounds = new OutOfBoundsFilter3D(_space->getMinX(), _space->getMaxX(),
                                          _space->getMinY(), _space->getMaxY(),
                                          _space->getMinZ(), _space->getMaxZ());
  _filterBounds->setPose(&T);
  assigner->addPreFilter(_filterBounds);

  // Subsampling filter for accelartion of icp
  IPreAssignmentFilter* filterS = (IPreAssignmentFilter*) new SubsamplingFilter(subSampling);
  assigner->addPreFilter(filterS);

  // Decreasing threshhold filter
  IPostAssignmentFilter* filterD = (IPostAssignmentFilter*)new DistanceFilter(0.10, 0.01, maxIterations);
  assigner->addPostFilter(filterD);

  // Deactivate early termination
  _icp = new Icp(assigner, estimator);
  _icp->setMaxRMS(0.00);
  _icp->setMaxIterations(maxIterations);
  _icp->setConvergenceCounter(maxIterations);

  unsigned int id_camNano, id_xtion;
  _sensor_fusion = new TsdSensorFusion(_space, _icp, _filterBounds);
  _sensor_fusion->addSensor(_sensor,  &id_camNano);
//  _sensor_fusion->addSensor(_sensor2, &id_xtion);
  _sensor_fusion->switchSensor(id_camNano);

  // configure viewer
  _viewer->addCloud(_cloud      = new VtkCloud());
  _viewer->addCloud(_cloudScene = new VtkCloud());
  _viewer->addCloud(_cloud      = new VtkCloud());
  _cloudScene = new VtkCloud();
  _viewer->addCloud(_xtion_live = new VtkCloud());
  _viewer->addCloud(_liveSensor = new VtkCloud());

  double P[16];
  _sensor->getPose()->getData(P);
  _viewer->showSensorPose(P);
  _sensor2->getPose()->getData(P);
  _viewer->showSensorPose(P);

  _viewer->addAxisAlignedCube(0, volumeSizeX, 0, volumeSizeY, 0, volumeSizeZ);
  _viewer->showAxes();

  // initialize flags
  _showSpace = false;
  _reg       = false;
  _push      = false;
  _contiRegister  = false;
}

void calc(void)
{
  obvious::Timer t;
  double P[16];

  // in case of new push or registration
  if(_push || _reg || _contiRegister)
  {
    _push = false;
    _nano->grab();
    _sensor_fusion->pushCurrentSensor(_nano->getDistImage(),
                                      _nano->getMask());
    _sensor_fusion->getPointCloudFromSensorView(_cloud);

    // registration
    if (_reg || _contiRegister)
    {
      _reg = false;
      unsigned int size = _nano->getValidSize();
      double* coords     = _nano->getValidCoords();

      // flip y coords for correct visualization in vtk
      for(unsigned int i=0 ; i<size*3 ; i+=3) {
        coords[i]   = -coords[i];
        coords[i+1] = -coords[i+1];
      }

      _sensor_fusion->localiseCurrentSensor(coords, size, _nano->getMask());

      LOGMSG(DBG_DEBUG, "Current Transformation: ");
      _sensor_fusion->getCurrentSensor()->getPose()->print();
      _filterBounds->setPose(_sensor_fusion->getCurrentSensor()->getPose());

      _cloudScene->setCoords(coords, size, 3);
      _sensor_fusion->getCurrentSensor()->getPose()->getData(P);
      _cloudScene->transform(P);
//      _cloudScene->serialize("scene.vtp");
      _viewer->showSensorPose(P);
      }
    }
  _viewer->update();
}


int main(int argc, char* argv[])
{
  double backcolor[3]={0.3, 0.3, 0.3};
  _viewer = new Obvious3D((char*) "Hokuyo TSD space", 1024, 768, 0, 0, backcolor);
  vtkSmartPointer<vtkTimerCallback> cb =  vtkSmartPointer<vtkTimerCallback>::New();
  vtkSmartPointer<vtkRenderWindowInteractor> interactor = _viewer->getWindowInteractor();
  interactor->AddObserver(vtkCommand::TimerEvent, cb);
  interactor->CreateRepeatingTimer(50);

  _nano  = new CamNano();
  _xtion = new Xtion(argv[1]);
  _nano->setIntegrationTime(400);

  init();
  _viewer->registerKeyboardCallback("space", _newPush);
  _viewer->registerKeyboardCallback("m",     _newReg);
  _viewer->registerKeyboardCallback("c",     _saveTsdToTmp);
  _viewer->registerKeyboardCallback("x",     _clearSpace);
  _viewer->registerKeyboardCallback("z",     _contiReg);

  std::cout << "#####################################" << std::endl;
  std::cout << "# space     ->      new push         " << std::endl;
  std::cout << "# m         ->      new registration " << std::endl;
  std::cout << "# x         ->      clear space      " << std::endl;
  std::cout << "#####################################" << std::endl;

  _viewer->startRendering();

  delete    _sensor;
  delete    _sensor2;
  delete    _space;
  delete    _icp;
  delete    _filterBounds;
  delete    _viewer;
}
