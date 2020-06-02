#include "SensorPolar3DBase.h"
#include "obcore/math/mathbase.h"


namespace obvious
{

SensorPolar3DBase::SensorPolar3DBase(double inclMin, double inclMax, double inclRes, double azimMin, double azimMax, 
double azimRes, double maxRange, double minRange, double lowReflectivityRange)
: Sensor(3, maxRange, minRange, lowReflectivityRange)
{
    
    ///////////ACHTUNG ICH KRIEG HIER EIN PROBLEM! raysINCL IST HIER 15 und nicht 16!!!!!!!!!!!!!!!!!!!!!!!!!!
    //!!!!!!!!!!!!!!!!!!!!! ich mach jetzt bei beiden +1 weil ich nicht nur 15 raysIncl hab sondern 16 und 
    //!!! weil bei azimuth ich den 0° und 360° ray haben will? idk. testen
    unsigned int raysIncl = round(static_cast<unsigned>((abs(inclMin) + abs(inclMax)) / inclRes)) + 1;
    unsigned int raysAzim = round(static_cast<unsigned>(2 * M_PI / azimRes)) + 1;

    _azimRes = azimRes;
    _inclRes = inclRes;

    // inherited from class Sensor
    _width = raysAzim;       
    _height = raysIncl;
    _size = _width * _height;
    
    _data = new double[_size];
    _mask = new bool[_size];
    for(unsigned int i = 0; i < _size; i++)
        _mask[i] = true;

    _rays = new obvious::Matrix(3, _size);
    _raysLocal = new obvious::Matrix(3, _size);
    *_raysLocal = *_rays;

    unsigned int n = 0; //count all iterations of inner and outer loop for storing vals in _rays
    double currIncl = inclMin;  //incremented by inclRes in each loop
    double currAzim = 0.0;      //azimuth start angle incremented by azimRes in each loop

    //!!!!!!!!!!!!!!!!!!!! < oder <= ?
    for(unsigned int i = 0; i < _width; i++)
    {
        for(unsigned int j = 0; j < _height; j++, n++)
        {
            obvious::Matrix calcRay(3, 1);

            //to follow inclination angle theta in spherical coordinate system (from y-axis down)
            double theta = 0.0;
            const double piHalf = deg2rad(90.0);

            if(currIncl < 0)
            {
                theta = piHalf + currIncl * (-1);
            }
            else
            {
                theta = piHalf - currIncl;
            }

            calcRay(0, 0) = sin(theta) * cos(currAzim);
            calcRay(1, 0) = sin(theta) * sin(currAzim);
            calcRay(2, 0) = cos(theta);
            
        }
    }

}

SensorPolar3DBase::~SensorPolar3DBase()
{
    std::cout << "destruction baby" << std::endl;
}

// M sind die Koordinaten des TSD SPACES! von allen VOXELN die Mittelpunkte!
void SensorPolar3DBase::backProject(obvious::Matrix* M, int* indices, obvious::Matrix* T)
{

}

} // namespace obvious