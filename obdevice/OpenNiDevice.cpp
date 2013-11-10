#include "OpenNiDevice.h"

#include <iostream>

namespace obvious
{

OpenNiDevice::OpenNiDevice(const Flag flags, const std::string& deviceURI) :
    _flags(flags)
{
  _status = openni::OpenNI::initialize();

  std::cout << "After initialization:" << std::endl
      << openni::OpenNI::getExtendedError();

  if((_status = _device.open(
      deviceURI.length() ? deviceURI.c_str() : openni::ANY_DEVICE))
      != openni::STATUS_OK)
  {
    std::cout << "Device " << deviceURI << " open failed:" << std::endl
        << openni::OpenNI::getExtendedError();
    openni::OpenNI::shutdown();
    return;
  }
  else
  {
    std::cout << "Device " << deviceURI << " opened." << std::endl;
  }

  if(_flags && Depth)
  {
    if((_status = _depth.create(_device, openni::SENSOR_DEPTH))
        == openni::STATUS_OK)
    {
      std::cout << "Found depth stream." << std::endl;

      if((_status = _depth.start()) != openni::STATUS_OK)
      {
        std::cout << "Couldn't start depth stream:" << std::endl
            << openni::OpenNI::getExtendedError();
        openni::OpenNI::shutdown();
        return;
      }

      std::cout << "Depth stream started." << std::endl;
    }
    else
    {
      std::cout << "Couldn't find depth stream:" << std::endl
          << openni::OpenNI::getExtendedError();
    }
  }

  if(_flags & Color)
  {
    if((_status = _color.create(_device, openni::SENSOR_COLOR))
        == openni::STATUS_OK)
    {
      std::cout << "Found color stream." << std::endl;

      if((_status = _color.start()) != openni::STATUS_OK)
      {
        std::cout << "Couldn't start color stream:" << std::endl
            << openni::OpenNI::getExtendedError();
        openni::OpenNI::shutdown();
        return;
      }

      std::cout << "Color stream started." << std::endl;
    }
    else
    {
      std::cout << "Couldn't find any color stream:" << std::endl
          << openni::OpenNI::getExtendedError();
    }
  }

  if(!_color.isValid() && _flags && Ir)
  {
    if((_status = _ir.create(_device, openni::SENSOR_IR))
        == openni::STATUS_OK)
    {
      std::cout << "Found ir stream." << std::endl;

      if((_status = _ir.start()) != openni::STATUS_OK)
      {
        std::cout << "Couldn't start ir stream:" << std::endl
            << openni::OpenNI::getExtendedError();
        openni::OpenNI::shutdown();
        return;
      }

      std::cout << "Ir stream started." << std::endl;
    }
    else
    {
      std::cout << "Couldn't find any ir stream:" << std::endl
          << openni::OpenNI::getExtendedError();
    }
  }

  if(!_depth.isValid() && !_color.isValid() && !_ir.isValid())
  {
    std::cout << "Device " << deviceURI << ": no valid streams." << std::endl;
    openni::OpenNI::shutdown();
    return;
  }

  _ir_image = NULL;
  _coords   = NULL;
  _rgb      = NULL;
  _width    = 0;
  _height   = 0;
  _z        = NULL;
  this->init();
}

OpenNiDevice::~OpenNiDevice(void)
{
  if(_coords)
    delete[] _coords;
  if(_z)
    delete[] _z;
  if(_rgb)
    delete[] _rgb;
  if(_ir_image)
    delete[] _ir_image;

  if(_status == openni::STATUS_OK)
    openni::OpenNI::shutdown();
}

bool
OpenNiDevice::init(void)
{
  openni::VideoMode depthVideoMode;
  openni::VideoMode colorVideoMode;
  openni::VideoMode irVideoMode;

  if(_depth.isValid() && _color.isValid() && _ir.isValid())
  {
    depthVideoMode = _depth.getVideoMode();
    colorVideoMode = _color.getVideoMode();
    irVideoMode = _ir.getVideoMode();

    const unsigned int depthWidth = depthVideoMode.getResolutionX();
    const unsigned int depthHeight = depthVideoMode.getResolutionY();
    const unsigned int colorWidth = colorVideoMode.getResolutionX();
    const unsigned int colorHeight = colorVideoMode.getResolutionY();
    const unsigned int irWidth = irVideoMode.getResolutionX();
    const unsigned int irHeight = irVideoMode.getResolutionY();

    if(depthWidth == colorWidth && colorWidth == irWidth
        && depthHeight == colorHeight && colorHeight == irHeight)
    {
      _width    = depthWidth;
      _height   = depthHeight;
      _z        = new double[_width * _height];
      _coords   = new double[_width * _height * 3];
      _rgb      = new unsigned char[_width * _height * 3];
      _ir_image = new unsigned char[_width * _height * 3];

      if(_flags == Any)
        _flags = static_cast<Flag>(Depth | Color | Ir);

      return true;
    }

    std::cout
        << "Error - expect color and depth to be in same resolution: D: "
        << depthWidth << ", " << depthHeight << ", C: " << colorWidth << ", "
        << colorHeight << std::endl;

    return false;
  }
  else if(_depth.isValid() && _ir.isValid())
  {
    depthVideoMode = _depth.getVideoMode();
    irVideoMode = _ir.getVideoMode();

    _width      = depthVideoMode.getResolutionX();
    _height     = depthVideoMode.getResolutionY();
    _z          = new double[_width * _height];
    _coords     = new double[_width * _height * 3];
    _ir_image   = new unsigned char[_width * _height * 3];

    if(_flags == Any)
      _flags = static_cast<Flag>(Depth | Ir);

    return true;
  }
  else if(_depth.isValid() && _color.isValid())
  {
    depthVideoMode = _depth.getVideoMode();
    colorVideoMode = _color.getVideoMode();

    _width  = depthVideoMode.getResolutionX();
    _height = depthVideoMode.getResolutionY();
    _z      = new double[_width * _height];
    _coords = new double[_width * _height * 3];
    _rgb    = new unsigned char[_width * _height * 3];

    if(_flags == Any)
      _flags = static_cast<Flag>(Depth | Color);

    return true;
  }
  else if(_color.isValid())
  {
    colorVideoMode = _color.getVideoMode();
    _width  = colorVideoMode.getResolutionX();
    _height = colorVideoMode.getResolutionY();

    if(_flags == Any)
      _flags = Color;
    return true;
  }

  std::cout << "Error - expects at least one of the streams to be valid..."
      << std::endl;
  return false;
}

bool
OpenNiDevice::grab(void)
{
  if(!_depth.isValid() && !_color.isValid() && !_ir.isValid())
  {
    std::cout << "No stream available." << std::endl;
    return false;
  }

  if(_depth.isValid())
  {
    _depth.readFrame(&_frameDepth);

    const openni::DepthPixel* data =
        reinterpret_cast<const openni::DepthPixel*>(_frameDepth.getData());

    unsigned int i = 0;
    for(unsigned int row = 0; row < _height; ++row) {
      for(unsigned int col = 0; col < _width; ++col)
      {
        float x, y, z;

        openni::CoordinateConverter::convertDepthToWorld(_depth, col, row,
            *data, &x, &y, &z);

        _z[i]            = *data / 1000.0;
        _coords[3*i + 0] = (double)x / 1000.0;
        _coords[3*i + 1] = (double)y / 1000.0;
        _coords[3*i + 2] = (double)z / 1000.0;

        if(i==2000)
          std::cout << _coords[3*i + 0]
             << " " << _coords[3*i + 1]
             << " " << _coords[3*i + 2] << std::endl;

        i++;
        data++;
      }
    }
  }

if(_color.isValid())
{
  _color.readFrame(&_frameColor);

  const openni::RGB888Pixel* data =
      reinterpret_cast<const openni::RGB888Pixel*>(_frameColor.getData());
  const unsigned int size = _width * _height;

  for(unsigned int i= 0; i<size; ++data)
  {
    _rgb[i+0] = data->r;
    _rgb[i+1] = data->g;
    _rgb[i+2] = data->b;
    i+=3;
  }

}

if(_ir.isValid())
{
  _ir.readFrame(&_frameIr);

  const uint16_t* data =
      reinterpret_cast<const uint16_t*>(_frameIr.getData());
  const unsigned int size = _width * _height;

  for(unsigned int i = 0; i<size*3; ++data)
  {
    _ir_image[i]    = *data >> 2;
    _ir_image[i+1]  = *data >> 2;
    _ir_image[i+2]  = *data >> 2;
    i+=3;
  }
}
  return true;
}

}  // end namespace obvious
