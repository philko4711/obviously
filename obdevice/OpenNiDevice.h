#ifndef __OPEN_NI_DEVICE__
#define __OPEN_NI_DEVICE__

#include <OpenNI.h>

#include <string>
#include <vector>

namespace obvious {

class OpenNiDevice
{
public:

    enum Flag {
        Any   = 0xff,
        Depth = 1,
        Color = 2,
        Ir    = 4,
        All   = Depth | Color | Ir
    };

    OpenNiDevice(const Flag flags = Any, const std::string& deviceURI = std::string());
    ~OpenNiDevice(void);

    Flag flags(void) const { return _flags; }
    bool init(void);
    bool grab(void);
    unsigned int getCols(void) const    { return _width; }
    unsigned int getRows(void) const   { return _height; }
    double* getCoords(void) const     { return _coords; }
    double* getZ(void) const           { return _z; }
    unsigned char* getRGB(void) const { return _rgb; }
    unsigned char* getIR(void) const  { return _ir_image; }
//    const std::vector<float>& z(void) const { return _z; }
//    const std::vector<float>& coords(void) const { return _coords; }
//    const MatRGB& image(void) const { return _flags & Color ? _imgRgb : _imgIr; }
//    const MatRGB& ir(void) const { return _imgIr; }
//    const MatRGB& rgb(void) const { return _imgRgb; }

private:
    openni::Status        _status;
    openni::Device        _device;
    openni::VideoStream   _depth;
    openni::VideoStream   _color;
    openni::VideoStream   _ir;
    openni::VideoFrameRef _frameDepth;
    openni::VideoFrameRef _frameColor;
    openni::VideoFrameRef _frameIr;

    Flag _flags;
    unsigned int _width;
    unsigned int _height;
//    std::vector<float> _z;
//    std::vector<float> _coords;
    double*           _coords;
    double*           _z;
    unsigned char*   _rgb;
    unsigned char*   _ir_image;
//    MatRGB _imgRgb;
//    MatRGB _imgIr;
};

} // end namespace obvious

#endif
