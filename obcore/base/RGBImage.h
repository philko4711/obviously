/**
* @file   RGBImage.h
* @author Christian Pfitzner
* @date   29.01.2013
*
*
*/

#ifndef RGBIMAGE_H_
#define RGBIMAGE_H_

#include "obcore/base/Image.h"
#include "obcore/math/MatRGB.h"
#include "obcore/rgbColor.h"

/**
 * @namespace obvious
 */
namespace obvious{
/**
 * @class RGBImage
 * Container class derived from image to hold image and image specific data like width and height
 */
class RGBImage : public Image
{
public:
  /**
   * Standard constructor
   * @param   width   width of image in pixels
   * @param   height  height of image in pixels
   */
  RGBImage(unsigned int width, unsigned int height)
    : Image(width, height, Image::COLORED) { }
  /**
   * Function to return one pixel in unsigned char array
   * @param   col     column
   * @param   row     row
   * @return  unsigned char with the size 3 for red, green and blue
   */
  const unsigned char* at(const unsigned int col, const unsigned int row) const
  {
    unsigned char* rgb = new unsigned char[3];
    for(unsigned int i=RGBColor::RED ; i<=RGBColor::BLUE ; i++)
      rgb[i] = _img[3*(col*_width + row) + i];
    return(rgb);
  }
  /**
   * Function to return
   * @param   col     column
   * @param   row     row
   * @param   color   which color (please see @see PixelEnum)
   * @return  reference on pixel
   */
  unsigned char& at(const unsigned int col, const unsigned int row,unsigned int color)
  {
    return(_img[3*(col*_width + row) + color]);
  }

private:

};
}; // namespace



#endif /* RGBIMAGE_H_ */
