/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       WhoopLibTiny.h                                            */
/*    Author:       Connor White (WHOOP)                                      */
/*    Created:      Thu July 25 2024                                          */
/*    Description:  Image of WhoopLib Logo                                    */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "whooplib/includer.hpp"

/**
  How to make your own image:
  1. Upload your image to https://lvgl.io/tools/imageconverter
  2. LVGL v8
  3. Color format CF_TRUE_COLOR_ALPHA
  4. "Convert"
  5. Use this .h and the src's .c as reference for formatting
 */

#ifndef  WHOOPLIB_TINY_H
#define WHOOPLIB_TINY_H

#ifdef __cplusplus
extern "C" {
#endif

namespace whoop{

extern lv_img_dsc_t WhoopLibTiny;

}

#ifdef __cplusplus
}
#endif

#endif // WHOOPLIB_IMAGE