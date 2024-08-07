/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       MainScreen.hpp                                            */
/*    Author:       Connor White (WHOOP)                                      */
/*    Created:      Thu July 25 2024                                          */
/*    Description:  WhoopLib LVGL Main Screen                                 */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "whooplib/includer.hpp"

// Images
#include "images/WhoopLibTiny.h"

#ifndef  MAIN_SCREEN
#define MAIN_SCREEN

namespace whoop{

namespace screen{

static lv_obj_t * win = nullptr;
static lv_obj_t * txt = nullptr;

static bool screen_initialized = false;

/**
 * Initializes the whoop screen
 */
void initialize();

static void create_log_window();

} // screen

} // whoop

#endif // MAIN_SCREEN