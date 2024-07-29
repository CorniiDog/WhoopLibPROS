/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       WhoopMutex.cpp                                            */
/*    Author:       Connor White (WHOOP)                                      */
/*    Created:      Thu July 25 2024                                          */
/*    Description:  Whoop Mutex to allow PROS and VEXCode                     */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "whooplib/include/devices/WhoopMutex.hpp"

namespace whoop {

#if USE_VEXCODE

void WhoopMutex::lock() { vex::mutex::lock(); }

void WhoopMutex::unlock() { vex::mutex::unlock(); }

#else

void WhoopMutex::lock() { pros::Mutex::take(); }

void WhoopMutex::unlock() { pros::Mutex::give(); }
#endif

} // namespace whoop
