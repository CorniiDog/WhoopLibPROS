/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       WhoopMutex.cpp                                            */
/*    Author:       Connor White (WHOOP)                                      */
/*    Created:      Thu July 25 2024                                          */
/*    Description:  Whoop Mutex to allow PROS and VEXCode                     */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "whooplib/include/calculators/WhoopMutex.hpp"

#if USE_VEXCODE

void WhoopMutex::lock(){
    vexcode_mutex.lock();
}

void WhoopMutex::unlock(){
    vexcode_mutex.unlock();
}
#else

void WhoopMutex::lock(){
    pros_mutex.take();
}

void WhoopMutex::unlock(){
    pros_mutex.give();
}

#endif