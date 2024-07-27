/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       WhoopMutex.cpp                                            */
/*    Author:       Connor White (WHOOP)                                      */
/*    Created:      Thu July 25 2024                                          */
/*    Description:  Whoop Mutex to allow PROS and VEXCode                     */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "whooplib/include/calculators/WhoopMutex.hpp"

namespace whoop{

/**
 * This locks the mutex
 */
void WhoopMutex::lock(){
    #if USE_VEXCODE
    vexcode_mutex.lock();
    #else
    pros_mutex.take();
    #endif
}

/**
 * This unlocks the mutex
 */
void WhoopMutex::unlock(){
    #if USE_VEXCODE
    vexcode_mutex.unlock();
    #else
    pros_mutex.give();
    #endif
}

} // namespace whoop
