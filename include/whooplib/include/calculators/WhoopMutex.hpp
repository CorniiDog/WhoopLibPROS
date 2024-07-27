/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       WhoopMutex.hpp                                            */
/*    Author:       Connor White (WHOOP)                                      */
/*    Created:      Thu July 25 2024                                          */
/*    Description:  Whoop Mutex to allow PROS and VEXCode                     */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "whooplib/includer.hpp"

#ifndef WHOOP_MUTEX_H
#define WHOOP_MUTEX_H

namespace whoop{

#if USE_VEXCODE
class WhoopMutex{
public:
    vex::mutex vexcode_mutex;

    // Locks the mutex
    void lock();

    // Unlocks the mutex
    void unlock();
};

#else
class WhoopMutex{
public:
    pros::Mutex pros_mutex;

    // Locks the mutex
    void lock();

    // Unlocks the mutex
    void unlock();
};
#endif

} // namespace whoop

#endif // WHOOP_MUTEX_H