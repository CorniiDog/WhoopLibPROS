/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       Slew.hpp                                                  */
/*    Author:       Connor White (WHOOP)                                      */
/*    Created:      Thu July 4 2024                                           */
/*    Description:  Slew Calculator                                           */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#ifndef SLEW_HPP
#define SLEW_HPP

/**
 * General-use Slew class for motor voltage. 
 * The default update period is 10ms or 100Hz
 */

class Slew
{
public:
    double slewed_error = 0;
    double max_increase_per_second;
    double max_increase_per_step;
    bool can_slew_down;
    int milliseconds_per_step;

public:
    /**
     * Slew Constructor
     *
     * @param error Difference in desired and current position
     * @param can_slew_down Set to false to disable slewing if slowing down. But set to true to slew downwards too
     * @param milliseconds_per_step The amount of milliseconds per step (recommended to be 10ms)
     */
    Slew(double max_increase_per_second, bool can_slew_down = false, int milliseconds_per_step=10);

    /**
     * Steps and computes
     * @param error Difference in desired and current position
     * @return Output power
     */
    double step(double error);
};

#endif // SLEW_HPP