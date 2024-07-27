/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       Slew.cpp                                                   */
/*    Author:       2775Josh, Modified by Connor White (WHOOP)                */
/*    Created:      Thu July 4 2024                                           */
/*    Description:  Whoop PID Calculator                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "whooplib/include/calculators/Slew.hpp"
#include <cmath>

namespace whoop{

Slew::Slew(double max_increase_per_second, bool can_slew_down, int milliseconds_per_step) : max_increase_per_second(max_increase_per_second), can_slew_down(can_slew_down), milliseconds_per_step(milliseconds_per_step)
{
  max_increase_per_step = max_increase_per_second * milliseconds_per_step / 1000.0;
}

// If we assume slewed error is 10, and error is 9, difference would be -1 (positive error and negative difference is slowing down)
// If we assume slewed error is -10, and error is -9, difference would be +1 (negative error and positive difference is slowing down)
// If we assume slewed error is 9, and error is 10, difference would be 1 (positive error and positive difference is speeding up)
// If we assume slewed error is -9, and error is -10, difference would be -1 (negative error and negative difference is speeding up)

double Slew::step(double error)
{
  double difference = error - slewed_error;

  // If error crossed the zero and cannot slew down, then set slewed_error = 0
  if (((error > 0 && slewed_error < 0) || (error < 0 && slewed_error > 0)) && !can_slew_down)
  {
    slewed_error = 0; // Adjust slewed_error by the maximum allowable step in the correct direction.
    difference = error;
  }

  // Determine the direction of the error adjustment using the sign of the difference.
  double sign = difference > 0 ? 1.0 : -1.0;

  // Check if the absolute value of the difference is less than the maximum increase per step.
  if (std::abs(difference) < max_increase_per_step)
  {
    slewed_error = error; // If so, directly set slewed_error to the new error.
  }
  else
  {
    // If speeding up or if
    if ((error > 0 && difference > 0) || (error < 0 && difference < 0) || can_slew_down)
    {
      slewed_error += sign * max_increase_per_step; // Adjust slewed_error by the maximum allowable step in the correct direction.
    }
    else
    {                       // If motor power is to be slowing down and cannot slew down (basically imply setting)
      slewed_error = error; // If so, directly set slewed_error to the new error.
    }
  }
  return slewed_error; // Return the adjusted error, representing the output power after slew rate limiting.
}

} // namespace whoop
