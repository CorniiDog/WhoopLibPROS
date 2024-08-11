/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       SlewRateLimiter.cpp                                       */
/*    Author:       Connor White (WHOOP)                                      */
/*    Created:      Thu July 4 2024                                           */
/*    Description:  Whoop PID Calculator                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "whooplib/include/calculators/SlewRateLimiter.hpp"
#include <cmath>

namespace whoop {

SlewRateLimiter::SlewRateLimiter(double max_slew_rate,
                                 double step_time_milliseconds)
    : max_slew_rate(max_slew_rate),
      step_time_milliseconds(step_time_milliseconds), previous_output(0.0) {
  max_slew_rate_scaled = max_slew_rate * (step_time_milliseconds / 1000.0);
}

double SlewRateLimiter::step(double desired_output) {
  // Calculate the desired change in output
  double delta_output = desired_output - previous_output;

  // Limit the change (delta_output) to the max_slew_rate_scaled
  if (delta_output > max_slew_rate_scaled) {
    delta_output = max_slew_rate_scaled;
  } else if (delta_output < -max_slew_rate_scaled) {
    delta_output = -max_slew_rate_scaled;
  }

  // Update the output based on the limited delta
  double output = previous_output + delta_output;

  // Store the output for the next step
  previous_output = output;

  return output;
}

} // namespace whoop
