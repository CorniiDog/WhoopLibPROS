/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       SlewRateLimiter.hpp                                       */
/*    Author:       Connor White (WHOOP)                                      */
/*    Created:      Thu July 4 2024                                           */
/*    Description:  Slew Calculator                                           */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include <cmath> // For std::abs and std::min

#ifndef SLEW_RATE_LIMITER_HPP
#define SLEW_RATE_LIMITER_HPP

namespace whoop {

/**
 * General-use Slew class for motor voltage.
 * The default update period is 10ms or 100Hz
 */
class SlewRateLimiter {
public:

/**
 * Constructor for slew rate limiting
 * @param max_slew_rate The maximum rate
 */
  SlewRateLimiter(double max_slew_rate, double step_time_milliseconds);

  /**
   * Steps the slew rate limiter
   * @param desired_output Difference in desired and current position
   * @return Output power
   */
  double step(double desired_output);

private:
  double max_slew_rate;
  double step_time_milliseconds;
  double max_slew_rate_scaled;
  double previous_output;
};

} // namespace whoop

#endif // SLEW_RATE_LIMITER_HPP