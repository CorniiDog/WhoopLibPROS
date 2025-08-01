/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       PID.cpp                                                   */
/*    Author:       Connor White                                              */
/*    Created:      Thu July 4 2024                                           */
/*    Description:  Whoop PID Calculator                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "whooplib/include/calculators/PID.hpp"
#include <cmath>

namespace whoop {

/**
 * PID constructor with P, I, D, and starti
 * Starti keeps the I term at 0 until error is less than starti
 *
 * @param error Difference in desired and current position
 * @param kp Proportional constant
 * @param ki Integral constant
 * @param kd Derivative constant
 * @param ka Integral anti-windup constant
 * @param starti Maximum error to start integrating
 */

PID::PID(double error, double kp, double ki, double kd, double ka, double starti,
         double max_integral_power)
    : error(error), kp(kp), ki(ki), kd(kd), ka(ka), starti(starti),
      max_integral_power(max_integral_power) {
  max_integral_power_scaled = max_integral_power / ki;
}

PID::PID(double error, double kp, double ki, double kd, double ka, double starti,
         double max_integral_power, double settle_error, double settle_time,
         double timeout)
    : error(error), kp(kp), ki(ki), kd(kd), ka(ka), starti(starti),
      settle_error(settle_error), settle_time(settle_time), timeout(timeout),
      max_integral_power(max_integral_power) {
  max_integral_power_scaled = max_integral_power / ki;
}

double PID::step(double error) {

  double derivative = error - previous_error;
  double error_abs = fabs(error);

  if (reject_first_accumulation) {
    reject_first_accumulation = false;
  } else {
    if (error_abs < starti) {
      accumulated_error += error;

      // Anti-Windup 
      accumulated_error -= derivative * (1 - error_abs / starti) * ka;
    } else {
      accumulated_error = 0;
    }
  }

  // Checks if the error has crossed 0, and if it has, it eliminates the
  // integral term
  // if ((error > 0 && previous_error < 0) || (error < 0 && previous_error > 0)) {
  //   accumulated_error = 0;
  // }

  double max_integral_power_scaled = max_integral_power / ki;
  if (accumulated_error > max_integral_power_scaled)
    accumulated_error = max_integral_power_scaled;
  if (accumulated_error < -max_integral_power_scaled)
    accumulated_error = -max_integral_power_scaled;

  output = kp * error + ki * accumulated_error + kd * derivative;

  previous_error = error;

  if (error_abs < settle_error) {
    time_spent_settled += 10;
  } else {
    time_spent_settled = 0;
  }

  time_spent_running += 10;

  return output;
}

bool PID::is_settled() {
  if (timeout != 0 && time_spent_running > timeout * 1000) {
    return (true);
  } // If timeout does equal 0, the move will never actually time out. Setting
    // timeout to 0 is the
  // equivalent of setting it to infinity
  if (time_spent_settled > settle_time * 1000) {
    return (true);
  }
  return (false);
}

bool PID::settling() {
  if (is_settled()) {
    return true;
  }
  if (time_spent_settled > (settle_time / 5) * 1000) {
    return true;
  }
  return false;
}

void PID::zeroize_accumulated() {
  accumulated_error = 0;
  reject_first_accumulation = true;
}

} // namespace whoop
