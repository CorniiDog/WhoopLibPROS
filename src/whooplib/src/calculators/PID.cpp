/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       PID.cpp                                                   */
/*    Author:       2775Josh, Modified by Connor White (WHOOP)                */
/*    Created:      Thu July 4 2024                                           */
/*    Description:  Whoop PID Calculator                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "whooplib/include/calculators/PID.hpp"

#include <cmath>

/**
 * PID constructor with P, I, D, and starti
 * Starti keeps the I term at 0 until error is less than starti
 *
 * @param error Difference in desired and current position
 * @param kp Proportional constant
 * @param ki Integral constant
 * @param kd Derivative constant
 * @param starti Maximum error to start integrating
 */

PID::PID(double error, double kp, double ki, double kd, double starti) : error(error),
                                                                         kp(kp),
                                                                         ki(ki),
                                                                         kd(kd),
                                                                         starti(starti)
{
}

PID::PID(double error, double kp, double ki, double kd, double starti,
         double settle_error, double settle_time, double timeout) : error(error),
                                                                    kp(kp),
                                                                    ki(ki),
                                                                    kd(kd),
                                                                    starti(starti),
                                                                    settle_error(settle_error),
                                                                    settle_time(settle_time),
                                                                    timeout(timeout)
{
}

double PID::step(double error)
{
  if (fabs(error) < starti)
  {
    accumulated_error += error;
  }
  // Checks if the error has crossed 0, and if it has, it eliminates the integral term
  if ((error > 0 && previous_error < 0) || (error < 0 && previous_error > 0))
  {
    accumulated_error = 0;
  }

  output = kp * error + ki * accumulated_error + kd * (error - previous_error);

  previous_error = error;

  if (fabs(error) < settle_error)
  {
    time_spent_settled += 10;
  }
  else
  {
    time_spent_settled = 0;
  }

  time_spent_running += 10;

  return output;
}

bool PID::is_settled()
{
  if (timeout != 0 && time_spent_running > timeout * 1000)
  {
    return (true);
  } // If timeout does equal 0, the move will never actually time out. Setting timeout to 0 is the
  // equivalent of setting it to infinity
  if (time_spent_settled > settle_time * 1000)
  {
    return (true);
  }
  return (false);
}

bool PID::settling()
{
  if (is_settled())
  {
    return true;
  }
  if (time_spent_settled > (settle_time / 5) * 1000)
  {
    return true;
  }
  return false;
}

void PID::zeroize_accumulated()
{
  accumulated_error = 0;
}