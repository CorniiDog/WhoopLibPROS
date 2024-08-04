/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       RollingAverage.hpp                                        */
/*    Author:       Connor White (WHOOP)                                      */
/*    Created:      Thu Jun 21 2024                                           */
/*    Description:  Rolling Average Filter for Odometry                       */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#ifndef ROLLING_AVERAGE_HPP
#define ROLLING_AVERAGE_HPP

#include "whooplib/include/devices/WhoopVision.hpp"
#include "whooplib/includer.hpp"
#include <deque>

namespace whoop {

/**
 * Rolling Average Filter Object
 */
class RollingAverageFilter {
private:
  std::deque<Pose> buffer_pose;
  std::deque<double> buffer_double;
  std::deque<double> buffer_int;
  int capacity;

public:
  /**
   * Constructor for Rolling Average
   * @param n Number of elements for rolling average
   */
  RollingAverageFilter(int capacity);

  /**
   * Processes double into rolling average
   * @return rolling average result
   */
  double process(double newMeasurement);

  /**
   * Processes double into rolling average
   * @return rolling average result
   */
  int process(int newMeasurement);
};

} // namespace whoop

#endif // ROLLING_AVERAGE_HPP