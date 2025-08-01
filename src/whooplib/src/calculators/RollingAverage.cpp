/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       RollingAverage.cpp                                        */
/*    Author:       Connor White                                              */
/*    Created:      Thu Jun 21 2024                                           */
/*    Description:  Rolling Average Filter for Odometry                       */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "whooplib/include/calculators/RollingAverage.hpp"
#include "whooplib/includer.hpp"

namespace whoop {

RollingAverageFilter::RollingAverageFilter(int capacity) : capacity(capacity) {}

double RollingAverageFilter::process(double newMeasurement) {
  if (buffer_double.size() >= capacity) {
    buffer_double.pop_front();
  }
  buffer_double.push_back(newMeasurement);

  double sum = 0;
  for (const auto &measurement : buffer_double) {
    sum += measurement;
  }
  int size = buffer_double.size();
  return sum / size;
}

int RollingAverageFilter::process(int newMeasurement) {
  if (buffer_int.size() >= capacity) {
    buffer_int.pop_front();
  }
  buffer_int.push_back(newMeasurement);

  double sum = 0;
  for (const auto &measurement : buffer_int) {
    sum += measurement;
  }
  int size = buffer_int.size();
  return (int)(sum / size);
}

} // namespace whoop
