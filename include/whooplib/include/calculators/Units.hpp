/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       Units.cpp                                                 */
/*    Author:       Connor White (WHOOP)                                      */
/*    Created:      Thu July 25 2024                                          */
/*    Description:  Units System for C++                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include <ratio>

#ifndef DESIGNATED_UNITS_H
#define DESIGNATED_UNITS_H

// Define a namespace for user-defined literals
namespace whoop {
// Conversion factor: 1 inch = 0.0254 meters
constexpr long double operator"" _in(long double inches) {
  return inches * 0.0254;
}

constexpr long double operator"" _in(unsigned long long inches) {
  return inches * 0.0254;
}

// Conversion factor: 1 millimeter = 0.001 meters
constexpr long double operator"" _mm(long double millimeters) {
  return millimeters * 0.001;
}

constexpr long double operator"" _mm(unsigned long long millimeters) {
  return millimeters * 0.001;
}

// Meters remain the same
constexpr long double operator"" _m(long double meters) { return meters; }

constexpr long double operator"" _m(unsigned long long meters) {
  return meters;
}

// Conversion factor: 1 degree = pi/180 radians
constexpr long double operator"" _deg(long double degrees) {
  return degrees * M_PI / 180.0;
}

constexpr long double operator"" _deg(unsigned long long degrees) {
  return degrees * M_PI / 180.0;
}

// Radians remain the same
constexpr long double operator"" _rad(long double radians) { return radians; }

constexpr long double operator"" _rad(unsigned long long radians) {
  return radians;
}

// Volts remain the same
constexpr long double operator"" _v(long double volts) { return volts; }

constexpr long double operator"" _v(unsigned long long volts) { return volts; }

// Conversion factor: 1 millisecond = 0.001 seconds
constexpr long double operator"" _msec(long double milliseconds) {
  return milliseconds * 0.001;
}

constexpr long double operator"" _msec(unsigned long long milliseconds) {
  return milliseconds * 0.001;
}

// Seconds remain the same
constexpr long double operator"" _sec(long double seconds) { return seconds; }

constexpr long double operator"" _sec(unsigned long long seconds) {
  return seconds;
}

// Proportional gain remains the same
constexpr long double operator"" _kp(long double kp) { return kp; }

constexpr long double operator"" _kp(unsigned long long kp) { return kp; }

// Integral gain remains the same
constexpr long double operator"" _ki(long double ki) { return ki; }

constexpr long double operator"" _ki(unsigned long long ki) { return ki; }

// Derivative gain remains the same
constexpr long double operator"" _kd(long double kd) { return kd; }

constexpr long double operator"" _kd(unsigned long long kd) { return kd; }
} // namespace whoop

#endif // DESIGNATED_UNITS_H