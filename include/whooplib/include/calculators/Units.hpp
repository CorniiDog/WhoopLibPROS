/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       Units.cpp                                                 */
/*    Author:       Connor White (WHOOP)                                      */
/*    Created:      Thu July 25 2024                                          */
/*    Description:  Units System for C++                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// actual measurements for the omni-wheels are from LemLib documentation
// Which can be found here: https://lemlib.readthedocs.io/en/v0.5.0/tutorials/2_configuration.html

#include <ratio>
#include <cmath>

#ifndef DESIGNATED_UNITS_H
#define DESIGNATED_UNITS_H

// Define a namespace for user-defined literals
namespace units {
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
constexpr long double operator"" _volts(long double volts) { return volts; }

constexpr long double operator"" _volts(unsigned long long volts) { return volts; }

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

// Integral anti-windup constant
constexpr long double operator"" _kr(long double kr) { return kr; }

constexpr long double operator"" _kr(unsigned long long kr) { return kr; }

// Points remains the same
constexpr unsigned int operator"" _points(unsigned long long points) {
    return static_cast<unsigned int>(points);
}

// Source: https://lemlib.readthedocs.io/en/v0.5.0/tutorials/2_configuration.html#wheel-diameter
/**
 * MIT License
 * 
 * Copyright (c) 2024 Liam Teale and LemLib contributors
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * 
 * Reference to original work: https://lemlib.readthedocs.io/en/stable/tutorials/2_configuration.html
 * Reference to license: https://github.com/LemLib/LemLib/blob/master/LICENSE
 */
namespace Omniwheel{
  constexpr double NEW_2 = 2.125_in;
  constexpr double NEW_275 = 2.75_in;
  constexpr double OLD_275 = 2.75_in;
  constexpr double NEW_275_HALF = 2.744_in;
  constexpr double OLD_275_HALF = 2.74_in;
  constexpr double NEW_325 = 3.25_in;
  constexpr double OLD_325 = 3.25_in;
  constexpr double NEW_325_HALF = 3.246_in;
  constexpr double OLD_325_HALF = 3.246_in;
  constexpr double NEW_4 = 4.00_in;
  constexpr double OLD_4 = 4.18_in;
  constexpr double NEW_4_HALF = 3.995_in;
  constexpr double OLD_4_HALF = 4.175_in;
}

} // namespace units

#endif // DESIGNATED_UNITS_H