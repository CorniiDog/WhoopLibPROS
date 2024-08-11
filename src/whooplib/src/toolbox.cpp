/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       toolbox.cpp                                               */
/*    Author:       Aggie Robotics                                            */
/*    Created:      Thu Jun 21 2024                                           */
/*    Description:  Contains an Assortment of Useful Functions                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "whooplib/include/toolbox.hpp"
#include <algorithm>
#include <cmath>
#include <cstdarg> // Needed for va_list and related operations
#include <iomanip> // Include for std::setprecision
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace whoop {

// Function to find all indexes of a substring in a string
std::vector<int> find_all_indexes(const std::string &str,
                                  const std::string &substring) {
  std::vector<int> indexes;
  size_t start = 0;
  while (true) {
    start = str.find(substring, start);
    if (start == std::string::npos) {
      break;
    }
    indexes.push_back(start);
    start += substring.length(); // Move to the next possible start position
  }
  return indexes;
}

// Function to read messages from buffer between start_marker and end_marker
std::vector<std::string>
read_messages_from_buffer(const std::string &buffer,
                          const std::string &start_marker,
                          const std::string &end_marker) {
  std::vector<int> start_markers = find_all_indexes(buffer, start_marker);
  std::vector<int> end_markers = find_all_indexes(buffer, end_marker);

  // Combine and sort the markers
  std::vector<std::pair<int, std::string>> start_and_end_markers;
  for (int index : start_markers) {
    start_and_end_markers.push_back({index, start_marker});
  }
  for (int index : end_markers) {
    start_and_end_markers.push_back({index, end_marker});
  }
  std::sort(start_and_end_markers.begin(), start_and_end_markers.end());

  std::vector<std::string> messages;
  bool starter = false;
  int start_i = -1;

  for (const auto &[index, marker] : start_and_end_markers) {
    if (marker == start_marker && !starter) {
      starter = true;
      start_i = index + start_marker.length();
    } else if (marker == end_marker && starter) {
      starter = false;
      messages.push_back(buffer.substr(start_i, index - start_i));
      start_i = -1;
    }
  }

  return messages;
}

double clamp(double val, double min, double max) {
  if (val > max) {
    return max;
  }
  if (val < min) {
    return min;
  }
  return val;
}

double linearize_voltage(double volt, double c) {
  if (volt < 0) {
    return -std::sqrt(std::pow(std::abs(volt) / 12.0, c)) * 12.0;
  } else if (volt > 0) {
    return std::sqrt(std::pow(std::abs(volt) / 12.0, c)) * 12.0;
  } else {
    return 0;
  }
}

int int_distance(int point_1, int point_2) {
  return std::abs(point_1 - point_2);
}

double deadband_inverse(double val, double deadband) {
  if (val > -deadband && val < 0) {
    return -deadband;
  }
  if (val < deadband && val > 0) {
    return deadband;
  }
  return val;
}

double volts_clamp(double volts) { return clamp(volts, -12.0, 12.0); }

// Function to get the latest message from buffer
std::string get_latest_message_from_buffer(const std::string &buffer,
                                           const std::string &start_marker,
                                           const std::string &end_marker) {
  std::vector<std::string> messages =
      read_messages_from_buffer(buffer, start_marker, end_marker);
  if (messages.empty()) {
    return "";
  }
  return messages.back();
}

// This strips a string and removes whitespaces and newlines
std::string strip(const std::string &str) {
  auto start = std::find_if_not(str.begin(), str.end(),
                                [](int ch) { return std::isspace(ch); });

  auto end = std::find_if_not(str.rbegin(), str.rend(), [](int ch) {
               return std::isspace(ch);
             }).base();

  if (start >= end) {
    return ""; // return empty string if no non-whitespace character is found
  }

  return std::string(start, end);
}

std::string boolToString(bool b) { return b ? "true" : "false"; }

std::string intToString(int value) {
  std::stringstream ss;
  ss << value;
  return ss.str();
}

std::string doubleToString(double value, int decimal_places) {
  std::stringstream ss;
  ss << std::fixed << std::setprecision(decimal_places) << value;
  return ss.str();
}

int stringToInt(const std::string &str) {
  std::stringstream ss(str);
  int result;
  ss >> result;
  if (ss.fail()) {
    throw("Conversion failed: stringToInt");
  }
  return result;
}

double stringToDouble(const std::string &str) {
  std::stringstream ss(str);
  double result;
  ss >> result;
  if (ss.fail()) {
    throw("Conversion failed: stringToInt");
  }
  return result;
}

float intToFloat(int value) {
  std::stringstream ss;
  ss << value;
  float result;
  ss >> result;
  return result;
}

int floatToInt(float value) {
  std::stringstream ss;
  ss << value;
  int result;
  ss >> result;
  return result;
}

double intToDouble(int value) {
  std::stringstream ss;
  ss << value;
  double result;
  ss >> result;
  return result;
}

int doubleToInt(double value) {
  std::stringstream ss;
  ss << value;
  int result;
  ss >> result;
  return result;
}

bool intToBool(int value) { return value != 0; }

int boolToInt(bool value) { return value ? 1 : 0; }

double to_rad(double deg) { return deg * (M_PI / 180.0); }

double to_deg(double radians) { return radians * (180.0 / M_PI); }

double to_meters(double inches) { return inches / 39.3700787402; }

double to_inches(double meters) { return meters * 39.3700787402; }

// Normalizes a radian angle to [-pi, pi]
double normalize_angle(double angle_radians) {
  angle_radians = fmod(angle_radians + M_PI, 2 * M_PI) - M_PI;
  if (angle_radians < -M_PI) {
    angle_radians += 2 * M_PI;
  }
  return angle_radians;
}

// De-normalizes a radian angle to [0, 2pi)
double denormalize_angle(double angle_radians) {
  angle_radians = fmod(angle_radians, 2 * M_PI);
  if (angle_radians < 0) {
    angle_radians += 2 * M_PI;
  }
  return angle_radians;
}

double circumference_from_radius(double radius) { return 2 * M_PI * radius; }

double circumference_from_diameter(double diameter) { return M_PI * diameter; }

double area_from_radius(double radius) { return M_PI * radius * radius; }

double area_from_diameter(double diameter) {
  double radius = diameter / 2;
  return M_PI * radius * radius;
}

double safeDivide(double numerator, double denominator,
                  double max_possible_number) {
  if (std::abs(denominator) < SMALL_NUMBER_THRESHOLD) {
    if (std::abs(numerator) < SMALL_NUMBER_THRESHOLD) {
      // Both numerator and denominator are very small
      // Calculate an appropriate scaling factor to bring values into a
      // numerically stable range
      double maxAbsValue = std::max(std::abs(numerator), std::abs(denominator));
      double scaleFactor = 1.0 / maxAbsValue;

      numerator *= scaleFactor;
      denominator *= scaleFactor;

      // This may not be sufficient if their discrepency of decimal places are
      // still high, so we do a recheck Recheck to ensure the scaling factor is
      // sufficient
      if (std::abs(denominator) < SMALL_NUMBER_THRESHOLD) {
        return (numerator >= 0) ? max_possible_number : -max_possible_number;
      }
    } else {
      // Denominator is very small, but numerator is not
      return (numerator > 0) ? max_possible_number : -max_possible_number;
    }
  }

  double result = numerator / denominator;
  if (std::abs(result) > max_possible_number) {
    return (result > 0) ? max_possible_number : -max_possible_number;
  }

  return result;
}

std::string truncate(const std::string& text, int truncated_n){
  return text.substr(0, truncated_n);
}

std::string center(const std::string& text, int n){
  // Calculate the number of spaces needed to center the text
  int padding = (n - static_cast<int>(text.size())) / 2;

  // Construct the centered string
  std::string centered_text = std::string(std::max(0, padding), ' ') + text;

  // If the text is shorter than n, pad the right side as well
  centered_text += std::string(std::max(0, n - static_cast<int>(centered_text.size())), ' ');

  // Return the centered and possibly truncated text
  return centered_text;
}

} // namespace whoop
