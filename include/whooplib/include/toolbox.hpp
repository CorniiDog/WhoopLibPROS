/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       toolbox.hpp                                               */
/*    Author:       Connor White                                              */
/*    Created:      Thu Jun 21 2024                                           */
/*    Description:  Contains an Assortment of Useful Functions                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#ifndef TOOLBOX_HPP
#define TOOLBOX_HPP

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace whoop {

/**
 * Finds all indices of a substring within a string.
 * @param str The main string to search within.
 * @param substring The substring to find.
 * @return A vector of integers representing the starting indices of each
 * occurrence of substring in str.
 */
std::vector<int> find_all_indexes(const std::string &str,
                                  const std::string &substring);

/**
 * Extracts messages from a buffer delimited by start and end markers.
 * @param buffer The string buffer to parse.
 * @param start_marker The string marking the start of a message.
 * @param end_marker The string marking the end of a message.
 * @return A vector of strings, each representing a message found between a pair
 * of start and end markers.
 */
std::vector<std::string>
read_messages_from_buffer(const std::string &buffer,
                          const std::string &start_marker,
                          const std::string &end_marker);

/**
 * Retrieves the latest message from a buffer based on the specified start and
 * end markers.
 * @param buffer The string buffer to search.
 * @param start_marker The string marking the start of a message.
 * @param end_marker The string marking the end of a message.
 * @return The most recently added message found in the buffer.
 */
std::string get_latest_message_from_buffer(const std::string &buffer,
                                           const std::string &start_marker,
                                           const std::string &end_marker);

/**
 * Removes leading, trailing, and excess embedded whitespace characters from a
 * string.
 * @param str The string to clean up.
 * @return A trimmed and cleaned string with no leading/trailing or consecutive
 * whitespace characters.
 */
std::string strip(const std::string &str);

// Conversion functions
std::string boolToString(bool b);
std::string intToString(int value);
std::string doubleToString(double value, int decimal_places = 4);
int stringToInt(const std::string &str);
double stringToDouble(const std::string &str);
float intToFloat(int value);
int floatToInt(float value);
double intToDouble(int value);
int doubleToInt(double value);
bool intToBool(int value);
int boolToInt(bool value);
double stringToDouble(const std::string &str);

/**
 * Converts degrees to radians.
 * @param deg Angle in degrees.
 * @return Equivalent angle in radians.
 */
double to_rad(double deg);

/**
 * Converts radians to degrees.
 * @param rad Angle in radians.
 * @return Equivalent angle in degrees.
 */
double to_deg(double rad);

/**
 * Converts inches to meters.
 * @param inches Length in inches.
 * @return Equivalent length in meters.
 */
double to_meters(double inches);

/**
 * Converts meters to inches.
 * @param meters Length in meters.
 * @return Equivalent length in inches.
 */
double to_inches(double meters);

/**
 * Normalizes a radian angle to the range [-π, π].
 * @param angle_radians Angle in radians to normalize.
 * @return Normalized angle in radians.
 */
double normalize_angle(double angle_radians);

/**
 * Normalizes a radian angle to the range [0, 2π].
 * @param angle_radians Angle in radians to normalize.
 * @return Normalized angle in radians.
 */
double denormalize_angle(double angle_radians);

/**
 * Calculates the circumference of a circle given its radius.
 * @param radius Radius of the circle in meters.
 * @return Circumference of the circle in meters.
 */
double circumference_from_radius(double radius);

/**
 * Calculates the circumference of a circle given its diameter.
 * @param diameter Diameter of the circle in meters.
 * @return Circumference of the circle in meters.
 */
double circumference_from_diameter(double diameter);

/**
 * Calculates the area of a circle given its radius.
 * @param radius Radius of the circle in meters.
 * @return Area of the circle in square meters.
 */
double area_from_radius(double radius);

/**
 * Calculates the area of a circle given its diameter.
 * @param diameter Diameter of the circle in meters.
 * @return Area of the circle in square meters.
 */
double area_from_diameter(double diameter);

const double SMALL_NUMBER_THRESHOLD = 1e-9;
const double MAX_POSSIBLE_DOUBLE_NUMBER =
    std::numeric_limits<double>::infinity();

/**
 * Does a safe divide calculation that completely avoids
 * divide by zero errors.
 * @param numerator the numerator, or the value that will be divided
 * @param denominator  the denominator, or the value to divide from the
 * numerator
 */
double safeDivide(double numerator, double denominator,
                  double max_possible_number = MAX_POSSIBLE_DOUBLE_NUMBER);

/**
 * This clamps to volts value [-12.0, 12.0]
 * @param volts is the volts to pass through, as a double
 * @returns the clamped variables
 */
double volts_clamp(double volts);

/**
 * Clamps a value between [min, max]
 */
double clamp(double val, double min, double max);

/**
 * The opposite of a deadband function. If within the deadband, it returns the
 * deadband or -deadband, based upon 0 at middle
 */
double deadband_inverse(double val, double deadband);

/**
 * Gets the distance between point 1 and point 2
 */
int int_distance(int point_1, int point_2);

/**
 * Attempts to linearize the motor power
 * Voltage is from [-12.0, 12.0]. You can modify c to make
 * the curve less or more agressive.
 * Visual representation of curving:
 * https://www.desmos.com/calculator/anyejul5wg
 */
double linearize_voltage(double volt, double c = 1.5);


/**
 * Centers text to n place
 */
std::string center(const std::string& text, int n);

/**
 * Centers text to n characters
 */
std::string truncate(const std::string& text, int truncated_n);

} // namespace whoop

#endif // TOOLBOX_HPP
