/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       WhoopRotation.hpp                                         */
/*    Author:       Connor White                                              */
/*    Created:      Thu Jun 21 2024                                           */
/*    Description:  Virtual Rotation Sensor With Streamlined Functions        */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#ifndef WHOOP_ROTATION_HPP
#define WHOOP_ROTATION_HPP

#include "whooplib/include/devices/WhoopMotor.hpp"
#include "whooplib/include/toolbox.hpp"
#include "whooplib/includer.hpp"

namespace whoop {

/**
 * Represents a rotation sensor with control over its measurement capabilities.
 */

#if USE_VEXCODE
class WhoopRotation : private vex::rotation {

#else
class WhoopRotation : private pros::Rotation {
#endif
protected:
  double pos_offset =
      0; // Offset applied to the position readings of the rotation sensor.
  double wheel_diameter = to_meters(
      2.75); // Wheel Diameter in meters. For example, 0.06985 is 2.75 inches
  double wheel_circumference = circumference_from_diameter(wheel_diameter);

public:
  /**
   * Constructor to initialize a rotation sensor on a specified port.
   * @param port The port number where the rotation sensor is connected.
   */
  WhoopRotation(std::int32_t port);

  /**
   * Constructor to initialize a rotation sensor with an option to reverse its
   * direction.
   * @param port The port number where the rotation sensor is connected.
   * @param reversed Enum value to set the rotation sensor direction reversed or
   * not.
   */
  WhoopRotation(std::int32_t port, reversed reversed);

  /**
   * Constructor to initialize a rotation sensor on a specified port with wheel
   * diameter in meters. Use case would be for a tracking wheel
   * @param wheel_diameter_meters The diameter of the wheel, in meters, sharing
   * the same axle as the rotation sensor
   * @param port The port number where the rotation sensor is connected.
   */
  WhoopRotation(double wheel_diameter_meters, std::int32_t port);

  /**
   * Constructor to initialize a rotation sensor with an option to reverse its
   * direction and with wheel diameter in meters. Use case would be for a
   * tracking wheel
   * @param wheel_diameter_meters The diameter of the wheel, in meters, sharing
   * the same axle as the rotation sensor
   * @param port The port number where the rotation sensor is connected.
   * @param reversed Enum value to set the rotation sensor direction reversed or
   * not.
   */
  WhoopRotation(double wheel_diameter_meters, std::int32_t port,
                reversed reversed);

  /**
   * Sets the wheel diameter multiplier for the rotation sensor, in meters
   * @param diameter_meters The wheel diameter in meters (i.e. 0.08255 for 3.25"
   * wheels)
   */
  void set_wheel_diameter(double diameter_meters);

  /**
   * Gets the distance traveled by the rotation sensor in meters (use case would
   * be for a drivetrain)
   * @returns
   */
  double get_distance_meters(); // Gets distance traveled in meters

  // Receiving rotation
  double get_rotation();         // Degrees is default
  double get_rotation_degrees(); // Returns the current rotation sensor rotation
                                 // in degrees.
  double get_rotation_radians(); // Returns the current rotation sensor rotation
                                 // in radians.
  double get_rotation_rotations(); // Returns the current rotation sensor
                                   // rotation in full rotations.

  // Receiving velocity
  double get_velocity();       // degree/s
  double get_velocity_deg_s(); // explicitly defining degrees/sec
  double get_velocity_rad_s(); // explicitly defining rad/sec
  double get_velocity_rpm();   // explicitly defining rpm

  /**
   * Gets the velocity of the rotation sensor in meters/sec
   * @returns
   */
  double get_velocity_meters_s();

  // Tare (reset)
  void tare();                       // Resets to zero
  void tare(double degrees);         // Degrees is default
  void tare_degrees(double degrees); // Resets the rotation sensor encoder count
                                     // to a specified degree.
  void tare_radians(double radians); // Resets the rotation sensor encoder count
                                     // to a specified radian.
  void
  tare_rotations(double rotations); // Resets the rotation sensor encoder count
                                    // to a specified number of rotations.

  /**
   * Tares the wheels to set meters, if wheel diameter and gear ratio is set
   * appropriately.
   * @param meters tares to the specified meter distance value
   */
  void tare_meters(double meters); // For a drivetrain, to tare by meters
};

} // namespace whoop

#endif // WHOOP_ROTATION_HPP
