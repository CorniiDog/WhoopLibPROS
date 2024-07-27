/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       WhoopMotorGroup.hpp                                       */
/*    Author:       Connor White (WHOOP)                                      */
/*    Created:      Thu Jun 21 2024                                           */
/*    Description:  Virtual Motor Group with Additional Reliability Features  */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#ifndef WHOOP_MOTOR_GROUP_HPP
#define WHOOP_MOTOR_GROUP_HPP

#include "whooplib/include/devices/WhoopMotor.hpp"
#include "whooplib/include/toolbox.hpp"
#include "whooplib/includer.hpp"
#include <vector>

namespace whoop{

/**
 * Manages a group of WhoopMotors, allowing synchronized control over multiple motors.
 */
class WhoopMotorGroup
{
private:
    /**
     * Applies a specified member function to all motors in the group with a double value argument.
     * @param func Pointer to the member function to be applied.
     * @param value The double value to pass to the function.
     */
    void apply_to_all(void (WhoopMotor::*func)(double), double value);

    /**
     * Applies a specified member function to all motors in the group without arguments.
     * @param func Pointer to the member function to be applied.
     */
    void apply_to_all(void (WhoopMotor::*func)());

protected:
    // Upon initialization
    std::vector<WhoopMotor *> whoop_motors; // Vector of pointers to WhoopMotor objects.

    double gear_ratio = 1;                // Gear ratio for scaling motor output, default is 1.
    double wheel_diameter = to_meters(4); // Gear ratio for scaling motor output, default is 0.1016 meters, or 4 inches
    double wheel_circumference = circumference_from_diameter(wheel_diameter);

public:
    /**
     * Constructor that initializes a motor group with a vector of motors.
     * @param motors Vector of pointers to initialized WhoopMotors.
     */
    WhoopMotorGroup(std::vector<WhoopMotor *> motors);

    /**
     * Constructor that initializes a motor group with a vector of motors, with addition to gear ratio
     * @param ratio i.e. motor on 32 tooth powering the 64 tooth: ratio = 32.0/64.0 = 0.5
     * @param motors Vector of pointers to initialized WhoopMotors.
     */
    WhoopMotorGroup(double ratio, std::vector<WhoopMotor *> motors);

    /**
     * Constructor that initializes a motor group with a vector of motors, with addition to gear ratio
     * @param ratio i.e. motor on 32 tooth powering the 64 tooth: ratio = 32.0/64.0 = 0.5
     * @param diameter_meters The wheel diameter in meters (i.e. 0.08255 for 3.25" wheels)
     * @param motors Vector of pointers to initialized WhoopMotors.
     */
    WhoopMotorGroup(double ratio, double diameter_meters, std::vector<WhoopMotor *> motors);

    /**
     * Adds a motor to the motor group.
     * @param whoop_motor Pointer to the WhoopMotor to be added to the group.
     */
    void add_motor(WhoopMotor *whoop_motor);

    // Motor commands
    void spin(double volts);                 // Commands all motors to spin at a specified voltage. -12.0 -> 0 -> 12.0, with 0 being stopped.
    void spin_unit(double unit);             // Commands all motors to spin at a unit -1 -> 0 -> 1, with 0 being stopped.
    void spin_percentage(double percentage); // Commands all motors to spin at a specified percentage -100 -> 0 -> 100, with 0 being stopped.
    void stop_hold();                        // Commands all motors to stop and hold their position.
    void stop_brake();                       // Commands all motors to stop with braking.
    void stop_coast();                       // Commands all motors to stop and coast.

    /**
     * Sets the gear ratio multiplier for the motor group.
     * i.e. motor on 32 tooth powering the 64 tooth: ratio = 32.0/64.0 = 0.5
     * @param ratio The gear ratio multiplier to set.
     */
    void set_gear_ratio_mult(double ratio); // motor on 32 tooth powering the 64 toth: ratio = 32.0/64.0

    /**
     * Sets the wheel diameter multiplier for the motor group, in meters
     * @param diameter_meters The wheel diameter in meters (i.e. 0.08255 for 3.25" wheels)
     */
    void set_wheel_diameter(double diameter_meters);

    // Receiving rotation
    // Note: If 3 or more motors are in a motor group, the motor group would get the average of n-1 motors (excluding the outlier motor furthest from average).
    double get_rotation();           // Returns the average rotation across all motors in degrees.
    double get_rotation_degrees();   // Returns the average rotation across all motors in degrees.
    double get_rotation_radians();   // Returns the average rotation across all motors in radians.
    double get_rotation_rotations(); // Returns the average rotation across all motors in full rotations.

    // Receiving velocity
    double get_velocity();       // degrees/sec
    double get_velocity_deg_s(); // explicitly defining degrees/sec
    double get_velocity_rad_s(); // explicitly defining rad/sec
    double get_velocity_rpm();   // explicitly defining rpm

    /**
     * Gets the velocity of the motor group in meters/sec
     * @returns
     */
    double get_velocity_meters_s();

    /**
     * Gets the distance traveled by the motor group in meters (use case would be for a drivetrain)
     * @returns
     */
    double get_distance_meters(); // Gets distance traveled in meters

    // Tare (reset)
    void tare();                           // Resets the encoder count for all motors in the group.
    void tare(double degrees);             // Resets the encoder count for all motors to a specified degree value.
    void tare_degrees(double degrees);     // Resets the encoder count for all motors to a specified degree value.
    void tare_radians(double radians);     // Resets the encoder count for all motors to a specified radian value.
    void tare_rotations(double rotations); // Resets the encoder count for all motors to a specified number of full rotations.

    /**
     * Tares the wheels to set meters, if wheel diameter and gear ratio is set appropriately.
     * @param meters tares to the specified meter distance value
     */
    void tare_meters(double meters); // For a drivetrain, to tare by meters
};

} // namespace whoop

#endif // WHOOP_MOTOR_GROUP_HPP
