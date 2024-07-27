/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       WhoopRotation.hpp                                         */
/*    Author:       Connor White (WHOOP)                                      */
/*    Created:      Thu Jun 21 2024                                           */
/*    Description:  Virtual Rotation Sensor With Streamlined Functions        */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "whooplib/includer.hpp"
#include "whooplib/include/devices/WhoopRotation.hpp"
#include "whooplib/include/toolbox.hpp"
#include <stdexcept>

namespace whoop{

WhoopRotation::WhoopRotation(std::int32_t port) : 
#if USE_VEXCODE
vex_rotation(vex::rotation(port, false)) 
#else
pros_rotation(pros::Rotation(port))
#endif
{}

WhoopRotation::WhoopRotation(std::int32_t port, reversed reversed) : 
#if USE_VEXCODE
vex_rotation(vex::rotation(port, reversed)) {}
#else
pros_rotation(pros::Rotation(port))
{
    pros_rotation.set_reversed(reversed);
}
#endif

WhoopRotation::WhoopRotation(double wheel_diameter_meters, std::int32_t port) : WhoopRotation(port)
{
    set_wheel_diameter(wheel_diameter_meters);
}

WhoopRotation::WhoopRotation(double wheel_diameter_meters, std::int32_t port, reversed reversed) : WhoopRotation(port, reversed)
{
    set_wheel_diameter(wheel_diameter_meters);
}

void WhoopRotation::set_wheel_diameter(double diameter_meters)
{
    wheel_diameter = diameter_meters;
    wheel_circumference = circumference_from_diameter(wheel_diameter);
}

double WhoopRotation::get_rotation()
{
    #if USE_VEXCODE
    return vex_rotation.position(rotationUnits::deg) + pos_offset;
    #else
    return pros_rotation.get_position()/100.0; // Position in centidegrees, so we convert
    #endif
}

double WhoopRotation::get_rotation_rotations()
{
    return this->get_rotation() / 360.0;
}

double WhoopRotation::get_rotation_degrees()
{
    return this->get_rotation();
}

double WhoopRotation::get_rotation_radians()
{
    return to_rad(this->get_rotation());
}

double WhoopRotation::get_velocity()
{   
    #if USE_VEXCODE
    return vex_rotation.velocity(velocityUnits::dps);
    #else
    return pros_rotation.get_velocity() / 100.0; // Convert from centidegrees/s to degrees/s
    #endif
}
double WhoopRotation::get_velocity_deg_s()
{
    return get_velocity();
}
double WhoopRotation::get_velocity_rad_s()
{
    return to_rad(get_velocity());
}
double WhoopRotation::get_velocity_rpm()
{
    return get_velocity() / 8.0; // Converts degree/s to rpm
}

double WhoopRotation::get_velocity_meters_s()
{
    return get_velocity() * wheel_circumference / 360.0;
}

double WhoopRotation::get_distance_meters()
{
    return get_rotation_rotations() * wheel_circumference;
}

void WhoopRotation::tare(double degrees)
{
    pos_offset = degrees;
    #if USE_VEXCODE
    vex_rotation.resetPosition();
    #else
    pros_rotation.reset_position();
    #endif
}

void WhoopRotation::tare()
{
    this->tare(0);
}

void WhoopRotation::tare_degrees(double degrees)
{
    this->tare(degrees);
}

void WhoopRotation::tare_rotations(double rotations)
{
    this->tare(rotations * 360.0);
}

void WhoopRotation::tare_radians(double radians)
{
    this->tare(to_deg(radians));
}

void WhoopRotation::tare_meters(double meters)
{
    if (wheel_diameter <= 0)
    {
        throw std::invalid_argument("Wheel diameter must be set and positive to tare by meters.");
    }
    // Convert meters to the number of rotations needed
    double rotations_needed = meters / wheel_circumference;
    tare_rotations(rotations_needed);
}

} // namespace whoop
