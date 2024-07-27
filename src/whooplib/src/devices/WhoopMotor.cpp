/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       WhoopMotor.cpp                                            */
/*    Author:       Connor White (WHOOP)                                      */
/*    Created:      Thu Jun 21 2024                                           */
/*    Description:  Virtual Motor With Streamlined Functions                  */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "whooplib/includer.hpp"
#include "whooplib/include/devices/WhoopMotor.hpp"
#include "whooplib/include/toolbox.hpp"

#define ANALOG_TO_VOLTAGE 127.0/12.0

namespace whoop{

WhoopMotor::WhoopMotor(std::int32_t port) : 
#if USE_VEXCODE
vex_motor(vex::motor(port)) 
#else
pros_motor(pros::Motor(port))
#endif
{}

WhoopMotor::WhoopMotor(std::int32_t port, reversed reversed) : 
#if USE_VEXCODE
vex_motor(vex::motor(port, gear_ratios::green, reversed)) {}
#else
pros_motor(pros::Motor(port))
{
    pros_motor.set_reversed(reversed);
}
#endif

WhoopMotor::WhoopMotor(std::int32_t port, cartridge motorCartridge) : 
#if USE_VEXCODE
vex_motor(vex::motor(port, motorCartridge)) {}
#else
pros_motor(pros::Motor(port))
{
    switch(motorCartridge){
    case cartridge::red:
        pros_motor.get_gearing(pros::E_MOTOR_GEARSET_36);
        break;
    case cartridge::green:
        pros_motor.get_gearing(pros::E_MOTOR_GEARSET_18);
        break;
    case cartridge::blue:
        pros_motor.get_gearing(pros::E_MOTOR_GEARSET_06);
        break;
    }
}
#endif

WhoopMotor::WhoopMotor(std::int32_t port, cartridge motorCartridge, reversed reversed) : 
#if USE_VEXCODE
vex_motor(vex::motor(port, motorCartridge, reversed)) {}
#else
pros_motor(pros::Motor(port))
{
    switch(motorCartridge){
    case cartridge::red:
        pros_motor.get_gearing(pros::E_MOTOR_GEARSET_36);
        break;
    case cartridge::green:
        pros_motor.get_gearing(pros::E_MOTOR_GEARSET_18);
        break;
    case cartridge::blue:
        pros_motor.get_gearing(pros::E_MOTOR_GEARSET_06);
        break;
    }

    pros_motor.set_reversed(reversed);
}
#endif

void WhoopMotor::spin(double volts)
{
    // Linearizes the voltage. Visual representation of the linearization: https://www.desmos.com/calculator/anyejul5wg
    // It attempts to make the voltage and motor power more linearly proportional
    #if USE_VEXCODE
    vex_motor.spin(fwd, linearize_voltage(volts), voltageUnits::volt);
    #else
    pros_motor.move(volts*ANALOG_TO_VOLTAGE);
    #endif
}

void WhoopMotor::spin_unit(double unit)
{ // Unit being -1 to 1, being 0 stopped
    spin(unit*12.0);
}

void WhoopMotor::spin_percentage(double percentage)
{ // Percentage being -100 to 100
    this->spin_unit(percentage / 100.0);
}

void WhoopMotor::stop_hold()
{
    #if USE_VEXCODE
    vex_motor.stop(brakeType::hold);
    #else
    pros_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    pros_motor.brake();
    #endif
}

void WhoopMotor::stop_brake()
{
    #if USE_VEXCODE
    vex_motor.stop(brakeType::brake);
    #else
    pros_motor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    pros_motor.brake();
    #endif
}

void WhoopMotor::stop_coast()
{
    #if USE_VEXCODE
    vex_motor.stop(brakeType::coast);
    #else
    pros_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    pros_motor.brake();
    #endif
}

double WhoopMotor::get_rotation()
{
    #if USE_VEXCODE
    return vex_motor.position(rotationUnits::deg) + pos_offset;
    #else
    return pros_motor.get_position(); // Degrees by default
    #endif
}

double WhoopMotor::get_rotation_rotations()
{
    return this->get_rotation() / 360.0;
}

double WhoopMotor::get_rotation_degrees()
{
    return this->get_rotation();
}

double WhoopMotor::get_rotation_radians()
{
    return to_rad(this->get_rotation());
}

// Receiving velocity
double WhoopMotor::get_velocity()
{
    #if USE_VEXCODE
    return vex_motor.velocity(vel);
    #else
    return pros_motor.get_actual_velocity() * 6.0; // In RPM, so we multiply it by 6 to convert to deg/s
    #endif
}

double WhoopMotor::get_velocity_deg_s()
{
    return get_velocity();
}

double WhoopMotor::get_velocity_rad_s()
{
    return to_rad(get_velocity());
}

double WhoopMotor::get_velocity_rpm()
{
    return get_velocity() / 6.0; // Divide by 6 to convert deg/s to RPM
}

void WhoopMotor::tare(double degrees)
{
    pos_offset = degrees;
    #if USE_VEXCODE
    vex_motor.resetPosition();
    #else
    pros_motor.tare_position();
    #endif
}

void WhoopMotor::tare()
{
    this->tare(0);
}

void WhoopMotor::tare_degrees(double degrees)
{
    this->tare(degrees);
}

void WhoopMotor::tare_rotations(double rotations)
{
    this->tare(rotations * 360.0);
}

void WhoopMotor::tare_radians(double radians)
{
    this->tare(to_deg(radians));
}

} // namespace whoop
