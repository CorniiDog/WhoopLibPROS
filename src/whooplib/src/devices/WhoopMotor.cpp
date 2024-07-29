/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       WhoopMotor.cpp                                            */
/*    Author:       Connor White (WHOOP)                                      */
/*    Created:      Thu Jun 21 2024                                           */
/*    Description:  Virtual Motor With Streamlined Functions                  */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "whooplib/include/devices/WhoopMotor.hpp"
#include "whooplib/include/toolbox.hpp"
#include "whooplib/includer.hpp"

#define ANALOG_TO_VOLTAGE (127.0 / 12.0)

namespace whoop {

WhoopMotor::WhoopMotor(std::int32_t port)
    : WhoopMotor(port, reversed::no_reverse) {}

WhoopMotor::WhoopMotor(std::int32_t port, reversed reversed)
    : WhoopMotor(port, cartridge::green, reversed) {}

WhoopMotor::WhoopMotor(std::int32_t port, cartridge motorCartridge)
    : WhoopMotor(port, motorCartridge, whoop::reversed::no_reverse) {}

WhoopMotor::WhoopMotor(std::int32_t port, cartridge motorCartridge,
                       reversed reversed)
    :
#if USE_VEXCODE
      vex::motor(port, gearSetting(cartridge::green), reversed) {
}
#else
      pros::Motor(reversed ? -port : port,
                  motorCartridge == cartridge::red
                      ? pros::v5::MotorGears::red
                      : (motorCartridge == cartridge::green
                             ? pros::v5::MotorGears::green
                             : pros::v5::MotorGears::blue)) {
}
#endif

void WhoopMotor::spin(double volts) {
// Linearizes the voltage. Visual representation of the linearization:
// https://www.desmos.com/calculator/anyejul5wg It attempts to make the voltage
// and motor power more linearly proportional
#if USE_VEXCODE
  vex::motor::spin(fwd, linearize_voltage(volts), voltageUnits::volt);
#else
  pros::Motor::move(volts * ANALOG_TO_VOLTAGE);
#endif
}

void WhoopMotor::spin_unit(double unit) { // Unit being -1 to 1, being 0 stopped
  spin(unit * 12.0);
}

void WhoopMotor::spin_percentage(
    double percentage) { // Percentage being -100 to 100
  this->spin_unit(percentage / 100.0);
}

void WhoopMotor::stop_hold() {
#if USE_VEXCODE
  vex::motor::stop(brakeType::hold);
#else
  pros::Motor::set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  pros::Motor::brake();
#endif
}

void WhoopMotor::stop_brake() {
#if USE_VEXCODE
  vex::motor::stop(brakeType::brake);
#else
  pros::Motor::set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  pros::Motor::brake();
#endif
}

void WhoopMotor::stop_coast() {
#if USE_VEXCODE
  vex::motor::stop(brakeType::coast);
#else
  pros::Motor::set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  pros::Motor::brake();
#endif
}

double WhoopMotor::get_rotation() {
#if USE_VEXCODE
  return vex::motor::position(rotationUnits::deg) + pos_offset;
#else
  return pros::Motor::get_position(); // Degrees by default
#endif
}

double WhoopMotor::get_rotation_rotations() {
  return this->get_rotation() / 360.0;
}

double WhoopMotor::get_rotation_degrees() { return this->get_rotation(); }

double WhoopMotor::get_rotation_radians() {
  return to_rad(this->get_rotation());
}

// Receiving velocity
double WhoopMotor::get_velocity() {
#if USE_VEXCODE
  return vex::motor::velocity(velocityUnits::dps);
#else
  return pros::Motor::get_actual_velocity() *
         6.0; // In RPM, so we multiply it by 6 to convert to deg/s
#endif
}

double WhoopMotor::get_velocity_deg_s() { return get_velocity(); }

double WhoopMotor::get_velocity_rad_s() { return to_rad(get_velocity()); }

double WhoopMotor::get_velocity_rpm() {
  return get_velocity() / 6.0; // Divide by 6 to convert deg/s to RPM
}

void WhoopMotor::tare(double degrees) {
  pos_offset = degrees;
#if USE_VEXCODE
  vex::motor::resetPosition();
#else
  pros::Motor::tare_position();
#endif
}

void WhoopMotor::tare() { this->tare(0); }

void WhoopMotor::tare_degrees(double degrees) { this->tare(degrees); }

void WhoopMotor::tare_rotations(double rotations) {
  this->tare(rotations * 360.0);
}

void WhoopMotor::tare_radians(double radians) { this->tare(to_deg(radians)); }

} // namespace whoop
