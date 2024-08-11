/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       WhoopController.cpp                                       */
/*    Author:       Connor White (WHOOP)                                      */
/*    Created:      Thu Jun 21 2024                                           */
/*    Description:  Virtual Controller With Additional Functions              */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "whooplib/include/devices/WhoopController.hpp"
#include "whooplib/include/toolbox.hpp"
#include <cmath>
#include <functional>
#include <iostream>
#include <vector>

#define ANALOG_TO_PCT (100.0 / 127.0)

namespace whoop {

// Initialization Constructors
WhoopController::WhoopController(joystickmode mode)
    : WhoopController(mode, controllertype::controller_primary) {}

WhoopController::WhoopController(joystickmode mode,
                                 controllertype controller_type)
    :
#if USE_VEXCODE
      vex::controller(controller_type == controllertype::controller_primary
                          ? vex::controllerType::primary
                          : vex::controllerType::partner),
#else
      pros::Controller(controller_type == controllertype::controller_primary
                           ? pros::E_CONTROLLER_MASTER
                           : pros::E_CONTROLLER_PARTNER),
#endif
      joystick_mode(mode) {
}

void WhoopController::notify(std::string message, double duration_seconds) {
  is_cleared = false;
#if USE_VEXCODE
  vex::controller::Screen.clearLine(1);
  vex::controller::Screen.setCursor(1, 1);
  vex::controller::Screen.print("%s", message.c_str());
  vex::controller::rumble(".");
#else
  pros::Controller::rumble(".");
  pros::delay(
      50); // https://www.vexforum.com/t/unable-to-clear-the-controller-screen/62997/2
  pros::Controller::clear_line(2);
  pros::delay(50);
  pros::Controller::print(2, 0, "%s", message.c_str());
  pros::delay(50);
#endif
  time_left_to_clear =
      duration_seconds * std::round(safeDivide(1000, step_time_ms,
                                               10000)); // in milliseconds
}

void WhoopController::display_text(std::string message) {
  text_to_display = message;
  if (!is_cleared) {
    return;
  }

#if USE_VEXCODE
  vex::controller::Screen.clearLine(1);
  vex::controller::Screen.setCursor(1, 1);
  vex::controller::Screen.print("%s", message.c_str());
#else
  pros::delay(
      50); // https://www.vexforum.com/t/unable-to-clear-the-controller-screen/62997/2
  pros::Controller::clear_line(2);
  pros::delay(50);
  pros::Controller::print(2, 0, "%s", message.c_str());
  pros::delay(50);
#endif
}

void WhoopController::clear_text() {
  text_to_display = "";
  if (!is_cleared) {
    return;
  }
#if USE_VEXCODE
  vex::controller::Screen.clearLine(1);
#else
  pros::delay(
      50); // https://www.vexforum.com/t/unable-to-clear-the-controller-screen/62997/2
  pros::Controller::clear_line(2);
#endif
}

/////////////////////////////////////////////
// Controller joystick
double WhoopController::get_left_joystick_x() {
#if USE_VEXCODE
  return vex::controller::Axis4.position(pct);
#else
  return pros::Controller::get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X) *
         ANALOG_TO_PCT;
#endif
}
double WhoopController::get_left_joystick_y() {
#if USE_VEXCODE
  return vex::controller::Axis3.position(pct);
#else
  return pros::Controller::get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) *
         ANALOG_TO_PCT;
#endif
}
double WhoopController::get_right_joystick_x() {
#if USE_VEXCODE
  return vex::controller::Axis1.position(pct);
#else
  return pros::Controller::get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) *
         ANALOG_TO_PCT;
#endif
}
double WhoopController::get_right_joystick_y() {
#if USE_VEXCODE
  return vex::controller::Axis2.position(pct);
#else
  return pros::Controller::get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) *
         ANALOG_TO_PCT;
#endif
}

/////////////////////////////////////////////
// UDLR Reading
bool WhoopController::up_pressing() {
#if USE_VEXCODE
  return vex::controller::ButtonUp.pressing();
#else
  return pros::Controller::get_digital(pros::E_CONTROLLER_DIGITAL_UP);
#endif
}
bool WhoopController::down_pressing() {
#if USE_VEXCODE
  return vex::controller::ButtonDown.pressing();
#else
  return pros::Controller::get_digital(pros::E_CONTROLLER_DIGITAL_DOWN);
#endif
}
bool WhoopController::left_pressing() {
#if USE_VEXCODE
  return vex::controller::ButtonLeft.pressing();
#else
  return pros::Controller::get_digital(pros::E_CONTROLLER_DIGITAL_LEFT);
#endif
}
bool WhoopController::right_pressing() {
#if USE_VEXCODE
  return vex::controller::ButtonRight.pressing();
#else
  return pros::Controller::get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT);
#endif
}

/////////////////////////////////////////////
// ABXY Reading
bool WhoopController::a_pressing() {
#if USE_VEXCODE
  return vex::controller::ButtonA.pressing();
#else
  return pros::Controller::get_digital(pros::E_CONTROLLER_DIGITAL_A);
#endif
}
bool WhoopController::b_pressing() {
#if USE_VEXCODE
  return vex::controller::ButtonB.pressing();
#else
  return pros::Controller::get_digital(pros::E_CONTROLLER_DIGITAL_B);
#endif
}
bool WhoopController::x_pressing() {
#if USE_VEXCODE
  return vex::controller::ButtonX.pressing();
#else
  return pros::Controller::get_digital(pros::E_CONTROLLER_DIGITAL_X);
#endif
}
bool WhoopController::y_pressing() {
#if USE_VEXCODE
  return vex::controller::ButtonY.pressing();
#else
  return pros::Controller::get_digital(pros::E_CONTROLLER_DIGITAL_Y);
#endif
}

/////////////////////////////////////////////
// Bumpers Reading
bool WhoopController::right_top_bumper_pressing() {
#if USE_VEXCODE
  return vex::controller::ButtonR1.pressing();
#else
  return pros::Controller::get_digital(pros::E_CONTROLLER_DIGITAL_R1);
#endif
}
bool WhoopController::right_bottom_bumper_pressing() {
#if USE_VEXCODE
  return vex::controller::ButtonR2.pressing();
#else
  return pros::Controller::get_digital(pros::E_CONTROLLER_DIGITAL_R2);
#endif
}
bool WhoopController::left_top_bumper_pressing() {
#if USE_VEXCODE
  return vex::controller::ButtonL1.pressing();
#else
  return pros::Controller::get_digital(pros::E_CONTROLLER_DIGITAL_L1);
#endif
}
bool WhoopController::left_bottom_bumper_pressing() {
#if USE_VEXCODE
  return vex::controller::ButtonL2.pressing();
#else
  return pros::Controller::get_digital(pros::E_CONTROLLER_DIGITAL_L2);
#endif
}

void WhoopController::__step() {
  time_left_to_clear--;

  if (time_left_to_clear < -1) {

    time_left_to_clear = -1;
    if (!is_cleared) {
      is_cleared = true;
#if USE_VEXCODE
      vex::controller::Screen.clearLine(1);
      wait(50, msec);
#else
      pros::Controller::clear_line(2);
      pros::delay(50);
#endif
      display_text(text_to_display);
    }
  } else {
    is_cleared = false;
  }
}

} // namespace whoop