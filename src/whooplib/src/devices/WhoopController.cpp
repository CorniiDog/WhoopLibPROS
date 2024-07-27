/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       WhoopController.cpp                                       */
/*    Author:       Connor White (WHOOP)                                      */
/*    Created:      Thu Jun 21 2024                                           */
/*    Description:  Virtual Controller With Additional Functions              */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "whooplib/include/toolbox.hpp"
#include "whooplib/include/devices/WhoopController.hpp"
#include <vector>
#include <functional>
#include <cmath>

#define ANALOG_TO_PCT (100.0/127.0)

namespace whoop{

// Initialization Constructors
WhoopController::WhoopController(joystickmode mode) : WhoopController(mode, controllertype::controller_primary) {}

WhoopController::WhoopController(joystickmode mode, controllertype controller_type) :
#if USE_VEXCODE
vex_controller(controller_type == controllertype::controller_primary ? vex::controllerType::primary : vex::controllerType::partner),
#else
pros_controller(controller_type == controllertype::controller_partner ? pros::E_CONTROLLER_MASTER : pros::E_CONTROLLER_PARTNER),
#endif
joystick_mode(mode)
{}

void WhoopController::notify(std::string message, double duration_seconds)
{
    #if USE_VEXCODE
    vex_controller.Screen.clearLine(1);
    vex_controller.Screen.setCursor(1, 1);
    vex_controller.Screen.print("%s", message.c_str());
    vex_controller.rumble(".");
    #else
    pros_controller.clear_line(1);
    pros_controller.print(1,2, "%s", message.c_str());
    pros_controller.rumble(".");
    #endif

    time_left_to_clear = duration_seconds * std::round(safeDivide(1000, step_time_ms, 10000)); // in milliseconds
}

/////////////////////////////////////////////
// Controller joystick
double WhoopController::get_left_joystick_x()
{   
    #if USE_VEXCODE
    return vex_controller.Axis4.position(pct);
    #else
    return pros_controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X) * ANALOG_TO_PCT;
    #endif
}
double WhoopController::get_left_joystick_y()
{
    #if USE_VEXCODE
    return vex_controller.Axis3.position(pct);
    #else
    return pros_controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) * ANALOG_TO_PCT;
    #endif
}
double WhoopController::get_right_joystick_x()
{
    #if USE_VEXCODE
    return vex_controller.Axis1.position(pct);
    #else
    return pros_controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) * ANALOG_TO_PCT;
    #endif
}
double WhoopController::get_right_joystick_y()
{
    #if USE_VEXCODE
    return vex_controller.Axis2.position(pct);
    #else
    return pros_controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) * ANALOG_TO_PCT;
    #endif
}

/////////////////////////////////////////////
// UDLR Reading
bool WhoopController::up_pressing()
{
    #if USE_VEXCODE
    return vex_controller.ButtonUp.pressing();
    #else
    return pros_controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP);
    #endif
}
bool WhoopController::down_pressing()
{
    #if USE_VEXCODE
    return vex_controller.ButtonDown.pressing();
    #else
    return pros_controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN);
    #endif
}
bool WhoopController::left_pressing()
{
    #if USE_VEXCODE
    return vex_controller.ButtonLeft.pressing();
    #else
    return pros_controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT);
    #endif
}
bool WhoopController::right_pressing()
{
    #if USE_VEXCODE
    return vex_controller.ButtonRight.pressing();
    #else
    return pros_controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT);
    #endif
}

/////////////////////////////////////////////
// ABXY Reading
bool WhoopController::a_pressing()
{
    #if USE_VEXCODE
    return vex_controller.ButtonA.pressing();
    #else
    return pros_controller.get_digital(pros::E_CONTROLLER_DIGITAL_A);
    #endif
}
bool WhoopController::b_pressing()
{
    #if USE_VEXCODE
    return vex_controller.ButtonB.pressing();
    #else
    return pros_controller.get_digital(pros::E_CONTROLLER_DIGITAL_B);
    #endif
}
bool WhoopController::x_pressing()
{
    #if USE_VEXCODE
    return vex_controller.ButtonX.pressing();
    #else
    return pros_controller.get_digital(pros::E_CONTROLLER_DIGITAL_X);
    #endif
}
bool WhoopController::y_pressing()
{
    #if USE_VEXCODE
    return vex_controller.ButtonY.pressing();
    #else
    return pros_controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y);
    #endif
}

/////////////////////////////////////////////
// Bumpers Reading
bool WhoopController::right_top_bumper_pressing()
{
    #if USE_VEXCODE
    return vex_controller.ButtonR1.pressing();
    #else
    return pros_controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
    #endif
}
bool WhoopController::right_bottom_bumper_pressing()
{
    #if USE_VEXCODE
    return vex_controller.ButtonR2.pressing();
    #else
    return pros_controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
    #endif
}
bool WhoopController::left_top_bumper_pressing()
{
    #if USE_VEXCODE
    return vex_controller.ButtonL1.pressing();
    #else
    return pros_controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
    #endif
}
bool WhoopController::left_bottom_bumper_pressing()
{
    #if USE_VEXCODE
    return vex_controller.ButtonL2.pressing();
    #else
    return pros_controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
    #endif
}

void WhoopController::__step()
{
    time_left_to_clear -= 1;

    if (time_left_to_clear < -1)
    {
        time_left_to_clear = -1;
    }

    if (time_left_to_clear == 0)
    {
        #if USE_VEXCODE
        vex_controller.Screen.clearLine(1);
        #else
        pros_controller.clear_line(1);
        #endif
    }
}

} // namespace whoop