/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       WhoopController.hpp                                       */
/*    Author:       Connor White (WHOOP)                                      */
/*    Created:      Thu Jun 21 2024                                           */
/*    Description:  Virtual Controller With Additional Functions              */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#ifndef WHOOP_CONTROLLER_HPP
#define WHOOP_CONTROLLER_HPP

#include "whooplib/include/nodes/NodeManager.hpp"
#include "whooplib/includer.hpp"
#include <functional>
#include <vector>

namespace whoop {
/**
 * Enumerates the available joystick control modes.
 */
enum joystickmode {
  joystickmode_tank = 1,
  joystickmode_split_arcade = 2,
  joystickmode_left_arcade = 3,
  joystickmode_right_arcade = 4
};

/*
 * VEX Controller type for primary and partner
 */
enum controllertype { controller_primary, controller_partner };

/**
 * Controls and manages inputs from a VEX controller.
 */
#if USE_VEXCODE
class WhoopController : public ComputeNode, private vex::controller {
#else
class WhoopController : public ComputeNode, private pros::Controller {
#endif
public:
  joystickmode joystick_mode; // Current joystick mode.

  int time_left_to_clear = 0;

  /**
   * Constructor that initializes the controller with a specific joystick mode.
   * @param mode The joystick mode to be used.
   */
  WhoopController(joystickmode mode = joystickmode::joystickmode_split_arcade);

  /**
   * Constructor that initializes the controller with a specific joystick mode
   * and controller type.
   * @param mode The joystick mode to be used.
   * @param controller_type The type of controller (primary or partner).
   */
  WhoopController(joystickmode mode, controllertype controller_type);

  /**
   * Retrieves the horizontal axis value of the left joystick.
   * @return The x-coordinate value from the left joystick [-100, 100].
   */
  double get_left_joystick_x();

  /**
   * Retrieves the vertical axis value of the left joystick.
   * @return The y-coordinate value from the left joystick [-100, 100].
   */
  double get_left_joystick_y();

  /**
   * Retrieves the horizontal axis value of the right joystick.
   * @return The x-coordinate value from the right joystick [-100, 100].
   */
  double get_right_joystick_x();

  /**
   * Retrieves the vertical axis value of the right joystick.
   * @return The y-coordinate value from the right joystick [-100, 100].
   */
  double get_right_joystick_y();

  /**
   * Notifies for a set period of time
   * @param message The message to send the notification
   * @param duration_seconds The duration to display the message
   */
  void notify(std::string message, double duration_seconds = 5);

  // UDLR
  bool up_pressing();
  bool down_pressing();
  bool left_pressing();
  bool right_pressing();

  // ABXY
  bool a_pressing();
  bool b_pressing();
  bool x_pressing();
  bool y_pressing();

  // Bumpers
  bool right_top_bumper_pressing();
  bool right_bottom_bumper_pressing();
  bool left_top_bumper_pressing();
  bool left_bottom_bumper_pressing();

public:
  void __step() override; // Protected helper function for processing steps
};

} // Namespace whoop

#endif // WHOOP_CONTROLLER_HPP
