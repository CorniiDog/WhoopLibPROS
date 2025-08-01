/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       WhoopDrivetrain.cpp                                       */
/*    Author:       Connor White                                              */
/*    Created:      Thu Jun 21 2024                                           */
/*    Description:  Virtual Drivetrain for Controlling Chassis                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "whooplib/include/devices/WhoopDrivetrain.hpp"
#include "whooplib/include/devices/WhoopOdomFusion.hpp"
#include "whooplib/include/toolbox.hpp"
#include "whooplib/includer.hpp"
#include <cmath>
#include <iostream>
#include <memory> // For std::unique_ptr
#include <sstream>
#include <string>

namespace whoop {

void WhoopDrivetrain::init_motor_groups(WhoopMotorGroup *leftGroup,
                                        WhoopMotorGroup *rightGroup) {
  left_motor_group = std::make_unique<WhoopMotorGroup>(*leftGroup);
  right_motor_group = std::make_unique<WhoopMotorGroup>(*rightGroup);
}

void WhoopDrivetrain::init_motor_groups(
    const std::vector<WhoopMotor *> &leftMotors,
    const std::vector<WhoopMotor *> &rightMotors) {
  left_motor_group = std::make_unique<WhoopMotorGroup>(leftMotors);
  right_motor_group = std::make_unique<WhoopMotorGroup>(rightMotors);
}

WhoopDrivetrain::WhoopDrivetrain(PursuitParams *default_pursuit_parameters,
                                 WhoopOdomFusion *odom_fusion,
                                 PoseUnits pose_units,
                                 WhoopController *controller,
                                 WhoopMotorGroup *leftMotorGroup,
                                 WhoopMotorGroup *rightMotorGroup)
    : whoop_controller(controller),
      pursuit_conductor(default_pursuit_parameters) {
  init_motor_groups(leftMotorGroup, rightMotorGroup);
  this->odom_fusion = odom_fusion;
  this->pose_units = pose_units;
  this->default_pose_units = pose_units;
}

WhoopDrivetrain::WhoopDrivetrain(PursuitParams *default_pursuit_parameters,
                                 WhoopOdomFusion *odom_fusion,
                                 PoseUnits pose_units,
                                 WhoopController *controller,
                                 std::vector<WhoopMotor *> leftMotors,
                                 std::vector<WhoopMotor *> rightMotors)
    : whoop_controller(controller),
      pursuit_conductor(default_pursuit_parameters) {
  init_motor_groups(leftMotors, rightMotors);
  this->odom_fusion = odom_fusion;
  this->pose_units = pose_units;
  this->default_pose_units = pose_units;
}

void WhoopDrivetrain::set_state(drivetrainState state) {
  if (state == drivetrainState::mode_autonomous) {
    // Calibration logic to ensure that the robot is calibrated properly before
    // anything else
    if (is_calibrating) {
      while (is_calibrating) {
#if USE_VEXCODE
        wait(5, msec);
#else
        pros::delay(5);
#endif
      }
    } else if (!is_calibrated) {
      calibrate();
    }

    pose_units = default_pose_units;
  }
  drive_state = state;
}

bool WhoopDrivetrain::using_inches() {
  return (pose_units == PoseUnits::in_deg_ccw ||
          pose_units == PoseUnits::in_deg_cw ||
          pose_units == PoseUnits::in_rad_ccw ||
          pose_units == PoseUnits::in_rad_cw);
}
bool WhoopDrivetrain::using_degrees() {
  return (pose_units == PoseUnits::m_deg_cw ||
          pose_units == PoseUnits::m_deg_ccw ||
          pose_units == PoseUnits::in_deg_ccw ||
          pose_units == PoseUnits::in_deg_cw);
}
bool WhoopDrivetrain::using_clockwise() {
  return (
      pose_units == PoseUnits::m_deg_cw || pose_units == PoseUnits::m_rad_cw ||
      pose_units == PoseUnits::in_deg_cw || pose_units == PoseUnits::in_rad_cw);
}

void WhoopDrivetrain::turn(double angle, double timeout_seconds) {
  double current_rotation = desired_position.yaw;

  if (using_degrees()) {
    current_rotation = to_deg(current_rotation); // (radians -> degrees)
  }

  if (using_clockwise()) {
    current_rotation *= -1; // (counter-clockwise -> clockwise)
  }

  current_rotation += angle;

  turn_to(current_rotation, timeout_seconds);
}

void WhoopDrivetrain::turn_to(double yaw, double timeout_seconds) {
  TwoDPose target_pose = desired_position;

  // Convert to standardized
  if (using_degrees()) {
    yaw = to_rad(yaw); // (degrees -> radians)
  }

  if (using_clockwise()) {
    yaw *= -1; // (clockwise -> counter-clockwise)
  }

  target_pose.yaw = yaw; // change yaw

  this->wait_until_completed(); // Wait before previous action is completed
                                // before generating path

  pursuit_conductor.generate_turn(target_pose, timeout_seconds);

  auton_traveling = true;

  last_desired_position = desired_position;
  desired_position = target_pose;
}

void WhoopDrivetrain::turn_to_position(double x, double y,
                                       double timeout_seconds) {

  if (using_inches()) { // Convert x and y to standardized
    x = to_meters(x);   // (inches -> meters)
    y = to_meters(y);   // (inches -> meters)
  }

  TwoDPose p = desired_position.lookAt(
      x, y); // Looking at x and y from the desired position

  // Convert back to respective units
  if (using_degrees()) {
    p.yaw = to_deg(p.yaw); // (radians -> degrees)
  }

  if (using_clockwise()) {
    p.yaw *= -1; // (counter-clockwise -> clockwise)
  }

  turn_to(p.yaw, timeout_seconds);
}

void WhoopDrivetrain::drive_forward(double distance, double timeout_seconds) {
  TwoDPose current_position = desired_position;

  bool reverse = distance < 0;

  // Convert distance to standardized units
  if (using_inches()) {
    distance = to_meters(distance); // (inches -> meters)
  }

  current_position *=
      TwoDPose(0, distance, 0); // Translate forward or backwards the distance

  if (reverse) { // Flip 180 if in reverse
    current_position.yaw = normalize_angle(current_position.yaw + M_PI);
  }

  // Convert back to respecable units
  if (using_inches()) {
    current_position.x = to_inches(current_position.x); // (meters -> inches)
    current_position.y = to_inches(current_position.y); // (meters -> inches)
  }

  if (using_degrees()) {
    current_position.yaw = to_deg(current_position.yaw); // (radian -> degrees)
  }

  if (using_clockwise()) {
    current_position.yaw *= -1; // (counter-clockwise -> clockwise)
  }

  if (reverse) {
    reverse_to_pose(current_position.x, current_position.y,
                    current_position.yaw, timeout_seconds, -1, 0);
  } else {
    drive_to_pose(current_position.x, current_position.y, current_position.yaw,
                  timeout_seconds, -1, 0);
  }
}

void WhoopDrivetrain::drive_to_point(double x, double y, double timeout_seconds,
                                     double landing_strip) {
  drive_through_path({{x, y}}, timeout_seconds, -1, landing_strip);
}

void WhoopDrivetrain::drive_to_pose(double x, double y, double yaw,
                                    double timeout_seconds,
                                    double turning_radius,
                                    double landing_strip) {
  drive_through_path({{x, y, yaw}}, timeout_seconds, turning_radius,
                     landing_strip);
}

void WhoopDrivetrain::reverse_backward(double distance,
                                       double timeout_seconds) {
  drive_forward(-distance, timeout_seconds);
}

void WhoopDrivetrain::reverse_to_point(double x, double y,
                                       double timeout_seconds,
                                       double landing_strip) {
  request_reverse = true;
  drive_to_point(x, y, timeout_seconds, landing_strip);
}

void WhoopDrivetrain::reverse_to_pose(double x, double y, double yaw,
                                      double timeout_seconds,
                                      double turning_radius,
                                      double landing_strip) {
  request_reverse = true;
  drive_to_pose(x, y, yaw, timeout_seconds, turning_radius, landing_strip);
}

void WhoopDrivetrain::reverse_through_path(
    std::vector<std::vector<double>> waypoints, double timeout_seconds,
    double turning_radius, double landing_strip) {
  request_reverse = true;
  drive_through_path(waypoints, timeout_seconds, turning_radius, landing_strip);
}

void WhoopDrivetrain::drive_through_path(
    std::vector<std::vector<double>> waypoints, double timeout_seconds,
    double turning_radius, double landing_strip) {
  this->wait_until_completed(); // Wait before generating path

  std::cout << "Generating Path" << std::endl;
  if (request_reverse) {
    request_reverse = false;
    auton_reverse = true;
  } else {
    auton_reverse = false;
  }

  // Ensure that waypoints are 2 or greater
  size_t waypoints_size = waypoints.size();
  if (waypoints_size < 1) {
#if USE_VEXCODE
    Brain.Screen.print("A path requires at least 1 waypoint");
#else
    // whoop::screen::print_at(1, "A path requires at least 1 waypoint");
#endif
    std::cout << "A path requires at least 1 waypoint" << std::endl;
  }

  // First check the validity of the list of lists to be appropriately
  // structured
  size_t size_of;
  for (size_t i = 0; i < waypoints_size; i++) {
    size_of = waypoints[i].size();
    if (size_of != 2 &&
        size_of != 3) { // If regular waypoint, must be either 2 or 3 variables
#if USE_VEXCODE
      Brain.Screen.print("Waypoints must consist of either 3 variables {x, y, "
                         "yaw}, or 2 variables {x, y}");
#else
      // whoop::screen::print_at(
      //     1, "Waypoints must consist of either 3 variables {x, y, "
      //        "yaw}, or 2 variables {x, y}");
#endif
      std::cout << "Waypoints must consist of either 3 variables {x, y, yaw}, "
                   "or 2 variables {x, y}"
                << std::endl;
    }
  }

  // Converting from inches to standardized meters
  bool convert_to_meters = false;
  if (using_inches()) {
    convert_to_meters = true;
    turning_radius = to_meters(turning_radius); // (inches -> meters)
    landing_strip = to_meters(landing_strip);   // (inches -> meters)
  }

  bool convert_to_radians = false;
  // Converting from degrees to standardized radians
  if (using_degrees()) {
    convert_to_radians = true;
  }

  bool reverse_rotation = false;
  // Flipping from clockwise to standardized counter-clockwise
  if (using_clockwise()) {
    reverse_rotation = true;
  }

  // Create a new waypoints list
  std::vector<std::vector<double>> validated_waypoints;

  // Add start pose to the beginning
  TwoDPose start_pose = odom_fusion->get_pose_2d();
  validated_waypoints.push_back({start_pose.x, start_pose.y, start_pose.yaw});

  TwoDPose target_pose;

  // Adjust to valid sizes with respect to configured units, then add to
  // validated
  for (size_t i = 0; i < waypoints_size; i++) {
    size_of = waypoints[i].size();

    std::vector<double> waypoint_data;
    waypoint_data.push_back(waypoints[i][0]);
    waypoint_data.push_back(waypoints[i][1]);

    // Converting from inches to standardized meters
    if (convert_to_meters) {
      waypoint_data[0] = to_meters(waypoint_data[0]); // (inches -> meters)
      waypoint_data[1] = to_meters(waypoint_data[1]); // (inches -> meters)
    }

    if (size_of == 3) // If contains yaw
    {
      waypoint_data.push_back(waypoints[i][2]);

      if (convert_to_radians)
        waypoint_data[2] = to_rad(waypoint_data[2]); // (degrees -> radians)

      if (reverse_rotation)
        waypoint_data[2] *=
            -1; // (clockwise-positive -> counter-clockwise-positive)
    }
    validated_waypoints.push_back(waypoint_data);

    // If last element, we need to document it
    if (i == waypoints_size - 1) {
      if (size_of == 3) {
        target_pose =
            TwoDPose(waypoint_data[0], waypoint_data[1], waypoint_data[2]);
      } else { // 2
        // Yoink yaw from start pose with its own pose
        target_pose = TwoDPose(waypoint_data[0], waypoint_data[1],
                               validated_waypoints[0][2]);
      }
    }
  }

  // If in reverse, flip the yaw of the first waypoint
  if (auton_reverse) {
    validated_waypoints[0][2] =
        normalize_angle(validated_waypoints[0][2] + M_PI);
  }

  pursuit_conductor.generate_path(validated_waypoints, timeout_seconds,
                                  turning_radius, landing_strip);

  auton_traveling = true;

  // Flip target pose so that the system knows the direction the robot is
  // actually looking
  if (auton_reverse) {
    target_pose.yaw = normalize_angle(target_pose.yaw + M_PI);
  }

  last_desired_position = desired_position;
  desired_position = target_pose;
}

// This is the protocol for calibrating the drivetrain while in a disabled
// state.
void WhoopDrivetrain::run_disabled_calibration_protocol() {
  if (drive_state == drivetrainState::mode_disabled) {
    if (odom_fusion->is_moving()) {
      is_calibrated = false;
      calibration_timer = 0;
      if (moved_one_time_notif) {
        whoop_controller->notify("Robot Moved");
        moved_one_time_notif = false;
      }
    } else if (!is_calibrated &&
               !is_calibrating) { // Stationary and needs calibration
      calibration_timer += 20;
      if (calibration_timer >
          time_until_calibration) { // If stationary for more than period of
                                    // time (like 500 milliseconds) then
                                    // calibrate
        calibrate();
        moved_one_time_notif = true;
      }
    }
  }
}

void WhoopDrivetrain::calibrate() {
  if (is_calibrating) {
    return; // because already calibrating, duh
  }
  whoop_controller->notify("Calibrating Dont Move");

  is_calibrating = true;
  odom_fusion->calibrate();
#if USE_VEXCODE
  wait(2800, msec);
#else
  pros::delay(2800);
#endif
  whoop_controller->notify("Calibration Finished.", 2);
  // Update desired position to 0,0,0
  desired_position = TwoDPose(0, 0, 0);
  last_desired_position = desired_position;
  is_calibrating = false;
  is_calibrated = true;
}

/**
 * Gets the x, y, z, pitch, yaw, roll of the robot
 * @return Pose object. With Pose.x, Pose.y, etc...
 */
Pose WhoopDrivetrain::get_pose() {
  Pose p = odom_fusion->get_pose();

  // Converting from standardized meters to inches
  if (using_inches()) {
    p.x = to_inches(p.x); // (meters -> inches)
    p.y = to_inches(p.y); // (meters -> inches)
    p.z = to_inches(p.z); // (meters -> inches)
  }

  // Converting from standardized radians to degrees
  if (using_degrees()) {
    p.pitch = to_deg(p.pitch); // (radians -> degrees)
    p.roll = to_deg(p.roll);   // (radians -> degrees)
    p.yaw = to_deg(p.yaw);     // (radians -> degrees)
  }

  // Flipping from standardized counter-clockwise to clockwise
  if (using_clockwise()) {
    p.yaw *= -1; // (counter-clockwise-positive -> degrees clockwise-positive)
  }

  return p;
}

/**
 * Sets the pose of the robot
 * @param x_in the location, units configured upon initialization, parallel to
 * the width of the driver station
 * @param y_in the location, units configured upon initialization, perpendicular
 * to the width of the driver station
 * @param yaw_deg the rotation, units configured upon initialization
 */
void WhoopDrivetrain::set_pose(double x, double y, double yaw) {

  // Converting from inches to standardized meters (in -> m)
  if (using_inches()) {
    x = to_meters(x); // (inches -> meters)
    y = to_meters(y); // (inches -> meters)
  }

  // Converting from degrees to standardized radians (deg -> rad)
  if (using_degrees()) {
    yaw = to_rad(yaw); // (degrees -> radians)
  }

  // Flipping from clockwise to standardized counter-clockwise (cw -> ccw)
  if (using_clockwise()) {
    yaw *= -1; // (counter-clockwise-positive -> clockwise-positive)
  }

  odom_fusion->tare(x, y, yaw);

  // Update with respective position
  desired_position = TwoDPose(x, y, yaw);
  last_desired_position = desired_position;
}

void WhoopDrivetrain::wait_until_completed(double additional_time_msec) {
  while (auton_traveling) {
#if USE_VEXCODE
    wait(5, msec);
#else
    pros::delay(5);
#endif
  }
#if USE_VEXCODE
  wait(additional_time_msec, msec);
#else
  pros::delay(additional_time_msec);
#endif
}

void WhoopDrivetrain::fuse(double seconds) {
  odom_fusion->accept_fuses();
#if USE_VEXCODE
  wait(seconds, sec);
#else
  pros::delay(seconds * 1000);
#endif
  odom_fusion->reject_fuses();
}

void WhoopDrivetrain::step_usercontrol() {
  switch (whoop_controller->joystick_mode) {
  case joystickmode::joystickmode_tank:
    left_motor_group->spin_percentage(whoop_controller->get_left_joystick_y());
    right_motor_group->spin_percentage(
        whoop_controller->get_right_joystick_y());
    break;
  case joystickmode::joystickmode_split_arcade:
    left_motor_group->spin_percentage(whoop_controller->get_left_joystick_y() +
                                      whoop_controller->get_right_joystick_x());
    right_motor_group->spin_percentage(
        whoop_controller->get_left_joystick_y() -
        whoop_controller->get_right_joystick_x());
    break;
  case joystickmode::joystickmode_left_arcade:
    left_motor_group->spin_percentage(whoop_controller->get_left_joystick_y() +
                                      whoop_controller->get_left_joystick_x());
    right_motor_group->spin_percentage(whoop_controller->get_left_joystick_y() -
                                       whoop_controller->get_left_joystick_x());
    break;
  case joystickmode::joystickmode_right_arcade:
    left_motor_group->spin_percentage(whoop_controller->get_right_joystick_y() +
                                      whoop_controller->get_right_joystick_x());
    right_motor_group->spin_percentage(
        whoop_controller->get_right_joystick_y() -
        whoop_controller->get_right_joystick_x());
    break;
  }
}

void WhoopDrivetrain::step_disabled() {
  left_motor_group->spin(0);
  right_motor_group->spin(0);
  run_disabled_calibration_protocol();
}

void WhoopDrivetrain::step_autonomous() {

  if (auton_traveling) {
    TwoDPose robot_pose = odom_fusion->get_pose_2d();
    if (auton_reverse) {
      robot_pose.yaw = normalize_angle(robot_pose.yaw + M_PI);
    }

    pursuit_result = pursuit_conductor.step(robot_pose);
    if (temp_disable) {
      left_motor_group->spin(0);
      right_motor_group->spin(0);
      return;
    }

    if (pursuit_result.is_completed) {
      auton_traveling = false;
      left_motor_group->spin(0);
      right_motor_group->spin(0);
      return;
    }

    if (!pursuit_result.is_valid) {
      left_motor_group->spin(0);
      right_motor_group->spin(0);
      auton_traveling = false;
      return;
    }

    if (pursuit_conductor.forward_pid.is_settled() ||
        pursuit_result.suggest_point_turn) {
      if (auton_reverse) {
        left_motor_group->spin(-pursuit_result.forward_power -
                               pursuit_result.steering_power / 1.5);
        right_motor_group->spin(-pursuit_result.forward_power +
                                pursuit_result.steering_power / 1.5);
      } else {
        left_motor_group->spin(pursuit_result.forward_power -
                               pursuit_result.steering_power / 1.5);
        right_motor_group->spin(pursuit_result.forward_power +
                                pursuit_result.steering_power / 1.5);
      }
    } else {
      if (auton_reverse) {
        left_motor_group->spin(-pursuit_result.forward_power +
                               std::max(-pursuit_result.steering_power, 0.0));
        right_motor_group->spin(-pursuit_result.forward_power +
                                std::max(pursuit_result.steering_power, 0.0));
      } else {
        left_motor_group->spin(pursuit_result.forward_power +
                               std::min(-pursuit_result.steering_power, 0.0));
        right_motor_group->spin(pursuit_result.forward_power +
                                std::min(pursuit_result.steering_power, 0.0));
      }
    }
  } else {
    left_motor_group->spin(0);
    right_motor_group->spin(0);
  }
}

void WhoopDrivetrain::__step() {
  odom_fusion->__step(); // Step odometry fusion module

  switch (drive_state) {
  case drivetrainState::mode_usercontrol:
    step_usercontrol();
    break;
  case drivetrainState::mode_autonomous:
    step_autonomous();
    break;
  case drivetrainState::mode_disabled:
    step_disabled();
    break;
  }
}

/**
 * Gets units that the odometry is using
 * @returns units desciber, as a string
 */
std::string WhoopDrivetrain::get_units_str() {
  switch (pose_units) {
  case m_deg_cw:
    return "m_deg_cw";
  case m_deg_ccw:
    return "m_deg_ccw";
  case m_rad_cw:
    return "m_rad_cw";
  case m_rad_ccw:
    return "m_rad_ccw";
  case in_deg_cw:
    return "in_deg_cw";
  case in_deg_ccw:
    return "in_deg_ccw";
  case in_rad_cw:
    return "in_rad_cw";
  case in_rad_ccw:
    return "in_rad_ccw";
  default:
    return "Unknown units";
  }
}

/**
 * Gets units that the odometry is using
 * @returns units desciber, as a Pose/UnitsObject
 */
PoseUnits WhoopDrivetrain::get_units() { return pose_units; }

void WhoopDrivetrain::set_pose_units(PoseUnits units) { pose_units = units; }

} // namespace whoop
