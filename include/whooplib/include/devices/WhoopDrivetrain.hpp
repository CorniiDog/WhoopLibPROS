/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       WhoopDrivetrain.hpp                                       */
/*    Author:       Connor White (WHOOP)                                      */
/*    Created:      Thu Jun 21 2024                                           */
/*    Description:  Virtual Drivetrain for Controlling Chassis                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#ifndef WHOOP_DRIVETRAIN_HPP
#define WHOOP_DRIVETRAIN_HPP

#include "whooplib/include/calculators/PurePursuitConductor.hpp"
#include "whooplib/include/calculators/WheelOdom.hpp"
#include "whooplib/include/devices/WhoopController.hpp"
#include "whooplib/include/devices/WhoopMotor.hpp"
#include "whooplib/include/devices/WhoopMotorGroup.hpp"
#include "whooplib/include/devices/WhoopMutex.hpp"
#include "whooplib/include/devices/WhoopOdomFusion.hpp"
#include "whooplib/include/nodes/BufferNode.hpp"
#include "whooplib/include/nodes/NodeManager.hpp"
#include "whooplib/includer.hpp"
#include <memory>
#include <vector>

namespace whoop {

/**
 * Enum representing the possible states of the drivetrain.
 */
enum drivetrainState {
  mode_disabled = 1, // The drivetrain is disabled and not responsive to input.
  mode_autonomous = 2, // The drivetrain is operating under autonomous control.
  mode_usercontrol = 3 // The drivetrain is responsive to user control.
};

/**
 * Enum representing the pose units for movement
 */
enum PoseUnits {
  m_deg_cw,   // meters and degrees clockwise
  m_deg_ccw,  // meters and degrees counter-clockwise
  m_rad_cw,   // meters and radians clockwise
  m_rad_ccw,  // meters and radians counter-clockwise
  in_deg_cw,  // inches and degrees clockwise
  in_deg_ccw, // inches and degrees counter-clockwise
  in_rad_cw,  // inches and radians clockwise
  in_rad_ccw, // inches and radians counter-clockwise
};

/**
 * Enum representing the blocking mode to use
 */
enum waitUntilCompleted {
  no_wait, // Continues to next line even while the action is running
  yes_wait // Yields until the action is completed
};

/**
 * Class responsible for managing the drivetrain of a robot, including motor
 * control and state management.
 */
class WhoopDrivetrain : public ComputeNode {
private:
  // Calibration protocol settings
  double time_until_calibration = 1000; // ms
  // Calibration protocol Modifiables
  bool needs_calibration = true;
  double calibration_timer = 0;
  bool moved_one_time_notif = false;
  // This runs the calibration protocol for the drivetrain
  void run_disabled_calibration_protocol();

  void step_usercontrol();
  void step_disabled();
  void step_autonomous();

  bool auton_traveling = false;
  bool auton_reverse = false;
  bool request_reverse = false;

protected:
  // Upon initialization
  WhoopController *whoop_controller; // Controller object for receiving input
                                     // from VEX controllers.
  std::unique_ptr<WhoopMotorGroup>
      left_motor_group; // Group of motors on the left side of the drivetrain.
  std::unique_ptr<WhoopMotorGroup>
      right_motor_group; // Group of motors on the right side of the drivetrain.
  WhoopOdomFusion
      *odom_fusion; // Group of motors on the right side of the drivetrain.
  PoseUnits pose_units = PoseUnits::m_rad_ccw;
  PoseUnits default_pose_units = PoseUnits::m_rad_ccw;

  PurePursuitConductor pursuit_conductor;

  bool drive_calibrated = false;

  bool autonomous_driving = false;

  PursuitResult pursuit_result;

  TwoDPose desired_position;
  TwoDPose last_desired_position;

private:
  // Initializes motor groups directly from pointers.
  void init_motor_groups(WhoopMotorGroup *leftGroup,
                         WhoopMotorGroup *rightGroup);
  // Initializes motor groups from a vector of motors.
  void init_motor_groups(const std::vector<WhoopMotor *> &leftMotors,
                         const std::vector<WhoopMotor *> &rightMotors);

  bool using_inches();
  bool using_degrees();
  bool using_clockwise();

public:
  bool temp_disable = false; // Set to true to temp disable drivetrain

  WhoopMutex
      thread_lock; // Mutex for synchronizing access to drivetrain components.
  drivetrainState drive_state =
      drivetrainState::mode_disabled; // Current operational state of the
                                      // drivetrain.

  /**
   * Constructor for initializing the drivetrain with predefined motor groups.
   * @param default_pursuit_parameters The default pure pursuit parameters for
   * operating the robot in autonomous
   * @param odom_fusion The odometry fusion system for pose estimation
   * @param pose_units Configure the units for the odometry. "m_deg_cw" means
   * "meters, degrees, clockwise-positive yaw", "in_deg_ccw" means "inches,
   * degrees, counter-clockwise-positive yaw", and so forth.
   * @param controller Pointer to the controller managing user input.
   * @param leftMotorGroup Pointer to the motor group controlling the left side.
   * @param rightMotorGroup Pointer to the motor group controlling the right
   * side.
   */
  WhoopDrivetrain(PursuitParams *default_pursuit_parameters,
                  WhoopOdomFusion *odom_fusion, PoseUnits pose_units,
                  WhoopController *controller, WhoopMotorGroup *leftMotorGroup,
                  WhoopMotorGroup *rightMotorGroup);

  /**
   * Constructor for initializing the drivetrain with a list of motors for each
   * side.
   * @param default_pursuit_parameters The default pure pursuit parameters for
   * operating the robot in autonomous
   * @param odom_fusion The odometry fusion system for pose estimation
   * @param pose_units Configure the units for the odometry. "m_deg_cw" means
   * "meters, degrees, clockwise-positive yaw", "in_deg_ccw" means "inches,
   * degrees, counter-clockwise-positive yaw", and so forth.
   * @param controller Pointer to the controller managing user input.
   * @param leftMotors Vector of motors on the left side.
   * @param rightMotors Vector of motors on the right side.
   */
  WhoopDrivetrain(PursuitParams *default_pursuit_parameters,
                  WhoopOdomFusion *odom_fusion, PoseUnits pose_units,
                  WhoopController *controller,
                  std::vector<WhoopMotor *> leftMotors,
                  std::vector<WhoopMotor *> rightMotors);

  /**
   * Turns the robot relative to its current yaw
   * @param angle The angle, yaw rotation to rotate by from the robot's current
   * position
   * @param timeout_seconds The timeout of the movement, in seconds
   */
  void turn(double angle, double timeout_seconds = -1);

  /**
   * Turns the robot to the world's yaw
   * @param yaw The yaw rotation to rotate to
   * @param timeout_seconds The timeout of the movement, in seconds
   */
  void turn_to(double yaw, double timeout_seconds = -1);

  /**
   * Turns the robot to face the given x and y
   * @param x The x position to face
   * @param y The y position to face
   * @param timeout_seconds The timeout of the movement, in seconds
   */
  void turn_to_position(double x, double y, double timeout_seconds = -1);

  /**
   * Drive the robot forward the respectable distance
   * @param distance The x position to travel to, in specified units configured
   * @param timeout_seconds The The timeout of the movement, in seconds
   */
  void drive_forward(double distance, double timeout_seconds = -1);

  /**
   * This drives to a designated point using pure pursuit on a dubins curve
   * @param x The x position to travel to, in specified units configured
   * @param y The y position to travel to, in specified units configured
   * @param timeout_seconds The timeout of the movement, in seconds
   * @param landing_strip Set to -1 to default to a landing strip of
   * lookahead_distance. Set to 0 to have no landing strip for a movemnet. Set
   * to a positive non-zero to change the length
   */
  void drive_to_point(double x, double y, double timeout_seconds = -1,
                      double landing_strip = -1);

  /**
   * This drives to a designated pose using pure pursuit on a dubins curve
   * @param x The x position to travel to, in specified units configured
   * @param y The y position to travel to, in specified units configured
   * @param yaw The yaw rotation to travel to, in specified units configured
   * @param timeout_seconds The timeout of the movement, in seconds
   * @param turning_radius The radius, in meters, of the turning
   * @param landing_strip Set to -1 to default to a landing strip of
   * lookahead_distance. Set to 0 to have no landing strip for a movemnet. Set
   * to a positive non-zero to change the length
   */
  void drive_to_pose(double x, double y, double yaw,
                     double timeout_seconds = -1, double turning_radius = -1,
                     double landing_strip = -1);

  /**
   * This drives to a designated pose using pure pursuit on a dubins curve
   * @param waypoints The waypoints for generating the path. Example would be
   * {TwoDPose(0,0,0), TwoDPose(20,10,M_PI_2)} The yaw for each position in the
   * list must be explicitly stated when using TwoDPose objects
   * @param timeout_seconds The timeout of the movement, in seconds
   * @param turning_radius The radius, in meters, of the turning
   * @param landing_strip Set to -1 to default to a landing strip of
   * lookahead_distance. Set to 0 to have no landing strip for a movemnet. Set
   * to a positive non-zero to change the length
   */
  void drive_through_path(std::vector<std::vector<double>> waypoints,
                          double timeout_seconds = -1,
                          double turning_radius = -1,
                          double landing_strip = -1);

  /**
   * Drive the robot backward the respectable distance
   * @param distance The x position to travel to, in specified units configured
   * @param timeout_seconds The The timeout of the movement, in seconds
   */
  void reverse_backward(double distance, double timeout_seconds = -1);

  /**
   * This drives to a designated point using pure pursuit on a dubins curve, in
   * reverse
   * @param x The x position to travel to, in specified units configured
   * @param y The y position to travel to, in specified units configured
   * @param timeout_seconds The timeout of the movement, in seconds
   * @param landing_strip Set to -1 to default to a landing strip of
   * lookahead_distance. Set to 0 to have no landing strip for a movemnet. Set
   * to a positive non-zero to change the length
   */
  void reverse_to_point(double x, double y, double timeout_seconds = -1,
                        double landing_strip = -1);

  /**
   * This drives to a designated pose using pure pursuit on a dubins curve, in
   * reverse
   * @param x The x position to travel to, in specified units configured
   * @param y The y position to travel to, in specified units configured
   * @param yaw The yaw rotation to travel to, in specified units configured
   * @param timeout_seconds The timeout of the movement, in seconds
   * @param turning_radius The radius, in meters, of the turning
   * @param landing_strip Set to -1 to default to a landing strip of
   * lookahead_distance. Set to 0 to have no landing strip for a movemnet. Set
   * to a positive non-zero to change the length
   */
  void reverse_to_pose(double x, double y, double yaw,
                       double timeout_seconds = -1, double turning_radius = -1,
                       double landing_strip = -1);

  /**
   * This drives to a designated pose using pure pursuit on a dubins curve, in
   * reverse
   * @param waypoints The waypoints for generating the path. Example would be
   * {TwoDPose(0,0,0), TwoDPose(20,10,M_PI_2)} The yaw for each position in the
   * list must be explicitly stated when using TwoDPose objects
   * @param timeout_seconds The timeout of the movement, in seconds
   * @param turning_radius The radius, in meters, of the turning
   * @param landing_strip Set to -1 to default to a landing strip of
   * lookahead_distance. Set to 0 to have no landing strip for a movemnet. Set
   * to a positive non-zero to change the length
   */
  void reverse_through_path(std::vector<std::vector<double>> waypoints,
                            double timeout_seconds = -1,
                            double turning_radius = -1,
                            double landing_strip = -1);

  /**
   * Waits until a drivetrain action during auton is complete.
   */
  void wait_until_completed(double additional_time_msec = 0);

  /**
   * Sets the operational state of the drivetrain.
   * @param state The new state to set (disabled, autonomous, or user control).
   */
  void set_state(drivetrainState state);

  /**
   * Calibrates the robot drivetrain
   */
  void calibrate();

  /**
   * Sets the pose units for the system. This can be changed at any point in
   * time, even mid-autonomous
   */
  void set_pose_units(PoseUnits units);

  /**
   * Gets the x, y, z, pitch, yaw, roll of the robot
   * @return Pose object. With Pose.x, Pose.y, etc...
   */
  Pose get_pose();

  /**
   * Sets the pose of the robot
   * @param x_in the location, in inches, parallel to the width of the driver
   * station
   * @param y_in the location, in inches, perpendicular to the width of the
   * driver station
   * @param yaw_deg the rotation, clockwise-positive, in degrees
   */
  void set_pose(double x_in, double y_in, double yaw_deg);

  /**
   * Gets units that the odometry is using
   * @returns units desciber, as a string
   */
  std::string get_units_str();

  /**
   * Gets units that the odometry is using
   * @returns units desciber, as a Pose/UnitsObject
   */
  PoseUnits get_units();

  void fuse(double seconds);

protected:
  /**
   * Override of ComputeNode's __step method to update the drivetrain's
   * operation each cycle.
   */
  void __step() override; // Protected helper function for processing steps
};

} // namespace whoop

#endif // WHOOP_DRIVETRAIN_HPP
