/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       WhoopDriveOdomUnit.hpp                                    */
/*    Author:       Connor White                                              */
/*    Created:      Thu Jun 21 2024                                           */
/*    Description:  Odometry Module for Pose Estimation                       */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#ifndef WHOOP_DRIVE_ODOM_UNIT_HPP
#define WHOOP_DRIVE_ODOM_UNIT_HPP

#include "whooplib/include/calculators/WheelOdom.hpp"
#include "whooplib/include/devices/WhoopInertial.hpp"
#include "whooplib/include/devices/WhoopMotor.hpp"
#include "whooplib/include/devices/WhoopMotorGroup.hpp"
#include "whooplib/include/devices/WhoopMutex.hpp"
#include "whooplib/include/devices/WhoopRotation.hpp"
#include "whooplib/include/devices/WhoopVision.hpp"
#include "whooplib/include/nodes/NodeManager.hpp"
#include "whooplib/includer.hpp"
#include <memory>
#include <vector>

namespace whoop {

// Enum for configuring odom
enum class DriveOdomConfig {
  DRIVE_ONLY,
  DRIVE_WITH_SIDEWAYS_TRACKER,
  DRIVE_WITH_BOTH_TRACKERS,
};

/**
 * Class responsible for managing the odometry unit.
 */
class WhoopDriveOdomUnit : public ComputeNode, public WheelOdom {
private:
  // Upon initialization
  std::unique_ptr<WhoopMotorGroup> left_motor_group =
      nullptr; // Group of motors on the left side of the drivetrain.
  std::unique_ptr<WhoopMotorGroup> right_motor_group =
      nullptr; // Group of motors on the right side of the drivetrain.
  WhoopRotation *forward_tracker = nullptr;
  WhoopRotation *sideways_tracker = nullptr;
  DriveOdomConfig drive_odom_config;

  // Initializes motor groups directly from pointers.
  void init_motor_groups(WhoopMotorGroup *leftGroup,
                         WhoopMotorGroup *rightGroup);
  // Initializes motor groups from a vector of motors.
  void init_motor_groups(const std::vector<WhoopMotor *> &leftMotors,
                         const std::vector<WhoopMotor *> &rightMotors);
  // Configures motor groups for diameter and gear ratio
  void set_motor_ratio_and_diameter(double wheel_diameter_meters,
                                    double gear_ratio);

  /**
   * Sets the gear ratio multiplier for the drivetrain.
   * i.e. motor on 32 tooth powering the 64 toth: ratio = 32.0/64.0 = 0.5
   * @param ratio The new gear ratio to apply.
   */
  void
  set_motor_gear_ratio_mult(double ratio); // motor on 32 tooth powering the 64
                                           // toth: ratio = 32.0/64.0

  /**
   * Sets the wheel diameter multiplier for the drivetrain, in meters
   * @param diameter_meters The wheel diameter in meters (i.e. 0.08255 for 3.25"
   * wheels)
   */
  void set_motor_wheel_diameter(double diameter_meters);

public:
  WhoopInertial *inertial_sensor;

  TwoDPose pose = TwoDPose(0, 0, 0);
  WhoopMutex
      thread_lock; // Mutex for synchronizing access to odometry components.

  /**
   * Constructor for Drive Odom.
   * The odom unit center is the virtual intercept of the perpendicular faces of
   * the odometry trackers. Visual Representation of Tracker Distances:
   * https://imgur.com/rWCCCfz
   * @param drive_width Distance between the left and right wheels, in meters.
   * @param drive_wheel_diameter_meters Diameter of the wheel, in meters (e.g.,
   * 0.08255 for 3.25-inch wheels).
   * @param drive_gear_ratio motor on 32 tooth powering the 64 tooth: ratio
   * = 32.0/64.0
   * @param inertialSensor Pointer to the WhoopInertial sensor for tracking.
   * @param leftMotorGroup Pointer to the motor group controlling the left side.
   * @param rightMotorGroup Pointer to the motor group controlling the right
   * side.
   */
  WhoopDriveOdomUnit(double drive_width, double drive_wheel_diameter_meters,
                     double drive_gear_ratio, WhoopInertial *inertialSensor,
                     WhoopMotorGroup *leftMotorGroup,
                     WhoopMotorGroup *rightMotorGroup);

  /**
   * Constructor for Drive Odom.
   * The odom unit center is the virtual intercept of the perpendicular faces of
   * the odometry trackers. Visual Representation of Tracker Distances:
   * https://imgur.com/rWCCCfz
   * @param drive_center_distance Distance between the center of the robot and
   * the right wheels, in meters.
   * @param drive_wheel_diameter_meters Diameter of the wheel, in meters (e.g.,
   * 0.08255 for 3.25-inch wheels).
   * @param drive_gear_ratio motor on 32 tooth powering the 64 tooth: ratio
   * = 32.0/64.0
   * @param sideways_tracker_distance Distance from the odom unit center to the
   * sideways tracker, in meters (positive implies a shift forward from the odom
   * unit center).
   * @param sideways_tracker_wheel_diameter_meters Diameter of the sideways
   * tracker wheel, in meters (e.g., 0.08255 for 3.25-inch wheels).
   * @param inertialSensor Pointer to the WhoopInertial sensor for tracking.
   * @param sideways_tracker Pointer to the WhoopRotation sensor for the
   * sideways tracker.
   * @param leftMotorGroup Pointer to the motor group controlling the left side.
   * @param rightMotorGroup Pointer to the motor group controlling the right
   * side.
   */
  WhoopDriveOdomUnit(double drive_center_distance,
                     double drive_wheel_diameter_meters,
                     double drive_gear_ratio, double sideways_tracker_distance,
                     double sideways_tracker_wheel_diameter_meters,
                     WhoopInertial *inertialSensor,
                     WhoopRotation *sideways_tracker,
                     WhoopMotorGroup *leftMotorGroup,
                     WhoopMotorGroup *rightMotorGroup);

  /**
   * Constructor for Drive Odom.
   * The odom unit center is the virtual intercept of the perpendicular faces of
   * the odometry trackers. Visual Representation of Tracker Distances:
   * https://imgur.com/rWCCCfz
   * @param forward_tracker_distance Distance from the odom unit center to the
   * forward tracker, in meters (positive implies a shift to the right from the
   * odom unit center).
   * @param forward_tracker_wheel_diameter_meters Diameter of the forward
   * tracker wheel, in meters (e.g., 0.08255 for 3.25-inch wheels).
   * @param sideways_tracker_distance Distance from the odom unit center to the
   * sideways tracker, in meters (positive implies a shift forward from the odom
   * unit center).
   * @param sideways_tracker_wheel_diameter_meters Diameter of the sideways
   * tracker wheel, in meters (e.g., 0.08255 for 3.25-inch wheels).
   * @param inertialSensor Pointer to the WhoopInertial sensor for tracking.
   * @param forward_tracker Pointer to the WhoopRotation sensor for the forward
   * tracker.
   * @param sideways_tracker Pointer to the WhoopRotation sensor for the
   * sideways tracker.
   */
  WhoopDriveOdomUnit(double forward_tracker_distance,
                     double forward_tracker_wheel_diameter_meters,
                     double sideways_tracker_distance,
                     double sideways_tracker_wheel_diameter_meters,
                     WhoopInertial *inertialSensor,
                     WhoopRotation *forward_tracker,
                     WhoopRotation *sideways_tracker);

  /**
   * Constructor for Drive Odom.
   * The odom unit center is the virtual intercept of the perpendicular faces of
   * the odometry trackers. Visual Representation of Tracker Distances:
   * https://imgur.com/rWCCCfz
   * @param drive_width Distance between the left and right wheels, in meters.
   * @param drive_wheel_diameter_meters Diameter of the wheel, in meters (e.g.,
   * 0.08255 for 3.25-inch wheels).
   * @param drive_gear_ratio motor on 32 tooth powering the 64 tooth: ratio
   * = 32.0/64.0
   * @param inertialSensor Pointer to the WhoopInertial sensor for tracking.
   * @param leftMotors Vector of motors on the left side.
   * @param rightMotors Vector of motors on the right side.
   */
  WhoopDriveOdomUnit(double drive_width, double drive_wheel_diameter_meters,
                     double drive_gear_ratio, WhoopInertial *inertialSensor,
                     std::vector<WhoopMotor *> leftMotors,
                     std::vector<WhoopMotor *> rightMotors);

  /**
   * Constructor for Drive Odom.
   * The odom unit center is the virtual intercept of the perpendicular faces of
   * the odometry trackers. Visual Representation of Tracker Distances:
   * https://imgur.com/rWCCCfz
   * @param drive_center_distance Distance between the center of the robot and
   * the right wheels, in meters.
   * @param drive_wheel_diameter_meters Diameter of the drive wheels, in meters
   * (e.g., 0.08255 for 3.25-inch wheels).
   * @param drive_gear_ratio motor on 32 tooth powering the 64 tooth: ratio
   * = 32.0/64.0
   * @param sideways_tracker_distance Distance from the odom unit center to the
   * sideways tracker, in meters (positive implies a shift forward from the odom
   * unit center).
   * @param sideways_tracker_wheel_diameter_meters Diameter of the sideways
   * tracker wheel, in meters (e.g., 0.08255 for 3.25-inch wheels).
   * @param inertialSensor Pointer to the WhoopInertial sensor for tracking.
   * @param sideways_tracker Pointer to the WhoopRotation sensor for the
   * sideways tracker.
   * @param leftMotors Vector of pointers to motors on the left side.
   * @param rightMotors Vector of pointers to motors on the right side.
   */
  WhoopDriveOdomUnit(double drive_center_distance,
                     double drive_wheel_diameter_meters,
                     double drive_gear_ratio, double sideways_tracker_distance,
                     double sideways_tracker_wheel_diameter_meters,
                     WhoopInertial *inertialSensor,
                     WhoopRotation *sideways_tracker,
                     std::vector<WhoopMotor *> leftMotors,
                     std::vector<WhoopMotor *> rightMotors);

  /**
   * Calibrates the IMU and tares all devices
   */
  void calibrate();

  // Taring (resetting) methods for the pose estimation.
  void tare(double x, double y, double yaw);
  void tare();

  // Returns true if the system is moving
  bool is_moving(double rads_s_threshold = 0.02);

  /**
   * Retrieves the corrected and computed pose.
   * @return The current pose of the system.
   */
  TwoDPose get_pose();

public: // This is one of the ONLY exceptions to be public, as another module
        // requires this step function.
  /**
   * Override of ComputeNode's __step method to update the drivetrain's
   * operation each cycle.
   */
  void __step() override; // Protected helper function for processing steps
};

} // namespace whoop

#endif // WHOOP_DRIVE_ODOM_UNIT_HPP
