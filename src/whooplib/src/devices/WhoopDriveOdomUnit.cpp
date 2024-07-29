/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       WhoopDriveOdomUnit.cpp                                    */
/*    Author:       Connor White (WHOOP)                                      */
/*    Created:      Thu Jun 21 2024                                           */
/*    Description:  Odometry Module for Pose Estimation                       */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "whooplib/include/devices/WhoopDriveOdomUnit.hpp"
#include "whooplib/includer.hpp"
#include <cmath>
#include <memory>
#include <vector>

namespace whoop {

WhoopDriveOdomUnit::WhoopDriveOdomUnit(double drive_width,
                                       double drive_wheel_diameter_meters,
                                       double drive_gear_ratio,
                                       WhoopInertial *inertialSensor,
                                       WhoopMotorGroup *leftMotorGroup,
                                       WhoopMotorGroup *rightMotorGroup)
    : inertial_sensor(inertialSensor) {
  init_motor_groups(leftMotorGroup, rightMotorGroup);
  set_motor_ratio_and_diameter(drive_wheel_diameter_meters, drive_gear_ratio);
  set_physical_distances(drive_width / 2.0, 0); // From odom class
  drive_odom_config = DriveOdomConfig::DRIVE_ONLY;
}

WhoopDriveOdomUnit::WhoopDriveOdomUnit(
    double drive_center_distance, double drive_wheel_diameter_meters,
    double drive_gear_ratio, double sideways_tracker_distance,
    double sideways_tracker_wheel_diameter_meters,
    WhoopInertial *inertialSensor, WhoopRotation *sideways_tracker,
    WhoopMotorGroup *leftMotorGroup, WhoopMotorGroup *rightMotorGroup)
    : inertial_sensor(inertialSensor) {
  init_motor_groups(leftMotorGroup, rightMotorGroup);
  set_motor_ratio_and_diameter(drive_wheel_diameter_meters, drive_gear_ratio);
  this->sideways_tracker = sideways_tracker;
  sideways_tracker->set_wheel_diameter(sideways_tracker_wheel_diameter_meters);
  set_physical_distances(drive_center_distance,
                         sideways_tracker_distance); // From odom class
  drive_odom_config = DriveOdomConfig::DRIVE_WITH_SIDEWAYS_TRACKER;
}

WhoopDriveOdomUnit::WhoopDriveOdomUnit(
    double forward_tracker_distance,
    double forward_tracker_wheel_diameter_meters,
    double sideways_tracker_distance,
    double sideways_tracker_wheel_diameter_meters,
    WhoopInertial *inertialSensor, WhoopRotation *forward_tracker,
    WhoopRotation *sideways_tracker)
    : inertial_sensor(inertialSensor) {
  this->forward_tracker = forward_tracker;
  this->sideways_tracker = sideways_tracker;
  forward_tracker->set_wheel_diameter(sideways_tracker_wheel_diameter_meters);
  sideways_tracker->set_wheel_diameter(sideways_tracker_wheel_diameter_meters);
  set_physical_distances(forward_tracker_distance,
                         sideways_tracker_distance); // From odom class
  drive_odom_config = DriveOdomConfig::DRIVE_WITH_BOTH_TRACKERS;
}

WhoopDriveOdomUnit::WhoopDriveOdomUnit(double drive_width,
                                       double drive_wheel_diameter_meters,
                                       double drive_gear_ratio,
                                       WhoopInertial *inertialSensor,
                                       std::vector<WhoopMotor *> leftMotors,
                                       std::vector<WhoopMotor *> rightMotors)
    : inertial_sensor(inertialSensor) {
  init_motor_groups(leftMotors, rightMotors);
  set_motor_ratio_and_diameter(drive_wheel_diameter_meters, drive_gear_ratio);
  set_physical_distances(drive_width / 2.0, 0); // From odom class
  drive_odom_config = DriveOdomConfig::DRIVE_ONLY;
}

WhoopDriveOdomUnit::WhoopDriveOdomUnit(
    double drive_center_distance, double drive_wheel_diameter_meters,
    double drive_gear_ratio, double sideways_tracker_distance,
    double sideways_tracker_wheel_diameter_meters,
    WhoopInertial *inertialSensor, WhoopRotation *sideways_tracker,
    std::vector<WhoopMotor *> leftMotors, std::vector<WhoopMotor *> rightMotors)
    : inertial_sensor(inertialSensor) {
  init_motor_groups(leftMotors, rightMotors);
  set_motor_ratio_and_diameter(drive_wheel_diameter_meters, drive_gear_ratio);
  this->sideways_tracker = sideways_tracker;
  sideways_tracker->set_wheel_diameter(sideways_tracker_wheel_diameter_meters);
  set_physical_distances(drive_center_distance,
                         sideways_tracker_distance); // From odom class
  drive_odom_config = DriveOdomConfig::DRIVE_WITH_SIDEWAYS_TRACKER;
}

// Configures motor groups for diameter and gear ratio
void WhoopDriveOdomUnit::set_motor_ratio_and_diameter(
    double wheel_diameter_meters, double gear_ratio) {
  set_motor_gear_ratio_mult(gear_ratio);
  set_motor_wheel_diameter(wheel_diameter_meters);
}

// Initializes motor groups directly from pointers.
void WhoopDriveOdomUnit::init_motor_groups(WhoopMotorGroup *leftGroup,
                                           WhoopMotorGroup *rightGroup) {
  left_motor_group = std::make_unique<WhoopMotorGroup>(*leftGroup);
  right_motor_group = std::make_unique<WhoopMotorGroup>(*rightGroup);
}
// Initializes motor groups from a vector of motors.
void WhoopDriveOdomUnit::init_motor_groups(
    const std::vector<WhoopMotor *> &leftMotors,
    const std::vector<WhoopMotor *> &rightMotors) {
  left_motor_group = std::make_unique<WhoopMotorGroup>(leftMotors);
  right_motor_group = std::make_unique<WhoopMotorGroup>(rightMotors);
}

void WhoopDriveOdomUnit::set_motor_gear_ratio_mult(double ratio) {
  left_motor_group->set_gear_ratio_mult(ratio);
  right_motor_group->set_gear_ratio_mult(ratio);
}

void WhoopDriveOdomUnit::set_motor_wheel_diameter(double diameter_meters) {
  left_motor_group->set_wheel_diameter(diameter_meters);
  right_motor_group->set_wheel_diameter(diameter_meters);
}

void WhoopDriveOdomUnit::calibrate() {
  inertial_sensor->calibrate();
  tare();
}

void WhoopDriveOdomUnit::tare(double x, double y, double yaw) {
  thread_lock.lock();
  inertial_sensor->tare_radians(yaw);
  set_position(x, y, yaw);
  thread_lock.unlock();
}

void WhoopDriveOdomUnit::tare() { tare(0, 0, 0); }

TwoDPose WhoopDriveOdomUnit::get_pose() {
  thread_lock.lock();
  TwoDPose result = pose;
  thread_lock.unlock();
  return result;
}

bool WhoopDriveOdomUnit::is_moving(double rads_s_threshold) {
  double rad_s = 0;

  if (left_motor_group != nullptr)
    rad_s += std::abs(left_motor_group->get_velocity_rad_s());
  if (right_motor_group != nullptr)
    rad_s += std::abs(right_motor_group->get_velocity_rad_s());
  if (forward_tracker != nullptr)
    rad_s += std::abs(forward_tracker->get_velocity_rad_s());
  if (sideways_tracker != nullptr)
    rad_s += std::abs(sideways_tracker->get_velocity_rad_s());

  if (rad_s > rads_s_threshold) {
    return true;
  }
  return false;
}

void WhoopDriveOdomUnit::__step() {
  thread_lock.lock();
  if (drive_odom_config == DriveOdomConfig::DRIVE_ONLY) {
    update_pose(right_motor_group->get_distance_meters(), 0,
                inertial_sensor->get_yaw_radians());
  } else if (drive_odom_config ==
             DriveOdomConfig::DRIVE_WITH_SIDEWAYS_TRACKER) {
    update_pose(right_motor_group->get_distance_meters(),
                sideways_tracker->get_distance_meters(),
                inertial_sensor->get_yaw_radians());
  } else if (drive_odom_config == DriveOdomConfig::DRIVE_WITH_BOTH_TRACKERS) {
    update_pose(forward_tracker->get_distance_meters(),
                sideways_tracker->get_distance_meters(),
                inertial_sensor->get_yaw_radians());
  }

  pose.x = X_position;
  pose.y = Y_position;
  pose.yaw = orientation_rad;
  thread_lock.unlock();
}

} // namespace whoop
