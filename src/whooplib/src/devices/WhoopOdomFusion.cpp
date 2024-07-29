/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       WhoopOdomFusion.cpp                                       */
/*    Author:       Connor White (WHOOP)                                      */
/*    Created:      Thu Jun 25 2024                                           */
/*    Description:  Fuses Wheel and Visual Odometry                           */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "whooplib/include/devices/WhoopOdomFusion.hpp"
#include "whooplib/include/devices/WhoopDrivetrain.hpp"
#include "whooplib/include/toolbox.hpp"
#include <cmath>

namespace whoop {

WhoopOdomFusion::WhoopOdomFusion(WhoopVision *whoop_vision,
                                 WhoopDriveOdomOffset *odom_offset,
                                 double min_confidence_threshold,
                                 fusionmode fusion_mode,
                                 double max_fusion_shift_meters,
                                 double max_fusion_shift_radians) {
  this->odom_offset = odom_offset;
  this->max_fusion_shift_meters = max_fusion_shift_meters / 55.6;
  this->max_fusion_shift_radians = max_fusion_shift_radians / 55.6;
  this->fusion_mode = fusion_mode;
  this->whoop_vision = whoop_vision;
  this->whoop_vision->on_update(std::bind(
      &WhoopOdomFusion::on_vision_pose_received, this, std::placeholders::_1));
}

WhoopOdomFusion::WhoopOdomFusion(WhoopDriveOdomOffset *odom_offset) {
  this->odom_offset = odom_offset;
  this->max_fusion_shift_meters = 0;
  this->max_fusion_shift_radians = 0;
  this->fusion_mode = fusionmode::wheel_odom_only;
  this->whoop_vision = nullptr;
}

void WhoopOdomFusion::on_vision_pose_received(Pose p) {
  if (fusion_mode == fusionmode::wheel_odom_only || !accepting_fuses) {
    return;
  }

  if (p.confidence >= min_confidence_threshold) {
    frame_rejected = false;
    // Normalize angle difference to handle angle wrapping correctly
    double yaw_difference = normalize_angle(p.yaw - pose.yaw);

    double distance =
        std::sqrt(std::pow(p.x - pose.x, 2) + std::pow(p.y - pose.y, 2));
    double angle_difference = std::fabs(yaw_difference);

    // Handle linear position adjustment
    if (fusion_mode == fusionmode::fusion_gradual &&
        distance > max_fusion_shift_meters) {
      double dx = p.x - pose.x;
      double dy = p.y - pose.y;
      double norm = std::sqrt(dx * dx + dy * dy);

      if (std::abs(norm) > 1e-10) {
        dx = safeDivide(dx * max_fusion_shift_meters, norm,
                        max_fusion_shift_meters);
        dy = safeDivide(dy * max_fusion_shift_meters, norm,
                        max_fusion_shift_meters);

        self_lock.lock();
        pose.x += dx;
        pose.y += dy;
        self_lock.unlock();
      }
    } else {
      self_lock.lock();
      pose.x = p.x;
      pose.y = p.y;
      self_lock.unlock();
    }

    // Handle angular position adjustment
    self_lock.lock();
    if (fusion_mode == fusionmode::fusion_gradual &&
        angle_difference > max_fusion_shift_radians) {
      pose.yaw += std::copysign(max_fusion_shift_radians, yaw_difference);
    } else {
      pose.yaw = p.yaw;
    }
    pose.yaw = normalize_angle(pose.yaw);

    odom_offset->tare(pose.x, pose.y, pose.yaw);
    self_lock.unlock();
  } else {
    frame_rejected = true;
  }
  self_lock.lock();
  pose.z = p.z;
  pose.confidence = p.confidence;
  self_lock.unlock();
}

void WhoopOdomFusion::tare(double x, double y, double z, double yaw) {
  self_lock.lock();

  if (whoop_vision != nullptr) {
    whoop_vision->tare(x, y, z, 0, yaw, 0);
  }

  // Tare
  odom_offset->tare(x, y, yaw);
  pose.x = x;
  pose.y = y;
  pose.z = z;
  pose.yaw = yaw;
  self_lock.unlock();
}

void WhoopOdomFusion::tare(double x, double y, double yaw) {
  tare(x, y, 0, yaw);
}

void WhoopOdomFusion::tare() { tare(0, 0, 0); }

void WhoopOdomFusion::calibrate() {

  self_lock.lock();

  odom_offset->calibrate();
  if (whoop_vision != nullptr) {
    whoop_vision->tare();
  }

  odom_offset->tare();
  self_lock.unlock();
}

Pose WhoopOdomFusion::get_pose() {
  self_lock.lock();
  Pose p = pose;
  self_lock.unlock();
  return p;
}

TwoDPose WhoopOdomFusion::get_pose_2d() {
  Pose p = get_pose();
  return TwoDPose(p.x, p.y, p.yaw);
}

bool WhoopOdomFusion::is_moving(double rads_s_threshold) {
  return odom_offset->is_moving(rads_s_threshold);
}

bool WhoopOdomFusion::approving_frames() { return !frame_rejected; }

void WhoopOdomFusion::accept_fuses() { accepting_fuses = true; }

void WhoopOdomFusion::reject_fuses() { accepting_fuses = false; }

void WhoopOdomFusion::__step() {
  self_lock.lock();

  if (fusion_mode != fusionmode::vision_only) {
    odom_offset->__step_down(); // Step down wheel odometry ladder
    TwoDPose result = odom_offset->get_pose();
    pose.x = result.x;
    pose.y = result.y;
    pose.yaw = result.yaw;
  }
  pose.roll = odom_offset->odom_unit->inertial_sensor->get_roll_radians();
  pose.pitch = odom_offset->odom_unit->inertial_sensor->get_pitch_radians();
  self_lock.unlock();
}

} // namespace whoop
