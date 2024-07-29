/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       WhoopOdomFusion.hpp                                       */
/*    Author:       Connor White (WHOOP)                                      */
/*    Created:      Thu Jun 25 2024                                           */
/*    Description:  Fuses Wheel and Visual Odometry                           */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#ifndef WHOOP_ODOM_FUSION_HPP
#define WHOOP_ODOM_FUSION_HPP

#include "whooplib/include/calculators/RollingAverage.hpp"
#include "whooplib/include/devices/WhoopDriveOdomOffset.hpp"
#include "whooplib/include/devices/WhoopVision.hpp"
#include "whooplib/include/toolbox.hpp"
#include "whooplib/includer.hpp"
#include <memory>
#include <vector>

namespace whoop {

// Enumeration defining possible fusion modes between visual and wheel odometry
// data.
enum fusionmode {
  fusion_instant, // Instantly aligns wheel odometry to vision odometry upon
                  // data retrieval.
  fusion_gradual, // Gradually aligns wheel odometry to vision odometry over
                  // time.
  vision_only,    // Vision Odometry Only
  wheel_odom_only // Wheel Odometry Only
};

// Class responsible for fusing visual and wheel odometry data.
class WhoopOdomFusion : public ComputeNode {
protected:
  WhoopMutex self_lock;            // Mutex for thread-safe operations.
  WhoopVision *whoop_vision;       // Pointer to the vision odometry unit.
  double min_confidence_threshold; // Minimum confidence level required to
                                   // accept new vision data.
  fusionmode fusion_mode;          // Current mode of odometry data fusion.
  double max_fusion_shift_meters;  // Maximum shift in meters per step when
                                   // gradually fusing data.
  double max_fusion_shift_radians; // Maximum rotational shift in radians per
                                   // step when gradually fusing data.

  WhoopDriveOdomOffset *odom_offset;

  // Callback function that handles new vision pose data.
  void on_vision_pose_received(Pose p);

  bool frame_rejected = true;

  bool accepting_fuses = false;

public:
  Pose pose = Pose(); // Current fused pose of the odometry system.

  /**
   * Constructs a new odometry fusion object.
   * @param whoop_vision Pointer to the vision odometry system.
   * @param odom_offset Pointer to the wheel odometry offset object.
   * @param min_confidence_threshold Minimum confidence required to consider
   * vision data (0.0 - 1.0).
   * @param fusion_mode Method of fusing vision with wheel odometry (instant,
   * gradual, vision_only, wheel_odom_only).
   * @param max_fusion_shift_meters If FusionMode is fusion_gradual, it is the
   * maximum allowable shift in meters for gradual fusion, per second.
   * @param max_fusion_shift_radians If FusionMode is fusion_gradual, it is the
   * maximum allowable rotational shift in radians for gradual fusion, per
   * second.
   */
  WhoopOdomFusion(WhoopVision *whoop_vision, WhoopDriveOdomOffset *odom_offset,
                  double min_confidence_threshold, fusionmode fusion_mode,
                  double max_fusion_shift_meters,
                  double max_fusion_shift_radians);

  /**
   * Constructor for just wheel odometry
   * @param odom_offset Pointer to the wheel odometry offset object.
   */
  WhoopOdomFusion(WhoopDriveOdomOffset *odom_offset);

  /**
   * Retreives the pose from the odom fusion
   * @returns Pose object
   */
  Pose get_pose();

  /**
   * Retreives the pose from the odom fusion
   * @returns Pose object, two dimension
   */
  TwoDPose get_pose_2d();

  /**
   * Runs calibration process
   */
  void calibrate();

  /**
   * Returns true if approving frames
   */
  bool approving_frames();

  /**
   * Sets the current odometry to the specified coordinates and yaw.
   * @param x the x coordinate (forwards), in meters
   * @param y the y coordinate (right), in meters
   * @param yaw the yaw (counter-clockwise), in radians
   */
  void tare(double x, double y, double yaw);

  /**
   * Sets the current odometry to the specified coordinates in 3D space, and
   * yaw.
   * @param x the x coordinate (forwards), in meters
   * @param y the y coordinate (right), in meters
   * @param z the z coordinate (up), in meters
   * @param yaw the yaw (counter-clockwise), in radians
   */
  void tare(double x, double y, double z, double yaw);

  // Resets the current odometry to the origin (0,0,0).
  void tare();

  // Returns true if moving
  bool is_moving(double rads_s_threshold = 0.02);

  void accept_fuses();
  void reject_fuses();

public: // Exceptionally public to allow access from another module.
  /**
   * Processes a single step of odometry updates.
   * This method is called periodically to integrate new sensor data and adjust
   * the internal state.
   */
  void __step() override; // Protected helper function for processing steps
};

} // namespace whoop

#endif // WHOOP_ODOM_FUSION_HPP
