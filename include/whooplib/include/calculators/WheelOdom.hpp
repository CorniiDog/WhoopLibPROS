/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       WheelOdom.hpp                                             */
/*    Author:       Connor White (WHOOP)                                      */
/*    Created:      Thu Jun 21 2024                                           */
/*    Description:  Wheel X, Y, Yaw Tracking                                  */
/*                                                                            */
/*----------------------------------------------------------------------------*/

/*
 * Copyright (c) 2023 2775Josh
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef WHEEL_ODOM_HPP
#define WHEEL_ODOM_HPP

#include "whooplib/includer.hpp"

namespace whoop
{

  /**
   * Wheel Odometry Object
   */
  class WheelOdom
  {
  private:
    double tare_angle{0.0}; // Stores the initial gyro angle for taring
    double last_forward_tracker_pos{0.0};
    double last_sideways_tracker_pos{0.0};
    double forward_tracker_center_distance{0.0};
    double sideways_tracker_center_distance{0.0};

  public:
    double X_position{0.0};
    double Y_position{0.0};
    double orientation_rad{0.0};
    WheelOdom() = default;

    /**
     * Setter method for tracker center distances.
     * The forward tracker center distance is the horizontal distance from the
     * center of the robot to the center of the wheel the sensor is measuring.
     * The sideways tracker center distance is the vertical distance from the
     * center of the robot to the center of the sideways wheel being measured.
     * If there's really no sideways wheel we set the center distance to 0 and
     * pretend the wheel never spins, which is equivalent to a no-drift robot.
     *
     * @param x The desired x position to start at
     * @param y The desired y position to start at
     * @param orientation The desired yaw position to start at (radians, counter-clockwise)
     */
    void set_position(double x, double y, double orientation);

    /**
     * Resets the position, including tracking wheels.
     * Position is field-centric, and orientation is such that 0 radians
     * is in the positive Y direction. Orientation can be provided with
     * some flexibility, including less than 0 and greater than 2*pi.
     *
     * @param forward_tracker_pos Current position of the sensor in meters.
     * @param sideways_tracker_pos Current position of the sensor in meters.
     * @param gyro_angle_rad Current angle of the gyroscope (radians, counter-clockwise positive)
     */
    void update_pose(double forward_tracker_pos, double sideways_tracker_pos, double gyro_angle_rad);

    /**
     * Does the odometry math to update position
     * Uses the Pilons arc method outlined here: https://wiki.purduesigbots.com/software/odometry
     * All the deltas are done by getting member variables and comparing them to
     * the input. Ultimately this all works to update the public member variable
     * X_position. This function needs to be run at 200Hz or so for best results.
     *
     * @param forward_distance Distance from the odom unit center to the forward tracker, in meters (positive implies a shift to the right from the odom unit center).
     * @param sideways_distance Distance from the odom unit center to the sideways tracker, in meters (positive implies a shift forward from the odom unit center).
     */
    void set_physical_distances(double forward_distance, double sideways_distance);
    ;
  };

} // namespace whoop

#endif // WHEEL_ODOM_HPP