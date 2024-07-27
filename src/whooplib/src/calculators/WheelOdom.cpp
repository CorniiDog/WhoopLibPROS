/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       WheelOdom.cpp                                             */
/*    Author:       Connor White (WHOOP)                                      */
/*    Created:      Thu Jun 21 2024                                           */
/*                                                                            */
/*    Reference:    Jar-Template                                              */
/*    Ref Link:     https://github.com/JacksonAreaRobotics/JAR-Template       */
/*                                                                            */
/*    Description:  Wheel X, Y, Yaw Tracking                                  */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "whooplib/include/calculators/WheelOdom.hpp"
#include "whooplib/include/toolbox.hpp"

namespace whoop{

void WheelOdom::set_position(double x, double y, double orientation)
{
    X_position = x;
    Y_position = y;
    orientation_rad = orientation;
    tare_angle = orientation; // Set the tare angle to the initial orientation

    // Normalize the orientation_rad
    orientation_rad = normalize_angle(orientation_rad);
}

void WheelOdom::set_physical_distances(double forward_distance, double sideways_distance)
{
    forward_tracker_center_distance = forward_distance;
    sideways_tracker_center_distance = sideways_distance;
}

void WheelOdom::update_pose(double forward_tracker_pos, double sideways_tracker_pos, double orientation_rad)
{
    // this-> always refers to the old version of the variable, so subtracting this->x from x gives delta x.
    double delta_forward = forward_tracker_pos - last_forward_tracker_pos;
    double delta_sideways = sideways_tracker_pos - last_sideways_tracker_pos;
    last_forward_tracker_pos = forward_tracker_pos;
    last_sideways_tracker_pos = sideways_tracker_pos;

    double prev_orientation_rad = this->orientation_rad;
    double orientation_delta_rad = orientation_rad - prev_orientation_rad;
    this->orientation_rad = orientation_rad;

    double local_X_position;
    double local_Y_position;

    if (orientation_delta_rad == 0)
    {
        local_X_position = delta_sideways;
        local_Y_position = delta_forward;
    }
    else
    {
        local_X_position = (2 * sin(-orientation_delta_rad / 2)) * ((delta_sideways / (-orientation_delta_rad)) + sideways_tracker_center_distance);
        local_Y_position = (2 * sin(-orientation_delta_rad / 2)) * ((delta_forward / (-orientation_delta_rad)) + forward_tracker_center_distance);
    }

    double local_polar_angle;
    double local_polar_length;

    if (local_X_position == 0 && local_Y_position == 0)
    {
        local_polar_angle = 0;
        local_polar_length = 0;
    }
    else
    {
        local_polar_angle = atan2(local_Y_position, local_X_position);
        local_polar_length = sqrt(pow(local_X_position, 2) + pow(local_Y_position, 2));
    }

    double global_polar_angle = local_polar_angle + prev_orientation_rad + (orientation_delta_rad / 2);

    double X_position_delta = local_polar_length * sin(global_polar_angle);
    double Y_position_delta = -local_polar_length * cos(global_polar_angle);

    X_position += X_position_delta;
    Y_position += Y_position_delta;
}

} // namespace whoop
