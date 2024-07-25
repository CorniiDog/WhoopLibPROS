/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       PurePursuitConductor.cpp                                  */
/*    Author:       Connor White (WHOOP)                                      */
/*    Created:      Thu July 3 2024                                           */
/*    Description:  Pure Pursuit Conductor for Motor Movements                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "whooplib/include/calculators/PurePursuitConductor.hpp"
#include "vex.h"
#include <iostream>

PurePursuitConductor::PurePursuitConductor(PursuitParams *default_pursuit_parameters) : turn_pid(0, default_pursuit_parameters->turning_kp, default_pursuit_parameters->turning_ki, default_pursuit_parameters->turning_kd, default_pursuit_parameters->turning_i_activation, default_pursuit_parameters->settle_rotation, default_pursuit_parameters->settle_time, default_pursuit_parameters->timeout),
                                                                                        forward_pid(0, default_pursuit_parameters->forward_kp, default_pursuit_parameters->forward_ki, default_pursuit_parameters->forward_kp, default_pursuit_parameters->forward_i_activation, default_pursuit_parameters->settle_distance, default_pursuit_parameters->settle_time, default_pursuit_parameters->timeout),
                                                                                        turn_slew(default_pursuit_parameters->max_voltage_change, false, 10),
                                                                                        forward_slew(default_pursuit_parameters->max_voltage_change, false, 10),
                                                                                        pursuit_path(TwoDPose(), TwoDPose(), default_pursuit_parameters->turning_radius, default_pursuit_parameters->lookahead_distance, default_pursuit_parameters->num_path_segments),
                                                                                        default_pursuit_parameters(default_pursuit_parameters)
{
}

void PurePursuitConductor::generate_path(TwoDPose start_position, TwoDPose destination_position)
{
    generate_path(start_position, destination_position, -1, -1);
}

void PurePursuitConductor::generate_path(TwoDPose start_position, TwoDPose destination_position, double timeout)
{
    generate_path(start_position, destination_position, timeout, -1);
}

void PurePursuitConductor::generate_path(TwoDPose start_position, TwoDPose destination_position, double timeout, double turning_radius)
{
    generate_path({start_position, destination_position}, timeout, turning_radius);
}

void PurePursuitConductor::generate_path(std::vector<std::vector<double>> waypoints)
{
    generate_path(waypoints, -1, -1);
}

void PurePursuitConductor::generate_path(std::vector<std::vector<double>> waypoints, double timeout)
{
    generate_path(waypoints, timeout, -1);
}

void PurePursuitConductor::generate_path(std::vector<std::vector<double>> waypoints, double timeout, double turning_radius, double landing_strip)
{
    // Ensure that waypoints are 2 or greater
    size_t waypoints_size = waypoints.size();

    // Generate waypoints constructed with TwoDPose
    std::vector<TwoDPose> constructed_waypoints;

    size_t size_of;
    for (size_t i = 0; i < waypoints_size; i++)
    {
        size_of = waypoints[i].size();
        if (size_of == 3)
        { // If size is 3 then just push back
            constructed_waypoints.push_back(TwoDPose(waypoints[i][0], waypoints[i][1], waypoints[i][2]));
        }
        else
        { // Size is 2
            if (i == waypoints_size - 1)
            { // If last element and size is 2. Use x and y of last element and yaw of first
                TwoDPose looker = TwoDPose(waypoints[i - 1][0], waypoints[i - 1][1], 0).lookAt(waypoints[i][0], waypoints[i][1]);

                constructed_waypoints.push_back(TwoDPose(waypoints[i][0], waypoints[i][1], looker.yaw));
            }
            else
            {
                // Push back waypoint, but with yaw looking at the next target
                constructed_waypoints.push_back(TwoDPose(waypoints[i][0], waypoints[i][1], 0).lookAt(waypoints[i + 1][0], waypoints[i + 1][1]));
            }
        }
    }

    generate_path(constructed_waypoints, timeout, turning_radius, landing_strip);
}

/**
 * Generates the path
 * @param waypoints The waypoints for generating the path. Example would be {TwoDPose(0,0,0), TwoDPose(20,10,M_PI_2)}
 * The yaw for each position in the list must be explicitly stated when using TwoDPose objects
 */
void PurePursuitConductor::generate_path(std::vector<TwoDPose> waypoints)
{
    generate_path(waypoints, -1, -1);
}

void PurePursuitConductor::generate_path(std::vector<TwoDPose> waypoints, double timeout)
{
    generate_path(waypoints, timeout, -1);
}

void PurePursuitConductor::generate_path(std::vector<TwoDPose> waypoints, double timeout, double turning_radius, double landing_strip)
{
    if (waypoints.size() < 2)
    {
        Brain.Screen.print("A path requires at least 2 waypoints");
        std::cout << "A path requires at least 2 waypoints" << std::endl;
    }

    is_turn = false;
    double turn_rad;
    if (turning_radius >= 0)
    {
        turn_rad = turning_radius;
    }
    else
    {
        turn_rad = default_pursuit_parameters->turning_radius;
    }

    double t_out;
    if (timeout >= 0)
    {
        t_out = timeout;
    }
    else
    {
        t_out = default_pursuit_parameters->timeout;
    }

    forward_pid = PID(0, default_pursuit_parameters->forward_kp, default_pursuit_parameters->forward_ki, default_pursuit_parameters->forward_kp, default_pursuit_parameters->forward_i_activation, default_pursuit_parameters->settle_distance, default_pursuit_parameters->settle_time, t_out),
    turn_pid = PID(0, default_pursuit_parameters->turning_kp, default_pursuit_parameters->turning_ki, default_pursuit_parameters->turning_kd, default_pursuit_parameters->turning_i_activation, default_pursuit_parameters->settle_rotation, default_pursuit_parameters->settle_time, t_out),
    this->end_position = waypoints[waypoints.size() - 1]; // Last element of the waypoints list (as starting index is 0 instead of 1)
    pursuit_path = PurePursuitPath(waypoints, turn_rad, default_pursuit_parameters->lookahead_distance, default_pursuit_parameters->num_path_segments, landing_strip);
    enabled = true;
}

void PurePursuitConductor::generate_turn(TwoDPose turn_pose, double timeout)
{
    this->turn_pose = turn_pose;
    is_turn = true;
    enabled = true;

    double t_out;
    if (timeout >= 0)
    {
        t_out = timeout;
    }
    else
    {
        t_out = default_pursuit_parameters->timeout;
    }
}

size_t i = 0;

PursuitResult PurePursuitConductor::step(TwoDPose current_pose)
{
    if (!enabled)
    { // If not enabled, set is_valid as true and is_completed as true
        return PursuitResult(true, 0, 0, 0, 0, true);
    }

    PursuitEstimate estimate;
    if (is_turn)
    { // If command is to turn
        estimate = PursuitEstimate(true, normalize_angle(turn_pose.yaw - current_pose.yaw), 0, true, 0, true);
        estimate.last_steering = estimate.steering_angle;
    }
    else
    {
        estimate = pursuit_path.calculate_pursuit_estimate(current_pose, true, forward_pid.settle_error);
    }

    if (!estimate.is_valid)
    { // If error or something, return is_valid as false
        return PursuitResult(false, 0, 0, 0, 0, false);
    }

    double forward_power = forward_slew.step(forward_pid.step(estimate.distance));
    if (forward_pid.settling())
    {
        forward_power = 0;
        forward_pid.zeroize_accumulated();
        estimate.steering_angle = estimate.last_steering;
        estimate.suggest_point_turn = true;
    }

    double turn_power = turn_slew.step(turn_pid.step(estimate.steering_angle));
    if (turn_pid.settling())
    {
        turn_power = 0;
        turn_pid.zeroize_accumulated();
    }

    PursuitResult result = PursuitResult(true, estimate.steering_angle, estimate.distance,
                                         clamp(forward_power, -default_pursuit_parameters->forward_max_voltage, default_pursuit_parameters->forward_max_voltage),
                                         clamp(turn_power, -default_pursuit_parameters->turning_max_voltage, default_pursuit_parameters->turning_max_voltage),
                                         false, estimate.suggest_point_turn);

    if ((forward_pid.is_settled()) && turn_pid.is_settled())
    {
        result.is_completed = true;
    }
    else if (turn_pid.settling() && !forward_pid.settling())
    {
        turn_pid.time_spent_settled = 0;
    }

    return result;
}