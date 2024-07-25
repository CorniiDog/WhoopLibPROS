/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       PurePursuitConductor.hpp                                  */
/*    Author:       Connor White (WHOOP)                                      */
/*    Created:      Thu July 3 2024                                           */
/*    Description:  Pure Pursuit Conductor for Motor Movements                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#ifndef PURE_PURSUIT_CONDUCTOR_HPP
#define PURE_PURSUIT_CONDUCTOR_HPP

#include "whooplib/include/calculators/TwoDPose.hpp"
#include "whooplib/include/calculators/PurePursuit.hpp"
#include "whooplib/include/calculators/PID.hpp"
#include "whooplib/include/calculators/Slew.hpp"
#include "whooplib/include/toolbox.hpp"
#include <vector>

struct PursuitParams
{
    double turning_radius;
    double lookahead_distance;

    double forward_max_voltage;
    double turning_max_voltage;

    double max_voltage_change;

    double settle_distance;
    double settle_rotation;
    double settle_time;
    double timeout;

    double turning_kp;
    double turning_ki;
    double turning_kd;
    double turning_i_activation;

    double forward_kp;
    double forward_ki;
    double forward_kd;
    double forward_i_activation;

    int num_path_segments;

    /**
     * @param turning_radius Radius of the turns, in meters
     * @param lookahead_distance Pure Pursuit look ahead distance, in meters
     * @param forward_max_voltage The maximum voltage the motors can spin while going forward
     * @param turning_max_voltage The maximum voltage the motors can spin while turning
     * @param max_voltage_change // The maximum voltage change per second, as a slew rate (only applies speeding up)
     * @param settle_distance Exits when within this distance of target, in meters
     * @param settle_rotation Exits when within this rotation of target, in radians
     * @param settle_time Minimum time to be considered settled, in seconds
     * @param timeout Time after which to give up and move on, in seconds
     * @param turning_kp Turning Proportional Tuning
     * @param turning_ki Turning Integral Tuning
     * @param turning_kd Turning Derivative Tuning
     * @param turning_i_activation The rotation distance (error), in radians, to activate turning_ki
     * @param forward_kp Forward Proportional Tuning
     * @param forward_ki Forward Integral Tuning
     * @param forward_kd Forward Derivative Tuning
     * @param forward_i_activation The forward distance (error), in meters, to activate forward_ki
     * @param num_path_segments The number of points when generating the path. More points mean higher detail of the path, but at a higher computational cost
     */
    PursuitParams(double turning_radius = to_meters(5), double lookahead_distance = to_meters(5),
                  double forward_max_voltage = 8.0, double turning_max_voltage = 12.0,
                  double max_voltage_change = 50,
                  double settle_distance = to_meters(1.25), double settle_rotation = to_rad(1),
                  double settle_time = 0.3, double timeout = 0,
                  double turning_kp = 14, double turning_ki = 0.1, double turning_kd = 20, double turning_i_activation = to_meters(15),
                  double forward_kp = 55, double forward_ki = 0.01, double forward_kd = 250, double forward_i_activation = to_meters(2),
                  int num_path_segments = 100) : turning_radius(turning_radius), lookahead_distance(lookahead_distance),
                                                 forward_max_voltage(forward_max_voltage), turning_max_voltage(turning_max_voltage),
                                                 max_voltage_change(max_voltage_change),
                                                 settle_distance(settle_distance), settle_rotation(settle_rotation),
                                                 settle_time(settle_time), timeout(timeout),
                                                 turning_kp(turning_kp), turning_ki(turning_ki), turning_kd(turning_kd), turning_i_activation(turning_i_activation),
                                                 forward_kp(forward_kp), forward_ki(forward_ki), forward_kd(forward_kd), forward_i_activation(forward_i_activation),
                                                 num_path_segments(num_path_segments) {}
};

struct PursuitResult
{
    bool is_valid;
    double steering_angle;
    double distance;
    double forward_power;
    double steering_power;
    bool is_completed;
    bool suggest_point_turn;
    /**
     * @param is_valid would be true if the pursuit estimate returned no error
     * @param steering_angle would be the angle to turn towards for course correction in radians, counter-clockwise-positive
     * @param distance would be the distance from the target, in meters
     * @param forward_power the suggested motor power to go forward
     * @param steering_power the suggestedmotor power for steering
     * @param is_completed if true, the pure pursuit is complete
     * @param suggest_point_turn is true when it suggests a point turn instead of swing turn
     */
    PursuitResult(bool is_valid = false, double steering_angle = 0, double distance = 0, double forward_power = 0, double steering_power = 0, bool is_completed = false, bool suggest_point_turn = false) : is_valid(is_valid), steering_angle(steering_angle), distance(distance), forward_power(forward_power), steering_power(steering_power), is_completed(is_completed), suggest_point_turn(suggest_point_turn) {}
};

class PurePursuitConductor
{

public:
    PID turn_pid;
    PID forward_pid;
    Slew turn_slew;
    Slew forward_slew;
    PurePursuitPath pursuit_path;
    TwoDPose end_position;
    PursuitParams *default_pursuit_parameters = nullptr;

    bool is_turn = false;
    TwoDPose turn_pose;

public:
    bool enabled = false;

    /**
     * Constructs the conductor for the pure pursuit object
     * @param default_pursuit_parameters The parameters for the pure pursuit
     */
    PurePursuitConductor(PursuitParams *default_pursuit_parameters);

    /**
     * Generates the path for the point
     * @param start_position The TwoDPose of the start position
     * @param destination_position The TwoDPose of the destination position
     */
    void generate_path(TwoDPose start_position, TwoDPose destination_position);

    /**
     * Generates the path for the point
     * @param start_position The TwoDPose of the start position
     * @param destination_position The TwoDPose of the destination position
     * @param timeout The timeout of the movement, in seconds
     */
    void generate_path(TwoDPose start_position, TwoDPose destination_position, double timeout);

    /**
     * Generates the path for the point
     * @param start_position The TwoDPose of the start position
     * @param destination_position The TwoDPose of the destination position
     * @param timeout The timeout of the movement, in seconds
     * @param turning_radius The radius, in meters, of the turning
     */
    void generate_path(TwoDPose start_position, TwoDPose destination_position, double timeout, double turning_radius);

    /**
     * Generates the path
     * @param waypoints The waypoints for generating the path. Example would be {{0,0,0}, {20,10,M_PI_2}}
     * As long as the start has a yaw, the rest does not really need one, however not providing the end yaw explicitly will cause the end position to have the same yaw as the start yaw.
     * Additional valid path examples would be {{0,0,0}, {5,5}, {10,20}} and {{0,0,0}, {5,5}, {10,20, M_PI_2}}
     */
    void generate_path(std::vector<std::vector<double>> waypoints);

    /**
     * Generates the path
     * @param waypoints The waypoints for generating the path. Example would be {{0,0,0}, {20,10,M_PI_2}}
     * As long as the start has a yaw, the rest does not really need one, however not providing the end yaw explicitly will cause the end position to have the same yaw as the start yaw.
     * Additional valid path examples would be {{0,0,0}, {5,5}, {10,20}} and {{0,0,0}, {5,5}, {10,20, M_PI_2}}
     * @param timeout The timeout of the movement, in seconds
     */
    void generate_path(std::vector<std::vector<double>> waypoints, double timeout);

    /**
     * Generates the path
     * @param waypoints The waypoints for generating the path. Example would be {{0,0,0}, {20,10,M_PI_2}}
     * As long as the start has a yaw, the rest does not really need one, however not providing the end yaw explicitly will cause the end position to have the same yaw as the start yaw.
     * Additional valid path examples would be {{0,0,0}, {5,5}, {10,20}} and {{0,0,0}, {5,5}, {10,20, M_PI_2}}
     * @param timeout The timeout of the movement, in seconds
     * @param turning_radius The radius, in meters, of the turning
     */
    void generate_path(std::vector<std::vector<double>> waypoints, double timeout, double turning_radius, double landing_strip = -1);

    /**
     * Generates the path
     * @param waypoints The waypoints for generating the path. Example would be {TwoDPose(0,0,0), TwoDPose(20,10,M_PI_2)}
     * The yaw for each position in the list must be explicitly stated when using TwoDPose objects
     */
    void generate_path(std::vector<TwoDPose> waypoints);

    /**
     * Generates the path
     * @param waypoints The waypoints for generating the path. Example would be {TwoDPose(0,0,0), TwoDPose(20,10,M_PI_2)}
     * The yaw for each position in the list must be explicitly stated when using TwoDPose objects
     * @param timeout The timeout of the movement, in seconds
     */
    void generate_path(std::vector<TwoDPose> waypoints, double timeout);

    /**
     * Generates the path
     * @param waypoints The waypoints for generating the path. Example would be {TwoDPose(0,0,0), TwoDPose(20,10,M_PI_2)}
     * The yaw for each position in the list must be explicitly stated when using TwoDPose objects
     * @param timeout The timeout of the movement, in seconds
     * @param turning_radius The radius, in meters, of the turning
     */
    void generate_path(std::vector<TwoDPose> waypoints, double timeout, double turning_radius, double landing_strip = -1);

    /**
     * Generates the turn request
     * @param turn_pose The pose of the desired turn
     * @param timeout The timeout of the movement, in seconds
     */
    void generate_turn(TwoDPose turn_pose, double timeout);

    /**
     * Steps the conductor
     */
    PursuitResult step(TwoDPose current_pose);
};

#endif