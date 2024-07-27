/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       PurePursuit.hpp                                           */
/*    Author:       Connor White (WHOOP)                                      */
/*    Created:      Thu July 3 2024                                           */
/*    Description:  Pure Pursuit Steering Calculator                          */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#ifndef PURE_PURSUIT_HPP
#define PURE_PURSUIT_HPP

#include "whooplib/include/calculators/Dubins.hpp"
#include "whooplib/include/calculators/TwoDPose.hpp"
#include <vector>

namespace whoop
{

    struct PursuitEstimate
    {
        bool is_valid;
        double steering_angle;
        double distance;
        bool is_past_point;
        double last_steering;
        bool suggest_point_turn;

        /**
         * @param is_valid would be true if the pursuit estimate returned no error
         * @param steering_angle would be the angle to turn towards for course correction in radians, counter-clockwise-positive
         * @param distance would be the distance from the target, in meters
         * @param is_past_point Returns true if the robot passes the point slightly
         * @param last_steering Is the suggested steering for last point
         * @param suggest_point_turn Would be true if the pure pursuits suggest a point turn instead of swing turn
         */
        PursuitEstimate(bool is_valid = false, double steering_angle = 0, double distance = 0, bool is_past_point = false, double last_steering = 0, bool suggest_point_turn = false) : is_valid(is_valid), steering_angle(steering_angle), distance(distance), is_past_point(is_past_point), last_steering(last_steering), suggest_point_turn(suggest_point_turn) {}
    };

    struct barebonesPose
    {
        double x = 0;
        double y = 0;
        double yaw = 0;
        barebonesPose(double x = 0, double y = 0, double yaw = 0) : x(x), y(y), yaw(yaw) {}
    };

    struct pursuitCheckpoint
    {
        double i;
        bool visited;
        bool is_last;
        pursuitCheckpoint(double i, bool visited = false, bool is_last = false) : i(i), visited(visited), is_last(is_last) {}
    };

    class PurePursuitPath
    {
    private:
        TwoDPose start, end;

        std::vector<TwoDPose> waypoints;

        double turning_radius;

        TwoDPose end_translated_back;

    public:
        double lookahead_distance;
        barebonesPose lookahead_pos;

    private:
        double q0[3] = {0, 0, 0};
        double q1[3] = {0, 0, 0};
        DubinsPath path;
        bool path_valid;
        double t_max = 0;
        double num_segments;
        double step_size;

        double landing_strip;
        double push_back_distance = 0;

        void initializeWaypoints(std::vector<TwoDPose> waypoints);
        void computeDubinsPath();

    public:
        std::vector<barebonesPose> pursuit_points;
        std::vector<pursuitCheckpoint> pursuit_checkpoints;

        int create_points(double q[3], double x);

        /**
         * Creates a path for pure pursuit, using Dubin-Curves. NOTE: Yaw is ccw-positive
         * You can find more information about Dubin-Curves here: https://github.com/AndrewWalker/Dubins-Curves?tab=readme-ov-file
         * @param start The TwoDPose object representing the start of the path (like robot position)
         * @param end The TwoDPose object representing the end of the path (pose to drive to)
         * @param turning_radius Turning radius of the dubin curve, in meters. You can see visual representations of the turning radius here: https://imgur.com/BahIst0
         * @param lookahead_distance The look-ahead distance for pure pursuit along a path, in meters. Visual representation of lookahead distance here: https://imgur.com/WT5G0Z1
         * @param num_segments The number of points when generating the path. More points mean higher detail of the path, but at a higher computational cost
         * @param landing_strip The length of the landing strip of the robot, similar to that of an airport runway landing strip at the end of a move
         */
        PurePursuitPath(const TwoDPose start, const TwoDPose end, double turning_radius, double lookahead_distance, double num_segments = 200, double landing_strip = -1);

        /**
         * Creates a path for pure pursuit, using Dubin-Curves. NOTE: Yaw is ccw-positive
         * You can find more information about Dubin-Curves here: https://github.com/AndrewWalker/Dubins-Curves?tab=readme-ov-file
         * @param waypoints The points to generate a path for. The starting point should be the position the robot is already in for best results
         * @param turning_radius Turning radius of the dubin curve, in meters. You can see visual representations of the turning radius here: https://imgur.com/BahIst0
         * @param lookahead_distance The look-ahead distance for pure pursuit along a path, in meters. Visual representation of lookahead distance here: https://imgur.com/WT5G0Z1
         * @param num_segments The number of points when generating the path. More points mean higher detail of the path, but at a higher computational cost
         * @param landing_strip The length of the landing strip of the robot, similar to that of an airport runway landing strip at the end of the move
         */
        PurePursuitPath(std::vector<TwoDPose> waypoints, double turning_radius, double lookahead_distance, double num_segments = 200, double landing_strip = -1);

        /**
         * Calculates the pure pursuit estimate relative to the path. NOTE: Yaw is ccw-positive
         * @param current_position The TwoDPose of the current position
         * @param find_closest_if_off_course Set to true to find the closest point of the path
         * @returns PursuitEstimate, which contains "is_valid" bool which returns true if succeeded successfully, or false if the robot is off the path.
         * Also includes "steering_angle" which is the angle to steer to (if + means steer left, if - means steer right).
         * "distance" is how far away from the lookahead point.
         */
        PursuitEstimate calculate_pursuit_estimate(TwoDPose current_position, bool find_closest_if_off_course = true, double deviation_min = 0);
    };

} // namespace whoop

#endif // PURE_PURSUIT_HPP