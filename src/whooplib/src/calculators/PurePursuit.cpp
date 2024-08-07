/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       PurePursuit.cpp                                           */
/*    Author:       Connor White (WHOOP)                                      */
/*    Created:      Thu July 3 2024                                           */
/*    Description:  Pure Pursuit Steering Calculator                          */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "whooplib/include/calculators/PurePursuit.hpp"
#include "whooplib/include/calculators/Dubins.hpp"
#include "whooplib/include/toolbox.hpp"
#include "whooplib/includer.hpp"
#include <iostream>

namespace whoop {

void PurePursuitPath::initializeWaypoints(std::vector<TwoDPose> waypoints) {
  if (waypoints.size() < 2) {
#if USE_VEXCODE
    Brain.Screen.print("Error. Waypoints must have 2 or more points.");
#else
    //whoop::screen::print_at(1, "Error. Waypoints must have 2 or more points.");
#endif
    std::cout << "Error. Waypoints must have 2 or more points." << std::endl;
  }

  end = waypoints[waypoints.size() - 1]; // Retreieve last element (end)

  waypoints.pop_back(); // Remove last element

  if (landing_strip < 0) {
    push_back_distance = lookahead_distance;
  } else {
    push_back_distance = landing_strip;
  }

  end_translated_back =
      end * TwoDPose(0, -push_back_distance,
                     0); // Translate the end back so that the end of the path
                         // is a decent straight line for the look ahead

  waypoints.push_back(
      end_translated_back); // Add the translated back point to the waypoints

  start =
      waypoints[0]; // Assign start as the first point of the list of waypoints

  // Now copy over to initialization
  for (size_t i = 0; i < waypoints.size(); i++) {
    this->waypoints.push_back(waypoints[i]);
  }
}

PurePursuitPath::PurePursuitPath(std::vector<TwoDPose> waypoints,
                                 double turning_radius,
                                 double lookahead_distance, double num_segments,
                                 double landing_strip)
    : turning_radius(turning_radius), lookahead_distance(lookahead_distance),
      num_segments(num_segments), landing_strip(landing_strip) {
  initializeWaypoints(waypoints);
  computeDubinsPath();
}

PurePursuitPath::PurePursuitPath(const TwoDPose start, const TwoDPose end,
                                 double turning_radius,
                                 double lookahead_distance, double num_segments,
                                 double landing_strip)
    : turning_radius(turning_radius), lookahead_distance(lookahead_distance),
      num_segments(num_segments), landing_strip(landing_strip) {
  initializeWaypoints({start, end});
  computeDubinsPath();
}

static int create_points_bridge(double q[3], double t, void *user_data) {
  return static_cast<PurePursuitPath *>(user_data)->create_points(q, t);
}

// Pretty much generates the path to drive through
void PurePursuitPath::computeDubinsPath() {
  // Wipe pursuit points
  pursuit_points = {};
  pursuit_checkpoints = {};

  path_valid = true;

  // Iterate through the waypoints, and generate the path
  for (size_t i = 0; i < waypoints.size() - 1; i++) {

    // Start of sub-section
    q0[0] = waypoints[i].x;
    q0[1] = waypoints[i].y;
    q0[2] = waypoints[i].yaw;

    // End of sub-section
    q1[0] = waypoints[i + 1].x;
    q1[1] = waypoints[i + 1].y;
    q1[2] = waypoints[i + 1].yaw;

    // Create shortest path and remember the status of the result
    int creation_result = dubins_shortest_path(&path, q0, q1, turning_radius);

    if (creation_result == EDUBOK) { // If no error
      if (i == 0) { // If first path (NOTE the first path determines step_size)
        t_max = dubins_path_length(&path);
        step_size = t_max / num_segments;
      } else { // Add on to path size
        t_max += dubins_path_length(&path);
      }

      // Generate the sample
      if (dubins_path_sample_many(&path, step_size, create_points_bridge,
                                  this) != EDUBOK) {
        path_valid = false;
        return;
      }
    } else {
      std::cout << "Creation result error: " << creation_result << std::endl;
      path_valid = false;
      return;
    }

    // Create a checkpoint at the halfway mark and append the checkpoint
    pursuitCheckpoint halfway_checkpoint(pursuit_points.size() - 1 -
                                         floatToInt(num_segments / 2));
    pursuit_checkpoints.push_back(halfway_checkpoint);

    // Create a checkpoint at the end and append the checkpoint
    pursuitCheckpoint checkpoint(pursuit_points.size() - 1);
    pursuit_checkpoints.push_back(checkpoint);
  }

  if (push_back_distance > 0) { // If the length of the landing strip
    // Create extrapolated forward steps that respect the pushed-back distance
    double dx = end.x - end_translated_back.x;
    double dy = end.y - end_translated_back.y;
    double distance = sqrt(dx * dx + dy * dy);
    t_max += distance;
    int n = static_cast<int>(distance /
                             step_size); // Calculate the number of full steps
    double fraction_step = step_size / distance;

    for (int i = 1; i < n; ++i) {
      double fraction = i * fraction_step;
      barebonesPose intermediate;
      intermediate.x = end_translated_back.x + fraction * dx;
      intermediate.y = end_translated_back.y + fraction * dy;
      pursuit_points.push_back(intermediate);
    }

    // Create a checkpoint at the end and append the checkpoint
    pursuitCheckpoint checkpoint(pursuit_points.size() - 1);
    pursuit_checkpoints.push_back(checkpoint);
  }
  pursuit_checkpoints[pursuit_checkpoints.size() - 1].is_last = true;
}

int PurePursuitPath::create_points(double q[3], double x) {
  barebonesPose p(q[0], q[1], q[2]);
  pursuit_points.push_back(p);
  return 0;
}

PursuitEstimate
PurePursuitPath::calculate_pursuit_estimate(TwoDPose current_position,
                                            bool find_closest_if_off_course,
                                            double deviation_min) {
  if (!path_valid) {
    return PursuitEstimate();
  }

  // Figure out the travel locations between checkpoints
  size_t start_i = 0;
  size_t end_i = 0;
  for (size_t i = 0; i < pursuit_checkpoints.size(); i++) {
    if (i == 0) { // If first index
      if (!pursuit_checkpoints[i].visited) {
        end_i = pursuit_checkpoints[i].i;
        break;
      }
    } else { // If not, get the indexes in between what is allowed
      if (!pursuit_checkpoints[i].visited || pursuit_checkpoints[i].is_last) {
        start_i = pursuit_checkpoints[i - 1].i;
        end_i = pursuit_checkpoints[i].i;
        break;
      }
    }
  }

  barebonesPose look_ahead_position;
  barebonesPose closest_position;
  bool lookahead_found = false;
  bool closest_found = false;

  double point_ahead_distance =
      lookahead_distance; // Distance from the current position to lookahead
                          // position
  double closest_distance = std::numeric_limits<double>::max();
  size_t closest_i = 0;

  double length_lookahead = 0; // The distance from the lookahead position to
                               // the point around the curve
  double length_closest = 0;

  int last_element = pursuit_points.size() - 1;

  double rough_distance;
  double distance;

  // Reverse iteration from last_element (which is size - 1) to index 1
  // We omit the first index 0 intentionally as that is the start location
  for (std::size_t i = last_element; i > 0; i--) {
    // Ensure it's within a valid checkpoint bounds
    if (i * step_size < start_i * step_size - lookahead_distance ||
        i * step_size > end_i * step_size + lookahead_distance) {
      continue;
    }

    // Rough distance first to avoid un-needed computational cost
    rough_distance =
        std::max(std::abs(pursuit_points[i].x - current_position.x),
                 std::abs(pursuit_points[i].y - current_position.y));
    if (rough_distance > closest_distance) {
      continue;
    }

    distance = sqrt(pow(pursuit_points[i].x - current_position.x, 2) +
                    pow(pursuit_points[i].y - current_position.y, 2));
    if (distance <= point_ahead_distance) {
      if (!lookahead_found) {
        point_ahead_distance = distance;
        look_ahead_position = pursuit_points[i];
        length_lookahead = (last_element - i) * step_size;
        lookahead_found = true;
      }
      if (!find_closest_if_off_course) {
        break;
      }
    }

    if (find_closest_if_off_course) {
      if (distance <= closest_distance) {
        closest_distance = distance;
        closest_position = pursuit_points[i];
        length_closest = (last_element - i) * step_size;
        closest_i = i;
        closest_found = true;
      }
    }
  }

  // Mark checkpoint as visited if reached
  for (size_t j = 0; j < pursuit_checkpoints.size(); j++) {
    if (int_distance(pursuit_checkpoints[j].i, closest_i) * step_size <
        lookahead_distance) {
      pursuit_checkpoints[j].visited = true;
    }
  }

  if (!lookahead_found) {
    if (!closest_found) {
      return PursuitEstimate(); // Return invalid pursuit estimate if no point
                                // is found
    }
    look_ahead_position = closest_position;
    point_ahead_distance = closest_distance;
    length_lookahead = length_closest;
  }

  double dx = look_ahead_position.x - current_position.x;
  double dy = look_ahead_position.y - current_position.y;
  double path_angle = atan2(dy, dx);
  double steering_angle = normalize_angle(path_angle - current_position.yaw);

  lookahead_pos = look_ahead_position;

  bool is_past_point = false;

  if (length_lookahead <= step_size) {
    // If robot is within a respective distance, then just turn
    /*double dist_from_end = std::sqrt(std::pow(q1[0] - current_position.x, 2)+
    std::pow(q1[1] - current_position.y, 2)); if(dist_from_end < deviation_min){
        steering_angle = normalize_angle(q1[2] - current_position.yaw);
    }*/

    // If the angle of the robot at the end point, facing the same direction
    // roughly as the end point, and over-passes (designated by steering angle)
    // Go in reverse
    if (std::abs(normalize_angle(end.yaw - current_position.yaw)) < M_PI_2 &&
        std::abs(steering_angle) > M_PI_2) {
      is_past_point = true;
      point_ahead_distance *= -1;
      steering_angle = normalize_angle(steering_angle + M_PI);
    }
  }

  double end_steering = normalize_angle(end.yaw - current_position.yaw);

  bool suggest_point_turn = false; // Suggesting point turn if the robot is
                                   // facing the opposite direction
  if (std::abs(steering_angle) > M_PI_2) {
    suggest_point_turn = true;
  }

  // point_ahead_distance is the distance from the robot to the center of the
  // track. length_lookahead is the path distance from target (like a racecar
  // track length to finish line)
  return PursuitEstimate(true, steering_angle,
                         point_ahead_distance + length_lookahead, is_past_point,
                         end_steering, suggest_point_turn);
}

} // namespace whoop
