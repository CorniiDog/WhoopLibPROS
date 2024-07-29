/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       NodeManager.cpp                                           */
/*    Author:       Connor White (WHOOP)                                      */
/*    Created:      Thu Jun 21 2024                                           */
/*    Description:  Manages the Nodes (Anything that extends ComputeNode)     */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "whooplib/include/nodes/NodeManager.hpp"
#include "whooplib/include/toolbox.hpp"
#include <algorithm>
#include <cstdio>
#include <iostream>
#include <stdexcept>

namespace whoop {

////////////////////////////////////////////////////////////////////////////////
// Node Class Manager
////////////////////////////////////////////////////////////////////////////////

// ComputeManager Methods
ComputeManager::ComputeManager(bool debugMode) : debug_mode(debugMode) {}

// ComputeManager Methods
ComputeManager::ComputeManager(std::vector<ComputeNode *> nodes, bool debugMode)
    : debug_mode(debugMode) {
  for (size_t i = 0; i < nodes.size(); ++i) {
    ComputeNode *node = nodes[i];
    add_compute_node(node);
  }
}

void ComputeManager::add_compute_node(ComputeNode *node) {
  computes.push_back(node);
  node->lock_ptr = &thread_lock; // Assign the manager's mutex to the node
}

void ComputeManager::start() {
  for (auto &compute : computes) {
    compute->start_pipeline(debug_mode);
  }
}

////////////////////////////////////////////////////////////////////////////////
// Compute Node Base
////////////////////////////////////////////////////////////////////////////////

void placeholder_task() {}

// ComputeNode Methods
ComputeNode::ComputeNode() {}

int ComputeNode::task_runner(void *param) {
  auto *node = static_cast<ComputeNode *>(param);

  int start_time = 0;
  if (node->omit_steptime_compensation) {
    node->initial_computational_time = 0;
  } else {
    node->initial_computational_time = -1;
  }

  while (node->node_running) {
    if (node->node_debug) // If in debug mode, simply step knowing the
                          // consequence of an error breaking the thread
    {
      node->__step();
    } else {
      try {
        if (node->initial_computational_time ==
            -1) { // Try to accomodate process time to improve accuracy
#if USE_VEXCODE
          start_time = Brain.timer(timeUnits::msec);
#else
          start_time = pros::c::millis();
#endif
        }

        node->__step();

        if (node->initial_computational_time == -1) {
#if USE_VEXCODE
          int end_time = Brain.timer(timeUnits::msec) - start_time;
#else
          int end_time = pros::c::millis() - start_time;
#endif

          if (end_time < node->step_time_ms) { // Accept if only within
                                               // acceptable threshold.
#if USE_VEXCODE
            node->initial_computational_time =
                Brain.timer(timeUnits::msec) - start_time;
#else
            node->initial_computational_time = pros::c::millis() - start_time;
#endif
          } else { // Assume it takes same processing time as step_time_ms
            node->initial_computational_time = node->step_time_ms;
          }
        }
      } catch (const std::exception &e) {
#if USE_VEXCODE
        Brain.Screen.clearLine(1);
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("Error: %s", e.what());
#else
        pros::lcd::clear_line(1);
        pros::lcd::print(1, "Error: %s", e.what());
#endif
      }
    }

    // Initial computational time application for delay
    if (node->initial_computational_time > 0 &&
        node->initial_computational_time < node->step_time_ms) {
#if USE_VEXCODE
      vex::wait(node->step_time_ms - node->initial_computational_time,
                vex::timeUnits::msec);
#else
      pros::delay(node->step_time_ms - node->initial_computational_time);
#endif
    } else if (node->initial_computational_time >= node->step_time_ms) {
#if USE_VEXCODE
      vex::wait(0, vex::timeUnits::msec);
#else
      pros::delay(0);
#endif
    } else { // Omit and just wait using step_time_ms
#if USE_VEXCODE
      vex::wait(node->step_time_ms, vex::timeUnits::msec);
#else
      pros::delay(node->step_time_ms);
#endif
    }
  }
  return 1;
}

void ComputeNode::task_runner_void(void *param) { task_runner(param); }

void ComputeNode::start_pipeline(bool debug_mode) {
#if USE_VEXCODE
  if (node_running) // Unfortunately cannot do anything about this... Cannot
                    // check if the task is running
  {
    return;
  }
  vex::task vexTask(
      ComputeNode::task_runner,
      this); // VEXCode requires an int return variable (hence task_runner)
#else
  if (node_running) {
    return;
  }
  pros::Task(ComputeNode::task_runner_void, this,
             ""); // PROS requires no return variable (hence task_runner_void)
#endif
  node_debug = debug_mode;
  node_running = true;
}

void ComputeNode::stop_pipeline() { node_running = false; }

void ComputeNode::set_step_time(
    int step_time_ms, omitStepCompensation omit_steptime_compensation) {
  if (step_time_ms < 0) {
    throw std::invalid_argument("Step-time must be positive or zero.");
  }
  this->step_time_ms = step_time_ms;
  this->omit_steptime_compensation = omit_steptime_compensation;
}

void ComputeNode::__step() {
  if (lock_ptr) {
    lock_ptr->lock(); // Acquire the mutex
    // Simulate some computations or operations here
    lock_ptr->unlock(); // Release the mutex
  }
}

} // namespace whoop
