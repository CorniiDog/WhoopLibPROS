/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       NodeManager.hpp                                           */
/*    Author:       Connor White (WHOOP)                                      */
/*    Created:      Thu Jun 21 2024                                           */
/*    Description:  Manages the Nodes (Anything that extends ComputeNode)     */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#ifndef NODE_MANAGER_HPP
#define NODE_MANAGER_HPP

#include "whooplib/include/devices/WhoopMutex.hpp"
#include "whooplib/includer.hpp"
#include <vector>

namespace whoop {

enum omitStepCompensation { yes_omit = true, dont_omit = false };

class ComputeNode; // Forward declaration to allow references in ComputeManager

/**
 * Manages a collection of ComputeNode instances, facilitating controlled
 * execution and synchronization.
 */
class ComputeManager {
private:
  bool running = false;

public:
  WhoopMutex thread_lock; // Mutex for synchronizing access to compute nodes
  std::vector<ComputeNode *>
      computes;    // Vector storing pointers to compute nodes
  bool debug_mode; // Flag to enable debug mode for additional logging and
                   // diagnostics

  /**
   * Default constructor initializing the ComputeManager with optional debug
   * mode.
   * @param debugMode Enables debug mode if set to true.
   */
  ComputeManager(bool debugMode = false);

  /**
   * Constructor to initialize the ComputeManager with a list of compute nodes
   * and optional debug mode.
   * @param nodes Initial list of compute nodes to manage.
   * @param debugMode Enables debug mode if set to true.
   */
  ComputeManager(std::vector<ComputeNode *> nodes, bool debugMode = false);

  /**
   * Adds a compute node to the manager's list of nodes.
   * @param node Pointer to the ComputeNode to add.
   */
  void add_compute_node(ComputeNode *node);

  /**
   * Starts the computation process for all managed compute nodes.
   */
  void start();
};

/**
 * Abstract class representing a node capable of performing computation or
 * processing tasks.
 */
class ComputeNode {
public:
  WhoopMutex *lock_ptr = nullptr; // Pointer to a mutex for synchronization,
                                  // typically shared with a ComputeManager
  bool node_running =
      false; // Flag indicating whether the node's computation task is active
  bool node_debug = false; // Flag to enable debug mode for this specific node
  int step_time_ms = 10;   // time between each computational activity
  bool omit_steptime_compensation = false;
  int initial_computational_time =
      0; // Time to process data (to try to adapt step time to be more precise)

  /**
   * Constructor for ComputeNode.
   */
  ComputeNode(); // Constructor

  /**
   * Starts the computation pipeline, utilizing an internal mutex pointer for
   * synchronization.
   * @param debug_mode Enables debug mode for this computation cycle if set to
   * true.
   */
  void start_pipeline(
      bool debug_mode =
          false); // Starts the computation process using internal mutex pointer

  /**
   * Stops the computation pipeline, terminating any ongoing tasks.
   */
  void stop_pipeline(); // Stops the computation process

  /**
   * Sets step time between each step for the node. The compute node would
   * compensate the time to complete a step procedure to be as close to
   * step_time_ms as possible, based upon the first initial step.
   * @param step_time_ms time between each computational activity
   * @param omit_steptime_compensation Set to "omitStepCompensation::yes_omit"
   * to omit step compensation
   */
  void set_step_time(
      int step_time_ms,
      omitStepCompensation omit_steptime_compensation =
          omitStepCompensation::dont_omit); // Stops the computation process
protected:
  /**
   * Virtual function intended to be overridden by derived classes to implement
   * specific computation steps.
   */
  virtual void __step(); // Protected helper function for processing steps

  /**
   * Static function that serves as a task runner for a computation process,
   * compatible with the VEX task management.
   * @param param Generic pointer to any data needed for the task, typically
   * pointing to an instance of ComputeNode.
   * @return Returns an integer status code, generally used for debugging or
   * error handling.
   */
  static int task_runner(void *param); // Static task runner function for VEX

  /**
   * Static function that serves as a task runner for a computation process,
   * compatible with the VEX task management.
   * @param param Generic pointer to any data needed for the task, typically
   * pointing to an instance of ComputeNode.
   * @return Returns an integer status code, generally used for debugging or
   * error handling.
   */
  static void
  task_runner_void(void *param); // Static task runner function for VEX
};

} // namespace whoop

#endif // NODE_MANAGER_HPP
