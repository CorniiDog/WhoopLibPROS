/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       WhoopAutonSelector.hpp                                    */
/*    Author:       Connor White                                              */
/*    Created:      Thu Jun 21 2024                                           */
/*    Description:  Autonomous Selector Using Controller                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#ifndef WHOOP_AUTON_SELECTOR_HPP
#define WHOOP_AUTON_SELECTOR_HPP

#include "whooplib/include/devices/WhoopController.hpp"
#include "whooplib/include/devices/WhoopSD.hpp"
#include "whooplib/include/nodes/NodeManager.hpp"
#include "whooplib/includer.hpp"

namespace whoop {

struct AutonRoutine {
  std::string auton_name;
  std::function<void()> callback;
  AutonRoutine(std::string auton_name, std::function<void()> callback)
      : auton_name(auton_name), callback(callback) {}
};

class WhoopAutonSelector : public ComputeNode {
private:
  WhoopController *whoop_controller;
  std::vector<AutonRoutine> routines;

  int selected_auton = 0;
  bool selector_running = false;
  std::string auton_sd_save;
  WhoopSD sd_reader;
  bool button_pressing = false;

  void update_selected_auton(int auton_choice);


public:
  WhoopAutonSelector(WhoopController *whoop_controller, std::vector<AutonRoutine> routines, std::string auton_sd_save = "");

    /**
     * Runs the selector protocol
     * Usually ran during initialization
     */
    void run_selector();

    /** 
     * Stops the auton selector protocol.
     * This gets auto-ran when run_autonomous runs.
    */
    void stop_selector();

    /**
     * Runs the internally selected autonomous.
     * Recommended to run in autonomous()
     */
    void run_autonomous();

private:
    void __step() override;
};

} // namespace whoop

#endif // WHOOP_AUTON_SELECTOR_HPP