/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       WhoopAutonSelector.cpp                                    */
/*    Author:       Connor White                                              */
/*    Created:      Thu Jun 21 2024                                           */
/*    Description:  Autonomous Selector Using Controller                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "whooplib/include/devices/WhoopAutonSelector.hpp"
#include "whooplib/include/toolbox.hpp"
#include <iostream>

namespace whoop {
WhoopAutonSelector::WhoopAutonSelector(WhoopController *whoop_controller,
                                       std::vector<AutonRoutine> routines,
                                       std::string auton_sd_save)
    : whoop_controller(whoop_controller), routines(routines),
      auton_sd_save(auton_sd_save), sd_reader(WhoopSD(auton_sd_save)) {}

void WhoopAutonSelector::update_selected_auton(int auton_choice) {

  if (auton_choice > routines.size() - 1) {
    auton_choice = 0;
  } else if (auton_choice < 0) {
    auton_choice = routines.size() - 1;
  }

  this->selected_auton = auton_choice;

  std::string auto_name = routines[selected_auton].auton_name;
  if (auton_sd_save != "" && sd_inserted()) {
    sd_reader.write_string(auto_name);
  }


  whoop_controller->display_text(">" + auto_name);
}

void WhoopAutonSelector::run_selector() {
  selector_running = true;
  if (auton_sd_save != "" && sd_inserted()) {
    std::string result = sd_reader.get_string();
    if (result != "FAILURE") { // If success
      // Manually search for the autonomous routine with the name from the file
      int found_index = -1;
      for (int i = 0; i < routines.size(); ++i) {
        if (routines[i].auton_name == result) {
          found_index = i;
          break;
        }
      }

      // If found, update the selected auton to the one from the file
      if (found_index != -1) {
        update_selected_auton(found_index);
      } else {
        // Handle the case where the auton name is not found
        whoop_controller->notify("SD Error, Defaulting");
        update_selected_auton(
            selected_auton); // Fallback to the current selection
      }
    } else {
      update_selected_auton(selected_auton);
    }
  } else {
    update_selected_auton(selected_auton);
  }
}

void WhoopAutonSelector::stop_selector() {
  selector_running = false;
  whoop_controller->clear_text();
}

void WhoopAutonSelector::run_autonomous() {
  stop_selector();
  routines[selected_auton].callback();
}

void WhoopAutonSelector::__step() {
  if (selector_running) {
    if (whoop_controller->right_pressing() && !button_pressing) {
      button_pressing = true;
      if(!whoop_controller->is_cleared) return;
      selected_auton++;
      update_selected_auton(selected_auton);
    } else if (whoop_controller->left_pressing() && !button_pressing) {
      button_pressing = true;
      if(!whoop_controller->is_cleared) return;
      selected_auton--;
      update_selected_auton(selected_auton);
    } else if (!whoop_controller->right_pressing() &&
               !whoop_controller->left_pressing() && button_pressing) {
      button_pressing = false;
    }
  }
}

} // namespace whoop