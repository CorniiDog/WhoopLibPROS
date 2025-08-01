/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       WhoopSD.cpp                                               */
/*    Author:       Connor White                                              */
/*    Created:      Thu Jun 21 2024                                           */
/*    Description:  Whoop SD Card Reading Utility                             */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "whooplib/includer.hpp"
#include "whooplib/include/devices/WhoopSD.hpp"
#include <cmath>
#include <fstream>
#include <memory>
#include <sstream>
#include <vector>
#include <string>

namespace whoop {

WhoopSD::WhoopSD(std::string file_name): file_name(file_name){}

bool WhoopSD::write_string(std::string text){
    return write_string_to_sd(file_name, text);
}

std::string WhoopSD::get_string(){
    return get_string_from_sd(file_name);
}

int tries = 0;

bool write_string_to_sd(std::string filename, std::string text) {
#if USE_PROS
  filename = "/usd/" + filename;
#endif
  if (sd_inserted()) {
    std::ofstream outfs(filename);
    if (outfs.is_open() && outfs.good()) {
      outfs.clear();
      outfs << text << std::endl;
      return true;
    } else {
      ++tries;
      if (tries < 5) {
#if USE_VEXCODE
        vex::task::sleep(100);
#else
        pros::delay(100);
#endif
        return write_string_to_sd(filename, text);
      }
      return false;
    }
  } else {
    return false;
  }
}

bool sd_inserted() {
#if USE_VEXCODE
  return Brain.SDcard.isInserted();
#else
  return pros::usd::is_installed();
#endif
}

std::string get_string_from_sd(std::string filename) {
#if USE_PROS
  filename = "/usd/" + filename;
#endif
  if (sd_inserted()) {
    std::ifstream infs(filename);
    if (infs.is_open() && infs.good()) {
      std::stringstream buffer;
      buffer << infs.rdbuf();  // Read entire file into a stringstream
      std::string text = buffer.str();

      // Trim newline characters from the beginning and end
      text.erase(0, text.find_first_not_of("\n\r"));
      text.erase(text.find_last_not_of("\n\r") + 1);

      return text;
    }
  }
  return "FAILURE";
}

} // namespace whoop