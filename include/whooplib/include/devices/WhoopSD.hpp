/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       WhoopController.hpp                                       */
/*    Author:       Connor White (WHOOP)                                      */
/*    Created:      Thu Jun 21 2024                                           */
/*    Description:  Virtual Controller With Additional Functions              */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#ifndef WHOOP_SD_HPP
#define WHOOP_SD_HPP

#include "whooplib/include/nodes/NodeManager.hpp"
#include "whooplib/includer.hpp"
#include <functional>
#include <vector>

namespace whoop {

class WhoopSD {
private:
    std::string file_name;
public:
    /**
     * Constructor for WhoopSD
     * @param file_name the file name to act upon (i.e. "hello.txt")
     */
    WhoopSD(std::string file_name);

    /**
     * Writes text to file in the sd card
     * @param text The text to put into the file (i.e. "Hi")
     */
    bool write_string(std::string text);

    /**
     * Gets text from file in the sd card
     */
    std::string get_string();
};


/**
 * Writes to the micro SD card
 * @param filename The name of the file to write to (i.e. "hello.txt")
 * @param text The text to put into the file (i.e. "Hi")
 * @returns A boolean. True if successfully worked, false otherwise
 */
bool write_string_to_sd(std::string filename, std::string text);

/**
 * Gets text from the sd card
 * @param filename The file to get text from (i.e. "hello.txt")
 * @returns The text. If there is a failure, however, it would return "FAILURE"
 */
std::string get_string_from_sd(std::string filename);

/**
 * Returns true if the MicroSD card is inserted
 */
bool sd_inserted();

} // namespace whoop

#endif