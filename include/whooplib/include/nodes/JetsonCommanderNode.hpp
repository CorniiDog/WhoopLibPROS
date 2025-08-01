/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       JetsonCommanderNode.hpp                                   */
/*    Author:       Connor White                                              */
/*    Created:      Thu Jun 21 2024                                           */
/*    Description:  A keep-alive system for the Jetson Nano                   */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#ifndef JETSON_COMMANDER_HPP
#define JETSON_COMMANDER_HPP

#include "whooplib/include/devices/WhoopController.hpp"
#include "whooplib/include/nodes/BufferNode.hpp"
#include "whooplib/include/nodes/NodeManager.hpp"
#include <functional>
#include <memory> // For std::unique_ptr
#include <string>
#include <unordered_map>

namespace whoop {

enum jetsonCommunication { enable_comms = true, disable_comms = false };

class JetsonCommander : public ComputeNode {
private:
  int time_waited_ms = 0;

  void setup_messenger(BufferNode *bufferSystem,
                       const std::string &pose_stream);
  void _on_message_received(std::string message);
  WhoopController *controller_for_messages;

  int raw_connected = 5;

  int keep_alive_time_seconds;

  bool comms_disabled = false;

public:
  std::unique_ptr<Messenger> keepalive_messenger =
      nullptr; // Handles messaging for pose data from Jetson Nano

  bool connected = false;

  /**
   * Commander for the Jetson Nano
   * @param controller_for_messages Controller to send notifications
   * @param bufferSystem The buffer master
   * @param communication_stream The communication stream identifier
   * @param keep_alive_time_seconds In seconds. When the V5 Brain shuts down or
   * disconnects, the Jetson Nano will keep the program running for this time
   * before it shuts off
   * @param step_time_ms How many seconds to wait before sending anoter keep
   * alive message to Jetson (suggested 2)
   * @param enable_jetson_comms If you don't have a Vision Tesseract on your
   * robot, set to disable_comms
   */
  JetsonCommander(WhoopController *controller_for_messages,
                  BufferNode *bufferSystem, std::string communication_stream,
                  int keep_alive_time_seconds, int step_time_ms,
                  jetsonCommunication enable_jetson_comms);

  /**
   * Restarts Jetson Nano
   */
  void reboot_jetson();

  /**
   * Shuts down Jetson Nano
   */
  void shutdown_jetson();

  /**
   * Restarts the vision process on Jetson Nano
   */
  void restart_vision_process();

  /**
   * Sends initialization message to Jetson Nano
   */
  void initialize();

  /**
   * Returns true if connected to Jetson Nano
   * @returns true if connected, false otherwise (delay 5-6 seconds)
   */
  bool is_connected_to_jetson();

  /**
   * Processes messages and manages buffer space.
   */
  void __step() override; // Protected helper function for processing steps
};

} // namespace whoop

#endif // JETSON_COMMANDER_HPP