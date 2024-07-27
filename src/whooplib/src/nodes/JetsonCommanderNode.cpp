/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       JetsonCommanderNode.cpp                                   */
/*    Author:       Connor White (WHOOP)                                      */
/*    Created:      Thu Jun 21 2024                                           */
/*    Description:  A keep-alive system for the Jetson Nano                   */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "whooplib/include/nodes/JetsonCommanderNode.hpp"
#include "whooplib/include/toolbox.hpp"
#include <unordered_map>
#include <string>
#include <functional>

namespace whoop
{

    void JetsonCommander::setup_messenger(BufferNode *bufferSystem, const std::string &pose_stream)
    {
        keepalive_messenger = std::make_unique<Messenger>(bufferSystem, pose_stream, deleteafterread::no_delete);
        keepalive_messenger->on_message(std::bind(&JetsonCommander::_on_message_received, this, std::placeholders::_1));
    }

    void JetsonCommander::_on_message_received(std::string message)
    {
        raw_connected += 2;
        if (raw_connected > 5)
        {
            raw_connected = 5;
        }

        if (message == "Hello")
        {
            keepalive_messenger->send(intToString(keep_alive_time_seconds)); //+ " " + "Initialize");
        }
        else if (message == "Rebooting")
        { // If failed to initialize realsense system
            if (!comms_disabled)
            {
                controller_for_messages->notify("Rebooting Jetson", 2);
            }
        }
        else if (message == "ReInitializing" || message == "Initializing")
        { // If failed to initialize realsense system
            if (!comms_disabled)
            {
                controller_for_messages->notify("Initializing Jetson", 2);
            }
        }
        else if (message == "Failed")
        { // If failed to initialize realsense system
            if (!comms_disabled)
            {
                controller_for_messages->notify("Replug RSense USBs", 2);
            }
        }
    }

    JetsonCommander::JetsonCommander(WhoopController *controller_for_messages, BufferNode *bufferSystem, std::string communication_stream, int keep_alive_time_seconds, int step_time_s, jetsonCommunication enable_jetson_comms)
    {
        if (enable_jetson_comms == jetsonCommunication::disable_comms)
        {
            comms_disabled = true;
        }
        this->controller_for_messages = controller_for_messages;
        this->keep_alive_time_seconds = keep_alive_time_seconds;
        setup_messenger(bufferSystem, communication_stream);
        this->set_step_time(step_time_s * 1000); // Configure step time in milliseconds
    }

    void JetsonCommander::reboot_jetson()
    {
        keepalive_messenger->send("Reboot");
    }

    void JetsonCommander::shutdown_jetson()
    {
        keepalive_messenger->send("Shutdown");
    }

    void JetsonCommander::restart_vision_process()
    {
        keepalive_messenger->send("RestartProcess");
    }

    bool JetsonCommander::is_connected_to_jetson()
    {
        return connected;
    }

    void JetsonCommander::initialize()
    {
        keepalive_messenger->send(intToString(keep_alive_time_seconds) + " " + "Initialize");
    }

    void JetsonCommander::__step()
    {

        // Logic for determining connection
        if (raw_connected > 0)
        {
            connected = true;
        }
        else
        {
            connected = false;
        }

        if (raw_connected <= 0)
        {
            raw_connected = 0;
            if (!comms_disabled)
            {
                controller_for_messages->notify("Jetson Disconnected", 1);
            }
        }
        else if (raw_connected > 5)
        {
            raw_connected = 5;
        }

        raw_connected -= 1;

        keepalive_messenger->send(intToString(keep_alive_time_seconds));
    }

} // namespace whoop
