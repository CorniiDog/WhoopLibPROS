/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       BufferNode.hpp                                       */
/*    Author:       Connor White (WHOOP)                                      */
/*    Created:      Thu Jun 21 2024                                           */
/*    Description:  A Streamlined Communication System (like for Jetson nano) */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#ifndef BUFFER_NODE_HPP
#define BUFFER_NODE_HPP

#include "whooplib/include/nodes/NodeManager.hpp"
#include <unordered_map>
#include <string>
#include <functional>

namespace whoop
{

    /**
     * Enum for controlling whether messages should be deleted after reading.
     */
    enum deleteafterread
    {
        no_delete = true,
        yes_delete = false
    };

    /**
     * Enum for setting the debug mode of the BufferNode.
     */
    enum debugmode
    {
        debug_disabled = false,
        debug_enabled = true
    };

    class Messenger; // Forward declaration to allow reference within BufferNode

    /**
     * Manages message buffering and processing for inter-process or device communication.
     */
    class BufferNode : public ComputeNode
    {
    protected:
        int max_buffer_size;                                           // Maximum buffer size for storing messages.
        std::string serial_conn_out = MICRO_USB_SERIAL_CONNECTION_OUT; // Serial connection identifier for OUT.
        std::string serial_conn_in = MICRO_USB_SERIAL_CONNECTION_IN;   // Serial connection identifier for IN.

        // Additional modifiables
        std::string my_buffer = "";                            // Global buffer to store USB input data
        std::vector<Messenger *> registered_messengers;        // List of messengers registered to this buffer.
        std::unordered_map<std::string, std::string> messages; // Stored messages indexed by stream.
    public:
        bool debug_mode; // Debug mode state.

        /**
         * Constructor to initialize BufferNode with optional parameters.
         * @param maxBufferSize Maximum size of the buffer.
         * @param debugMode Initial state of debug mode.
         */
        BufferNode(int maxBufferSize = 512, debugmode debugMode = debugmode::debug_disabled); // Constructor declaration

        /**
         * Registers a messenger for listening to specific streams.
         * @param messenger Pointer to the Messenger to be registered.
         */
        void register_stream(Messenger *messenger); // Registers a stream for listening

        /**
         * Retrieves a message from a specified stream, optionally deleting it after reading.
         * @param stream The name of the stream to read from.
         * @param delete_after_read Whether to delete the message after reading.
         * @return The message as a string, or an empty string ("") if no message is available.
         */
        std::string get_message(std::string stream, bool delete_after_read = false); // Receive a message from a stream from USB (returns empty string if nothing)

        /**
         * Sends a message to a specified stream over USB.
         * Returns the result of the message.
         * @param stream The stream identifier.
         * @param message The message to send.
         * @param end The terminator string, defaults to newline ("\\n").
         * @return The result of the message:
         * 0 = successfully sent |
         * 1 = did not send successfully |
         * 2 = did not establish communication |
         * 3 = may have sent but did not close after writing
         */
        int send_message(std::string stream, std::string message, std::string end = "\n"); // Sends a message to a stream over USB
    protected:
        /**
         * Processes messages and manages buffer space.
         */
        void __step() override; // Protected helper function for processing steps
    };

    /**
     * Facilitates communication between nodes by managing messaging operations on a specified stream.
     */
    class Messenger
    {
    protected:
        BufferNode *buffer_system; // Buffer system managing this messenger.
    public:
        std::string messenger_stream;                                     // Stream identifier for this messenger.
        bool delete_after_read;                                           // Whether to delete messages after reading them.
        std::vector<std::function<void(std::string)>> callback_functions; // Callbacks registered for incoming messages.

        /**
         * Constructor to create a Messenger for a specific stream.
         * @param bufferSystem The BufferNode managing message buffering.
         * @param stream The stream identifier for this messenger.
         * @param deleteAfterRead Controls whether messages are deleted after reading.
         */
        Messenger(BufferNode *bufferSystem, std::string stream, deleteafterread deleteAfterRead = deleteafterread::no_delete);

        /**
         * Sends a message to the associated stream.
         * @param message The message to send (i.e. "Hello Jetson").
         */
        void send(std::string message); // Send message to stream

        /**
         * Reads the latest message from the associated stream.
         * @return The latest message as a string. May be an empty string ("") if no message received.
         */
        std::string read(); // Receive message from stream

        /**
         * Registers a callback function to be called when a new message is received on the stream.
         * @param callback The function to register as a callback.
         */
        void on_message(std::function<void(std::string)> callback);
    };

} // namespace whoop

#endif // BUFFER_NODE_HPP
