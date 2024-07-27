/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       WhoopVision.hpp                                           */
/*    Author:       Connor White (WHOOP)                                      */
/*    Created:      Thu Jun 21 2024                                           */
/*    Description:  A Streamlined Jetson Nano Vision System                   */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#ifndef WHOOP_VISION_HPP
#define WHOOP_VISION_HPP

#include "whooplib/includer.hpp"
#include "whooplib/include/nodes/NodeManager.hpp"
#include "whooplib/include/nodes/BufferNode.hpp"
#include "whooplib/include/calculators/TwoDPose.hpp"
#include <vector>
#include <memory>

namespace whoop
{

    /**
     * Struct representing a three-dimensional pose with orientation.
     * @param x meters (right-positive)
     * @param y meters (forward-positive)
     * @param z meters (up-positive, non-zero with vision system)
     * @param pitch radians (pitch-up-positive)
     * @param yaw radians (counter-clockwise-positive)
     * @param roll radians (counter-clockwise-positive)
     * @param confidence (non-zero with vision system) If for the vision system, it would output a value between [0,1] where 1 is the highest confidence. 0.3 or higher means it tracks, and 0.5 or higher means the tracking is good.
     */
    struct Pose
    {
        double x = 0, y = 0, z = 0;
        double pitch = 0, yaw = 0, roll = 0;
        double confidence = 0; // If for the vision system, it would output a value between 0 and 1 where 1 is the highest confidence. 0.3 or higher means it tracks, and 0.5 or higher means the tracking is good.
        Pose() {}
        Pose(double x, double y, double z, double pitch, double yaw, double roll, double confidence = 0) : x(x), y(y), z(z), pitch(pitch), yaw(yaw), roll(roll), confidence(confidence) {}
    };

    /**
     * Enum to control whether taring (resetting) operations should be applied.
     */
    enum tare_remaining_0
    {
        do_tare = true,
        dont_tare = false
    };

    /**
     * Represents an offset used for vision-based calculations.
     */
    class RobotVisionOffset
    {
    public:
        double x = 0;
        double y = 0;

        /**
         * Constructor to set the x and y offsets.
         * @param x Horizontal offset.
         * @param y Vertical offset.
         */
        RobotVisionOffset(double x, double y);
    };

    /**
     * Manages vision processing for robotics, handling pose estimation and transformations based on vision sensor input.
     */
    class WhoopVision
    {
    public:
        Pose raw_pose = Pose(); // Raw pose data from vision sensor.

    protected:
        // Upon initialization
        Messenger pose_messenger; // Handles messaging for pose data from Jetson Nano

        // Tares
        double tare_x = 0;
        double tare_y = 0;
        double tare_z = 0;
        double tare_pitch = 0;
        double tare_roll = 0;
        double tare_yaw = 0;
        double confidence = 0;

        double last_vision_message_time = 0;

        // Tared computes
        double tared_z = this->raw_pose.z - tare_z;
        double tared_pitch = this->raw_pose.pitch - tare_pitch;
        double tared_roll = this->raw_pose.roll - tare_roll;
        TwoDPose tared_position; // Position adjusted for tare.
        TwoDPose offset_change;  // Computed change due to offset adjustments.

        RobotVisionOffset *robot_offset; // Offset configuration for vision adjustments.

        std::vector<std::function<void(Pose)>> callback_functions; // Callbacks registered for incoming messages.

        /**
         * Transforms the raw pose data based on the current configuration and tare settings.
         * @param apply_delta Flag to determine if delta adjustments should be applied.
         */
        void _transform_pose(bool apply_delta = false);

        /**
         * Updates the pose based on incoming data.
         * @param pose_data The string containing serialized pose data.
         */
        void _update_pose(std::string pose_data);

    public:
        WhoopMutex thread_lock; // Mutex for synchronization of pose data updates.

        Pose pose; // The corrected and computed pose of the robot.

        /**
         * Constructor for initializing the vision system with a specific configuration.
         * @param robotOffset The offset configuration for vision calculations.
         * @param bufferSystem The buffer node system for data handling.
         * @param pose_stream The stream identifier for incoming pose data.
         */
        WhoopVision(RobotVisionOffset *robotOffset, BufferNode *bufferSystem, std::string pose_stream);

        // Taring (resetting) methods for the pose estimation.
        void tare(double x, double y, double z, double pitch, double yaw, double roll);
        void tare(double x, double y, double yaw, tare_remaining_0 tare_rest_to_zero);
        void tare(double x, double y, double yaw);
        void tare();

        void on_update(std::function<void(Pose)> callback);

        bool vision_running();

        /**
         * Retrieves the corrected and computed pose.
         * @return The current pose of the system.
         */
        Pose get_pose();
    };

} // namespace whoop

#endif // WHOOP_VISION_HPP
