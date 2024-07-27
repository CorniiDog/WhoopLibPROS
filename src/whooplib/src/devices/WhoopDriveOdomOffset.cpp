/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       WhoopDriveOdomOffset.cpp                                  */
/*    Author:       Connor White (WHOOP)                                      */
/*    Created:      Thu Jun 21 2024                                           */
/*    Description:  Odometry Offset Module for Pose Estimation                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "whooplib/include/nodes/NodeManager.hpp"
#include "whooplib/include/calculators/TwoDPose.hpp"
#include "whooplib/include/devices/WhoopDriveOdomOffset.hpp"
#include "whooplib/includer.hpp"
#include <vector>
#include <memory>
#include <iostream>

namespace whoop
{

    WhoopDriveOdomOffset::WhoopDriveOdomOffset(WhoopDriveOdomUnit *odom_unit, double x_offset, double y_offset) : offset(x_offset, -y_offset, 0)
    { // The x and y offsets are flipped... Idk why it just is.
        this->odom_unit = odom_unit;
    }

    void WhoopDriveOdomOffset::calibrate()
    {
        thread_lock.lock();
        odom_unit->calibrate();
        thread_lock.unlock();
        tare();
    }

    void WhoopDriveOdomOffset::tare(double x, double y, double yaw)
    {
        thread_lock.lock();
        is_clean = false;
        TwoDPose TaredOffset = TwoDPose(x, y, yaw);

        // If there is an offset
        if (offset.x != 0 || offset.y != 0 || offset.yaw != 0)
        {
            TaredOffset *= offset;
        }

        odom_unit->tare(TaredOffset.x, TaredOffset.y, TaredOffset.yaw);

        if (offset.x == offset.y == offset.yaw == 0)
        {                           // If offset is not applied
            pose = odom_unit->pose; // Update pose without offset, to reduce computational time
        }
        else
        {
            pose = odom_unit->pose * -offset; // Update pose with offset
        }

        last_pose = pose; // Just set last_pose to pose to prevent it from flying out the wazoo

        thread_lock.unlock();
    }

    void WhoopDriveOdomOffset::tare()
    {
        tare(0, 0, 0);
    }

    TwoDPose WhoopDriveOdomOffset::get_pose()
    {
        thread_lock.lock();
        TwoDPose result = pose;
        thread_lock.unlock();
        return result;
    }

    TwoDPose WhoopDriveOdomOffset::get_last_pose()
    {
        thread_lock.lock();
        TwoDPose result = last_pose;
        thread_lock.unlock();
        return result;
    }

    void WhoopDriveOdomOffset::__step_down()
    {
        thread_lock.lock();
        odom_unit->__step();
        thread_lock.unlock();

        this->__step();
    }

    bool WhoopDriveOdomOffset::is_moving(double rads_s_threshold)
    {
        return odom_unit->is_moving(rads_s_threshold);
    }

    velocityVector WhoopDriveOdomOffset::get_velocity_vector()
    {
        thread_lock.lock();
        velocityVector vel((pose.x - last_pose.x) / 0.01, (pose.y - last_pose.y) / 0.01, (pose.yaw - last_pose.yaw) / 0.01, is_clean);
        thread_lock.unlock();

        return vel;
    }

    velocityVector WhoopDriveOdomOffset::get_velocity_vector(TwoDPose offset)
    {
        thread_lock.lock();
        TwoDPose p = pose * offset; // Apply offset to position of realsense device, or whatever
        TwoDPose l_p = last_pose * offset;
        velocityVector vel((p.x - l_p.x) / 0.01, (p.y - l_p.y) / 0.01, (p.yaw - l_p.yaw) / 0.01, is_clean);
        thread_lock.unlock();

        return vel;
    }

    void WhoopDriveOdomOffset::__step()
    {
        thread_lock.lock();
        last_pose = pose;
        is_clean = true;

        if (offset.x == offset.y == offset.yaw == 0)
        {                           // If offset is not applied
            pose = odom_unit->pose; // Update pose without offset, to reduce computational time
        }
        else
        {
            pose = odom_unit->pose * -offset; // Update pose with offset
        }
        thread_lock.unlock();
    }

} // namespace whoop
