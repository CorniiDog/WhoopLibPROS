/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       RollingAverage.cpp                                        */
/*    Author:       Connor White (WHOOP)                                      */
/*    Created:      Thu Jun 21 2024                                           */
/*    Description:  Rolling Average Filter for Odometry                       */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "whooplib/includer.hpp"
#include "whooplib/include/calculators/RollingAverage.hpp"

namespace whoop
{

    RollingAverageFilter::RollingAverageFilter(int capacity) : capacity(capacity) {}

    Pose RollingAverageFilter::process(const Pose &newMeasurement)
    {
        if (buffer_pose.size() >= capacity)
        {
            buffer_pose.pop_front();
        }
        buffer_pose.push_back(newMeasurement);

        double sumX = 0, sumY = 0, sumZ = 0, sumPitch = 0, sumYaw = 0, sumRoll = 0, sumConfidence = 0;
        for (const auto &pose : buffer_pose)
        {
            sumX += pose.x;
            sumY += pose.y;
            sumZ += pose.z;
            sumPitch += pose.pitch;
            sumYaw += pose.yaw;
            sumRoll += pose.yaw;
            sumConfidence += pose.confidence;
        }
        int size = buffer_pose.size();
        return Pose(sumX / size, sumY / size, sumZ / size, sumPitch / size, sumYaw / size, sumRoll / size, sumConfidence / size);
    }

    double RollingAverageFilter::process(double newMeasurement)
    {
        if (buffer_double.size() >= capacity)
        {
            buffer_double.pop_front();
        }
        buffer_double.push_back(newMeasurement);

        double sum = 0;
        for (const auto &measurement : buffer_double)
        {
            sum += measurement;
        }
        int size = buffer_double.size();
        return sum / size;
    }

    int RollingAverageFilter::process(int newMeasurement)
    {
        if (buffer_int.size() >= capacity)
        {
            buffer_int.pop_front();
        }
        buffer_int.push_back(newMeasurement);

        double sum = 0;
        for (const auto &measurement : buffer_int)
        {
            sum += measurement;
        }
        int size = buffer_int.size();
        return (int)(sum / size);
    }

} // namespace whoop
