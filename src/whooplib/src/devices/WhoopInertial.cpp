/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       WhoopInertial.cpp                                         */
/*    Author:       Connor White (WHOOP)                                      */
/*    Created:      Thu Jun 21 2024                                           */
/*    Description:  Virtual Interial Sensor that Follow Robotic Standard      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "whooplib/includer.hpp"
#include "whooplib/include/devices/WhoopInertial.hpp"
#include "whooplib/include/toolbox.hpp"

#include "whooplib/include/devices/WhoopMotor.hpp"

namespace whoop
{

    // Initialization Constructors
    WhoopInertial::WhoopInertial(std::int32_t port) :
#if USE_VEXCODE
                                                      vex::inertial(inertial(port))
#else
                                                      pros::IMU(port)
#endif
    {
    }

    WhoopInertial::WhoopInertial(std::int32_t port, double correction_multiplier) : WhoopInertial(port)
    {
        this->correction_multiplier = correction_multiplier;
    }

    // Receiving rotation
    double WhoopInertial::get_yaw()
    {
#if USE_VEXCODE
        double yaw = -(vex::inertial::heading(rotationUnits::deg) * correction_multiplier);
#else
        double yaw = -(pros::IMU::get_heading() * correction_multiplier);
#endif

        yaw += yaw_offset;

        // Normalize the yaw to be within -180 to 180 degrees
        yaw = fmod(yaw + 180, 360);
        if (yaw < 0)
        {
            yaw += 360;
        }
        yaw -= 180;

        return yaw;
    }

    double WhoopInertial::get_yaw_degrees()
    {
        return this->get_yaw();
    }

    double WhoopInertial::get_yaw_radians()
    {
        return to_rad(this->get_yaw());
    }

    double WhoopInertial::get_roll()
    {
#if USE_VEXCODE
        return vex::inertial::roll();
#else
        return pros::IMU::get_roll();
#endif
    }

    double WhoopInertial::get_roll_degrees()
    {
        return get_roll();
    }

    double WhoopInertial::get_roll_radians()
    {
        return to_rad(get_roll());
    }

    double WhoopInertial::get_pitch()
    {
#if USE_VEXCODE
        return vex::inertial::pitch();
#else
        return pros::IMU::get_roll();
#endif
    }

    double WhoopInertial::get_pitch_degrees()
    {
        return get_pitch();
    }

    double WhoopInertial::get_pitch_radians()
    {
        return to_rad(get_pitch());
    }

    // Calibrate
    void WhoopInertial::calibrate()
    {
#if USE_VEXCODE
        vex::inertial::calibrate();
#else
        pros::IMU::reset();
#endif
    }

    // Tare (reset)
    void WhoopInertial::tare()
    {
        this->tare(0);
    }
    void WhoopInertial::tare(double degrees)
    {
        yaw_offset = degrees;
#if USE_VEXCODE
        vex::inertial::resetHeading();
#else
        pros::IMU::tare_heading();
#endif
    }
    void WhoopInertial::tare_degrees(double degrees)
    {
        this->tare(degrees);
    }
    void WhoopInertial::tare_radians(double radians)
    {
        this->tare(to_deg(radians));
    }

} // namespace whoop
