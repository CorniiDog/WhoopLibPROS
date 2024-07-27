/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       Includer.hpp                                              */
/*    Author:       Connor White (WHOOP)                                      */
/*    Created:      Thu Jun 21 2024                                           */
/*    Description:  Includes the VEXCode or PROS API                          */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#define USE_VEXCODE false // Change to false if not using VEXCode (i.e. PROS)

#ifndef INCLUDER_H
#define INCLUDER_H

#if USE_VEXCODE

#ifndef VEX_H
#define VEX_H
#include "vex.h"
#define USE_PROS false
#define MICRO_USB_SERIAL_CONNECTION_OUT "/dev/serial1"
#define MICRO_USB_SERIAL_CONNECTION_IN "/dev/serial1"
#endif // VEX_H

#else // !USE_VEXCODE (PROS)

#ifndef API_H
#define API_H
#include "api.h"
#define USE_PROS true
#define MICRO_USB_SERIAL_CONNECTION_OUT "sout"
#define MICRO_USB_SERIAL_CONNECTION_IN "sinp"

enum smart_ports
{
    PORT1 = 1,
    PORT2,
    PORT3,
    PORT4,
    PORT5,
    PORT6,
    PORT7,
    PORT8,
    PORT9,
    PORT10,
    PORT11,
    PORT12,
    PORT13,
    PORT14,
    PORT15,
    PORT16,
    PORT17,
    PORT18,
    PORT19,
    PORT20,
    PORT21,
    PORT22
};
#endif // API_H (or PROS)

#endif // USE_VEXCODE

#endif // INCLUDER_H