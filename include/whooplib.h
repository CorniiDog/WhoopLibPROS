/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       whooplib.h                                                */
/*    Author:       Aggie Robotics                                            */
/*    Created:      Thu Jun 21 2024                                           */
/*    Description:  Primary Headers for WhoopLib                              */
/*                                                                            */
/*----------------------------------------------------------------------------*/

/**
 * MIT License
 *
 * Copyright (c) 2024 Connor White
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef WHOOP_LIB_H
#define WHOOP_LIB_H

#include "whooplib/includer.hpp"

// Toolbox
#include "whooplib/include/toolbox.hpp"

// Calculators
#include "whooplib/include/calculators/Dubins.hpp"
#include "whooplib/include/calculators/PurePursuit.hpp"
#include "whooplib/include/calculators/PurePursuitConductor.hpp"
#include "whooplib/include/calculators/RollingAverage.hpp"
#include "whooplib/include/calculators/Slew.hpp"
#include "whooplib/include/calculators/TwoDPose.hpp"
#include "whooplib/include/calculators/Units.hpp"
#include "whooplib/include/calculators/WheelOdom.hpp"

// Nodes
#include "whooplib/include/nodes/BufferNode.hpp"
#include "whooplib/include/nodes/JetsonCommanderNode.hpp"
#include "whooplib/include/nodes/NodeManager.hpp"
#include "whooplib/include/toolbox.hpp"

// Devices
#include "whooplib/include/devices/WhoopController.hpp"
#include "whooplib/include/devices/WhoopDriveOdomOffset.hpp"
#include "whooplib/include/devices/WhoopDriveOdomUnit.hpp"
#include "whooplib/include/devices/WhoopDrivetrain.hpp"
#include "whooplib/include/devices/WhoopInertial.hpp"
#include "whooplib/include/devices/WhoopMotor.hpp"
#include "whooplib/include/devices/WhoopMotorGroup.hpp"
#include "whooplib/include/devices/WhoopMutex.hpp"
#include "whooplib/include/devices/WhoopOdomFusion.hpp"
#include "whooplib/include/devices/WhoopRotation.hpp"
#include "whooplib/include/devices/WhoopVision.hpp"

using namespace whoop; // This is to help newer teams get used to C/C++. If you
                       // want to be more explicit, remove this and use
                       // whoop::abc instead

#endif // WHOOP_LIB_H