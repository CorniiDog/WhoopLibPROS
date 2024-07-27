#include "main.h"

////////////////////////////////////////////////////////////
/**
 *    Globals
 */
////////////////////////////////////////////////////////////

// Primary controller
WhoopController controller1(joystickmode::joystickmode_split_arcade, controllertype::controller_primary);

// Left drive motors
WhoopMotor l1(PORT12, cartridge::blue, reversed::yes_reverse);
WhoopMotor l2(PORT13, cartridge::blue, reversed::yes_reverse);
WhoopMotor l3(PORT14, cartridge::blue, reversed::yes_reverse);
WhoopMotor l4(PORT15, cartridge::blue, reversed::yes_reverse);
WhoopMotorGroup left_motors({&l1, &l2, &l3, &l4});

// Right drive motors
WhoopMotor r1(PORT1, cartridge::blue, reversed::no_reverse);
WhoopMotor r2(PORT2, cartridge::blue, reversed::no_reverse);
WhoopMotor r3(PORT3, cartridge::blue, reversed::no_reverse);
WhoopMotor r4(PORT4, cartridge::blue, reversed::no_reverse);
WhoopMotorGroup right_motors({&r1, &r2, &r3, &r4});

// Sensors
WhoopInertial inertial_sensor(PORT7);
WhoopRotation forward_tracker(PORT6, reversed::no_reverse);
WhoopRotation sideways_tracker(PORT9, reversed::no_reverse);

// ////////////////////////////////////////////////////////////
// /**
//  *    Wheel Odometry Configuration
//  */
// ////////////////////////////////////////////////////////////

WhoopDriveOdomUnit odom_unit(
    to_meters(1.51),   // The forward tracker distance, in meters, from the odom unit's center. (positive implies a shift to the right from the odom unit's center)
    to_meters(2.5189), // Diameter of the forward tracker, in meters (e.g., 0.08255 for 3.25-inch wheels).
    to_meters(-4.468), // The sideways tracker distance, in meters, from the odom unit's center (positive implies a shift forward from the odom unit center)
    to_meters(2.5189), // Diameter of the sideways tracker, in meters (e.g., 0.08255 for 3.25-inch wheels).
    &inertial_sensor,  // Pointer to the WhoopInertial sensor
    &forward_tracker,  // Pointer to the forward tracker, as a WhoopRotation sensor
    &sideways_tracker  // Pointer to the sideways tracker, as a WhoopRotation sensor
);

WhoopDriveOdomOffset odom_offset(
    &odom_unit,      // Pointer to the odometry unit (will manage the odom unit)
    to_meters(-0.6), // The x offset of the odom unit from the center of the robot (positive implies a shift right from the center of the robot).
    to_meters(4.95)  // The y offset of the odom unit from the center of the robot (positive implies a shift forward from the center of the robot).
);

// ////////////////////////////////////////////////////////////
// /**
//  *    VISION TESSERACT
//  */
// ////////////////////////////////////////////////////////////

// Serial communication module
BufferNode buffer_system(
    256,                      // The buffer size, in characters. Increase if necessary, but at the cost of computational efficiency.
    debugmode::debug_disabled // debugMode::debug_disabled for competition use, debugMode::debug_enabled to allow the code to pass errors through
);

// Vision Offset of the Vision Tesseract from the Center of Robot
RobotVisionOffset vision_offset(
    0.0,           // The x offset in meters, (right-positive from the center of the robot).
    220.0 / 1000.0 // The y offset in meters (forward-positive from the center of the robot).
);

// Jetson Nano pose retreival object (also configured on Nano-side)
WhoopVision vision_system(
    &vision_offset, // pointer to the vision offset
    &buffer_system, // Pointer to the buffer system (will be managed by the buffer system)
    "P"             // The subscribed stream name to receive the pose from the Jetson Nano
);

// This is the jetson commander. It sends keep-alive messages intermittently and also commands the Jetson Nano.
// This is essential to ensure that the nano starts its internal program, stop program, restarts program,
// and can be told to reboot or shutdown.
JetsonCommander jetson_commander(
    &controller1,                      // The controller to send messages to upon error
    &buffer_system,                    // Pointer to the buffer system (will be managed by the buffer system)
    "C",                               // The subscribed stream name for keep-alive, shutdown, and reboot
    60,                                // In seconds. When the V5 Brain shuts down or disconnects, the Jetson Nano will keep the program running for this time before it shuts off
    2,                                 // How many seconds to wait before sending anoter keep alive message to Jetson (suggested 2)
    jetsonCommunication::disable_comms // If you don't have a Vision Tesseract on your robot, set to disable_comms
);

////////////////////////////////////////////////////////////
/**
 *    Vision x Wheel Odometry Fusion
 */
////////////////////////////////////////////////////////////
WhoopOdomFusion odom_fusion(
    &vision_system,              // Pointer to the vision system
    &odom_offset,                // Pointer to the odometry offset
    0.9,                         // Minimum confidence threshold to apply vision system to odometry
    fusionmode::wheel_odom_only, // The method of fusing
    to_meters(50),               // If FusionMode is fusion_gradual, it is the maximum allowable lateral shift the vision camera can update in meters per second.
    to_rad(500)                  // If FusionMode is fusion_gradual, it is the maximum allowable yaw rotational shift the vision camera can update in radians per second.
);

////////////////////////////////////////////////////////////
/**
 *    Pure Pursuit Default Parameters
 */
////////////////////////////////////////////////////////////

PursuitParams pursuit_parameters(
    to_meters(5),    // Radius of the turns, in meters
    to_meters(5),    // Pure Pursuit look ahead distance, in meters
    8.0,             // Pure pursuit forward max motor voltage (0.0, 12.0]
    12.0,            // Pure pursuit turning max motor voltage (0.0, 12.0]
    50.0,            // The maximum voltage change per second, as a slew rate (only applies speeding up)
    to_meters(1.25), // Settle Distance. Exits when within this distance of target, in meters
    to_rad(1),       // Settle Rotation. Exits when within this rotation of target, in radians
    0.3,             // Minimum time to be considered settled, in seconds
    0,               // Time after which to give up and move on, in seconds (set to 0 to disable)
    14,              // Turning (kP) Proportional Tuning
    0.1,             // Turning (kI) Integral Tuning
    20,              // Turning (kD) Derivative Tuning
    to_rad(15),      // The rotation distance (error), in radians, to activate turning_ki
    55,              // Forward (kP) Proportional Tuning
    0.01,            // Forward (kI) Integral Tuning
    250,             // Forward (kD) Derivative Tuning
    to_meters(2),    // The forward distance (error), in meters, to activate forward_ki
    100              // The number of points when generating the path. More points mean higher detail of the path, but at a higher computational cost
);

////////////////////////////////////////////////////////////
/**
 *    Robot Drivetrain and Manager
 */
////////////////////////////////////////////////////////////
WhoopDrivetrain robot_drivetrain(
    &pursuit_parameters,  // The default pure pursuit parameters for operating the robot in autonomous
    &odom_fusion,         // Odometry fusion module
    PoseUnits::in_deg_cw, // Set default pose units if not defined. "m_deg_cw" means "meters, degrees, clockwise-positive yaw", "in_deg_ccw" means "inches, degrees, counter-clockwise-positive yaw", and so forth.
    &controller1,         // Pointer to the controller
    &left_motors,         // Pointer to the left motor group (optionally can be a list of motors as well)
    &right_motors         // Pointer to the right motor group (optionally can be a list of motors as well)
);

ComputeManager manager({&buffer_system, &jetson_commander, &robot_drivetrain, &controller1});

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{
    pros::lcd::initialize();
    pros::lcd::set_text(1, "Hello PROS User!");
    controller1.notify("Initializing");
    manager.start();
    jetson_commander.initialize(); // If you don't have Tesseract, omit this line
    robot_drivetrain.calibrate();

}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled()
{
    robot_drivetrain.set_state(drivetrainState::mode_disabled);
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous()
{
    robot_drivetrain.set_state(drivetrainState::mode_autonomous);
    pros::delay(5000);

    robot_drivetrain.set_pose_units(PoseUnits::in_deg_cw);
    robot_drivetrain.set_pose(0, 0, 0);

    // robot_drivetrain.turn_to_position(15, 15);
    robot_drivetrain.drive_forward(15);

    robot_drivetrain.turn_to(90);

    robot_drivetrain.drive_forward(-15);

    robot_drivetrain.drive_forward(15);

    robot_drivetrain.turn_to(0);

    robot_drivetrain.drive_forward(-15);

    // robot_drivetrain.drive_to_point(15, 15);
    // robot_drivetrain.reverse_to_point(0,0);
    robot_drivetrain.drive_through_path({{15, 15, 0}, {0, 0, 90}}, 7);
    robot_drivetrain.reverse_through_path({{15, 15, 180}, {0, 0, 180}}, 7);
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol()
{
    autonomous();

    robot_drivetrain.set_state(drivetrainState::mode_usercontrol);

    while (true)
    {
        pros::delay(20); // Run for 20 ms then update
    }
}