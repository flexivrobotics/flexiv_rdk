/**
 * @example intermediate5_realtime_cartesian_motion_force_control.cpp
 * This tutorial runs real-time Cartesian-space unified motion-force control. The Z axis of the
 * chosen reference frame will be activated for explicit force control, while the rest axes in the
 * same reference frame will stay motion controlled.
 * @copyright Copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <flexiv/Robot.hpp>
#include <flexiv/Exception.hpp>
#include <flexiv/Log.hpp>
#include <flexiv/Scheduler.hpp>
#include <flexiv/Utility.hpp>

#include <iostream>
#include <cmath>
#include <thread>
#include <atomic>

namespace {
/** RT loop period [sec] */
constexpr double k_loopPeriod = 0.001;

/** TCP sine-sweep amplitude [m] */
constexpr double k_swingAmp = 0.1;

/** TCP sine-sweep frequency [Hz] */
constexpr double k_swingFreq = 0.3;

/** Pressing force to apply during the unified motion-force control [N] */
constexpr double k_pressingForce = 5.0;

/** Cartesian linear velocity used to search for contact [m/s] */
constexpr double k_searchVelocity = 0.02;

/** Maximum distance to travel when searching for contact [m] */
constexpr double k_searchDistance = 1.0;

/** Maximum contact wrench during contact search for soft contact */
const std::vector<double> k_maxWrenchForContactSearch = {10.0, 10.0, 10.0, 3.0, 3.0, 3.0};

/** Atomic signal to stop scheduler tasks */
std::atomic<bool> g_schedStop = {false};
}

/** @brief Print tutorial description */
void printDescription()
{
    std::cout << "This tutorial runs real-time Cartesian-space unified motion-force control. The Z "
                 "axis of the chosen reference frame will be activated for explicit force control, "
                 "while the rest axes in the same reference frame will stay motion controlled."
              << std::endl
              << std::endl;
}

/** @brief Print program usage help */
void printHelp()
{
    // clang-format off
    std::cout << "Required arguments: [robot IP] [local IP]" << std::endl;
    std::cout << "    robot IP: address of the robot server" << std::endl;
    std::cout << "    local IP: address of this PC" << std::endl;
    std::cout << "Optional arguments: [--TCP] [--polish]" << std::endl;
    std::cout << "    --TCP: use TCP frame as reference frame for force control, otherwise use base frame" << std::endl;
    std::cout << "    --polish: run a simple polish motion along XY plane in base frame, otherwise hold robot motion in non-force-control axes"
              << std::endl
              << std::endl;
    // clang-format on
}

/** Callback function for realtime periodic task */
void periodicTask(flexiv::Robot& robot, flexiv::Log& log, const std::vector<double>& initPose,
    const std::string frameStr, bool enablePolish)
{
    // Local periodic loop counter
    static uint64_t loopCounter = 0;

    try {
        // Monitor fault on robot server
        if (robot.isFault()) {
            throw flexiv::ServerException(
                "periodicTask: Fault occurred on robot server, exiting ...");
        }

        // Initialize target pose to initial pose
        std::vector<double> targetPose = initPose;

        // Set Fz according to reference frame to achieve a "pressing down" behavior
        double Fz = 0.0;
        if (frameStr == "BASE") {
            Fz = -k_pressingForce;
        } else if (frameStr == "TCP") {
            Fz = k_pressingForce;
        }
        std::vector<double> targetWrench = {0.0, 0.0, Fz, 0.0, 0.0, 0.0};

        // Apply constant force along Z axis of chosen reference frame, and do a simple polish
        // motion along XY plane in robot base frame
        if (enablePolish) {
            // Create motion command to sine-sweep along Y direction
            targetPose[1] = initPose[1]
                            + k_swingAmp * sin(2 * M_PI * k_swingFreq * loopCounter * k_loopPeriod);
            // Command target pose and target wrench
            robot.streamCartesianMotionForce(targetPose, targetWrench);
        }
        // Apply constant force along Z axis of chosen reference frame, and hold motions in all
        // other axes
        else {
            // Command initial pose and target wrench
            robot.streamCartesianMotionForce(initPose, targetWrench);
        }

        // Increment loop counter
        loopCounter++;

    } catch (const flexiv::Exception& e) {
        log.error(e.what());
        g_schedStop = true;
    }
}

int main(int argc, char* argv[])
{
    // Program Setup
    // =============================================================================================
    // Logger for printing message with timestamp and coloring
    flexiv::Log log;

    // Parse parameters
    if (argc < 3 || flexiv::utility::programArgsExistAny(argc, argv, {"-h", "--help"})) {
        printHelp();
        return 1;
    }
    // IP of the robot server
    std::string robotIP = argv[1];
    // IP of the workstation PC running this program
    std::string localIP = argv[2];

    // Print description
    log.info("Tutorial description:");
    printDescription();

    // The reference frame to use for force control, see Robot::setForceControlFrame() for
    // more details
    std::string frameStr = "BASE";
    if (flexiv::utility::programArgsExist(argc, argv, "--TCP")) {
        log.info("Reference frame used for force control: robot TCP frame");
        frameStr = "TCP";
    } else {
        log.info("Reference frame used for force control: robot base frame");
    }

    // Whether to enable polish motion
    bool enablePolish = false;
    if (flexiv::utility::programArgsExist(argc, argv, "--polish")) {
        log.info("Robot will run a polish motion along XY plane in robot base frame");
        enablePolish = true;
    } else {
        log.info("Robot will hold its motion in all non-force-controlled axes");
    }

    try {
        // RDK Initialization
        // =========================================================================================
        // Instantiate robot interface
        flexiv::Robot robot(robotIP, localIP);

        // Create data struct for storing robot states
        flexiv::RobotStates robotStates;

        // Clear fault on robot server if any
        if (robot.isFault()) {
            log.warn("Fault occurred on robot server, trying to clear ...");
            // Try to clear the fault
            robot.clearFault();
            std::this_thread::sleep_for(std::chrono::seconds(2));
            // Check again
            if (robot.isFault()) {
                log.error("Fault cannot be cleared, exiting ...");
                return 1;
            }
            log.info("Fault on robot server is cleared");
        }

        // Enable the robot, make sure the E-stop is released before enabling
        log.info("Enabling robot ...");
        robot.enable();

        // Wait for the robot to become operational
        while (!robot.isOperational()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        log.info("Robot is now operational");

        // Move robot to home pose
        log.info("Moving to home pose");
        robot.setMode(flexiv::Mode::NRT_PRIMITIVE_EXECUTION);
        robot.executePrimitive("Home()");

        // Wait for the primitive to finish
        while (robot.isBusy()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        // Zero Force-torque Sensor
        // =========================================================================================
        // IMPORTANT: must zero force/torque sensor offset for accurate force/torque measurement
        robot.executePrimitive("ZeroFTSensor()");

        // WARNING: during the process, the robot must not contact anything, otherwise the result
        // will be inaccurate and affect following operations
        log.warn("Zeroing force/torque sensors, make sure nothing is in contact with the robot");

        // Wait for primitive completion
        while (robot.isBusy()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        log.info("Sensor zeroing complete");

        // Search for Contact
        // =========================================================================================
        // NOTE: there are several ways to do contact search, such as using primitives, or real-time
        // and non-real-time direct motion controls, etc. Here we use non-real-time direct Cartesian
        // control for example.
        log.info("Searching for contact ...");

        // Set initial pose to current TCP pose
        robot.getRobotStates(robotStates);
        std::vector<double> initPose = robotStates.tcpPose;
        log.info("Initial TCP pose set to [position 3x1, rotation (quaternion) 4x1]: "
                 + flexiv::utility::vec2Str(initPose));

        // Use non-real-time mode to make the robot go to a set point with its own motion generator
        robot.setMode(flexiv::Mode::NRT_CARTESIAN_MOTION_FORCE);

        // Search for contact with max contact wrench set to a small value for making soft contact
        robot.setMaxContactWrench(k_maxWrenchForContactSearch);

        // Set target point along -Z direction and expect contact to happen during the travel
        auto targetPose = initPose;
        targetPose[2] -= k_searchDistance;

        // Send target point to robot to start searching for contact and limit the velocity. Keep
        // target wrench 0 at this stage since we are not doing force control yet
        robot.sendCartesianMotionForce(targetPose, std::vector<double>(6), k_searchVelocity);

        // Use a while loop to poll robot states and check if a contact is made
        bool isContacted = false;
        while (!isContacted) {
            // Get the latest robot states
            robot.getRobotStates(robotStates);

            // Compute norm of sensed external force applied on robot TCP
            Eigen::Vector3d extForce = {robotStates.extWrenchInBase[0],
                robotStates.extWrenchInBase[1], robotStates.extWrenchInBase[2]};

            // Contact is considered to be made if sensed TCP force exceeds the threshold
            if (extForce.norm() > k_pressingForce) {
                isContacted = true;
                log.info("Contact detected at robot TCP");
            }
            // Check at 1ms interval
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        // Configure Force Control
        // =========================================================================================
        // The force control configurations can only be updated when the robot is in IDLE mode
        robot.stop();

        // Set force control reference frame based on program argument. See function doc for more
        // details
        robot.setForceControlFrame(frameStr);

        // Set which Cartesian axis(s) to activate for force control. See function doc for more
        // details. Here we only active Z axis
        robot.setForceControlAxis(std::vector<bool> {false, false, true, false, false, false});

        // Uncomment the following line to enable passive force control, otherwise active force
        // control is used by default. See function doc for more details
        /* robot.setPassiveForceControl(true); */

        // NOTE: motion control always uses robot base frame, while force control can use
        // either base or TCP frame as reference frame

        // Start Unified Motion Force Control
        // =========================================================================================
        // Switch to real-time mode for continuous motion force control
        robot.setMode(flexiv::Mode::RT_CARTESIAN_MOTION_FORCE);

        // Disable max contact wrench regulation. Need to do this AFTER the force control in Z axis
        // is activated (i.e. motion control disabled in Z axis) and the motion force control mode
        // is entered, this way the contact force along Z axis is explicitly regulated and will not
        // spike after the max contact wrench regulation for motion control is disabled
        robot.resetMaxContactWrench();

        // Update initial pose to current TCP pose
        robot.getRobotStates(robotStates);
        initPose = robotStates.tcpPose;

        // Create real-time scheduler to run periodic tasks
        flexiv::Scheduler scheduler;
        // Add periodic task with 1ms interval and highest applicable priority
        scheduler.addTask(std::bind(periodicTask, std::ref(robot), std::ref(log),
                              std::ref(initPose), std::ref(frameStr), enablePolish),
            "HP periodic", 1, scheduler.maxPriority());
        // Start all added tasks
        scheduler.start();

        // Block and wait for signal to stop scheduler tasks
        while (!g_schedStop) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        // Received signal to stop scheduler tasks
        scheduler.stop();

        // Wait a bit for any last-second robot log message to arrive and get printed
        std::this_thread::sleep_for(std::chrono::seconds(2));

    } catch (const flexiv::Exception& e) {
        log.error(e.what());
        return 1;
    }

    return 0;
}
