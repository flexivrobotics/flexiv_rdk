/**
 * @example intermediate4_realtime_cartesian_pure_motion_control.cpp
 * This tutorial runs real-time Cartesian-space pure motion control to hold or sine-sweep the robot
 * TCP. A simple collision detection is also included.
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
/** RT loop frequency [Hz] */
constexpr size_t k_loopFreq = 1000;

/** RT loop period [sec] */
constexpr double k_loopPeriod = 0.001;

/** TCP sine-sweep amplitude [m] */
constexpr double k_swingAmp = 0.1;

/** TCP sine-sweep frequency [Hz] */
constexpr double k_swingFreq = 0.3;

/** External TCP force threshold for collision detection, value is only for demo purpose [N] */
constexpr double k_extForceThreshold = 10.0;

/** External joint torque threshold for collision detection, value is only for demo purpose [Nm] */
constexpr double k_extTorqueThreshold = 5.0;

/** Atomic signal to stop scheduler tasks */
std::atomic<bool> g_schedStop = {false};
}

/** @brief Print tutorial description */
void printDescription()
{
    std::cout << "This tutorial runs real-time Cartesian-space pure motion control to hold or "
                 "sine-sweep the robot TCP. A simple collision detection is also included."
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
    std::cout << "Optional arguments: [--hold] [--collision]" << std::endl;
    std::cout << "    --hold: robot holds current TCP pose, otherwise do a sine-sweep" << std::endl;
    std::cout << "    --collision: enable collision detection, robot will stop upon collision" << std::endl;
    std::cout << std::endl;
    // clang-format on
}

/** @brief Callback function for realtime periodic task */
void periodicTask(flexiv::Robot& robot, flexiv::Log& log, flexiv::RobotStates& robotStates,
    const std::vector<double>& initPose, bool enableHold, bool enableCollision)
{
    // Local periodic loop counter
    static uint64_t loopCounter = 0;

    try {
        // Monitor fault on robot server
        if (robot.isFault()) {
            throw flexiv::ServerException(
                "periodicTask: Fault occurred on robot server, exiting ...");
        }

        // Read robot states
        robot.getRobotStates(robotStates);

        // Initialize target pose to initial pose
        auto targetTcpPose = initPose;

        // Sine-sweep TCP along Y axis
        if (!enableHold) {
            targetTcpPose[1]
                = initPose[1]
                  + k_swingAmp * sin(2 * M_PI * k_swingFreq * loopCounter * k_loopPeriod);
        }
        // Otherwise robot TCP will hold at initial pose

        // Send command. Calling this method with only target pose input results in pure motion
        // control
        robot.streamCartesianMotionForce(targetTcpPose);

        // Do the following operations in sequence for every 20 seconds
        switch (loopCounter % (20 * k_loopFreq)) {
            // Online change preferred joint positions at 3 seconds
            case (3 * k_loopFreq): {
                std::vector<double> preferredJntPos
                    = {0.938, -1.108, -1.254, 1.464, 1.073, 0.278, -0.658};
                robot.setNullSpacePosture(preferredJntPos);
                log.info("Preferred joint positions set to: "
                         + flexiv::utility::vec2Str(preferredJntPos));
            } break;
            // Online change stiffness to half of nominal at 6 seconds
            case (6 * k_loopFreq): {
                std::vector<double> newK = robot.info().nominalK;
                for (auto& v : newK) {
                    v *= 0.5;
                }
                robot.setCartesianStiffness(newK);
                log.info("Cartesian stiffness set to: " + flexiv::utility::vec2Str(newK));
            } break;
            // Online change to another preferred joint positions at 9 seconds
            case (9 * k_loopFreq): {
                std::vector<double> preferredJntPos
                    = {-0.938, -1.108, 1.254, 1.464, -1.073, 0.278, 0.658};
                robot.setNullSpacePosture(preferredJntPos);
                log.info("Preferred joint positions set to: "
                         + flexiv::utility::vec2Str(preferredJntPos));
            } break;
            // Online reset stiffness to nominal at 12 seconds
            case (12 * k_loopFreq): {
                robot.resetCartesianStiffness();
                log.info("Cartesian stiffness is reset");
            } break;
            // Online reset preferred joint positions to nominal at 14 seconds
            case (14 * k_loopFreq): {
                robot.resetNullSpacePosture();
                log.info("Preferred joint positions are reset");
            } break;
            // Online enable max contact wrench regulation at 16 seconds
            case (16 * k_loopFreq): {
                std::vector<double> maxWrench = {10.0, 10.0, 10.0, 2.0, 2.0, 2.0};
                robot.setMaxContactWrench(maxWrench);
                log.info("Max contact wrench set to: " + flexiv::utility::vec2Str(maxWrench));
            } break;
            // Disable max contact wrench regulation at 19 seconds
            case (19 * k_loopFreq): {
                robot.resetMaxContactWrench();
                log.info("Max contact wrench is reset");
            } break;
            default:
                break;
        }

        // Simple collision detection: stop robot if collision is detected from either end-effector
        // or robot body
        if (enableCollision) {
            bool collisionDetected = false;
            Eigen::Vector3d extForce = {robotStates.extWrenchInBase[0],
                robotStates.extWrenchInBase[1], robotStates.extWrenchInBase[2]};
            if (extForce.norm() > k_extForceThreshold) {
                collisionDetected = true;
            }
            for (const auto& v : robotStates.tauExt) {
                if (fabs(v) > k_extTorqueThreshold) {
                    collisionDetected = true;
                }
            }
            if (collisionDetected) {
                robot.stop();
                log.warn("Collision detected, stopping robot and exit program ...");
                g_schedStop = true;
            }
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

    // Type of motion specified by user
    bool enableHold = false;
    if (flexiv::utility::programArgsExist(argc, argv, "--hold")) {
        log.info("Robot holding current TCP pose");
        enableHold = true;
    } else {
        log.info("Robot running TCP sine-sweep");
    }

    // Whether to enable collision detection
    bool enableCollision = false;
    if (flexiv::utility::programArgsExist(argc, argv, "--collision")) {
        log.info("Collision detection enabled");
        enableCollision = true;
    } else {
        log.info("Collision detection disabled");
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

        // Configure Motion Control
        // =========================================================================================
        // The Cartesian motion force modes do pure motion control out of the box, thus nothing
        // needs to be explicitly configured

        // NOTE: motion control always uses robot base frame, while force control can use
        // either base or TCP frame as reference frame

        // Start Pure Motion Control
        // =========================================================================================
        // Switch to real-time mode for continuous motion control
        robot.setMode(flexiv::Mode::RT_CARTESIAN_MOTION_FORCE);

        // Set initial pose to current TCP pose
        robot.getRobotStates(robotStates);
        auto initPose = robotStates.tcpPose;
        log.info("Initial TCP pose set to [position 3x1, rotation (quaternion) 4x1]: "
                 + flexiv::utility::vec2Str(initPose));

        // Create real-time scheduler to run periodic tasks
        flexiv::Scheduler scheduler;
        // Add periodic task with 1ms interval and highest applicable priority
        scheduler.addTask(
            std::bind(periodicTask, std::ref(robot), std::ref(log), std::ref(robotStates),
                std::ref(initPose), enableHold, enableCollision),
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
