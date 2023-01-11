/**
 * @example RT_cartesian_impedance_control.cpp
 * Real-time Cartesian impedance control to hold or sine-sweep the robot TCP.
 * A simple collision detection is also included.
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

namespace {
/** RT loop frequency [Hz] */
constexpr size_t k_loopFreq = 1000;

/** RT loop period [sec] */
constexpr double k_loopPeriod = 0.001;

/** TCP sine-sweep amplitude [m] */
constexpr double k_swingAmp = 0.1;

/** TCP sine-sweep frequency [Hz] */
constexpr double k_swingFreq = 0.3;

/** External TCP force threshold for collision detection [N] */
constexpr double k_extForceThreshold = 10.0;

/** External joint torque threshold for collision detection [Nm] */
constexpr double k_extTorqueThreshold = 5.0;
}

/** Callback function for realtime periodic task */
void periodicTask(flexiv::Robot& robot, flexiv::Scheduler& scheduler,
    flexiv::Log& log, flexiv::RobotStates& robotStates,
    const std::string& motionType, const std::vector<double>& initTcpPose)
{
    // Local periodic loop counter
    static size_t loopCounter = 0;

    try {
        // Monitor fault on robot server
        if (robot.isFault()) {
            throw flexiv::ServerException(
                "periodicTask: Fault occurred on robot server, exiting ...");
        }

        // Read robot states
        robot.getRobotStates(robotStates);

        // Set target pose based on motion type
        if (motionType == "hold") {
            robot.streamTcpPose(initTcpPose);
        } else if (motionType == "sine-sweep") {
            auto targetTcpPose = initTcpPose;
            targetTcpPose[1] = initTcpPose[1]
                               + k_swingAmp
                                     * sin(2 * M_PI * k_swingFreq * loopCounter
                                           * k_loopPeriod);
            robot.streamTcpPose(targetTcpPose);
        } else {
            throw flexiv::InputException(
                "periodicTask: unknown motion type. Accepted motion types: "
                "hold, sine-sweep");
        }

        // Do the following operations in sequence for every 20 seconds
        switch (loopCounter % (20 * k_loopFreq)) {
            // Online change preferred joint positions at 3 seconds
            case (3 * k_loopFreq): {
                std::vector<double> preferredJntPos
                    = {-0.938, -1.108, -1.254, 1.464, 1.073, 0.278, -0.658};
                robot.setNullSpacePosture(preferredJntPos);
                log.info("Preferred joint positions set to: "
                         + flexiv::utility::vec2Str(preferredJntPos));
            } break;
            // Online change stiffness to softer at 6 seconds
            case (6 * k_loopFreq): {
                std::vector<double> newK = {2000, 2000, 2000, 200, 200, 200};
                robot.setCartesianStiffness(newK);
                log.info("Cartesian stiffness set to: "
                         + flexiv::utility::vec2Str(newK));
            } break;
            // Online change to another preferred joint positions at 9 seconds
            case (9 * k_loopFreq): {
                std::vector<double> preferredJntPos
                    = {0.938, -1.108, 1.254, 1.464, -1.073, 0.278, 0.658};
                robot.setNullSpacePosture(preferredJntPos);
                log.info("Preferred joint positions set to: "
                         + flexiv::utility::vec2Str(preferredJntPos));
            } break;
            // Online reset stiffness to original at 12 seconds
            case (12 * k_loopFreq): {
                robot.setCartesianStiffness();
                log.info("Cartesian stiffness is reset");
            } break;
            // Online reset preferred joint positions at 15 seconds
            case (15 * k_loopFreq): {
                robot.setNullSpacePosture();
                log.info("Preferred joint positions are reset");
            } break;
            default:
                break;
        }

        // Simple collision detection: stop robot if collision is detected from
        // either end-effector or robot body
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
            scheduler.stop();
        }

        // Increment loop counter
        loopCounter++;

    } catch (const flexiv::Exception& e) {
        log.error(e.what());
        scheduler.stop();
    }
}

void printHelp()
{
    // clang-format off
    std::cout << "Required arguments: [robot IP] [local IP]" << std::endl;
    std::cout << "    robot IP: address of the robot server" << std::endl;
    std::cout << "    local IP: address of this PC" << std::endl;
    std::cout << "Optional arguments: [--hold]" << std::endl;
    std::cout << "    --hold: robot holds current TCP pose, otherwise do a sine-sweep" << std::endl;
    std::cout << std::endl;
    // clang-format on
}

int main(int argc, char* argv[])
{
    // Log object for printing message with timestamp and coloring
    flexiv::Log log;

    // Parse Parameters
    //=============================================================================
    if (argc < 3
        || flexiv::utility::programArgsExistAny(argc, argv, {"-h", "--help"})) {
        printHelp();
        return 1;
    }

    // IP of the robot server
    std::string robotIP = argv[1];

    // IP of the workstation PC running this program
    std::string localIP = argv[2];

    // Type of motion specified by user
    std::string motionType = "";
    if (flexiv::utility::programArgsExist(argc, argv, "--hold")) {
        log.info("Robot holding current TCP pose");
        motionType = "hold";
    } else {
        log.info("Robot running TCP sine-sweep");
        motionType = "sine-sweep";
    }

    try {
        // RDK Initialization
        //=============================================================================
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
        int secondsWaited = 0;
        while (!robot.isOperational()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            if (++secondsWaited == 10) {
                log.warn(
                    "Still waiting for robot to become operational, please "
                    "check that the robot 1) has no fault, 2) is booted "
                    "into Auto mode");
            }
        }
        log.info("Robot is now operational");

        // IMPORTANT: must calibrate force/torque sensor for accurate collision
        // detection
        robot.setMode(flexiv::MODE_PRIMITIVE_EXECUTION);
        while (robot.getMode() != flexiv::MODE_PRIMITIVE_EXECUTION) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        log.warn(
            "Calibrating force/torque sensors, please don't touch the robot");
        robot.executePrimitive("CaliForceSensor()");
        // Wait for primitive completion
        while (robot.isBusy()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        // Set mode after sensor calibration
        robot.setMode(flexiv::MODE_CARTESIAN_IMPEDANCE);
        while (robot.getMode() != flexiv::MODE_CARTESIAN_IMPEDANCE) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        // Set initial TCP pose
        robot.getRobotStates(robotStates);
        auto initTcpPose = robotStates.tcpPose;
        log.info(
            "Initial TCP pose set to [position 3x1, rotation (quaternion) "
            "4x1]: "
            + flexiv::utility::vec2Str(initTcpPose));

        // Periodic Tasks
        //=============================================================================
        flexiv::Scheduler scheduler;
        // Add periodic task with 1ms interval and highest applicable priority
        scheduler.addTask(
            std::bind(periodicTask, std::ref(robot), std::ref(scheduler),
                std::ref(log), std::ref(robotStates), std::ref(motionType),
                std::ref(initTcpPose)),
            "HP periodic", 1, scheduler.maxPriority());
        // Start all added tasks, this is by default a blocking method
        scheduler.start();

    } catch (const flexiv::Exception& e) {
        log.error(e.what());
        return 1;
    }

    return 0;
}
