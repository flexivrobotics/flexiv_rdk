/**
 * @example cartesian_impedance_control.cpp
 * Run Cartesian impedance control to hold or sine-sweep the robot TCP.
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
/** Size of Cartesian pose vector [position 3x1 + rotation (quaternion) 4x1 ] */
const unsigned int k_cartPoseSize = 7;

/** RT loop period [sec] */
const double k_loopPeriod = 0.001;

/** TCP sine-sweep amplitude [m] */
const double k_swingAmp = 0.1;

/** TCP sine-sweep frequency [Hz] */
const double k_swingFreq = 0.3;

/** Type of motion specified by user */
std::string motionType = {};
}

/** Callback function for realtime periodic task */
void periodicTask(flexiv::Robot* robot, flexiv::Scheduler* scheduler,
    flexiv::Log* log, flexiv::RobotStates& robotStates)
{
    // Sine counter
    static unsigned int loopCounter = 0;

    // Flag whether initial Cartesian position is set
    static bool isInitPoseSet = false;

    // Initial Cartesian-space pose (position + rotation) of robot TCP
    static std::vector<double> initTcpPose;

    try {
        // Monitor fault on robot server
        if (robot->isFault()) {
            throw flexiv::ServerException(
                "periodicTask: Fault occurred on robot server, exiting ...");
        }

        // Read robot states
        robot->getRobotStates(robotStates);

        // Set initial TCP pose
        if (!isInitPoseSet) {
            // Check vector size before saving
            if (robotStates.tcpPose.size() == k_cartPoseSize) {
                initTcpPose = robotStates.tcpPose;
                isInitPoseSet = true;
                log->info(
                    "Initial Cartesion pose of robot TCP set to [position "
                    "3x1 + rotation (quaternion) 4x1]: "
                    + flexiv::utility::vec2Str(initTcpPose));
            }
        }
        // Run control only after initial pose is set
        else {
            // Set target position based on motion type
            if (motionType == "hold") {
                robot->streamTcpPose(initTcpPose);
            } else if (motionType == "sine-sweep") {
                auto targetTcpPose = initTcpPose;
                targetTcpPose[1] = initTcpPose[1]
                                   + k_swingAmp
                                         * sin(2 * M_PI * k_swingFreq
                                               * loopCounter * k_loopPeriod);
                robot->streamTcpPose(targetTcpPose);
            } else {
                log->error("Unknown motion type");
                log->info("Accepted motion types: hold, sine-sweep");
                exit(1);
            }

            // online change stiffness to softer at 5 seconds
            if (loopCounter % 10000 == 5000) {
                std::vector<double> newKp = {2000, 2000, 2000, 200, 200, 200};
                robot->setCartesianStiffness(newKp);
                log->info("Cartesian stiffness set to: "
                          + flexiv::utility::vec2Str(newKp));
            }

            // online reset stiffness to original at 9 seconds
            if (loopCounter % 10000 == 9000) {
                robot->setCartesianStiffness();
                log->info("Cartesian stiffness is reset");
            }

            loopCounter++;
        }

    } catch (const flexiv::Exception& e) {
        log->error(e.what());
        scheduler->stop();
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

        // Set mode after robot is operational
        robot.setMode(flexiv::MODE_CARTESIAN_IMPEDANCE);

        // Wait for the mode to be switched
        while (robot.getMode() != flexiv::MODE_CARTESIAN_IMPEDANCE) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        // Choose the index of tool being used. No need to call this method if
        // the mounted tool on the robot has only one TCP, it'll be used by
        // default
        robot.switchTcp(0);

        // Periodic Tasks
        //=============================================================================
        flexiv::Scheduler scheduler;
        // Add periodic task with 1ms interval and highest applicable priority
        scheduler.addTask(
            std::bind(periodicTask, &robot, &scheduler, &log, robotStates),
            "HP periodic", 1, 45);
        // Start all added tasks, this is by default a blocking method
        scheduler.start();

    } catch (const flexiv::Exception& e) {
        log.error(e.what());
        return 1;
    }

    return 0;
}
