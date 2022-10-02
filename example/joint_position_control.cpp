/**
 * @example joint_position_control.cpp
 * Run joint position control to hold or sine-sweep all joints.
 * @copyright Copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <flexiv/Robot.hpp>
#include <flexiv/Exception.hpp>
#include <flexiv/Log.hpp>
#include <flexiv/Scheduler.hpp>
#include <flexiv/Utility.hpp>

#include <iostream>
#include <string>
#include <cmath>
#include <thread>

namespace {
/** Robot DOF */
const int k_robotDofs = 7;

/** RT loop period [sec] */
const double k_loopPeriod = 0.001;

/** Sine-sweep trajectory amplitude and frequency */
const std::vector<double> k_sineAmp
    = {0.035, 0.035, 0.035, 0.035, 0.035, 0.035, 0.035};
const std::vector<double> k_sineFreq = {0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3};

/** Type of motion specified by user */
std::string motionType = {};
}

/** Callback function for realtime periodic task */
void periodicTask(flexiv::Robot* robot, flexiv::Scheduler* scheduler,
    flexiv::Log* log, flexiv::RobotStates& robotStates)
{
    // Sine counter
    static unsigned int sineCounter = 0;

    // Whether initial joint position is set
    static bool isInitPositionSet = false;

    // Initial position of robot joints
    static std::vector<double> initPosition;

    try {
        // Monitor fault on robot server
        if (robot->isFault()) {
            throw flexiv::ServerException(
                "periodicTask: Fault occurred on robot server, exiting ...");
        }

        // Read robot states
        robot->getRobotStates(robotStates);

        // Set initial joint position
        if (!isInitPositionSet) {
            // Check vector size before saving
            if (robotStates.q.size() == k_robotDofs) {
                initPosition = robotStates.q;
                isInitPositionSet = true;
                log->info("Initial joint position set to: "
                          + flexiv::utility::vec2Str(initPosition));
            }
        }
        // Run control only after init position is set
        else {
            // Initialize target vectors to hold position
            auto targetPosition = initPosition;
            std::vector<double> targetVelocity(k_robotDofs, 0);
            std::vector<double> targetAcceleration(k_robotDofs, 0);

            // Set target vectors based on motion type
            if (motionType == "hold") {
                // Do nothing, target vectors are initialized to hold position
            } else if (motionType == "sine-sweep") {
                for (size_t i = 0; i < k_robotDofs; ++i) {
                    targetPosition[i]
                        = initPosition[i]
                          + k_sineAmp[i]
                                * sin(2 * M_PI * k_sineFreq[i] * sineCounter
                                      * k_loopPeriod);
                    std::fill(
                        targetVelocity.begin(), targetVelocity.end(), 0.5);
                    std::fill(targetAcceleration.begin(),
                        targetAcceleration.end(), 0);
                }
                sineCounter++;
            } else {
                log->error("Unknown motion type");
                log->info("Accepted motion types: hold, sine-sweep");
                exit(1);
            }

            // Send target joint position to RDK server
            robot->streamJointPosition(
                targetPosition, targetVelocity, targetAcceleration);
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
    std::cout << "    --hold: robot holds current joint positions, otherwise do a sine-sweep" << std::endl;
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
        log.info("Robot holding current pose");
        motionType = "hold";
    } else {
        log.info("Robot running joint sine-sweep");
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
        robot.setMode(flexiv::MODE_JOINT_POSITION);

        // Wait for the mode to be switched
        while (robot.getMode() != flexiv::MODE_JOINT_POSITION) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

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
