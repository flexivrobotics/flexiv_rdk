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

/** Helper function to print a std::vector */
void printVector(const std::vector<double>& vec)
{
    for (auto i : vec) {
        std::cout << i << "  ";
    }
    std::cout << std::endl;
}

/** Callback function for realtime periodic task */
void periodicTask(flexiv::Robot* robot, flexiv::RobotStates* robotStates,
    flexiv::Scheduler* scheduler, flexiv::Log* log)
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
            if (robotStates->m_q.size() == k_robotDofs) {
                initPosition = robotStates->m_q;
                isInitPositionSet = true;
                log->info("Initial joint position set to:");
                printVector(initPosition);
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

int main(int argc, char* argv[])
{
    // Log object for printing message with timestamp and coloring
    flexiv::Log log;

    // Parse Parameters
    //=============================================================================
    // Check if program has 3 arguments
    if (argc != 4) {
        log.error(
            "Invalid program arguments. Usage: <robot_ip> <local_ip> "
            "<motion_type>");
        log.info("Accepted motion types: hold, sine-sweep");
        return 0;
    }
    // IP of the robot server
    std::string robotIP = argv[1];

    // IP of the workstation PC running this program
    std::string localIP = argv[2];

    // Type of motion specified by user
    motionType = argv[3];

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
                return 0;
            }
            log.info("Fault on robot server is cleared");
        }

        // Enable the robot, make sure the E-stop is released before enabling
        log.info("Enabling robot ...");
        robot.enable();

        // Wait for the robot to become operational
        while (!robot.isOperational()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
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
            std::bind(periodicTask, &robot, &robotStates, &scheduler, &log),
            "HP periodic", 1, 45);
        // Start all added tasks, this is by default a blocking method
        scheduler.start();

    } catch (const flexiv::Exception& e) {
        log.error(e.what());
        return 0;
    }

    return 0;
}
