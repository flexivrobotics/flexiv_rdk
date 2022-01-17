/**
 * @example joint_impedance_control.cpp
 * Run joint impedance control using RDK's joint torque streaming API.
 * @copyright Copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <Robot.hpp>
#include <Log.hpp>

#include <iostream>
#include <string>
#include <cmath>
#include <thread>

namespace {
/** Robot DOF */
const int k_robotDofs = 7;

/** RT loop period [sec] */
const double k_loopPeiord = 0.001;

/** Joint impedance control gains */
const std::vector<double> k_impedanceKp
    = {3000.0, 3000.0, 800.0, 800.0, 200.0, 200.0, 200.0};
const std::vector<double> k_impedanceKd
    = {80.0, 80.0, 40.0, 40.0, 8.0, 8.0, 8.0};

/** Sine-sweep trajectory amplitude and frequency */
const std::vector<double> k_sineAmp
    = {0.035, 0.035, 0.035, 0.035, 0.035, 0.035, 0.035};
const std::vector<double> k_sineFreq = {0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3};

/** Initial position of robot joints */
std::vector<double> g_initPosition;

/** Flag whether initial joint position is set */
bool g_isInitPositionSet = false;

/** Loop counter */
unsigned int g_loopCounter = 0;
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
void periodicTask(std::shared_ptr<flexiv::RobotStates> robotStates,
    std::shared_ptr<flexiv::Robot> robot, std::string motionType,
    flexiv::Log& log)
{
    // Read robot states
    robot->getRobotStates(robotStates.get());

    // Set initial joint position
    if (!g_isInitPositionSet) {
        // Check vector size before saving
        if (robotStates->m_q.size() == k_robotDofs) {
            g_initPosition = robotStates->m_q;
            g_isInitPositionSet = true;
            log.info("Initial joint position set to:");
            printVector(g_initPosition);
        }
    }
    // Run control only after init position is set
    else {
        // Initialize target position to hold position
        auto targetPosition = g_initPosition;

        // Set target position based on motion type
        if (motionType == "hold") {
            // Do nothing, target position is initialized to hold position
        } else if (motionType == "sine-sweep") {
            for (size_t i = 0; i < k_robotDofs; ++i) {
                targetPosition[i] = g_initPosition[i]
                                    + k_sineAmp[i]
                                          * sin(2 * M_PI * k_sineFreq[i]
                                                * g_loopCounter * k_loopPeiord);
            }
        } else {
            log.error("Unknown motion type");
            log.info("Accepted motion types: hold, sine-sweep");
            exit(1);
        }

        std::vector<double> torqueDesired(k_robotDofs);
        // Impedance control on all joints
        for (size_t i = 0; i < k_robotDofs; ++i) {
            torqueDesired[i]
                = k_impedanceKp[i] * (targetPosition[i] - robotStates->m_q[i])
                  - k_impedanceKd[i] * robotStates->m_dtheta[i];
        }

        // Send target joint torque to RDK server
        robot->streamJointTorque(torqueDesired, true);
    }

    g_loopCounter++;
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
    std::string motionType = argv[3];

    // RDK Initialization
    //=============================================================================
    // Instantiate robot interface
    auto robot = std::make_shared<flexiv::Robot>();

    // Create data struct for storing robot states
    auto robotStates = std::make_shared<flexiv::RobotStates>();

    // Initialize robot interface and connect to the robot server
    robot->init(robotIP, localIP);

    // Wait for the connection to be established
    do {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } while (!robot->isConnected());

    // Enable the robot, make sure the E-stop is released before enabling
    if (robot->enable()) {
        log.info("Enabling robot ...");
    }

    // Wait for the robot to become operational
    do {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } while (!robot->isOperational());
    log.info("Robot is now operational");

    // Set mode after robot is operational
    robot->setMode(flexiv::MODE_JOINT_TORQUE);

    // Wait for the mode to be switched
    do {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } while (robot->getMode() != flexiv::MODE_JOINT_TORQUE);

    // High-priority Realtime Periodic Task @ 1kHz
    //=============================================================================
    // This is a blocking method, so all other user-defined background threads
    // should be spawned before this
    robot->start(std::bind(periodicTask, robotStates, robot, motionType, log));

    return 0;
}
