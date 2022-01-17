/**
 * @example joint_position_control.cpp
 * Run joint position control to hold or sine-sweep all joints.
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
        // Initialize target vectors to hold position
        auto targetPosition = g_initPosition;
        std::vector<double> targetVelocity(k_robotDofs, 0);
        std::vector<double> targetAcceleration(k_robotDofs, 0);

        // Set target vectors based on motion type
        if (motionType == "hold") {
            // Do nothing, target vectors are initialized to hold position
        } else if (motionType == "sine-sweep") {
            for (size_t i = 0; i < k_robotDofs; ++i) {
                targetPosition[i] = g_initPosition[i]
                                    + k_sineAmp[i]
                                          * sin(2 * M_PI * k_sineFreq[i]
                                                * g_loopCounter * k_loopPeiord);
                std::fill(targetVelocity.begin(), targetVelocity.end(), 0.5);
                std::fill(
                    targetAcceleration.begin(), targetAcceleration.end(), 0);
            }
        } else {
            log.error("Unknown motion type");
            log.info("Accepted motion types: hold, sine-sweep");
            exit(1);
        }

        // Send target joint position to RDK server
        robot->streamJointPosition(
            targetPosition, targetVelocity, targetAcceleration);
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
    robot->setMode(flexiv::MODE_JOINT_POSITION);

    // Wait for the mode to be switched
    do {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } while (robot->getMode() != flexiv::MODE_JOINT_POSITION);

    // High-priority Realtime Periodic Task @ 1kHz
    //=============================================================================
    // This is a blocking method, so all other user-defined background threads
    // should be spawned before this
    robot->start(std::bind(periodicTask, robotStates, robot, motionType, log));

    return 0;
}
