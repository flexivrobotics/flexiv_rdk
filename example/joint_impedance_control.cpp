/**
 * @example Run joint impedance control using RDK's joint torque streaming API
 * @copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <config.h>
#include <Robot.hpp>

#include <iostream>
#include <string>
#include <cmath>
#include <thread>

namespace {
// robot DOF
const int k_robotDofs = 7;

// RT loop period [sec]
const double k_loopPeiord = 0.001;

// joint impedance control gains
const std::vector<double> k_impedanceKp
    = {3000.0, 3000.0, 800.0, 800.0, 200.0, 200.0, 200.0};
const std::vector<double> k_impedanceKd
    = {80.0, 80.0, 40.0, 40.0, 8.0, 8.0, 8.0};

// sine-sweep trajectory amplitude and frequency
const std::vector<double> k_sineAmp
    = {0.035, 0.035, 0.035, 0.035, 0.035, 0.035, 0.035};
const std::vector<double> k_sineFreq = {0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3};

// initial position of robot joints
std::vector<double> g_initPosition;

// flag whether initial joint position is set
bool g_isInitPositionSet = false;

// loop counter
unsigned int g_loopCounter = 0;
}

void printVector(const std::vector<double>& vec)
{
    for (auto i : vec) {
        std::cout << i << "  ";
    }
    std::cout << std::endl;
}

// callback function for realtime periodic task
void periodicTask(std::shared_ptr<flexiv::RobotStates> robotStates,
    std::shared_ptr<flexiv::Robot> robot, std::string motionType)
{
    // read robot states
    robot->getRobotStates(robotStates.get());

    // set initial joint position
    if (!g_isInitPositionSet) {
        // check vector size before saving
        if (robotStates->m_linkPosition.size() == k_robotDofs) {
            g_initPosition = robotStates->m_linkPosition;
            g_isInitPositionSet = true;
            std::cout << "Initial joint position set to:\n";
            printVector(g_initPosition);
        }
    }
    // run control only after init position is set
    else {
        // initialize target position to hold position
        auto targetPosition = g_initPosition;

        // set target position based on motion type
        if (motionType == "hold") {
            // do nothing, target position is initialized to hold position
        } else if (motionType == "sine-sweep") {
            for (size_t i = 0; i < k_robotDofs; ++i) {
                targetPosition[i] = g_initPosition[i]
                                    + k_sineAmp[i]
                                          * sin(2 * M_PI * k_sineFreq[i]
                                                * g_loopCounter * k_loopPeiord);
            }
        } else {
            std::cout << "Unknown motion type" << std::endl;
            std::cout << "Accepted motion types: hold, sine-sweep" << std::endl;
            exit(1);
        }

        std::vector<double> targetTorque(k_robotDofs);
        // impedance control on all joints
        for (size_t i = 0; i < k_robotDofs; ++i) {
            targetTorque[i]
                = k_impedanceKp[i]
                      * (targetPosition[i] - robotStates->m_linkPosition[i])
                  - k_impedanceKd[i] * robotStates->m_linkVelocity[i];
        }

        // send target joint torque to RDK server
        robot->streamJointTorque(targetTorque, true);
    }

    g_loopCounter++;
}

int main(int argc, char* argv[])
{
    // Parse Parameters
    //=============================================================================
    if (argc != 2) {
        std::cerr << "Invalid program arguments. Usage: <motion_type>"
                  << std::endl;
        std::cout << "Accepted motion types: hold, sine-sweep" << std::endl;
        return 0;
    }

    // type of motion specified by user
    std::string motionType = argv[1];

    // print loop frequency
    std::cout << "Example client running at 1000 Hz" << std::endl;

    // RDK Initialization
    //=============================================================================
    // RDK robot interface
    auto robot = std::make_shared<flexiv::Robot>();

    // robot states data from RDK server
    auto robotStates = std::make_shared<flexiv::RobotStates>();

    // initialize connection
    robot->init(ROBOT_IP, LOCAL_IP);

    // wait for the connection to be established
    do {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } while (!robot->isConnected());

    // enable the robot, make sure the E-stop is released before enabling
    if (robot->enable()) {
        std::cout << "Enabling robot ..." << std::endl;
    }

    // wait for the robot to become operational
    do {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } while (!robot->isOperational());
    std::cout << "Robot is now operational" << std::endl;

    // set mode after robot is operational
    robot->setMode(flexiv::MODE_JOINT_TORQUE);

    // wait for the mode to be switched
    do {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } while (robot->getMode() != flexiv::MODE_JOINT_TORQUE);

    // High-priority Realtime Periodic Task @ 1kHz
    //=============================================================================
    // this is a blocking method, so all other user-defined background threads
    // should be spawned before this
    robot->start(std::bind(periodicTask, robotStates, robot, motionType));

    return 0;
}
