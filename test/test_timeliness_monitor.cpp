/**
 * @test test_timeliness_monitor.cpp
 * A test to evaluate RDK's internal timeliness monitor on real-time modes. Bad
 * communication or insufficient real-time performance of the workstation PC
 * will cause the monitor's timeliness check to fail. A warning will be issued
 * first, then if the check has failed too many times, the RDK connection with
 * the server will be closed. During this test, the robot will hold its position
 * using joint torque streaming mode.
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
// robot DOF
const int k_robotDofs = 7;

// joint impedance control gains
const std::vector<double> k_impedanceKp
    = {3000.0, 3000.0, 800.0, 800.0, 200.0, 200.0, 200.0};
const std::vector<double> k_impedanceKd
    = {80.0, 80.0, 40.0, 40.0, 8.0, 8.0, 8.0};

// initial position of robot joints
std::vector<double> g_initPosition;

// flag whether initial joint position is set
bool g_isInitPositionSet = false;

// loop counter
unsigned int g_loopCounter = 0;
}

// callback function for realtime periodic task
void periodicTask(std::shared_ptr<flexiv::RobotStates> robotStates,
    std::shared_ptr<flexiv::Robot> robot, flexiv::Log& log)
{
    // read robot states
    robot->getRobotStates(robotStates.get());

    // set initial joint position
    if (!g_isInitPositionSet) {
        // check vector size before saving
        if (robotStates->m_q.size() == k_robotDofs) {
            g_initPosition = robotStates->m_q;
            g_isInitPositionSet = true;
        }
    }
    // run control only after init position is set
    else {
        // initialize target position to hold position
        auto targetPosition = g_initPosition;

        std::vector<double> torqueDesired(k_robotDofs);
        // impedance control on all joints
        for (size_t i = 0; i < k_robotDofs; ++i) {
            torqueDesired[i]
                = k_impedanceKp[i] * (targetPosition[i] - robotStates->m_q[i])
                  - k_impedanceKd[i] * robotStates->m_dtheta[i];
        }

        // send target joint torque to RDK server
        robot->streamJointTorque(torqueDesired, true);
    }

    if (g_loopCounter == 5000) {
        log.warn(">>>>>>>>>>>>> Adding simulated loop delay <<<<<<<<<<<<<<<");
    }
    // simulate prolonged loop time after 5 seconds
    else if (g_loopCounter > 5000) {
        std::this_thread::sleep_for(std::chrono::microseconds(995));
    }

    g_loopCounter++;
}

int main(int argc, char* argv[])
{
    // log object for printing message with timestamp and coloring
    flexiv::Log log;

    // Parse Parameters
    //=============================================================================
    // check if program has 3 arguments
    if (argc != 3) {
        log.error("Invalid program arguments. Usage: <robot_ip> <local_ip>");
        return 0;
    }
    // IP of the robot server
    std::string robotIP = argv[1];

    // IP of the workstation PC running this program
    std::string localIP = argv[2];

    // RDK Initialization
    //=============================================================================
    // instantiate robot interface
    auto robot = std::make_shared<flexiv::Robot>();

    // create data struct for storing robot states
    auto robotStates = std::make_shared<flexiv::RobotStates>();

    // initialize robot interface and connect to the robot server
    robot->init(robotIP, localIP);

    // wait for the connection to be established
    do {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } while (!robot->isConnected());

    // enable the robot, make sure the E-stop is released before enabling
    if (robot->enable()) {
        log.info("Enabling robot ...");
    }

    // wait for the robot to become operational
    do {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } while (!robot->isOperational());
    log.info("Robot is now operational");

    // set mode after robot is operational
    robot->setMode(flexiv::MODE_JOINT_TORQUE);

    // wait for the mode to be switched
    do {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } while (robot->getMode() != flexiv::MODE_JOINT_TORQUE);

    log.warn(
        ">>>>>>>>>>>>> Simulated loop delay will be added after 5 "
        "seconds <<<<<<<<<<<<<<<");

    // High-priority Realtime Periodic Task @ 1kHz
    //=============================================================================
    // this is a blocking method, so all other user-defined background threads
    // should be spawned before this
    robot->start(std::bind(periodicTask, robotStates, robot, log));

    return 0;
}
