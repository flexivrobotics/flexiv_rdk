/**
 * @example Run damped floating with soft limits enabled to keep joints from
 * hitting position limits
 * @copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <config.h>
#include <Robot.hpp>

#include <iostream>
#include <string>
#include <thread>

namespace {
// robot DOF
const int k_robotDofs = 7;

const std::vector<double> k_floatingDamping
    = {10.0, 10.0, 5.0, 5.0, 1.0, 1.0, 1.0};
}

// callback function for realtime periodic task
void periodicTask(std::shared_ptr<flexiv::RobotStates> robotStates,
    std::shared_ptr<flexiv::Robot> robot)
{
    // read robot states
    robot->getRobotStates(robotStates.get());

    // set 0 joint torques
    std::vector<double> targetTorque(k_robotDofs, 0.0);

    // add some velocity damping
    for (size_t i = 0; i < k_robotDofs; ++i) {
        targetTorque[i]
            = -k_floatingDamping[i] * robotStates->m_linkVelocity[i];
    }

    // send target joint torque to RDK server, enable gravity compensation and
    // joint soft limits
    robot->streamJointTorque(targetTorque, true, true);
}

int main(int argc, char* argv[])
{
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
    robot->start(std::bind(periodicTask, robotStates, robot));

    return 0;
}
