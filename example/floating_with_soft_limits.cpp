/**
 * @example floating_with_soft_limits.cpp
 * Run damped floating with soft limits enabled to keep joints from
 * hitting position limits.
 * @copyright Copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <Robot.hpp>
#include <Log.hpp>

#include <string>
#include <thread>

namespace {
/** Robot DOF */
const int k_robotDofs = 7;

/** Damping gains for floating */
const std::vector<double> k_floatingDamping
    = {10.0, 10.0, 5.0, 5.0, 1.0, 1.0, 1.0};
}

/** Callback function for realtime periodic task */
void periodicTask(std::shared_ptr<flexiv::RobotStates> robotStates,
    std::shared_ptr<flexiv::Robot> robot)
{
    // Read robot states
    robot->getRobotStates(robotStates.get());

    // Set 0 joint torques
    std::vector<double> torqueDesired(k_robotDofs, 0.0);

    // Add some velocity damping
    for (size_t i = 0; i < k_robotDofs; ++i) {
        torqueDesired[i] = -k_floatingDamping[i] * robotStates->m_dtheta[i];
    }

    // Send target joint torque to RDK server, enable gravity compensation and
    // joint soft limits
    robot->streamJointTorque(torqueDesired, true, true);
}

int main(int argc, char* argv[])
{
    // Log object for printing message with timestamp and coloring
    flexiv::Log log;

    // Parse Parameters
    //=============================================================================
    // Check if program has 3 arguments
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
    robot->start(std::bind(periodicTask, robotStates, robot));

    return 0;
}
