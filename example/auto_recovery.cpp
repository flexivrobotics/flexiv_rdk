/**
 * @example auto_recovery.cpp
 * Run auto-recovery if the robot's safety system is in recovery state,
 * otherwise run damped floating with joint soft limit disabled, so that the
 * user can manually trigger a safety system recovery state by moving any joint
 * close to its limit. See flexiv::Mode::MODE_AUTO_RECOVERY for more details.
 * @copyright Copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <Robot.hpp>
#include <Log.hpp>

#include <string>
#include <thread>

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

    // Application-specific Code
    //=============================================================================
    // If the system is in recovery state, we can't use isOperational to tell if
    // the enabling process is done, so just wait long enough for the process to
    // finish
    std::this_thread::sleep_for(std::chrono::seconds(8));

    // Start auto recovery if the system is in recovery state, the involved
    // Joints will start to move back into allowed position range
    if (robot->isRecoveryState()) {
        robot->startAutoRecovery();
        // Block forever, must reboot the robot and restart user program after
        // auto recovery is done
        do {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        } while (true);
    }
    // Otherwise the system is normal, do nothing
    else {
        log.info(
            "Robot system is not in recovery state, nothing to be done, "
            "exiting ...");
    }

    return 0;
}
