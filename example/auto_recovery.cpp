/**
 * @example auto_recovery.cpp
 * Run auto-recovery if the robot's safety system is in recovery state,
 * otherwise run damped floating with joint soft limit disabled, so that the
 * user can manually trigger a safety system recovery state by moving any joint
 * close to its limit. See flexiv::Robot::isRecoveryState() for more details.
 * @copyright Copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <flexiv/Robot.hpp>
#include <flexiv/Exception.hpp>
#include <flexiv/Log.hpp>
#include <flexiv/Utility.hpp>

#include <iostream>
#include <string>
#include <thread>

void printHelp()
{
    // clang-format off
    std::cout << "Required arguments: [robot SN]" << std::endl;
    std::cout << "    robot SN: Serial number of the robot to connect to. "
                 "Remove any space, for example: Rizon4s-123456" << std::endl;
    std::cout << "Optional arguments: None" << std::endl;
    std::cout << std::endl;
    // clang-format on
}

int main(int argc, char* argv[])
{
    // Log object for printing message with timestamp and coloring
    flexiv::Log log;

    // Parse Parameters
    //=============================================================================
    if (argc < 2
        || flexiv::utility::programArgsExistAny(argc, argv, {"-h", "--help"})) {
        printHelp();
        return 1;
    }

    // Serial number of the robot to connect to. Remove any space, for example:
    // Rizon4s-123456
    std::string robotSN = argv[1];

    try {
        // RDK Initialization
        //=============================================================================
        // Instantiate robot interface
        flexiv::Robot robot(robotSN);

        // Create data struct for storing robot states
        flexiv::RobotStates robotStates;

        // Enable the robot, make sure the E-stop is released before enabling
        log.info("Enabling robot ...");
        robot.enable();

        // Application-specific Code
        //=============================================================================
        // If the system is in recovery state, we can't use isOperational to
        // tell if the enabling process is done, so just wait long enough for
        // the process to finish
        std::this_thread::sleep_for(std::chrono::seconds(8));

        // Start auto recovery if the system is in recovery state, the involved
        // Joints will start to move back into allowed position range
        if (robot.isRecoveryState()) {
            robot.startAutoRecovery();
            // Block forever, must reboot the robot and restart user program
            // after auto recovery is done
            while (true) {
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        }
        // Otherwise the system is normal, do nothing
        else {
            log.info(
                "Robot system is not in recovery state, nothing to be done, "
                "exiting ...");
        }
    } catch (const flexiv::Exception& e) {
        log.error(e.what());
        return 1;
    }

    return 0;
}
