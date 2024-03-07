/**
 * @example basics7_auto_recovery.cpp
 * This tutorial runs an automatic recovery process if the robot's safety system is in recovery
 * state. See flexiv::Robot::recovery() and RDK manual for more details.
 * @copyright Copyright (C) 2016-2023 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <flexiv/robot.h>
#include <flexiv/log.h>
#include <flexiv/utility.h>

#include <iostream>
#include <string>
#include <thread>

/** @brief Print tutorial description */
void PrintDescription()
{
    std::cout
        << "This tutorial runs an automatic recovery process if the robot's safety system is in "
           "recovery state. See flexiv::Robot::recovery() and RDK manual for more details."
        << std::endl
        << std::endl;
}

/** @brief Print program usage help */
void PrintHelp()
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
    // Program Setup
    // =============================================================================================
    // Logger for printing message with timestamp and coloring
    flexiv::Log log;

    // Parse parameters
    if (argc < 2 || flexiv::utility::ProgramArgsExistAny(argc, argv, {"-h", "--help"})) {
        PrintHelp();
        return 1;
    }
    // Serial number of the robot to connect to. Remove any space, for example: Rizon4s-123456
    std::string robot_sn = argv[1];

    // Print description
    log.Info("Tutorial description:");
    PrintDescription();

    try {
        // RDK Initialization
        // =========================================================================================
        // Instantiate robot interface
        flexiv::Robot robot(robot_sn);

        // Enable the robot, make sure the E-stop is released before enabling
        log.Info("Enabling robot ...");
        robot.Enable();

        // Run Auto-recovery
        // =========================================================================================
        // If the system is in recovery state, we can't use isOperational to tell if the enabling
        // process is done, so just wait long enough for the process to finish
        std::this_thread::sleep_for(std::chrono::seconds(8));

        // Run automatic recovery if the system is in recovery state, the involved joints will start
        // to move back into allowed position range
        if (robot.recovery()) {
            robot.RunAutoRecovery();
        }
        // Otherwise the system is normal, do nothing
        else {
            log.Info("Robot system is not in recovery state, nothing to be done, exiting ...");
        }
    } catch (const std::exception& e) {
        log.Error(e.what());
        return 1;
    }

    return 0;
}
