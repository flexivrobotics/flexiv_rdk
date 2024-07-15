/**
 * @example basics7_auto_recovery.cpp
 * This tutorial runs an automatic recovery process if the robot's safety system is in recovery
 * state. See flexiv::rdk::Robot::recovery() and RDK manual for more details.
 * @copyright Copyright (C) 2016-2024 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <flexiv/rdk/robot.hpp>
#include <flexiv/rdk/utility.hpp>
#include <spdlog/spdlog.h>

#include <iostream>
#include <string>
#include <thread>

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
    // Parse parameters
    if (argc < 2 || flexiv::rdk::utility::ProgramArgsExistAny(argc, argv, {"-h", "--help"})) {
        PrintHelp();
        return 1;
    }
    // Serial number of the robot to connect to. Remove any space, for example: Rizon4s-123456
    std::string robot_sn = argv[1];

    // Print description
    spdlog::info(
        ">>> Tutorial description <<<\nThis tutorial runs an automatic recovery process if the "
        "robot's safety system is in recovery state. See flexiv::rdk::Robot::recovery() and RDK "
        "manual for more details.");

    try {
        // RDK Initialization
        // =========================================================================================
        // Instantiate robot interface
        flexiv::rdk::Robot robot(robot_sn);

        // Enable the robot, make sure the E-stop is released before enabling
        spdlog::info("Enabling robot ...");
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
            spdlog::info("Robot system is not in recovery state, nothing to be done, exiting ...");
        }
    } catch (const std::exception& e) {
        spdlog::error(e.what());
        return 1;
    }

    return 0;
}
