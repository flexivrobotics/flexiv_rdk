/**
 * @example basics2_clear_fault.cpp
 * This tutorial clears minor faults from the robot server if any. Note that critical faults cannot
 * be cleared, see RDK manual for more details.
 * @copyright Copyright (C) 2016-2023 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <flexiv/Robot.hpp>
#include <flexiv/Log.hpp>
#include <flexiv/Utility.hpp>

#include <iostream>
#include <string>
#include <thread>

/** @brief Print tutorial description */
void printDescription()
{
    std::cout << "This tutorial clears minor faults from the robot server if any. Note that "
                 "critical faults cannot be cleared, see RDK manual for more details."
              << std::endl
              << std::endl;
}

/** @brief Print program usage help */
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
    // Program Setup
    // =============================================================================================
    // Logger for printing message with timestamp and coloring
    flexiv::Log log;

    // Parse parameters
    if (argc < 2 || flexiv::utility::programArgsExistAny(argc, argv, {"-h", "--help"})) {
        printHelp();
        return 1;
    }
    // Serial number of the robot to connect to. Remove any space, for example: Rizon4s-123456
    std::string robotSN = argv[1];

    // Print description
    log.info("Tutorial description:");
    printDescription();

    try {
        // RDK Initialization
        // =========================================================================================
        // Instantiate robot interface
        flexiv::Robot robot(robotSN);

        // Fault Clearing
        // =========================================================================================
        // Clear fault on the connected robot if any
        if (robot.isFault()) {
            log.warn("Fault occurred on the connected robot, trying to clear ...");
            // Try to clear the fault
            if (!robot.clearFault()) {
                log.error("Fault cannot be cleared, exiting ...");
                return 1;
            }
            log.info("Fault on the connected robot is cleared");
        } else {
            log.info("No fault on the connected robot");
        }
    } catch (const std::exception& e) {
        log.error(e.what());
        return 1;
    }

    return 0;
}
