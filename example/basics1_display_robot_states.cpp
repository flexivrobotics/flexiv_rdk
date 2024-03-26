/**
 * @example basics1_display_robot_states.cpp
 * This tutorial does the very first thing: check connection with the robot server and print
 * received robot states.
 * @copyright Copyright (C) 2016-2023 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <flexiv/robot.h>
#include <flexiv/log.h>
#include <flexiv/utility.h>

#include <iostream>
#include <thread>

/** @brief Print tutorial description */
void PrintDescription()
{
    std::cout << "This tutorial does the very first thing: check connection with the robot server "
                 "and print received robot states."
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

/** @brief Print robot states data @ 1Hz */
void printRobotStates(flexiv::Robot& robot, flexiv::Log& log)
{
    while (true) {
        // Print all robot states in JSON format using the built-in ostream operator overloading
        log.Info("Current robot states:");
        std::cout << robot.states() << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
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

        // Clear fault on the connected robot if any
        if (robot.fault()) {
            log.Warn("Fault occurred on the connected robot, trying to clear ...");
            // Try to clear the fault
            if (!robot.ClearFault()) {
                log.Error("Fault cannot be cleared, exiting ...");
                return 1;
            }
            log.Info("Fault on the connected robot is cleared");
        }

        // Enable the robot, make sure the E-stop is released before enabling
        log.Info("Enabling robot ...");
        robot.Enable();

        // Wait for the robot to become operational
        while (!robot.operational()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        log.Info("Robot is now operational");

        // Print States
        // =========================================================================================
        // Use std::thread to do scheduling so that this example can run on all OS, since not all OS
        // support flexiv::Scheduler
        std::thread low_priority_thread(
            std::bind(printRobotStates, std::ref(robot), std::ref(log)));

        // Properly exit thread
        low_priority_thread.join();

    } catch (const std::exception& e) {
        log.Error(e.what());
        return 1;
    }

    return 0;
}
