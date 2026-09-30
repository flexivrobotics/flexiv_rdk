/**
 * @example basics1_display_robot_states.cpp
 * This tutorial does the very first thing: check connection with the robot server and print
 * received robot states and actions.
 * @copyright Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <flexiv/rdk/robot.hpp>
#include <flexiv/rdk/utility.hpp>

#include <iostream>
#include <thread>

using namespace flexiv;

/** @brief Print program usage help */
void PrintHelp()
{
    // clang-format off
    std::cout << "Required arguments: [robot_sn]" << std::endl;
    std::cout << "    robot_sn: Serial number of the robot to connect. Remove any space, e.g. Rizon4s-123456" << std::endl;
    std::cout << "Optional arguments: None" << std::endl;
    std::cout << std::endl;
    // clang-format on
}

/** @brief Print robot states and actions data @ 1Hz */
void PrintRobotStates(rdk::Robot& robot)
{
    while (true) {
        // Print all robot states in JSON format using the built-in ostream operator overloading
        std::cout << "Current robot states:" << std::endl;
        std::cout << robot.states() << std::endl;

        // Print all robot actions in JSON format using the built-in ostream operator overloading
        std::cout << "Current robot actions:" << std::endl;
        std::cout << robot.actions() << std::endl;

        // Print digital inputs
        std::cout << "Current digital inputs:" << std::endl;
        std::cout << rdk::utility::Arr2Str(robot.digital_inputs()) << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

int main(int argc, char* argv[])
{
    // Program Setup
    // =============================================================================================
    // Parse parameters
    if (argc < 2 || rdk::utility::ProgramArgsExistAny(argc, argv, {"-h", "--help"})) {
        PrintHelp();
        return 1;
    }
    // Serial number of the robot to connect to. Remove any space, for example: Rizon4s-123456
    std::string robot_sn = argv[1];

    // Print description
    std::cout << ">>> Tutorial description <<<\nThis tutorial does the very first thing: check "
                 "connection with the robot server and print received robot states and actions.\n"
              << std::endl;

    try {
        // RDK Initialization
        // =========================================================================================
        // Instantiate robot interface
        rdk::Robot robot(robot_sn);

        // Clear fault on the connected robot if any
        if (robot.fault()) {
            std::cerr << "[warn] Fault occurred on the connected robot, trying to clear ..."
                      << std::endl;
            // Try to clear the fault
            if (!robot.ClearFault()) {
                std::cerr << "[error] Fault cannot be cleared, exiting ..." << std::endl;
                return 1;
            }
            std::cout << "Fault on the connected robot is cleared" << std::endl;
        }

        // Enable the robot, make sure the E-stop is released before enabling
        std::cout << "Enabling robot ..." << std::endl;
        robot.Enable();

        // Wait for the robot to become operational
        while (!robot.operational()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        std::cout << "Robot is now operational" << std::endl;

        // Print States
        // =========================================================================================
        // Use std::thread to do scheduling so that this example can run on all OS, since not all OS
        // support rdk::Scheduler
        std::thread low_priority_thread(std::bind(PrintRobotStates, std::ref(robot)));

        // Properly exit thread
        low_priority_thread.join();

    } catch (const std::exception& e) {
        std::cerr << "[error] " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
