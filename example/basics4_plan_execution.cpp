/**
 * @example basics4_plan_execution.cpp
 * This tutorial executes a plan selected by the user from a list of available plans. A plan is a
 * pre-written script to execute a series of robot primitives with pre-defined transition conditions
 * between 2 adjacent primitives. Users can use Flexiv Elements to compose their own plan and assign
 * to the robot, which will appear in the plan list.
 * @copyright Copyright (C) 2016-2024 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <flexiv/rdk/robot.hpp>
#include <flexiv/rdk/utility.hpp>
#include <spdlog/spdlog.h>

#include <iostream>
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
        ">>> Tutorial description <<<\nThis tutorial executes a plan selected by the user from a "
        "list of available plans. A plan is a pre-written script to execute a series of robot "
        "primitives with pre-defined transition conditions between 2 adjacent primitives. Users "
        "can use Flexiv Elements to compose their own plan and assign to the robot, which "
        "will appear in the plan list.");

    try {
        // RDK Initialization
        // =========================================================================================
        // Instantiate robot interface
        flexiv::rdk::Robot robot(robot_sn);

        // Clear fault on the connected robot if any
        if (robot.fault()) {
            spdlog::warn("Fault occurred on the connected robot, trying to clear ...");
            // Try to clear the fault
            if (!robot.ClearFault()) {
                spdlog::error("Fault cannot be cleared, exiting ...");
                return 1;
            }
            spdlog::info("Fault on the connected robot is cleared");
        }

        // Enable the robot, make sure the E-stop is released before enabling
        spdlog::info("Enabling robot ...");
        robot.Enable();

        // Wait for the robot to become operational
        while (!robot.operational()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        spdlog::info("Robot is now operational");

        // Execute Plans
        // =========================================================================================
        // Switch to plan execution mode
        robot.SwitchMode(flexiv::rdk::Mode::NRT_PLAN_EXECUTION);

        while (true) {
            // Monitor fault on the connected robot
            if (robot.fault()) {
                throw std::runtime_error("Fault occurred on the connected robot, exiting ...");
            }

            // Get user input
            spdlog::info("Choose an action:");
            std::cout << "[1] Show available plans" << std::endl;
            std::cout << "[2] Execute a plan by index" << std::endl;
            std::cout << "[3] Execute a plan by name" << std::endl;

            std::string inputBuffer;
            std::cin >> inputBuffer;
            int userInput = std::stoi(inputBuffer);

            switch (userInput) {
                // Get and show plan list
                case 1: {
                    auto plan_list = robot.plan_list();
                    for (size_t i = 0; i < plan_list.size(); i++) {
                        std::cout << "[" << i << "] " << plan_list[i] << std::endl;
                    }
                    std::cout << std::endl;
                } break;
                // Execute plan by index
                case 2: {
                    spdlog::info("Enter plan index to execute:");
                    int index;
                    std::cin >> index;
                    // Allow the plan to continue its execution even if the RDK program is closed or
                    // the connection is lost
                    robot.ExecutePlan(index, true);

                    // Print plan info while the current plan is running
                    while (robot.busy()) {
                        spdlog::info("Current plan info:");
                        std::cout << robot.plan_info() << std::endl;
                        std::this_thread::sleep_for(std::chrono::seconds(1));
                    }
                } break;
                // Execute plan by name
                case 3: {
                    spdlog::info("Enter plan name to execute:");
                    std::string name;
                    std::cin >> name;
                    // Allow the plan to continue its execution even if the RDK program is closed or
                    // the connection is lost
                    robot.ExecutePlan(name, true);

                    // Print plan info while the current plan is running
                    while (robot.busy()) {
                        spdlog::info("Current plan info:");
                        std::cout << robot.plan_info() << std::endl;
                        std::this_thread::sleep_for(std::chrono::seconds(1));
                    }
                } break;
                default:
                    spdlog::warn("Invalid input");
                    break;
            }
        }

    } catch (const std::exception& e) {
        spdlog::error(e.what());
        return 1;
    }

    return 0;
}
