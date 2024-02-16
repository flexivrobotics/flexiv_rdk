/**
 * @example basics4_plan_execution.cpp
 * This tutorial executes a plan selected by the user from a list of available plans. A plan is a
 * pre-written script to execute a series of robot primitives with pre-defined transition conditions
 * between 2 adjacent primitives. Users can use Flexiv Elements to compose their own plan and assign
 * to the robot, which will appear in the plan list.
 * @copyright Copyright (C) 2016-2023 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <flexiv/Robot.hpp>
#include <flexiv/Log.hpp>
#include <flexiv/Utility.hpp>

#include <iostream>
#include <thread>

/** @brief Print tutorial description */
void printDescription()
{
    std::cout << "This tutorial executes a plan selected by the user from a list of available "
                 "plans. A plan is a pre-written script to execute a series of robot primitives "
                 "with pre-defined transition conditions between 2 adjacent primitives. Users can "
                 "use Flexiv Elements to compose their own plan and assign to the robot, which "
                 "will appear in the plan list."
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

        // Clear fault on the connected robot if any
        if (robot.isFault()) {
            log.warn("Fault occurred on the connected robot, trying to clear ...");
            // Try to clear the fault
            if (!robot.clearFault()) {
                log.error("Fault cannot be cleared, exiting ...");
                return 1;
            }
            log.info("Fault on the connected robot is cleared");
        }

        // Enable the robot, make sure the E-stop is released before enabling
        log.info("Enabling robot ...");
        robot.enable();

        // Wait for the robot to become operational
        while (!robot.isOperational()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        log.info("Robot is now operational");

        // Execute Plans
        // =========================================================================================
        // Switch to plan execution mode
        robot.setMode(flexiv::Mode::NRT_PLAN_EXECUTION);

        while (true) {
            // Monitor fault on the connected robot
            if (robot.isFault()) {
                throw std::runtime_error("Fault occurred on the connected robot, exiting ...");
            }

            // Get user input
            log.info("Choose an action:");
            std::cout << "[1] Show available plans" << std::endl;
            std::cout << "[2] Execute a plan by index" << std::endl;
            std::cout << "[3] Execute a plan by name" << std::endl;

            std::string inputBuffer;
            std::cin >> inputBuffer;
            int userInput = std::stoi(inputBuffer);

            switch (userInput) {
                // Get and show plan list
                case 1: {
                    auto planList = robot.getPlanNameList();
                    for (size_t i = 0; i < planList.size(); i++) {
                        std::cout << "[" << i << "] " << planList[i] << std::endl;
                    }
                    std::cout << std::endl;
                } break;
                // Execute plan by index
                case 2: {
                    log.info("Enter plan index to execute:");
                    int index;
                    std::cin >> index;
                    // Allow the plan to continue its execution even if the RDK program is closed or
                    // the connection is lost
                    robot.executePlan(index, 100, true);

                    // Print plan info while the current plan is running
                    while (robot.isBusy()) {
                        log.info("===============================================");
                        std::cout << robot.getPlanInfo() << std::endl;
                        std::this_thread::sleep_for(std::chrono::seconds(1));
                    }
                } break;
                // Execute plan by name
                case 3: {
                    log.info("Enter plan name to execute:");
                    std::string name;
                    std::cin >> name;
                    // Allow the plan to continue its execution even if the RDK program is closed or
                    // the connection is lost
                    robot.executePlan(name, 100, true);

                    // Print plan info while the current plan is running
                    while (robot.isBusy()) {
                        log.info("===============================================");
                        std::cout << robot.getPlanInfo() << std::endl;
                        std::this_thread::sleep_for(std::chrono::seconds(1));
                    }
                } break;
                default:
                    log.warn("Invalid input");
                    break;
            }
        }

    } catch (const std::exception& e) {
        log.error(e.what());
        return 1;
    }

    return 0;
}
