/**
 * @example plan_execution.cpp
 * Select a plan from a list to execute using RDK's plan execution API.
 * @copyright Copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <flexiv/Robot.hpp>
#include <flexiv/Exception.hpp>
#include <flexiv/Log.hpp>
#include <flexiv/Utility.hpp>

#include <iostream>
#include <thread>

void printHelp()
{
    // clang-format off
    std::cout << "Required arguments: [robot IP] [local IP]" << std::endl;
    std::cout << "    robot IP: address of the robot server" << std::endl;
    std::cout << "    local IP: address of this PC" << std::endl;
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
    if (argc < 3
        || flexiv::utility::programArgsExistAny(argc, argv, {"-h", "--help"})) {
        printHelp();
        return 1;
    }

    // IP of the robot server
    std::string robotIP = argv[1];

    // IP of the workstation PC running this program
    std::string localIP = argv[2];

    try {
        // RDK Initialization
        //=============================================================================
        // Instantiate robot interface
        flexiv::Robot robot(robotIP, localIP);

        // Clear fault on robot server if any
        if (robot.isFault()) {
            log.warn("Fault occurred on robot server, trying to clear ...");
            // Try to clear the fault
            robot.clearFault();
            std::this_thread::sleep_for(std::chrono::seconds(2));
            // Check again
            if (robot.isFault()) {
                log.error("Fault cannot be cleared, exiting ...");
                return 1;
            }
            log.info("Fault on robot server is cleared");
        }

        // Enable the robot, make sure the E-stop is released before enabling
        log.info("Enabling robot ...");
        robot.enable();

        // Wait for the robot to become operational
        int secondsWaited = 0;
        while (!robot.isOperational()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            if (++secondsWaited == 10) {
                log.warn(
                    "Still waiting for robot to become operational, please "
                    "check that the robot 1) has no fault, 2) is booted "
                    "into Auto mode");
            }
        }
        log.info("Robot is now operational");

        // Set mode after robot is operational
        robot.setMode(flexiv::MODE_PLAN_EXECUTION);

        // Wait for the mode to be switched
        while (robot.getMode() != flexiv::MODE_PLAN_EXECUTION) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        // Application-specific Code
        //=============================================================================
        // Plan info data
        flexiv::PlanInfo planInfo;

        while (true) {
            // Monitor fault on robot server
            if (robot.isFault()) {
                throw flexiv::ServerException(
                    "Fault occurred on robot server, exiting ...");
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
                        std::cout << "[" << i << "] " << planList[i]
                                  << std::endl;
                    }
                    std::cout << std::endl;
                } break;
                // Execute plan by index
                case 2: {
                    log.info("Enter plan index to execute:");
                    int index;
                    std::cin >> index;
                    robot.executePlanByIndex(index);

                    // Print plan info while the current plan is running
                    while (robot.isBusy()) {
                        robot.getPlanInfo(planInfo);
                        log.info(
                            "===============================================");
                        std::cout << planInfo << std::endl;
                        std::this_thread::sleep_for(std::chrono::seconds(1));
                    }
                } break;
                // Execute plan by name
                case 3: {
                    log.info("Enter plan name to execute:");
                    std::string name;
                    std::cin >> name;
                    robot.executePlanByName(name);

                    // Print plan info while the current plan is running
                    while (robot.isBusy()) {
                        robot.getPlanInfo(planInfo);
                        log.info(
                            "===============================================");
                        std::cout << planInfo << std::endl;
                        std::this_thread::sleep_for(std::chrono::seconds(1));
                    }
                } break;
                default:
                    log.warn("Invalid input");
                    break;
            }
        }

    } catch (const flexiv::Exception& e) {
        log.error(e.what());
        return 1;
    }

    return 0;
}
