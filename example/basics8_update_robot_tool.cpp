/**
 * @example basics8_update_robot_tool.cpp
 * This tutorial shows how to online update and interact with the robot tools. All changes made to
 * the robot tool system will take effect immediately without needing to reboot. However, the robot
 * must be put into IDLE mode when making these changes.
 * @copyright Copyright (C) 2016-2023 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <flexiv/Robot.hpp>
#include <flexiv/Tool.hpp>
#include <flexiv/Utility.hpp>
#include <flexiv/Log.hpp>

#include <iostream>
#include <string>
#include <thread>

/** @brief Print tutorial description */
void printDescription()
{
    std::cout
        << "This tutorial shows how to online update and interact with the robot tools. All "
           "changes made to the robot tool system will take effect immediately without needing to "
           "reboot. However, the robot must be put into IDLE mode when making these changes."
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

        // Update Robot Tool
        // =========================================================================================
        // Make sure the robot is in IDLE mode
        robot.setMode(flexiv::Mode::IDLE);

        // Instantiate tool interface
        flexiv::Tool tool(robot);

        // Get and print a list of already configured tools currently in the robot's tools pool
        log.info("All configured tools:");
        auto toolList = tool.list();
        for (size_t i = 0; i < toolList.size(); i++) {
            std::cout << "[" << i << "] " << toolList[i] << std::endl;
        }
        std::cout << std::endl;

        // Get and print the current active tool
        log.info("Current active tool: " + tool.name());

        // Set name and parameters for a new tool
        std::string newToolName = "ExampleTool1";
        flexiv::ToolParams newToolParams;
        newToolParams.mass = 0.9;
        newToolParams.CoM = {0.0, 0.0, 0.057};
        newToolParams.inertia = {2.768e-03, 3.149e-03, 5.64e-04, 0.0, 0.0, 0.0};
        newToolParams.tcpLocation = {0.0, -0.207, 0.09, 0.7071068, 0.7071068, 0.0, 0.0};

        // If there's already a tool with the same name in the robot's tools pool, then remove it
        // first, because duplicate tool names are not allowed
        if (tool.isExist(newToolName)) {
            log.warn(
                "Tool with the same name [" + newToolName + "] already exists, removing it now");
            // Switch to other tool or no tool (Flange) before removing the current tool
            tool.switchTo("Flange");
            tool.remove(newToolName);
        }

        // Add the new tool
        log.info("Adding new tool [" + newToolName + "] to the robot");
        tool.add(newToolName, newToolParams);

        // Get and print the tools list again, the new tool should appear at the end
        log.info("All configured tools:");
        toolList = tool.list();
        for (size_t i = 0; i < toolList.size(); i++) {
            std::cout << "[" << i << "] " << toolList[i] << std::endl;
        }
        std::cout << std::endl;

        // Switch to the newly added tool, i.e. set it as the active tool
        log.info("Switching to tool [" + newToolName + "]");
        tool.switchTo(newToolName);

        // Get and print the current active tool again, should be the new tool
        log.info("Current active tool: " + tool.name());

        // Switch to other tool or no tool (Flange) before removing the current tool
        tool.switchTo("Flange");

        // Clean up by removing the new tool
        std::this_thread::sleep_for(std::chrono::seconds(2));
        log.info("Removing tool [" + newToolName + "]");
        tool.remove(newToolName);

        log.info("Program finished");

    } catch (const std::exception& e) {
        log.error(e.what());
        return 1;
    }

    return 0;
}
