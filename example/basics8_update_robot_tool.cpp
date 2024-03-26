/**
 * @example basics8_update_robot_tool.cpp
 * This tutorial shows how to online update and interact with the robot tools. All changes made to
 * the robot tool system will take effect immediately without needing to reboot. However, the robot
 * must be put into IDLE mode when making these changes.
 * @copyright Copyright (C) 2016-2023 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <flexiv/robot.h>
#include <flexiv/tool.h>
#include <flexiv/utility.h>
#include <flexiv/log.h>

#include <iostream>
#include <string>
#include <thread>

/** @brief Print tutorial description */
void PrintDescription()
{
    std::cout
        << "This tutorial shows how to online update and interact with the robot tools. All "
           "changes made to the robot tool system will take effect immediately without needing to "
           "reboot. However, the robot must be put into IDLE mode when making these changes."
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

        // Update Robot Tool
        // =========================================================================================
        // Make sure the robot is in IDLE mode
        robot.SwitchMode(flexiv::Mode::IDLE);

        // Instantiate tool interface
        flexiv::Tool tool(robot);

        // Get and print a list of already configured tools currently in the robot's tools pool
        log.Info("All configured tools:");
        auto tool_list = tool.list();
        for (size_t i = 0; i < tool_list.size(); i++) {
            std::cout << "[" << i << "] " << tool_list[i] << std::endl;
        }
        std::cout << std::endl;

        // Get and print the current active tool
        log.Info("Current active tool: " + tool.name());

        // Set name and parameters for a new tool
        std::string new_tool_name = "ExampleTool1";
        flexiv::ToolParams new_tool_params;
        new_tool_params.mass = 0.9;
        new_tool_params.CoM = {0.0, 0.0, 0.057};
        new_tool_params.inertia = {2.768e-03, 3.149e-03, 5.64e-04, 0.0, 0.0, 0.0};
        new_tool_params.tcp_location = {0.0, -0.207, 0.09, 0.7071068, 0.7071068, 0.0, 0.0};

        // If there's already a tool with the same name in the robot's tools pool, then remove it
        // first, because duplicate tool names are not allowed
        if (tool.exist(new_tool_name)) {
            log.Warn(
                "Tool with the same name [" + new_tool_name + "] already exists, removing it now");
            // Switch to other tool or no tool (Flange) before removing the current tool
            tool.Switch("Flange");
            tool.Remove(new_tool_name);
        }

        // Add the new tool
        log.Info("Adding new tool [" + new_tool_name + "] to the robot");
        tool.Add(new_tool_name, new_tool_params);

        // Get and print the tools list again, the new tool should appear at the end
        log.Info("All configured tools:");
        tool_list = tool.list();
        for (size_t i = 0; i < tool_list.size(); i++) {
            std::cout << "[" << i << "] " << tool_list[i] << std::endl;
        }
        std::cout << std::endl;

        // Switch to the newly added tool, i.e. set it as the active tool
        log.Info("Switching to tool [" + new_tool_name + "]");
        tool.Switch(new_tool_name);

        // Get and print the current active tool again, should be the new tool
        log.Info("Current active tool: " + tool.name());

        // Switch to other tool or no tool (Flange) before removing the current tool
        tool.Switch("Flange");

        // Clean up by removing the new tool
        std::this_thread::sleep_for(std::chrono::seconds(2));
        log.Info("Removing tool [" + new_tool_name + "]");
        tool.Remove(new_tool_name);

        log.Info("Program finished");

    } catch (const std::exception& e) {
        log.Error(e.what());
        return 1;
    }

    return 0;
}
