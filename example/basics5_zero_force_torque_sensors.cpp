/**
 * @example basics5_zero_force_torque_sensors.cpp
 * This tutorial zeros the robot's force and torque sensors, which is a recommended (but not
 * mandatory) step before any operations that require accurate force/torque measurement.
 * @copyright Copyright (C) 2016-2026 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <flexiv/rdk/robot.hpp>
#include <flexiv/rdk/utility.hpp>
#include <spdlog/spdlog.h>

#include <iostream>
#include <thread>
#include <algorithm>

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
    spdlog::info(
        ">>> Tutorial description <<<\nThis tutorial zeros the robot's force and torque sensors, "
        "which is a recommended (but not mandatory) step before any operations that require "
        "accurate force/torque measurement.\n");

    try {
        // RDK Initialization
        // =========================================================================================
        // Instantiate robot interface
        rdk::Robot robot(robot_sn);

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

        // Zero Sensors
        // =========================================================================================
        // Get and print the current TCP force/moment readings
        for (const auto& [group, states] : robot.states()) {
            spdlog::info(
                "[{}] TCP force and moment reading in world frame BEFORE sensor zeroing: {} N-Nm",
                rdk::kJointGroupNames.at(group), rdk::utility::Arr2Str(states.tcp_wrench));
        }

        // All available joint groups of the robot
        const auto joint_groups = robot.groups();

        // Run the "ZeroFTSensor" primitive to automatically zero force and torque sensors
        robot.SwitchMode(rdk::Mode::NRT_PRIMITIVE_EXECUTION);
        std::map<rdk::JointGroup, rdk::PrimitiveArgs> pt_args;
        for (const auto& group : joint_groups) {
            pt_args[group] = rdk::PrimitiveArgs("ZeroFTSensor", {});
        }
        robot.ExecutePrimitive(pt_args);

        // WARNING: during the process, the robot must not contact anything, otherwise the result
        // will be inaccurate and affect following operations
        spdlog::warn(
            "Zeroing force/torque sensors, make sure nothing is in contact with the robot");

        // Wait for primitive to finish
        while (!std::all_of(joint_groups.begin(), joint_groups.end(), [&robot](const auto& group) {
            return std::get<int>(
                robot.primitive_states().at(group).names_and_values.at("terminated"));
        })) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        spdlog::info("Sensor zeroing complete");

        // Get and print the current TCP force/moment readings
        for (const auto& group : joint_groups) {
            spdlog::info(
                "[{}] TCP force and moment reading in world frame AFTER sensor zeroing: {} N-Nm",
                rdk::kJointGroupNames.at(group),
                rdk::utility::Arr2Str(robot.states().at(group).tcp_wrench));
        }

    } catch (const std::exception& e) {
        spdlog::error(e.what());
        return 1;
    }

    return 0;
}
