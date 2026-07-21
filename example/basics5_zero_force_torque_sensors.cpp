/**
 * @example basics5_zero_force_torque_sensors.cpp
 * This tutorial zeros the robot's force and torque sensors, which is a recommended (but not
 * mandatory) step before any operations that require accurate force/torque measurement.
 * @copyright Copyright (C) 2016-2026 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <flexiv/rdk/robot.hpp>
#include <flexiv/rdk/utility.hpp>

#include <iostream>
#include <thread>
#include <algorithm>

using namespace flexiv;

/** @brief Print program usage help */
void PrintHelp()
{
    // clang-format off
    std::cout << "Required arguments: [robot_sn]" << std::endl;
    std::cout << "    robot_sn: Serial number of the robot to connect. Remove any space, e.g. Enlight-L-123456" << std::endl;
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
    // Serial number of the robot to connect to
    std::string robot_sn = argv[1];

    // Print description
    std::cout << ">>> Tutorial description <<<\nThis tutorial zeros the robot's force and torque "
                 "sensors, which is a recommended (but not mandatory) step before any operations "
                 "that require accurate force/torque measurement.\n"
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

        // Servo on the robot, make sure the E-stop is released
        std::cout << "Servo on the robot ..." << std::endl;
        robot.ServoOn();

        // Wait for the robot to become operational
        while (!robot.operational()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        std::cout << "Robot is now operational" << std::endl;

        // Zero Sensors
        // =========================================================================================
        // Get and print the current TCP force/moment readings
        for (const auto& [group, states] : robot.states()) {
            std::cout << "[" << rdk::kJointGroupNames.at(group)
                      << "] TCP force and moment reading in world frame BEFORE sensor zeroing: "
                      << rdk::utility::Arr2Str(states.tcp_wrench) << " N-Nm" << std::endl;
        }

        // Primitives can only be executed on single-arm joint groups
        const auto& single_arm_groups = robot.info().single_arm_groups;
        if (single_arm_groups.empty()) {
            throw std::runtime_error("No single-arm joint group found on the connected robot");
        }

        // Run the "ZeroFTSensor" primitive to automatically zero force and torque sensors
        robot.SwitchMode(rdk::Mode::NRT_PRIMITIVE_EXECUTION);
        std::map<rdk::JointGroup, rdk::PrimitiveArgs> pt_args;
        for (const auto& [group, _] : single_arm_groups) {
            pt_args[group] = rdk::PrimitiveArgs("ZeroFTSensor", {});
        }
        robot.ExecutePrimitive(pt_args);

        // WARNING: during the process, the robot must not contact anything, otherwise the result
        // will be inaccurate and affect following operations
        std::cerr
            << "[warn] Zeroing force/torque sensors, make sure nothing is in contact with the robot"
            << std::endl;

        // Wait for primitive to finish
        while (!rdk::utility::PrimitiveStateTrueForGroups(robot.primitive_states(), "terminated")) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        std::cout << "Sensor zeroing complete" << std::endl;

        // Get and print the current TCP force/moment readings
        for (const auto& [group, _] : single_arm_groups) {
            std::cout << "[" << rdk::kJointGroupNames.at(group)
                      << "] TCP force and moment reading in world frame AFTER sensor zeroing: "
                      << rdk::utility::Arr2Str(robot.states().at(group).tcp_wrench) << " N-Nm"
                      << std::endl;
        }

    } catch (const std::exception& e) {
        std::cerr << "[error] " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
