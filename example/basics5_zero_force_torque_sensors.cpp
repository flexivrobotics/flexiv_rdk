/**
 * @example basics5_zero_force_torque_sensors.cpp
 * This tutorial zeros the robot's force and torque sensors, which is a recommended (but not
 * mandatory) step before any operations that require accurate force/torque measurement.
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
    std::cout << "This tutorial zeros the robot's force and torque sensors, which is a recommended "
                 "(but not mandatory) step before any operations that require accurate "
                 "force/torque measurement."
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

        // Zero Sensors
        // =========================================================================================
        // Get and print the current TCP force/moment readings
        log.Info("TCP force and moment reading in base frame BEFORE sensor zeroing: "
                 + flexiv::utility::Arr2Str(robot.states().ext_wrench_in_world) + "[N][Nm]");

        // Run the "ZeroFTSensor" primitive to automatically zero force and torque sensors
        robot.SwitchMode(flexiv::Mode::NRT_PRIMITIVE_EXECUTION);
        robot.ExecutePrimitive("ZeroFTSensor()");

        // WARNING: during the process, the robot must not contact anything, otherwise the result
        // will be inaccurate and affect following operations
        log.Warn("Zeroing force/torque sensors, make sure nothing is in contact with the robot");

        // Wait for primitive completion
        while (robot.busy()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        log.Info("Sensor zeroing complete");

        // Get and print the current TCP force/moment readings
        log.Info("TCP force and moment reading in base frame AFTER sensor zeroing: "
                 + flexiv::utility::Arr2Str(robot.states().ext_wrench_in_world) + "[N][Nm]");

    } catch (const std::exception& e) {
        log.Error(e.what());
        return 1;
    }

    return 0;
}
