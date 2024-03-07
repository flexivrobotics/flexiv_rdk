/**
 * @example basics6_gripper_control.cpp
 * This tutorial does position and force control (if available) for grippers supported by Flexiv.
 * @copyright Copyright (C) 2016-2023 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <flexiv/robot.h>
#include <flexiv/log.h>
#include <flexiv/gripper.h>
#include <flexiv/utility.h>

#include <iostream>
#include <string>
#include <thread>
#include <atomic>

namespace {
/** Global flag: whether the gripper control tasks are finished */
std::atomic<bool> g_finished = {false};
}

/** @brief Print tutorial description */
void PrintDescription()
{
    std::cout << "This tutorial does position and force control (if available) for grippers "
                 "supported by Flexiv."
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

/** @brief Print gripper states data @ 1Hz */
void PrintGripperStates(flexiv::Gripper& gripper, flexiv::Log& log)
{
    while (!g_finished) {
        // Print all gripper states in JSON format using the built-in ostream operator overloading
        log.Info("Current gripper states:");
        std::cout << gripper.states() << std::endl;
        std::cout << "moving: " << gripper.moving() << std::endl;
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

        // Enable the robot, make sure the E-Stop is released before enabling
        log.Info("Enabling robot ...");
        robot.Enable();

        // Wait for the robot to become operational
        while (!robot.operational()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        log.Info("Robot is now operational");

        // Gripper Control
        // =========================================================================================
        // Gripper control is not available if the robot is in IDLE mode, so switch to some mode
        // other than IDLE
        robot.SwitchMode(flexiv::Mode::NRT_PLAN_EXECUTION);
        robot.ExecutePlan("PLAN-Home");
        std::this_thread::sleep_for(std::chrono::seconds(1));

        // Instantiate gripper control interface
        flexiv::Gripper gripper(robot);

        // Manually initialize the gripper, not all grippers need this step
        gripper.Init();

        // Thread for printing gripper states
        std::thread print_thread(PrintGripperStates, std::ref(gripper), std::ref(log));

        // Position control
        log.Info("Closing gripper");
        gripper.Move(0.01, 0.1, 20);
        std::this_thread::sleep_for(std::chrono::seconds(2));
        log.Info("Opening gripper");
        gripper.Move(0.09, 0.1, 20);
        std::this_thread::sleep_for(std::chrono::seconds(2));

        // Stop
        log.Info("Closing gripper");
        gripper.Move(0.01, 0.1, 20);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        log.Info("Stopping gripper");
        gripper.Stop();
        std::this_thread::sleep_for(std::chrono::seconds(2));
        log.Info("Closing gripper");
        gripper.Move(0.01, 0.1, 20);
        std::this_thread::sleep_for(std::chrono::seconds(2));
        log.Info("Opening gripper");
        gripper.Move(0.09, 0.1, 20);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        log.Info("Stopping gripper");
        gripper.Stop();
        std::this_thread::sleep_for(std::chrono::seconds(2));

        // Force control, if available (sensed force is not zero)
        if (fabs(gripper.states().force) > std::numeric_limits<double>::epsilon()) {
            log.Info("Gripper running zero force control");
            gripper.Grasp(0);
            // Exit after 10 seconds
            std::this_thread::sleep_for(std::chrono::seconds(10));
        }

        // Finished, exit all threads
        gripper.Stop();
        g_finished = true;
        log.Info("Program finished");
        print_thread.join();

    } catch (const std::exception& e) {
        log.Error(e.what());
        return 1;
    }

    return 0;
}
