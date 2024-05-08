/**
 * @example basics6_gripper_control.cpp
 * This tutorial does position and force control (if available) for grippers supported by Flexiv.
 * @copyright Copyright (C) 2016-2023 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <flexiv/robot.h>
#include <flexiv/gripper.h>
#include <flexiv/utility.h>
#include <spdlog/spdlog.h>

#include <iostream>
#include <string>
#include <thread>
#include <atomic>

namespace {
/** Global flag: whether the gripper control tasks are finished */
std::atomic<bool> g_finished = {false};
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
void PrintGripperStates(flexiv::Gripper& gripper)
{
    while (!g_finished) {
        // Print all gripper states in JSON format using the built-in ostream operator overloading
        spdlog::info("Current gripper states:");
        std::cout << gripper.states() << std::endl;
        std::cout << "moving: " << gripper.moving() << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

int main(int argc, char* argv[])
{
    // Program Setup
    // =============================================================================================
    // Parse parameters
    if (argc < 2 || flexiv::utility::ProgramArgsExistAny(argc, argv, {"-h", "--help"})) {
        PrintHelp();
        return 1;
    }
    // Serial number of the robot to connect to. Remove any space, for example: Rizon4s-123456
    std::string robot_sn = argv[1];

    // Print description
    spdlog::info(
        ">>> Tutorial description <<<\nThis tutorial does position and force control (if "
        "available) for grippers supported by Flexiv.");

    try {
        // RDK Initialization
        // =========================================================================================
        // Instantiate robot interface
        flexiv::Robot robot(robot_sn);

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

        // Enable the robot, make sure the E-Stop is released before enabling
        spdlog::info("Enabling robot ...");
        robot.Enable();

        // Wait for the robot to become operational
        while (!robot.operational()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        spdlog::info("Robot is now operational");

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
        spdlog::info("Initializing gripper, this process takes about 10 seconds ...");
        gripper.Init();
        spdlog::info("Initialization complete");

        // Thread for printing gripper states
        std::thread print_thread(PrintGripperStates, std::ref(gripper));

        // Position control
        spdlog::info("Closing gripper");
        gripper.Move(0.01, 0.1, 20);
        std::this_thread::sleep_for(std::chrono::seconds(2));
        spdlog::info("Opening gripper");
        gripper.Move(0.09, 0.1, 20);
        std::this_thread::sleep_for(std::chrono::seconds(2));

        // Stop
        spdlog::info("Closing gripper");
        gripper.Move(0.01, 0.1, 20);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        spdlog::info("Stopping gripper");
        gripper.Stop();
        std::this_thread::sleep_for(std::chrono::seconds(2));
        spdlog::info("Closing gripper");
        gripper.Move(0.01, 0.1, 20);
        std::this_thread::sleep_for(std::chrono::seconds(2));
        spdlog::info("Opening gripper");
        gripper.Move(0.09, 0.1, 20);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        spdlog::info("Stopping gripper");
        gripper.Stop();
        std::this_thread::sleep_for(std::chrono::seconds(2));

        // Force control, if available (sensed force is not zero)
        if (fabs(gripper.states().force) > std::numeric_limits<double>::epsilon()) {
            spdlog::info("Gripper running zero force control");
            gripper.Grasp(0);
            // Exit after 10 seconds
            std::this_thread::sleep_for(std::chrono::seconds(10));
        }

        // Finished, exit all threads
        gripper.Stop();
        g_finished = true;
        spdlog::info("Program finished");
        print_thread.join();

    } catch (const std::exception& e) {
        spdlog::error(e.what());
        return 1;
    }

    return 0;
}
