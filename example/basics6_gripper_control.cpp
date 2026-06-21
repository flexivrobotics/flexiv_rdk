/**
 * @example basics6_gripper_control.cpp
 * This tutorial does position and force (if available) control of grippers supported by Flexiv.
 * @copyright Copyright (C) 2016-2026 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <flexiv/rdk/robot.hpp>
#include <flexiv/rdk/gripper.hpp>
#include <flexiv/rdk/tool.hpp>
#include <flexiv/rdk/utility.hpp>
#include <spdlog/spdlog.h>

#include <iostream>
#include <iomanip>
#include <thread>
#include <atomic>
#include <limits>

using namespace flexiv;

namespace {
/** Global flag: whether the gripper control tasks are finished */
std::atomic<bool> g_finished = {false};
}

/** @brief Print program usage help */
void PrintHelp()
{
    // clang-format off
    std::cout << "Required arguments: [robot_sn] [gripper_device_name] [gripper_tool_name]" << std::endl;
    std::cout << "    robot_sn: Serial number of the robot to connect to. Remove any space, for example: Enlight-L-123456" << std::endl;
    std::cout << "    gripper_device_name: Full name of the device representing the gripper, can be found in Flexiv Elements->Settings->Device" << std::endl;
    std::cout << "    gripper_tool_name: Full name of the tool representing the gripper, can be found in Flexiv Elements->Settings->Tool" << std::endl;
    std::cout << "Optional arguments: None" << std::endl;
    std::cout << std::endl;
    // clang-format on
}

/** @brief Print gripper states data @ 1Hz */
void PrintGripperStates(rdk::Gripper& gripper)
{
    while (!g_finished) {
        const auto gripper_states = gripper.states();
        spdlog::info("Current gripper states:");
        for (const auto& [group, states] : gripper_states) {
            std::cout << "[" << rdk::kJointGroupNames.at(group) << "]\n" << states << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

int main(int argc, char* argv[])
{
    // Program Setup
    // =============================================================================================
    // Parse parameters
    if (argc < 4 || rdk::utility::ProgramArgsExistAny(argc, argv, {"-h", "--help"})) {
        PrintHelp();
        return 1;
    }
    // Serial number of the robot to connect to
    std::string robot_sn = argv[1];
    std::string gripper_device_name = argv[2];
    std::string gripper_tool_name = argv[3];

    // Print description
    spdlog::info(
        ">>> Tutorial description <<<\nThis tutorial does position and force (if available) "
        "control of grippers supported by Flexiv.\n");

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

        // Servo on the robot, make sure the E-stop is released
        spdlog::info("Servo on the robot ...");
        robot.ServoOn();

        // Wait for the robot to become operational
        while (!robot.operational()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        spdlog::info("Robot is now operational");

        // Gripper Control
        // =========================================================================================
        // Instantiate gripper control interface
        rdk::Gripper gripper(robot);

        // Instantiate tool interface. Gripper is categorized as both a device and a tool. The
        // device attribute allows a gripper to be interactively controlled by the user; whereas the
        // tool attribute tells the robot to account for its mass properties and TCP location.
        rdk::Tool tool(robot);

        // Grippers can only be assigned to single-arm joint groups
        const auto& single_arm_groups = robot.info().single_arm_groups;
        if (single_arm_groups.empty()) {
            throw std::runtime_error("No single-arm joint group found on the connected robot");
        }

        // Enable the specified gripper as a device. This is equivalent to enabling the specified
        // gripper in Flexiv Elements -> Settings -> Device
        spdlog::info("Enabling gripper device [{}] for all single-arm groups", gripper_device_name);
        for (const auto& [group, _] : single_arm_groups) {
            gripper.Enable(group, gripper_device_name);
        }

        // Print parameters of the enabled gripper
        spdlog::info("Gripper params:");
        const auto gripper_params = gripper.params();
        for (const auto& [group, params] : gripper_params) {
            std::cout << "[" << rdk::kJointGroupNames.at(group) << "]\n"
                      << std::fixed << std::setprecision(2) << "{\n"
                      << "name: " << params.name << "\nmin_width: " << params.min_width
                      << "\nmax_width: " << params.max_width << "\nmin_force: " << params.min_force
                      << "\nmax_force: " << params.max_force << "\nmin_vel: " << params.min_vel
                      << "\nmax_vel: " << params.max_vel << "\n}" << std::endl;
        }

        // Switch robot tool to gripper so the gravity compensation and TCP location is updated
        spdlog::info("Switching robot tool to [{}] for all single-arm groups", gripper_tool_name);
        for (const auto& [group, _] : single_arm_groups) {
            tool.Switch(group, gripper_tool_name);
        }

        // User needs to determine if this gripper requires manual initialization
        int choice = 0;
        spdlog::info(
            "Manually trigger initialization for the gripper now? Choose Yes if it's a 48v Grav "
            "gripper");
        std::cout << "[1] No, it has already initialized automatically when power on" << std::endl;
        std::cout << "[2] Yes, it does not initialize itself when power on" << std::endl;
        std::cin >> choice;

        // Trigger manual initialization based on choice
        if (choice == 1) {
            spdlog::info("Skipped manual initialization");
        } else if (choice == 2) {
            for (const auto& [group, _] : single_arm_groups) {
                gripper.Init(group);
            }
            // User determines if the manual initialization is finished
            spdlog::info(
                "Triggered manual initialization, press Enter when the initialization is finished "
                "to continue");
            std::cin.get();
            std::cin.get();
        } else {
            spdlog::error("Invalid choice");
            return 1;
        }

        // Start a separate thread to print gripper states. Use a scope guard so the thread is
        // always signaled and joined when leaving this scope, including via an exception thrown by
        // gripper.Move() etc. Otherwise the std::thread destructor would run while still joinable
        // and call std::terminate(), aborting before the exception reaches the catch block below.
        std::thread print_thread(PrintGripperStates, std::ref(gripper));
        auto join_print_thread = [&]() {
            g_finished = true;
            if (print_thread.joinable()) {
                print_thread.join();
            }
        };
        std::shared_ptr<void> print_thread_guard(nullptr, [&](void*) { join_print_thread(); });

        // Position control
        spdlog::info("Closing gripper");
        for (const auto& [group, params] : gripper_params) {
            gripper.Move(group, params.min_width, params.max_vel, 0.25 * params.max_force);
        }
        std::this_thread::sleep_for(std::chrono::seconds(2));
        spdlog::info("Opening gripper");
        for (const auto& [group, params] : gripper_params) {
            gripper.Move(group, params.max_width, params.max_vel, 0.25 * params.max_force);
        }
        std::this_thread::sleep_for(std::chrono::seconds(2));

        // Stop
        spdlog::info("Closing gripper");
        for (const auto& [group, params] : gripper_params) {
            gripper.Move(group, params.min_width, params.max_vel, 0.25 * params.max_force);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        spdlog::info("Stopping gripper");
        for (const auto& [group, _] : gripper_params) {
            gripper.Stop(group);
        }
        std::this_thread::sleep_for(std::chrono::seconds(2));
        spdlog::info("Closing gripper");
        for (const auto& [group, params] : gripper_params) {
            gripper.Move(group, params.min_width, params.max_vel, 0.25 * params.max_force);
        }
        std::this_thread::sleep_for(std::chrono::seconds(2));
        spdlog::info("Opening gripper");
        for (const auto& [group, params] : gripper_params) {
            gripper.Move(group, params.max_width, params.max_vel, 0.25 * params.max_force);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        spdlog::info("Stopping gripper");
        for (const auto& [group, _] : gripper_params) {
            gripper.Stop(group);
        }
        std::this_thread::sleep_for(std::chrono::seconds(2));

        // Force control, if available (sensed force is not zero)
        const auto gripper_states = gripper.states();
        bool has_force_control = false;
        for (const auto& [_, states] : gripper_states) {
            if (fabs(states.force) > std::numeric_limits<double>::epsilon()) {
                has_force_control = true;
                break;
            }
        }
        if (has_force_control) {
            spdlog::info("Gripper running zero force control");
            for (const auto& [group, _] : gripper_states) {
                gripper.Grasp(group, 0);
            }
            // Exit after 10 seconds
            std::this_thread::sleep_for(std::chrono::seconds(10));
        }

        // Finished, exit all threads
        for (const auto& [group, _] : gripper_states) {
            gripper.Stop(group);
        }
        spdlog::info("Program finished");
        // The print thread is signaled and joined by the scope guard above.

    } catch (const std::exception& e) {
        spdlog::error(e.what());
        return 1;
    }

    return 0;
}
