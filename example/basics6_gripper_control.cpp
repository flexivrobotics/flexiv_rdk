/**
 * @example basics6_gripper_control.cpp
 * This tutorial does position and force (if available) control of grippers supported by Flexiv.
 * @copyright Copyright (C) 2016-2024 Flexiv Ltd. All Rights Reserved.
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

using namespace flexiv;

namespace {
/** Global flag: whether the gripper control tasks are finished */
std::atomic<bool> g_finished = {false};
}

/** @brief Print program usage help */
void PrintHelp()
{
    // clang-format off
    std::cout << "Required arguments: [robot_sn] [gripper_name]" << std::endl;
    std::cout << "    robot_sn: Serial number of the robot to connect to. Remove any space, for example: Rizon4s-123456" << std::endl;
    std::cout << "    gripper_name: Full name of the gripper to be controlled, can be found in Flexiv Elements -> Settings -> Device" << std::endl;
    std::cout << "Optional arguments: None" << std::endl;
    std::cout << std::endl;
    // clang-format on
}

/** @brief Print gripper states data @ 1Hz */
void PrintGripperStates(rdk::Gripper& gripper)
{
    while (!g_finished) {
        // Print all gripper states in JSON format using the built-in ostream operator overloading
        spdlog::info("Current gripper states:");
        std::cout << gripper.states() << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

int main(int argc, char* argv[])
{
    // Program Setup
    // =============================================================================================
    // Parse parameters
    if (argc < 3 || rdk::utility::ProgramArgsExistAny(argc, argv, {"-h", "--help"})) {
        PrintHelp();
        return 1;
    }
    // Serial number of the robot to connect to. Remove any space, for example: Rizon4s-123456
    std::string robot_sn = argv[1];
    std::string gripper_name = argv[2];

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
        // Instantiate gripper control interface
        rdk::Gripper gripper(robot);

        // Instantiate tool interface. Gripper is categorized as both a device and a tool. The
        // device attribute allows a gripper to be interactively controlled by the user; whereas the
        // tool attribute tells the robot to account for its mass properties and TCP location.
        rdk::Tool tool(robot);

        // Enable the specified gripper as a device. This is equivalent to enabling the specified
        // gripper in Flexiv Elements -> Settings -> Device
        spdlog::info("Enabling gripper [{}]", gripper_name);
        gripper.Enable(gripper_name);

        // Print parameters of the enabled gripper
        spdlog::info("Gripper params:");
        std::cout << std::fixed << std::setprecision(3) << "{\n"
                  << "name: " << gripper.params().name
                  << "\nmin_width: " << gripper.params().min_width
                  << "\nmax_width: " << gripper.params().max_width
                  << "\nmin_force: " << gripper.params().min_force
                  << "\nmax_force: " << gripper.params().max_force
                  << "\nmin_vel: " << gripper.params().min_vel
                  << "\nmax_vel: " << gripper.params().max_vel << "\n}" << std::endl;

        // Switch robot tool to gripper so the gravity compensation and TCP location is updated
        spdlog::info("Switching robot tool to [{}]", gripper_name);
        tool.Switch(gripper_name);

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
            gripper.Init();
            // User determines if the manual initialization is finished
            spdlog::info(
                "Triggered manual initialization, press Enter when the initialization is finished");
            std::cin.get();
            std::cin.get();
        } else {
            spdlog::error("Invalid choice");
            return 1;
        }

        // Start a separate thread to print gripper states
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
