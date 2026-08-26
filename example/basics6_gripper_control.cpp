/**
 * @example basics6_gripper_control.cpp
 * This tutorial does position and force (if available) control of grippers supported by Flexiv.
 * @copyright Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <flexiv/rdk/robot.hpp>
#include <flexiv/rdk/gripper.hpp>
#include <flexiv/rdk/tool.hpp>
#include <flexiv/rdk/utility.hpp>

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
        std::cout << "Current gripper states:" << std::endl;
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
    std::cout << ">>> Tutorial description <<<\nThis tutorial does position and force (if "
                 "available) control of grippers supported by Flexiv.\n"
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

        // Enable the robot, make sure the E-Stop is released before enabling
        std::cout << "Enabling robot ..." << std::endl;
        robot.Enable();

        // Wait for the robot to become operational
        while (!robot.operational()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        std::cout << "Robot is now operational" << std::endl;

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
        std::cout << "Enabling gripper [" << gripper_name << "]" << std::endl;
        gripper.Enable(gripper_name);

        // Print parameters of the enabled gripper
        std::cout << "Gripper params:" << std::endl;
        std::cout << std::fixed << std::setprecision(3) << "{\n"
                  << "name: " << gripper.params().name
                  << "\nmin_width: " << gripper.params().min_width
                  << "\nmax_width: " << gripper.params().max_width
                  << "\nmin_force: " << gripper.params().min_force
                  << "\nmax_force: " << gripper.params().max_force
                  << "\nmin_vel: " << gripper.params().min_vel
                  << "\nmax_vel: " << gripper.params().max_vel << "\n}" << std::endl;

        // Switch robot tool to gripper so the gravity compensation and TCP location is updated
        std::cout << "Switching robot tool to [" << gripper_name << "]" << std::endl;
        tool.Switch(gripper_name);

        // User needs to determine if this gripper requires manual initialization
        int choice = 0;
        std::cout << "Manually trigger initialization for the gripper now? Choose Yes if it's a "
                     "48v Grav gripper"
                  << std::endl;
        std::cout << "[1] No, it has already initialized automatically when power on" << std::endl;
        std::cout << "[2] Yes, it does not initialize itself when power on" << std::endl;
        std::cin >> choice;

        // Trigger manual initialization based on choice
        if (choice == 1) {
            std::cout << "Skipped manual initialization" << std::endl;
        } else if (choice == 2) {
            gripper.Init();
            // User determines if the manual initialization is finished
            std::cout << "Triggered manual initialization, press Enter when the initialization is "
                         "finished to continue"
                      << std::endl;
            std::cin.get();
            std::cin.get();
        } else {
            std::cerr << "[error] Invalid choice" << std::endl;
            return 1;
        }

        // Start a separate thread to print gripper states
        std::thread print_thread(PrintGripperStates, std::ref(gripper));

        // Position control
        std::cout << "Closing gripper" << std::endl;
        gripper.Move(0.01, 0.1, 20);
        std::this_thread::sleep_for(std::chrono::seconds(2));
        std::cout << "Opening gripper" << std::endl;
        gripper.Move(0.09, 0.1, 20);
        std::this_thread::sleep_for(std::chrono::seconds(2));

        // Stop
        std::cout << "Closing gripper" << std::endl;
        gripper.Move(0.01, 0.1, 20);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std::cout << "Stopping gripper" << std::endl;
        gripper.Stop();
        std::this_thread::sleep_for(std::chrono::seconds(2));
        std::cout << "Closing gripper" << std::endl;
        gripper.Move(0.01, 0.1, 20);
        std::this_thread::sleep_for(std::chrono::seconds(2));
        std::cout << "Opening gripper" << std::endl;
        gripper.Move(0.09, 0.1, 20);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std::cout << "Stopping gripper" << std::endl;
        gripper.Stop();
        std::this_thread::sleep_for(std::chrono::seconds(2));

        // Force control, if available (sensed force is not zero)
        if (fabs(gripper.states().force) > std::numeric_limits<double>::epsilon()) {
            std::cout << "Gripper running zero force control" << std::endl;
            gripper.Grasp(0);
            // Exit after 10 seconds
            std::this_thread::sleep_for(std::chrono::seconds(10));
        }

        // Finished, exit all threads
        gripper.Stop();
        g_finished = true;
        std::cout << "Program finished" << std::endl;
        print_thread.join();

    } catch (const std::exception& e) {
        std::cerr << "[error] " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
