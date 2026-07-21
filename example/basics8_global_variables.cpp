/**
 * @example basics8_global_variables.cpp
 * This tutorial shows how to get and set global variables.
 * @copyright Copyright (C) 2016-2026 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <flexiv/rdk/robot.hpp>
#include <flexiv/rdk/utility.hpp>

#include <iostream>
#include <thread>

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
    std::cout << ">>> Tutorial description <<<\nThis tutorial shows how to get and set global "
                 "variables.\n"
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

        // Get existing global variables
        // =========================================================================================
        auto global_vars = robot.global_variables();
        if (global_vars.empty()) {
            std::cerr << "[warn] No global variables available" << std::endl;
            return 1;
        } else {
            std::cout << "Existing global variables and their original values:" << std::endl;
            for (const auto& var : global_vars) {
                std::cout << var.first << ": " << rdk::utility::FlexivTypes2Str(var.second)
                          << std::endl;
            }
        }

        // Set global variables
        // =========================================================================================
        ///@warning These specified global variables need to be created first using Flexiv Elements
        std::cout << "Setting new values to existing global variables" << std::endl;
        robot.SetGlobalVariables({
            {"test_bool", 1},
            {"test_int", 100},
            {"test_double", 100.123},
            {"test_string", "Flexiv"},
            {"test_int_vec", std::vector<int> {1, 2, 3}},
            {"test_double_vec", std::vector<double> {1.1, 2.2, 3.3}},
            {"test_string_vec", std::vector<std::string> {"Go", "Flexiv", "Go!"}},
            {"test_pose", std::vector<double> {0.1, -0.2, 0.3, -90, -45, 120}},
            {"test_coord", rdk::Coord({0.1, -0.2, 0.3}, {-90, -45, 120}, {"WORLD", "WORLD_ORIGIN"},
                               {1, 2, 3, 4, 5, 6, 7}, {10, 20, 0, 0, 0, 0})},
            {"test_coord_array",
                std::vector<rdk::Coord> {rdk::Coord({1, 2, 3}, {4, 5, 6}, {"WORK", "WorkCoord0"}),
                    rdk::Coord({10, 20, 30}, {40, 50, 60}, {"WORLD", "WORLD_ORIGIN"},
                        {1, 2, 3, 4, 5, 6, 7}, {10, 20, 0, 0, 0, 0}),
                    rdk::Coord({3, 2, 1}, {180, 0, 180}, {"WORLD", "WORLD_ORIGIN"})}},
        });

        // Get updated global variables
        // =========================================================================================
        global_vars = robot.global_variables();
        if (global_vars.empty()) {
            std::cerr << "[warn] No global variables available" << std::endl;
            return 1;
        } else {
            std::cout << "Updated global variables:" << std::endl;
            for (const auto& var : global_vars) {
                std::cout << var.first << ": " << rdk::utility::FlexivTypes2Str(var.second)
                          << std::endl;
            }
        }

        std::cout << "Program finished" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "[error] " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
