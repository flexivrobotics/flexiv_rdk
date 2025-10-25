/**
 * @test test_repeated_instantiation.cpp
 * A test to repeatedly construct and destruct rdk::Robot instance.
 * @copyright Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <flexiv/rdk/robot.hpp>
#include <flexiv/rdk/utility.hpp>
#include <spdlog/spdlog.h>

#include <iostream>

using namespace flexiv;

/** @brief Print program usage help */
void PrintHelp()
{
    // clang-format off
    std::cout << "Required arguments: [robot_sn] [test_cycles]" << std::endl;
    std::cout << "    robot_sn: Serial number of the robot to connect. Remove any space, e.g. Rizon4s-123456" << std::endl;
    std::cout << "    test_cycles: Number of cycles to repeat the instantiation" << std::endl;
    std::cout << "Optional arguments: None" << std::endl;
    std::cout << std::endl;
    // clang-format on
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

    // Number of cycles
    unsigned int test_cycles = std::stoi(argv[2]);

    try {
        for (size_t i = 1; i <= test_cycles; i++) {
            spdlog::info("Test cycle: {}/{}", i, test_cycles);

            // Instantiate robot interface
            rdk::Robot robot(robot_sn);

            // Enable the robot, make sure the E-stop is released before enabling
            spdlog::info("Enabling robot ...");
            robot.Enable();

            // Wait for the robot to become operational
            while (!robot.operational()) {
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
            spdlog::info("Robot is now operational");
        }

    } catch (const std::exception& e) {
        spdlog::error(e.what());
        return 1;
    }

    return 0;
}
