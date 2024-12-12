/**
 * @example intermediate7_robot_dynamics.cpp
 * This tutorial runs the integrated dynamics engine to obtain robot Jacobian, mass matrix, and
 * gravity torques. Also checks reachability of a Cartesian pose.
 * @copyright Copyright (C) 2016-2024 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <flexiv/rdk/robot.hpp>
#include <flexiv/rdk/model.hpp>
#include <flexiv/rdk/utility.hpp>
#include <spdlog/spdlog.h>

#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>
#include <mutex>

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
        ">>> Tutorial description <<<\nThis tutorial runs the integrated dynamics engine to obtain "
        "robot Jacobian, mass matrix, and gravity torques. Also checks reachability of a Cartesian "
        "pose.\n");

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

        // Move robot to home pose
        spdlog::info("Moving to home pose");
        robot.SwitchMode(rdk::Mode::NRT_PLAN_EXECUTION);
        robot.ExecutePlan("PLAN-Home");
        // Wait for the plan to finish
        while (robot.busy()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        // Robot Dynamics
        // =========================================================================================
        // Initialize dynamics engine
        rdk::Model model(robot);

        // Step dynamics engine 5 times
        for (size_t i = 0; i < 5; i++) {
            // Mark timer start point
            auto tic = std::chrono::high_resolution_clock::now();

            // Update robot model in dynamics engine
            model.Update(robot.states().q, robot.states().dtheta);

            // Compute gravity vector
            auto g = model.g();

            // Compute mass matrix
            auto M = model.M();

            // Compute Jacobian
            auto J = model.J("flange");

            // Mark timer end point and get loop time
            auto toc = std::chrono::high_resolution_clock::now();
            auto computation_time
                = std::chrono::duration_cast<std::chrono::microseconds>(toc - tic).count();

            // Print time used to compute g, M, J
            spdlog::info("Computation time = {} us", computation_time);
            // Print gravity
            std::cout << "g = \n"
                      << std::fixed << std::setprecision(5) << g.transpose() << std::endl;
            // Print mass matrix
            std::cout << "M = \n" << std::fixed << std::setprecision(5) << M << std::endl;
            // Print Jacobian
            std::cout << "J = \n" << std::fixed << std::setprecision(5) << J << std::endl;
            std::cout << std::endl;
        }

        // Check reachability of a Cartesian pose based on current pose
        auto pose_to_check = robot.states().tcp_pose;
        pose_to_check[0] += 0.1;
        spdlog::info(
            "Checking reachability of Cartesian pose [{}]", rdk::utility::Arr2Str(pose_to_check));
        auto result = model.reachable(pose_to_check, robot.states().q, true);
        spdlog::info("Got a result: reachable = {}, IK solution = [{}]", result.first,
            rdk::utility::Vec2Str(result.second));

    } catch (const std::exception& e) {
        spdlog::error(e.what());
        return 1;
    }

    return 0;
}
