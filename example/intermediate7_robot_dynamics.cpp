/**
 * @example intermediate7_robot_dynamics.cpp
 * This tutorial runs the integrated dynamics engine to obtain robot Jacobian, mass matrix, and
 * gravity torques. Also checks reachability of a Cartesian pose.
 * @copyright Copyright (C) 2016-2026 Flexiv Ltd. All Rights Reserved.
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

        // Servo on the robot, make sure the E-stop is released
        spdlog::info("Servo on the robot ...");
        robot.ServoOn();

        // Wait for the robot to become operational
        while (!robot.operational()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        spdlog::info("Robot is now operational");

        // Move robot to home pose
        spdlog::info("Moving to home pose");
        robot.Home();

        // Robot Dynamics
        // =========================================================================================
        // Initialize dynamics engine
        rdk::Model model(robot);

        // Step dynamics engine 5 times
        for (size_t i = 0; i < 5; i++) {
            // Mark timer start point
            auto tic = std::chrono::high_resolution_clock::now();

            // Update robot model in dynamics engine
            const auto robot_states = robot.states();
            model.Update(robot_states.at(rdk::JointGroup::ALL).q,
                robot_states.at(rdk::JointGroup::ALL).dq);

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

        // Check IK feasibility for a nearby Cartesian pose on all available single-arm joint groups
        const auto& single_arm_groups = robot.info().single_arm_groups;
        if (single_arm_groups.empty()) {
            throw std::runtime_error("No single-arm joint group found on the connected robot");
        }

        const auto robot_states = robot.states();
        std::map<rdk::JointGroup, rdk::IKParams> ik_params_by_group;
        for (const auto& [group, _] : single_arm_groups) {
            auto pose_to_check = robot_states.at(group).tcp_pose;
            pose_to_check[0] += 0.1;
            spdlog::info("[{}] Checking IK feasibility of Cartesian pose [{}]",
                rdk::kJointGroupNames.at(group), rdk::utility::Arr2Str(pose_to_check));

            rdk::IKParams ik_params;
            ik_params.cartesian_pose = pose_to_check;
            ik_params.seed_q = robot_states.at(group).q;
            ik_params.free_orientation = false;
            ik_params_by_group[group] = ik_params;
        }

        // Print result
        auto result = model.SolveConstrainedIK(ik_params_by_group);
        spdlog::info("IK result success = {}", result.success);
        for (const auto& [group, q] : result.solved_q) {
            spdlog::info(
                "[{}] solved_q = [{}]", rdk::kJointGroupNames.at(group), rdk::utility::Vec2Str(q));
        }

    } catch (const std::exception& e) {
        spdlog::error(e.what());
        return 1;
    }

    return 0;
}
