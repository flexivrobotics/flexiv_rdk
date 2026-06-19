/**
 * @example basics3_primitive_execution.cpp
 * This tutorial executes several basic robot primitives (unit skills). For detailed documentation
 * on all available primitives, please see [Flexiv Primitives](https://www.flexiv.com/primitives/).
 * @copyright Copyright (C) 2016-2026 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <flexiv/rdk/robot.hpp>
#include <flexiv/rdk/utility.hpp>
#include <spdlog/spdlog.h>

#include <iostream>
#include <iomanip>
#include <thread>
#include <algorithm>

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
        ">>> Tutorial description <<<\nThis tutorial executes several basic robot primitives (unit "
        "skills). For detailed documentation on all available primitives, please see [Flexiv "
        "Primitives](https://www.flexiv.com/primitives/).\n");

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

        // Execute Primitives
        // =========================================================================================
        // Primitives can only be executed on single-arm joint groups
        const auto& single_arm_groups = robot.info().single_arm_groups;
        if (single_arm_groups.empty()) {
            throw std::runtime_error("No single-arm joint group found on the connected robot");
        }

        // (1) Move robot to home pose
        // -----------------------------------------------------------------------------------------
        spdlog::info("Moving to home pose");
        robot.Home();

        // Switch to primitive execution mode
        robot.SwitchMode(rdk::Mode::NRT_PRIMITIVE_EXECUTION);

        // (2) Move robot joints to target positions
        // -----------------------------------------------------------------------------------------
        // Required parameters:
        //     target: final joint positions, unit: degrees
        //         {arm joint positions}, {external axis joint positions}
        // Optional parameters:
        //     waypoints: waypoints to pass before reaching the target
        //         (same format as above, but can be more than one)
        //     vel: TCP linear velocity, unit: m/s
        spdlog::info("Executing primitive: MoveJ");

        // Send command to robot
        std::map<rdk::JointGroup, rdk::PrimitiveArgs> pt_args;
        for (const auto& [group, _] : single_arm_groups) {
            pt_args[group] = rdk::PrimitiveArgs("MoveJ",
                {
                    {"target", rdk::JPos({30, -45, 0, 90, 0, 40, 30}, {-50, 30})},
                    {"waypoints",
                        std::vector<rdk::JPos> {rdk::JPos({10, -30, 10, 30, 10, 15, 10}, {-15, 10}),
                            rdk::JPos({20, -60, -10, 60, -10, 30, 20}, {-30, 20})}},
                });
        }
        robot.ExecutePrimitive(pt_args);

        // Most primitives won't exit by themselves and require users to explicitly trigger
        // transitions based on specific primitive states. Here we check if the primitive state
        // [reachedTarget] becomes true and trigger the transition manually by sending a new
        // primitive command.
        while (true) {
            const auto primitive_states = robot.primitive_states();
            if (rdk::utility::PrimitiveStateTrueForGroups(
                    primitive_states, "reachedTarget")) {
                break;
            }

            // Print current primitive states
            spdlog::info("Current primitive states:");
            for (const auto& [group, pt_states] : primitive_states) {
                std::cout << rdk::kJointGroupNames.at(group) << ":" << std::endl;
                std::cout << "primitiveName: " << pt_states.pt_name << std::endl;
                for (const auto& [name, value] : pt_states.names_and_values) {
                    std::cout << name << ": " << rdk::utility::FlexivTypes2Str(value);
                    std::cout << std::endl;
                }
            }
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        // (3) Move robot TCP to a target pose in world (base) frame
        // -----------------------------------------------------------------------------------------
        // Required parameters:
        //     target: final TCP pose, unit: m and degrees
        //         {pos_x, pos_y, pos_z}, {rot_x, rot_y, rot_z}, {ref_frame, ref_point}
        // Optional parameters:
        //     waypoints: waypoints to pass before reaching the target
        //         (same format as above, but can be more than one)
        //     vel: TCP linear velocity, unit: m/s
        // NOTE: The rotations use Euler ZYX convention, rot_x means Euler ZYX angle around X axis
        spdlog::info("Executing primitive: MoveL");

        // Send command to robot
        pt_args.clear();
        for (const auto& [group, _] : single_arm_groups) {
            pt_args[group] = rdk::PrimitiveArgs("MoveL",
                {{"target",
                     rdk::Coord({0.3, -0.1, 0.2}, {160, 20, 180}, {"WORLD", "WORLD_ORIGIN"})},
                    {"waypoints",
                        std::vector<rdk::Coord> {rdk::Coord({0.1, 0.1, 0.1}, {-160, -20, 180},
                                                     {"WORLD", "WORLD_ORIGIN"}),
                            rdk::Coord({0.3, 0.2, 0.1}, {180, 0, 180}, {"WORLD", "WORLD_ORIGIN"})}},

                    {"vel", 0.6}, {"zoneRadius", "Z50"}});
        }
        robot.ExecutePrimitive(pt_args);

        // Wait for reached target
        while (!rdk::utility::PrimitiveStateTrueForGroups(robot.primitive_states(), "reachedTarget")) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        // (4) Another MoveL that uses TCP frame
        // -----------------------------------------------------------------------------------------
        // In this example the reference frame is changed from WORLD::WORLD_ORIGIN to TRAJ::START,
        // which represents the current TCP frame
        spdlog::info("Executing primitive: MoveL");

        // Example to convert target quaternion [w,x,y,z] to Euler ZYX using utility functions
        std::array<double, 4> targetQuat = {0.9185587, 0.1767767, 0.3061862, 0.1767767};
        // ZYX = [30, 30, 30] degrees
        auto targetEulerDeg = rdk::utility::Rad2Deg(rdk::utility::Quat2EulerZYX(targetQuat));

        // Send command to robot. This motion will hold current TCP position and only do rotation
        pt_args.clear();
        for (const auto& [group, _] : single_arm_groups) {
            pt_args[group] = rdk::PrimitiveArgs("MoveL",
                {
                    {"target", rdk::Coord({0.0, 0.0, 0.0}, targetEulerDeg, {"TRAJ", "START"})},
                    {"vel", 0.2},
                });
        }
        robot.ExecutePrimitive(pt_args);
        // Wait for reached target
        while (!rdk::utility::PrimitiveStateTrueForGroups(robot.primitive_states(), "reachedTarget")) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        // All done, stop robot and put into IDLE mode
        robot.Stop();

    } catch (const std::exception& e) {
        spdlog::error(e.what());
        return 1;
    }

    return 0;
}
