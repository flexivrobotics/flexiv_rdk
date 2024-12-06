/**
 * @example basics3_primitive_execution.cpp
 * This tutorial executes several basic robot primitives (unit skills). For detailed documentation
 * on all available primitives, please see [Flexiv Primitives](https://www.flexiv.com/primitives/).
 * @copyright Copyright (C) 2016-2024 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <flexiv/rdk/robot.hpp>
#include <flexiv/rdk/utility.hpp>
#include <spdlog/spdlog.h>

#include <iostream>
#include <iomanip>
#include <thread>

using namespace flexiv;

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

        // Enable the robot, make sure the E-stop is released before enabling
        spdlog::info("Enabling robot ...");
        robot.Enable();

        // Wait for the robot to become operational
        while (!robot.operational()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        spdlog::info("Robot is now operational");

        // Execute Primitives
        // =========================================================================================
        // Switch to primitive execution mode
        robot.SwitchMode(rdk::Mode::NRT_PRIMITIVE_EXECUTION);

        // (1) Go to home pose
        // -----------------------------------------------------------------------------------------
        // All parameters of the "Home" primitive are optional, thus we can skip the parameters and
        // the default values will be used
        spdlog::info("Executing primitive: Home");

        // Send command to robot
        robot.ExecutePrimitive("Home", std::map<std::string, rdk::FlexivDataTypes> {});

        // Wait for reached target
        while (!std::get<int>(robot.primitive_states()["reachedTarget"])) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        // (2) Move robot joints to target positions
        // -----------------------------------------------------------------------------------------
        // Required parameters:
        //     target: final joint positions, unit: degrees
        //         {arm joint positions}, {external axis joint positions}
        // Optional parameters:
        //     waypoints: waypoints to pass before reaching the target
        //         (same format as above, but can be more than one)
        //     vel: TCP linear velocity, unit: m/s
        // Optional properties:
        //     lockExternalAxes: whether to allow the external axes to move or not
        spdlog::info("Executing primitive: MoveJ");

        // Send command to robot
        robot.ExecutePrimitive("MoveJ",
            {
                {"target", rdk::JPos({30, -45, 0, 90, 0, 40, 30}, {-50, 30})},
                {"waypoints",
                    std::vector<rdk::JPos> {rdk::JPos({10, -30, 10, 30, 10, 15, 10}, {-15, 10}),
                        rdk::JPos({20, -60, -10, 60, -10, 30, 20}, {-30, 20})}},
            },
            {
                {"lockExternalAxes", 0},
            });

        // Most primitives won't exit by themselves and require users to explicitly trigger
        // transitions based on specific primitive states. Here we check if the primitive state
        // [reachedTarget] becomes true and trigger the transition manually by sending a new
        // primitive command.
        while (!std::get<int>(robot.primitive_states()["reachedTarget"])) {
            // Print current primitive states
            spdlog::info("Current primitive states:");
            for (const auto& st : robot.primitive_states()) {
                std::cout << st.first << ": " << rdk::utility::FlexivTypes2Str(st.second);
                std::cout << std::endl;
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
        robot.ExecutePrimitive("MoveL",
            {
                {"target", rdk::Coord({0.65, -0.3, 0.2}, {180, 0, 180}, {"WORLD", "WORLD_ORIGIN"})},
                {"waypoints",
                    std::vector<rdk::Coord> {
                        rdk::Coord({0.45, 0.1, 0.2}, {180, 0, 180}, {"WORLD", "WORLD_ORIGIN"}),
                        rdk::Coord({0.45, -0.3, 0.2}, {180, 0, 180}, {"WORLD", "WORLD_ORIGIN"})}},
                {"vel", 0.6},
                {"zoneRadius", "Z50"},
            });

        // Wait for reached target
        while (!std::get<int>(robot.primitive_states()["reachedTarget"])) {
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
        robot.ExecutePrimitive(
            "MoveL", {
                         {"target", rdk::Coord({0.0, 0.0, 0.0}, targetEulerDeg, {"TRAJ", "START"})},
                         {"vel", 0.2},
                     });
        // Wait for reached target
        while (!std::get<int>(robot.primitive_states()["reachedTarget"])) {
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
