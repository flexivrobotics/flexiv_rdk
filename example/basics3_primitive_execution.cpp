/**
 * @example basics3_primitive_execution.cpp
 * This tutorial executes several basic robot primitives (unit skills). For detailed documentation
 * on all available primitives, please see [Flexiv Primitives](https://www.flexiv.com/primitives/).
 * @copyright Copyright (C) 2016-2023 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <flexiv/robot.h>
#include <flexiv/utility.h>
#include <spdlog/spdlog.h>

#include <iostream>
#include <thread>

/** @brief Print tutorial description */
void PrintDescription()
{
    std::cout << "This tutorial executes several basic robot primitives (unit skills). For "
                 "detailed documentation on all available primitives, please see [Flexiv "
                 "Primitives](https://www.flexiv.com/primitives/)."
              << std::endl
              << std::endl;
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
    spdlog::info("Tutorial description:");
    PrintDescription();

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
        robot.SwitchMode(flexiv::Mode::NRT_PRIMITIVE_EXECUTION);

        // (1) Go to home pose
        // -----------------------------------------------------------------------------------------
        // All parameters of the "Home" primitive are optional, thus we can skip the parameters and
        // the default values will be used
        spdlog::info("Executing primitive: Home");

        // Send command to robot
        robot.ExecutePrimitive("Home()");

        // Wait for the primitive to finish
        while (robot.busy()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        // (2) Move robot joints to target positions
        // -----------------------------------------------------------------------------------------
        // The required parameter <target> takes in 7 target joint positions. Unit: degrees
        spdlog::info("Executing primitive: MoveJ");

        // Send command to robot
        robot.ExecutePrimitive("MoveJ(target=30 -45 0 90 0 40 30)");

        // Wait for reached target
        while (flexiv::utility::ParsePtStates(robot.primitive_states(), "reachedTarget") != "1") {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        // (3) Move robot TCP to a target position in world (base) frame
        // -----------------------------------------------------------------------------------------
        // Required parameter:
        //   target: final target position
        //       [pos_x pos_y pos_z rot_x rot_y rot_z ref_frame ref_point]
        //       Unit: m, deg
        // Optional parameter:
        //   waypoints: waypoints to pass before reaching final target
        //       (same format as above, but can repeat for number of waypoints)
        //   maxVel: maximum TCP linear velocity
        //       Unit: m/s
        // NOTE: The rotations use Euler ZYX convention, rot_x means Euler ZYX angle around X axis
        spdlog::info("Executing primitive: MoveL");

        // Send command to robot
        robot.ExecutePrimitive(
            "MoveL(target=0.65 -0.3 0.2 180 0 180 WORLD WORLD_ORIGIN,waypoints=0.45 0.1 0.2 180 0 "
            "180 WORLD WORLD_ORIGIN : 0.45 -0.3 0.2 180 0 180 WORLD WORLD_ORIGIN, maxVel=0.2)");

        // The [Move] series primitive won't terminate itself, so we determine if the robot has
        // reached target location by checking the primitive state "reachedTarget = 1" in the list
        // of current primitive states, and terminate the current primitive manually by sending a
        // new primitive command.
        while (flexiv::utility::ParsePtStates(robot.primitive_states(), "reachedTarget") != "1") {
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
        auto targetEulerDeg = flexiv::utility::Rad2Deg(flexiv::utility::Quat2EulerZYX(targetQuat));

        // Send command to robot. This motion will hold current TCP position and only do TCP
        // rotation
        robot.ExecutePrimitive(
            "MoveL(target=0.0 0.0 0.0 " + flexiv::utility::Arr2Str(targetEulerDeg) + "TRAJ START)");

        // Wait for reached target
        while (flexiv::utility::ParsePtStates(robot.primitive_states(), "reachedTarget") != "1") {
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
