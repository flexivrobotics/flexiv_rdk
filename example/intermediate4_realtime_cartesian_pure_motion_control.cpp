/**
 * @example intermediate4_realtime_cartesian_pure_motion_control.cpp
 * This tutorial runs real-time Cartesian-space pure motion control to hold or sine-sweep the robot
 * TCP. A simple collision detection is also included.
 * @copyright Copyright (C) 2016-2024 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <flexiv/rdk/robot.hpp>
#include <flexiv/rdk/scheduler.hpp>
#include <flexiv/rdk/utility.hpp>
#include <spdlog/spdlog.h>

#include <iostream>
#include <cmath>
#include <thread>
#include <atomic>

namespace {
/** RT loop frequency [Hz] */
constexpr size_t kLoopFreq = 1000;

/** RT loop period [sec] */
constexpr double kLoopPeriod = 0.001;

/** TCP sine-sweep amplitude [m] */
constexpr double kSwingAmp = 0.1;

/** TCP sine-sweep frequency [Hz] */
constexpr double kSwingFreq = 0.3;

/** External TCP force threshold for collision detection, value is only for demo purpose [N] */
constexpr double kExtForceThreshold = 10.0;

/** External joint torque threshold for collision detection, value is only for demo purpose [Nm] */
constexpr double kExtTorqueThreshold = 5.0;

/** Atomic signal to stop scheduler tasks */
std::atomic<bool> g_stop_sched = {false};
}

/** @brief Print program usage help */
void PrintHelp()
{
    // clang-format off
    std::cout << "Required arguments: [robot SN]" << std::endl;
    std::cout << "    robot SN: Serial number of the robot to connect to. "
                 "Remove any space, for example: Rizon4s-123456" << std::endl;
    std::cout << "Optional arguments: [--hold] [--collision]" << std::endl;
    std::cout << "    --hold: robot holds current TCP pose, otherwise do a sine-sweep" << std::endl;
    std::cout << "    --collision: enable collision detection, robot will stop upon collision" << std::endl;
    std::cout << std::endl;
    // clang-format on
}

/** @brief Callback function for realtime periodic task */
void PeriodicTask(flexiv::rdk::Robot& robot,
    const std::array<double, flexiv::rdk::kPoseSize>& init_pose, bool enable_hold,
    bool enable_collision)
{
    // Local periodic loop counter
    static uint64_t loop_counter = 0;

    try {
        // Monitor fault on the connected robot
        if (robot.fault()) {
            throw std::runtime_error(
                "PeriodicTask: Fault occurred on the connected robot, exiting ...");
        }

        // Initialize target pose to initial pose
        auto target_pose = init_pose;

        // Sine-sweep TCP along Y axis
        if (!enable_hold) {
            target_pose[1] = init_pose[1]
                             + kSwingAmp * sin(2 * M_PI * kSwingFreq * loop_counter * kLoopPeriod);
        }
        // Otherwise robot TCP will hold at initial pose

        // Send command. Calling this method with only target pose input results in pure motion
        // control
        robot.StreamCartesianMotionForce(target_pose);

        // Do the following operations in sequence for every 20 seconds
        switch (loop_counter % (20 * kLoopFreq)) {
            // Online change preferred joint positions at 3 seconds
            case (3 * kLoopFreq): {
                std::vector<double> preferred_jnt_pos
                    = {0.938, -1.108, -1.254, 1.464, 1.073, 0.278, -0.658};
                robot.SetNullSpacePosture(preferred_jnt_pos);
                spdlog::info("Preferred joint positions set to: "
                             + flexiv::rdk::utility::Vec2Str(preferred_jnt_pos));
            } break;
            // Online change stiffness to half of nominal at 6 seconds
            case (6 * kLoopFreq): {
                auto new_K = robot.info().K_x_nom;
                for (auto& v : new_K) {
                    v *= 0.5;
                }
                robot.SetCartesianImpedance(new_K);
                spdlog::info(
                    "Cartesian stiffness set to: {}", flexiv::rdk::utility::Arr2Str(new_K));
            } break;
            // Online change to another preferred joint positions at 9 seconds
            case (9 * kLoopFreq): {
                std::vector<double> preferred_jnt_pos
                    = {-0.938, -1.108, 1.254, 1.464, -1.073, 0.278, 0.658};
                robot.SetNullSpacePosture(preferred_jnt_pos);
                spdlog::info("Preferred joint positions set to: "
                             + flexiv::rdk::utility::Vec2Str(preferred_jnt_pos));
            } break;
            // Online reset stiffness to nominal at 12 seconds
            case (12 * kLoopFreq): {
                robot.ResetCartesianImpedance();
                spdlog::info("Cartesian stiffness is reset");
            } break;
            // Online reset preferred joint positions to nominal at 14 seconds
            case (14 * kLoopFreq): {
                robot.ResetNullSpacePosture();
                spdlog::info("Preferred joint positions are reset");
            } break;
            // Online enable max contact wrench regulation at 16 seconds
            case (16 * kLoopFreq): {
                std::array<double, flexiv::rdk::kCartDoF> max_wrench
                    = {10.0, 10.0, 10.0, 2.0, 2.0, 2.0};
                robot.SetMaxContactWrench(max_wrench);
                spdlog::info(
                    "Max contact wrench set to: {}", flexiv::rdk::utility::Arr2Str(max_wrench));
            } break;
            // Disable max contact wrench regulation at 19 seconds
            case (19 * kLoopFreq): {
                robot.ResetMaxContactWrench();
                spdlog::info("Max contact wrench is reset");
            } break;
            default:
                break;
        }

        // Simple collision detection: stop robot if collision is detected from either end-effector
        // or robot body
        if (enable_collision) {
            bool collision_detected = false;
            Eigen::Vector3d ext_force = {robot.states().ext_wrench_in_world[0],
                robot.states().ext_wrench_in_world[1], robot.states().ext_wrench_in_world[2]};
            if (ext_force.norm() > kExtForceThreshold) {
                collision_detected = true;
            }
            for (const auto& v : robot.states().tau_ext) {
                if (fabs(v) > kExtTorqueThreshold) {
                    collision_detected = true;
                }
            }
            if (collision_detected) {
                robot.Stop();
                spdlog::warn("Collision detected, stopping robot and exit program ...");
                g_stop_sched = true;
            }
        }

        // Increment loop counter
        loop_counter++;

    } catch (const std::exception& e) {
        spdlog::error(e.what());
        g_stop_sched = true;
    }
}

int main(int argc, char* argv[])
{
    // Program Setup
    // =============================================================================================
    // Parse parameters
    if (argc < 2 || flexiv::rdk::utility::ProgramArgsExistAny(argc, argv, {"-h", "--help"})) {
        PrintHelp();
        return 1;
    }
    // Serial number of the robot to connect to. Remove any space, for example: Rizon4s-123456
    std::string robot_sn = argv[1];

    // Print description
    spdlog::info(
        ">>> Tutorial description <<<\nThis tutorial runs real-time Cartesian-space pure motion "
        "control to hold or sine-sweep the robot TCP. A simple collision detection is also "
        "included.");

    // Type of motion specified by user
    bool enable_hold = false;
    if (flexiv::rdk::utility::ProgramArgsExist(argc, argv, "--hold")) {
        spdlog::info("Robot holding current TCP pose");
        enable_hold = true;
    } else {
        spdlog::info("Robot running TCP sine-sweep");
    }

    // Whether to enable collision detection
    bool enable_collision = false;
    if (flexiv::rdk::utility::ProgramArgsExist(argc, argv, "--collision")) {
        spdlog::info("Collision detection enabled");
        enable_collision = true;
    } else {
        spdlog::info("Collision detection disabled");
    }

    try {
        // RDK Initialization
        // =========================================================================================
        // Instantiate robot interface
        flexiv::rdk::Robot robot(robot_sn);

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
        robot.SwitchMode(flexiv::rdk::Mode::NRT_PRIMITIVE_EXECUTION);
        robot.ExecutePrimitive("Home()");

        // Wait for the primitive to finish
        while (robot.busy()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        // Zero Force-torque Sensor
        // =========================================================================================
        // IMPORTANT: must zero force/torque sensor offset for accurate force/torque measurement
        robot.ExecutePrimitive("ZeroFTSensor()");

        // WARNING: during the process, the robot must not contact anything, otherwise the result
        // will be inaccurate and affect following operations
        spdlog::warn(
            "Zeroing force/torque sensors, make sure nothing is in contact with the robot");

        // Wait for primitive completion
        while (robot.busy()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        spdlog::info("Sensor zeroing complete");

        // Configure Motion Control
        // =========================================================================================
        // The Cartesian motion force modes do pure motion control out of the box, thus nothing
        // needs to be explicitly configured

        // NOTE: motion control always uses robot world frame, while force control can use
        // either world or TCP frame as reference frame

        // Start Pure Motion Control
        // =========================================================================================
        // Switch to real-time mode for continuous motion control
        robot.SwitchMode(flexiv::rdk::Mode::RT_CARTESIAN_MOTION_FORCE);

        // Set initial pose to current TCP pose
        auto init_pose = robot.states().tcp_pose;
        spdlog::info("Initial TCP pose set to [position 3x1, rotation (quaternion) 4x1]: "
                     + flexiv::rdk::utility::Arr2Str(init_pose));

        // Create real-time scheduler to run periodic tasks
        flexiv::rdk::Scheduler scheduler;
        // Add periodic task with 1ms interval and highest applicable priority
        scheduler.AddTask(std::bind(PeriodicTask, std::ref(robot), std::ref(init_pose), enable_hold,
                              enable_collision),
            "HP periodic", 1, scheduler.max_priority());
        // Start all added tasks
        scheduler.Start();

        // Block and wait for signal to stop scheduler tasks
        while (!g_stop_sched) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        // Received signal to stop scheduler tasks
        scheduler.Stop();

    } catch (const std::exception& e) {
        spdlog::error(e.what());
        return 1;
    }

    return 0;
}
