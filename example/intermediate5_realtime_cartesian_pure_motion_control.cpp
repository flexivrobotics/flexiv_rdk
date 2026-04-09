/**
 * @example intermediate5_realtime_cartesian_pure_motion_control.cpp
 * This tutorial runs real-time Cartesian-space pure motion control to hold or sine-sweep the robot
 * TCP. A simple collision detection is also included.
 * @copyright Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved.
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
#include <algorithm>

using namespace flexiv;

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
    std::cout << "Required arguments: [robot_sn]" << std::endl;
    std::cout << "    robot_sn: Serial number of the robot to connect. Remove any space, e.g. Rizon4s-123456" << std::endl;
    std::cout << "Optional arguments: [--hold] [--collision]" << std::endl;
    std::cout << "    --hold: robot holds current TCP pose, otherwise do a sine-sweep" << std::endl;
    std::cout << "    --collision: enable collision detection, robot will stop upon collision" << std::endl;
    std::cout << std::endl;
    // clang-format on
}

/** @brief Callback function for realtime periodic task */
void PeriodicTask(rdk::Robot& robot,
    const std::map<rdk::JointGroup, std::array<double, rdk::kPoseSize>>& all_init_pose,
    const std::map<rdk::JointGroup, std::vector<double>>& all_init_q, bool enable_hold,
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

        std::map<rdk::JointGroup, rdk::RtCartesianCmd> rt_cmds;
        for (const auto& [group, init_pose] : all_init_pose) {
            auto target_pose = init_pose;

            // Sine-sweep TCP along Y axis
            if (!enable_hold) {
                target_pose[1]
                    = init_pose[1]
                      + kSwingAmp * sin(2 * M_PI * kSwingFreq * loop_counter * kLoopPeriod);
            }
            // Otherwise robot TCP will hold at initial pose

            rt_cmds[group] = rdk::RtCartesianCmd(target_pose);
        }

        // Send command. Calling this method with only target pose input results in pure motion
        // control
        robot.StreamCartesianMotionForce(rt_cmds);

        // Do the following operations in sequence for every 20 seconds
        switch (loop_counter % (20 * kLoopFreq)) {
            // Online change reference joint positions at 3 seconds
            case (3 * kLoopFreq): {
                const std::vector<double> ref_q
                    = {0.938, -1.108, -1.254, 1.464, 1.073, 0.278, -0.658};
                for (const auto& [group, _] : all_init_pose) {
                    robot.SetNullSpacePosture(group, ref_q);
                }
                spdlog::info("Reference joint positions updated for all groups");
            } break;
            // Online change stiffness to half of nominal at 6 seconds
            case (6 * kLoopFreq): {
                auto new_K = robot.info().K_x_nom;
                for (auto& v : new_K) {
                    v *= 0.5;
                }
                for (const auto& [group, _] : all_init_pose) {
                    robot.SetCartesianImpedance(group, new_K);
                }
                spdlog::info("Cartesian stiffness set to: {}", rdk::utility::Arr2Str(new_K));
            } break;
            // Online change to another reference joint positions at 9 seconds
            case (9 * kLoopFreq): {
                const std::vector<double> ref_q
                    = {-0.938, -1.108, 1.254, 1.464, -1.073, 0.278, 0.658};
                for (const auto& [group, _] : all_init_pose) {
                    robot.SetNullSpacePosture(group, ref_q);
                }
                spdlog::info("Reference joint positions updated for all groups");
            } break;
            // Online reset impedance properties to nominal at 12 seconds
            case (12 * kLoopFreq): {
                for (const auto& [group, _] : all_init_pose) {
                    robot.SetCartesianImpedance(group, robot.info().K_x_nom);
                }
                spdlog::info("Cartesian impedance properties are reset");
            } break;
            // Online reset reference joint positions to nominal at 14 seconds
            case (14 * kLoopFreq): {
                for (const auto& [group, init_q] : all_init_q) {
                    robot.SetNullSpacePosture(group, init_q);
                }
                spdlog::info("Reference joint positions are reset");
            } break;
            // Online enable max contact wrench regulation at 16 seconds
            case (16 * kLoopFreq): {
                std::array<double, rdk::kCartDoF> max_wrench = {10.0, 10.0, 10.0, 2.0, 2.0, 2.0};
                for (const auto& [group, _] : all_init_pose) {
                    robot.SetMaxContactWrench(group, max_wrench);
                }
                spdlog::info("Max contact wrench set to: {}", rdk::utility::Arr2Str(max_wrench));
            } break;
            // Disable max contact wrench regulation at 19 seconds
            case (19 * kLoopFreq): {
                std::array<double, rdk::kCartDoF> inf;
                inf.fill(std::numeric_limits<double>::infinity());
                for (const auto& [group, _] : all_init_pose) {
                    robot.SetMaxContactWrench(group, inf);
                }
                spdlog::info("Max contact wrench regulation is disabled");
            } break;
            default:
                break;
        }

        // Simple collision detection: stop robot if collision is detected from either end-effector
        // or robot body
        if (enable_collision) {
            bool collision_detected = false;
            for (const auto& [_, states] : robot.states()) {
                Eigen::Vector3d ext_force
                    = {states.tcp_wrench[0], states.tcp_wrench[1], states.tcp_wrench[2]};
                if (ext_force.norm() > kExtForceThreshold) {
                    collision_detected = true;
                }
                for (const auto& v : states.tau_ext) {
                    if (fabs(v) > kExtTorqueThreshold) {
                        collision_detected = true;
                    }
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
    if (argc < 2 || rdk::utility::ProgramArgsExistAny(argc, argv, {"-h", "--help"})) {
        PrintHelp();
        return 1;
    }
    // Serial number of the robot to connect to. Remove any space, for example: Rizon4s-123456
    std::string robot_sn = argv[1];

    // Print description
    spdlog::info(
        ">>> Tutorial description <<<\nThis tutorial runs real-time Cartesian-space pure motion "
        "control to hold or sine-sweep the robot TCP. A simple collision detection is also "
        "included.\n");

    // Type of motion specified by user
    bool enable_hold = false;
    if (rdk::utility::ProgramArgsExist(argc, argv, "--hold")) {
        spdlog::info("Robot holding current TCP pose");
        enable_hold = true;
    } else {
        spdlog::info("Robot running TCP sine-sweep");
    }

    // Whether to enable collision detection
    bool enable_collision = false;
    if (rdk::utility::ProgramArgsExist(argc, argv, "--collision")) {
        spdlog::info("Collision detection enabled");
        enable_collision = true;
    } else {
        spdlog::info("Collision detection disabled");
    }

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

        // Zero Force-torque Sensor
        // =========================================================================================
        robot.SwitchMode(rdk::Mode::NRT_PRIMITIVE_EXECUTION);
        // IMPORTANT: must zero force/torque sensor offset for accurate force/torque measurement
        std::map<rdk::JointGroup, rdk::PrimitiveArgs> pt_args;
        for (const auto& group : robot.groups()) {
            pt_args[group] = rdk::PrimitiveArgs("ZeroFTSensor", {});
        }
        robot.ExecutePrimitive(pt_args);

        // WARNING: during the process, the robot must not contact anything, otherwise the result
        // will be inaccurate and affect following operations
        spdlog::warn(
            "Zeroing force/torque sensors, make sure nothing is in contact with the robot");

        // Wait for primitive to finish
        while (!std::all_of(
            robot.primitive_states().begin(), robot.primitive_states().end(), [](const auto& kv) {
                return std::get<int>(kv.second.names_and_values.at("terminated"));
            })) {
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
        robot.SwitchMode(rdk::Mode::RT_CARTESIAN_MOTION_FORCE);

        // Set all Cartesian axis(s) to motion control
        for (const auto& group : robot.groups()) {
            robot.SetForceControlAxis(
                group, std::array<bool, rdk::kCartDoF> {false, false, false, false, false, false});
        }

        // Save initial poses and joint positions
        std::map<rdk::JointGroup, std::array<double, rdk::kPoseSize>> all_init_pose;
        std::map<rdk::JointGroup, std::vector<double>> all_init_q;
        for (const auto& [group, states] : robot.states()) {
            all_init_pose[group] = states.tcp_pose;
            all_init_q[group] = states.q;
        }

        // Create real-time scheduler to run periodic tasks
        rdk::Scheduler scheduler;
        // Add periodic task with 1ms interval and highest applicable priority
        scheduler.AddTask(std::bind(PeriodicTask, std::ref(robot), std::cref(all_init_pose),
                              std::cref(all_init_q), enable_hold, enable_collision),
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
