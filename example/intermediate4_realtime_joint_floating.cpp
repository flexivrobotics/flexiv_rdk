/**
 * @example intermediate4_realtime_joint_floating.cpp
 * This tutorial runs real-time joint floating with gentle velocity damping, gravity compensation,
 * and soft protection against position limits. This example is ideal for verifying the system's
 * whole-loop real-timeliness, accuracy of the robot dynamics model, and joint torque control
 * performance. If everything works well, all joints should float smoothly.
 * @copyright Copyright (C) 2016-2026 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <flexiv/rdk/robot.hpp>
#include <flexiv/rdk/scheduler.hpp>
#include <flexiv/rdk/utility.hpp>
#include <spdlog/spdlog.h>

#include <iostream>
#include <string>
#include <thread>
#include <atomic>

using namespace flexiv;

namespace {
/** Joint velocity damping gains for floating */
const std::vector<double> kFloatingDamping = {10.0, 10.0, 5.0, 5.0, 1.0, 1.0, 1.0};

/** Atomic signal to stop scheduler tasks */
std::atomic<bool> g_stop_sched = {false};
}

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

/** @brief Callback function for realtime periodic task */
void PeriodicTask(rdk::Robot& robot, const std::map<rdk::JointGroup, std::string>& exe_groups)
{
    try {
        // Monitor fault on the connected robot
        if (robot.fault()) {
            throw std::runtime_error(
                "PeriodicTask: Fault occurred on the connected robot, exiting ...");
        }

        std::map<rdk::JointGroup, rdk::RtJointTorqueCmd> rt_cmds;
        const auto all_states = robot.states();
        for (const auto& [group, _] : exe_groups) {
            const auto& states = all_states.at(group);
            std::vector<double> target_torque(states.q.size());

            // Don't add damping for external axis
            if (group != rdk::JointGroup::EXT_AXIS) {
                // Add some velocity damping
                for (size_t i = 0; i < target_torque.size(); ++i) {
                    target_torque[i] += -kFloatingDamping[i] * states.dtheta[i];
                }
            }

            rt_cmds[group] = rdk::RtJointTorqueCmd(target_torque, true, true);
        }

        // Send target joint torque to RDK server, enable gravity compensation and joint limits soft
        // protection
        robot.StreamJointTorque(rt_cmds);

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
    // Serial number of the robot to connect to
    std::string robot_sn = argv[1];

    // Print description
    spdlog::info(
        ">>> Tutorial description <<<\nThis tutorial runs real-time joint floating with gentle "
        "velocity damping, gravity compensation, and soft protection against position limits. This "
        "example is ideal for verifying the system's whole-loop real-timeliness, accuracy of the "
        "robot dynamics model, and joint torque control performance. If everything works well, all "
        "joints should float smoothly.\n");

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

        // Real-time Joint Floating
        // =========================================================================================
        // Direct joint control can be executed by single-arm joint groups and the external axis
        auto exe_groups = robot.info().single_arm_groups;
        if (exe_groups.empty()) {
            throw std::runtime_error("No single-arm joint group found on the connected robot");
        }
        // The external axis joint group (if exists) also supports direct joint control
        if (robot.info().all_groups.contains(rdk::JointGroup::EXT_AXIS)) {
            exe_groups.emplace(
                rdk::JointGroup::EXT_AXIS, robot.info().all_groups.at(rdk::JointGroup::EXT_AXIS));
        }

        // Switch to real-time joint torque control mode
        robot.SwitchMode(rdk::Mode::RT_JOINT_TORQUE);

        // Create real-time scheduler to run periodic tasks
        rdk::Scheduler scheduler;
        // Add periodic task with 1ms interval and highest applicable priority
        scheduler.AddTask(std::bind(PeriodicTask, std::ref(robot), std::cref(exe_groups)),
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
