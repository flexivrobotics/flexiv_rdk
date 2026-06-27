/**
 * @example intermediate3_realtime_joint_torque_control.cpp
 * This tutorial runs real-time joint torque control to hold or sine-sweep all robot joints. An
 * outer position loop is used to generate joint torque commands. This outer position loop + inner
 * torque loop together is also known as an impedance controller.
 * @warning The impedance controller in this example is for demo purpose only and has no performance
 * guarantee. To use the robot's built-in high-performance joint impedance controller, please refer
 * to intermediate2_realtime_joint_impedance_control.cpp.
 * @copyright Copyright (C) 2016-2026 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <flexiv/rdk/robot.hpp>
#include <flexiv/rdk/scheduler.hpp>
#include <flexiv/rdk/utility.hpp>
#include <spdlog/spdlog.h>

#include <iostream>
#include <string>
#include <cmath>
#include <thread>
#include <atomic>

using namespace flexiv;

namespace {
/** RT loop period [sec] */
constexpr double kLoopPeriod = 0.001;

/** Outer position loop (impedance) gains, values are only for demo purpose */
const std::vector<double> kImpedanceKp = {3000.0, 3000.0, 800.0, 800.0, 200.0, 200.0, 200.0};
const std::vector<double> kImpedanceKd = {80.0, 80.0, 40.0, 40.0, 8.0, 8.0, 8.0};

/** Sine-sweep trajectory amplitude and frequency */
constexpr double kSineAmp = 0.035;
constexpr double kSineFreq = 0.3;

/** Atomic signal to stop scheduler tasks */
std::atomic<bool> g_stop_sched = {false};
}

/** @brief Print program usage help */
void PrintHelp()
{
    // clang-format off
    std::cout << "Required arguments: [robot_sn]" << std::endl;
    std::cout << "    robot_sn: Serial number of the robot to connect. Remove any space, e.g. Enlight-L-123456" << std::endl;
    std::cout << "Optional arguments: [--hold]" << std::endl;
    std::cout << "    --hold: robot holds current joint positions, otherwise do a sine-sweep" << std::endl;
    std::cout << std::endl;
    // clang-format on
}

/** @brief Callback function for realtime periodic task */
void PeriodicTask(rdk::Robot& robot, const std::string& motion_type,
    const std::map<rdk::JointGroup, std::vector<double>>& all_init_pos)
{
    // Local periodic loop counter
    static unsigned int loop_counter = 0;

    try {
        // Monitor fault on the connected robot
        if (robot.fault()) {
            throw std::runtime_error(
                "PeriodicTask: Fault occurred on the connected robot, exiting ...");
        }

        if ((motion_type != "hold") && (motion_type != "sine-sweep")) {
            throw std::invalid_argument(
                "PeriodicTask: Unknown motion type. Accepted motion types: hold, sine-sweep");
        }

        std::map<rdk::JointGroup, rdk::RtJointTorqueCmd> rt_cmds;
        const auto all_states = robot.states();
        for (const auto& [group, init_pos] : all_init_pos) {
            std::vector<double> target_pos(init_pos.size());
            if (motion_type == "hold") {
                target_pos = init_pos;
            } else {
                for (size_t i = 0; i < target_pos.size(); ++i) {
                    target_pos[i]
                        = init_pos[i]
                          + kSineAmp * sin(2 * M_PI * kSineFreq * loop_counter * kLoopPeriod);
                }
            }

            // Compute target joint torque using outer position loop (impedance)
            std::vector<double> target_torque(target_pos.size());
            // Set target torque for external axis to zero, as the demo impedance gains are defined
            // for only the arms
            if (group != rdk::JointGroup::EXT_AXIS) {
                const auto& states = all_states.at(group);
                for (size_t i = 0; i < target_torque.size(); ++i) {
                    target_torque[i] = kImpedanceKp[i] * (target_pos[i] - states.q[i])
                                       - kImpedanceKd[i] * states.dtheta[i];
                }
            }

            rt_cmds[group] = rdk::RtJointTorqueCmd(target_torque, true, true);
        }

        // Send target joint torque to RDK server
        robot.StreamJointTorque(rt_cmds);

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
    // Serial number of the robot to connect to
    std::string robot_sn = argv[1];

    // Print description
    spdlog::info(
        ">>> Tutorial description <<<\nThis tutorial runs real-time joint torque control to hold "
        "or sine-sweep all robot joints. An outer position loop is used to generate joint torque "
        "commands. This outer position loop + inner torque loop together is also known as an "
        "impedance controller.\n");

    // Type of motion specified by user
    std::string motion_type = "";
    if (rdk::utility::ProgramArgsExist(argc, argv, "--hold")) {
        spdlog::info("Robot holding current pose");
        motion_type = "hold";
    } else {
        spdlog::info("Robot running joint sine-sweep");
        motion_type = "sine-sweep";
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

        // Real-time Joint Torque Control
        // =========================================================================================
        // Direct joint control can be executed by single-arm joint groups and the external axis
        auto exe_groups = robot.info().single_arm_groups;
        if (exe_groups.empty()) {
            throw std::runtime_error("No single-arm joint group found on the connected robot");
        }
        // The external axis joint group (if it exists) also supports direct joint control
        if (robot.info().all_groups.contains(rdk::JointGroup::EXT_AXIS)) {
            exe_groups.emplace(
                rdk::JointGroup::EXT_AXIS, robot.info().all_groups.at(rdk::JointGroup::EXT_AXIS));
        }

        // Switch to real-time joint torque control mode
        robot.SwitchMode(rdk::Mode::RT_JOINT_TORQUE);

        // Set initial joint positions
        std::map<rdk::JointGroup, std::vector<double>> all_init_pos;
        const auto robot_states = robot.states();
        for (const auto& [group, _] : exe_groups) {
            all_init_pos[group] = robot_states.at(group).q;
            spdlog::info("[{}] Initial joint positions: {}", rdk::kJointGroupNames.at(group),
                rdk::utility::Vec2Str(all_init_pos.at(group)));
        }

        // Create real-time scheduler to run periodic tasks
        rdk::Scheduler scheduler;
        // Add periodic task with 1ms interval and highest applicable priority
        scheduler.AddTask(std::bind(PeriodicTask, std::ref(robot), std::ref(motion_type),
                              std::cref(all_init_pos)),
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
