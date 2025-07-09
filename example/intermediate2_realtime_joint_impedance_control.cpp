/**
 * @example intermediate2_realtime_joint_impedance_control.cpp
 * This tutorial runs real-time joint impedance control to hold or sine-sweep all robot joints.
 * @copyright Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved.
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
    std::cout << "    robot_sn: Serial number of the robot to connect. Remove any space, e.g. Rizon4s-123456" << std::endl;
    std::cout << "Optional arguments: [--hold]" << std::endl;
    std::cout << "    --hold: robot holds current joint positions, otherwise do a sine-sweep" << std::endl;
    std::cout << std::endl;
    // clang-format on
}

/** @brief Callback function for realtime periodic task */
void PeriodicTask(
    rdk::Robot& robot, const std::string& motion_type, const std::vector<double>& init_pos)
{
    // Local periodic loop counter
    static unsigned int loop_counter = 0;

    try {
        // Monitor fault on the connected robot
        if (robot.fault()) {
            throw std::runtime_error(
                "PeriodicTask: Fault occurred on the connected robot, exiting ...");
        }

        // Initialize target vectors to hold position
        std::vector<double> target_pos(robot.info().DoF);
        std::vector<double> target_vel(robot.info().DoF);
        std::vector<double> target_acc(robot.info().DoF);

        // Set target vectors based on motion type
        if (motion_type == "hold") {
            target_pos = init_pos;
        } else if (motion_type == "sine-sweep") {
            for (size_t i = 0; i < target_pos.size(); ++i) {
                target_pos[i] = init_pos[i]
                                + kSineAmp * sin(2 * M_PI * kSineFreq * loop_counter * kLoopPeriod);
            }
        } else {
            throw std::invalid_argument(
                "PeriodicTask: Unknown motion type. Accepted motion types: hold, sine-sweep");
        }

        // Reduce stiffness to half of nominal values after 5 seconds
        if (loop_counter == 5000) {
            auto new_Kq = robot.info().K_q_nom;
            for (auto& v : new_Kq) {
                v *= 0.5;
            }
            robot.SetJointImpedance(new_Kq);
            spdlog::info("Joint stiffness set to [{}]", rdk::utility::Vec2Str(new_Kq));
        }

        // Reset impedance properties to nominal values after another 5 seconds
        if (loop_counter == 10000) {
            robot.SetJointImpedance(robot.info().K_q_nom);
            spdlog::info("Joint impedance properties are reset");
        }

        // Send commands
        robot.StreamJointPosition(target_pos, target_vel, target_acc);

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
        ">>> Tutorial description <<<\nThis tutorial runs real-time joint impedance control to "
        "hold or sine-sweep all robot joints.\n");

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

        // Real-time Joint Impedance Control
        // =========================================================================================
        // Switch to real-time joint impedance control mode
        robot.SwitchMode(rdk::Mode::RT_JOINT_IMPEDANCE);

        // Set initial joint positions
        auto init_pos = robot.states().q;
        spdlog::info("Initial joint positions set to: {}", rdk::utility::Vec2Str(init_pos));

        // Create real-time scheduler to run periodic tasks
        rdk::Scheduler scheduler;
        // Add periodic task with 1ms interval and highest applicable priority
        scheduler.AddTask(
            std::bind(PeriodicTask, std::ref(robot), std::ref(motion_type), std::ref(init_pos)),
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
