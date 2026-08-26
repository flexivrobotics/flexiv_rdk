/**
 * @example intermediate3_realtime_joint_torque_control.cpp
 * This tutorial runs real-time joint torque control to hold or sine-sweep all robot joints. An
 * outer position loop is used to generate joint torque commands. This outer position loop + inner
 * torque loop together is also known as an impedance controller.
 * @warning The impedance controller in this example is for demo purpose only and has no performance
 * guarantee. To use the robot's built-in high-performance joint impedance controller, please refer
 * to intermediate2_realtime_joint_impedance_control.cpp.
 * @copyright Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <flexiv/rdk/robot.hpp>
#include <flexiv/rdk/scheduler.hpp>
#include <flexiv/rdk/utility.hpp>

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

        // Target joint positions
        std::vector<double> target_pos(robot.info().DoF);

        // Set target position based on motion type
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

        // Run impedance control on all joints
        std::vector<double> target_torque(robot.info().DoF);
        for (size_t i = 0; i < target_torque.size(); ++i) {
            target_torque[i] = kImpedanceKp[i] * (target_pos[i] - robot.states().q[i])
                               - kImpedanceKd[i] * robot.states().dtheta[i];
        }

        // Send target joint torque to RDK server
        robot.StreamJointTorque(target_torque, true);

        loop_counter++;

    } catch (const std::exception& e) {
        std::cerr << "[error] " << e.what() << std::endl;
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
    std::cout << ">>> Tutorial description <<<\nThis tutorial runs real-time joint torque control "
                 "to hold or sine-sweep all robot joints. An outer position loop is used to "
                 "generate joint torque commands. This outer position loop + inner torque loop "
                 "together is also known as an impedance controller.\n"
              << std::endl;

    // Type of motion specified by user
    std::string motion_type = "";
    if (rdk::utility::ProgramArgsExist(argc, argv, "--hold")) {
        std::cout << "Robot holding current pose" << std::endl;
        motion_type = "hold";
    } else {
        std::cout << "Robot running joint sine-sweep" << std::endl;
        motion_type = "sine-sweep";
    }

    try {
        // RDK Initialization
        // =========================================================================================
        // Instantiate robot interface
        rdk::Robot robot(robot_sn);

        // Clear fault on the connected robot if any
        if (robot.fault()) {
            std::cerr << "[warn] Fault occurred on the connected robot, trying to clear ..."
                      << std::endl;
            // Try to clear the fault
            if (!robot.ClearFault()) {
                std::cerr << "[error] Fault cannot be cleared, exiting ..." << std::endl;
                return 1;
            }
            std::cout << "Fault on the connected robot is cleared" << std::endl;
        }

        // Enable the robot, make sure the E-stop is released before enabling
        std::cout << "Enabling robot ..." << std::endl;
        robot.Enable();

        // Wait for the robot to become operational
        while (!robot.operational()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        std::cout << "Robot is now operational" << std::endl;

        // Move robot to home pose
        std::cout << "Moving to home pose" << std::endl;
        robot.SwitchMode(rdk::Mode::NRT_PLAN_EXECUTION);
        robot.ExecutePlan("PLAN-Home");
        // Wait for the plan to finish
        while (robot.busy()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        // Real-time Joint Torque Control
        // =========================================================================================
        // Switch to real-time joint torque control mode
        robot.SwitchMode(rdk::Mode::RT_JOINT_TORQUE);

        // Set initial joint positions
        auto init_pos = robot.states().q;
        std::cout << "Initial joint positions set to: " << rdk::utility::Vec2Str(init_pos)
                  << std::endl;

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
        std::cerr << "[error] " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
