/**
 * @example intermediate2_realtime_joint_torque_control.cpp
 * This tutorial runs real-time joint torque control to hold or sine-sweep all robot joints. An
 * outer position loop is used to generate joint torque commands. This outer position loop + inner
 * torque loop together is also known as an impedance controller.
 * @copyright Copyright (C) 2016-2023 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <flexiv/robot.h>
#include <flexiv/log.h>
#include <flexiv/scheduler.h>
#include <flexiv/utility.h>

#include <iostream>
#include <string>
#include <cmath>
#include <thread>
#include <atomic>

namespace {
/** RT loop period [sec] */
constexpr double kLoopPeriod = 0.001;

/** Outer position loop (impedance) gains, values are only for demo purpose */
const std::array<double, flexiv::kJointDOF> kImpedanceKp
    = {3000.0, 3000.0, 800.0, 800.0, 200.0, 200.0, 200.0};
const std::array<double, flexiv::kJointDOF> kImpedanceKd = {80.0, 80.0, 40.0, 40.0, 8.0, 8.0, 8.0};

/** Sine-sweep trajectory amplitude and frequency */
constexpr double kSineAmp = 0.035;
constexpr double kSineFreq = 0.3;

/** Atomic signal to stop scheduler tasks */
std::atomic<bool> g_stop_sched = {false};
}

/** @brief Print tutorial description */
void PrintDescription()
{
    std::cout
        << "This tutorial runs real-time joint torque control to hold or sine-sweep all robot "
           "joints. An outer position loop is used to generate joint torque commands. This outer "
           "position loop + inner torque loop together is also known as an impedance controller."
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
    std::cout << "Optional arguments: [--hold]" << std::endl;
    std::cout << "    --hold: robot holds current joint positions, otherwise do a sine-sweep" << std::endl;
    std::cout << std::endl;
    // clang-format on
}

/** @brief Callback function for realtime periodic task */
void PeriodicTask(flexiv::Robot& robot, flexiv::Log& log, const std::string& motion_type,
    const std::array<double, flexiv::kJointDOF>& init_pos)
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
        std::array<double, flexiv::kJointDOF> target_pos = {};

        // Set target position based on motion type
        if (motion_type == "hold") {
            target_pos = init_pos;
        } else if (motion_type == "sine-sweep") {
            for (size_t i = 0; i < flexiv::kJointDOF; ++i) {
                target_pos[i] = init_pos[i]
                                + kSineAmp * sin(2 * M_PI * kSineFreq * loop_counter * kLoopPeriod);
            }
        } else {
            throw std::invalid_argument(
                "PeriodicTask: unknown motion type. Accepted motion types: hold, sine-sweep");
        }

        // Run impedance control on all joints
        std::array<double, flexiv::kJointDOF> target_torque = {};
        for (size_t i = 0; i < flexiv::kJointDOF; ++i) {
            target_torque[i] = kImpedanceKp[i] * (target_pos[i] - robot.states().q[i])
                               - kImpedanceKd[i] * robot.states().dtheta[i];
        }

        // Send target joint torque to RDK server
        robot.StreamJointTorque(target_torque, true);

        loop_counter++;

    } catch (const std::exception& e) {
        log.Error(e.what());
        g_stop_sched = true;
    }
}

int main(int argc, char* argv[])
{
    // Program Setup
    // =============================================================================================
    // Logger for printing message with timestamp and coloring
    flexiv::Log log;

    // Parse parameters
    if (argc < 2 || flexiv::utility::ProgramArgsExistAny(argc, argv, {"-h", "--help"})) {
        PrintHelp();
        return 1;
    }
    // Serial number of the robot to connect to. Remove any space, for example: Rizon4s-123456
    std::string robot_sn = argv[1];

    // Print description
    log.Info("Tutorial description:");
    PrintDescription();

    // Type of motion specified by user
    std::string motion_type = "";
    if (flexiv::utility::ProgramArgsExist(argc, argv, "--hold")) {
        log.Info("Robot holding current pose");
        motion_type = "hold";
    } else {
        log.Info("Robot running joint sine-sweep");
        motion_type = "sine-sweep";
    }

    try {
        // RDK Initialization
        // =========================================================================================
        // Instantiate robot interface
        flexiv::Robot robot(robot_sn);

        // Clear fault on the connected robot if any
        if (robot.fault()) {
            log.Warn("Fault occurred on the connected robot, trying to clear ...");
            // Try to clear the fault
            if (!robot.ClearFault()) {
                log.Error("Fault cannot be cleared, exiting ...");
                return 1;
            }
            log.Info("Fault on the connected robot is cleared");
        }

        // Enable the robot, make sure the E-stop is released before enabling
        log.Info("Enabling robot ...");
        robot.Enable();

        // Wait for the robot to become operational
        while (!robot.operational()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        log.Info("Robot is now operational");

        // Move robot to home pose
        log.Info("Moving to home pose");
        robot.SwitchMode(flexiv::Mode::NRT_PRIMITIVE_EXECUTION);
        robot.ExecutePrimitive("Home()");

        // Wait for the primitive to finish
        while (robot.busy()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        // Real-time Joint Torque Control
        // =========================================================================================
        // Switch to real-time joint torque control mode
        robot.SwitchMode(flexiv::Mode::RT_JOINT_TORQUE);

        // Set initial joint positions
        auto init_pos = robot.states().q;
        log.Info("Initial joint positions set to: " + flexiv::utility::Arr2Str(init_pos));

        // Create real-time scheduler to run periodic tasks
        flexiv::Scheduler scheduler;
        // Add periodic task with 1ms interval and highest applicable priority
        scheduler.AddTask(std::bind(PeriodicTask, std::ref(robot), std::ref(log),
                              std::ref(motion_type), std::ref(init_pos)),
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
        log.Error(e.what());
        return 1;
    }

    return 0;
}
