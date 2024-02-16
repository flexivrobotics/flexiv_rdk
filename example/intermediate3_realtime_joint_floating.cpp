/**
 * @example intermediate3_realtime_joint_floating.cpp
 * This tutorial runs real-time joint floating with gentle velocity damping, gravity compensation,
 * and soft protection against position limits. This example is ideal for verifying the system's
 * whole-loop real-timeliness, accuracy of the robot dynamics model, and joint torque control
 * performance. If everything works well, all joints should float smoothly.
 * @copyright Copyright (C) 2016-2023 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <flexiv/Robot.hpp>
#include <flexiv/Log.hpp>
#include <flexiv/Scheduler.hpp>
#include <flexiv/Utility.hpp>

#include <iostream>
#include <string>
#include <thread>
#include <atomic>

namespace {
/** Joint velocity damping gains for floating */
const std::array<double, flexiv::k_jointDOF> k_floatingDamping
    = {10.0, 10.0, 5.0, 5.0, 1.0, 1.0, 1.0};

/** Atomic signal to stop scheduler tasks */
std::atomic<bool> g_schedStop = {false};
}

/** @brief Print tutorial description */
void printDescription()
{
    std::cout << "This tutorial runs real-time joint floating with gentle velocity damping, "
                 "gravity compensation, and soft protection against position limits. This example "
                 "is ideal for verifying the system's whole-loop real-timeliness, accuracy of the "
                 "robot dynamics model, and joint torque control performance. If everything works "
                 "well, all joints should float smoothly."
              << std::endl
              << std::endl;
}

/** @brief Print program usage help */
void printHelp()
{
    // clang-format off
    std::cout << "Required arguments: [robot SN]" << std::endl;
    std::cout << "    robot SN: Serial number of the robot to connect to. "
                 "Remove any space, for example: Rizon4s-123456" << std::endl;
    std::cout << "Optional arguments: None" << std::endl;
    std::cout << std::endl;
    // clang-format on
}

/** @brief Callback function for realtime periodic task */
void periodicTask(flexiv::Robot& robot, flexiv::Log& log)
{
    try {
        // Monitor fault on the connected robot
        if (robot.isFault()) {
            throw std::runtime_error(
                "periodicTask: Fault occurred on the connected robot, exiting ...");
        }

        // Set 0 joint torques
        std::array<double, flexiv::k_jointDOF> targetTorque = {};

        // Add some velocity damping
        for (size_t i = 0; i < flexiv::k_jointDOF; ++i) {
            targetTorque[i] += -k_floatingDamping[i] * robot.states().dtheta[i];
        }

        // Send target joint torque to RDK server, enable gravity compensation and joint limits soft
        // protection
        robot.streamJointTorque(targetTorque, true, true);

    } catch (const std::exception& e) {
        log.error(e.what());
        g_schedStop = true;
    }
}

int main(int argc, char* argv[])
{
    // Program Setup
    // =============================================================================================
    // Logger for printing message with timestamp and coloring
    flexiv::Log log;

    // Parse parameters
    if (argc < 2 || flexiv::utility::programArgsExistAny(argc, argv, {"-h", "--help"})) {
        printHelp();
        return 1;
    }
    // Serial number of the robot to connect to. Remove any space, for example: Rizon4s-123456
    std::string robotSN = argv[1];

    // Print description
    log.info("Tutorial description:");
    printDescription();

    try {
        // RDK Initialization
        // =========================================================================================
        // Instantiate robot interface
        flexiv::Robot robot(robotSN);

        // Clear fault on the connected robot if any
        if (robot.isFault()) {
            log.warn("Fault occurred on the connected robot, trying to clear ...");
            // Try to clear the fault
            if (!robot.clearFault()) {
                log.error("Fault cannot be cleared, exiting ...");
                return 1;
            }
            log.info("Fault on the connected robot is cleared");
        }

        // Enable the robot, make sure the E-stop is released before enabling
        log.info("Enabling robot ...");
        robot.enable();

        // Wait for the robot to become operational
        while (!robot.isOperational()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        log.info("Robot is now operational");

        // Move robot to home pose
        log.info("Moving to home pose");
        robot.setMode(flexiv::Mode::NRT_PRIMITIVE_EXECUTION);
        robot.executePrimitive("Home()");

        // Wait for the primitive to finish
        while (robot.isBusy()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        // Real-time Joint Floating
        // =========================================================================================
        // Switch to real-time joint torque control mode
        robot.setMode(flexiv::Mode::RT_JOINT_TORQUE);

        // Create real-time scheduler to run periodic tasks
        flexiv::Scheduler scheduler;
        // Add periodic task with 1ms interval and highest applicable priority
        scheduler.addTask(std::bind(periodicTask, std::ref(robot), std::ref(log)), "HP periodic", 1,
            scheduler.maxPriority());
        // Start all added tasks
        scheduler.start();

        // Block and wait for signal to stop scheduler tasks
        while (!g_schedStop) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        // Received signal to stop scheduler tasks
        scheduler.stop();

    } catch (const std::exception& e) {
        log.error(e.what());
        return 1;
    }

    return 0;
}
