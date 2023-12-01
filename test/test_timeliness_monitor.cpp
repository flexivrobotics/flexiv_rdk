/**
 * @test test_timeliness_monitor.cpp
 * A test to evaluate RDK's internal timeliness monitor on real-time modes. Bad communication or
 * insufficient real-time performance of the workstation PC will cause the monitor's timeliness
 * check to fail. A warning will be issued first, then if the check has failed too many times, the
 * RDK connection with the server will be closed. During this test, the robot will hold its position
 * using joint torque streaming mode.
 * @copyright Copyright (C) 2016-2023 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <flexiv/Robot.hpp>
#include <flexiv/Log.hpp>
#include <flexiv/Scheduler.hpp>
#include <flexiv/Utility.hpp>

#include <iostream>
#include <string>
#include <cmath>
#include <thread>
#include <atomic>

namespace {
/** Atomic signal to stop scheduler tasks */
std::atomic<bool> g_schedStop = {false};
}

// callback function for realtime periodic task
void periodicTask(
    flexiv::Robot& robot, flexiv::Log& log, const std::array<double, flexiv::k_jointDOF>& initPos)
{
    // Loop counter
    static unsigned int loopCounter = 0;

    try {
        // Monitor fault on robot server
        if (robot.isFault()) {
            throw std::runtime_error("periodicTask: Fault occurred on robot server, exiting ...");
        }
        // Hold position
        std::array<double, flexiv::k_jointDOF> targetVel = {};
        std::array<double, flexiv::k_jointDOF> targetAcc = {};
        robot.streamJointPosition(initPos, targetVel, targetAcc);

        if (loopCounter == 5000) {
            log.warn(">>>>> Adding simulated loop delay <<<<<");
        }
        // simulate prolonged loop time after 5 seconds
        else if (loopCounter > 5000) {
            std::this_thread::sleep_for(std::chrono::microseconds(995));
        }

        loopCounter++;

    } catch (const std::exception& e) {
        log.error(e.what());
        g_schedStop = true;
    }
}

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

int main(int argc, char* argv[])
{
    // log object for printing message with timestamp and coloring
    flexiv::Log log;

    // Parse Parameters
    //==============================================================================================
    if (argc < 2 || flexiv::utility::programArgsExistAny(argc, argv, {"-h", "--help"})) {
        printHelp();
        return 1;
    }

    // Serial number of the robot to connect to. Remove any space, for example: Rizon4s-123456
    std::string robotSN = argv[1];

    try {
        // RDK Initialization
        //==========================================================================================
        // Instantiate robot interface
        flexiv::Robot robot(robotSN);

        // create data struct for storing robot states
        flexiv::RobotStates robotStates;

        // Clear fault on robot server if any
        if (robot.isFault()) {
            log.warn("Fault occurred on robot server, trying to clear ...");
            // Try to clear the fault
            robot.clearFault();
            std::this_thread::sleep_for(std::chrono::seconds(2));
            // Check again
            if (robot.isFault()) {
                log.error("Fault cannot be cleared, exiting ...");
                return 1;
            }
            log.info("Fault on robot server is cleared");
        }

        // enable the robot, make sure the E-stop is released before enabling
        log.info("Enabling robot ...");
        robot.enable();

        // Wait for the robot to become operational
        while (!robot.isOperational()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        log.info("Robot is now operational");

        // set mode after robot is operational
        robot.setMode(flexiv::Mode::RT_JOINT_POSITION);

        // Set initial joint positions
        robot.getRobotStates(robotStates);
        auto initPos = robotStates.q;
        log.info("Initial joint positions set to: " + flexiv::utility::arr2Str(initPos));
        log.warn(">>>>> Simulated loop delay will be added after 5 seconds <<<<<");

        // Periodic Tasks
        //==========================================================================================
        flexiv::Scheduler scheduler;
        // Add periodic task with 1ms interval and highest applicable priority
        scheduler.addTask(
            std::bind(periodicTask, std::ref(robot), std::ref(log), std::ref(initPos)),
            "HP periodic", 1, scheduler.maxPriority());
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
