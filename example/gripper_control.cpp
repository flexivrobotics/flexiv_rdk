/**
 * @example gripper_control.cpp
 * Position and force control with grippers supported by Flexiv.
 * @copyright Copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <flexiv/Robot.hpp>
#include <flexiv/Exception.hpp>
#include <flexiv/Log.hpp>
#include <flexiv/Gripper.hpp>
#include <flexiv/Utility.hpp>

#include <iostream>
#include <string>
#include <thread>
#include <atomic>

namespace {
/** Global flag: whether the gripper control tasks are finished */
std::atomic<bool> g_isDone = {false};
}

/** Print gripper states data @ 1Hz */
void printGripperStates(flexiv::Gripper& gripper, flexiv::Log& log)
{
    // Data struct storing gripper states
    flexiv::GripperStates gripperStates;

    while (!g_isDone) {
        // Get the latest gripper states
        gripper.getGripperStates(gripperStates);

        // Print all gripper states in JSON format using the built-in ostream
        // operator overloading
        log.info("Current gripper states:");
        std::cout << gripperStates << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
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
    // Log object for printing message with timestamp and coloring
    flexiv::Log log;

    // Parse Parameters
    //=============================================================================
    if (argc < 2
        || flexiv::utility::programArgsExistAny(argc, argv, {"-h", "--help"})) {
        printHelp();
        return 1;
    }

    // Serial number of the robot to connect to. Remove any space, for example:
    // Rizon4s-123456
    std::string robotSN = argv[1];

    try {
        // RDK Initialization
        //=============================================================================
        // Instantiate robot interface
        flexiv::Robot robot(robotSN);

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

        // Enable the robot, make sure the E-stop is released before enabling
        log.info("Enabling robot ...");
        robot.enable();

        // Wait for the robot to become operational
        int secondsWaited = 0;
        while (!robot.isOperational()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            if (++secondsWaited == 10) {
                log.warn(
                    "Still waiting for robot to become operational, please "
                    "check that the robot 1) has no fault, 2) is booted "
                    "into Auto mode");
            }
        }
        log.info("Robot is now operational");

        // Gripper control is not available if the robot is in IDLE mode, so put
        // robot into some mode other than IDLE
        robot.setMode(flexiv::Mode::NRT_PLAN_EXECUTION);
        robot.executePlan("PLAN-Home");
        std::this_thread::sleep_for(std::chrono::seconds(1));

        // Application-specific Code
        //=============================================================================
        // Instantiate gripper
        flexiv::Gripper gripper(robot);

        // Thread for printing gripper states
        std::thread printThread(
            printGripperStates, std::ref(gripper), std::ref(log));

        // Position control test
        log.info("Closing gripper");
        gripper.move(0.01, 0.1, 20);
        std::this_thread::sleep_for(std::chrono::seconds(2));
        log.info("Opening gripper");
        gripper.move(0.09, 0.1, 20);
        std::this_thread::sleep_for(std::chrono::seconds(2));

        // Stop test
        log.info("Closing gripper");
        gripper.move(0.01, 0.1, 20);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        log.info("Stopping gripper");
        gripper.stop();
        std::this_thread::sleep_for(std::chrono::seconds(2));
        log.info("Closing gripper");
        gripper.move(0.01, 0.1, 20);
        std::this_thread::sleep_for(std::chrono::seconds(2));
        log.info("Opening gripper");
        gripper.move(0.09, 0.1, 20);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        log.info("Stopping gripper");
        gripper.stop();
        std::this_thread::sleep_for(std::chrono::seconds(2));

        // Force control test, if available
        flexiv::GripperStates gripperStates;
        gripper.getGripperStates(gripperStates);
        if (fabs(gripperStates.force)
            > std::numeric_limits<double>::epsilon()) {
            log.info("Gripper running zero force control");
            gripper.grasp(0);
            // Exit after 10 seconds
            std::this_thread::sleep_for(std::chrono::seconds(10));
        }

        // Finished, exit all threads
        gripper.stop();
        g_isDone = true;
        log.info("Program finished");
        printThread.join();

    } catch (const flexiv::Exception& e) {
        log.error(e.what());
        return 1;
    }

    return 0;
}
