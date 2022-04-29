/**
 * @example gripper_control.cpp
 * Position and force control with grippers that use the communication protocol
 * template provided by Flexiv.
 * @copyright Copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <flexiv/Robot.hpp>
#include <flexiv/Exception.hpp>
#include <flexiv/Log.hpp>
#include <flexiv/Gripper.hpp>

#include <string>
#include <thread>

int main(int argc, char* argv[])
{
    // Log object for printing message with timestamp and coloring
    flexiv::Log log;

    // Parse Parameters
    //=============================================================================
    // Check if program has 3 arguments
    if (argc != 3) {
        log.error("Invalid program arguments. Usage: <robot_ip> <local_ip>");
        return 0;
    }
    // IP of the robot server
    std::string robotIP = argv[1];

    // IP of the workstation PC running this program
    std::string localIP = argv[2];

    try {
        // RDK Initialization
        //=============================================================================
        // Instantiate robot interface
        flexiv::Robot robot(robotIP, localIP);

        // Clear fault on robot server if any
        if (robot.isFault()) {
            log.warn("Fault occurred on robot server, trying to clear ...");
            // Try to clear the fault
            robot.clearFault();
            std::this_thread::sleep_for(std::chrono::seconds(2));
            // Check again
            if (robot.isFault()) {
                log.error("Fault cannot be cleared, exiting ...");
                return 0;
            }
            log.info("Fault on robot server is cleared");
        }

        // Enable the robot, make sure the E-stop is released before enabling
        log.info("Enabling robot ...");
        // TODO: remove this extra try catch block after the destructor bug in
        // Windows library is fixed
        try {
            robot.enable();
        } catch (const flexiv::Exception& e) {
            log.error(e.what());
            return 0;
        }

        // Wait for the robot to become operational
        while (!robot.isOperational()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        log.info("Robot is now operational");

        // Set mode after robot is operational
        robot.setMode(flexiv::MODE_PLAN_EXECUTION);

        // Wait for the mode to be switched
        while (robot.getMode() != flexiv::MODE_PLAN_EXECUTION) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        robot.executePlanByName("PLAN-Home");
        std::this_thread::sleep_for(std::chrono::seconds(1));

        // Application-specific Code
        //=============================================================================
        // Instantiate gripper
        flexiv::Gripper gripper(&robot);

        // Position control
        // Close to width = 0.02m, using velocity = 0.1m/s
        log.info("Closing fingers");
        gripper.move(0.02, 0.1);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        // Open to width = 0.08m, using velocity = 0.1m/s
        log.info("Opening fingers");
        gripper.move(0.08, 0.1);
        std::this_thread::sleep_for(std::chrono::seconds(1));

        // Force control
        // Close fingers with 10N
        log.info("Grasping with constant force");
        gripper.grasp(10);
        // Hold for 3 seconds
        std::this_thread::sleep_for(std::chrono::seconds(3));

        // Open fingers and stop halfway
        log.info("Opening fingers");
        gripper.move(0.08, 0.1);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        log.info("Stopping gripper");
        gripper.stop();

    } catch (const flexiv::Exception& e) {
        log.error(e.what());
        return 0;
    }

    return 0;
}
