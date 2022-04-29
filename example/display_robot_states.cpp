/**
 * @example display_robot_states.cpp
 * Print received robot states without enabling the robot.
 * @copyright Copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <flexiv/Robot.hpp>
#include <flexiv/Exception.hpp>
#include <flexiv/Log.hpp>

#include <iostream>
#include <thread>

/** User-defined periodic task @ 1Hz */
void periodicTask(flexiv::Robot* robot, flexiv::Log* log)
{
    unsigned int printCounter = 0;

    // Data struct for storing robot states
    flexiv::RobotStates robotStates;

    try {
        while (true) {
            // Wake up every second to do something
            std::this_thread::sleep_for(std::chrono::seconds(1));

            // Get robot states
            robot->getRobotStates(&robotStates);

            // Print all robot states in JSON format using the built-in ostream
            // operator overloading
            std::cout << "\n\n[" << ++printCounter << "]";
            std::cout
                << "========================================================="
                << std::endl;

            std::cout << robotStates << std::endl;
        }

    } catch (const flexiv::Exception& e) {
        log->error(e.what());
        return;
    }
}

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

        // Periodic Tasks
        //=============================================================================
        // Use std::thread to do scheduling since this example is used for both
        // Linux and Windows, and the latter does not support flexiv::Scheduler
        std::thread lowPriorityThread(std::bind(periodicTask, &robot, &log));

        // Properly exit thread
        lowPriorityThread.join();

    } catch (const flexiv::Exception& e) {
        log.error(e.what());
        return 0;
    }

    return 0;
}
