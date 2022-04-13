/**
 * @example display_robot_states.cpp
 * Print received robot states without enabling the robot.
 * @copyright Copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <flexiv/Robot.hpp>
#include <flexiv/Exception.hpp>
#include <flexiv/Log.hpp>
#include <flexiv/Scheduler.hpp>

#include <iostream>
#include <iomanip>
#include <thread>
#include <mutex>

namespace {
/** Mutex on shared data */
std::mutex g_mutex;
}

/** User-defined high-priority periodic task @ 1kHz */
void highPriorityTask(flexiv::Robot* robot, flexiv::RobotStates* robotStates,
    flexiv::Scheduler* scheduler, flexiv::Log* log)
{
    try {
        // Safely write shared data
        std::lock_guard<std::mutex> lock(g_mutex);
        // Get robot states
        robot->getRobotStates(robotStates);

    } catch (const flexiv::Exception& e) {
        log->error(e.what());
        scheduler->stop();
    }
}

/** User-defined low-priority periodic task @ 1Hz */
void lowPriorityTask(flexiv::RobotStates* robotStates)
{
    static unsigned int printCounter = 0;
    constexpr size_t k_nominalDof = 7;

    // Safely read shared data
    flexiv::RobotStates robotStatesCopy;
    {
        std::lock_guard<std::mutex> lock(g_mutex);
        robotStatesCopy = *robotStates;
    }

    // Print only when the data is valid
    if (robotStatesCopy.m_q.size() == k_nominalDof) {
        // Print divider
        std::cout << "\n\n[" << ++printCounter << "]";
        std::cout << "========================================================="
                  << std::endl;

        // Print all robot states in JSON format using the built-in ostream
        // operator overloading
        std::cout << robotStatesCopy << std::endl;
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

        // Create data struct for storing robot states
        flexiv::RobotStates robotStates;

        // Periodic Tasks
        //=============================================================================
        flexiv::Scheduler scheduler;
        // Add periodic task with 1ms interval and highest applicable priority
        scheduler.addTask(
            std::bind(highPriorityTask, &robot, &robotStates, &scheduler, &log),
            "HP periodic", 1, 45);
        // Add periodic task with 1s interval and lowest applicable priority
        scheduler.addTask(
            std::bind(lowPriorityTask, &robotStates), "LP periodic", 1000, 0);
        // Start all added tasks, this is by default a blocking method
        scheduler.start();

    } catch (const flexiv::Exception& e) {
        log.error(e.what());
        return 0;
    }

    return 0;
}
