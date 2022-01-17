/**
 * @example display_robot_states.cpp
 * Print received robot states.
 * @copyright Copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <Robot.hpp>
#include <Log.hpp>

#include <iostream>
#include <iomanip>
#include <thread>
#include <mutex>

namespace {
/** Print counter */
unsigned int g_printCounter = 0;

/** Robot states to be printed in another low-priority thread */
flexiv::RobotStates g_robotStatesPrintBuffer;

/** Mutex on robot states print buffer */
std::mutex g_printBufferMutex;
}

/** User-defined high-priority realtime periodic task, running at 1kHz */
void highPriorityPeriodicTask(std::shared_ptr<flexiv::RobotStates> robotStates,
    std::shared_ptr<flexiv::Robot> robot)
{
    // Get robot states
    robot->getRobotStates(robotStates.get());

    // Save data to buffer for printing in another thread
    {
        std::lock_guard<std::mutex> lock(g_printBufferMutex);
        g_robotStatesPrintBuffer = *(robotStates.get());
    }
}

// User-defined low-priority non-realtime task
// NOT strictly scheduled at a fixed rate
void lowPriorityTask()
{
    // Wait a while for the data to start streaming
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // Use while loop to prevent this thread from return
    while (true) {
        // Wake up every second to do something
        std::this_thread::sleep_for(std::chrono::seconds(1));

        // Safely save data in print buffer to local first to avoid prolonged
        // mutex lock
        flexiv::RobotStates robotStates;
        {
            std::lock_guard<std::mutex> lock(g_printBufferMutex);
            robotStates = g_robotStatesPrintBuffer;
        }

        // Print divider
        std::cout << "\n\n[" << ++g_printCounter << "]";
        std::cout << "========================================================="
                  << std::endl;

        // Print all robot states in JSON format using the built-in ostream
        // operator overloading
        std::cout << robotStates << std::endl;
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

    // RDK Initialization
    //=============================================================================
    // Instantiate robot interface
    auto robot = std::make_shared<flexiv::Robot>();

    // Create data struct for storing robot states
    auto robotStates = std::make_shared<flexiv::RobotStates>();

    // Initialize robot interface and connect to the robot server
    robot->init(robotIP, localIP);

    // Wait for the connection to be established
    do {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } while (!robot->isConnected());

    // Enable the robot, make sure the E-stop is released before enabling
    if (robot->enable()) {
        log.info("Enabling robot ...");
    }

    // Wait for the robot to become operational
    do {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } while (!robot->isOperational());
    log.info("Robot is now operational");

    // Set mode after robot is operational
    robot->setMode(flexiv::MODE_IDLE);

    // Low-priority Background Tasks
    //=============================================================================
    // Use std::thread for some non-realtime tasks, not joining (non-blocking)
    std::thread lowPriorityThread(lowPriorityTask);

    // High-priority Realtime Periodic Task @ 1kHz
    //=============================================================================
    // This is a blocking method, so all other user-defined background threads
    // should be spawned before this
    robot->start(std::bind(highPriorityPeriodicTask, robotStates, robot));

    return 0;
}
