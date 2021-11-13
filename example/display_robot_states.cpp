/**
 * @example display_robot_states.cpp
 * Print received robot states.
 * @copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <Robot.hpp>
#include <Log.hpp>

#include <iostream>
#include <iomanip>
#include <thread>
#include <mutex>

namespace {

// print counter
unsigned int g_printCounter = 0;

// robot states to be printed in another low-priority thread
flexiv::RobotStates g_robotStatesPrintBuffer;

// mutex on robot states print buffer
std::mutex g_printBufferMutex;

}

// user-defined high-priority realtime periodic task, running at 1kHz
void highPriorityPeriodicTask(std::shared_ptr<flexiv::RobotStates> robotStates,
    std::shared_ptr<flexiv::Robot> robot)
{
    // get robot states
    robot->getRobotStates(robotStates.get());

    // save data to buffer for printing in another thread
    {
        std::lock_guard<std::mutex> lock(g_printBufferMutex);
        g_robotStatesPrintBuffer = *(robotStates.get());
    }
}

// user-defined low-priority non-realtime task
// NOT strictly scheduled at a fixed rate
void lowPriorityTask()
{
    // wait a while for the data to start streaming
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // use while loop to prevent this thread from return
    while (true) {
        // wake up every second to do something
        std::this_thread::sleep_for(std::chrono::seconds(1));

        // safely save data in print buffer to local first to avoid prolonged
        // mutex lock
        flexiv::RobotStates robotStates;
        {
            std::lock_guard<std::mutex> lock(g_printBufferMutex);
            robotStates = g_robotStatesPrintBuffer;
        }

        // divider
        std::cout << "\n\n[" << ++g_printCounter << "]";
        std::cout << "========================================================="
                  << std::endl;

        // print all robot states in JSON format using the built-in ostream
        // operator overloading
        std::cout << robotStates << std::endl;
    }
}

int main(int argc, char* argv[])
{
    // log object for printing message with timestamp and coloring
    flexiv::Log log;

    // Parse Parameters
    //=============================================================================
    // check if program has 3 arguments
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
    // instantiate robot interface
    auto robot = std::make_shared<flexiv::Robot>();

    // create data struct for storing robot states
    auto robotStates = std::make_shared<flexiv::RobotStates>();

    // initialize robot interface and connect to the robot server
    robot->init(robotIP, localIP);

    // wait for the connection to be established
    do {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } while (!robot->isConnected());

    // enable the robot, make sure the E-stop is released before enabling
    if (robot->enable()) {
        log.info("Enabling robot ...");
    }

    // wait for the robot to become operational
    do {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } while (!robot->isOperational());
    log.info("Robot is now operational");

    // set mode after robot is operational
    robot->setMode(flexiv::MODE_IDLE);

    // Low-priority Background Tasks
    //=============================================================================
    // use std::thread for some non-realtime tasks, not joining (non-blocking)
    std::thread lowPriorityThread(lowPriorityTask);

    // High-priority Realtime Periodic Task @ 1kHz
    //=============================================================================
    // this is a blocking method, so all other user-defined background threads
    // should be spawned before this
    robot->start(std::bind(highPriorityPeriodicTask, robotStates, robot));

    return 0;
}
