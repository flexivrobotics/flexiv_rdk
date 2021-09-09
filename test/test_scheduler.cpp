/**
 * @test Test to evaluate RDK's internal real-time scheduler
 * @copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <config.h>
#include <Robot.hpp>

#include <iostream>
#include <thread>
#include <chrono>
#include <mutex>

namespace {
// robot DOF
const int k_robotDofs = 7;

// loop counter
unsigned int g_loopCounter = 0;

// timer to measure scheduler performance
std::chrono::high_resolution_clock::time_point g_tic, g_toc;

// scheduler time interval of the high-priority periodic task [us]
int64_t g_timeInterval;

// mutex on the time interval data
std::mutex g_intervalMutex;

}

// user-defined high-priority realtime periodic task, running at 1kHz
void highPriorityPeriodicTask(std::shared_ptr<flexiv::RobotStates> robotStates,
    std::shared_ptr<flexiv::Robot> robot)
{
    // mark timer end point
    g_toc = std::chrono::high_resolution_clock::now();

    // calculate scheduler's interrupt interval and print
    auto timeInterval
        = std::chrono::duration_cast<std::chrono::microseconds>(g_toc - g_tic)
              .count();

    // save to global variable for printing in another thread
    {
        std::lock_guard<std::mutex> lock(g_intervalMutex);
        g_timeInterval = timeInterval;
    }

    // do some random stuff to verify the callback's params are working
    robot->getRobotStates(robotStates.get());
    if ((robotStates->m_linkPosition.size() != k_robotDofs)
        || (robotStates->m_linkTorque.size() != k_robotDofs)) {
        std::cerr << "robotStates message corrupted!" << std::endl;
    }

    // mark timer start point
    g_tic = std::chrono::high_resolution_clock::now();
}

// user-defined low-priority non-realtime task
// NOT strictly scheduled at a fixed rate
void lowPriorityTask()
{
    uint64_t totalTime = 0;
    uint64_t measureCount = 0;
    float avgInterval = 0.0;

    // wait for a while for the time interval data to be available
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // use while loop to prevent this thread from return
    while (true) {
        // wake up every second to do something
        std::this_thread::sleep_for(std::chrono::seconds(1));

        // safely read the global variable
        int timeInterval;
        {
            std::lock_guard<std::mutex> lock(g_intervalMutex);
            timeInterval = g_timeInterval;
        }

        // calculate average time interval
        totalTime += timeInterval;
        measureCount++;
        avgInterval = (float)totalTime / (float)measureCount;

        // print time interval of high-priority periodic task
        std::cout << "RT task interval (curr | avg) = " << timeInterval << " | "
                  << avgInterval << " us" << std::endl;
    }
}

int main(int argc, char* argv[])
{
    // print loop frequency
    std::cout << "Example client running at 1000 Hz" << std::endl;

    // RDK Initialization
    //=============================================================================
    // RDK robot interface
    auto robot = std::make_shared<flexiv::Robot>();

    // robot states data from RDK server
    auto robotStates = std::make_shared<flexiv::RobotStates>();

    // initialize connection
    robot->init(ROBOT_IP, LOCAL_IP);

    // wait for the connection to be established
    do {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } while (!robot->isConnected());

    // enable the robot, make sure the E-stop is released before enabling
    if (robot->enable()) {
        std::cout << "Enabling robot ..." << std::endl;
    }

    // wait for the robot to become operational
    do {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } while (!robot->isOperational());
    std::cout << "Robot is now operational" << std::endl;

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
