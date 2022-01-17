/**
 * @test test_scheduler.cpp
 * A test to evaluate RDK's internal real-time scheduler.
 * @copyright Copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <Robot.hpp>
#include <Log.hpp>

#include <iostream>
#include <thread>
#include <chrono>
#include <mutex>

namespace {
// timer to measure scheduler performance
std::chrono::high_resolution_clock::time_point g_tic, g_toc;

// scheduler time interval of the high-priority periodic task [us]
int64_t g_timeInterval;

// mutex on the time interval data
std::mutex g_intervalMutex;

// mutex on position error
std::mutex g_errorPosMutex;

// flag for position error
bool g_flagErrorPos = false;

// mutex on orientation error
std::mutex g_errorRotMutex;

// flag for orientation error
bool g_flagErrorRot = false;

// error counter
int g_numError = 0;

// flag for test complete
bool g_flagTestComplete = false;

// Test time in seconds
const int k_testTime = 600;
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

    // Get position and position norm
    double posX = robotStates->m_tcpPose[0];
    double posY = robotStates->m_tcpPose[1];
    double posZ = robotStates->m_tcpPose[2];

    double posLength = sqrt(pow(posX, 2) + pow(posY, 2) + pow(posZ, 2));
    if (posLength >= 2.0) {
        {
            std::lock_guard<std::mutex> lock(g_errorPosMutex);
            g_flagErrorPos = true;
        }
    }

    // Get orientation and orientation norm
    double quadW = robotStates->m_tcpPose[3];
    double quadX = robotStates->m_tcpPose[4];
    double quadY = robotStates->m_tcpPose[5];
    double quadZ = robotStates->m_tcpPose[6];

    double quadNorm
        = sqrt(pow(quadW, 2) + pow(quadX, 2) + pow(quadY, 2) + pow(quadZ, 2));

    if (abs(quadNorm - 1.0) >= 1e-2) {
        {
            std::lock_guard<std::mutex> lock(g_errorRotMutex);
            g_flagErrorRot = true;
        }
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

    // log object for printing message with timestamp and coloring
    flexiv::Log log;

    // wait for a while for the time interval data to be available
    std::this_thread::sleep_for(std::chrono::seconds(2));

    log.info("Starting error counting...");

    // use while loop to prevent this thread from return
    while (true) {
        // wake up every second to do something
        std::this_thread::sleep_for(std::chrono::seconds(1));

        // Get error in position
        bool flagErrorPos = false;
        {
            std::lock_guard<std::mutex> lock(g_errorPosMutex);
            flagErrorPos = g_flagErrorPos;
            g_flagErrorPos = false;
        }

        if (flagErrorPos) {
            log.warn("Error in position");
            g_numError++;
        }

        // Get error in rotation
        bool flagErrorRot = false;
        {
            std::lock_guard<std::mutex> lock(g_errorRotMutex);
            flagErrorRot = g_flagErrorRot;
            g_flagErrorRot = false;
        }

        if (flagErrorRot) {
            log.warn("Error in orientation");
            g_numError++;
        }

        // safely read the global variable
        int timeInterval;
        {
            std::lock_guard<std::mutex> lock(g_intervalMutex);
            timeInterval = g_timeInterval;
        }

        if ((float)totalTime >= (float)k_testTime * 1000.0) {
            if (g_flagTestComplete == false) {
                log.info(
                    "Test ended - Total error = " + std::to_string(g_numError));

                g_flagTestComplete = true;
            }
        } else {
            // calculate average time interval
            totalTime += timeInterval;
            measureCount++;
            avgInterval = (float)totalTime / (float)measureCount;
        }
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
