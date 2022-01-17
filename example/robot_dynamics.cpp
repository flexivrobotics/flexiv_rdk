/**
 * @example robot_dynamics.cpp
 * Demonstrate how to use RDK's built-in dynamics engine using APIs in
 * flexiv::Model.
 * @copyright Copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <Robot.hpp>
#include <Model.hpp>
#include <Log.hpp>

#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>
#include <mutex>

namespace {

/** Loop counter */
unsigned int g_loopCounter = 0;

/** Timer to measure scheduler performance */
std::chrono::high_resolution_clock::time_point g_tic, g_toc;

/** Data to be printed in low-priority thread */
struct PrintData
{
    int64_t loopTime;
    Eigen::VectorXd gravity;
} g_printData;

/** Mutex on data to be printed in low-priority thread */
std::mutex g_printDataMutex;

}

/** User-defined high-priority realtime periodic task, running at 1kHz */
void highPriorityPeriodicTask(std::shared_ptr<flexiv::RobotStates> robotStates,
    std::shared_ptr<flexiv::Robot> robot, std::shared_ptr<flexiv::Model> model)
{
    // Mark timer start point
    g_tic = std::chrono::high_resolution_clock::now();

    // Get new robot states
    robot->getRobotStates(robotStates.get());

    // Update robot model in dynamics engine
    model->updateModel(robotStates->m_q, robotStates->m_dtheta);

    // Get and print gravity vector
    auto gravity = model->getGravityForce();

    // Mark timer end point and get loop time
    g_toc = std::chrono::high_resolution_clock::now();
    auto loopTime
        = std::chrono::duration_cast<std::chrono::microseconds>(g_toc - g_tic)
              .count();

    // Save to global struct for printing in another thread
    {
        std::lock_guard<std::mutex> lock(g_printDataMutex);
        g_printData.loopTime = loopTime;
        g_printData.gravity = gravity;
    }
}

/** User-defined low-priority non-realtime task
 * @note NOT strictly scheduled at a fixed rate
 */
void lowPriorityTask()
{
    // Wait for a while for the time interval data to be available
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // Use while loop to prevent this thread from return
    while (true) {
        // Wake up every second to do something
        std::this_thread::sleep_for(std::chrono::seconds(1));

        // Safely read the global variable
        int loopTime;
        Eigen::VectorXd gravity;
        {
            std::lock_guard<std::mutex> lock(g_printDataMutex);
            loopTime = g_printData.loopTime;
            gravity = g_printData.gravity;
        }

        // Print time interval of high-priority periodic task
        std::cout << "Loop time = " << loopTime << " us  ||  ";

        // Print gravity
        std::cout << std::fixed << std::setprecision(5)
                  << "Gravity = " << gravity.transpose() << std::endl;
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
    robot->setMode(flexiv::MODE_PLAN_EXECUTION);

    // Wait for mode to be switched
    do {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } while (robot->getMode() != flexiv::MODE_PLAN_EXECUTION);

    // Bring Robot To Home
    //=============================================================================
    robot->executePlanByName("PLAN-Home");

    flexiv::SystemStatus systemStatus;
    // Wait for the plan to start
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // Wait for the execution to finish
    do {
        robot->getSystemStatus(&systemStatus);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } while (systemStatus.m_programRunning == true);

    // Put mode back to IDLE
    robot->setMode(flexiv::MODE_IDLE);

    // Wait for the mode to be switched
    do {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } while (robot->getMode() != flexiv::MODE_IDLE);

    // Robot Model (Dynamics Engine) Initialization
    //=============================================================================
    auto model = std::make_shared<flexiv::Model>();
    // Load model from robot client
    robot->loadModel(model.get());

    // Low-priority Background Tasks
    //=============================================================================
    // Use std::thread for some non-realtime tasks, not joining (non-blocking)
    std::thread lowPriorityThread(lowPriorityTask);

    // High-priority Realtime Periodic Task @ 1kHz
    //=============================================================================
    // This is a blocking method, so all other user-defined background threads
    // should be spawned before this
    robot->start(
        std::bind(highPriorityPeriodicTask, robotStates, robot, model));

    return 0;
}
