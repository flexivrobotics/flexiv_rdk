/**
 * @example robot_dynamics.cpp
 * Demonstrate how to use RDK's built-in dynamics engine using APIs in
 * flexiv::Model.
 * @copyright Copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <flexiv/Robot.hpp>
#include <flexiv/Exception.hpp>
#include <flexiv/Model.hpp>
#include <flexiv/Log.hpp>
#include <flexiv/Scheduler.hpp>

#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>
#include <mutex>

namespace {
/** Data shared between threads */
struct SharedData
{
    int64_t loopTime;
    Eigen::VectorXd gravity;
} g_data;

/** Mutex on the shared data */
std::mutex g_mutex;

}

/** User-defined high-priority periodic task @ 1kHz */
void highPriorityTask(flexiv::Robot* robot, flexiv::RobotStates* robotStates,
    flexiv::Scheduler* scheduler, flexiv::Log* log, flexiv::Model* model)
{
    try {
        // Monitor fault on robot server
        if (robot->isFault()) {
            throw flexiv::ServerException(
                "highPriorityTask: Fault occurred on robot server, exiting "
                "...");
        }

        // Mark timer start point
        auto tic = std::chrono::high_resolution_clock::now();

        // Read robot states
        robot->getRobotStates(robotStates);

        // Update robot model in dynamics engine
        model->updateModel(robotStates->m_q, robotStates->m_dtheta);

        // Get and print gravity vector
        auto gravity = model->getGravityForce();

        // Mark timer end point and get loop time
        auto toc = std::chrono::high_resolution_clock::now();
        auto loopTime
            = std::chrono::duration_cast<std::chrono::microseconds>(toc - tic)
                  .count();

        // Save to global struct for printing in another thread
        {
            std::lock_guard<std::mutex> lock(g_mutex);
            g_data.loopTime = loopTime;
            g_data.gravity = gravity;
        }

    } catch (const flexiv::Exception& e) {
        log->error(e.what());
        scheduler->stop();
    }
}

/** User-defined low-priority periodic task @ 1Hz */
void lowPriorityTask()
{
    // Safely read shared data
    int loopTime;
    Eigen::VectorXd gravity;
    {
        std::lock_guard<std::mutex> lock(g_mutex);
        loopTime = g_data.loopTime;
        gravity = g_data.gravity;
    }

    // Print time interval of high-priority periodic task
    std::cout << "Loop time = " << loopTime << " us  ||  ";

    // Print gravity
    std::cout << std::fixed << std::setprecision(5)
              << "Gravity = " << gravity.transpose() << std::endl;
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
        robot.enable();

        // Wait for the robot to become operational
        while (!robot.isOperational()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        log.info("Robot is now operational");

        // Set mode after robot is operational
        robot.setMode(flexiv::MODE_PLAN_EXECUTION);

        // Wait for mode to be switched
        while (robot.getMode() != flexiv::MODE_PLAN_EXECUTION) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        // Bring Robot To Home
        //=============================================================================
        robot.executePlanByName("PLAN-Home");

        flexiv::SystemStatus systemStatus;
        // Wait for the plan to start
        std::this_thread::sleep_for(std::chrono::seconds(1));

        // Wait for the execution to finish
        do {
            robot.getSystemStatus(&systemStatus);
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        } while (systemStatus.m_programRunning);

        // Put mode back to IDLE
        robot.setMode(flexiv::MODE_IDLE);

        // Wait for the mode to be switched
        while (robot.getMode() != flexiv::MODE_IDLE) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        // Robot Model (Dynamics Engine) Initialization
        //=============================================================================
        flexiv::Model model(&robot);

        // Periodic Tasks
        //=============================================================================
        flexiv::Scheduler scheduler;
        // Add periodic task with 1ms interval and highest applicable priority
        scheduler.addTask(std::bind(highPriorityTask, &robot, &robotStates,
                              &scheduler, &log, &model),
            "HP periodic", 1, 45);
        // Add periodic task with 1s interval and lowest applicable priority
        scheduler.addTask(lowPriorityTask, "LP periodic", 1000, 0);
        // Start all added tasks, this is by default a blocking method
        scheduler.start();

    } catch (const flexiv::Exception& e) {
        log.error(e.what());
        return 0;
    }

    return 0;
}
