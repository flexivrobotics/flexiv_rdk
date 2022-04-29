/**
 * @example plan_execution.cpp
 * Select a plan from a list to execute using RDK's plan execution API.
 * @copyright Copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <flexiv/Robot.hpp>
#include <flexiv/Exception.hpp>
#include <flexiv/Log.hpp>

#include <string>
#include <iostream>
#include <cmath>
#include <mutex>
#include <thread>
#include <atomic>

namespace {
/** User input string */
std::string g_userInput;

/** User input mutex */
std::mutex g_userInputMutex;

/** Flag to exit main thread */
std::atomic<bool> g_exitMain = {false};
}

/** Callback function for user input state machine */
void userInputStateMachine(flexiv::Robot* robot)
{
    // Log object for printing message with timestamp and coloring
    flexiv::Log log;

    try {
        // Use while loop to prevent this thread from return
        while (true) {
            // Wake up every 0.1 second to do something
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            // Monitor fault on robot server
            if (robot->isFault()) {
                throw flexiv::ServerException(
                    "userInputStateMachine: Fault occurred on robot server, "
                    "input anything to exit ...");
            }

            // Check user input
            std::string inputBuffer;
            {
                std::lock_guard<std::mutex> lock(g_userInputMutex);
                inputBuffer = g_userInput;
            }

            if (inputBuffer.empty()) {
                // Do nothing, waiting for new user input or plan to finish
            }
            // List available plans
            else if (inputBuffer == "l") {
                auto planList = robot->getPlanNameList();
                std::cout << "===================== Plan name list "
                             "====================="
                          << std::endl;
                // Print plan list
                for (unsigned int i = 0; i < planList.size(); i++) {
                    std::cout << "[" << i << "] " << planList[i] << std::endl;
                }
                log.info("Enter index to select a plan to execute:");
            }
            // Stop plan execution
            else if (inputBuffer == "s") {
                robot->stop();
                log.info(
                    "Execution stopped, enter index to execute another plan:");

                // After calling stop(), the robot will enter Idle mode, so put
                // the robot back to plan execution mode to take more plan
                // commands set mode after robot is operational
                robot->setMode(flexiv::MODE_PLAN_EXECUTION);

                // Wait for the mode to be switched
                while (robot->getMode() != flexiv::MODE_PLAN_EXECUTION) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                }
            }
            // Choose plan to execute
            else {
                // Check system status
                flexiv::SystemStatus systemStatus;
                robot->getSystemStatus(&systemStatus);

                // Execute new plan if no plan is running
                if (systemStatus.m_programRunning == false) {
                    // Convert string to int
                    int index = std::stoi(inputBuffer);
                    // Start executing
                    robot->executePlanByIndex(index);
                }
            }

            // Reset user input every loop
            {
                std::lock_guard<std::mutex> lock(g_userInputMutex);
                g_userInput.clear();
            }
        }

    } catch (const flexiv::Exception& e) {
        log.error(e.what());
        g_exitMain = true;
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

        // Set mode after robot is operational
        robot.setMode(flexiv::MODE_PLAN_EXECUTION);

        // Wait for the mode to be switched
        while (robot.getMode() != flexiv::MODE_PLAN_EXECUTION) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        // Periodic Tasks
        //=============================================================================
        // Use std::thread for some low-priority tasks, not joining
        // (non-blocking)
        std::thread lowPriorityThread(std::bind(userInputStateMachine, &robot));

        // User Input
        //=============================================================================
        log.info("Choose from the following options to continue:");
        std::cout << "[l] - show plan list" << std::endl;
        std::cout << "[s] - stop execution of current plan" << std::endl;

        // User input polling
        std::string inputBuffer;
        while (!g_exitMain) {
            std::cin >> inputBuffer;
            {
                std::lock_guard<std::mutex> lock(g_userInputMutex);
                g_userInput = inputBuffer;
            }
        }

        // Properly exit thread
        lowPriorityThread.join();

    } catch (const flexiv::Exception& e) {
        log.error(e.what());
        return 0;
    }

    return 0;
}
