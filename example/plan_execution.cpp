/**
 * @example plan_execution.cpp
 * Select a plan from a list to execute using RDK's plan execution API.
 * @copyright Copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <Robot.hpp>
#include <Log.hpp>

#include <string>
#include <iostream>
#include <cmath>
#include <mutex>
#include <thread>

namespace {
/** User input string */
std::string g_userInput;

/** User input mutex */
std::mutex g_userInputMutex;
}

/** Callback function for user input state machine */
void userInputStateMachine(std::shared_ptr<flexiv::Robot> robot)
{
    // Log object for printing message with timestamp and coloring
    flexiv::Log log;

    // Use while loop to prevent this thread from return
    while (true) {
        // Wake up every 0.1 second to do something
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

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
            std::cout
                << "===================== Plan name list ====================="
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
            log.info("Execution stopped, enter index to execute another plan:");

            // After calling stop(), the robot will enter Idle mode, so put the
            // robot back to plan execution mode to take more plan commands
            // set mode after robot is operational
            robot->setMode(flexiv::MODE_PLAN_EXECUTION);

            // Wait for the mode to be switched
            do {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            } while (robot->getMode() != flexiv::MODE_PLAN_EXECUTION);
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
    // RDK robot client
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

    // Wait for the mode to be switched
    do {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } while (robot->getMode() != flexiv::MODE_PLAN_EXECUTION);

    // Low-priority Background Tasks
    //=============================================================================
    // Use std::thread for some non-realtime tasks, not joining (non-blocking)
    std::thread lowPriorityThread(std::bind(userInputStateMachine, robot));

    // User Input
    //=============================================================================
    log.info("Choose from the following options to continue:");
    std::cout << "[l] - show plan list" << std::endl;
    std::cout << "[s] - stop execution of current plan" << std::endl;

    // User input polling
    std::string inputBuffer;
    while (true) {
        std::cin >> inputBuffer;
        {
            std::lock_guard<std::mutex> lock(g_userInputMutex);
            g_userInput = inputBuffer;
        }
    }

    return 0;
}
