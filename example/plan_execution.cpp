/**
 * @example Select a plan from a list to execute using RDK's plan execution API
 * @copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <config.h>
#include <Robot.hpp>

#include <string>
#include <iostream>
#include <cmath>
#include <mutex>
#include <thread>

namespace {
// user input string
std::string g_userInput;

// user input mutex
std::mutex g_userInputMutex;
}

// callback function for user input state machine
void userInputStateMachine(std::shared_ptr<flexiv::Robot> robot)
{
    // use while loop to prevent this thread from return
    while (true) {
        // wake up every 0.1 second to do something
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // check user input
        std::string inputBuffer;
        {
            std::lock_guard<std::mutex> lock(g_userInputMutex);
            inputBuffer = g_userInput;
        }

        if (inputBuffer.empty()) {
            // do nothing, waiting for new user input or plan to finish
        }
        // list available plans
        else if (inputBuffer == "l") {
            std::vector<std::string> planList;
            robot->getPlanNameList(&planList);
            std::cout
                << "===================== Plan name list ====================="
                << std::endl;
            // print plan list
            for (unsigned int i = 0; i < planList.size(); i++) {
                std::cout << "[" << i << "] " << planList[i] << std::endl;
            }
            std::cout << "Enter index to select a plan to execute:"
                      << std::endl;
        }
        // stop plan execution
        else if (inputBuffer == "s") {
            robot->stop();
            std::cout
                << "Execution stopped, enter index to execute another plan:"
                << std::endl;

            // after calling stop(), the robot will enter Idle mode, so put the
            // robot back to plan execution mode to take more plan commands
            // set mode after robot is operational
            robot->setMode(flexiv::MODE_PLAN_EXECUTION);

            // wait for the mode to be switched
            do {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            } while (robot->getMode() != flexiv::MODE_PLAN_EXECUTION);
        }
        // choose plan to execute
        else {
            // check system status
            flexiv::SystemStatus systemStatus;
            robot->getSystemStatus(&systemStatus);

            // execute new plan if no plan is running
            if (systemStatus.m_programRunning == false) {
                // convert string to int
                int index = std::stoi(inputBuffer);
                // start executing
                robot->executePlanByIndex(index);
            }
        }

        // reset user input every loop
        {
            std::lock_guard<std::mutex> lock(g_userInputMutex);
            g_userInput.clear();
        }
    }
}

int main(int argc, char* argv[])
{
    // RDK Initialization
    //=============================================================================
    //! RDK robot client
    auto robot = std::make_shared<flexiv::Robot>();

    //! robot states data from RDK server
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
    robot->setMode(flexiv::MODE_PLAN_EXECUTION);

    // wait for the mode to be switched
    do {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } while (robot->getMode() != flexiv::MODE_PLAN_EXECUTION);

    // Low-priority Background Tasks
    //=============================================================================
    // use std::thread for some non-realtime tasks, not joining (non-blocking)
    std::thread lowPriorityThread(std::bind(userInputStateMachine, robot));

    // User Input
    //=============================================================================
    std::cout << "Choose the following option to continue:" << std::endl;
    std::cout << "[l] - show plan list" << std::endl;
    std::cout << "[s] - stop execution of current plan" << std::endl;

    // user input polling
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
