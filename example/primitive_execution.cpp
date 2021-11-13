/**
 * @example primitive_execution.cpp
 * Execute various primitives using RDK's primitive execution API.
 * @copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <Robot.hpp>
#include <Log.hpp>

#include <vector>
#include <string>
#include <iostream>
#include <sstream>
#include <cmath>
#include <mutex>
#include <thread>

namespace {
/** set this to true to print primitive states */
const bool k_printPtStates = false;
}

// callback function for user-defined primitive state machine
void primitiveStateMachine(std::shared_ptr<flexiv::Robot> robot)
{
    // log object for printing message with timestamp and coloring
    flexiv::Log log;

    // data struct for system status
    flexiv::SystemStatus systemStatus;

    // if transitting to the next primitive
    bool transitPrimitive = false;

    // index for iterating through various primitives
    unsigned int ptIndex = 1;

    // start any primitive once to pass the first transition condition check
    robot->executePrimitive("Home()");

    // use while loop to prevent this thread from return
    while (true) {
        // run periodic tasks at 10Hz in this thread
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // reset transition flag before checking for transition conditions
        transitPrimitive = false;

        // get system status
        robot->getSystemStatus(&systemStatus);

        // check for transition conditions
        for (const auto& state : systemStatus.m_ptStates) {
            // parse key information from the string list for transition
            // control
            std::stringstream ss(state);
            std::string buffer;
            std::vector<std::string> parsedState;
            while (ss >> buffer) {
                parsedState.push_back(buffer);
            }

            // transition condition: "reachedTarget = 1"
            if (parsedState.front() == "reachedTarget") {
                // save the corresponding value, it should be at the end
                if (std::stoi(parsedState.back()) == 1) {
                    transitPrimitive = true;
                }
            }
        }

        // iterate through commanding various primitives
        if (transitPrimitive) {
            switch (ptIndex) {
                case 0: {
                    // (1) go home
                    // all parameters of the "Home" primitive are optional, thus
                    // we can skip the parameters and the default values will be
                    // used
                    robot->executePrimitive("Home()");

                    // execute the next primitive
                    ptIndex++;

                    break;
                }
                case 1: {
                    // (2) move TCP to point A in task space w.r.t world (base)
                    // frame the mandatory parameter "target" requires 8 values:
                    // [x y z roll pitch yaw reference_frame reference_point]
                    // unit: meters and degrees
                    robot->executePrimitive(
                        "MoveL(target=0.387 -0.11 0.203 90.0 0.0 180.0 WORLD "
                        "WORLD_ORIGIN)");

                    // execute the next primitive
                    ptIndex++;

                    break;
                }
                case 2: {
                    // (3) move TCP to point B in task space w.r.t world (base)
                    // frame the mandatory parameter "target" requires 8 values:
                    // [x y z roll pitch yaw reference_frame reference_point]
                    // unit: meters and degrees
                    robot->executePrimitive(
                        "MoveL(target=0.687 0.1 0.264 180.0 0.0 180.0 WORLD "
                        "WORLD_ORIGIN)");

                    // execute the next primitive
                    ptIndex++;

                    break;
                }
                case 3: {
                    // (4) move TCP to point C in task space w.r.t world (base)
                    // frame the mandatory parameter "target" requires 8 values:
                    // [x y z roll pitch yaw reference_frame reference_point]
                    // unit: meters and degrees
                    robot->executePrimitive(
                        "MoveL(target=0.387 0.5 0.1 -90.0 0.0 180.0 WORLD "
                        "WORLD_ORIGIN)");

                    // execute the next primitive
                    ptIndex++;
                    break;
                }
                case 4: {
                    // (4) move TCP to point C in task space w.r.t world (base)
                    // frame the mandatory parameter "target" requires 8 values:
                    // [x y z roll pitch yaw reference_frame reference_point]
                    // unit: meters and degrees
                    robot->executePrimitive(
                        "MoveL(target=0.5 0.0 0.3 180.0 0.0 180.0 WORLD "
                        "WORLD_ORIGIN)");

                    // execute the next primitive
                    ptIndex++;
                    break;
                }
                case 5: {
                    // repeat the moves
                    ptIndex = 0;

                    break;
                }

                default:
                    log.error("Invalid ptIndex");
                    break;
            }
        }

        // print each state in the primitive states string list
        if (k_printPtStates) {
            for (const auto& state : systemStatus.m_ptStates) {
                std::cout << state << std::endl;
            }
            // empty line
            std::cout << std::endl;
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
    // RDK robot client
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
    robot->setMode(flexiv::MODE_PRIMITIVE_EXECUTION);

    // wait for the mode to be switched
    do {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } while (robot->getMode() != flexiv::MODE_PRIMITIVE_EXECUTION);

    // Low-priority Background Tasks
    //=============================================================================
    // use std::thread for some non-realtime tasks, not joining
    // (non-blocking)
    std::thread lowPriorityThread(std::bind(primitiveStateMachine, robot));

    // block main thread
    lowPriorityThread.join();

    return 0;
}
