/**
 * @example primitive_execution.cpp
 * Execute various primitives using RDK's primitive execution API.
 * @copyright Copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <flexiv/Robot.hpp>
#include <flexiv/Exception.hpp>
#include <flexiv/Log.hpp>

#include <vector>
#include <string>
#include <iostream>
#include <sstream>
#include <cmath>
#include <mutex>
#include <thread>

namespace {
/** Set this to true to print primitive states */
const bool k_printPtStates = true;
}

/** Callback function for user-defined primitive state machine */
void primitiveStateMachine(flexiv::Robot* robot)
{
    // Log object for printing message with timestamp and coloring
    flexiv::Log log;

    // If transitting to the next primitive
    bool transitPrimitive = false;

    // Index for iterating through various primitives
    unsigned int ptIndex = 1;

    try {
        // Start any primitive once to pass the first transition condition check
        robot->executePrimitive("Home()");

        // Use while loop to prevent this thread from return
        while (true) {
            // Run user periodic tasks at 10Hz in this thread
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            // Monitor fault on robot server
            if (robot->isFault()) {
                throw flexiv::ServerException(
                    "primitiveStateMachine: Fault occurred on robot server, "
                    "exiting ...");
            }

            // Reset transition flag before checking for transition conditions
            transitPrimitive = false;

            // Get system status
            std::vector<std::string> ptStates = robot->getPrimitiveStates();

            // Check for transition conditions
            for (const auto& state : ptStates) {
                // Parse key information from the string list for transition
                // control
                std::stringstream ss(state);
                std::string buffer;
                std::vector<std::string> parsedState;
                while (ss >> buffer) {
                    parsedState.push_back(buffer);
                }

                // Transition condition: "reachedTarget = 1"
                if (parsedState.front() == "reachedTarget") {
                    // Save the corresponding value, it should be at the end
                    if (std::stoi(parsedState.back()) == 1) {
                        transitPrimitive = true;
                    }
                }
            }

            // Iterate through commanding various primitives
            if (transitPrimitive) {
                switch (ptIndex) {
                    case 0: {
                        // (1) Go home, all parameters of the "Home" primitive
                        // are optional, thus we can skip the parameters and the
                        // default values will be used
                        robot->executePrimitive("Home()");

                        // Execute the next primitive
                        ptIndex++;

                        break;
                    }
                    case 1: {
                        // (2) Move TCP to point A in world (base) frame. The
                        // mandatory parameter "target" requires 8 values: [x y
                        // z roll pitch yaw reference_frame reference_point]
                        // unit: meters and degrees
                        robot->executePrimitive(
                            "MoveL(target=0.387 -0.11 0.203 180.0 0.0 180.0 "
                            "WORLD "
                            "WORLD_ORIGIN)");

                        // Execute the next primitive
                        ptIndex++;

                        break;
                    }
                    case 2: {
                        // (3) Move TCP to point B in world (base) frame
                        robot->executePrimitive(
                            "MoveL(target=0.687 0.0 0.264 180.0 0.0 180.0 "
                            "WORLD "
                            "WORLD_ORIGIN)");

                        // Execute the next primitive
                        ptIndex++;

                        break;
                    }
                    case 3: {
                        // (4) Move TCP to point C in world (base) frame
                        robot->executePrimitive(
                            "MoveL(target=0.387 0.1 0.1 -90.0 0.0 180.0 WORLD "
                            "WORLD_ORIGIN)");

                        // Execute the next primitive
                        ptIndex++;
                        break;
                    }
                    case 4: {
                        // (5) Move TCP to point D in world (base) frame
                        robot->executePrimitive(
                            "MoveL(target=0.5 0.0 0.3 180.0 0.0 180.0 WORLD "
                            "WORLD_ORIGIN)");

                        // Execute the next primitive
                        ptIndex++;
                        break;
                    }
                    case 5: {
                        // Repeat the moves
                        ptIndex = 0;

                        break;
                    }

                    default:
                        log.error("Invalid ptIndex");
                        break;
                }
            }

            // Print each state in the primitive states string list
            if (k_printPtStates) {
                for (const auto& state : ptStates) {
                    std::cout << state << std::endl;
                }
                // Empty line
                std::cout << std::endl;
            }
        }
    } catch (const flexiv::Exception& e) {
        log.error(e.what());
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
        robot.setMode(flexiv::MODE_PRIMITIVE_EXECUTION);

        // Wait for the mode to be switched
        while (robot.getMode() != flexiv::MODE_PRIMITIVE_EXECUTION) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        // Periodic Tasks
        //=============================================================================
        // Use std::thread for some low-priority tasks, not joining
        // (non-blocking)
        std::thread lowPriorityThread(std::bind(primitiveStateMachine, &robot));

        // Wait for thread to finish
        lowPriorityThread.join();

    } catch (const flexiv::Exception& e) {
        log.error(e.what());
        return 0;
    }

    return 0;
}
