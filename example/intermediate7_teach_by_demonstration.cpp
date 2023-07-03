/**
 * @example intermediate7_teach_by_demonstration.cpp
 * This tutorial shows a demo implementation for teach by demonstration: free-drive the robot and
 * record a series of Cartesian poses, which are then reproduced by the robot.
 * @copyright Copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <flexiv/Robot.hpp>
#include <flexiv/Log.hpp>
#include <flexiv/Utility.hpp>

#include <iostream>
#include <mutex>
#include <thread>
#include <atomic>

namespace {
/** User input string */
std::string g_userInput;

/** User input mutex */
std::mutex g_userInputMutex;

/** Maximum contact wrench [fx, fy, fz, mx, my, mz] [N][Nm]*/
const std::array<double, flexiv::k_cartDOF> k_maxContactWrench
    = {50.0, 50.0, 50.0, 15.0, 15.0, 15.0};
}

/** @brief Print tutorial description */
void printDescription()
{
    std::cout
        << "This tutorial shows a demo implementation for teach by demonstration: free-drive the "
           "robot and record a series of Cartesian poses, which are then reproduced by the robot."
        << std::endl
        << std::endl;
}

/** @brief Print program usage help */
void printHelp()
{
    // clang-format off
    std::cout << "Required arguments: [robot SN]" << std::endl;
    std::cout << "    robot SN: Serial number of the robot to connect to. "
                 "Remove any space, for example: Rizon4s-123456" << std::endl;
    std::cout << "Optional arguments: None" << std::endl;
    std::cout << std::endl;
    // clang-format on
}

int main(int argc, char* argv[])
{
    // Program Setup
    // =============================================================================================
    // Logger for printing message with timestamp and coloring
    flexiv::Log log;

    // Parse parameters
    if (argc < 2 || flexiv::utility::programArgsExistAny(argc, argv, {"-h", "--help"})) {
        printHelp();
        return 1;
    }
    // Serial number of the robot to connect to. Remove any space, for example: Rizon4s-123456
    std::string robotSN = argv[1];

    // Print description
    log.info("Tutorial description:");
    printDescription();

    try {
        // RDK Initialization
        // =========================================================================================
        // Instantiate robot interface
        flexiv::Robot robot(robotSN);

        // Clear fault on robot server if any
        if (robot.isFault()) {
            log.warn("Fault occurred on robot server, trying to clear ...");
            // Try to clear the fault
            robot.clearFault();
            std::this_thread::sleep_for(std::chrono::seconds(2));
            // Check again
            if (robot.isFault()) {
                log.error("Fault cannot be cleared, exiting ...");
                return 1;
            }
            log.info("Fault on robot server is cleared");
        }

        // Enable the robot, make sure the E-stop is released before enabling
        log.info("Enabling robot ...");
        robot.enable();

        // Wait for the robot to become operational
        int secondsWaited = 0;
        while (!robot.isOperational()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            if (++secondsWaited == 10) {
                log.warn(
                    "Still waiting for robot to become operational, please check that the robot 1) "
                    "has no fault, 2) is in [Auto (remote)] mode");
            }
        }
        log.info("Robot is now operational");

        // Teach By Demonstration
        // =========================================================================================
        // Recorded robot poses
        std::vector<std::array<double, flexiv::k_poseSize>> savedPoses = {};

        // Robot states data
        flexiv::RobotStates robotStates = {};

        // Acceptable user inputs
        log.info("Accepted key inputs:");
        std::cout << "[n] - start new teaching process" << std::endl;
        std::cout << "[r] - record current robot pose" << std::endl;
        std::cout << "[e] - finish recording and start execution" << std::endl;

        // User input polling
        std::string inputBuffer;
        while (true) {
            std::cin >> inputBuffer;
            // Start new teaching process
            if (inputBuffer == "n") {
                // Clear storage
                savedPoses.clear();

                // Put robot to plan execution mode
                robot.setMode(flexiv::Mode::NRT_PLAN_EXECUTION);

                // Robot run free drive
                robot.executePlan("PLAN-FreeDriveAuto");

                log.info("New teaching process started");
                log.warn("Hold down the enabling button on the motion bar to activate free drive");
            }
            // Save current robot pose
            else if (inputBuffer == "r") {
                if (!robot.isBusy()) {
                    log.warn("Please start a new teaching process first");
                    continue;
                }

                robot.getRobotStates(robotStates);
                savedPoses.push_back(robotStates.tcpPose);
                log.info("New pose saved: " + flexiv::utility::arr2Str(robotStates.tcpPose));
                log.info("Number of saved poses: " + std::to_string(savedPoses.size()));
            }
            // Reproduce recorded poses
            else if (inputBuffer == "e") {
                if (savedPoses.empty()) {
                    log.warn("No pose is saved yet");
                    continue;
                }

                // Put robot to primitive execution mode
                robot.setMode(flexiv::Mode::NRT_PRIMITIVE_EXECUTION);

                for (size_t i = 0; i < savedPoses.size(); i++) {
                    log.info("Executing pose " + std::to_string(i + 1) + "/"
                             + std::to_string(savedPoses.size()));

                    std::array<double, 3> targetPos
                        = {savedPoses[i][0], savedPoses[i][1], savedPoses[i][2]};
                    // Convert quaternion to Euler ZYX required by MoveCompliance primitive
                    std::array<double, 4> targetQuat
                        = {savedPoses[i][3], savedPoses[i][4], savedPoses[i][5], savedPoses[i][6]};
                    auto targetEulerDeg
                        = flexiv::utility::rad2Deg(flexiv::utility::quat2EulerZYX(targetQuat));
                    robot.executePrimitive(
                        "MoveCompliance(target="
                        + flexiv::utility::arr2Str(targetPos)
                        + flexiv::utility::arr2Str(targetEulerDeg)
                        + "WORLD WORLD_ORIGIN, maxVel=0.3, enableMaxContactWrench=1, maxContactWrench=" 
                        + flexiv::utility::arr2Str(k_maxContactWrench)+ ")");

                    // Wait for reached target
                    while (
                        flexiv::utility::parsePtStates(robot.getPrimitiveStates(), "reachedTarget")
                        != "1") {
                        std::this_thread::sleep_for(std::chrono::seconds(1));
                    }
                }

                log.info(
                    "All saved poses are executed, enter 'n' to start a new teaching process, 'r' "
                    "to record more poses, 'e' to repeat execution");

                // Put robot back to free drive
                robot.setMode(flexiv::Mode::NRT_PLAN_EXECUTION);
                robot.executePlan("PLAN-FreeDriveAuto");
            } else {
                log.warn("Invalid input");
            }
        }

    } catch (const std::exception& e) {
        log.error(e.what());
        return 1;
    }

    return 0;
}
