/**
 * @example joint_position_control.cpp
 * Run joint position control using RDK's joint position streaming API.
 * @copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <Robot.hpp>
#include <Log.hpp>

#include <iostream>
#include <string>
#include <cmath>
#include <thread>

namespace {
// robot DOF
const int k_robotDofs = 7;

// RT loop period [sec]
const double k_loopPeiord = 0.001;

// sine-sweep trajectory amplitude and frequency
const std::vector<double> k_sineAmp
    = {0.035, 0.035, 0.035, 0.035, 0.035, 0.035, 0.035};
const std::vector<double> k_sineFreq = {0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3};

// initial position of robot joints
std::vector<double> g_initPosition;

// flag whether initial joint position is set
bool g_isInitPositionSet = false;

// loop counter
unsigned int g_loopCounter = 0;
}

void printVector(const std::vector<double>& vec)
{
    for (auto i : vec) {
        std::cout << i << "  ";
    }
    std::cout << std::endl;
}

// callback function for realtime periodic task
void periodicTask(std::shared_ptr<flexiv::RobotStates> robotStates,
    std::shared_ptr<flexiv::Robot> robot, std::string motionType,
    flexiv::Log& log)
{
    // read robot states
    robot->getRobotStates(robotStates.get());

    // set initial joint position
    if (!g_isInitPositionSet) {
        // check vector size before saving
        if (robotStates->m_q.size() == k_robotDofs) {
            g_initPosition = robotStates->m_q;
            g_isInitPositionSet = true;
            log.info("Initial joint position set to:");
            printVector(g_initPosition);
        }
    }
    // run control only after init position is set
    else {
        // initialize target vectors to hold position
        auto targetPosition = g_initPosition;
        std::vector<double> targetVelocity(k_robotDofs, 0);
        std::vector<double> targetAcceleration(k_robotDofs, 0);

        // set target vectors based on motion type
        if (motionType == "hold") {
            // do nothing, target vectors are initialized to hold position
        } else if (motionType == "sine-sweep") {
            for (size_t i = 0; i < k_robotDofs; ++i) {
                targetPosition[i] = g_initPosition[i]
                                    + k_sineAmp[i]
                                          * sin(2 * M_PI * k_sineFreq[i]
                                                * g_loopCounter * k_loopPeiord);
                std::fill(targetVelocity.begin(), targetVelocity.end(), 0.5);
                std::fill(
                    targetAcceleration.begin(), targetAcceleration.end(), 0);
            }
        } else {
            log.error("Unknown motion type");
            log.info("Accepted motion types: hold, sine-sweep");
            exit(1);
        }

        // send target joint position to RDK server
        robot->streamJointPosition(
            targetPosition, targetVelocity, targetAcceleration);
    }

    g_loopCounter++;
}

int main(int argc, char* argv[])
{
    // log object for printing message with timestamp and coloring
    flexiv::Log log;

    // Parse Parameters
    //=============================================================================
    // check if program has 3 arguments
    if (argc != 4) {
        log.error(
            "Invalid program arguments. Usage: <robot_ip> <local_ip> "
            "<motion_type>");
        log.info("Accepted motion types: hold, sine-sweep");
        return 0;
    }
    // IP of the robot server
    std::string robotIP = argv[1];

    // IP of the workstation PC running this program
    std::string localIP = argv[2];

    // type of motion specified by user
    std::string motionType = argv[3];

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
    robot->setMode(flexiv::MODE_JOINT_POSITION);

    // wait for the mode to be switched
    do {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } while (robot->getMode() != flexiv::MODE_JOINT_POSITION);

    // High-priority Realtime Periodic Task @ 1kHz
    //=============================================================================
    // this is a blocking method, so all other user-defined background threads
    // should be spawned before this
    robot->start(std::bind(periodicTask, robotStates, robot, motionType, log));

    return 0;
}
