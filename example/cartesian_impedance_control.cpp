/**
 * @example Run Cartesian impedance control to hold or sine-sweep the robot TCP
 * @copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <config.h>
#include <Robot.hpp>

#include <iostream>
#include <cmath>
#include <thread>

namespace {

// size of Cartesian pose vector [position 3x1 + rotation (quaternion) 4x1 ]
const unsigned int k_cartPoseSize = 7;

// RT loop period [sec]
const double k_loopPeiord = 0.001;

// TCP sine-sweep amplitude [m]
const double k_swingAmp = 0.1;

// TCP sine-sweep frequency [Hz]
const double k_swingFreq = 0.3;

// initial Cartesian-space pose (position + rotation) of robot TCP
std::vector<double> g_initTcpPose;

// current Cartesian-space pose (position + rotation) of robot TCP
std::vector<double> g_currentTcpPose;

// flag whether initial Cartesian position is set
bool g_isInitPoseSet = false;

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
    std::shared_ptr<flexiv::Robot> robot, std::string motionType)
{
    // read robot states
    robot->getRobotStates(robotStates.get());

    // use target TCP velocity and acceleration = 0
    std::vector<double> tcpVel(6, 0);
    std::vector<double> tcpAcc(6, 0);

    // set initial TCP pose
    if (!g_isInitPoseSet) {
        // check vector size before saving
        if (robotStates->m_tcpPose.size() == k_cartPoseSize) {
            g_initTcpPose = robotStates->m_tcpPose;
            g_currentTcpPose = g_initTcpPose;
            g_isInitPoseSet = true;
            std::cout << "Initial Cartesion pose of robot TCP set to [position "
                         "3x1 + rotation (quaternion) 4x1]:\n";
            printVector(g_initTcpPose);
        }
    }
    // run control only after initial pose is set
    else {
        // set target position based on motion type
        if (motionType == "hold") {
            robot->streamTcpPose(g_initTcpPose, tcpVel, tcpAcc);
        } else if (motionType == "sine-sweep") {
            g_currentTcpPose[1] = g_initTcpPose[1]
                                  + k_swingAmp
                                        * sin(2 * M_PI * k_swingFreq
                                              * g_loopCounter * k_loopPeiord);
            robot->streamTcpPose(g_currentTcpPose, tcpVel, tcpAcc);
        } else {
            std::cout << "Unknown motion type" << std::endl;
            std::cout << "Accepted motion types: hold, sine-sweep" << std::endl;
            exit(1);
        }
    }
    g_loopCounter++;
}

int main(int argc, char* argv[])
{
    // Parse Parameters
    //=============================================================================
    if (argc != 2) {
        std::cerr << "Invalid program arguments. Usage: <motion_type>"
                  << std::endl;
        std::cout << "Accepted motion types: hold, sine-sweep" << std::endl;
        return 0;
    }

    // type of motion specified by user
    std::string motionType = argv[1];

    // print loop frequency
    std::cout << "Example client running at 1000 Hz" << std::endl;

    // RDK Initialization
    //=============================================================================
    // RDK robot interface
    auto robot = std::make_shared<flexiv::Robot>();

    // robot states data from RDK server
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
    robot->setMode(flexiv::MODE_CARTESIAN_IMPEDANCE);

    // wait for the mode to be switched
    do {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } while (robot->getMode() != flexiv::MODE_CARTESIAN_IMPEDANCE);

    // choose the index of tool being used, default is the 1st one
    robot->switchTcp(0);

    // High-priority Realtime Periodic Task @ 1kHz
    //=============================================================================
    // this is a blocking method, so all other user-defined background threads
    // should be spawned before this
    robot->start(std::bind(periodicTask, robotStates, robot, motionType));

    return 0;
}
