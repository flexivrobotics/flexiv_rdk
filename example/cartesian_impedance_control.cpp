/**
 * @example cartesian_impedance_control.cpp
 * Run Cartesian impedance control to hold or sine-sweep the robot TCP.
 * @copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <Robot.hpp>
#include <Log.hpp>

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
            flexiv::Log log;
            log.error("Unknown motion type");
            log.info("Accepted motion types: hold, sine-sweep");
            exit(1);
        }
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
    robot->setMode(flexiv::MODE_CARTESIAN_IMPEDANCE);

    // wait for the mode to be switched
    do {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } while (robot->getMode() != flexiv::MODE_CARTESIAN_IMPEDANCE);

    // choose the index of tool being used. No need to call this method if the
    // mounted tool on the robot has only one TCP, it'll be used by default
    robot->switchTcp(0);

    // High-priority Realtime Periodic Task @ 1kHz
    //=============================================================================
    // this is a blocking method, so all other user-defined background threads
    // should be spawned before this
    robot->start(std::bind(periodicTask, robotStates, robot, motionType));

    return 0;
}
