/**
 * @example Print received robot states
 * @copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <config.h>
#include <Robot.hpp>

#include <iostream>
#include <iomanip>
#include <thread>
#include <mutex>

namespace {

// print counter
unsigned int g_printCounter = 0;

// data to be printed in low-priority thread
struct PrintData
{
    std::vector<double> q;
    std::vector<double> dq;
    std::vector<double> tau_J;
    std::vector<double> tau_J_ext;
    std::vector<double> tcp_pose;
    std::vector<double> tcp_vel;
    std::vector<double> tcp_acc;
    std::vector<double> tcp_wrench;
} g_printData;

// mutex on data to be printed in low-priority thread
std::mutex g_printDataMutex;

}

void printVector(std::string title, const std::vector<double>& vec)
{
    std::cout << "\n" << title << ": \n";
    for (auto i : vec) {
        std::cout << std::fixed << std::setprecision(5) << i << "  ";
    }
    std::cout << std::endl;
}

// user-defined high-priority realtime periodic task, running at 1kHz
void highPriorityPeriodicTask(std::shared_ptr<flexiv::RobotStates> robotStates,
    std::shared_ptr<flexiv::Robot> robot)
{
    // get robot states
    robot->getRobotStates(robotStates.get());

    // save data to global variable for printing in another thread
    {
        std::lock_guard<std::mutex> lock(g_printDataMutex);
        g_printData.q = robotStates->m_linkPosition;
        g_printData.dq = robotStates->m_linkVelocity;
        g_printData.tau_J = robotStates->m_linkTorque;
        g_printData.tau_J_ext = robotStates->m_linkTorqueExt;
        g_printData.tcp_pose = robotStates->m_tcpPose;
        g_printData.tcp_vel = robotStates->m_tcpVel;
        g_printData.tcp_acc = robotStates->m_tcpAcc;
        g_printData.tcp_wrench = robotStates->m_tcpWrench;
    }
}

// user-defined low-priority non-realtime task
// NOT strictly scheduled at a fixed rate
void lowPriorityTask()
{
    // wait a while for the data to start streaming
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // use while loop to prevent this thread from return
    while (true) {
        // wake up every second to do something
        std::this_thread::sleep_for(std::chrono::seconds(1));

        // safely save the global data to local
        PrintData printData;
        {
            std::lock_guard<std::mutex> lock(g_printDataMutex);
            printData = g_printData;
        }

        // devider
        std::cout << "\n\n[" << ++g_printCounter << "]";
        std::cout << "========================================================="
                  << std::endl;

        // print everything
        printVector("q [A1 ~ A7]", printData.q);
        printVector("dq [A1 ~ A7]", printData.dq);
        printVector("tau_J [A1 ~ A7]", printData.tau_J);
        printVector("tau_J_ext [A1 ~ A7]", printData.tau_J_ext);
        printVector("tcp_pose [X, Y, Z, w, x, y, z]", printData.tcp_pose);
        printVector("tcp_vel [vx, vy, vz, wx, wy, wz]", printData.tcp_vel);
        printVector("tcp_acc [ax, ay, az, αx, αy, αz]", printData.tcp_acc);
        printVector(
            "tcp_wrench [Fx, Fy, Fz, Mx, My, Mz]", printData.tcp_wrench);
    }
}

int main(int argc, char* argv[])
{
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
    robot->setMode(flexiv::MODE_IDLE);

    // Low-priority Background Tasks
    //=============================================================================
    // use std::thread for some non-realtime tasks, not joining (non-blocking)
    std::thread lowPriorityThread(lowPriorityTask);

    // High-priority Realtime Periodic Task @ 1kHz
    //=============================================================================
    // this is a blocking method, so all other user-defined background threads
    // should be spawned before this
    robot->start(std::bind(highPriorityPeriodicTask, robotStates, robot));

    return 0;
}
