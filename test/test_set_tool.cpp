/**
 * @test test_set_tool.cpp
 * A test to evaluate the setTool API.
 * @copyright Copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <Robot.hpp>
#include <Model.hpp>
#include <Log.hpp>

#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>
#include <mutex>

namespace {

// loop counter
unsigned int g_loopCounter = 0;

// timer to measure scheduler performance
std::chrono::high_resolution_clock::time_point g_tic, g_toc;

// M, G true value (in MATLAB) after setTool at Home position
Eigen::MatrixXd g_M(7, 7);
Eigen::VectorXd g_G(7, 1);

// M, G after setTool in user RT task with computation time [us]
struct PrintData
{
    int64_t loopTime;
    Eigen::VectorXd G;
    Eigen::MatrixXd M;
} g_printData;

// mutex on data to be printed in low-priority thread
std::mutex g_printDataMutex;

}

// user-defined high-priority realtime periodic task, running at 1kHz
void highPriorityPeriodicTask(std::shared_ptr<flexiv::RobotStates> robotStates,
    std::shared_ptr<flexiv::Robot> robot, std::shared_ptr<flexiv::Model> model)
{
    // get new robot states
    robot->getRobotStates(robotStates.get());

    // update robot model in dynamics engine
    model->updateModel(robotStates->m_q, robotStates->m_dtheta);

    // mark timer start point
    g_tic = std::chrono::high_resolution_clock::now();

    // get new M and G after setTool from dynamic engine
    auto M = model->getMassMatrix();
    auto G = model->getGravityForce();

    // mark timer end point and get loop time
    g_toc = std::chrono::high_resolution_clock::now();
    auto loopTime
        = std::chrono::duration_cast<std::chrono::microseconds>(g_toc - g_tic)
              .count();

    // save to global struct for printing in another thread
    {
        std::lock_guard<std::mutex> lock(g_printDataMutex);
        g_printData.loopTime = loopTime;
        g_printData.M = M;
        g_printData.G = G;
    }
}

// user-defined low-priority non-realtime task
// NOT strictly scheduled at a fixed rate
void lowPriorityTask()
{
    // wait for a while for the time interval data to be available
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // use while loop to prevent this thread from return
    while (true) {
        // wake up every second to do something
        std::this_thread::sleep_for(std::chrono::seconds(1));

        // safely read the global variable
        int loopTime;
        Eigen::MatrixXd M;
        Eigen::VectorXd G;
        {
            std::lock_guard<std::mutex> lock(g_printDataMutex);
            loopTime = g_printData.loopTime;
            M = g_printData.M;
            G = g_printData.G;
        }

        // print time interval of high-priority periodic task
        std::cout << "====================================================="
                  << std::endl;
        std::cout << "Loop time = " << loopTime << " us" << std::endl;

        // evaluate M, G after setTool and compute their norm
        auto deltaM = M - g_M;
        auto deltaG = G - g_G;

        std::cout << std::fixed << std::setprecision(5);
        std::cout << "Difference of M between ground truth (MATLAB) and "
                     "integrated dynamics engine after setTool() = "
                  << std::endl
                  << deltaM << std::endl;
        std::cout << "Norm of delta M: " << deltaM.norm() << '\n' << std::endl;

        std::cout << "Difference of G between ground truth (MATLAB) and "
                     "integrated dynamics engine after setTool() = "
                  << std::endl
                  << deltaG.transpose() << std::endl;
        std::cout << "Norm of delta G: " << deltaG.norm() << '\n' << std::endl;
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
    robot->setMode(flexiv::MODE_PLAN_EXECUTION);

    // wait for mode to be set
    do {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } while (robot->getMode() != flexiv::MODE_PLAN_EXECUTION);

    // Bring Robot To Home
    //=============================================================================
    robot->executePlanByName("PLAN-Home");
    // wait for the plan to start
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // wait for the execution to finish
    flexiv::SystemStatus systemStatus;
    do {
        robot->getSystemStatus(&systemStatus);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } while (systemStatus.m_programRunning == true);

    // put mode back to IDLE
    robot->setMode(flexiv::MODE_IDLE);
    do {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } while (robot->getMode() != flexiv::MODE_IDLE);

    // Robot Model (Dynamics Engine) Initialization
    //=============================================================================
    auto model = std::make_shared<flexiv::Model>();
    // load model from robot client
    robot->loadModel(model.get());

    // Set Tool
    //=============================================================================
    // artificial tool parameters for verification
    double mass = 0.9;
    // com is relative to tcp frame
    Eigen::Vector3d com = {0.0, 0.0, -0.093};
    Eigen::Vector3d momentum = mass * com;
    // inertia relative to com
    Eigen::Matrix3d inertia;
    inertia << 2.768e-03, 0, 0, 0, 3.149e-03, 0, 0, 0, 5.64e-04;
    // tcp frame relative to flange
    Eigen::Vector3d tcpPosition = {0.0, 0.0, 0.15};

    if (model->setTool(mass, inertia, com, tcpPosition) == false) {
        log.error("setTool() failed");
        return 0;
    }
    log.info("setTool() successful");

    // Hard-coded Dynamics Ground Truth from MATLAB
    //=============================================================================
    // clang-format off
    g_M << 
     2.916316686749461, -0.052869517013466,  1.903540434220357, -0.124348845003517, -0.041914639740668,  0.027649255000000, -0.001464000000000,
    -0.052869517013466,  3.222619358431081,  0.053667041477633, -1.414236317529289, -0.184390078851855,  0.259867572215541, -0.000000000000000,
     1.903540434220357,  0.053667041477633,  1.416023230140298, -0.002724223477633,  0.000093550780077,  0.001155982477633, -0.001121489064726,
    -0.124348845003517, -1.414236317529289, -0.002724223477633,  1.287676548627496,  0.177147398851855, -0.244344118313748,  0.000000000000000,
    -0.041914639740668, -0.184390078851855,  0.000093550780077,  0.177147398851855,  0.054219756143365, -0.038829881851855,  0.000941041060581,
     0.027649255000000,  0.259867572215541,  0.001155982477633, -0.244344118313748, -0.038829881851855,  0.082171270000000,                  0,
    -0.001464000000000, -0.000000000000000, -0.001121489064726,  0.000000000000000,  0.000941041060581,                  0,  0.001464000000000;

    // Matlab value is the joint torque to resist gravity
    g_G << 
    -0.000000000000001,  52.664497076609663,  0.830964961569619, -22.968509865473024, -2.721399343355234, 3.272076450000000,                 0;
    // clang-format on

    // Low-priority Background Tasks
    //=============================================================================
    // use std::thread for some non-realtime tasks, not joining (non-blocking)
    std::thread lowPriorityThread(lowPriorityTask);

    // High-priority Realtime Periodic Task @ 1kHz
    //=============================================================================
    // this is a blocking method, so all other user-defined background threads
    // should be spawned before this
    robot->start(
        std::bind(highPriorityPeriodicTask, robotStates, robot, model));

    return 0;
}
