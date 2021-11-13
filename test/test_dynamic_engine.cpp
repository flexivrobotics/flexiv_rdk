/**
 * @test test_dynamic_engine.cpp
 * A test to evaluate the dynamics engine (J, M, G)
 * @copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved.
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

// J, M, G true value (in MATLAB) at Home position
Eigen::MatrixXd g_J(6, 7);
Eigen::MatrixXd g_M(7, 7);
Eigen::VectorXd g_G(7, 1);

// J, M, G get from rdk dynamic engine in user RT task with computation time
// [us]
struct PrintData
{
    int64_t loopTime;
    Eigen::MatrixXd J;
    Eigen::MatrixXd M;
    Eigen::VectorXd G;
} g_printData;

// mutex on data to be printed in low-priority thread
std::mutex g_printDataMutex;

}

// user-defined high-priority realtime periodic task, running at 1kHz
void highPriorityPeriodicTask(std::shared_ptr<flexiv::RobotStates> robotStates,
    std::shared_ptr<flexiv::Robot> robot, std::shared_ptr<flexiv::Model> model)
{
    // mark timer start point
    g_tic = std::chrono::high_resolution_clock::now();

    // get new robot states
    robot->getRobotStates(robotStates.get());

    // update robot model in dynamics engine
    model->updateModel(robotStates->m_q, robotStates->m_dtheta);

    // get J, M, G from dynamic engine
    Eigen::MatrixXd J = model->getJacobian("flange");
    Eigen::MatrixXd M = model->getMassMatrix();
    Eigen::VectorXd G = model->getGravityForce();

    // mark timer end point and get loop time
    g_toc = std::chrono::high_resolution_clock::now();
    auto loopTime
        = std::chrono::duration_cast<std::chrono::microseconds>(g_toc - g_tic)
              .count();

    // save to global struct for printing in another thread
    {
        std::lock_guard<std::mutex> lock(g_printDataMutex);
        g_printData.loopTime = loopTime;
        g_printData.J = J;
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
        Eigen::MatrixXd J;
        Eigen::MatrixXd M;
        Eigen::VectorXd G;
        {
            std::lock_guard<std::mutex> lock(g_printDataMutex);
            loopTime = g_printData.loopTime;
            J = g_printData.J;
            M = g_printData.M;
            G = g_printData.G;
        }

        // print time interval of high-priority periodic task
        std::cout << "====================================================="
                  << std::endl;
        std::cout << "Loop time = " << loopTime << " us" << std::endl;

        // evaluate J, M, G with true value
        auto deltaJ = J - g_J;
        auto deltaM = M - g_M;
        auto deltaG = G - g_G;

        std::cout << std::fixed << std::setprecision(5);
        std::cout << "DIFFERENCE of J between ground truth (MATLAB) and rdk = "
                  << std::endl
                  << deltaJ << std::endl;
        std::cout << "norm of delta J: " << deltaJ.norm() << std::endl;

        std::cout << "DIFFERENCE of M between ground truth (MATLAB) and rdk = "
                  << std::endl
                  << deltaM << std::endl;
        std::cout << "norm of delta M: " << deltaM.norm() << std::endl;

        std::cout << "DIFFERENCE of G between ground truth (MATLAB) and rdk = "
                  << std::endl
                  << deltaG.transpose() << std::endl;
        std::cout << "norm of delta G: " << deltaG.norm() << std::endl;
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

    flexiv::SystemStatus systemStatus;
    // wait until execution begin
    do {
        robot->getSystemStatus(&systemStatus);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } while (systemStatus.m_programRunning == false);

    // wait until execution finished
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

    // J, M, G Hard Code From MATLAB Robotics System Toolbox
    //=============================================================================
    g_J << 0.11000, 0.07142, 0.03447, -0.36115, -0.06428, 0.12900, 0.00000,
        0.68700, -0.00000, 0.57218, 0.00000, 0.02811, 0.00000, -0.00000,
        0.00000, 0.68700, -0.02893, -0.41778, -0.07660, 0.11000, 0.00000,
        0.00000, -0.00000, 0.64279, 0.00000, 0.76604, -0.00000, 0.00000,
        0.00000, -1.00000, -0.00000, 1.00000, -0.00000, -1.00000, -0.00000,
        1.00000, -0.00000, 0.76604, -0.00000, -0.64279, 0.00000, -1.00000;

    g_M << 2.56854, -0.06665, 1.58254, -0.08723, -0.08345, 0.00977, -0.00108,
        -0.06665, 2.87356, 0.06816, -1.14695, -0.13624, 0.17474, -0.00000,
        1.58254, 0.06816, 1.12183, -0.00052, -0.04325, -0.00175, -0.00084,
        -0.08723, -1.14695, -0.00052, 1.00788, 0.13052, -0.13536, -0.00007,
        -0.08345, -0.13624, -0.04325, 0.13052, 0.03951, -0.02153, 0.00070,
        0.00977, 0.17474, -0.00175, -0.13536, -0.02153, 0.03731, 0.00001,
        -0.00108, -0.00000, -0.00084, -0.00007, 0.00070, 0.00001, 0.00111;

    // Matlab value is the joint torque to resist gravity
    g_G << 0.00000, 48.11561, 1.09735, -19.99695, -2.15003, 2.42853, 0.00000;

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
