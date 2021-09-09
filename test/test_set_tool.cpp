/**
 * @test Test to evaluate the setTool API
 * @copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <config.h>
#include <Robot.hpp>
#include <Model.hpp>

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
    model->updateModel(
        robotStates->m_linkPosition, robotStates->m_linkVelocity);

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
        std::cout << "DIFFERENCE of M between ground truth (MATLAB) and rdk after "
                     "setTool = "
                  << std::endl
                  << deltaM << std::endl;
        std::cout << "norm of delta M: " << deltaM.norm() << std::endl;

        std::cout << "DIFFERENCE of G between ground truth (MATLAB) and rdk after "
                     "setTool = "
                  << std::endl
                  << deltaG.transpose() << std::endl;
        std::cout << "norm of delta G: " << deltaG.norm() << std::endl;
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

    // Set Tool
    //=============================================================================
    // Robotiq tool info
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
        std::cout << "set tool failed! " << std::endl;
        return 0;
    }
    std::cout << "set tool successfully" << std::endl;

    // M, G Hard Code From MATLAB Robotics System Toolbox After Set Tool
    //=============================================================================
    g_M << 3.00477, -0.05394, 1.96283, -0.12863, -0.04579, 0.02819, -0.00164,
        -0.05394, 3.31633, 0.05426, -1.45674, -0.19104, 0.26740, -0.00000,
        1.96283, 0.05426, 1.45873, -0.00262, -0.00284, 0.00116, -0.00127,
        -0.12863, -1.45674, -0.00262, 1.32548, 0.18352, -0.24987, -0.00007,
        -0.04579, -0.19104, -0.00284, 0.18352, 0.05500, -0.03987, 0.00106,
        0.02819, 0.26740, 0.00116, -0.24987, -0.03987, 0.08249, 0.00001,
        -0.00164, -0.00000, -0.00127, -0.00007, 0.00106, 0.00001, 0.00168;

    // Matlab value is the joint torque to resist gravity
    g_G << 0.00000, 54.18118, 0.84197, -23.68555, -2.82637, 3.39972, 0.00000;

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
