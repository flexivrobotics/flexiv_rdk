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
        std::cout << "Difference of J between ground truth (MATLAB) and "
                     "integrated dynamics engine = "
                  << std::endl
                  << deltaJ << std::endl;
        std::cout << "Norm of delta J: " << deltaJ.norm() << '\n' << std::endl;

        std::cout << "Difference of M between ground truth (MATLAB) and "
                     "integrated dynamics engine = "
                  << std::endl
                  << deltaM << std::endl;
        std::cout << "Norm of delta M: " << deltaM.norm() << '\n' << std::endl;

        std::cout << "Difference of G between ground truth (MATLAB) and "
                     "integrated dynamics engine = "
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

    // Hard-coded Dynamics Ground Truth from MATLAB
    //=============================================================================
    // clang-format off
    g_J <<
    0.110000000000000,  0.078420538028673,  0.034471999940354, -0.368152340866938, -0.064278760968654,  0.136000000000000, -0.000000000000000,
    0.687004857483100, -0.000000000000000,  0.576684003660457,  0.000000000000000,  0.033475407198662, -0.000000000000000, -0.000000000000000,
    0.000000000000000,  0.687004857483100, -0.028925442435894, -0.417782862794537, -0.076604444311898,  0.110000000000000, -0.000000000000000,
    0.000000000000000, -0.000000000000000,  0.642787609686539,  0.000000000000000,  0.766044443118978, -0.000000000000000,  0.000000000000000,
                    0, -1.000000000000000, -0.000000000000000,  1.000000000000000,  0.000000000000000, -1.000000000000000,  0.000000000000000,
    1.000000000000000, -0.000000000000000,  0.766044443118978,  0.000000000000000, -0.642787609686539, -0.000000000000000, -1.000000000000000;

    g_M << 
    2.480084579964625, -0.066276150278304,  1.520475428405090, -0.082258763257690, -0.082884472612488,  0.008542255000000, -0.000900000000000,
   -0.066276150278304,  2.778187401738267,  0.067350373889147, -1.100953424157635, -0.129191018084721,  0.165182543869134, -0.000000000000000,
    1.520475428405090,  0.067350373889147,  1.074177611590570, -0.000410055889147, -0.043572229972025, -0.001968185110853, -0.000689439998807,
   -0.082258763257690, -1.100953424157635, -0.000410055889147,  0.964760218577003,  0.123748338084721, -0.125985653288502,  0.000000000000000,
   -0.082884472612488, -0.129191018084721, -0.043572229972025,  0.123748338084721,  0.038006882479315, -0.020080821084721,  0.000578508848718,
    0.008542255000000,  0.165182543869134, -0.001968185110853, -0.125985653288502, -0.020080821084721,  0.034608170000000,                  0,
   -0.000900000000000, -0.000000000000000, -0.000689439998807,  0.000000000000000,  0.000578508848718,                  0,  0.000900000000000;

    // Matlab value is the joint torque to resist gravity
    g_G << 
    0.000000000000000, 46.598931189891374, 1.086347692836129, -19.279904969860052, -2.045058704525489,  2.300886450000000,                  0;
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
