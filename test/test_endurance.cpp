/**
 * @test test_endurance.cpp
 * Endurance test running Cartesian impedance control to slowly sine-sweep near
 * home for a duration of user-specified hours. Raw data will be logged to CSV
 * files continuously.
 * @copyright Copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <Robot.hpp>
#include <Log.hpp>

#include <fstream>
#include <string>
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
const double k_swingFreq = 0.025; // = 10mm/s linear velocity

// initial Cartesian-space pose (position + rotation) of robot TCP
std::vector<double> g_initTcpPose;

// current Cartesian-space pose (position + rotation) of robot TCP
std::vector<double> g_currentTcpPose;

// flag whether initial Cartesian position is set
bool g_isInitPoseSet = false;

// high-priority task loop counter
uint64_t g_hpLoopCounter = 0;

// test duration converted from user-specified hours to loop counts
uint64_t g_testDurationLoopCounts = 0;

// time duration for each log file [loop counts]
const unsigned int k_logDurationLoopCounts = 10 * 60 * 1000; // = 10 min/file

// data to be logged in low-priority thread
struct LogData
{
    std::vector<double> tcpPose;
    std::vector<double> tcpForce;
} g_logData;

}

// callback function for realtime periodic task
void highPriorityTask(std::shared_ptr<flexiv::RobotStates> robotStates,
    std::shared_ptr<flexiv::Robot> robot)
{
    // read robot states
    robot->getRobotStates(robotStates.get());

    // TCP movement control
    //=====================================================================
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
        }
    }
    // run control after initial pose is set
    else {
        // move along Z direction
        g_currentTcpPose[2] = g_initTcpPose[2]
                              + k_swingAmp
                                    * sin(2 * M_PI * k_swingFreq
                                          * g_hpLoopCounter * k_loopPeiord);
        robot->streamTcpPose(g_currentTcpPose, tcpVel, tcpAcc);
    }

    // save data to global buffer, not using mutex to avoid interruption on RT
    // loop from potential priority inversion
    g_logData.tcpPose = robotStates->m_tcpPose;
    g_logData.tcpForce = robotStates->m_extForceInBaseFrame;

    // increment loop counter
    g_hpLoopCounter++;
}

void lowPriorityTask()
{
    // low-priority task loop counter
    uint64_t lpLoopCounter = 0;

    // data logging CSV file
    std::ofstream csvFile;

    // CSV file name
    std::string csvFileName;

    // CSV file counter (data during the test is divided to multiple files)
    unsigned int fileCounter = 0;

    // log object for printing message with timestamp and coloring
    flexiv::Log log;

    // wait for a while for the robot states data to be available
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // use while loop to prevent this thread from return
    while (true) {
        // run at 1kHz
        std::this_thread::sleep_for(std::chrono::milliseconds(1));

        // Data logging
        //=====================================================================
        // close existing log file and create a new one periodically
        if (lpLoopCounter % k_logDurationLoopCounts == 0) {
            // close file if exist
            if (csvFile.is_open()) {
                csvFile.close();
                log.info("Saved log file: " + csvFileName);
            }

            // increment log file counter
            fileCounter++;

            // create new file name using the updated counter as suffix
            csvFileName
                = "endurance_test_data_" + std::to_string(fileCounter) + ".csv";

            // open new log file
            csvFile.open(csvFileName);
            if (csvFile.is_open()) {
                log.info("Created new log file: " + csvFileName);
            } else {
                log.error("Failed to create log file: " + csvFileName);
            }
        }

        // log data to file in CSV format, avoid logging too much data otherwise
        // the RT loop will not hold
        if (csvFile.is_open()) {
            // loop counter x1, TCP pose x7, TCP external force x6
            csvFile << lpLoopCounter << ",";

            for (const auto& i : g_logData.tcpPose) {
                csvFile << i << ",";
            }

            for (const auto& i : g_logData.tcpForce) {
                csvFile << i << ",";
            }

            // end of line
            csvFile << '\n';
        }

        // check if the test duration has elapsed
        if (g_hpLoopCounter > g_testDurationLoopCounts) {
            log.info("Test duration has elapsed, saving any open log file ...");

            // close log file
            if (csvFile.is_open()) {
                csvFile.close();
                log.info("Saved log file: " + csvFileName);
            }

            // exit program
            exit(1);
        }

        // increment loop counter
        lpLoopCounter++;
    }
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
            "<test_hours>");
        log.info("Note: <test_hours> can be a float number");
        return 0;
    }
    // IP of the robot server
    std::string robotIP = argv[1];

    // IP of the workstation PC running this program
    std::string localIP = argv[2];

    // test duration in hours
    double testHours = std::stof(argv[3]);
    // convert duration in hours to loop counts
    g_testDurationLoopCounts = (uint64_t)(testHours * 3600.0 * 1000.0);
    log.info("Test duration: " + std::to_string(testHours) + " hours = "
             + std::to_string(g_testDurationLoopCounts) + " cycles");

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

    // Bring Robot To Home
    //=============================================================================
    // set mode after robot is operational
    robot->setMode(flexiv::MODE_PLAN_EXECUTION);

    // wait for mode to be set
    do {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } while (robot->getMode() != flexiv::MODE_PLAN_EXECUTION);

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

    // set mode after robot is at home
    robot->setMode(flexiv::MODE_CARTESIAN_IMPEDANCE);

    // wait for the mode to be switched
    do {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } while (robot->getMode() != flexiv::MODE_CARTESIAN_IMPEDANCE);

    // Low-priority Background Tasks
    //=============================================================================
    // use std::thread for some non-realtime tasks, not joining (non-blocking)
    std::thread lowPriorityThread(lowPriorityTask);

    // High-priority Realtime Periodic Task @ 1kHz
    //=============================================================================
    // this is a blocking method, so all other user-defined background threads
    // should be spawned before this
    robot->start(std::bind(highPriorityTask, robotStates, robot));

    return 0;
}
