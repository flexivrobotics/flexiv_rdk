/**
 * @test test_endurance.cpp
 * Endurance test running Cartesian impedance control to slowly sine-sweep near
 * home for a duration of user-specified hours. Raw data will be logged to CSV
 * files continuously.
 * @copyright Copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <flexiv/Robot.hpp>
#include <flexiv/Exception.hpp>
#include <flexiv/Log.hpp>
#include <flexiv/Scheduler.hpp>
#include <flexiv/Utility.hpp>

#include <iostream>
#include <fstream>
#include <string>
#include <cmath>
#include <thread>
#include <atomic>

namespace {
/** RT loop period [sec] */
const double k_loopPeriod = 0.001;

/** TCP sine-sweep amplitude [m] */
const double k_swingAmp = 0.1;

/** TCP sine-sweep frequency [Hz] */
const double k_swingFreq = 0.025; // = 10mm/s linear velocity

/** Current Cartesian-space pose (position + rotation) of robot TCP */
std::vector<double> g_currentTcpPose;

/** Test duration converted from user-specified hours to loop counts */
uint64_t g_testDurationLoopCounts = 0;

/** Time duration for each log file [loop counts] */
const unsigned int k_logDurationLoopCounts = 10 * 60 * 1000; // = 10 min/file

/** Data to be logged in low-priority thread */
struct LogData
{
    std::vector<double> tcpPose;
    std::vector<double> tcpForce;
} g_logData;

/** Atomic signal to stop the test */
std::atomic<bool> g_stop = {false};
}

void highPriorityTask(flexiv::Robot& robot, flexiv::Log& log, flexiv::RobotStates& robotStates,
    const std::vector<double>& initPose)
{
    // Local periodic loop counter
    static uint64_t loopCounter = 0;

    try {
        // Monitor fault on robot server
        if (robot.isFault()) {
            throw flexiv::ServerException(
                "highPriorityTask: Fault occurred on robot server, exiting ...");
        }

        // Read robot states
        robot.getRobotStates(robotStates);

        // Swing along Z direction
        g_currentTcpPose[2]
            = initPose[2] + k_swingAmp * sin(2 * M_PI * k_swingFreq * loopCounter * k_loopPeriod);
        robot.streamCartesianMotionForce(g_currentTcpPose);

        // Save data to global buffer, not using mutex to avoid interruption on
        // RT loop from potential priority inversion
        g_logData.tcpPose = robotStates.tcpPose;
        g_logData.tcpForce = robotStates.extWrenchInBase;

        // Stop after test duration has elapsed
        if (++loopCounter > g_testDurationLoopCounts) {
            g_stop = true;
        }

    } catch (const flexiv::Exception& e) {
        log.error(e.what());
        g_stop = true;
    }
}

void lowPriorityTask()
{
    // Low-priority task loop counter
    uint64_t loopCounter = 0;

    // Data logging CSV file
    std::ofstream csvFile;

    // CSV file name
    std::string csvFileName;

    // CSV file counter (data during the test is divided to multiple files)
    unsigned int fileCounter = 0;

    // Log object for printing message with timestamp and coloring
    flexiv::Log log;

    // Wait for a while for the robot states data to be available
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // Use while loop to prevent this thread from return
    while (true) {
        // Log data at 1kHz
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        loopCounter++;

        // Close existing log file and create a new one periodically
        if (loopCounter % k_logDurationLoopCounts == 0) {
            if (csvFile.is_open()) {
                csvFile.close();
                log.info("Saved log file: " + csvFileName);
            }

            // Increment log file counter
            fileCounter++;

            // Create new file name using the updated counter as suffix
            csvFileName = "endurance_test_data_" + std::to_string(fileCounter) + ".csv";

            // Open new log file
            csvFile.open(csvFileName);
            if (csvFile.is_open()) {
                log.info("Created new log file: " + csvFileName);
            } else {
                log.error("Failed to create log file: " + csvFileName);
            }
        }

        // Log data to file in CSV format
        if (csvFile.is_open()) {
            // Loop counter x1, TCP pose x7, TCP external force x6
            csvFile << loopCounter << ",";
            for (const auto& i : g_logData.tcpPose) {
                csvFile << i << ",";
            }
            for (const auto& i : g_logData.tcpForce) {
                csvFile << i << ",";
            }
            // End of line
            csvFile << '\n';
        }

        // Check if the test duration has elapsed
        if (g_stop) {
            log.info("Test duration has elapsed, saving any open log file ...");
            // Close log file
            if (csvFile.is_open()) {
                csvFile.close();
                log.info("Saved log file: " + csvFileName);
            }
            // Exit thread
            return;
        }
    }
}

void printHelp()
{
    // clang-format off
    std::cout << "Required arguments: [robot IP] [local IP] [test hours]" << std::endl;
    std::cout << "    robot IP: address of the robot server" << std::endl;
    std::cout << "    local IP: address of this PC" << std::endl;
    std::cout << "    test hours: duration of the test, can have decimals" << std::endl;
    std::cout << "Optional arguments: None" << std::endl;
    std::cout << std::endl;
    // clang-format on
}

int main(int argc, char* argv[])
{
    // log object for printing message with timestamp and coloring
    flexiv::Log log;

    // Parse Parameters
    //=============================================================================
    if (argc < 4 || flexiv::utility::programArgsExistAny(argc, argv, {"-h", "--help"})) {
        printHelp();
        return 1;
    }

    // IP of the robot server
    std::string robotIP = argv[1];

    // IP of the workstation PC running this program
    std::string localIP = argv[2];

    // test duration in hours
    double testHours = std::stof(argv[3]);
    // convert duration in hours to loop counts
    g_testDurationLoopCounts = (uint64_t)(testHours * 3600.0 * 1000.0);
    log.info("Test duration: " + std::to_string(testHours)
             + " hours = " + std::to_string(g_testDurationLoopCounts) + " cycles");

    try {
        // RDK Initialization
        //=============================================================================
        // Instantiate robot interface
        flexiv::Robot robot(robotIP, localIP);

        // create data struct for storing robot states
        flexiv::RobotStates robotStates;

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

        // enable the robot, make sure the E-stop is released before enabling
        log.info("Enabling robot ...");
        robot.enable();

        // Wait for the robot to become operational
        while (!robot.isOperational()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        log.info("Robot is now operational");

        // Bring Robot To Home
        //=============================================================================
        robot.setMode(flexiv::Mode::NRT_PLAN_EXECUTION);
        robot.executePlan("PLAN-Home");

        // Wait fot the plan to finish
        do {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        } while (robot.isBusy());

        // Switch mode after robot is at home
        robot.setMode(flexiv::Mode::RT_CARTESIAN_MOTION_FORCE);

        // Set initial pose to current TCP pose
        robot.getRobotStates(robotStates);
        auto initPose = robotStates.tcpPose;
        log.info("Initial TCP pose set to [position 3x1, rotation (quaternion) 4x1]: "
                 + flexiv::utility::vec2Str(initPose));
        g_currentTcpPose = initPose;

        // Periodic Tasks
        //=============================================================================
        flexiv::Scheduler scheduler;
        // Add periodic task with 1ms interval and highest applicable priority
        scheduler.addTask(std::bind(highPriorityTask, std::ref(robot), std::ref(log),
                              std::ref(robotStates), std::ref(initPose)),
            "HP periodic", 1, scheduler.maxPriority());
        // Start all added tasks
        scheduler.start();

        // Use std::thread for logging task without strict chronology
        std::thread lowPriorityThread(lowPriorityTask);

        // lowPriorityThread is responsible to release blocking
        lowPriorityThread.join();

    } catch (const flexiv::Exception& e) {
        log.error(e.what());
        return 1;
    }

    return 0;
}
