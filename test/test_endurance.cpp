/**
 * @test test_endurance.cpp
 * Endurance test running Cartesian impedance control to slowly sine-sweep near home for a duration
 * of user-specified hours. Raw data will be logged to CSV files continuously.
 * @copyright Copyright (C) 2016-2024 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <flexiv/rdk/robot.hpp>
#include <flexiv/rdk/scheduler.hpp>
#include <flexiv/rdk/utility.hpp>
#include <spdlog/spdlog.h>

#include <iostream>
#include <fstream>
#include <string>
#include <cmath>
#include <thread>
#include <atomic>

namespace {
/** RT loop period [sec] */
const double kLoopPeriod = 0.001;

/** TCP sine-sweep amplitude [m] */
const double kSwingAmp = 0.1;

/** TCP sine-sweep frequency [Hz] */
const double kSwingFreq = 0.025; // = 10mm/s linear velocity

/** Current Cartesian-space pose (position + rotation) of robot TCP */
std::array<double, flexiv::rdk::kPoseSize> g_curr_tcp_pose;

/** Test duration converted from user-specified hours to loop counts */
uint64_t g_test_duration_loop_counts = 0;

/** Time duration for each log file [loop counts] */
const unsigned int kLogDurationLoopCounts = 10 * 60 * 1000; // = 10 min/file

/** Data to be logged in low-priority thread */
struct LogData
{
    std::array<double, flexiv::rdk::kPoseSize> tcp_pose;
    std::array<double, flexiv::rdk::kCartDOF> tcp_force;
} g_log_data;

/** Atomic signal to stop the test */
std::atomic<bool> g_stop = {false};
}

void highPriorityTask(
    flexiv::rdk::Robot& robot, const std::array<double, flexiv::rdk::kPoseSize>& init_pose)
{
    // Local periodic loop counter
    static uint64_t loop_counter = 0;

    try {
        // Monitor fault on the connected robot
        if (robot.fault()) {
            throw std::runtime_error(
                "highPriorityTask: Fault occurred on the connected robot, exiting ...");
        }

        // Swing along Z direction
        g_curr_tcp_pose[2]
            = init_pose[2] + kSwingAmp * sin(2 * M_PI * kSwingFreq * loop_counter * kLoopPeriod);
        robot.StreamCartesianMotionForce(g_curr_tcp_pose);

        // Save data to global buffer, not using mutex to avoid interruption on RT loop from
        // potential priority inversion
        g_log_data.tcp_pose = robot.states().tcp_pose;
        g_log_data.tcp_force = robot.states().ext_wrench_in_world;

        // Stop after test duration has elapsed
        if (++loop_counter > g_test_duration_loop_counts) {
            g_stop = true;
        }

    } catch (const std::exception& e) {
        spdlog::error(e.what());
        g_stop = true;
    }
}

void lowPriorityTask()
{
    // Low-priority task loop counter
    uint64_t loop_counter = 0;

    // Data logging CSV file
    std::ofstream csv_file;

    // CSV file name
    std::string csv_file_name;

    // CSV file counter (data during the test is divided to multiple files)
    unsigned int file_counter = 0;

    // Wait for a while for the robot states data to be available
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // Use while loop to prevent this thread from return
    while (true) {
        // Log data at 1kHz
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        loop_counter++;

        // Close existing log file and create a new one periodically
        if (loop_counter % kLogDurationLoopCounts == 0) {
            if (csv_file.is_open()) {
                csv_file.close();
                spdlog::info("Saved log file: {}", csv_file_name);
            }

            // Increment log file counter
            file_counter++;

            // Create new file name using the updated counter as suffix
            csv_file_name = "endurance_test_data_" + std::to_string(file_counter) + ".csv";

            // Open new log file
            csv_file.open(csv_file_name);
            if (csv_file.is_open()) {
                spdlog::info("Created new log file: {}", csv_file_name);
            } else {
                spdlog::error("Failed to create log file: {}", csv_file_name);
            }
        }

        // Log data to file in CSV format
        if (csv_file.is_open()) {
            // Loop counter x1, TCP pose x7, TCP external force x6
            csv_file << loop_counter << ",";
            for (const auto& i : g_log_data.tcp_pose) {
                csv_file << i << ",";
            }
            for (const auto& i : g_log_data.tcp_force) {
                csv_file << i << ",";
            }
            // End of line
            csv_file << '\n';
        }

        // Check if the test duration has elapsed
        if (g_stop) {
            spdlog::info("Test duration has elapsed, saving any open log file ...");
            // Close log file
            if (csv_file.is_open()) {
                csv_file.close();
                spdlog::info("Saved log file: {}", csv_file_name);
            }
            // Exit thread
            return;
        }
    }
}

void PrintHelp()
{
    // clang-format off
    std::cout << "Required arguments: [robot SN] [test hours]" << std::endl;
    std::cout << "    robot SN: Serial number of the robot to connect to. "
                 "Remove any space, for example: Rizon4s-123456" << std::endl;
    std::cout << "    test hours: duration of the test, can have decimals" << std::endl;
    std::cout << "Optional arguments: None" << std::endl;
    std::cout << std::endl;
    // clang-format on
}

int main(int argc, char* argv[])
{
    // Parse Parameters
    //==============================================================================================
    if (argc < 3 || flexiv::rdk::utility::ProgramArgsExistAny(argc, argv, {"-h", "--help"})) {
        PrintHelp();
        return 1;
    }

    // Serial number of the robot to connect to. Remove any space, for example: Rizon4s-123456
    std::string robot_sn = argv[1];

    // test duration in hours
    double test_hours = std::stof(argv[2]);
    // convert duration in hours to loop counts
    g_test_duration_loop_counts = (uint64_t)(test_hours * 3600.0 * 1000.0);
    spdlog::info("Test duration: {} hours = {} cycles", test_hours, g_test_duration_loop_counts);

    try {
        // RDK Initialization
        //==========================================================================================
        // Instantiate robot interface
        flexiv::rdk::Robot robot(robot_sn);

        // Clear fault on the connected robot if any
        if (robot.fault()) {
            spdlog::warn("Fault occurred on the connected robot, trying to clear ...");
            // Try to clear the fault
            if (!robot.ClearFault()) {
                spdlog::error("Fault cannot be cleared, exiting ...");
                return 1;
            }
            spdlog::info("Fault on the connected robot is cleared");
        }

        // enable the robot, make sure the E-stop is released before enabling
        spdlog::info("Enabling robot ...");
        robot.Enable();

        // Wait for the robot to become operational
        while (!robot.operational()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        spdlog::info("Robot is now operational");

        // Bring Robot To Home
        //==========================================================================================
        robot.SwitchMode(flexiv::rdk::Mode::NRT_PLAN_EXECUTION);
        robot.ExecutePlan("PLAN-Home");

        // Wait fot the plan to finish
        do {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        } while (robot.busy());

        // Switch mode after robot is at home
        robot.SwitchMode(flexiv::rdk::Mode::RT_CARTESIAN_MOTION_FORCE);

        // Set initial pose to current TCP pose
        auto init_pose = robot.states().tcp_pose;
        spdlog::info("Initial TCP pose set to [position 3x1, rotation (quaternion) 4x1]: "
                     + flexiv::rdk::utility::Arr2Str(init_pose));
        g_curr_tcp_pose = init_pose;

        // Periodic Tasks
        //==========================================================================================
        flexiv::rdk::Scheduler scheduler;
        // Add periodic task with 1ms interval and highest applicable priority
        scheduler.AddTask(std::bind(highPriorityTask, std::ref(robot), std::ref(init_pose)),
            "HP periodic", 1, scheduler.max_priority());
        // Start all added tasks
        scheduler.Start();

        // Use std::thread for logging task without strict chronology
        std::thread low_priority_thread(lowPriorityTask);

        // low_priority_thread is responsible to release blocking
        low_priority_thread.join();

    } catch (const std::exception& e) {
        spdlog::error(e.what());
        return 1;
    }

    return 0;
}
